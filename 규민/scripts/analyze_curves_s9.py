#!/usr/bin/env python3
# 맵세션9 Phase A. 읽기전용, CARLA 불필요.
# A-1: road1247 통과커브(s=80~85) vs 정지커브(s=93~96) 곡률반경 + 세션7 CSV 속도 오버레이
# A-2: 확정경로 15 road 전체 급커브 전수(plain/connector 구분)
# A-3: road1247의 원본 OSM way 체인, s=93~96 대응 노드의 위경도/꺾임각/간격
#
# analyze_geometry.py(parse_roads/geom_point/geom_heading)와
# geo_calibrate.py(junction 이름=원본OSM노드ID, 유사변환 적합)를 재사용한다.
import csv
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_geometry import parse_roads, parse_junctions, geom_point, geom_heading
from geo_calibrate import (
    load_osm_nodes, junction_xodr_locations, fit_similarity,
    apply_transform, enu_to_latlon, latlon_to_enu, METERS_PER_DEG_LAT,
)

BASE = Path(__file__).resolve().parent.parent
XODR_PATH = BASE / "maps" / "cbnu_internal_only_localtm_tags73.xodr"
OSM_RAW_PATH = BASE / "data" / "raw" / "cbnu_campus.osm"
OSM_CONV_PATH = BASE / "data" / "processed" / "cbnu_campus_fixed_tags73.osm"
CSV_PATH = BASE / "docs" / "logs" / "drive_log_20260919_231354.csv"
OUT_PATH = BASE / "docs" / "map_session09_curve_analysis.md"

ROUTE_ROADS = [1247, 1914, 1356, 1917, 1355, 1446, 1172, 1428, 1240,
               1695, 1239, 1426, 1238, 1564, 1278]

STEP = 0.5  # m, s 샘플 간격


def sample_road_yaw(road, step=STEP):
    samples = []
    s = 0.0
    total_len = road["length"]
    geoms = road["geoms"]
    while s <= total_len + 1e-6:
        geom = None
        for g in geoms:
            if g["s"] - 1e-6 <= s <= g["s"] + g["length"] + 1e-6:
                geom = g
        if geom is None:
            geom = geoms[-1]
        p = (s - geom["s"]) / geom["length"] if geom["length"] > 0 else 0.0
        p = min(max(p, 0.0), 1.0)
        x, y = geom_point(geom, p)
        yaw = math.degrees(geom_heading(geom, p))
        samples.append((round(s, 4), x, y, yaw))
        s += step
    return samples


def curvature_segments(samples):
    # 연속 샘플 간 yaw 변화율(deg/m)과 곡률반경(m)
    out = []
    for i in range(1, len(samples)):
        s0, x0, y0, yaw0 = samples[i - 1]
        s1, x1, y1, yaw1 = samples[i]
        ds = s1 - s0
        dyaw = ((yaw1 - yaw0 + 180) % 360) - 180
        rate = dyaw / ds if ds > 0 else 0.0
        radius = abs(ds / math.radians(dyaw)) if abs(dyaw) > 1e-6 else float("inf")
        out.append({"s0": s0, "s1": s1, "dyaw": dyaw, "rate": rate, "radius": radius})
    return out


def worst_in_range(curv, s_lo, s_hi):
    seg = [c for c in curv if c["s0"] >= s_lo - 1e-6 and c["s1"] <= s_hi + 1e-6]
    if not seg:
        return None
    return min(seg, key=lambda c: c["radius"]), seg


def load_csv_speeds(road_id):
    rows = []
    with open(CSV_PATH, newline="") as f:
        for row in csv.DictReader(f):
            if row.get("road_id") == str(road_id):
                rows.append((float(row["s"]), float(row["speed_mps"]), float(row["t"])))
    return rows


def nearest_speed(rows, s_target):
    if not rows:
        return None
    return min(rows, key=lambda r: abs(r[0] - s_target))


# ---------- A-2: 급커브 전수 ----------
def find_sharp_turns(roads, threshold_deg_per_m):
    out = []
    for rid in ROUTE_ROADS:
        road = roads.get(str(rid))
        if road is None:
            continue
        samples = sample_road_yaw(road)
        curv = curvature_segments(samples)
        for c in curv:
            if abs(c["rate"]) >= threshold_deg_per_m and c["radius"] < 1000:
                out.append({
                    "road": rid, "junction": road["junction"],
                    "s0": c["s0"], "s1": c["s1"],
                    "dyaw": c["dyaw"], "rate": c["rate"], "radius": c["radius"],
                })
    return out


def merge_adjacent(sharp, gap_tol=STEP + 1e-3):
    # 연속 세그먼트를 하나의 급커브 구간으로 병합(같은 road, s 연속)
    sharp.sort(key=lambda c: (c["road"], c["s0"]))
    merged = []
    for c in sharp:
        if merged and merged[-1]["road"] == c["road"] and c["s0"] - merged[-1]["s1"] <= gap_tol:
            m = merged[-1]
            m["s1"] = c["s1"]
            m["dyaw_total"] += c["dyaw"]
            m["min_radius"] = min(m["min_radius"], c["radius"])
        else:
            merged.append({
                "road": c["road"], "junction": c["junction"],
                "s0": c["s0"], "s1": c["s1"],
                "dyaw_total": c["dyaw"], "min_radius": c["radius"],
            })
    return merged


# ---------- A-3: OSM 원본 대응 ----------
def load_osm_ways(path):
    ways = []
    for _, elem in ET.iterparse(path, events=("end",)):
        if elem.tag == "way":
            nds = [nd.get("ref") for nd in elem.findall("nd")]
            ways.append({"id": elem.get("id"), "nds": nds})
            elem.clear()
    return ways


def build_node_graph(ways):
    adj = {}
    for w in ways:
        nds = w["nds"]
        for a, b in zip(nds, nds[1:]):
            adj.setdefault(a, []).append((b, w["id"]))
            adj.setdefault(b, []).append((a, w["id"]))
    return adj


def bfs_path(adj, start, goal):
    from collections import deque
    if start not in adj or goal not in adj:
        return None
    q = deque([[start]])
    seen = {start}
    while q:
        path = q.popleft()
        node = path[-1]
        if node == goal:
            return path
        for nxt, _wid in adj.get(node, []):
            if nxt not in seen:
                seen.add(nxt)
                q.append(path + [nxt])
    return None


def bearing_deg(lat1, lon1, lat2, lon2):
    # 평면근사(짧은 구간), geo_calibrate와 동일 상수 사용
    ref_lat = (lat1 + lat2) / 2
    e1, n1 = latlon_to_enu(lat1, lon1, ref_lat, lon1)
    e2, n2 = latlon_to_enu(lat2, lon2, ref_lat, lon1)
    return math.degrees(math.atan2(n2 - n1, e2 - e1))


def haversine_m(lat1, lon1, lat2, lon2):
    ref_lat = (lat1 + lat2) / 2
    e, n = latlon_to_enu(lat2, lon2, lat1, lon1)
    return math.hypot(e, n)


def main():
    root = ET.parse(XODR_PATH).getroot()
    roads = parse_roads(root)
    junctions = parse_junctions(root)

    lines = []
    lines.append("# 맵세션9 Phase A 급커브 가설 검증 (읽기전용, 자동생성)")
    lines.append("")

    # --- A-1 ---
    lines.append("## A-1. 통과 커브(s=80~85) vs 정지 커브(s=93~96)")
    road1247 = roads["1247"]
    samples = sample_road_yaw(road1247)
    curv = curvature_segments(samples)

    pass_worst, pass_seg = worst_in_range(curv, 80, 85)
    stop_worst, stop_seg = worst_in_range(curv, 93, 96)

    csv_rows = load_csv_speeds(1247)
    lines.append(f"- CSV: {CSV_PATH.name}, road1247 행수={len(csv_rows)}")
    lines.append("")
    lines.append("### 통과 구간 s=80~85")
    for c in pass_seg:
        v = nearest_speed(csv_rows, c["s0"])
        vstr = f"{v[1]:.2f}m/s(t={v[2]:.2f}s)" if v else "csv없음"
        lines.append(f"- s={c['s0']:.1f}~{c['s1']:.1f} dyaw={c['dyaw']:+.2f}deg "
                     f"rate={c['rate']:+.2f}deg/m radius={c['radius']:.2f}m speed~={vstr}")
    lines.append(f"- **최소 곡률반경(통과)**: {pass_worst['radius']:.2f}m "
                 f"(s={pass_worst['s0']:.1f}~{pass_worst['s1']:.1f}, rate={pass_worst['rate']:.2f}deg/m)")
    lines.append("")
    lines.append("### 정지 구간 s=93~96")
    for c in stop_seg:
        v = nearest_speed(csv_rows, c["s0"])
        vstr = f"{v[1]:.2f}m/s(t={v[2]:.2f}s)" if v else "csv없음(정지 이후 구간)"
        lines.append(f"- s={c['s0']:.1f}~{c['s1']:.1f} dyaw={c['dyaw']:+.2f}deg "
                     f"rate={c['rate']:+.2f}deg/m radius={c['radius']:.2f}m speed~={vstr}")
    lines.append(f"- **최소 곡률반경(정지)**: {stop_worst['radius']:.2f}m "
                 f"(s={stop_worst['s0']:.1f}~{stop_worst['s1']:.1f}, rate={stop_worst['rate']:.2f}deg/m)")
    lines.append("")

    ratio = stop_worst["radius"] / pass_worst["radius"] if pass_worst["radius"] > 0 else float("nan")
    lines.append(f"- 비교: 정지구간 반경/통과구간 반경 = {ratio:.3f}")
    threshold_rate = min(abs(pass_worst["rate"]), abs(stop_worst["rate"]))
    lines.append(f"- 참고용 rate 값: 통과 {pass_worst['rate']:.2f}deg/m, 정지 {stop_worst['rate']:.2f}deg/m")
    lines.append("")

    # --- A-2 ---
    lines.append("## A-2. 확정경로 15 road 급커브 전수")
    THRESHOLD = 6.0  # deg/m, 30deg/5m 기준과 동일
    lines.append(f"- 기준: rate>={THRESHOLD}deg/m (30deg/5m 환산과 동일)")
    sharp = find_sharp_turns(roads, THRESHOLD)
    merged = merge_adjacent(sharp)
    n_plain = sum(1 for m in merged if m["junction"] == "-1")
    n_conn = sum(1 for m in merged if m["junction"] != "-1")
    lines.append(f"- 급커브 구간 수: {len(merged)} (plain road {n_plain} / connector {n_conn})")
    for m in sorted(merged, key=lambda x: x["min_radius"]):
        kind = "plain" if m["junction"] == "-1" else f"connector(junction{m['junction']})"
        lines.append(f"- road{m['road']} [{kind}] s={m['s0']:.1f}~{m['s1']:.1f} "
                     f"총꺾임={m['dyaw_total']:+.1f}deg 구간길이={m['s1']-m['s0']:.2f}m "
                     f"최소반경={m['min_radius']:.2f}m")
    lines.append("")

    # --- A-3 ---
    lines.append("## A-3. road1247 원본 OSM 대응")
    j2_name = junctions["2"]["name"]
    j1_name = junctions["1"]["name"]
    lines.append(f"- road1247 predecessor=junction2(OSM노드 {j2_name}), "
                 f"successor=junction1(OSM노드 {j1_name})")

    osm_raw_nodes = load_osm_nodes(OSM_RAW_PATH)
    ways_conv = load_osm_ways(OSM_CONV_PATH)
    osm_conv_nodes = load_osm_nodes(OSM_CONV_PATH)
    adj = build_node_graph(ways_conv)
    path = bfs_path(adj, j2_name, j1_name)

    if path is None:
        lines.append("- **경고**: 변환입력 OSM(tags73)에서 두 노드 간 경로를 찾지 못함. "
                     "원본(cbnu_campus.osm)으로 재시도 필요")
    else:
        lines.append(f"- OSM 노드 체인 길이: {len(path)}개 노드")
        # 위경도, 누적거리, 꺾임각
        node_info = []
        cum = 0.0
        for i, nid in enumerate(path):
            lat, lon = osm_conv_nodes.get(nid, osm_raw_nodes.get(nid, (None, None)))
            if i > 0:
                lat0, lon0 = node_info[-1]["lat"], node_info[-1]["lon"]
                d = haversine_m(lat0, lon0, lat, lon)
                cum += d
            else:
                d = 0.0
            node_info.append({"id": nid, "lat": lat, "lon": lon, "d_prev": d, "cum": cum})
        for ni in node_info:
            lines.append(f"  - 노드{ni['id']} lat={ni['lat']:.6f} lon={ni['lon']:.6f} "
                         f"간격={ni['d_prev']:.2f}m 누적={ni['cum']:.2f}m")

        # 꺾임각(노드별, 이전-다음 방위각 차)
        lines.append("")
        lines.append("### 노드간 방위각 변화")
        for i in range(1, len(node_info) - 1):
            a, b, c = node_info[i - 1], node_info[i], node_info[i + 1]
            br1 = bearing_deg(a["lat"], a["lon"], b["lat"], b["lon"])
            br2 = bearing_deg(b["lat"], b["lon"], c["lat"], c["lon"])
            dbr = ((br2 - br1 + 180) % 360) - 180
            lines.append(f"  - 노드{b['id']}(누적{b['cum']:.1f}m): 진입방위={br1:.1f}deg "
                         f"진출방위={br2:.1f}deg 꺾임={dbr:+.1f}deg")

        # xodr s=93~96 을 위경도로 변환해 어느 누적거리 구간에 해당하는지 표시
        roads_all = roads
        j_xy = junction_xodr_locations(roads_all, junctions)
        pairs = []
        for jid, junction in junctions.items():
            name = junction["name"]
            if name in osm_raw_nodes and jid in j_xy:
                lat, lon = osm_raw_nodes[name]
                x, y = j_xy[jid]
                pairs.append((name, x, y, lat, lon))
        ref_lat = sum(p[3] for p in pairs) / len(pairs)
        ref_lon = sum(p[4] for p in pairs) / len(pairs)
        src_xy = [(p[1], p[2]) for p in pairs]
        dst_en = [latlon_to_enu(p[3], p[4], ref_lat, ref_lon) for p in pairs]
        R, s_scale, t, angle_deg, residuals = fit_similarity(src_xy, dst_en)
        rms = math.sqrt(sum(r ** 2 for r in residuals) / len(residuals))
        lines.append("")
        lines.append(f"### 유사변환 적합(참고): 스케일={s_scale:.6f} 회전={angle_deg:.3f}deg "
                     f"잔차RMS={rms:.3f}m (보정점 {len(pairs)}개)")

        for s_target in (93.0, 96.0):
            geom = None
            for g in road1247["geoms"]:
                if g["s"] - 1e-6 <= s_target <= g["s"] + g["length"] + 1e-6:
                    geom = g
            p = (s_target - geom["s"]) / geom["length"]
            x, y = geom_point(geom, p)
            lat, lon = enu_to_latlon(*apply_transform(R, s_scale, t, x, y), ref_lat, ref_lon)
            # node_info 중 최근접
            best = min(node_info, key=lambda ni: haversine_m(ni["lat"], ni["lon"], lat, lon))
            dist = haversine_m(best["lat"], best["lon"], lat, lon)
            lines.append(f"- xodr s={s_target:.1f} -> 변환위경도 lat={lat:.6f} lon={lon:.6f} "
                         f"-> 최근접 OSM노드 {best['id']}(누적{best['cum']:.1f}m) 거리{dist:.2f}m")

    OUT_PATH.write_text("\n".join(lines), encoding="utf-8")
    print(f"완료: {OUT_PATH}")
    print(f"A-1 통과반경={pass_worst['radius']:.2f}m 정지반경={stop_worst['radius']:.2f}m")
    print(f"A-2 급커브 {len(merged)}건 (plain {n_plain} / connector {n_conn})")


if __name__ == "__main__":
    main()
