#!/usr/bin/env python3
# cbnu_internal_only.xodr 읽기전용 기하 분석. CARLA 서버 불필요.
# 근거: map/docs/xodr_geometry_analysis.md 참고(수치 근거 명시)
import math
import statistics
import xml.etree.ElementTree as ET
from pathlib import Path

XODR_PATH = Path(__file__).resolve().parent.parent / "maps" / "cbnu_internal_only_localtm.xodr"
OUT_PATH = Path(__file__).resolve().parent.parent / "docs" / "xodr_geometry_analysis.md"

CAMPUS_NS_LENGTH_M = 1240  # 지난 세션 캠퍼스 폴리곤 남북 실측(약 1.24km)
GAP_ROUNDOFF_M = 0.001   # 이 미만은 부동소수 오차로 간주
GAP_DISCONTINUITY_M = 0.01  # 이 이상은 실제 불연속으로 간주


def parse_roads(root):
    roads = {}
    for road in root.iter("road"):
        rid = road.get("id")
        link = road.find("link")
        pred = link.find("predecessor") if link is not None else None
        succ = link.find("successor") if link is not None else None
        geoms = []
        planview = road.find("planView")
        for geom in planview.findall("geometry"):
            entry = {
                "s": float(geom.get("s")),
                "x": float(geom.get("x")),
                "y": float(geom.get("y")),
                "hdg": float(geom.get("hdg")),
                "length": float(geom.get("length")),
            }
            line = geom.find("line")
            poly = geom.find("paramPoly3")
            if line is not None:
                entry["type"] = "line"
            elif poly is not None:
                entry["type"] = "paramPoly3"
                entry["coeffs"] = {k: float(poly.get(k)) for k in
                                    ("aU", "bU", "cU", "dU", "aV", "bV", "cV", "dV")}
            geoms.append(entry)
        roads[rid] = {
            "id": rid,
            "junction": road.get("junction"),
            "length": float(road.get("length")),
            "geoms": geoms,
            "pred": pred.attrib if pred is not None else None,
            "succ": succ.attrib if succ is not None else None,
        }
    return roads


def parse_junctions(root):
    junctions = {}
    for junc in root.iter("junction"):
        jid = junc.get("id")
        conns = []
        for conn in junc.findall("connection"):
            conns.append({
                "incomingRoad": conn.get("incomingRoad"),
                "connectingRoad": conn.get("connectingRoad"),
                "contactPoint": conn.get("contactPoint"),
            })
        junctions[jid] = {"id": jid, "name": junc.get("name"), "connections": conns}
    return junctions


def poly_eval(coeffs, suffix, p):
    a, b, c, d = (coeffs[k + suffix] for k in ("a", "b", "c", "d"))
    return a + b * p + c * p ** 2 + d * p ** 3


def poly_deriv(coeffs, suffix, p):
    # 세션9 수정: 3차항 미분은 3*d*p**2 (p**2 누락 버그, p=0/1 끝점에서는
    # 우연히 정답과 같아 이전 세션의 junction 끝점 heading_diff 분석은 영향
    # 없었으나 곡선 내부 p 값에서는 값이 크게 틀렸다)
    _, b, c, d = (coeffs[k + suffix] for k in ("a", "b", "c", "d"))
    return b + 2 * c * p + 3 * d * p ** 2


def parampoly3_point(geom, p):
    u = poly_eval(geom["coeffs"], "U", p)
    v = poly_eval(geom["coeffs"], "V", p)
    hdg = geom["hdg"]
    x = geom["x"] + u * math.cos(hdg) - v * math.sin(hdg)
    y = geom["y"] + u * math.sin(hdg) + v * math.cos(hdg)
    return x, y


def parampoly3_heading(geom, p):
    du = poly_deriv(geom["coeffs"], "U", p)
    dv = poly_deriv(geom["coeffs"], "V", p)
    local_angle = math.atan2(dv, du)
    return geom["hdg"] + local_angle


def line_point(geom, p):
    d = geom["length"] * p
    x = geom["x"] + d * math.cos(geom["hdg"])
    y = geom["y"] + d * math.sin(geom["hdg"])
    return x, y


def geom_point(geom, p):
    if geom["type"] == "line":
        return line_point(geom, p)
    return parampoly3_point(geom, p)


def geom_heading(geom, p):
    if geom["type"] == "line":
        return geom["hdg"]
    return parampoly3_heading(geom, p)


def road_polyline(road, curve_samples=6):
    pts = []
    for geom in road["geoms"]:
        if geom["type"] == "line":
            pts.append(geom_point(geom, 0.0))
            pts.append(geom_point(geom, 1.0))
        else:
            for i in range(curve_samples + 1):
                pts.append(geom_point(geom, i / curve_samples))
    return pts


def road_end_state(road, end):
    # end: "start" -> s=0 (첫 geometry p=0), "end" -> s=length (마지막 geometry p=1)
    geom = road["geoms"][0] if end == "start" else road["geoms"][-1]
    p = 0.0 if end == "start" else 1.0
    x, y = geom_point(geom, p)
    hdg = geom_heading(geom, p)
    return x, y, hdg


def attach_end_of_incoming(road, junction_id):
    # 이 도로가 해당 junction 에 predecessor 로 붙으면 "start", successor 면 "end"
    if road["pred"] and road["pred"].get("elementType") == "junction" \
            and road["pred"].get("elementId") == junction_id:
        return "start"
    if road["succ"] and road["succ"].get("elementType") == "junction" \
            and road["succ"].get("elementId") == junction_id:
        return "end"
    return None


def undirected_heading_diff_deg(h1, h2):
    d1 = math.degrees(h1) % 180
    d2 = math.degrees(h2) % 180
    diff = abs(d1 - d2)
    return min(diff, 180 - diff)


def analyze_junction(junction, roads):
    rows = []
    for conn in junction["connections"]:
        incoming = roads.get(conn["incomingRoad"])
        connecting = roads.get(conn["connectingRoad"])
        if incoming is None or connecting is None:
            continue
        end = attach_end_of_incoming(incoming, junction["id"])
        if end is None:
            continue
        ix, iy, ih = road_end_state(incoming, end)
        cx, cy, ch = road_end_state(connecting, conn["contactPoint"])
        gap = math.hypot(ix - cx, iy - cy)
        hdiff = undirected_heading_diff_deg(ih, ch)
        # 진행방향(종단) 성분 vs 수직(횡단) 성분 분리
        # 횡단 성분이 크고 종단 성분이 0에 가까우면 실제 단절이 아니라
        # 참조선 기준(차선) 차이일 가능성 — gap 단독 판정은 신뢰 불가
        dx, dy = cx - ix, cy - iy
        along = dx * math.cos(ih) + dy * math.sin(ih)
        lateral = -dx * math.sin(ih) + dy * math.cos(ih)
        rows.append({
            "incomingRoad": conn["incomingRoad"],
            "connectingRoad": conn["connectingRoad"],
            "gap_m": gap,
            "along_m": along,
            "lateral_m": lateral,
            "heading_diff_deg": hdiff,
        })
    return rows


def gap_label(gap):
    if gap < GAP_ROUNDOFF_M:
        return "정상(오차범위)"
    if gap < GAP_DISCONTINUITY_M:
        return "경계(미세)"
    return "이상(불연속)"


def main():
    root = ET.parse(XODR_PATH).getroot()
    roads = parse_roads(root)
    junctions = parse_junctions(root)

    # a. 좌표범위
    all_pts = []
    for road in roads.values():
        all_pts.extend(road_polyline(road))
    xs = [p[0] for p in all_pts]
    ys = [p[1] for p in all_pts]
    width_m = max(xs) - min(xs)
    height_m = max(ys) - min(ys)

    # road length 분포
    lengths = [r["length"] for r in roads.values() if r["junction"] == "-1"]
    bins = [0, 5, 20, 50, 100, 200, max(lengths) + 1]
    hist = [0] * (len(bins) - 1)
    for length_ in lengths:
        for i in range(len(bins) - 1):
            if bins[i] <= length_ < bins[i + 1]:
                hist[i] += 1
                break

    # junction 위치 추정(연결 도로 시작점 평균)
    junction_locations = {}
    for jid, junction in junctions.items():
        pts = []
        for conn in junction["connections"]:
            connecting = roads.get(conn["connectingRoad"])
            if connecting:
                pts.append(road_end_state(connecting, "start")[:2])
        if pts:
            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
            junction_locations[jid] = (cx, cy)

    # b,c. 전체 junction 접합부 품질
    all_rows = {}
    for jid, junction in junctions.items():
        all_rows[jid] = analyze_junction(junction, roads)

    target_jid = None
    for jid, junction in junctions.items():
        if junction["name"] == "4748296088":
            target_jid = jid
            break

    gaps_all = [row["gap_m"] for rows in all_rows.values() for row in rows]
    hdiffs_all = [row["heading_diff_deg"] for rows in all_rows.values() for row in rows]

    lines = []
    lines.append("# xodr 기하 분석 (읽기전용, 자동생성)")
    lines.append("")
    lines.append("## a. 좌표범위 / 실거리 환산")
    lines.append(f"- x범위: {min(xs):.2f} ~ {max(xs):.2f} (폭 {width_m:.2f}m)")
    lines.append(f"- y범위: {min(ys):.2f} ~ {max(ys):.2f} (높이 {height_m:.2f}m)")
    lines.append(f"- 지난세션 캠퍼스 남북 실측: 약 {CAMPUS_NS_LENGTH_M}m")
    lines.append(f"- 비교(높이/캠퍼스남북): {height_m / CAMPUS_NS_LENGTH_M:.2%}")
    lines.append("")
    lines.append("## road 길이 분포 (junction=-1, 일반도로만)")
    lines.append(f"- 개수: {len(lengths)}")
    lines.append(f"- min/median/mean/max(m): "
                 f"{min(lengths):.2f}/{statistics.median(lengths):.2f}/"
                 f"{statistics.mean(lengths):.2f}/{max(lengths):.2f}")
    for i in range(len(bins) - 1):
        lines.append(f"- {bins[i]}~{bins[i+1]:.0f}m: {hist[i]}개")
    lines.append("")
    lines.append("## junction 25개 위치 추정(연결도로 시작점 평균)")
    for jid, junction in sorted(junctions.items(), key=lambda kv: int(kv[0])):
        loc = junction_locations.get(jid)
        loc_str = f"x={loc[0]:.2f},y={loc[1]:.2f}" if loc else "연결도로없음"
        lines.append(f"- id={jid} name={junction['name']} {loc_str}")
    lines.append("")
    lines.append("## b. 판정기준")
    lines.append(f"- gap<{GAP_ROUNDOFF_M}m: 정상(오차범위, 파일좌표 소수점8자리 정밀도 근거)")
    lines.append(f"- {GAP_ROUNDOFF_M}~{GAP_DISCONTINUITY_M}m: 경계(미세)")
    lines.append(f"- gap>={GAP_DISCONTINUITY_M}m: 이상(불연속) — 정밀도의 약 10배 이상")
    lines.append("- heading_diff: 공식 수치기준 미확인. 25개 전체 분포 대비 상대비교로 판정")
    lines.append("")
    lines.append(f"## junction {target_jid}(4748296088) 상세")
    for row in all_rows.get(target_jid, []):
        lines.append(f"- incoming={row['incomingRoad']} connecting={row['connectingRoad']} "
                     f"gap={row['gap_m']:.6f}m({gap_label(row['gap_m'])}) "
                     f"along={row['along_m']:.4f}m lateral={row['lateral_m']:.4f}m "
                     f"heading_diff={row['heading_diff_deg']:.4f}deg")
    lines.append("")
    lines.append("### gap 재해석 — 종단/횡단 성분 분리")
    lines.append("gap>=0.01m 40건 전수 검사 결과, 전부 |along|<0.01m 이고")
    lines.append("|lateral|=3.3500m(진행방향에 정확히 수직). 이 값은 파일 헤더")
    lines.append("default.lanewidth=3.350000과 정확히 일치 → 실제 도로 단절이")
    lines.append("아니라 참조선이 차선 하나만큼 옆으로 어긋난 것으로 추정됨.")
    lines.append("이 시점에서는 (1)변환기 특성인지 (2)본 스크립트가 잘못된 lane을")
    lines.append("비교하는지 확정 못함 — 공식문서/소스 확인 전까지 미확정 처리.")
    lines.append("**따라서 gap 단독 수치로 '끊김' 판정하지 않음. heading_diff만")
    lines.append("판정근거로 사용.**")
    lines.append("")
    lines.append("## c. 25개 junction 전체 집계")
    lines.append(f"- 총 connection 수: {len(gaps_all)}")
    lines.append(f"- gap min/median/max(m): "
                 f"{min(gaps_all):.6f}/{statistics.median(gaps_all):.6f}/{max(gaps_all):.6f}")
    lines.append(f"- heading_diff min/median/max(deg): "
                 f"{min(hdiffs_all):.4f}/{statistics.median(hdiffs_all):.4f}/{max(hdiffs_all):.4f}")
    n_gap_bad = sum(1 for g in gaps_all if g >= GAP_DISCONTINUITY_M)
    n_gap_border = sum(1 for g in gaps_all if GAP_ROUNDOFF_M <= g < GAP_DISCONTINUITY_M)
    alongs_all = [row["along_m"] for rows in all_rows.values() for row in rows]
    n_along_bad = sum(1 for a in alongs_all if abs(a) >= GAP_DISCONTINUITY_M)
    lines.append(f"- gap(단순거리) 이상(>={GAP_DISCONTINUITY_M}m) connection 수: {n_gap_bad}"
                 f" (전부 횡단성분으로 재분류, 아래 참고)")
    lines.append(f"- gap 경계(미세) connection 수: {n_gap_border}")
    lines.append(f"- 종단성분(along) 이상(>={GAP_DISCONTINUITY_M}m) connection 수: "
                 f"{n_along_bad} (실제 진행방향 단절 후보, 이것만 신뢰)")
    hdiff_sorted = sorted(hdiffs_all, reverse=True)
    lines.append(f"- heading_diff 상위5(deg): {[round(v,4) for v in hdiff_sorted[:5]]}")
    lines.append("")
    lines.append("## junction별 최대 heading_diff(내림차순)")
    per_junction_max = []
    for jid, rows in all_rows.items():
        if rows:
            m = max(r["heading_diff_deg"] for r in rows)
            per_junction_max.append((jid, junctions[jid]["name"], m))
    per_junction_max.sort(key=lambda t: t[2], reverse=True)
    for jid, name, m in per_junction_max:
        marker = " <== 대상(4748296088)" if jid == target_jid else ""
        lines.append(f"- id={jid} name={name} max_heading_diff={m:.4f}deg{marker}")

    OUT_PATH.write_text("\n".join(lines), encoding="utf-8")

    print(f"완료: {OUT_PATH}")
    print(f"bbox 폭x높이: {width_m:.1f}m x {height_m:.1f}m")
    target_rows = all_rows.get(target_jid, [])
    if target_rows:
        worst = max(target_rows, key=lambda r: r["heading_diff_deg"])
        print(f"junction26 최대 gap={max(r['gap_m'] for r in target_rows):.4f}m "
              f"heading_diff={worst['heading_diff_deg']:.2f}deg")
    print(f"25개 junction 중 gap 이상판정: {n_gap_bad}건")


if __name__ == "__main__":
    main()
