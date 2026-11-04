#!/usr/bin/env python3
# 10/12(M19): 셔틀 승강장 정보(문 기준 정차 자세·뒷축 목표·정차 구역·문 구역·전방 통로)와 서버 없이 검사
# 입력(읽기만): v3.1 후보 s54 xodr, stops_v3.yaml, Lanelet2(I6-2 지도 83edb775), buildings_layout_v3.csv,
#   place_campus_layers.py(정류장 시설 바닥 계산식·mesh_sizes), s51 mesh_sizes.json
# 좌표: 모두 지도 좌표계(map = xodr 평면 = Autoware map). CARLA (x, y, yaw) -> map (x, -y, -yaw)
# 차: Autoware 1.9.0 sample_vehicle vehicle_info(이미지 안 파일을 읽기만). base_link = 뒷바퀴 축 가운데
# 출력: map/docs/logs/s56_platforms.json(계산 결과, yaml 은 s56_write.py 가 이걸로 씀), s56_shuttle.log
# 사용: .venv-carla/bin/python map/scripts/s56_shuttle.py
import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

HERE = Path(__file__).resolve().parent
BASE = HERE.parent
ROOT = BASE.parent
sys.path.insert(0, str(HERE))
import place_campus_layers as pcl  # noqa: E402  (함수만 씀, 파일 안 고침)
from s22_buildings import Frame  # noqa: E402

XODR = BASE / "maps/cbnu_campus_v31_candidate_s54.xodr"
OSM = ROOT / "integration/maps/build_v31_s54/lanelet2_map.osm"
PAIRS = BASE / "build_v31_s54/pairs30.json"
MESH = BASE / "docs/logs/s51_bench/mesh_sizes.json"
LAYERS_S54 = BASE / "docs/logs/campus_layers_dry_나_s54_v31.json"
OUT = BASE / "docs/logs/s56_platforms.json"

# 차(vehicle_info.param.yaml, 1.9.0 이미지)
WB, FO, RO, TREAD, LO, RGO = 2.79, 1.0, 1.1, 1.64, 0.128, 0.128
FRONT, REAR = WB + FO, -RO                      # 뒷축 기준 앞끝 3.79, 뒤끝 -1.1
HALF_W = TREAD / 2 + RGO                        # 0.948
DOOR_X = (FRONT + REAR) / 2                     # 1.345: 차 길이 가운데
DOOR_Y = -HALF_W                                # 오른쪽 옆면
# 가정값(문서 map/docs/shuttle_route_file.md 에 근거와 바꾸는 법)
STOP_MARGIN_FRONT = 1.0
STOP_MARGIN_REAR = 1.0
DOOR_ZONE_ALONG = 3.0                           # 문 폭 1.2 + 양옆 0.9
DOOR_ZONE_DEPTH = 1.5                           # 차선 가장자리에서 인도 쪽으로
CORRIDOR_LEN = 8.0                              # 앞끝에서
CORRIDOR_W = 2.5                                # 차 폭 1.896 + 양쪽 약 0.3
STEP = 0.25

# 시범 노선(구간 road 열). 2구간은 s56_route.py 에서 고른 U턴 없는 길(남문 이쪽 -> 북문)
PILOT = {
    "seg1": [1281, 1867, 1282, 1773, 1283, 1766, 1284, 1739, 1285, 1471, 1271, 1734, 1270, 1464, 1269, 1601, 1268],
    "seg2": [1268, 1456, 1263, 1438, 1262, 1831, 1220, 1914, 1219, 1826, 1386, 1451, 1405, 1599, 1406, 1459, 1407, 1732,
             1335, 1716, 1333, 1724, 1413, 1663, 1414, 1678, 1415],
    "seg3": [1415, 3006, 3004, 3008, 1312, 1699, 1195, 1691, 1194, 2045, 1321, 1670, 1322, 1672, 1276, 1661, 1275, 1722,
             1274, 1628, 1273, 1705, 1272, 1478, 1255, 3002, 3001, 3003, 1150],
}
PILOT_ROADS = {r for v in PILOT.values() for r in v}


def h16(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()[:16]


def mpose(wp):
    t = wp.transform
    return t.location.x, -t.location.y, -t.rotation.yaw


def off(x, y, yaw, fx, ly):
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    return x + fx * c - ly * s, y + fx * s + ly * c


def body(x, y, yaw):
    return Polygon([off(x, y, yaw, *p) for p in ((FRONT, HALF_W), (FRONT, -HALF_W), (REAR, -HALF_W), (REAR, HALF_W))])


def pick(cands, prefer, yaw):
    """갈림길: prefer road 우선, 없으면 방위 변화가 가장 작은(직진에 가까운) 쪽"""
    if not cands:
        return None
    p = [w for w in cands if w.road_id in prefer]
    if p:
        return p[0]
    return min(cands, key=lambda w: abs((mpose(w)[2] - yaw + 180) % 360 - 180))


def walk(wp, dist, prefer, back=False):
    """wp 에서 dist(m) 만큼 STEP 간격으로 앞(뒤)으로. 반환 waypoint 목록(wp 포함)"""
    out, cur, d = [wp], wp, 0.0
    while d < dist - 1e-6:
        st = min(STEP, dist - d)
        nx = cur.previous(st) if back else cur.next(st)
        nw = pick(nx, prefer, mpose(cur)[2])
        if nw is None:
            break
        out.append(nw)
        cur, d = nw, d + st
    return out[::-1] if back else out


def span(wp, s0, s1, prefer):
    """wp 기준 차로 거리 s0..s1(음수 = 뒤) 의 waypoint 열"""
    if s0 < 0:
        a = walk(wp, -s0, prefer, back=True)
        if s1 < 0:
            return a[: len(a) - int(round(-s1 / STEP))]
        return a + (walk(wp, s1, prefer)[1:] if s1 > 0 else [])
    seq = walk(wp, s1, prefer)
    return seq[int(round(s0 / STEP)):]


def strip(wps, lat_a, lat_b, half=False):
    """waypoint 열을 따라 옆 lat_a..lat_b(왼쪽 +, m). half=True 면 lat 은 차선 반폭 기준 바깥쪽 거리(오른쪽)"""
    A, B = [], []
    for w in wps:
        x, y, yaw = mpose(w)
        if half:
            h = w.lane_width / 2
            a, b = -(h + lat_a), -(h + lat_b)
        else:
            a, b = lat_a, lat_b
        A.append(off(x, y, yaw, 0, a))
        B.append(off(x, y, yaw, 0, b))
    p = Polygon(A + B[::-1])
    return p if p.is_valid else p.buffer(0)


def flip(poly):
    return Polygon([(x, -y) for x, y in poly.exterior.coords])


def pts(poly, nd=2):
    if poly.is_empty:
        return []
    if poly.geom_type == "MultiPolygon":
        poly = max(poly.geoms, key=lambda g: g.area)
    p = poly.simplify(0.01)
    return [[round(x, nd), round(y, nd)] for x, y in list(p.exterior.coords)[:-1]]


def load_lanelets():
    r = ET.parse(OSM).getroot()
    nodes = {}
    for n in r.iter("node"):
        t = {x.get("k"): x.get("v") for x in n.findall("tag")}
        nodes[n.get("id")] = (float(t["local_x"]), float(t["local_y"]))
    ways = {w.get("id"): [nd.get("ref") for nd in w.findall("nd")] for w in r.iter("way")}
    out = []
    for rel in r.iter("relation"):
        t = {x.get("k"): x.get("v") for x in rel.findall("tag")}
        if t.get("type") != "lanelet":
            continue
        mem = {m.get("role"): m.get("ref") for m in rel.findall("member")}
        L = [nodes[q] for q in ways[mem["left"]]]
        R = [nodes[q] for q in ways[mem["right"]]]
        poly = Polygon(L + R[::-1])
        if not poly.is_valid:
            poly = poly.buffer(0)
        n = min(len(L), len(R))
        cl = [((L[int(i * (len(L) - 1) / max(n - 1, 1))][0] + R[int(i * (len(R) - 1) / max(n - 1, 1))][0]) / 2,
               (L[int(i * (len(L) - 1) / max(n - 1, 1))][1] + R[int(i * (len(R) - 1) / max(n - 1, 1))][1]) / 2)
              for i in range(max(n, 2))]
        out.append(dict(id=int(rel.get("id")), road=int(t.get("xodr_road", -1)), poly=poly, cl=cl))
    return out


def ll_heading(ll, x, y):
    cl = ll["cl"]
    best = None
    for a, b in zip(cl, cl[1:]):
        d = _segdist((x, y), a, b)
        if best is None or d < best[0]:
            best = (d, math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])))
    return best[1]


def _segdist(p, a, b):
    ax, ay = b[0] - a[0], b[1] - a[1]
    L2 = ax * ax + ay * ay
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / L2))
    return math.hypot(p[0] - a[0] - t * ax, p[1] - a[1] - t * ay)


def angdiff(a, b):
    return abs((a - b + 180) % 360 - 180)


def main():
    import carla
    text = XODR.read_text()
    cmap = carla.Map("OpenDriveMap", text)
    root = ET.fromstring(text)
    RL = {int(r.get("id")): float(r.get("length")) for r in root.iter("road")}
    frame = Frame(text)
    pcl.load_mesh_sizes(MESH)
    log = []

    def P(*a):
        s = " ".join(str(x) for x in a)
        print(s)
        log.append(s)

    P(f"xodr {XODR.name} {h16(XODR)} / lanelet2 {h16(OSM)} / stops {h16(pcl.STOPS)} / layout {h16(pcl.LAYOUT)}")
    P(f"차: 길이 {FRONT - REAR:.2f} 폭 {2 * HALF_W:.3f}, 뒷축 기준 앞 {FRONT} 뒤 {REAR}, 문 (x {DOOR_X:.3f}, y {DOOR_Y:.3f})")
    P(f"정류장 시설 along {pcl.BUS['along']} depth {pcl.BUS['depth']} off {pcl.BUS_OFF} gap {pcl.BUS_GAP}")
    platforms = pcl.parse_platforms(pcl.STOPS)
    # 차로 조각(모든 road lane -1 = 양방향 반쪽 모두). CARLA -> map
    quads = [(rid, flip(q)) for rid, q in pcl.lane_quads(cmap, root)]
    P(f"차로 조각 {len(quads)}")
    from shapely.strtree import STRtree
    qtree = STRtree([q for _, q in quads])
    rows = pcl.read_layout(pcl.LAYOUT)
    blds = []
    for r in rows:
        cx, cy = frame.carla(float(r["center_lat"]), float(r["center_lon"]))
        blds.append((r["name"] or f"way{r['way_id']}", flip(pcl.rect(cx, cy, float(r["yaw_deg"]), float(r["length_m"]),
                                                                       float(r["width_m"])))))
    lls = load_lanelets()
    ltree = STRtree([l["poly"] for l in lls])
    P(f"건물 상자 {len(blds)}, lanelet {len(lls)}")
    s54 = json.loads(LAYERS_S54.read_text())
    s54bus = {a["owner"]: a for a in s54["actors"] if a["layer"] == "busstop"}

    res = []
    for p in platforms:
        lab = p["label"]
        wp_p = cmap.get_waypoint_xodr(p["road_id"], p["lane_id"], p["s"])
        prefer = PILOT_ROADS | {p["road_id"]}
        px, py, pyaw = mpose(wp_p)
        # 뒷축 목표: 승강장 표지 점에서 차로를 따라 DOOR_X 뒤
        wr = walk(wp_p, DOOR_X, prefer, back=True)[0]
        bx, by, byaw = mpose(wr)
        dx, dy = off(bx, by, byaw, DOOR_X, DOOR_Y)
        # 문 점을 차로에 투영해 승강장 s 와 앞뒤 차
        wd = cmap.get_waypoint(carla.Location(dx, -dy, 0.0))
        ex, ey, eyaw = mpose(wp_p)
        along_err = (dx - ex) * math.cos(math.radians(eyaw)) + (dy - ey) * math.sin(math.radians(eyaw))
        # 정차 구역(차로 전폭)
        sz_wps = span(wr, REAR - STOP_MARGIN_REAR, FRONT + STOP_MARGIN_FRONT, prefer)
        A, B = [], []
        for w in sz_wps:
            x, y, yaw = mpose(w)
            A.append(off(x, y, yaw, 0, w.lane_width / 2))
            B.append(off(x, y, yaw, 0, -w.lane_width / 2))
        stop_zone = Polygon(A + B[::-1]).buffer(0)
        # 정류장 시설 바닥(place_campus_layers.busstop_actors 와 같은 식, 앞뒤 이동 0 확인)
        fac = None
        if lab not in pcl.BUSSTOP_SKIP:
            t = wp_p.transform
            r = t.get_right_vector()
            o = wp_p.lane_width / 2 + pcl.BUS_GAP + pcl.BUS["depth"] / 2
            fx, fy = t.location.x + r.x * o, t.location.y + r.y * o
            myaw = t.rotation.yaw + pcl.BUS_YAW_OFF
            c, si = math.cos(math.radians(myaw)), math.sin(math.radians(myaw))
            ax, ay = fx - (pcl.BUS_OFF[0] * c - pcl.BUS_OFF[1] * si), fy - (pcl.BUS_OFF[0] * si + pcl.BUS_OFF[1] * c)
            ref = s54bus.get(lab)
            fac_err = math.hypot(ax - ref["x"], ay - ref["y"]) if ref else None
            fac = flip(pcl.rect(fx, fy, t.rotation.yaw, pcl.BUS["along"], pcl.BUS["depth"]))
        # 문 구역(원래): 문 앞뒤 DOOR_ZONE_ALONG, 차선 가장자리 ~ 바깥 DOOR_ZONE_DEPTH
        dz_wps = span(wp_p, -DOOR_ZONE_ALONG / 2, DOOR_ZONE_ALONG / 2, prefer)
        dz_raw = strip(dz_wps, 0.0, DOOR_ZONE_DEPTH, half=True)
        # 검사용 겹침
        near_q = [quads[j] for j in qtree.query(dz_raw.buffer(0.5))]
        lane_ov = {}
        for rid, q in near_q:
            a = dz_raw.intersection(q).area
            if a > 1e-4:
                lane_ov[rid] = lane_ov.get(rid, 0.0) + a
        own = p["road_id"]
        bld_ov = {n: dz_raw.intersection(g).area for n, g in blds if g.intersects(dz_raw) and dz_raw.intersection(g).area > 1e-4}
        fac_ov = dz_raw.intersection(fac).area if fac is not None else 0.0
        # 문 구역(최종) = 원래 - 시설 - 차로(모든 방향) - 건물
        cut = [fac] if fac is not None else []
        cut += [q for _, q in near_q] + [g for n, g in blds if g.intersects(dz_raw)]
        dz = dz_raw.difference(unary_union(cut)) if cut else dz_raw
        # 최종 문 구역 다시 검사
        fin_lane = sum(dz.intersection(q).area for _, q in [quads[j] for j in qtree.query(dz.buffer(0.5))])
        fin_bld = sum(dz.intersection(g).area for _, g in blds if g.intersects(dz))
        fin_fac = dz.intersection(fac).area if fac is not None else 0.0
        fin_ll = sum(dz.intersection(lls[j]["poly"]).area for j in ltree.query(dz))
        # 인도(lane -2) 안 비율
        sw_wps = []
        for w in dz_wps:
            sw = cmap.get_waypoint_xodr(w.road_id, -2, w.s)
            if sw is not None and sw.lane_type == carla.LaneType.Sidewalk:
                sw_wps.append(sw)
        sw_poly = strip(sw_wps, -sw_wps[0].lane_width / 2, sw_wps[0].lane_width / 2) if len(sw_wps) > 1 else None
        sw_in = dz.intersection(sw_poly).area / dz.area if (sw_poly is not None and dz.area > 0) else None
        # 전방 통로: 앞끝에서 CORRIDOR_LEN, 폭 CORRIDOR_W, 차로 중심 따라
        cw = span(wr, FRONT, FRONT + CORRIDOR_LEN, prefer)
        corr = strip(cw, CORRIDOR_W / 2, -CORRIDOR_W / 2)
        corr_roads = []
        for w in cw:
            if not corr_roads or corr_roads[-1] != w.road_id:
                corr_roads.append(w.road_id)
        # 우리 차로 = 통로가 지나는 road 의 lane -1 조각 + 정차 road
        ours = set(corr_roads) | {own}
        lane_ours = unary_union([q for rid, q in [quads[j] for j in qtree.query(corr.buffer(1.0))] if rid in ours])
        corr_out = corr.difference(lane_ours.buffer(0.01)).area
        corr_other = {}
        for rid, q in [quads[j] for j in qtree.query(corr)]:
            if rid not in ours:
                a = corr.intersection(q).area
                if a > 1e-3:
                    corr_other[rid] = corr_other.get(rid, 0.0) + a
        # 같은 방향 lanelet 합
        def same_dir_ll(g, yaw):
            out = []
            for j in ltree.query(g.buffer(1.0)):
                c = g.centroid
                if angdiff(ll_heading(lls[j], c.x, c.y), yaw) < 60:
                    out.append(lls[j])
            return out
        cl_ll = same_dir_ll(corr, byaw)
        corr_ll_out = corr.difference(unary_union([l["poly"] for l in cl_ll]).buffer(0.01)).area if cl_ll else corr.area
        # 차체가 lanelet 안에
        bd = body(bx, by, byaw)
        b_ll = same_dir_ll(bd, byaw)
        ll_u = unary_union([l["poly"] for l in b_ll]) if b_ll else Polygon()
        b_out = bd.difference(ll_u.buffer(0.01)).area
        b_corner_out = max(0.0, max(ll_u.exterior.distance(Point(c)) if not ll_u.contains(Point(c)) else 0.0
                                    for c in list(bd.exterior.coords)[:-1])) if not ll_u.is_empty else None
        b_lane = bd.difference(unary_union([q for rid, q in [quads[j] for j in qtree.query(bd.buffer(1))] if rid == wr.road_id
                                            or rid in ours]).buffer(0.01)).area
        ll_at = [l for l in b_ll if l["poly"].contains(Point(bx, by))]
        ll_door = [l for l in same_dir_ll(Polygon([(px - .1, py - .1), (px + .1, py - .1), (px + .1, py + .1)]), pyaw)
                   if l["poly"].contains(Point(px, py))]
        q = (round(math.sin(math.radians(byaw) / 2), 6), round(math.cos(math.radians(byaw) / 2), 6))
        rec = dict(
            label=lab, stop_id=p["stop_id"], stop_name=p["stop_name"], side=p["side"],
            road_id=p["road_id"], s=p["s"], lane_id=p["lane_id"],
            platform=dict(x=round(px, 3), y=round(py, 3), yaw_deg=round(pyaw, 2), lane_width=round(wp_p.lane_width, 3)),
            door=dict(x=round(dx, 3), y=round(dy, 3), yaw_deg=round(byaw, 2), along_err_m=round(along_err, 3),
                      right_of_lane_center_m=round((dx - ex) * math.sin(math.radians(eyaw)) - (dy - ey) * math.cos(math.radians(eyaw)), 3),
                      proj_road=wd.road_id if wd else None, proj_s=round(wd.s, 2) if wd else None),
            base_link=dict(x=round(bx, 3), y=round(by, 3), z=0.0, yaw_deg=round(byaw, 2), yaw_rad=round(math.radians(byaw), 5),
                           qz=q[0], qw=q[1], road_id=wr.road_id, s=round(wr.s, 2)),
            i62_goal_shift_m=round(math.hypot(bx - px, by - py), 3),
            body=pts(bd, 3),
            stop_zone=dict(length_m=round(FRONT - REAR + STOP_MARGIN_FRONT + STOP_MARGIN_REAR, 2), polygon=pts(stop_zone),
                           area_m2=round(stop_zone.area, 2)),
            facility=dict(present=fac is not None, polygon=pts(fac) if fac is not None else [],
                          pivot_vs_s54_m=round(fac_err, 4) if fac is not None and fac_err is not None else None),
            door_zone=dict(polygon=pts(dz), area_m2=round(dz.area, 3), raw_area_m2=round(dz_raw.area, 3),
                           raw_overlap=dict(facility_m2=round(fac_ov, 3), lanes={str(k): round(v, 3) for k, v in lane_ov.items()},
                                            buildings={k: round(v, 3) for k, v in bld_ov.items()}),
                           final_overlap=dict(lanes_m2=round(fin_lane, 4), buildings_m2=round(fin_bld, 4),
                                              facility_m2=round(fin_fac, 4), lanelets_m2=round(fin_ll, 4)),
                           sidewalk_frac=round(sw_in, 3) if sw_in is not None else None),
            corridor=dict(polygon=pts(corr), length_m=CORRIDOR_LEN, width_m=CORRIDOR_W, roads=corr_roads,
                          out_of_own_lane_m2=round(corr_out, 4), other_lanes_m2={str(k): round(v, 3) for k, v in corr_other.items()},
                          out_of_lanelets_m2=round(corr_ll_out, 4), lanelets=sorted({l["id"] for l in cl_ll})),
            body_check=dict(out_of_lanelets_m2=round(b_out, 4), corner_out_m=round(b_corner_out, 3) if b_corner_out is not None else None,
                            out_of_xodr_lane_m2=round(b_lane, 4), lanelets=sorted({l["id"] for l in b_ll})),
            lanelet_ref=dict(base_link=[l["id"] for l in ll_at], platform=[l["id"] for l in ll_door]),
        )
        # 문 구역 최소 깊이(문 점 옆, 바깥으로 얼마나 남았는지)
        ray = strip(span(wp_p, -0.05, 0.05, prefer), 0.0, DOOR_ZONE_DEPTH, half=True)
        rec["door_zone"]["depth_at_door_m"] = round(dz.intersection(ray).area / 0.1, 3)
        res.append(rec)
        P(f"{lab}: base_link ({bx:.2f},{by:.2f},{byaw:.1f}) r{wr.road_id} s{wr.s:.2f} / 문 ({dx:.2f},{dy:.2f}) 앞뒤차 {along_err:+.3f} "
          f"/ 문구역 {dz.area:.2f}m2(원래 {dz_raw.area:.2f}, 시설 {fac_ov:.2f}, 차로 {sum(lane_ov.values()):.3f} {dict((k, round(v, 3)) for k, v in lane_ov.items())}, "
          f"건물 {sum(bld_ov.values()):.3f}) 최종 겹침 차로 {fin_lane:.4f} 건물 {fin_bld:.4f} 시설 {fin_fac:.4f} lanelet {fin_ll:.4f} 인도안 {sw_in} "
          f"문옆깊이 {rec['door_zone']['depth_at_door_m']} / 통로 road {corr_roads} 차로밖 {corr_out:.4f} 다른차로 {corr_other} lanelet밖 {corr_ll_out:.4f} "
          f"/ 차체 lanelet밖 {b_out:.4f}m2 모서리 {b_corner_out} xodr차로밖 {b_lane:.4f} / 시설 s54 대비 {rec['facility']['pivot_vs_s54_m']} "
          f"/ lanelet {rec['lanelet_ref']}")
    OUT.write_text(json.dumps(dict(xodr=XODR.name, xodr_sha16=h16(XODR), lanelet2_sha16=h16(OSM), stops_sha16=h16(pcl.STOPS),
                                   vehicle=dict(wheel_base=WB, front_overhang=FO, rear_overhang=RO, wheel_tread=TREAD,
                                                left_overhang=LO, right_overhang=RGO, length=FRONT - REAR, width=2 * HALF_W,
                                                door_x=DOOR_X, door_y=DOOR_Y),
                                   params=dict(stop_margin_front=STOP_MARGIN_FRONT, stop_margin_rear=STOP_MARGIN_REAR,
                                               door_zone_along=DOOR_ZONE_ALONG, door_zone_depth=DOOR_ZONE_DEPTH,
                                               corridor_len=CORRIDOR_LEN, corridor_w=CORRIDOR_W,
                                               facility_along=pcl.BUS["along"], facility_depth=pcl.BUS["depth"], bus_gap=pcl.BUS_GAP),
                                   platforms=res), ensure_ascii=False, indent=1))
    (BASE / "docs/logs/s56_shuttle.log").write_text("\n".join(log) + "\n")
    P(f"-> {OUT}")


if __name__ == "__main__":
    main()
