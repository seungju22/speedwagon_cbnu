#!/usr/bin/env python3
# 맵세션40 6-b: 건물 초안 확인(읽기전용, 서버 없음, 판정 없음)
# 1) s39 초안 18동과 세션25 1순위 30(logs/s25_buildings.csv tier=1)의 대조
# 2) s39 초안에서 차선 겹침 7동 각각
#    - 겹침 깊이(m): 겹친 점마다, 그 점이 속한 road(반쪽 도로)의 바깥 끝(가장 오른쪽 차선 바깥 가장자리)까지 거리.
#      = 그 점을 도로 밖으로 빼려면 옆으로 물러나야 하는 거리. 겹친 점 중 최댓값. 차선 종류별(Driving/Sidewalk 등)로도
#    - 겹친 road 가 5 노선(test_drive.py ROUTES_V2) 경로 road 인가, 주행 차선(lane -1)인가
#    - 폴리곤(s39 기본) / 외접 사각형 / 사각형 2개·3개로 나누기(긴 축을 같은 폭으로 자르고 각 조각의 폴리곤 부분을 감싼 사각형) 각각 겹침 점 수
# 검사는 s39_building_draft.py 와 같다(오프라인 carla.Map(동결본), 1m 격자, 차선 Any)
# 사용: .venv-carla/bin/python map/scripts/s40_building_check.py
import csv
import math
import sys
from pathlib import Path

import carla

HERE = Path(__file__).resolve().parent
BASE = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(BASE / "tests"))
from s22_buildings import RAW_OSM, Frame, load_osm  # noqa: E402
from s39_building_draft import check_points, min_rect  # noqa: E402

sys.argv = sys.argv[:1]
import test_drive as td  # noqa: E402

XODR = BASE / "maps/cbnu_campus_frozen_v2.xodr"
DRAFT = BASE / "data/processed/s39_building_draft.csv"
S25 = BASE / "docs/logs/s25_buildings.csv"
ROUTE_ROADS = {rid: k for k, v in td.ROUTES_V2.items() for rid in v["roads"]}


def clip(poly, a, b, u_axis):
    """poly 를 u in [a, b] 띠로 자른다(Sutherland-Hodgman, 두 반평면)"""
    def cut(pts, keep, inter):
        out = []
        for i, p in enumerate(pts):
            q = pts[i - 1]
            if keep(p):
                if not keep(q):
                    out.append(inter(q, p))
                out.append(p)
            elif keep(q):
                out.append(inter(q, p))
        return out
    ux, uy = u_axis
    u = lambda p: p[0] * ux + p[1] * uy

    def at(c):
        return lambda q, p: (q[0] + (p[0] - q[0]) * (c - u(q)) / (u(p) - u(q)),
                             q[1] + (p[1] - q[1]) * (c - u(q)) / (u(p) - u(q)))
    pts = cut(poly, lambda p: u(p) >= a, at(a))
    return cut(pts, lambda p: u(p) <= b, at(b)) if pts else []


def split_rects(poly, k):
    r = min_rect(poly)
    ex, ey = r[1][0] - r[0][0], r[1][1] - r[0][1]
    fx, fy = r[3][0] - r[0][0], r[3][1] - r[0][1]
    if math.hypot(ex, ey) < math.hypot(fx, fy):
        ex, ey, fx, fy = fx, fy, ex, ey
    L = math.hypot(ex, ey)
    ux, uy = ex / L, ey / L
    vx, vy = -uy, ux
    u0 = min(p[0] * ux + p[1] * uy for p in poly)
    out = []
    for i in range(k):
        part = clip(poly, u0 + L * i / k, u0 + L * (i + 1) / k, (ux, uy))
        if len(part) < 3:
            continue
        us = [p[0] * ux + p[1] * uy for p in part]
        vs = [p[0] * vx + p[1] * vy for p in part]
        a, b, c, d = min(us), max(us), min(vs), max(vs)
        out.append([(uu * ux + vv * vx, uu * uy + vv * vy) for uu, vv in ((a, c), (b, c), (b, d), (a, d))])
    return out


def hits(cmap, pts):
    n = 0
    for x, y in pts:
        if cmap.get_waypoint(carla.Location(x, y, 0.0), project_to_road=False, lane_type=carla.LaneType.Any) is not None:
            n += 1
    return n


def depth_detail(cmap, pts):
    """겹친 점마다 (road, lane, 차선 종류, 바깥 끝까지 거리)"""
    out = []
    for x, y in pts:
        loc = carla.Location(x, y, 0.0)
        wp = cmap.get_waypoint(loc, project_to_road=False, lane_type=carla.LaneType.Any)
        if wp is None:
            continue
        c = wp.transform.location
        rv = wp.transform.get_right_vector()
        lat = (x - c.x) * rv.x + (y - c.y) * rv.y          # 차선 중심에서 오른쪽(바깥) + 방향
        d = wp.lane_width / 2 - lat
        nxt = wp.get_right_lane()
        guard = 0
        while nxt is not None and nxt.road_id == wp.road_id and guard < 6 and nxt.lane_id * wp.lane_id > 0:
            d += nxt.lane_width
            nxt = nxt.get_right_lane()
            guard += 1
        out.append((wp.road_id, wp.lane_id, str(wp.lane_type), d))
    return out


def main():
    text = XODR.read_text()
    cmap = carla.Map(XODR.stem, text)
    frame = Frame(text)
    draft = list(csv.DictReader(open(DRAFT)))
    s25 = [r for r in csv.DictReader(open(S25)) if r["tier"] == "1"]
    ids39 = {r["way_id"] for r in draft}
    ids25 = {r["osm_id"] for r in s25}
    print(f"[대조] s39 초안 {len(draft)}동(s22_buildings.py KEY_NAMES 이름 일치) / s25 1순위 {len(s25)}동(tier=1)")
    print(f"  둘 다: {len(ids39 & ids25)}  s39 에만: {len(ids39 - ids25)}  s25 1순위에만: {len(ids25 - ids39)}")
    for r in draft:
        if r["way_id"] not in ids25:
            print(f"  s39 에만: {r['name']} way{r['way_id']}")
    for r in s25:
        if r["osm_id"] not in ids39:
            print(f"  s25 1순위에만: {r['name'] or '(이름 없음)'} way{r['osm_id']} 노선 최소거리 {r['min_dist_m']}m")
    nodes, ways = load_osm(RAW_OSM)
    print("\n[겹침 7동] 깊이 = 겹친 점에서 그 반쪽 도로 바깥 끝까지 거리(m), 최댓값")
    for r in draft:
        if r["draft"] != "차선 겹침":
            continue
        nds, _ = ways[r["way_id"]]
        ll = [nodes[i] for i in nds if i in nodes]
        if ll[0] == ll[-1]:
            ll = ll[:-1]
        poly = [frame.carla(*p) for p in ll]
        det = depth_detail(cmap, check_points(poly))
        by_type = {}
        for rid, lid, lt, d in det:
            by_type[lt] = max(by_type.get(lt, 0.0), d)
        roads = sorted({(rid, lid) for rid, lid, _, _ in det})
        on_route = sorted({f"r{rid} lane{lid}({ROUTE_ROADS[rid]})" for rid, lid, _, _ in det if rid in ROUTE_ROADS})
        n_poly = len(det)
        n_rect = hits(cmap, check_points(min_rect(poly)))
        sp = {}
        for k in (2, 3):
            rs = split_rects(poly, k)
            sp[k] = sum(hits(cmap, check_points(q)) for q in rs)
        print(f"- {r['name']} way{r['way_id']}: 겹친 점 폴리곤 {n_poly} / 사각형 {n_rect} / 2분할 {sp[2]} / 3분할 {sp[3]}")
        print(f"  최대 깊이 {max(d for *_, d in det):.2f}m, 종류별 {{{', '.join(f'{k}: {v:.2f}' for k, v in sorted(by_type.items()))}}}")
        print(f"  겹친 (road, lane) {roads}")
        print(f"  노선 경로 road: {on_route or '없음'}")


if __name__ == "__main__":
    main()
