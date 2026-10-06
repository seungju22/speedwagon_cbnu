#!/usr/bin/env python3
# 맵세션26: 발표 각주 "경계 밖 24m", "게이트 노드까지 약 41m" 의 출처 생성(읽기전용, 서버 없음).
# road1247 lane-1 을 0.1m 간격으로 따라가며 캠퍼스 폴리곤에 처음 들어가는 s, 출발 노드와 OSM '정문' 노드 거리
import math
import sys
sys.path.insert(0, "/home/gyumin/campus_mobility_sim/map/scripts")
from s19_scope_common import BASE, RAW_OSM, XODR_V1, load_osm, load_map, tm, OFF_X, OFF_Y  # noqa
from classify_internal import load_polygon_rings, point_in_polygon  # noqa
from s23_field_survey import carla_to_ll  # noqa
rings = load_polygon_rings(BASE / "data/processed/cbnu_relation_polygon.json")
nodes, _ = load_osm(RAW_OSM)
cmap = load_map(XODR_V1)
first_in = None
s = 0.0
while s < 150:
    wp = cmap.get_waypoint_xodr(1247, -1, s)
    lat, lon = carla_to_ll(wp.transform.location.x, wp.transform.location.y)
    if point_in_polygon(lon, lat, rings):
        first_in = (s, lat, lon)
        break
    s += 0.1
print(f"road1247 lane-1 폴리곤 진입 s={first_in[0]:.1f}m 위치 {first_in[1]:.6f}, {first_in[2]:.6f}")
a, g = nodes["2261340221"], nodes["6394475285"]
ax, ay = tm(*a)
gx, gy = tm(*g)
print(f"출발 노드 2261340221 -> OSM 정문 노드 6394475285 직선 {math.hypot(gx - ax, gy - ay):.1f}m")
wp0 = cmap.get_waypoint_xodr(1247, -1, 0.0)
print(f"road1247 lane-1 s=0 위경도 {carla_to_ll(wp0.transform.location.x, wp0.transform.location.y)}")
# 기준선(= OSM way392532207 중심선) 기준: 출발 노드부터 0.1m 간격 진입점
_, ways = load_osm(RAW_OSM)
nd = ways["392532207"]["nds"]
if nd[0] != "2261340221":
    nd = nd[::-1]
pts = [nodes[n] for n in nd]
acc = 0.0
done = False
for i in range(len(pts) - 1):
    (x1, y1), (x2, y2) = tm(*pts[i]), tm(*pts[i + 1])
    L = math.hypot(x2 - x1, y2 - y1)
    k = int(L / 0.1)
    for j in range(k + 1):
        t = j / max(k, 1)
        la = pts[i][0] + t * (pts[i + 1][0] - pts[i][0])
        lo = pts[i][1] + t * (pts[i + 1][1] - pts[i][1])
        if point_in_polygon(lo, la, rings):
            print(f"way392532207 중심선(출발 노드 기준) 폴리곤 진입 {acc + t * L:.1f}m 위치 {la:.6f}, {lo:.6f}")
            done = True
            break
    if done:
        break
    acc += L
