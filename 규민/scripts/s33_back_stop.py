#!/usr/bin/env python3
# 맵세션33: 후문 첫 주행 멈춘 지점 주변 연결 상태(읽기전용). 사용: s33_back_stop.py <v2 xodr> <drive_log csv> <south drive_log csv>
import csv
import math
import sys
import xml.etree.ElementTree as ET
sys.path.insert(0, "/home/gyumin/campus_mobility_sim/map/scripts")
from s19_scope_common import load_map, road_lengths, successors  # noqa: E402

x, back_csv, south_csv = sys.argv[1:4]
root = ET.parse(x).getroot()
R = {int(r.get("id")): r for r in root.iter("road")}
m, L = load_map(x), road_lengths(x)
path = [1278, 1969, 1393, 1972, 1392, 1484, 1202, 1466, 1271, 1734, 1270, 1464, 1269, 1601, 1268, 1457, 1250,
        1828, 1249, 1619, 1254, 1609, 1253, 1822, 1252, 1860, 1251, 1948, 1391, 1952, 1248, 1896, 1151]
loop = {1366, 1368, 1892, 1893, 1894}
for rid in [1251, 1948, 1391, 1952, 1248, 1896, 1151]:
    r = R[rid]
    geos = [(g.find("*").tag, float(g.get("length"))) for g in r.find("planView")]
    pre = {w.road_id for w in m.get_waypoint_xodr(rid, -1, 0.01).previous(0.3)} - {rid} if m.get_waypoint_xodr(rid, -1, 0.01) else set()
    print(f"r{rid} name={r.get('name')} junction={r.get('junction')} 길이 {L[rid][0]:.2f} 기하 {geos[:6]}"
          f"{'...' if len(geos) > 6 else ''} 앞 {sorted(pre)} 뒤 {sorted(successors(m, rid, L[rid][0]))} 고리 구성={rid in loop}")
i = path.index(1391)
left = L[1391][0] - 15.0 + sum(L[r][0] for r in path[i + 1:-1]) + 23.1
print(f"정지 r1391 s=15.0 -> 게이트 종점(r1151 s=23.1)까지 경로상 {left:.1f}m, 정문 기준 진행 {1368.2 - left:.1f}m")
wp = m.get_waypoint_xodr(1391, -1, 15.0)
print(f"r1391 s=15 차선 중심 ({wp.transform.location.x:.2f},{wp.transform.location.y:.2f}) 폭 {wp.lane_width:.2f}")
for s in (0, 5, 10, 13, 14, 15, 16, 17, L[1391][0] - 0.1):
    w = m.get_waypoint_xodr(1391, -1, s)
    print(f"  s={s:5.1f} yaw={w.transform.rotation.yaw:7.1f}")
b = list(csv.DictReader(open(back_csv)))
print("충돌 직전 5행:")
for r in b[-5:]:
    print(f"  t={r['t']} ({float(r['x']):.2f},{float(r['y']):.2f}) z={r['z']} v={r['speed_mps']} road={r['road_id']} s={r['s']} steer={r['steer']} pitch={r['pitch']} roll={r['roll']} lat={r['lat_off_m']}")
# 남문 겹침 대조: 같은 세션 남문 주행과 같은 tick 위치 차(남문 완주 시각까지)
s_ = list(csv.DictReader(open(south_csv)))
n = len(s_)
d = [math.hypot(float(p["x"]) - float(q["x"]), float(p["y"]) - float(q["y"])) for p, q in zip(b[:n], s_)]
k = max(range(len(d)), key=lambda i: d[i])
print(f"남문 겹침 구간 {n}행(t<={s_[-1]['t']}): 같은 tick 위치 차 최대 {d[k]:.2f}m(t={b[k]['t']}), 평균 {sum(d)/len(d):.3f}m")
dd = [x for x in d[: n - 200]]
print(f"  남문 정차 감속 전(마지막 10s 제외): 최대 {max(dd):.2f}m")
