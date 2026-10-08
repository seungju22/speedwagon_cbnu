#!/usr/bin/env python3
# 맵세션34: 후문 주행 CSV 를 r1391 기준선(xodr 직접 계산)에 투영해 횡 위치 t 와 조향·자세를 s 별로 본다(읽기전용)
# t 는 OpenDRIVE 좌표(왼쪽 +). lane -1 = t in [-3.35, 0]. CARLA y = -xodr y
# 사용: python3 s34_track_vs_geom.py <xodr> <drive_log csv> <road id> [road id ...]
import csv
import math
import sys
import xml.etree.ElementTree as ET
sys.path.insert(0, __file__.rsplit("/", 1)[0])
from s34_curvature import analyze  # noqa: E402

root = ET.parse(sys.argv[1]).getroot()
roads = {int(r.get("id")): r for r in root.iter("road")}
rows = list(csv.DictReader(open(sys.argv[2])))
for rid in map(int, sys.argv[3:]):
    _, L, smp, _, _, _ = analyze(roads[rid])
    print(f"== r{rid} (길이 {L:.2f})")
    for r in rows:
        if r["road_id"] != str(rid):
            continue
        x, y = float(r["x"]), -float(r["y"])
        b = min(smp, key=lambda q: (q[1] - x) ** 2 + (q[2] - y) ** 2)
        s, bx, by, h, k = b[:5]
        t = -(x - bx) * math.sin(h) + (y - by) * math.cos(h)
        print(f"  t={r['t']:>7} s_geom={s:6.2f} s_carla={float(r['s']):6.2f} t_lat={t:+.2f} k={k:+.3f} v={float(r['speed_mps']):.2f} "
              f"steer={float(r['steer']):+.3f} z={r['z']} pitch={r['pitch']} roll={r['roll']} body={r['body_excess_m']} opp={r['opposite_lane']}")
