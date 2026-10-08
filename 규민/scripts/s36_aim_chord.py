#!/usr/bin/env python3
# 맵세션36: 목표점 로그의 "차 -> 목표점" 직선이 r1391 차선에서 지나는 최대 횡 위치(읽기전용)
# 사용: s36_aim_chord.py <xodr> <target_log csv> <s 하한> <s 상한>
import csv
import math
import sys
import xml.etree.ElementTree as ET
sys.path.insert(0, __file__.rsplit("/", 1)[0])
from s34_curvature import analyze  # noqa: E402
road = [r for r in ET.parse(sys.argv[1]).getroot().iter("road") if r.get("id") == "1391"][0]
_, L, smp, _, _, _ = analyze(road)
lo, hi = float(sys.argv[3]), float(sys.argv[4])


def tproj(x, y):
    q = min(smp, key=lambda q: (q[1] - x) ** 2 + (q[2] - y) ** 2)
    return q[0], -(x - q[1]) * math.sin(q[3]) + (y - q[2]) * math.cos(q[3])


seen = set()
for r in csv.DictReader(open(sys.argv[2])):
    if r["road_id"] != "1391" or not (lo <= float(r["s"]) <= hi):
        continue
    key = (r["target_road"], r["target_s"])
    if key in seen:
        continue
    seen.add(key)
    ax, ay = float(r["x"]), -float(r["y"])
    bx, by = float(r["target_x"]), -float(r["target_y"])
    best = None
    n = int(math.hypot(bx - ax, by - ay) / 0.05)
    for i in range(n + 1):
        x, y = ax + (bx - ax) * i / n, ay + (by - ay) * i / n
        s, t = tproj(x, y)
        if s >= L - 0.05:
            break
        if best is None or t > best[1]:
            best = (s, t)
    s0, t0 = tproj(ax, ay)
    print(f"차 s={float(r['s']):.2f}(t={t0:+.2f}) -> 목표 r{key[0]} s={key[1]} 거리 {r['target_dist_m']}m: "
          f"직선의 r1391 위 최대 t={best[1]:+.2f} (s={best[0]:.2f}), 차선 중심 대비 {best[1] + 1.675:+.2f}m")
