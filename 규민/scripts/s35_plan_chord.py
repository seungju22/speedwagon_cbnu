#!/usr/bin/env python3
# 맵세션35: 계획점(dry-plan) 사이 직선(현)이 r1391 차선에서 어디를 지나는가(읽기전용)
# 각 현을 0.1m 로 나눠 r1391 기준선에 투영한 횡 위치 t(왼쪽 +). 차선 중심 t=-1.675, 왼쪽 끝 t=0
# 사용: s35_plan_chord.py <xodr> <dry-plan csv> <road id>
import csv
import math
import sys
import xml.etree.ElementTree as ET
sys.path.insert(0, __file__.rsplit("/", 1)[0])
from s34_curvature import analyze  # noqa: E402
root = ET.parse(sys.argv[1]).getroot()
rid = int(sys.argv[3])
_, L, smp, _, _, _ = analyze([r for r in root.iter("road") if r.get("id") == str(rid)][0])
rows = list(csv.DictReader(open(sys.argv[2])))
pts = [(int(r["road_id"]), float(r["s"]), float(r["x"]), -float(r["y"])) for r in rows]
idx = [i for i, p in enumerate(pts) if p[0] == rid]
lo, hi = max(idx[0] - 1, 0), min(idx[-1] + 1, len(pts) - 1)
print(f"r{rid} 계획점: {[(p[0], p[1]) for p in pts[lo:hi + 1]]}")
for a, b in zip(pts[lo:hi], pts[lo + 1:hi + 1]):
    worst = None
    n = max(2, int(math.hypot(b[2] - a[2], b[3] - a[3]) / 0.1))
    for i in range(n + 1):
        x, y = a[2] + (b[2] - a[2]) * i / n, a[3] + (b[3] - a[3]) * i / n
        q = min(smp, key=lambda q: (q[1] - x) ** 2 + (q[2] - y) ** 2)
        if q[0] <= 0.05 or q[0] >= L - 0.05:
            continue
        t = -(x - q[1]) * math.sin(q[3]) + (y - q[2]) * math.cos(q[3])
        if worst is None or t > worst[0]:
            worst = (t, q[0])
    if worst:
        print(f"  현 r{a[0]} s={a[1]} -> r{b[0]} s={b[1]}: r{rid} 위 최대 t={worst[0]:+.2f} (s={worst[1]:.2f}), 차선 중심 대비 {worst[0] + 1.675:+.2f}m")
