#!/usr/bin/env python3
# 맵세션36 3: r1391 겨냥 거리 기본값 역산(읽기전용, 오프라인 모형)
# 모형: 차가 r1391 차선 중심(t=-w/2) 위를 s 따라 0.05m 씩 간다고 둔다(낙관 가정: 실제 차는 이미 치우쳐 있다).
#   목표점 = LocalPlanner 큐 규칙(local_planner.py 241~263행): 큐 앞에서부터 차와 거리 < L 이면 지우고, 처음으로 >= L 인 점.
#   L = b + 0.5 v (v = D 적용 속도, 주행 CSV 평균). 이탈 = 차 -> 목표점 직선의 r1391 위 최대 t - 차선 중심
# b(base_min_distance) 후보를 0.05m 단위로 내려가며 r1391 전 구간 최대 이탈 <= 여유 절반 이 되는 가장 큰 b 를 고른다
# 사용: s36_invert_aim.py <xodr> <drive_plan csv(서버 계획)> <v m/s> <여유 절반 m>
import csv
import math
import sys
import xml.etree.ElementTree as ET
import numpy as np
sys.path.insert(0, __file__.rsplit("/", 1)[0])
from s34_curvature import analyze  # noqa: E402

root = ET.parse(sys.argv[1]).getroot()
road = [r for r in root.iter("road") if r.get("id") == "1391"][0]
_, L391, smp, _, _, _ = analyze(road)
w = 3.35
plan = [(int(r["road_id"]), float(r["s"]), float(r["x"]), -float(r["y"])) for r in csv.DictReader(open(sys.argv[2]))]
v, half = float(sys.argv[3]), float(sys.argv[4])
i0 = next(i for i, p in enumerate(plan) if p[0] == 1391) - 1


A = np.array([q[:4] for q in smp])   # s, x, y, hdg


def tproj_many(xs, ys):
    d = (A[None, :, 1] - xs[:, None]) ** 2 + (A[None, :, 2] - ys[:, None]) ** 2
    j = d.argmin(axis=1)
    q = A[j]
    return q[:, 0], -(xs - q[:, 1]) * np.sin(q[:, 3]) + (ys - q[:, 2]) * np.cos(q[:, 3])


def center(s):
    q = min(smp, key=lambda q: abs(q[0] - s))
    return q[1] + (w / 2) * math.sin(q[3]), q[2] - (w / 2) * math.cos(q[3])


def worst(b):
    Lk = b + 0.5 * v
    k = i0
    out = (0.0, None)
    s = 0.0
    while s <= L391 - 0.05:
        cx, cy = center(s)
        while k < len(plan) - 1 and math.hypot(plan[k][2] - cx, plan[k][3] - cy) < Lk:
            k += 1
        tx, ty = plan[k][2], plan[k][3]
        n = max(2, int(math.hypot(tx - cx, ty - cy) / 0.1))
        f = np.linspace(0, 1, n + 1)
        ss, t = tproj_many(cx + (tx - cx) * f, cy + (ty - cy) * f)
        ok = np.cumprod(ss < L391 - 0.05).astype(bool)   # r1391 끝을 넘은 뒤는 제외
        if ok.any():
            dev = t[ok] + w / 2
            j = int(dev.argmax())
            if dev[j] > out[0]:
                out = (float(dev[j]), (round(s, 2), plan[k][0], plan[k][1], round(float(ss[ok][j]), 2)))
        s += 0.05
    return Lk, out


if len(sys.argv) > 5:   # 추가 인자: b 스캔 목록
    for b in map(float, sys.argv[5:]):
        Lk, (d, at) = worst(b)
        print(f"  스캔 b={b:.2f} L={Lk:.3f}: 최대 이탈 {d:.3f}m {at}")
    sys.exit(0)
print(f"v={v} m/s, 여유 절반 {half} m, r1391 계획점 {[p[1] for p in plan if p[0] == 1391]}")
for b in (3.0, 2.5, 2.0):
    Lk, (d, at) = worst(b)
    print(f"  b={b:.2f} L={Lk:.3f}: 최대 이탈 {d:.3f}m (차 s, 목표 road, 목표 s, 최대 지점 s)={at}")
b = 3.0
chosen = None
while b >= 0.5:
    Lk, (d, at) = worst(b)
    if d <= half:
        chosen = (b, Lk, d, at)
        break
    b = round(b - 0.05, 2)
print(f"선택: 최대 이탈 <= {half} 인 가장 큰 b = {chosen}")
