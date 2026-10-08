#!/usr/bin/env python3
# 맵세션36 2·3: 현(chord) 이탈 모형(읽기전용). 값은 xodr·코드·주행 CSV 에서 직접 읽는다
# 겨냥 거리 L = base_min_distance + distance_ratio * v (local_planner.py 87·88·241행: 3.0, 0.5, v=m/s)
# 예측 이탈 = R (1 - cos(asin(L / 2R)))   (반경 R 원호 위 길이 L 현의 가운데 처짐)
# 여유 = 반차선폭(xodr width/2) - 차량 반폭(vehicle_bbox extent_y)
# 사용: s36_model.py <xodr> <drive_log csv(감속 적용 회차)> <충돌 s> <bbox txt>
import csv
import math
import re
import sys
import xml.etree.ElementTree as ET
sys.path.insert(0, __file__.rsplit("/", 1)[0])
from s34_curvature import analyze, edge_r  # noqa: E402

xodr, dlog, s_hit, bbox = sys.argv[1], sys.argv[2], float(sys.argv[3]), sys.argv[4]
LP = open("/home/gyumin/carla/CARLA_0.9.15/PythonAPI/carla/agents/navigation/local_planner.py").read()
base = float(re.search(r"self\._base_min_distance = ([\d.]+)", LP).group(1))
ratio = float(re.search(r"self\._distance_ratio = ([\d.]+)", LP).group(1))
road = [r for r in ET.parse(xodr).getroot().iter("road") if r.get("id") == "1391"][0]
w = float(road.find("lanes").find("laneSection").find("right").find("lane").find("width").get("a"))
half_car = float(re.search(r"extent_y\(half_width\)=([\d.]+)", open(bbox).read()).group(1))
_, L, rows, _, _, _ = analyze(road)
rc = [(s, edge_r(k, off - wd / 2), k) for s, x, y, h, k, off, wd, ws, t in rows if abs(k) > 1e-9]
near = [r for r in rc if abs(r[0] - s_hit) <= 0.5]
R_hit = min(near, key=lambda r: abs(r[0] - s_hit))
R_win = min(r[1] for r in near)
d = [r for r in csv.DictReader(open(dlog)) if r["road_id"] == "1391"]
v = [float(r["speed_mps"]) for r in d if abs(float(r["s"]) - s_hit) <= 2.0]
vm = sum(v) / len(v)
Lk = base + ratio * vm
margin = w / 2 - half_car


def sag(R, Lc):
    return R * (1 - math.cos(math.asin(min(1.0, Lc / (2 * R)))))


def inv(R, e):
    # sag(R, L) = e -> L = 2R sin(acos(1 - e/R))
    return 2 * R * math.sin(math.acos(max(-1.0, 1 - e / R)))


print(f"코드: base_min_distance {base}, distance_ratio {ratio} -> L = {base} + {ratio} v")
print(f"xodr: lane -1 폭 {w} -> 반차선폭 {w / 2}; bbox 반폭 {half_car} -> 여유 {margin:.4f}m, 절반 {margin / 2:.4f}m")
print(f"충돌 s={s_hit}: 차선 중심 반경 R={R_hit[1]:.3f}m (s={R_hit[0]:.2f}, k={R_hit[2]:+.4f}), ±0.5m 안 최소 {R_win:.3f}m")
print(f"D 속도(주행 CSV, s±2m 평균) v={vm:.3f}m/s -> L={Lk:.3f}m")
print(f"예측 이탈(R=충돌 지점) {sag(R_hit[1], Lk):.3f}m")
print(f"역산: 예측 <= 여유 절반({margin / 2:.3f}m) 인 L <= {inv(R_hit[1], margin / 2):.3f}m (R=충돌 지점)")
# 참고: r1391 앞쪽 구간별 반경(현이 걸치는 곳)
for a, b in [(12.0, 14.0), (14.0, 16.0), (16.0, 17.0), (17.0, L)]:
    seg = [r[1] for r in rc if a <= r[0] <= b]
    if seg:
        Rm = min(seg)
        print(f"  참고 s {a:.1f}~{b:.2f} 최소 차선 중심 반경 {Rm:.3f}m: 예측 이탈 {sag(Rm, Lk):.3f}m, 역산 L <= {inv(Rm, margin / 2):.3f}m")
