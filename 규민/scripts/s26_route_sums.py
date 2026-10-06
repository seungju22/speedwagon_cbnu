#!/usr/bin/env python3
# 맵세션26: 발표 숫자 출처 생성(읽기전용). 노선별 경로 합(설계값) = 앞 road 길이 합 + 종점 s,
# 주행 궤적 거리·시간으로 평균 속도. 출력 표준출력 -> map/docs/logs/s26_route_sums.log
import csv
import math
import sys
sys.path.insert(0, "/home/gyumin/campus_mobility_sim/map/scripts")
from s19_scope_common import XODR_V1, road_lengths  # noqa
L = road_lengths(XODR_V1)
LOGS = "/home/gyumin/campus_mobility_sim/map/docs/logs/"
R = {"north": ("drive_plan_dry_north.csv", 78.0, "drive_log_20260928_224708.csv"),
     "south": ("drive_plan_dry_south.csv", 47.0, "drive_log_20260928_224907.csv"),
     "middle": ("drive_plan_dry_middle.csv", 70.0, "drive_log_20261002_012601.csv"),
     "yangseong": ("drive_plan_dry_yangseong.csv", 22.0, "drive_log_20261002_012739.csv")}
for k, (plan, end_s, log) in R.items():
    seq = list(dict.fromkeys(int(r["road_id"]) for r in csv.DictReader(open(LOGS + plan))))
    s = sum(L[r][0] for r in seq[:-1]) + end_s
    pts = [(float(r["t"]), float(r["x"]), float(r["y"]), float(r["speed_mps"])) for r in csv.DictReader(open(LOGS + log))]
    d = sum(math.hypot(pts[i + 1][1] - pts[i][1], pts[i + 1][2] - pts[i][2]) for i in range(len(pts) - 1))
    t = pts[-1][0]
    vmax = max(p[3] for p in pts)
    print(f"{k}: road {len(seq)} 경로합 {s:.1f}m | 궤적 {d:.1f}m 로그끝 t={t:.2f}s 평균 {d / t * 3.6:.1f}km/h "
          f"최고 {vmax * 3.6:.1f}km/h | 궤적-경로합 {d - s:+.1f}m ({(d - s) / s * 100:+.2f}%)")
