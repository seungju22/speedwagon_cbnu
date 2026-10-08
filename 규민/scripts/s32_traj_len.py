#!/usr/bin/env python3
# 맵세션32: 주행 CSV xy 누적 궤적 길이(세션20 "CSV xy 누적" 과 같은 정의). 사용: s32_traj_len.py <drive_log csv>...
import csv
import math
import sys
for p in sys.argv[1:]:
    rows = list(csv.DictReader(open(p)))
    L = sum(math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"])) for a, b in zip(rows, rows[1:]))
    print(f"{p.split('/')[-1]}: {len(rows)} 행, 궤적 {L:.1f}m, 마지막 t={rows[-1]['t']}")
