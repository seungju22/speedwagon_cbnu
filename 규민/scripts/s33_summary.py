#!/usr/bin/env python3
# 맵세션33: 양성재 반복 주행 회차별 필드 집계. 사용: s33_summary.py <test_drive stdout log>...
# 필드: 완주, 궤적(CSV xy 누적), 원값 시간, 정지 근접 대기 발동(예/아니오), 대기 제외 시간(발동 시 -5.0),
#       정차 s(완주 판정 순간 CSV 마지막 행), 정차점까지 거리(로그 "종점까지"), 충돌
import csv
import math
import re
import sys
from pathlib import Path

for p in map(Path, sys.argv[1:]):
    t = p.read_text(errors="replace")
    res = re.search(r"결과: (.*)", t)
    tm = re.search(r"주행 시간\(시뮬\): ([\d.]+)s", t)
    dist = re.search(r"완주: t=[\d.]+s 종점까지 ([\d.]+)m", t)
    near = "[정지 근접 완주]" in t
    col = re.search(r"충돌 누적 (\d+)건", t)
    csvp = re.search(r"궤적 로그: (\S+\.csv)", t)
    L, s_end = None, None
    if csvp:
        rows = list(csv.DictReader(open(csvp.group(1))))
        L = sum(math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"])) for a, b in zip(rows, rows[1:]))
        s_end = rows[-1]["s"]
    T = float(tm.group(1)) if tm else None
    print(f"{p.name}: 완주={res.group(1) if res else '없음'} 궤적={L:.1f}m 시간={T}s "
          f"대기발동={'예' if near else '아니오'} 대기제외={T - 5.0 if near else T:.1f}s "
          f"정차s={s_end} 정차점까지={dist.group(1) if dist else None}m 충돌={col.group(1) if col else None}")
