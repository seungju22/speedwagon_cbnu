#!/usr/bin/env python3
# 맵세션38: 후문 회차별 r1391 최대 이탈(소수 3자리, s35_back_metrics.py 와 같은 정의)·속도·조향·목표점 전환, 예측 0.601m 대조(읽기전용)
# 사용: s38_back_summary.py <xodr> <test_drive stdout log...>
import csv
import io
import re
import statistics
import sys
from contextlib import redirect_stdout
sys.path.insert(0, __file__.rsplit("/", 1)[0])
PRED = 0.601
xodr = sys.argv[1]
devs = []
for logp in sys.argv[2:]:
    sys.argv = ["m", xodr, logp]
    g = {"__name__": "__main__", "__file__": __file__.rsplit("/", 1)[0] + "/s35_back_metrics.py"}
    buf = io.StringIO()
    with redirect_stdout(buf):
        exec(open(g["__file__"]).read(), g)
    d = g["best"][0]
    devs.append(d)
    t = open(logp, errors="replace").read()
    rows = [r for r in csv.DictReader(open(re.search(r"궤적 로그: (\S+\.csv)", t).group(1))) if r["road_id"] == "1391"]
    st = [float(r["steer"]) for r in rows]
    sp = [float(r["speed_mps"]) * 3.6 for r in rows]
    tl = list(csv.DictReader(open(re.search(r"목표점 로그: (\S+\.csv)", t).group(1))))
    sw = next((r for r in tl if r["road_id"] == "1391" and r["target_road"] == "1952"), None)
    print(f"{logp.rsplit('/', 1)[-1]}: 최대 이탈 {d:+.3f}m (s={g['best'][2]:.2f}) 예측 대비 {d - PRED:+.3f}m, "
          f"r1391 {len(rows)}tick 속도 {min(sp):.2f}~{max(sp):.2f}km/h 조향 {min(st):+.3f}~{max(st):+.3f}, "
          f"r1952 전환 차 s={sw['s'] if sw else '-'} t={sw['t'] if sw else '-'}")
print(f"n={len(devs)} 평균 {statistics.mean(devs):.3f} 최소 {min(devs):.3f} 최대 {max(devs):.3f} m, 예측 {PRED}m, "
      f"차(평균-예측) {statistics.mean(devs) - PRED:+.3f}m, 범위 {max(devs) - min(devs):.3f}m")
