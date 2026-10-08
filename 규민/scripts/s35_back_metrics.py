#!/usr/bin/env python3
# 맵세션35: 후문 주행 1회의 r1391 지표(읽기전용). 사용: s35_back_metrics.py <xodr> <test_drive stdout log>
# 완주, 충돌(road, s: 충돌 CSV 위치를 주행 CSV 최근접 행으로), r1391 통과(r1391 뒤 r1952 행 존재),
# r1391 구간 차선 중심(t=-1.675, xodr width 3.35 의 절반) 대비 횡 이탈 최대와 그 지점 조향
import csv
import math
import re
import sys
import xml.etree.ElementTree as ET
sys.path.insert(0, __file__.rsplit("/", 1)[0])
from s34_curvature import analyze  # noqa: E402

xodr, logp = sys.argv[1], sys.argv[2]
t = open(logp, errors="replace").read()
res = re.search(r"결과: (.*)", t)
csvp = re.search(r"궤적 로그: (\S+\.csv)", t).group(1)
colp = re.search(r"충돌 센서 부착: .* -> (\S+\.csv)", t)
rows = list(csv.DictReader(open(csvp)))
root = ET.parse(xodr).getroot()
road = [r for r in root.iter("road") if r.get("id") == "1391"][0]
_, L, smp, _, _, _ = analyze(road)
w = float(road.find("lanes").find("laneSection").find("right").find("lane").find("width").get("a"))
center = -w / 2
best = None
seen1391 = False
passed = False
for r in rows:
    if r["road_id"] == "1391":
        seen1391 = True
        x, y = float(r["x"]), -float(r["y"])
        b = min(smp, key=lambda q: (q[1] - x) ** 2 + (q[2] - y) ** 2)
        tl = -(x - b[1]) * math.sin(b[3]) + (y - b[2]) * math.cos(b[3])
        dev = tl - center
        if best is None or abs(dev) > abs(best[0]):
            best = (dev, tl, b[0], float(r["steer"]), r["t"])
    elif seen1391 and r["road_id"] == "1952":
        passed = True
cols = list(csv.DictReader(open(colp.group(1)))) if colp else []
cinfo = []
for c in cols:
    cx, cy = float(c["loc_x"]), float(c["loc_y"])
    n = min(rows, key=lambda r: (float(r["x"]) - cx) ** 2 + (float(r["y"]) - cy) ** 2)
    cinfo.append(f"road{n['road_id']} s={n['s']} ({c['other_actor_type_id']})")
traj = sum(math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"])) for a, b in zip(rows, rows[1:]))
tm = re.search(r"주행 시간\(시뮬\): ([\d.]+)s", t)
print(f"{logp.split('/')[-1]}: 결과={res.group(1) if res else None} 시간={tm.group(1) if tm else None}s 궤적={traj:.1f}m "
      f"충돌={len(cols)} {cinfo} r1391통과={'예' if passed else '아니오'}")
if best:
    side = "왼쪽(t=0 가장자리 쪽)" if best[0] > 0 else "오른쪽(인도 쪽)"
    print(f"  r1391 차선 중심 대비 횡 이탈 최대 {best[0]:+.2f}m {side} (t_lat={best[1]:+.2f}, s={best[2]:.2f}, t={best[4]}) "
          f"조향 {best[3]:+.3f}, 차 왼쪽 끝 t={best[1] + 0.8943:+.2f}")
