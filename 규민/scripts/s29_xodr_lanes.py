#!/usr/bin/env python3
# 맵세션29: frozen_v1 복사본의 차선 중심 표본을 CSV 로 (lanelet 대응용, 읽기전용). .venv-carla 로 실행(carla 오프라인 Map)
# 열: road, junction, lane(-1 주행, -2 인도), length, 표본 s=시작·중간·끝의 xodr 좌표(x, y = CARLA (x, -y))
# 사용: python s29_xodr_lanes.py <xodr 복사본> <출력 csv>
import csv
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import carla

src, out = Path(sys.argv[1]), Path(sys.argv[2])
text = src.read_text()
cmap = carla.Map(src.stem, text)
root = ET.fromstring(text.encode())
with open(out, "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["road", "junction", "lane", "length", "x0", "y0", "xm", "ym", "x1", "y1"])
    for r in root.iter("road"):
        rid, L = int(r.get("id")), float(r.get("length"))
        for lane in (-1, -2):
            if lane not in {int(x.get("id")) for x in r.iter("lane")}:
                continue
            pts = []
            for s in (0.01, L / 2, L - 0.01):
                wp = cmap.get_waypoint_xodr(rid, lane, max(min(s, L - 0.001), 0.001))
                pts += ["", ""] if wp is None else [f"{wp.transform.location.x:.3f}", f"{-wp.transform.location.y:.3f}"]
            w.writerow([rid, r.get("junction"), lane, f"{L:.3f}"] + pts)
print(f"{out} 작성")
