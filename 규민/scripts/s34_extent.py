#!/usr/bin/env python3
# 맵세션34: 안쪽 가장자리 반경 <= 기준 인 구간의 s 범위·길이(읽기전용). 사용: s34_extent.py <xodr> <임계 m> <road id...>
# CARLA 메시는 vertex_distance(이 맵 로드 2.0m) 간격으로 만든다. 짧은 구간은 메시에 안 나타날 수 있다(코드 미확인, 추정)
import sys
import xml.etree.ElementTree as ET
sys.path.insert(0, __file__.rsplit("/", 1)[0])
from s34_curvature import analyze, edge_r  # noqa: E402
root = ET.parse(sys.argv[1]).getroot()
roads = {int(r.get("id")): r for r in root.iter("road")}
th = float(sys.argv[2])
for rid in map(int, sys.argv[3:]):
    _, L, rows, _, _, _ = analyze(roads[rid])
    segs, cur = [], None
    for s, x, y, h, k, off, w, ws, tag in rows:
        bad = abs(k) > 1e-9 and min(edge_r(k, off), edge_r(k, off - w)) <= th
        if bad and cur is None:
            cur = [s, s]
        elif bad:
            cur[1] = s
        elif cur is not None:
            segs.append(cur); cur = None
    if cur:
        segs.append(cur)
    tot = sum(b - a for a, b in segs)
    print(f"r{rid} 길이 {L:.2f} junction={roads[rid].get('junction')} 안쪽<= {th}m 구간 {len(segs)}개 합 {tot:.2f}m: "
          f"{[(round(a, 2), round(b, 2)) for a, b in segs][:8]}")
