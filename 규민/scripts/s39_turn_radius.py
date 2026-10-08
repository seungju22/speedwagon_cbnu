#!/usr/bin/env python3
# 맵세션39: 노선별 lane -1 차선 중심 반경이 기준 반경보다 작은 구간 목록(읽기전용, 서버 없음, 판정 없음)
# 반경 정의: s34_curvature.py 와 같다. 차선 중심 반경 Rc = (1 - k t) / |k|, t = laneOffset - w/2. Rc <= 0 = 중심선 뒤집힘
# 기준(세션39 지시문)
#   가 Autoware sample_vehicle: wheel_base 2.79m, max_steer 0.70rad -> 뒤축 중심 최소 반경 2.79/tan(0.70)
#   나 Rosa 공개 회전 지름 12.60m(CarsGuide, SWB 축거 3.995m) -> 반경 6.30m. 바깥 바퀴 기준 값을 그대로 쓴다
#   다 CARLA API 값(참고): s38 자전거 모형 fusorosa 2.049m, a2 0.912m (최대 조향 API 70도)
# 따로 표시: 회차 연결로(road 전체 방위 변화 >= 150도, 이 스크립트의 정의), r1391 끝(s >= 16.0, 세션34 끝 구간 16.5~17.7 포함)
# 사용: python3 s39_turn_radius.py <xodr>
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from s34_curvature import analyze, edge_r, wrap  # noqa: E402

sys.path.insert(0, str(Path(__file__).parents[1] / "tests"))
sys.argv, xodr = sys.argv[:1], sys.argv[1]
import test_drive as td  # noqa: E402

TH = {"가": 2.79 / math.tan(0.70), "나": 12.60 / 2}
REF = {"다_fusorosa": 2.049, "다_a2": 0.912}
UTURN_DEG = 150.0
R1391_END_S = 16.0

root = ET.parse(xodr).getroot()
roads = {int(r.get("id")): r for r in root.iter("road")}
cache = {}


def road_info(rid):
    if rid not in cache:
        _, L, rows, *_ = analyze(roads[rid])
        turn = 0.0
        for a, b in zip(rows, rows[1:]):
            turn += wrap(b[3] - a[3])
        cache[rid] = (L, rows, math.degrees(turn))
    return cache[rid]


def segments(rows, th, s_max):
    """Rc < th 인 연속 구간 [(s0, s1, 최소 Rc, 그 s)]. Rc <= 0 도 포함"""
    out, cur = [], None
    for s, x, y, h, k, off, w, ws, tag in rows:
        if s > s_max + 1e-9:
            break
        rc = edge_r(k, off - w / 2)
        if rc < th:
            if cur is None:
                cur = [s, s, rc, s]
            cur[1] = s
            if rc < cur[2]:
                cur[2], cur[3] = rc, s
        elif cur is not None:
            out.append(tuple(cur))
            cur = None
    if cur is not None:
        out.append(tuple(cur))
    return out


print(f"xodr {Path(xodr).name}")
print(f"기준 가 {TH['가']:.3f}m (2.79/tan 0.70) / 나 {TH['나']:.3f}m (12.60/2, 바깥 바퀴) / 참고 다 fusorosa {REF['다_fusorosa']} a2 {REF['다_a2']}m")
print(f"나 를 뒤축 중심으로 옮기면 sqrt(6.30^2 - 3.995^2) - 윤거/2 = {math.sqrt(6.30**2 - 3.995**2):.3f} - 윤거/2 (윤거 미확인)")
print("열: 노선, road, junction, 표시, 기준, s 구간, 길이 m, 최소 Rc m(그 s), road 방위 변화 도")
routes = {k: v for k, v in td.ROUTES_V2.items() if k in ("north", "south", "middle", "yangseong", "back")}
for name in ("north", "south", "middle", "yangseong", "back"):
    r = routes[name]
    n = {k: 0 for k in list(TH) + list(REF)}
    print(f"== {name} ({len(r['roads'])} road, 마지막 road s<= {r['end_s']})")
    for rid in r["roads"]:
        L, rows, turn = road_info(rid)
        s_max = r["end_s"] if rid == r["roads"][-1] else L
        for key, th in list(TH.items()) + list(REF.items()):
            for s0, s1, rmin, sat in segments(rows, th, s_max):
                tags = []
                if abs(turn) >= UTURN_DEG:
                    tags.append("회차연결로")
                if rid == 1391 and s1 >= R1391_END_S:
                    tags.append("r1391끝")
                n[key] += 1
                rm = "<=0(뒤집힘)" if rmin <= 0 else f"{rmin:.3f}"
                print(f"{name},r{rid},j{roads[rid].get('junction')},{'/'.join(tags) or '-'},{key}<{th:.3f},"
                      f"s {s0:.2f}~{s1:.2f},{s1 - s0:.2f},{rm}(s={sat:.2f}),{turn:+.1f}")
    print(f"   구간 수 {n}")
