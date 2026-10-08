#!/usr/bin/env python3
# 맵세션36 참고(적용 아님): r1391 계획점 간격을 바꾸면 같은 모형에서 이탈이 어떻게 되나(읽기전용)
# r1391 계획점만 차선 중심(xodr 직접 계산)으로 다시 깐다. 다른 road 계획점은 서버 계획 그대로
# 먼저 서버 계획점(get_waypoint_xodr)과 이 계산의 차선 중심 좌표 차를 찍어 근사가 맞는지 본다
# 사용: s36_spacing_model.py <xodr> <drive_plan csv> <v> <간격...>   (b 는 3.0 과 1.0 두 경우)
import csv
import math
import sys
sys.argv_saved = list(sys.argv)
xodr, planp, v = sys.argv[1], sys.argv[2], sys.argv[3]
spacings = [float(a) for a in sys.argv[4:]]
sys.argv = [sys.argv[0], xodr, planp, v, "0.3904"]
src = open(__file__.rsplit("/", 1)[0] + "/s36_invert_aim.py").read().split("if len(sys.argv) > 5:")[0]
exec(src)
orig = list(plan)
for p in orig:
    if p[0] == 1391:
        cx, cy = center(p[1])
        print(f"  계획점 r1391 s={p[1]}: 서버 ({p[2]:.3f},{p[3]:.3f}) vs 계산 ({cx:.3f},{cy:.3f}) 차 {math.hypot(p[2]-cx, p[3]-cy):.3f}m")
first = next(i for i, p in enumerate(orig) if p[0] == 1391)
last = max(i for i, p in enumerate(orig) if p[0] == 1391)
for sp in spacings:
    pts = []
    s = orig[first][1]
    while s < L391 - 0.3:
        x, y = center(s)
        pts.append((1391, round(s, 3), x, y))
        s += sp
    plan[:] = orig[:first] + pts + orig[last + 1:]
    for b in (3.0, 1.0):
        Lk, (d, at) = worst(b)
        print(f"간격 {sp}m (r1391 점 {len(pts)}개, 마지막 s={pts[-1][1]}) b={b}: L={Lk:.3f} 최대 이탈 {d:.3f}m {at}")
