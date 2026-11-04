#!/usr/bin/env python3
# 11/03: 주행 매 틱 기록(s75_track.py csv)으로 조향 명령 대비 실제 바퀴각. 서버 없음
#   명령 rad = CARLA steer(브리지: steer = 명령 rad x steering_curve 첫 값 1.0, 11/03 확인) -> 부호는 CARLA(오른쪽 +)
#   실제: 앞 왼·오른 바퀴각(도). 자전거 모형 등가각 = acot((cot 안쪽 + cot 바깥쪽) / 2)
#   |명령| > --thr 인 틱만 표(0.05rad 칸), 최대 명령·최대 바퀴각, 유턴 두 곳에 필요한 각(atan(축거 / 뒷축 반경))과 견줌
# 사용: python3 s75_steer_eval.py <track.csv> <out.json> [--wb 3.661] [--thr 0.5] [--need 0.729,0.77]
import argparse, csv, json, math
from collections import defaultdict

ap = argparse.ArgumentParser(); ap.add_argument("csv"); ap.add_argument("out"); ap.add_argument("--wb", type=float, default=3.661)
ap.add_argument("--thr", type=float, default=0.5); ap.add_argument("--need", default="0.729,0.77"); a = ap.parse_args()
rows = [r for r in csv.DictReader(open(a.csv)) if r.get("steer") not in (None, "")]


def eq(fl, fr):
    fl, fr = abs(math.radians(fl)), abs(math.radians(fr))
    if fl < 1e-3 or fr < 1e-3: return (fl + fr) / 2
    return math.atan(2 / (1 / math.tan(fl) + 1 / math.tan(fr)))


bins = defaultdict(list); allr = []
for r in rows:
    st = float(r["steer"]); fl, fr = float(r["fl_deg"]), float(r["fr_deg"]); v = float(r["v"])
    allr.append((abs(st), abs(fl), abs(fr), eq(fl, fr), v))
    if abs(st) > a.thr: bins[round(abs(st) / 0.05) * 0.05].append((abs(fl), abs(fr), eq(fl, fr), v, st))
tab = []
for k in sorted(bins):
    xs = bins[k]; n = len(xs)
    tab.append(dict(cmd_rad=round(k, 2), n=n, fl_deg=round(sum(x[0] for x in xs) / n, 1), fr_deg=round(sum(x[1] for x in xs) / n, 1),
                    eq_rad=round(sum(x[2] for x in xs) / n, 3), left_turn_frac=round(sum(1 for x in xs if x[4] < 0) / n, 2), v_mean=round(sum(x[3] for x in xs) / n, 2)))
need = [float(x) for x in a.need.split(",")]
res = dict(csv=a.csv, ticks=len(rows), over_thr=sum(len(v) for v in bins.values()), max_cmd_rad=round(max(x[0] for x in allr), 3) if allr else None,
           max_wheel_deg=round(max(max(x[1], x[2]) for x in allr), 1) if allr else None, max_eq_rad=round(max(x[3] for x in allr), 3) if allr else None,
           table=tab, need_rad=need, note="CARLA steer 부호: 오른쪽 +. 왼 회전이면 왼 바퀴가 안쪽(더 큰 각)")
json.dump(res, open(a.out, "w"), ensure_ascii=False, indent=1)
print(json.dumps({k: v for k, v in res.items() if k != "table"}, ensure_ascii=False))
for t in tab: print(t)
