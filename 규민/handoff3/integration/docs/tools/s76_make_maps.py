"""11/04 1단계: s76_pedal_bench.py 매 틱 기록 -> 브리지 accel_map.csv·brake_map.csv 와 같은 꼴의 지도(서버 없이)
가속도 = 같은 단계 안 앞뒤 3틱(0.15s) 속도 차 / 0.3s + g sin(pitch)(경사 보정, 이 길은 높이 0 이라 거의 0). 옮긴 뒤 10틱 안 것·멈춘 틱(v<0.05)은 뺌
칸 값 = 그 속도 열 ±0.4m/s 안 표본의 중앙값(v=0 열은 0.05~0.5m/s). 표본이 없으면(그 스로틀로 그 속도까지 못 감) 비워 두고 아래 규칙으로 채움
  가속 지도: 행 0 = 타력(brk0.00 단계). 빈 칸은 이웃 아래 행(스로틀 낮은 행) 값 + 측정된 행 간 차이의 마지막 값으로 채우지 않고,
             '그 스로틀로 못 가는 속도' 이므로 그 행의 마지막 잰 값과 타력 행 사이를 속도 따라 직선으로 내려 타력 행 값에 닿게 함(스로틀이 그 속도를 못 버티는 꼴)
  브레이크 지도: 행 0 = 타력과 같은 값(프리우스 지도와 같은 꼴). 빈 칸(높은 속도에서 못 잼은 없음, 시작 14.2m/s)은 같은 행 가장 가까운 잰 값
  단조: 가속 지도는 스로틀 행이 오를수록 커지게(아래 행 + 0.001 이상), 브레이크 지도는 브레이크 행이 오를수록 작아지게(위 행 - 0.001 이하) 고침. 고친 칸 수를 적음
사용: python s76_make_maps.py <ticks.csv> <출력 접두어> [--ref-accel 프리우스 accel_map.csv --ref-brake brake_map.csv](견줌)
"""
import argparse, csv, json, math
from collections import defaultdict
import numpy as np

ap = argparse.ArgumentParser(); ap.add_argument("ticks"); ap.add_argument("out")
ap.add_argument("--ref-accel"); ap.add_argument("--ref-brake"); ap.add_argument("--thr-rows"); ap.add_argument("--brake-low"); ap.add_argument("--brk-rows")
a = ap.parse_args()
VC = [0, 1.39, 2.78, 4.17, 5.56, 6.94, 8.33, 9.72, 11.11, 12.5, 13.89]
R = list(csv.DictReader(open(a.ticks)))
by = defaultdict(list)
for r in R: by[r["ph"]].append(r)
# 11/04: 칸 값 = 그 속도 칸(이웃 열과의 가운데까지) 안에서 보낸 틱 쌍의 sum(dv)/sum(dt) = 칸 안 시간 평균 가속도
#   (첫 판 '앞뒤 3틱 차분 중앙값' 은 단 내림 순간 튐·표본 적은 칸에 약했음). 옮긴 뒤 20틱 안, 멈춘 쌍(둘 다 v<0.02)은 뺌. 칸 안 시간 0.15s(3쌍) 못 되면 빈 칸
EDGE = [0.0] + [(VC[i] + VC[i + 1]) / 2 for i in range(len(VC) - 1)] + [VC[-1] + 0.7]
pairs = defaultdict(lambda: [[0.0, 0.0, 0] for _ in VC])  # (kind, pedal) -> 열마다 [sum dv(+경사), sum dt, 쌍 수]
for ph, rs in by.items():
    if not (ph.startswith("thr") or ph.startswith("brk")): continue
    kind, ped = ph[:3], float(ph[3:].split("@")[0])
    for r0, r1 in zip(rs, rs[1:]):
        if int(r1["tp"]) != int(r0["tp"]) + 1 or int(r0["tp"]) < 20: continue
        v0, v1 = float(r0["v"]), float(r1["v"])
        if v0 < 0.02 and v1 < 0.02: continue
        vm = (v0 + v1) / 2; dt = float(r1["t"]) - float(r0["t"])
        j = next((i for i in range(len(VC)) if EDGE[i] <= vm < EDGE[i + 1]), None)
        if j is None: continue
        c = pairs[(kind, ped)][j]; c[0] += (v1 - v0) + 9.81 * math.sin(math.radians(float(r0["pitch"]))) * dt; c[1] += dt; c[2] += 1
meas = {k: [((c[0] / c[1]) if c[1] >= 0.15 else None, round(c[1], 2)) for c in cols] for k, cols in pairs.items()}
# 11/04: 저속 제동 칸(--brake-low "3.0:0,1,2;4.4:3,4" = 시작 속도 VF 회차의 정지 평균 감속 -v0^2/(2d) 를 그 열 번호들에)
#   까닭: CARLA 는 저속에서 브레이크가 매우 세서 칸 평균이 0.1~0.3초 표본으로 -10~-20 처럼 튐. 정지까지 평균이 정차에 쓰는 값에 가까움
#   5.2m/s 시작 회차는 1단 최고 회전 근처에서 브레이크 값과 상관없이 0.5초 안에 섬(이상 상태) -> 안 씀
stopavg = {}
for ph, rs in by.items():
    if not ph.startswith("brk") or "@" not in ph: continue
    ped, vf = float(ph[3:].split("@")[0]), float(ph.split("@")[1])
    if ped == 0: continue
    v0 = float(rs[0]["v"]); d = 0.0
    for r0, r1 in zip(rs, rs[1:]):
        if float(r0["v"]) < 0.05: break
        if int(r1["tp"]) == int(r0["tp"]) + 1: d += math.hypot(float(r1["x"]) - float(r0["x"]), float(r1["y"]) - float(r0["y"]))
    stopavg[(ped, vf)] = (-v0 * v0 / (2 * max(d, 0.01)), round(v0, 2), round(d, 2))
if a.brake_low:
    for part in a.brake_low.split(";"):
        vf, cols = part.split(":"); vf = float(vf)
        for (ped, f), (val, v0, d) in stopavg.items():
            if f != vf: continue
            for j in [int(x) for x in cols.split(",")]:
                meas[("brk", ped)][j] = (val, f"stop@{vf}")
coast = [x[0] for x in meas[("brk", 0.0)]]
# 타력 빈 칸(아주 낮은 속도)은 가까운 값
for j in range(len(VC)):
    if coast[j] is None: coast[j] = next(coast[k] for k in sorted(range(len(VC)), key=lambda k: abs(k - j)) if coast[k] is not None)
thr_rows = sorted(p for k, p in meas if k == "thr"); brk_rows = sorted(p for k, p in meas if k == "brk" and p > 0)
if a.thr_rows: thr_rows = [float(x) for x in a.thr_rows.split(",")]
if a.brk_rows: brk_rows = [float(x) for x in a.brk_rows.split(",")]
filled = defaultdict(int)
acc = [coast[:]]
for p in thr_rows:
    row = [x[0] for x in meas[("thr", p)]]
    known = [j for j, x in enumerate(row) if x is not None]
    last = max(known)
    for j in range(len(VC)):
        if row[j] is None:
            if j > last:  # 못 간 속도: 마지막 잰 값에서 1 열 뒤 타력 값으로 직선
                row[j] = coast[j] if j > last + 1 else (row[last] + coast[j]) / 2
            else:  # 사이 빈 칸: 직선 보간
                lo = max(k for k in known if k < j); hi = min(k for k in known if k > j)
                row[j] = row[lo] + (row[hi] - row[lo]) * (j - lo) / (hi - lo)
            filled["accel_empty"] += 1
    acc.append(row)
for i in range(1, len(acc)):
    for j in range(len(VC)):
        if acc[i][j] < acc[i - 1][j] + 0.001: acc[i][j] = acc[i - 1][j] + 0.001; filled["accel_mono"] += 1
brk = [coast[:]]
for p in brk_rows:
    row = [x[0] for x in meas[("brk", p)]]
    known = [j for j, x in enumerate(row) if x is not None]
    for j in range(len(VC)):
        if row[j] is None: row[j] = row[min(known, key=lambda k: abs(k - j))]; filled["brake_empty"] += 1
    brk.append(row)
for i in range(1, len(brk)):
    for j in range(len(VC)):
        if brk[i][j] > brk[i - 1][j] - 0.001: brk[i][j] = brk[i - 1][j] - 0.001; filled["brake_mono"] += 1


def write(path, rows_p, tab):
    with open(path, "w") as f:
        f.write("default," + ",".join(f"{v:g}" for v in VC) + "\n")
        for p, row in zip(rows_p, tab): f.write(f"{p:.3f}," + ",".join(f"{x:.3f}" for x in row) + "\n")


write(a.out + "_accel_map.csv", [0.0] + thr_rows, acc); write(a.out + "_brake_map.csv", [0.0] + brk_rows, brk)
res = dict(stopavg={f"brk{p:.2f}@{f}": v for (p, f), v in sorted(stopavg.items())}, cols=VC, accel_rows=[0.0] + thr_rows, brake_rows=[0.0] + brk_rows, filled=dict(filled),
           counts={f"{k}{p:.2f}": [x[1] for x in meas[(k, p)]] for k, p in sorted(meas)},
           raw={f"{k}{p:.2f}": [None if x[0] is None else round(x[0], 3) for x in meas[(k, p)]] for k, p in sorted(meas)})


def readmap(p):
    rows = list(csv.reader(open(p))); return {float(r[0]): [float(x) for x in r[1:]] for r in rows[1:]}


if a.ref_accel:
    ra, rb = readmap(a.ref_accel), readmap(a.ref_brake); cmp = {}
    for k, p in sorted(meas):
        ref = (ra if k == "thr" else rb).get(p)
        if ref is None: continue
        d = [round(x[0] - r, 3) for x, r in zip(meas[(k, p)], ref) if x[0] is not None]
        cmp[f"{k}{p:.2f}"] = dict(diff=d, mean_abs=round(float(np.mean(np.abs(d))), 3) if d else None)
    res["vs_ref"] = cmp
open(a.out + "_maps.json", "w").write(json.dumps(res, ensure_ascii=False, indent=1))
print(json.dumps(dict(filled=res["filled"], vs_ref={k: v["mean_abs"] for k, v in res.get("vs_ref", {}).items()})))
