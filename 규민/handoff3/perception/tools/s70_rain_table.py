"""10/25 4단계: 비 조건(맑음 r0·비1 r9·비2 r25 mm/h) 거리별 검출률·거짓·문 구역 표와 정답 노드(차 시점) 인자 계산. 호스트 python3
- 클러스터: s69_dist_eval.py json(조합 fb_m<최소 점>_l03_r<비>), s67_door_compare.py json
- 정답 노드 인자(gt_objects2.py --ped-range·--veh-range·--prop-range·--miss-prob):
  범위 = min(맑음 범위, 모델 최대 거리). 모델 최대 거리 = rho/z^2 e^(-0.02 R^0.6 z) = 0.9/(pi zmax^2) 의 해(Goodin 2019 식 8·4, rho 0.07, zmax 100)
  놓침 확률 = 비 단계 클러스터가 '맑음 클러스터가 잡은 틱' 가운데 놓친 비율(보행자·차·콘 합, 새 범위 안 거리만). 맑음 대비 더 놓친 몫만 정답 노드에 줌
사용: python3 s70_rain_table.py <d55.json> <door.json> <min> <out.md> [--b <두 번째 d55.json> --bdoor <두 번째 door.json>]
"""
import argparse, json, math

ap = argparse.ArgumentParser(); ap.add_argument("dist"); ap.add_argument("door"); ap.add_argument("min"); ap.add_argument("out")
ap.add_argument("--b", default=""); ap.add_argument("--bdoor", default="")
a = ap.parse_args()
D = json.load(open(a.dist))["configs"]; R = json.load(open(a.door))["topics"]
RAINS = [("맑음", 0), ("비1", 9), ("비2", 25)]
CLEAR = {"ped": 17.5, "car": 35.0, "cone": 12.5}   # gt_objects2.py 기본(10/14 기준 센서 대조)
DISTS = [5, 10, 13, 15, 20, 25, 30, 40]


def zmax_rain(R, rho=0.07, zmax=100.0):
    pmin = 0.9 / (math.pi * zmax ** 2); lo, hi = 0.5, 300.0
    for _ in range(100):
        m = (lo + hi) / 2
        if rho / m ** 2 * math.exp(-0.02 * R ** 0.6 * m) > pmin: lo = m
        else: hi = m
    return lo


cf = lambda r: f"fb_m{a.min}_l03_r{r}"
L = [f"# 10/25 4단계 비 조건 (54.5만, 건물 점 지움, 최소 점 {a.min}, 복셀 0.3, bag s69d55·s69p55, 시드 0)", "",
     "## 클러스터 거리별 검출률(가운데·가장자리 합침)", "",
     "| 비 | 종류 | " + " | ".join(f"{d}m" for d in DISTS) + " | 90% 거리 m |", "|---|---|" + "---|" * (len(DISTS) + 1)]
for name, r in RAINS:
    v = D[cf(r)]
    for t in ("ped", "car", "cone"):
        L.append(f"| {name} | {t} | " + " | ".join(f"{v['by_bin'][f'{t}/all/{d}']['rate'] * 100:.0f}" for d in DISTS) + f" | {v['dist90'][t]['far90']:g} |")
L += ["", "| 비 | 거짓/틱 | 차 앞 차로 안 거짓/틱 | 차로 위 거짓/틱 | 차 면 오차 평균·절대 95% m | 보행자 위치 오차 평균 m | 문 구역 | 통로 | 문 구역 놓친 s |", "|---|---|---|---|---|---|---|---|---|"]
for name, r in RAINS:
    v = D[cf(r)]; dz = R[cf(r)]["in_door_zone"]; co = R[cf(r)]["in_corridor"]
    L.append(f"| {name} | {v['false_per_tick']:.3f} | {v['false_roi_per_tick']:.4f} | {v['false_lane_per_tick']:.4f} | {v['car_face_err']['mean']:+.2f}·{v['car_face_err']['abs_p95']:.2f} | "
             f"{v['err_by_type']['ped']['mean']:.3f} | {dz['hit_msgs']}/{dz['ped_msgs']} | {co['hit_msgs']}/{co['ped_msgs']} | {dz['miss_s']} |")
# 정답 노드 인자
L += ["", "## 정답 노드(차 시점) 비 인자", "", "| 비 | 모델 최대 거리 m(rho 0.07) | --ped-range | --veh-range | --prop-range | 배율 보행자/차/콘 | 맑음 대비 더 놓친 비율(범위 안) | --miss-prob |", "|---|---|---|---|---|---|---|---|"]
gt = {}
c0 = D[cf(0)]["by_bin"]
for name, r in RAINS:
    zm = zmax_rain(r) if r else float("inf")
    rng = {t: round(min(CLEAR[t], zm), 1) for t in CLEAR}
    lost = base = 0
    if r:
        cr = D[cf(r)]["by_bin"]
        for t in CLEAR:
            for d in DISTS:
                if d > rng[t]: continue
                for side in ("center", "edge"):
                    k = f"{t}/{side}/{d}"
                    h0, h1 = c0[k]["hit"], cr[k]["hit"]
                    base += h0; lost += max(0, h0 - h1)
    mp = round(lost / base, 3) if base else 0.0
    gt[name] = dict(rain_mmh=r, zmax_model=round(zm, 2) if r else None, ped_range=rng["ped"], veh_range=rng["car"], prop_range=rng["cone"], miss_prob=mp, lost=lost, base=base)
    sc = "/".join(f"{rng[t] / CLEAR[t]:.2f}" for t in ("ped", "car", "cone"))
    L.append(f"| {name} | {'-' if not r else f'{zm:.2f}'} | {rng['ped']} | {rng['car']} | {rng['cone']} | {sc} | {lost}/{base} | {mp} |")
if a.b:
    B = json.load(open(a.b))["configs"]; BD = json.load(open(a.bdoor))["topics"]
    same = all({k: v for k, v in D[cf(r)].items() if k != "ptime_ms"} == {k: v for k, v in B[cf(r)].items() if k != "ptime_ms"} for _, r in RAINS)
    dsame = all(R[cf(r)] == BD[cf(r)] for _, r in RAINS)
    L += ["", f"## 같은 시드 두 번: 거리별 json {'같음' if same else '다름'}(처리 시간 칸 뺌), 문 구역 json {'같음' if dsame else '다름'}"]
open(a.out, "w").write("\n".join(L) + "\n")
json.dump(gt, open(a.out.replace(".md", "_gt.json"), "w"), ensure_ascii=False, indent=1)
print("\n".join(L))
