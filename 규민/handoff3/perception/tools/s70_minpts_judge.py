"""10/25 1단계: 최소 점 3·4·5(54.5만, 건물 점 지움, 복셀 0.3) 표와 판정. 호스트 python3(숫자만 읽음)
입력: s69_dist_eval.py json(거리별), s67_door_compare.py json(문 구역). 10/25 json 과 같은 조합 숫자가 같은지도 봄
판정(정한 것): 4 가 보행자 13m 100%, 문 구역 모두, 차 앞 차로 안 거짓 0.01개/틱 이하, 전체 거짓 5 의 2배 이하,
  차 가장 가까운 면 오차 |95%| 가 5 보다 0.2m 넘게 안 나빠짐, 콘 13m 90% 이상 -> 기본 4. 아니면 5
사용: python3 s70_minpts_judge.py <d55.json> <door.json> <out.md> [--s69 <s69 d55.json> --s69door <s69 door_p55.json>]
"""
import argparse, json

ap = argparse.ArgumentParser(); ap.add_argument("dist"); ap.add_argument("door"); ap.add_argument("out")
ap.add_argument("--s69", default=""); ap.add_argument("--s69door", default=""); ap.add_argument("--suffix", default="")
a = ap.parse_args()
D = json.load(open(a.dist))["configs"]; R = json.load(open(a.door))["topics"]
CF = {m: f"fb_m{m}_l03{a.suffix}" for m in (3, 4, 5)}


def rate(c, typ, d):
    return D[c]["by_bin"][f"{typ}/all/{d}"]["rate"]


row = {}
for m, c in CF.items():
    v = D[c]; dz = R[c]["in_door_zone"]
    row[m] = dict(ped=[rate(c, "ped", d) for d in (10, 13, 15, 20)], cone=[rate(c, "cone", d) for d in (10, 13, 15)],
                  car=[rate(c, "car", d) for d in (20, 25, 30)], far=v["dist90"], false=v["false_per_tick"], roi=v["false_roi_per_tick"],
                  face95=v["car_face_err"]["abs_p95"], face_mean=v["car_face_err"]["mean"], door=(dz["hit_msgs"], dz["ped_msgs"]),
                  corr=(R[c]["in_corridor"]["hit_msgs"], R[c]["in_corridor"]["ped_msgs"]), ptime=v["ptime_ms"]["med"])
L = ["# 10/25 1단계 최소 점 3·4·5 (54.5만, 건물 점 지움, 복셀 0.3, bag s69d55·s69p55)", "",
     "| 최소 점 | 보행자 10·13·15·20m | 콘 10·13·15m | 차 20·25·30m | 90% 거리 보행자/차/콘 m | 거짓/틱 | 차 앞 차로 안 거짓/틱 | 차 면 오차 평균·절대 95% m | 문 구역 | 통로 | 처리 중앙 ms |",
     "|---|---|---|---|---|---|---|---|---|---|---|"]
pct = lambda xs: "·".join(f"{x * 100:.0f}" for x in xs) + "%"
for m, r in row.items():
    f = r["far"]
    L.append(f"| {m} | {pct(r['ped'])} | {pct(r['cone'])} | {pct(r['car'])} | {f['ped']['far90']:g}/{f['car']['far90']:g}/{f['cone']['far90']:g} | "
             f"{r['false']:.3f} | {r['roi']:.4f} | {r['face_mean']:+.2f}·{r['face95']:.2f} | {r['door'][0]}/{r['door'][1]} | {r['corr'][0]}/{r['corr'][1]} | {r['ptime']:.3f} |")
r4, r5 = row[4], row[5]
chk = [("보행자 13m 100%", r4["ped"][1] >= 1.0, f"{r4['ped'][1] * 100:.1f}%"),
       ("문 구역 모두", r4["door"][0] == r4["door"][1], f"{r4['door'][0]}/{r4['door'][1]}"),
       ("차 앞 차로 안 거짓 0.01 이하", r4["roi"] <= 0.01, f"{r4['roi']:.4f}"),
       ("전체 거짓 5 의 2배 이하", r4["false"] <= 2 * r5["false"], f"{r4['false']:.3f} vs 2 x {r5['false']:.3f} = {2 * r5['false']:.3f}"),
       ("면 오차 |95%| 5 + 0.2 이하", r4["face95"] <= r5["face95"] + 0.2, f"{r4['face95']:.2f} vs {r5['face95']:.2f} + 0.2"),
       ("콘 13m 90% 이상", r4["cone"][1] >= 0.9, f"{r4['cone'][1] * 100:.1f}%")]
ok = all(c[1] for c in chk)
L += ["", "## 판정(최소 점 4)"] + [f"- {n}: {'통과' if g else '못 넘음'} ({s})" for n, g, s in chk]
L.append(f"- 결과: {'기본 4 로 바꿈' if ok else '기본 5 그대로'}")
if a.s69:
    S = json.load(open(a.s69))["configs"]; SD = json.load(open(a.s69door))["topics"] if a.s69door else {}
    L.append("\n## 10/25 같은 조합과 비교(처리 시간 칸 뺌)")
    for m in (3, 5):
        c = CF[m]
        x = {k: v for k, v in D[c].items() if k != "ptime_ms"}; y = {k: v for k, v in S[c.replace(a.suffix, "")].items() if k != "ptime_ms"}
        same = x == y
        dsame = (R[c] == SD.get(c.replace(a.suffix, ""))) if SD else None
        L.append(f"- {c}: 거리별 json {'같음' if same else '다름'}, 문 구역 json {'같음' if dsame else ('다름' if dsame is False else '-')}")
open(a.out, "w").write("\n".join(L) + "\n")
print("\n".join(L))
print("VERDICT", 4 if ok else 5)
