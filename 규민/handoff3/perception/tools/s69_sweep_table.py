"""10/25: 쓸기 평가 결과(s69_dist_eval.py json, s67_door_compare.py json)를 조합별 표 하나로 묶음(마크다운 + json)
- 검출 거리 far90(검출률 90% 이상인 가장 먼 거리), 종류별 검출률, 틱당 거짓(전체·차 앞 차로 안 roi·차로 위), 문 구역·전방 통로 잡은 메시지, 처리 시간 중앙
10/25 3단계 뒤: 거리별 장면에 13m 가 있으면 보행자 칸에 13m 를 넣음(없으면 전과 같음)
사용: s69_sweep_table.py <out 접두어> --dist v1=a.json,v2=b.json --door cld=x.json,clf=y.json [--configs a,b]
"""
import argparse, json

ap = argparse.ArgumentParser(); ap.add_argument("out"); ap.add_argument("--dist", required=True); ap.add_argument("--door", default=""); ap.add_argument("--configs", default="")
a = ap.parse_args()
dist = {k: json.load(open(v))["configs"] for k, v in (x.split("=") for x in a.dist.split(","))}
door = {k: json.load(open(v))["topics"] for k, v in (x.split("=") for x in a.door.split(","))} if a.door else {}
cfs = a.configs.split(",") if a.configs else sorted(next(iter(dist.values())), key=lambda c: (c.split("_")[0], -int(c.split("_")[1][1:]), c))
rows = []
for c in cfs:
    r = {"config": c}
    for s, D in dist.items():
        v = D[c]
        r[s] = {"far90": {t: v["dist90"][t]["far90"] for t in ("ped", "car", "cone")}, "rate": v["rate_by_type"], "false": v["false_per_tick"],
                "roi": v["false_roi_per_tick"], "lane": v["false_lane_per_tick"], "ptime_med": v["ptime_ms"]["med"], "face": v["car_face_err"]["abs_mean"],
                "ped_bins": {d: v["by_bin"][f"ped/all/{d}"]["rate"] for d in (5, 10, 13, 15, 20, 25) if f"ped/all/{d}" in v["by_bin"]}}
    for s, D in door.items():
        z = D[c]
        r["door_" + s] = [z["in_door_zone"]["hit_msgs"], z["in_door_zone"]["ped_msgs"], z["in_corridor"]["hit_msgs"], z["in_corridor"]["ped_msgs"]]
    rows.append(r)
json.dump(rows, open(a.out + ".json", "w"), ensure_ascii=False, indent=1)
S = list(dist); Dk = list(door)
head = ["조합"] + [f"{s} 보행자/차/콘 거리(m)" for s in S] + [f"{s} 보행자 10·(13·)15·20m" for s in S] + [f"{s} 거짓/틱" for s in S] + [f"{s} 차로 안 거짓/틱" for s in S] + \
       [f"문 구역 {k}" for k in Dk] + [f"통로 {k}" for k in Dk] + [f"{S[-1]} 처리 ms"]
L = ["| " + " | ".join(head) + " |", "|" + "---|" * len(head)]
for r in rows:
    f = lambda s: "/".join("%g" % r[s]["far90"][t] for t in ("ped", "car", "cone"))
    pb = lambda s: "·".join("%d%%" % round(100 * (r[s]["ped_bins"][d] or 0)) for d in (10, 13, 15, 20) if d in r[s]["ped_bins"])
    L.append("| " + " | ".join([r["config"]] + [f(s) for s in S] + [pb(s) for s in S] + ["%.3f" % r[s]["false"] for s in S] + ["%.4f" % r[s]["roi"] for s in S] +
                                [f"{r['door_' + k][0]}/{r['door_' + k][1]}" for k in Dk] + [f"{r['door_' + k][2]}/{r['door_' + k][3]}" for k in Dk] +
                                ["%.3f" % r[S[-1]]["ptime_med"]]) + " |")
open(a.out + ".md", "w").write("\n".join(L) + "\n")
print("\n".join(L))
