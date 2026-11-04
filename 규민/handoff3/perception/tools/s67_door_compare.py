"""10/23: 주행 중 객체 토픽(s67_objects_dump.py _objects.csv)으로 문 구역·전방 통로 보행자 검출을 셈. 같은 회차 정답(차 시점, /perception/gt/objects)도 같은 방식으로 셈
- 시각: stamp = T0 + (frame - F0) x 0.05 (gtobj 'pub 1 frame F0 t T0'). 장면 csv(scn_<tag>.csv)의 in_door_zone·in_corridor 를 씀
- 잡음 = 그 토픽 메시지 시각의 보행자 중심에서 --r m 안에 객체 중심이 있음(s60_cluster_compare.py 와 같은 판정, 클러스터 점 대신 객체 중심)
- 놓친 시간 = 놓친 메시지 수 x 그 토픽 메시지 간격 중앙값
- 놓친 메시지마다 가장 가까운 객체가 정류장 시설 다각형(+1m) 안이면 '시설과 붙음'으로 셈
사용: s67_door_compare.py <_objects.csv> <scn csv> <F0> <T0> <out.json> [--r 1.0] --topics t1,t2,...
"""
import argparse, bisect, collections, csv, json, math

ap = argparse.ArgumentParser()
ap.add_argument("objects"); ap.add_argument("scn"); ap.add_argument("f0", type=int); ap.add_argument("t0", type=float); ap.add_argument("out")
ap.add_argument("--r", type=float, default=1.0); ap.add_argument("--topics", required=True)
a = ap.parse_args()
FAC = [(656.19, 679.08), (653.26, 681.17), (654.35, 682.71), (657.28, 680.62)]   # shuttle_platforms_v31_r1.yaml lib_north facility


def grow(poly, d):
    cx = sum(p[0] for p in poly) / len(poly); cy = sum(p[1] for p in poly) / len(poly)
    return [(cx + (x - cx) * (1 + d / math.hypot(x - cx, y - cy)), cy + (y - cy) * (1 + d / math.hypot(x - cx, y - cy))) for x, y in poly]


def in_poly(px, py, poly):
    ins = False; n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]; x2, y2 = poly[(i + 1) % n]
        if (y1 > py) != (y2 > py) and px < x1 + (py - y1) * (x2 - x1) / (y2 - y1): ins = not ins
    return ins


FACG = grow(FAC, 1.0)
by_t = collections.defaultdict(list)
for g in csv.DictReader(open(a.scn)):
    by_t[round(a.t0 + (int(g["frame"]) - a.f0) * 0.05, 2)].append(g)
bk = sorted(by_t)


def gts(s):
    i = bisect.bisect_left(bk, s); c = [k for k in bk[max(0, i - 1):i + 1] if abs(k - s) <= 0.03]
    return by_t[c[0]] if c else None


ticks = collections.defaultdict(dict)   # topic -> stamp -> [(x,y)]
for r in csv.DictReader(open(a.objects)):
    ticks[r["topic"]].setdefault(float(r["stamp"]), []).append((float(r["x"]), float(r["y"])))
tk_csv = a.objects.replace("_objects.csv", "_ticks.csv")
for r in csv.DictReader(open(tk_csv)):
    ticks[r["topic"]].setdefault(float(r["stamp"]), [])
out = {"radius_m": a.r, "topics": {}}
for t in a.topics.split(","):
    st = sorted(ticks[t]); dts = sorted(b - c for b, c in zip(st[1:], st)); dt = dts[len(dts) // 2] if dts else 0
    res = {"msg_dt_s": round(dt, 3)}
    for zone in ("in_door_zone", "in_corridor"):
        n = hit = fac = 0; per = {}; errs = []
        for s in st:
            gs = gts(s)
            if not gs: continue
            for g in gs:
                if g[zone] != "1": continue
                gx, gy = float(g["x"]), float(g["y"])
                dd = sorted((math.hypot(x - gx, y - gy), x, y) for x, y in ticks[t][s])
                ok = bool(dd) and dd[0][0] <= a.r
                n += 1; hit += ok
                if ok: errs.append(dd[0][0])
                elif dd and in_poly(dd[0][1], dd[0][2], FACG): fac += 1
                p = per.setdefault(g["id"], {"msgs": 0, "hit": 0, "first_in": s, "first_hit": None, "miss_at": []})
                p["msgs"] += 1; p["hit"] += ok
                if ok and p["first_hit"] is None: p["first_hit"] = s
                if not ok: p["miss_at"].append([round(s, 2), round(dd[0][0], 2) if dd else None])
        for p in per.values():
            p["late_s"] = round(p["first_hit"] - p["first_in"], 2) if p["first_hit"] is not None else None
            p["first_in"] = round(p["first_in"], 2); p["first_hit"] = round(p["first_hit"], 2) if p["first_hit"] is not None else None
            p["miss_at"] = p["miss_at"][:20]
        errs.sort()
        res[zone] = {"ped_msgs": n, "hit_msgs": hit, "hit_rate": round(hit / n, 3) if n else None, "miss_s": round((n - hit) * dt, 2),
                     "miss_nearest_in_facility": fac, "center_err_med_m": round(errs[len(errs) // 2], 2) if errs else None, "per_ped": per}
    out["topics"][t] = res
json.dump(out, open(a.out, "w"), ensure_ascii=False, indent=1)
for t, v in out["topics"].items():
    print(t, {z: {k: v[z][k] for k in ("ped_msgs", "hit_msgs", "hit_rate", "miss_s", "miss_nearest_in_facility")} for z in ("in_door_zone", "in_corridor")})
