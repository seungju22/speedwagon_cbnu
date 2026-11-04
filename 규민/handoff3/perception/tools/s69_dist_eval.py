"""10/25: 거리별 평가를 쓸기 조합마다 셈(s68_dist_eval.py 와 같은 규칙). 호스트 .venv-carla(CARLA 서버 없음)
- 검출: s69_sweep_dump.py 의 <접두어>_objects.csv(조합별), 정답: s67_objects_dump.py 가 푼 원래 bag 의 /perception/gt/objects(--gtcsv)
- 짝짓기·구간·거짓 객체 자리 가르기·위치 오차는 s68_dist_eval.py 와 같음(그 파일 주석 참고)
  - 차 앞 차로 안 거짓(roi) = 자차 기준 앞 0~45m, 옆 5m 안, 표적 근처(정답 4m 안) 뺀 것. 10/23 'false_roi_per_tick' 과 같은 정의
  - 차로 위 거짓(lane) = roi 가운데 자리가 '기타:차로 위' 인 것
- 차 '가장 가까운 면' 오차(10/25 새로): 정답 상자 네 꼭짓점을 자차 기준으로 옮겨 가장 작은 앞 거리(x) vs 짝지은 검출의 클러스터 점 가운데 가장 작은 x
  (자차 진행 방향 기준, 검출 - 정답. 음수면 검출이 더 가깝게 봄)
- 검출 거리 = 그 종류 검출률(가운데·가장자리 합침) 90% 이상인 가장 먼 거리(far90), 5m 부터 끊김 없이 90% 이상인 가장 먼 거리(cont90)
사용: s69_dist_eval.py <tag> <v1|v2> <쓸기 접두어> <out.json> [--configs a,b] [--gtcsv] [--scn] [--gtlog] [--gtobjlog]
"""
import argparse, bisect, collections, csv, json, math
from pathlib import Path
import numpy as np
import carla

C = Path.home() / "campus_mobility_sim"; L = C / "integration/logs"
ap = argparse.ArgumentParser(); ap.add_argument("tag"); ap.add_argument("set", choices=["v1", "v2"]); ap.add_argument("sweep"); ap.add_argument("out")
ap.add_argument("--configs", default=""); ap.add_argument("--near", type=float, default=4.0)
ap.add_argument("--gtcsv", default=""); ap.add_argument("--scn", default=""); ap.add_argument("--gtlog", default=""); ap.add_argument("--gtobjlog", default="")
ap.add_argument("--actors", default=str(C / "map/docs/logs/campus_layers_dry_s66_all_actors.csv"))
ap.add_argument("--trees", default=str(C / "map/data/trees_layout_s66.csv"))
ap.add_argument("--xodr", default=str(C / "map/maps/cbnu_campus_frozen_v31.xodr"))
a = ap.parse_args()
GTCSV = a.gtcsv or str(L / f"s68/step4/{a.tag}_objects.csv"); SCN = a.scn or str(L / f"s68/scn_{a.tag}.csv")
GTLOG = a.gtlog or str(L / f"i5_gt_{a.tag}.log"); GTOBJ = a.gtobjlog or str(L / f"s68/gtobj_{a.tag}_all.log")
GTT = "/perception/gt/objects"; R_MATCH = {7: 1.0, 0: 1.0, 1: 2.0}
FAC = [(656.19, 679.08), (653.26, 681.17), (654.35, 682.71), (657.28, 680.62)]
BUSSTOP = (3.598, 1.884)

ego = next(l.split() for l in open(GTLOG) if l.startswith("pub 1 "))
bx, by, byaw = float(ego[ego.index("base") + 1]), float(ego[ego.index("base") + 2]), math.radians(float(ego[ego.index("yaw") + 1]))
cy_, sy_ = math.cos(byaw), math.sin(byaw)
g0 = next(l.split() for l in open(GTOBJ) if l.startswith("pub 1 "))
F0, T0 = int(g0[3]), float(g0[5])
st = lambda fr: round(T0 + (fr - F0) * 0.05, 2)


def to_ego(x, y):
    dx, dy = x - bx, y - by
    return dx * cy_ + dy * sy_, -dx * sy_ + dy * cy_


def in_box(px, py, cx, cy, yaw, lx, ly, pad):
    c, s = math.cos(yaw), math.sin(yaw); dx, dy = px - cx, py - cy
    return abs(dx * c + dy * s) <= lx / 2 + pad and abs(-dx * s + dy * c) <= ly / 2 + pad


def in_poly(px, py, poly):
    ins = False; n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]; x2, y2 = poly[(i + 1) % n]
        if (y1 > py) != (y2 > py) and px < x1 + (py - y1) * (x2 - x1) / (y2 - y1): ins = not ins
    return ins


def grow(poly, d):
    cx = sum(p[0] for p in poly) / len(poly); cy = sum(p[1] for p in poly) / len(poly)
    return [(cx + (x - cx) * (1 + d / math.hypot(x - cx, y - cy)), cy + (y - cy) * (1 + d / math.hypot(x - cx, y - cy))) for x, y in poly]


FACG = grow(FAC, 1.0); bus, bld = [], []
LAYERS = ("buildings",) if a.set == "v1" else ("buildings", "buildings2")
for r in csv.DictReader(open(a.actors)):
    x, y, yaw = float(r["x"]), -float(r["y"]), -math.radians(float(r["yaw"]))
    if r["layer"] == "busstop": bus.append((x, y, yaw))
    elif r["layer"] in LAYERS: bld.append((x, y, yaw, float(r["scale"])))
trees = [] if a.set == "v1" else [(float(r["carla_x"]), -float(r["carla_y"]), float(r["crown_r_m"])) for r in csv.DictReader(open(a.trees))]
cmap = carla.Map("OpenDriveMap", open(a.xodr).read()); LT = carla.LaneType
_pc = {}


def place(x, y):
    key = (round(x * 4) / 4, round(y * 4) / 4)   # 0.25m 칸으로 캐시(자리 이름만 쓰므로)
    if key in _pc: return _pc[key]
    x, y = key
    if any(in_box(x, y, qx, qy, qyaw, BUSSTOP[0], BUSSTOP[1], 1.0) for qx, qy, qyaw in bus) or in_poly(x, y, FACG): r = "정류장 시설"
    elif any(math.hypot(x - tx, y - ty) <= tr for tx, ty, tr in trees): r = "나무"
    elif any(in_box(x, y, qx, qy, qyaw, sc, sc, 1.0) for qx, qy, qyaw, sc in bld if abs(x - qx) < sc + 3 and abs(y - qy) < sc + 3): r = "건물"
    else:
        loc = carla.Location(x, -y, 0.0)
        if cmap.get_waypoint(loc, project_to_road=False, lane_type=LT.Driving) is not None: r = "기타:차로 위"
        elif cmap.get_waypoint(loc, project_to_road=False, lane_type=LT.Sidewalk) is not None: r = "기타:보도"
        else:
            w = cmap.get_waypoint(loc, project_to_road=True, lane_type=LT.Driving)
            r = "연석" if w is not None and math.hypot(w.transform.location.x - loc.x, w.transform.location.y - loc.y) - w.lane_width / 2 <= 2.0 else "기타:그 밖"
    _pc[key] = r; return r


gt = collections.defaultdict(list)
for r in csv.DictReader(open(GTCSV)):
    if r["topic"] == GTT:
        gt[round(float(r["stamp"]), 2)].append((float(r["x"]), float(r["y"]), int(r["label"]), float(r["yaw"]), float(r["dx"]), float(r["dy"])))
gk = sorted(gt)
det = collections.defaultdict(lambda: collections.defaultdict(list)); ticks = collections.defaultdict(set)
for r in csv.DictReader(open(a.sweep + "_ticks.csv")): ticks[r["topic"]].add(round(float(r["stamp"]), 2))
for r in csv.DictReader(open(a.sweep + "_objects.csv")):
    det[r["topic"]][round(float(r["stamp"]), 2)].append((float(r["x"]), float(r["y"]), int(r["n_pts"]), float(r["minx_base"]) if r["minx_base"] else None))
ptime = collections.defaultdict(list)
for r in csv.DictReader(open(a.sweep + "_ptime.csv")): ptime[r["topic"]].append(float(r["ms"]))
scn = list(csv.DictReader(open(SCN)))
configs = a.configs.split(",") if a.configs else sorted(ticks)


def gt_at(s):
    i = bisect.bisect_left(gk, s); c = [k for k in gk[max(0, i - 1):i + 1] if abs(k - s) <= 0.03]
    return gt[min(c, key=lambda k: abs(k - s))] if c else []


def match(g, d):
    pairs = sorted((math.hypot(q[0] - e[0], q[1] - e[1]), i, j) for i, q in enumerate(g) for j, e in enumerate(d))
    ug, ud, m = set(), set(), {}
    for dist, i, j in pairs:
        if i in ug or j in ud or dist > R_MATCH.get(g[i][2], 1.0): continue
        ug.add(i); ud.add(j); m[i] = j
    return m


def near_face(gx, gy, yaw, lx, ly):   # 정답 상자 꼭짓점의 자차 기준 가장 작은 x
    c, s = math.cos(yaw), math.sin(yaw)
    return min(to_ego(gx + u * c - v * s, gy + u * s + v * c)[0] for u in (-lx / 2, lx / 2) for v in (-ly / 2, ly / 2))


q95 = lambda v: float(np.percentile(v, 95)) if len(v) else None
rnd = lambda v, k=3: None if v is None else round(v, k)
t_lo, t_hi = st(int(scn[0]["frame_on"])) - 1.0, st(int(scn[-1]["frame_off"])) + 1.0
DISTS = sorted({float(x["ahead_m"]) for x in scn})
out = {"tag": a.tag, "set": a.set, "sweep": a.sweep, "ego_base": [bx, by, round(math.degrees(byaw), 2)], "configs": {}}
for cf in configs:
    D = det[cf]; per_tick_false = []; false_rows = []; mt = {}
    for s in sorted(ticks[cf]):
        if not (t_lo <= s <= t_hi): continue
        g, d = gt_at(s), D.get(s, []); m = match(g, d); mt[s] = (g, d, m); used = set(m.values()); nf = 0
        for j, e in enumerate(d):
            if j in used: continue
            ex, ey = to_ego(e[0], e[1])
            cat = "표적 근처" if any(math.hypot(e[0] - q[0], e[1] - q[1]) <= a.near for q in g) else place(e[0], e[1])
            false_rows.append((s, ex, ey, cat)); nf += 1
        per_tick_false.append(nf)
    rows = []
    for r in scn:
        t_on, t_off = st(int(r["frame_on"])) + 0.5, st(int(r["frame_off"])) - 0.1
        n = hit = 0; err = []; face = []
        for s in sorted(mt):
            if not (t_on <= s <= t_off): continue
            g, d, m = mt[s]
            if not g: continue
            i = min(range(len(g)), key=lambda i: math.hypot(g[i][0] - float(r["x_map"]), g[i][1] - float(r["y_map"])))
            gx, gy, gl, gyaw, glx, gly = g[i]; gex, gey = to_ego(gx, gy); n += 1
            if i in m:
                hit += 1; e = d[m[i]]; dx_, dy_ = to_ego(e[0], e[1]); err.append((dx_ - gex, dy_ - gey, math.hypot(dx_ - gex, dy_ - gey)))
                if e[3] is not None: face.append(e[3] - near_face(gx, gy, gyaw, glx, gly))
        rows.append({"k": int(r["k"]), "type": r["type"], "ahead_m": float(r["ahead_m"]), "lat": r["lat_name"], "ticks": n, "hit": hit,
                     "rate": rnd(hit / n if n else None), "_err": err, "_face": face})
    bins = {}
    for tp in ("ped", "car", "cone"):
        for lat in ("center", "edge", "all"):
            for dm in DISTS:
                sel = [x for x in rows if x["type"] == tp and x["ahead_m"] == dm and (lat == "all" or x["lat"] == lat)]
                n = sum(x["ticks"] for x in sel); h = sum(x["hit"] for x in sel)
                e = [q[2] for x in sel for q in x["_err"]]; fc = [q for x in sel for q in x["_face"]]
                bins[f"{tp}/{lat}/{int(dm)}"] = {"ticks": n, "hit": h, "rate": rnd(h / n if n else None), "err_mean": rnd(float(np.mean(e)) if e else None),
                                               "face_mean": rnd(float(np.mean(fc)) if fc else None)}
    dist90 = {}
    for tp in ("ped", "car", "cone"):
        ok = [dm for dm in DISTS if (bins[f"{tp}/all/{int(dm)}"]["rate"] or 0) >= 0.9]
        cont = 0.0
        for dm in DISTS:
            if dm in ok: cont = dm
            else: break
        dist90[tp] = {"far90": max(ok) if ok else 0.0, "cont90": cont}
    nt = len(per_tick_false); cats = collections.Counter(f[3] for f in false_rows)
    roi = [f for f in false_rows if 0 <= f[1] <= 45 and abs(f[2]) <= 5 and f[3] != "표적 근처"]
    allerr = {tp: [q for x in rows if x["type"] == tp for q in x["_err"]] for tp in ("ped", "car", "cone")}
    face = [q for x in rows if x["type"] == "car" for q in x["_face"]]
    pt = sorted(ptime[cf])
    out["configs"][cf] = {
        "eval_ticks": nt, "rate_by_type": {tp: rnd(sum(x["hit"] for x in rows if x["type"] == tp) / max(1, sum(x["ticks"] for x in rows if x["type"] == tp))) for tp in ("ped", "car", "cone")},
        "dist90": dist90,
        "false_per_tick": rnd(sum(per_tick_false) / nt if nt else None), "false_max": max(per_tick_false) if nt else None,
        "false_by_place_per_tick": {k: rnd(v / nt, 4) for k, v in sorted(cats.items())},
        "false_roi_per_tick": rnd(len(roi) / nt if nt else None, 4), "false_roi_n": len(roi),
        "false_lane_per_tick": rnd(sum(1 for f in roi if f[3] == "기타:차로 위") / nt if nt else None, 4),
        "false_roi_by_place": dict(collections.Counter(f[3] for f in roi)),
        "err_by_type": {tp: {"n": len(v), "mean": rnd(float(np.mean([q[2] for q in v])) if v else None), "p95": rnd(q95([q[2] for q in v])),
                             "long_mean": rnd(float(np.mean([q[0] for q in v])) if v else None)} for tp, v in allerr.items()},
        "car_face_err": {"n": len(face), "mean": rnd(float(np.mean(face)) if face else None), "abs_mean": rnd(float(np.mean(np.abs(face))) if face else None),
                         "abs_p95": rnd(q95(np.abs(face))) if face else None},
        "ptime_ms": {"n": len(pt), "med": rnd(float(np.median(pt))) if pt else None, "p95": rnd(q95(pt)), "max": rnd(max(pt)) if pt else None},
        "by_bin": bins, "roi_false_rows": [[s, round(x, 2), round(y, 2), c] for s, x, y, c in roi][:200]}
json.dump(out, open(a.out, "w"), ensure_ascii=False, indent=1)
for cf, v in out["configs"].items():
    print(cf, json.dumps({k: v[k] for k in ("rate_by_type", "dist90", "false_per_tick", "false_roi_per_tick", "false_lane_per_tick")}, ensure_ascii=False))
