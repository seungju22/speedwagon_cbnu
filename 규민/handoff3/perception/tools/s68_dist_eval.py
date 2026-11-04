"""10/23 4단계(P5): 거리별 인지 평가. 거리별 장면 bag 을 푼 CSV(s67_objects_dump.py)·점(s68_bag_points.py)·장면 csv(s68_dist_scene.py)로 셈. 호스트 .venv-carla(CARLA 서버 없음)
- 좌표: 지도 좌표를 자차 기준(base_link, 앞 x·왼쪽 y)으로 옮김. 자차 자세는 정답 측위 로그(i5_gt_<tag>.log 'base x y yaw')
  장면의 '앞 거리'는 자차 중심(base_link 앞 1.399m)에서 잰 이름표. 구간은 이 이름표(5·10·15·20·25·30·40m)로 나눔
- 시각: stamp = T0 + (frame - F0) x 0.05 (gtobj 'pub 1 frame F0 t T0'). 표적 하나의 평가 구간 = 놓은 frame + 0.5s ~ 지운 frame - 0.1s
- 짝짓기: 검출 토픽 메시지(틱)마다 같은 시각 정답(/perception/gt/objects, 0.03s 안)과 검출을 중심 거리 오름차순으로 일대일(탐욕)
  기준 보행자·콘 1.0m, 차 2.0m(정답 라벨로 정함). 표적이 짝지어진 틱 / 구간 틱 = 검출률
- 거짓 객체 = 짝 못 지은 검출. 정답 중심 4m 안이면 '표적 근처'(같은 물체가 갈라졌거나 중심이 기준 밖), 아니면 자리를 가름
  자리: 정류장 시설 -> 나무 -> 건물 -> 연석(가장 가까운 차로 끝에서 2m 안, 차로·보도 밖) -> 기타(차로 위·보도·그 밖)
  가르는 법은 s67_false_objects.py 와 같음(레이어 actors CSV·나무 목록·v3.1 동결 xodr). --set v1 이면 건물 2순위·나무를 뺌(그 판에 없음)
- 위치 오차: 짝지은 표적의 자차 기준 앞뒤·옆·중심 거리(평균·95%)
- 처리 시간: 클러스터 노드 debug/processing_time_ms, 지면 뺀 점군 -> 클러스터 객체 받은 시각 차(bag)
- 표적 점 수: 정답 중심 둘레(보행자·콘 반경 0.8m, 차 2.6x1.4m 상자)의 브리지 LiDAR 점(raw)·지면 뺀 점(seg) 틱 중앙값
사용: s68_dist_eval.py <tag> <v1|v2> <out.json> [--topic 검출 토픽] [--csv 표적별 csv]
"""
import argparse, bisect, collections, csv, json, math
from pathlib import Path
import numpy as np
import carla

C = Path.home() / "campus_mobility_sim"; L = C / "integration/logs"
ap = argparse.ArgumentParser(); ap.add_argument("tag"); ap.add_argument("set", choices=["v1", "v2"]); ap.add_argument("out")
ap.add_argument("--topic", default="/perception/object_recognition/detection/clustering/objects")
ap.add_argument("--csv", default=""); ap.add_argument("--near", type=float, default=4.0)
ap.add_argument("--actors", default=str(C / "map/docs/logs/campus_layers_dry_s66_all_actors.csv"))
ap.add_argument("--trees", default=str(C / "map/data/trees_layout_s66.csv"))
ap.add_argument("--xodr", default=str(C / "map/maps/cbnu_campus_frozen_v31.xodr"))
a = ap.parse_args()
S4 = L / "s68/step4"; GTT = "/perception/gt/objects"
R_MATCH = {7: 1.0, 0: 1.0, 1: 2.0}; LBL = {7: "ped", 0: "cone", 1: "car"}
FAC = [(656.19, 679.08), (653.26, 681.17), (654.35, 682.71), (657.28, 680.62)]   # shuttle_platforms_v31_r1.yaml lib_north facility
BUSSTOP = (3.598, 1.884)   # s51 mesh_sizes busstop

# 자차 자세·시각 기준
ego = next(l.split() for l in open(L / f"i5_gt_{a.tag}.log") if l.startswith("pub 1 "))
bx, by, byaw = float(ego[ego.index("base") + 1]), float(ego[ego.index("base") + 2]), math.radians(float(ego[ego.index("yaw") + 1]))
cy_, sy_ = math.cos(byaw), math.sin(byaw)
g0 = next(l.split() for l in open(L / f"s68/gtobj_{a.tag}_all.log") if l.startswith("pub 1 "))
F0, T0 = int(g0[3]), float(g0[5])
st = lambda fr: round(T0 + (fr - F0) * 0.05, 2)


def to_ego(x, y):
    dx, dy = x - bx, y - by
    return dx * cy_ + dy * sy_, -dx * sy_ + dy * cy_


# 자리 가르기(s67_false_objects.py 와 같은 규칙)
def in_box(px, py, cx, cy, yaw, lx, ly, pad):
    c, s = math.cos(yaw), math.sin(yaw); dx, dy = px - cx, py - cy
    u, v = dx * c + dy * s, -dx * s + dy * c
    return abs(u) <= lx / 2 + pad and abs(v) <= ly / 2 + pad


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


def place(x, y):
    if any(in_box(x, y, qx, qy, qyaw, BUSSTOP[0], BUSSTOP[1], 1.0) for qx, qy, qyaw in bus) or in_poly(x, y, FACG): return "정류장 시설"
    if any(math.hypot(x - tx, y - ty) <= tr for tx, ty, tr in trees): return "나무"
    if any(in_box(x, y, qx, qy, qyaw, sc, sc, 1.0) for qx, qy, qyaw, sc in bld if abs(x - qx) < sc + 3 and abs(y - qy) < sc + 3): return "건물"
    loc = carla.Location(x, -y, 0.0)
    if cmap.get_waypoint(loc, project_to_road=False, lane_type=LT.Driving) is not None: return "기타:차로 위"
    if cmap.get_waypoint(loc, project_to_road=False, lane_type=LT.Sidewalk) is not None: return "기타:보도"
    w = cmap.get_waypoint(loc, project_to_road=True, lane_type=LT.Driving)
    if w is not None and math.hypot(w.transform.location.x - loc.x, w.transform.location.y - loc.y) - w.lane_width / 2 <= 2.0: return "연석"
    return "기타:그 밖"


# 입력 읽기
gt = collections.defaultdict(list); det = collections.defaultdict(list)
for r in csv.DictReader(open(S4 / f"{a.tag}_objects.csv")):
    s = round(float(r["stamp"]), 2)
    if r["topic"] == GTT: gt[s].append((float(r["x"]), float(r["y"]), int(r["label"])))
    elif r["topic"] == a.topic: det[s].append((float(r["x"]), float(r["y"])))
ticks = sorted({round(float(r["stamp"]), 2) for r in csv.DictReader(open(S4 / f"{a.tag}_ticks.csv")) if r["topic"] == a.topic})
lat_ms = sorted(float(r["lat_ms"]) for r in csv.DictReader(open(S4 / f"{a.tag}_ticks.csv")) if r["topic"] == a.topic and r["lat_ms"] not in ("", "None"))
gk = sorted(gt)
scn = list(csv.DictReader(open(L / f"s68/scn_{a.tag}.csv")))
pts = np.load(S4 / f"{a.tag}_pts.npz")
P = {k: (pts[k + "_stamp"], pts[k + "_idx"], pts[k + "_xyz"]) for k in ("raw", "seg")}


def gt_at(s):
    i = bisect.bisect_left(gk, s); c = [k for k in gk[max(0, i - 1):i + 1] if abs(k - s) <= 0.03]
    return gt[min(c, key=lambda k: abs(k - s))] if c else []


def pts_at(k, s):
    sts, idx, xyz = P[k]; i = int(np.argmin(np.abs(sts - s)))
    if abs(sts[i] - s) > 0.03: return None
    return xyz[idx[i]: idx[i + 1] if i + 1 < len(idx) else len(xyz)]


def npts(arr, ex, ey, lab):
    if arr is None or len(arr) == 0: return 0
    if lab == 1:   # 차 상자(자차 축과 나란히 놓음)
        return int(np.sum((np.abs(arr[:, 0] - ex) <= 1.3) & (np.abs(arr[:, 1] - ey) <= 0.7)))
    return int(np.sum(np.hypot(arr[:, 0] - ex, arr[:, 1] - ey) <= 0.8))


def match(g, d):
    pairs = sorted((math.hypot(gx - dx, gy - dy), i, j) for i, (gx, gy, gl) in enumerate(g) for j, (dx, dy) in enumerate(d))
    ug, ud, m = set(), set(), {}
    for dist, i, j in pairs:
        if i in ug or j in ud or dist > R_MATCH.get(g[i][2], 1.0): continue
        ug.add(i); ud.add(j); m[i] = j
    return m


# 틱마다 짝짓기 -> 거짓 객체
t_lo, t_hi = st(int(scn[0]["frame_on"])) - 1.0, st(int(scn[-1]["frame_off"])) + 1.0
false_rows = []; per_tick_false = []; m_by_tick = {}
for s in ticks:
    if not (t_lo <= s <= t_hi): continue
    g, d = gt_at(s), det.get(s, []); m = match(g, d); m_by_tick[s] = (g, d, m); used = set(m.values()); nf = 0
    for j, (x, y) in enumerate(d):
        if j in used: continue
        ex, ey = to_ego(x, y)
        cat = "표적 근처" if any(math.hypot(x - gx, y - gy) <= a.near for gx, gy, _ in g) else place(x, y)
        false_rows.append((s, round(ex, 2), round(ey, 2), cat)); nf += 1
    per_tick_false.append(nf)

# 표적별 검출률·오차·점 수
rows = []
for r in scn:
    t_on, t_off = st(int(r["frame_on"])) + 0.5, st(int(r["frame_off"])) - 0.1
    n = hit = 0; err = []; raw_n = []; seg_n = []; first_hit = None
    for s in ticks:
        if not (t_on <= s <= t_off) or s not in m_by_tick: continue
        g, d, m = m_by_tick[s]
        if not g: continue
        i = min(range(len(g)), key=lambda i: math.hypot(g[i][0] - float(r["x_map"]), g[i][1] - float(r["y_map"])))
        gx, gy, gl = g[i]; gex, gey = to_ego(gx, gy); n += 1
        raw_n.append(npts(pts_at("raw", s), gex, gey, gl)); seg_n.append(npts(pts_at("seg", s), gex, gey, gl))
        if i in m:
            hit += 1; dx_, dy_ = to_ego(*d[m[i]]); err.append((dx_ - gex, dy_ - gey, math.hypot(dx_ - gex, dy_ - gey)))
            first_hit = s if first_hit is None else first_hit
    med = lambda v: float(np.median(v)) if v else None
    rows.append({"k": int(r["k"]), "type": r["type"], "ahead_m": float(r["ahead_m"]), "lat": r["lat_name"], "ticks": n, "hit": hit,
                 "rate": round(hit / n, 3) if n else None, "err_mean_m": round(float(np.mean([e[2] for e in err])), 3) if err else None,
                 "raw_pts_med": med(raw_n), "seg_pts_med": med(seg_n), "_err": err})

# 묶음
q95 = lambda v: float(np.percentile(v, 95)) if len(v) else None
rnd = lambda v, k=3: None if v is None else round(v, k)
by = collections.OrderedDict()
for tp in ("ped", "car", "cone"):
    for lat in ("center", "edge", "all"):
        for dm in sorted({x["ahead_m"] for x in rows}):
            sel = [x for x in rows if x["type"] == tp and x["ahead_m"] == dm and (lat == "all" or x["lat"] == lat)]
            n = sum(x["ticks"] for x in sel); h = sum(x["hit"] for x in sel); e = [q for x in sel for q in x["_err"]]
            by[f"{tp}/{lat}/{int(dm)}"] = {"ticks": n, "hit": h, "rate": rnd(h / n if n else None),
                                           "err_mean": rnd(float(np.mean([q[2] for q in e])) if e else None), "err_p95": rnd(q95([q[2] for q in e])),
                                           "err_long_mean": rnd(float(np.mean([q[0] for q in e])) if e else None),
                                           "err_lat_mean": rnd(float(np.mean([q[1] for q in e])) if e else None)}
allerr = {tp: [q for x in rows if x["type"] == tp for q in x["_err"]] for tp in ("ped", "car", "cone")}
cats = collections.Counter(c for *_, c in false_rows); nt = len(per_tick_false)
roi = [f for f in false_rows if 0 <= f[1] <= 45 and abs(f[2]) <= 5 and f[3] != "표적 근처"]
pt = sorted(float(r["ms"]) for r in csv.DictReader(open(S4 / f"{a.tag}_ptime.csv")))
res = {"tag": a.tag, "set": a.set, "topic": a.topic, "ego_base": [bx, by, round(math.degrees(byaw), 2)], "F0": F0, "T0": T0,
       "targets": len(rows), "eval_ticks": nt,
       "rate_by_type": {tp: rnd(sum(x["hit"] for x in rows if x["type"] == tp) / max(1, sum(x["ticks"] for x in rows if x["type"] == tp))) for tp in ("ped", "car", "cone")},
       "err_by_type": {tp: {"n": len(v), "mean": rnd(float(np.mean([q[2] for q in v])) if v else None), "p95": rnd(q95([q[2] for q in v])),
                            "long_mean": rnd(float(np.mean([q[0] for q in v])) if v else None), "lat_mean": rnd(float(np.mean([q[1] for q in v])) if v else None)}
                       for tp, v in allerr.items()},
       "false_per_tick_mean": rnd(sum(per_tick_false) / nt if nt else None), "false_per_tick_med": float(np.median(per_tick_false)) if nt else None,
       "false_per_tick_max": max(per_tick_false) if nt else None, "false_total": len(false_rows),
       "false_by_place": dict(sorted(cats.items())), "false_by_place_per_tick": {k: rnd(v / nt) for k, v in sorted(cats.items())},
       "false_roi_per_tick": rnd(len(roi) / nt if nt else None), "false_roi_by_place": dict(collections.Counter(f[3] for f in roi)),
       "ptime_ms": {"n": len(pt), "med": rnd(float(np.median(pt))), "p95": rnd(q95(pt)), "max": rnd(max(pt))},
       "recv_lat_ms": {"n": len(lat_ms), "med": rnd(float(np.median(lat_ms))), "p95": rnd(q95(lat_ms)), "max": rnd(max(lat_ms))} if lat_ms else None,
       "by_bin": by, "targets_detail": [{k: v for k, v in x.items() if k != "_err"} for x in rows]}
json.dump(res, open(a.out, "w"), ensure_ascii=False, indent=1)
if a.csv:
    with open(a.csv, "w", newline="") as f:
        w = csv.writer(f); w.writerow(["stamp", "x_ego", "y_ego", "place"]); w.writerows(false_rows)
print(json.dumps({k: res[k] for k in ("tag", "rate_by_type", "err_by_type", "false_per_tick_mean", "false_by_place_per_tick", "false_roi_per_tick", "ptime_ms", "recv_lat_ms")}, ensure_ascii=False))
