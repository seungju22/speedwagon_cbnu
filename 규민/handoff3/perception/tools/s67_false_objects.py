"""10/23: 클러스터링 객체(s67_objects_dump.py 의 _objects.csv)를 정답과 맞춰 참/거짓으로 나누고, 거짓 객체의 자리를 가른다. 호스트 .venv-carla 로 돈다(CARLA 서버 없음)
- 시각: 객체 stamp(ROS) = T0 + (CARLA frame - F0) x 0.05 (gtobj 로그 'pub 1 frame F0 t T0')
- 정답: --gt 에 scn csv(문 구역 장면, frame·id·x·y) 또는 gt_objects2 --log csv(frame·aid·label·x·y, 모든 actor) 를 줌. 둘 다 지도 좌표
  참 = 같은 시각 정답 중심에서 보행자 --r-ped m, 차량은 bbox 반 길이 + --r-ped m 안에 객체 중심이 있음
- 거짓 객체 자리(앞 칸부터 먼저 맞는 것): 정류장 시설(busstop 액터 상자 + 1m, 또는 노선 파일 시설 다각형 + 1m) -> 나무(수관 반경 안)
  -> 건물(정육면체 상자 + 1m) -> 차로 위(xodr Driving) -> 보도(Sidewalk) -> 도로 가장자리(가장 가까운 차로 끝에서 2m 안, 연석·경계) -> 기타
  레이어 자리는 campus_layers_dry_s66_all_actors.csv(레이어 v2 978, CARLA 좌표 -> 지도 (x,-y)), 도로는 v3.1 동결 xodr 를 carla.Map 으로 오프라인 읽음
- 같은 자리 묶음: 거짓 객체 중심을 2m 격자로 묶어 자리마다 나온 틱 수·처음·끝 시각을 셈
사용: s67_false_objects.py <_objects.csv> <topic> <out.json> --gt <csv> --gt-kind scn|gtlog --f0 F --t0 T [--t-from s --t-to s]
"""
import argparse, bisect, collections, csv, json, math
from pathlib import Path
import carla

C = Path.home() / "campus_mobility_sim"
ap = argparse.ArgumentParser()
ap.add_argument("objects"); ap.add_argument("topic"); ap.add_argument("out")
ap.add_argument("--gt", required=True); ap.add_argument("--gt-kind", choices=["scn", "gtlog"], required=True)
ap.add_argument("--f0", type=int, required=True); ap.add_argument("--t0", type=float, required=True)
ap.add_argument("--r-ped", type=float, default=1.0)
ap.add_argument("--actors", default=str(C / "map/docs/logs/campus_layers_dry_s66_all_actors.csv"))
ap.add_argument("--trees", default=str(C / "map/data/trees_layout_s66.csv"))
ap.add_argument("--xodr", default=str(C / "map/maps/cbnu_campus_frozen_v31.xodr"))
ap.add_argument("--t-from", type=float, default=-1e9); ap.add_argument("--t-to", type=float, default=1e9)
ap.add_argument("--csv-out", default="", help="객체별 판정 CSV")
a = ap.parse_args()
FAC = [(656.19, 679.08), (653.26, 681.17), (654.35, 682.71), (657.28, 680.62)]   # shuttle_platforms_v31_r1.yaml lib_north facility
BUSSTOP = (3.598, 1.884)   # s51 mesh_sizes busstop


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


FACG = grow(FAC, 1.0)
bus, bld = [], []
for r in csv.DictReader(open(a.actors)):
    x, y, yaw = float(r["x"]), -float(r["y"]), -math.radians(float(r["yaw"]))
    if r["layer"] == "busstop": bus.append((x, y, yaw))
    elif r["layer"] in ("buildings", "buildings2"): bld.append((x, y, yaw, float(r["scale"])))
trees = [(float(r["carla_x"]), -float(r["carla_y"]), float(r["crown_r_m"])) for r in csv.DictReader(open(a.trees))]
cmap = carla.Map("OpenDriveMap", open(a.xodr).read())
LT = carla.LaneType


def place(x, y):
    if any(in_box(x, y, bx, by, byaw, BUSSTOP[0], BUSSTOP[1], 1.0) for bx, by, byaw in bus) or in_poly(x, y, FACG): return "정류장 시설"
    if any(math.hypot(x - tx, y - ty) <= tr for tx, ty, tr in trees): return "나무"
    if any(in_box(x, y, bx, by, byaw, sc, sc, 1.0) for bx, by, byaw, sc in bld if abs(x - bx) < sc + 3 and abs(y - by) < sc + 3): return "건물"
    loc = carla.Location(x, -y, 0.0)
    if cmap.get_waypoint(loc, project_to_road=False, lane_type=LT.Driving) is not None: return "차로 위"
    if cmap.get_waypoint(loc, project_to_road=False, lane_type=LT.Sidewalk) is not None: return "보도"
    w = cmap.get_waypoint(loc, project_to_road=True, lane_type=LT.Driving)
    if w is not None:
        d = math.hypot(w.transform.location.x - loc.x, w.transform.location.y - loc.y) - w.lane_width / 2
        if d <= 2.0: return "도로 가장자리"
    return "기타"


gt = collections.defaultdict(list)   # 시각(0.01 반올림) -> [(x, y, 반경)]
for g in csv.DictReader(open(a.gt)):
    t = round(a.t0 + (int(g["frame"]) - a.f0) * 0.05, 2)
    if a.gt_kind == "scn":
        gt[t].append((float(g["x"]), float(g["y"]), a.r_ped, "PEDESTRIAN"))
    else:
        lab = g["label"]; r = a.r_ped if lab == "PEDESTRIAN" else (2.5 + a.r_ped if lab in ("CAR", "TRUCK", "BUS") else 1.0 + a.r_ped)
        gt[t].append((float(g["x"]), float(g["y"]), r, lab))
gk = sorted(gt)


def gt_at(s):
    i = bisect.bisect_left(gk, s); c = [k for k in gk[max(0, i - 1):i + 1] if abs(k - s) <= 0.03]
    return gt[c[0]] if c else []


ticks = collections.OrderedDict(); out_rows = []
spots = {}
for r in csv.DictReader(open(a.objects)):
    if r["topic"] != a.topic: continue
    s = float(r["stamp"])
    if not (a.t_from <= s <= a.t_to): continue
    x, y = float(r["x"]), float(r["y"])
    tk = ticks.setdefault(s, {"n": 0, "true": 0, "false": 0})
    tk["n"] += 1
    hit = [g for g in gt_at(s) if math.hypot(x - g[0], y - g[1]) <= g[2]]
    if hit:
        tk["true"] += 1; cat = "참:" + hit[0][3]
    else:
        tk["false"] += 1; cat = place(x, y)
        k = (round(x / 2) * 2, round(y / 2) * 2)
        sp = spots.setdefault(k, {"place": cat, "ticks": 0, "first": s, "last": s, "dz_max": 0})
        sp["ticks"] += 1; sp["last"] = s; sp["dz_max"] = max(sp["dz_max"], float(r["dz"]))
    out_rows.append([s, x, y, r["dx"], r["dy"], r["dz"], cat])
# 객체 없는 틱도 틱으로 셈(dump 의 _ticks.csv)
tk_csv = a.objects.replace("_objects.csv", "_ticks.csv")
allt = [float(r["stamp"]) for r in csv.DictReader(open(tk_csv)) if r["topic"] == a.topic and a.t_from <= float(r["stamp"]) <= a.t_to]
for s in allt: ticks.setdefault(s, {"n": 0, "true": 0, "false": 0})
fl = sorted(v["false"] for v in ticks.values()); nn = sorted(v["n"] for v in ticks.values())
q = lambda v, p: v[min(len(v) - 1, int(p * len(v)))] if v else None
cats = collections.Counter(r[6] for r in out_rows)
bycat_spots = collections.Counter(sp["place"] for sp in spots.values())
top = sorted(spots.items(), key=lambda kv: -kv[1]["ticks"])[:15]
res = {"topic": a.topic, "ticks": len(ticks), "objects": len(out_rows), "true_obj_ticks": sum(v["true"] for v in ticks.values()),
       "false_obj_ticks": sum(v["false"] for v in ticks.values()), "n_per_tick_med": q(nn, 0.5), "n_per_tick_max": nn[-1] if nn else None,
       "false_per_tick_med": q(fl, 0.5), "false_per_tick_p95": q(fl, 0.95), "false_per_tick_max": fl[-1] if fl else None,
       "ticks_with_false": sum(1 for v in ticks.values() if v["false"]), "by_category_obj_ticks": dict(cats),
       "false_spots_2m": len(spots), "false_spots_by_place": dict(bycat_spots),
       "top_spots": [{"xy": k, **v} for k, v in top]}
json.dump(res, open(a.out, "w"), ensure_ascii=False, indent=1)
if a.csv_out:
    with open(a.csv_out, "w", newline="") as f:
        w = csv.writer(f); w.writerow(["stamp", "x", "y", "dx", "dy", "dz", "judge"]); w.writerows(out_rows)
print(json.dumps({k: v for k, v in res.items() if k != "top_spots"}, ensure_ascii=False))
