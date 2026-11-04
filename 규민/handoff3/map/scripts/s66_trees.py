#!/usr/bin/env python3
# 10/23: 가로수·풀숲 레이어 배치(서버 없음). OSM 원본에 있는 것만 씀
#   natural=tree 점 -> 그 자리에 한 그루. leaf_type=needleleaved 면 침엽수 메시
#   natural=tree_row 선 -> TREE_ROW_STEP 간격(캠퍼스 OSM 에는 0개)
#   natural=wood 면(way 와 multipolygon relation), natural=scrub 면 -> AREA_STEP 격자 + 흔들기(시드 고정)로 성기게
#   노선(test_drive ROUTES_V3S46 5노선+30쌍 road 의 lane -1) 100m 안만
# 침범: 수관(위에서 본 원, 반경 = 메시 반폭 x 배율)이 모든 road 의 차선+인도 띠 + 0.5m 와 겹치지 않게
#   수관 아래 높이를 몰라 수관 전체로 막음(차 높이 3m 아래 침범 0 보다 보수적). 모자라면 배율을 MIN_SCALE 까지 줄이고, 그래도 안 되면 뺌
#   건물 상자(1순위·2순위)와도 안 겹치게 같은 방법
# 메시: 반폭은 09/25 서버 bbox(Acer_02 4.32·4.06, Ash_01 6.58·7.30). Cypress 는 아직 안 잼 -> 가정 2.5m, 5단계에서 재고 다시 돌림(--sizes)
# 도로: v3.1 동결 xodr(214507af). 좌표: OSM 위경도 -> xodr 머리말 투영(s22 Frame) -> CARLA (x, -y)
# 출력: data/trees_layout_s66.csv, docs/logs/s66_trees.log(stdout), figures/s66_trees_plan.png
# 사용: .venv-carla/bin/python map/scripts/s66_trees.py [--sizes <json>]
import argparse
import csv
import hashlib
import json
import math
import random
import sys
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from shapely.geometry import LineString, Point, Polygon  # noqa: E402
from shapely.ops import linemerge, polygonize, unary_union  # noqa: E402

HERE = Path(__file__).resolve().parent
BASE = HERE.parent
sys.path.insert(0, str(HERE))
import s49_b2_v3 as b49  # noqa: E402
from s22_buildings import RAW_OSM, Frame  # noqa: E402
import place_campus_layers as pcl  # noqa: E402

XODR = BASE / "maps/cbnu_campus_frozen_v31.xodr"
XODR_SHA = "214507afc87c2523f191e1c19154fb44b781eb16930b15bbc965fa2ab37ceb84"
TIER1 = BASE / "data/buildings_layout_v3.csv"
TIER2 = BASE / "data/buildings_layout_tier2_s66.csv"
OUT = BASE / "data/trees_layout_s66.csv"
FIG = BASE / "docs/figures/s66_trees_plan.png"
ZONE_M = 100.0
MARGIN = 0.5
MIN_SCALE = 0.5
AREA_STEP = 20.0        # 면 안 격자 간격(m). 한 그루가 약 400m2 -> 성김(실제 숲 밀도보다 훨씬 낮음, 가정)
JITTER = 5.0
TREE_ROW_STEP = 8.0
SEED = 66
TREE = "/Game/Carla/Static/Vegetation/Trees/{0}.{0}"
BUSH = "/Game/Carla/Static/Vegetation/Bushes/{0}.{0}"
# 이름 -> (경로, 수관 반폭 m(배율 1), 근거)
MESH = {
    "SM_Acer_02": (TREE.format("SM_Acer_02"), 4.32, "세션15 bbox extent 4.32·4.06"),
    "SM_Ash_01": (TREE.format("SM_Ash_01"), 7.30, "세션15 bbox extent 6.58·7.30"),
    "SM_Cypress": (TREE.format("SM_Cypress"), 2.5, "가정(안 잼)"),
    "SM_Bush_M_v1": (BUSH.format("SM_Bush_M_v1"), 1.5, "가정(안 잼)"),
}


def osm_layers():
    root = ET.parse(RAW_OSM).getroot()
    nodes = {n.get("id"): (float(n.get("lat")), float(n.get("lon"))) for n in root.iter("node")}
    tg = lambda e: {t.get("k"): t.get("v") for t in e.iter("tag")}
    ways = {w.get("id"): w for w in root.iter("way")}
    pts, rows, areas = [], [], []
    for n in root.iter("node"):
        t = tg(n)
        if t.get("natural") == "tree":
            pts.append((f"node{n.get('id')}", nodes[n.get("id")], t))
    for w in root.iter("way"):
        t = tg(w)
        ll = [nodes[nd.get("ref")] for nd in w.iter("nd") if nd.get("ref") in nodes]
        if t.get("natural") == "tree_row":
            rows.append((f"way{w.get('id')}", ll, t))
        elif t.get("natural") in ("wood", "scrub") and len(ll) >= 4 and ll[0] == ll[-1]:
            areas.append((f"way{w.get('id')}", ll, t, None))
    for r in root.iter("relation"):
        t = tg(r)
        if t.get("natural") in ("wood", "scrub") and t.get("type") == "multipolygon":
            outer = [[nodes[nd.get("ref")] for nd in ways[m.get("ref")].iter("nd") if nd.get("ref") in nodes]
                     for m in r.iter("member") if m.get("type") == "way" and m.get("role") == "outer" and m.get("ref") in ways]
            areas.append((f"relation{r.get('id')}", outer, t, "multi"))
    return pts, rows, areas


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sizes", type=Path, default=None, help="잰 수관 반폭 JSON {메시 이름: 반폭 m}")
    args = ap.parse_args()
    if args.sizes:
        for k, v in json.loads(args.sizes.read_text()).items():
            if k in MESH:
                MESH[k] = (MESH[k][0], float(v), f"서버에서 잼 {args.sizes.name}")
    text = XODR.read_text()
    assert hashlib.sha256(text.encode()).hexdigest() == XODR_SHA
    fr = Frame(text)
    R = b49.road_polys(text)
    road = unary_union([v["full"] for v in R.values()])
    rroads, nroutes = pcl.route_roads()
    route_lane = unary_union([R[r]["lane"] for r in rroads if r in R])
    zone = route_lane.buffer(ZONE_M)
    bl = [pcl.rect(float(r["center_carla_x"]), float(r["center_carla_y"]), float(r["yaw_deg"]), float(r["length_m"]), float(r["width_m"]))
          for f in (TIER1, TIER2) for r in csv.DictReader(open(f))]
    bldg = unary_union(bl)
    print(f"xodr {XODR.name} {XODR_SHA[:16]}, road {len(R)}, 경로 {nroutes}개 road {len(rroads)}, 노선 {ZONE_M:g}m 띠 {zone.area:.0f}m2, 건물 상자 {len(bl)}")
    for k, (p, r, why) in MESH.items():
        print(f"메시 {k}: 수관 반폭 {r}m ({why})")
    pts, rows, areas = osm_layers()
    rng = random.Random(SEED)
    cand = []   # (src, kind, x, y, mesh, lat, lon)
    for src, ll, t in pts:
        x, y = fr.carla(*ll)
        mesh = "SM_Cypress" if t.get("leaf_type") == "needleleaved" else "SM_Acer_02"
        cand.append((src, "tree", x, y, mesh, ll[0], ll[1], t.get("leaf_type", "")))
    print(f"natural=tree 점 {len(pts)}개(캠퍼스 전체), tree_row 선 {len(rows)}개, wood·scrub 면 {len(areas)}개")
    for src, ll, t in rows:
        line = LineString([fr.carla(*p) for p in ll])
        n = max(1, int(line.length // TREE_ROW_STEP))
        for i in range(n + 1):
            q = line.interpolate(i * line.length / n)
            lat, lon = b49.inverse(fr, q.x, q.y)
            cand.append((src, "tree_row", q.x, q.y, "SM_Acer_02", lat, lon, t.get("leaf_type", "")))
    area_info = []
    for src, ll, t, kind in areas:
        if kind == "multi":
            lines = linemerge([LineString([fr.carla(*p) for p in ring]) for ring in ll])
            poly = unary_union(list(polygonize(lines)))
        else:
            poly = Polygon([fr.carla(*p) for p in ll]).buffer(0)
        inz = poly.intersection(zone)
        area_info.append((src, t.get("natural"), t.get("leaf_type", ""), poly.area, inz.area))
        if inz.is_empty:
            continue
        x0, y0, x1, y1 = inz.bounds
        gx = x0 + AREA_STEP / 2
        while gx < x1:
            gy = y0 + AREA_STEP / 2
            while gy < y1:
                x, y = gx + rng.uniform(-JITTER, JITTER), gy + rng.uniform(-JITTER, JITTER)
                if inz.contains(Point(x, y)):
                    if t.get("natural") == "scrub":
                        mesh = "SM_Bush_M_v1"
                    elif t.get("leaf_type") == "needleleaved":
                        mesh = "SM_Cypress"
                    else:
                        mesh = rng.choice(["SM_Acer_02", "SM_Ash_01"])   # mixed·없음 -> 활엽 두 종
                    lat, lon = b49.inverse(fr, x, y)
                    cand.append((src, t.get("natural"), x, y, mesh, lat, lon, t.get("leaf_type", "")))
                gy += AREA_STEP
            gx += AREA_STEP
    for src, nat, lt, a, ai in area_info:
        print(f"면 {src} natural={nat} leaf_type={lt or '-'}: 넓이 {a:.0f}m2, 노선 100m 안 {ai:.0f}m2")

    out, drop = [], Counter()
    for src, kind, x, y, mesh, lat, lon, lt in cand:
        p = Point(x, y)
        if not zone.contains(p):
            drop["노선 100m 밖"] += 1
            continue
        r0 = MESH[mesh][1]
        d_road = p.distance(road)
        d_bldg = p.distance(bldg)
        s = min(1.0, (d_road - MARGIN) / r0, d_bldg / r0)
        if s < MIN_SCALE:
            drop["차도·인도 또는 건물에 너무 가까움" if d_road - MARGIN < MIN_SCALE * r0 else "건물에 너무 가까움"] += 1
            continue
        s = math.floor(s * 100) / 100
        yaw = round(rng.uniform(0, 360), 1)
        out.append(dict(id=f"T{len(out) + 1:03d}", src=src, kind=kind, mesh=mesh, mesh_path=MESH[mesh][0], scale=s,
                        carla_x=round(x, 3), carla_y=round(y, 3), yaw_deg=yaw, lat=f"{lat:.7f}", lon=f"{lon:.7f}",
                        crown_r_m=round(r0 * s, 2), road_clear_m=round(d_road - r0 * s, 2), bldg_clear_m=round(d_bldg - r0 * s, 2),
                        leaf_type=lt, size_src=MESH[mesh][2]))
    with open(OUT, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(out[0]))
        w.writeheader()
        w.writerows(out)
    crown = [Point(r["carla_x"], r["carla_y"]).buffer(r["crown_r_m"]) for r in out]
    hit_road = sum(1 for c in crown if c.intersection(road.buffer(MARGIN)).area > 1e-3)
    hit_bldg = sum(1 for c in crown if c.intersection(bldg).area > 1e-3)
    print(f"\n후보 {len(cand)} -> 놓음 {len(out)}, 뺌 {dict(drop)}")
    print(f"놓은 것 출처: {dict(Counter(r['kind'] for r in out))}, 메시: {dict(Counter(r['mesh'] for r in out))}")
    print(f"배율 1.0 {sum(r['scale'] == 1.0 for r in out)}, 줄인 것 {sum(r['scale'] < 1.0 for r in out)}(최소 {min(r['scale'] for r in out)})")
    print(f"수관 vs 차도·인도+{MARGIN}m 겹침 {hit_road}, 수관 vs 건물 상자 겹침 {hit_bldg}, 도로 여유 최소 {min(r['road_clear_m'] for r in out):.2f}m")

    fig, ax = plt.subplots(figsize=(11, 11))
    for rid, v in R.items():
        for p in b49.pieces(v["full"]):
            ax.fill(*p.exterior.xy, color="0.8", lw=0)
        for p in b49.pieces(v["lane"]):
            ax.fill(*p.exterior.xy, color="#1baf7a" if rid in rroads else "0.6", alpha=0.6 if rid in rroads else 1, lw=0)
    for g in bl:
        ax.fill(*g.exterior.xy, color="#2a78d6", alpha=0.35, lw=0)
    for src, ll, t, kind in areas:
        rings = ll if kind == "multi" else [ll]
        for ring in rings:
            xs, ys = zip(*[fr.carla(*p) for p in ring])
            ax.plot(xs, ys, color="#6a8f3a", lw=0.8)
    for r in out:
        ax.add_patch(plt.Circle((r["carla_x"], r["carla_y"]), r["crown_r_m"], color="#3f7d20", alpha=0.6, lw=0))
    zb = zone.boundary
    for g in getattr(zb, "geoms", [zb]):
        ax.plot(*g.xy, color="#d55181", lw=0.7, ls="--")
    zx0, zy0, zx1, zy1 = zone.bounds
    ax.set_xlim(zx0, zx1)
    ax.set_ylim(zy1, zy0)
    ax.set_aspect("equal")
    ax.set_title(f"세션66 나무 {len(out)}그루(초록 원 = 수관), 연두 선 OSM wood, 분홍 점선 노선 {ZONE_M:g}m, 파랑 건물 상자", fontsize=9)
    fig.tight_layout()
    fig.savefig(FIG, dpi=110)
    print(f"-> {OUT.relative_to(BASE)}, {FIG.relative_to(BASE)}")


if __name__ == "__main__":
    main()
