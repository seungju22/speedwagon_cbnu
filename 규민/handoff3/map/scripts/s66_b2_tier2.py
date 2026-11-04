#!/usr/bin/env python3
# 10/23: 건물 2순위 11동 배치(서버 없음). 10/04 2순위(docs/logs/s25_buildings.csv tier=2, 노선 50~100m)
# 방법은 1순위 30동(s49_b2_v3.py)과 같음. 그 스크립트의 함수를 불러 씀(그 파일은 안 고침)
#   OSM 윤곽 -> CARLA 좌표 -> 모든 road 의 차선+인도 띠 + 0.5m 를 깎음 -> 주축 방향 직사각형 최대 4개(폭 4m 미만 버림)
#   높이 = building:levels x 4.0m, levels 없으면 4층(1순위와 같은 값. OSM 중앙값도 다시 세어 로그에 남김)
#   표현은 나(정육면체 덩어리). 2순위에는 노선이 지나는 동이 없어 물리기 없음
# 도로: v3.1 동결 xodr(214507af). 1순위 layout 은 v3(a17ab012)로 깎았지만 이번 것은 팀이 쓰는 v3.1 로 깎음
# 출력: data/buildings_layout_tier2_s66.csv(1순위 layout 과 같은 열), docs/logs/s66_b2_tier2.log(stdout), figures/s66_tier2_plan.png
# 사용: .venv-carla/bin/python map/scripts/s66_b2_tier2.py
import csv
import hashlib
import math
import statistics
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from shapely.geometry import Polygon  # noqa: E402
from shapely.ops import unary_union  # noqa: E402

HERE = Path(__file__).resolve().parent
BASE = HERE.parent
sys.path.insert(0, str(HERE))
import s49_b2_v3 as b49  # noqa: E402
from s22_buildings import RAW_OSM, Frame, load_osm  # noqa: E402
import place_campus_layers as pcl  # noqa: E402

XODR = BASE / "maps/cbnu_campus_frozen_v31.xodr"
XODR_SHA = "214507afc87c2523f191e1c19154fb44b781eb16930b15bbc965fa2ab37ceb84"
S25 = BASE / "docs/logs/s25_buildings.csv"
TIER1 = BASE / "data/buildings_layout_v3.csv"
OUT = BASE / "data/buildings_layout_tier2_s66.csv"
FIG = BASE / "docs/figures/s66_tier2_plan.png"
MARGIN = 0.5
FLOOR_H = 4.0
DEF_LV = 4


def main():
    text = XODR.read_text()
    assert hashlib.sha256(text.encode()).hexdigest() == XODR_SHA, "xodr 해시 다름"
    fr = Frame(text)
    print(f"xodr {XODR.name} sha {XODR_SHA[:16]}, 투영 {fr.proj} offset ({fr.ox}, {fr.oy})")
    R = b49.road_polys(text)
    cut = unary_union([v["full"] for v in R.values()]).buffer(MARGIN)
    cutc = unary_union([v["full"] for v in R.values()])
    rroads, nroutes = pcl.route_roads()
    route_lane = unary_union([R[r]["lane"] for r in rroads if r in R])
    print(f"road {len(R)}, 깎는 영역(차선+인도+{MARGIN}m) {cut.area:.0f}m2, 경로 {nroutes}개 road {len(rroads)}")

    nodes, ways = load_osm(RAW_OSM)
    ratio, levels_all = [], []
    for nds, tags in ways.values():
        if "building" not in tags:
            continue
        try:
            lv = float(tags.get("building:levels", ""))
        except ValueError:
            lv = None
        try:
            h = float(tags.get("height", "").replace("m", "").strip())
        except ValueError:
            h = None
        if lv:
            levels_all.append(lv)
        if lv and h:
            ratio.append(h / lv)
    print(f"참고: OSM height/levels 중앙값 {statistics.median(ratio):.2f}m({len(ratio)}동), levels 중앙값 {statistics.median(levels_all):g}"
          f"({len(levels_all)}동). 쓴 값 층고 {FLOOR_H}m, 기본 {DEF_LV}층(1순위와 같음)")

    platforms = pcl.parse_platforms()
    t1 = [pcl.rect(float(r["center_carla_x"]), float(r["center_carla_y"]), float(r["yaw_deg"]), float(r["length_m"]), float(r["width_m"]))
          for r in csv.DictReader(open(TIER1))]
    t1u = unary_union(t1)
    out, draw = [], []
    for b in csv.DictReader(open(S25)):
        if b["tier"] != "2":
            continue
        wid = b["osm_id"]
        nds, tags = ways[wid]
        ll = [nodes[i] for i in nds if i in nodes]
        if ll[0] == ll[-1]:
            ll = ll[:-1]
        P = Polygon([fr.carla(*p) for p in ll])
        if not P.is_valid:
            P = P.buffer(0)
        Pc = Polygon([(lon, lat) for lat, lon in ll]).centroid
        Cc = P.difference(cut)
        Cc = unary_union([p for p in b49.pieces(Cc) if p.area > 1.0]) if not Cc.is_empty else Cc
        a0 = P.area
        name = tags.get("name", "")
        if Cc.is_empty:
            print(f"{name or 'way' + wid}: 깎고 남은 것 없음, 안 놓음")
            continue
        ps = sorted(b49.pieces(Cc), key=lambda p: -p.area)
        yaw = b49.rect_params(ps[0].minimum_rotated_rectangle)[2]
        parts, cov = b49.decompose(Cc, yaw)
        if not parts:
            print(f"{name or 'way' + wid}: 폭 4m 넘는 상자 없음, 안 놓음")
            continue
        lv_tag = tags.get("building:levels", "")
        try:
            lv = float(lv_tag)
            src = f"OSM building:levels {lv_tag} x {FLOOR_H:g}m"
        except ValueError:
            lv = DEF_LV
            src = f"기본 {DEF_LV}층 x {FLOOR_H:g}m (levels 태그 없음)"
        H = lv * FLOOR_H
        boxes = [pcl.rect(*q) for q in parts]
        bu = unary_union(boxes)
        brow = []
        for k, (cx, cy, pyaw, L, W) in enumerate(parts, 1):
            lat_c, lon_c = b49.inverse(fr, cx, cy)
            rt = fr.carla(lat_c, lon_c)
            fit = pcl.fit_mesh(L, H)
            a, nl, nw, nh = pcl.cube_plan(L, W, H)
            brow.append(dict(name=name, code=b["code"], way_id=wid, part=k, parts=len(parts),
                             center_carla_x=round(cx, 3), center_carla_y=round(cy, 3),
                             center_lat=f"{lat_c:.9f}", center_lon=f"{lon_c:.9f}", latlon_roundtrip_m=round(math.dist(rt, (cx, cy)), 4),
                             yaw_deg=round(pyaw, 2), length_m=round(L, 2), width_m=round(W, 2), height_m=round(H, 1),
                             levels_tag=lv_tag, osm_height_tag=tags.get("height", ""), height_src=src,
                             area_osm_m2=round(a0, 1), area_cut_m2=round(Cc.area, 1), cut_ratio=round(max(0.0, 1 - Cc.area / a0), 4),
                             pieces=len(ps), boxes_cover_cut=round(cov, 3), boxes_cover_osm=round(bu.intersection(P).area / a0, 3),
                             push_out="", route_lane_overlap_osm_m2=round(P.intersection(route_lane).area, 2),
                             box_route_lane_overlap_m2=round(bu.intersection(route_lane).area, 3),
                             v2_cut_ratio="", v2_s40_cut_ratio="", v3_minus_v2_cut_m2="", change_reason="s66 2순위(v3.1 로 깎음)",
                             repr_a=f"{fit['mesh']} x{fit['scale']} (길이 {fit['len_err_m']:+.1f}m, 높이 {fit['h_err_m']:+.1f}m, 폭 미측정)",
                             repr_b=f"정육면체 {a:.2f}m x {nl * nw * nh}개({nl}x{nw}x{nh})", repr_c="나", repr_c_why="2순위는 나",
                             osm_lat=f"{Pc.y:.7f}", osm_lon=f"{Pc.x:.7f}"))
        out += brow
        draw.append((P, Cc, boxes, brow[0]))
        dims = ", ".join(f"{r['length_m']}x{r['width_m']}" for r in brow)
        n_cube = sum(math.prod(pcl.cube_plan(r["length_m"], r["width_m"], r["height_m"])[1:]) for r in brow)
        print(f"{name or '(이름 없음)'} way{wid} {b['code']}: 노선까지 {b['min_dist_m']}m({b['nearest_route']}), 깎임 {max(0.0, 1 - Cc.area / a0) * 100:.1f}%, "
              f"상자 {len(parts)}개 [{dims}] 높이 {H:g}m({src}), 덮음 깎은것의 {cov * 100:.0f}%/OSM의 {bu.intersection(P).area / a0 * 100:.0f}%, "
              f"정육면체 {n_cube}개, 경로 차선 겹침 {bu.intersection(route_lane).area:.3f}m2")

    with open(OUT, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(csv.DictReader(open(TIER1)).fieldnames))
        w.writeheader()
        w.writerows(out)
    allb = [(r["way_id"], pcl.rect(r["center_carla_x"], r["center_carla_y"], r["yaw_deg"], r["length_m"], r["width_m"])) for r in out]
    bad = [(w, round(g.intersection(cut).area, 3)) for w, g in allb if g.intersection(cut).area > 1e-3]
    bad0 = sum(1 for w, g in allb if g.intersection(cutc).area > 1e-3)
    ov1 = [(w, round(g.intersection(t1u).area, 2)) for w, g in allb if g.intersection(t1u).area > 0.01]
    ovs = sorted({(a[0], c[0]) for i, a in enumerate(allb) for c in allb[i + 1:] if a[0] != c[0] and a[1].intersection(c[1]).area > 0.01})
    print(f"\n동 {len({r['way_id'] for r in out})}, 상자 {len(out)}, 정육면체 합 "
          f"{sum(math.prod(pcl.cube_plan(r['length_m'], r['width_m'], r['height_m'])[1:]) for r in out)}")
    print(f"상자 vs 도로(차선+인도)+0.5m 겹침 {len(bad)} {bad}, 도로 자체 겹침 {bad0}, 경로 차선 겹침 합 "
          f"{sum(r['box_route_lane_overlap_m2'] for r in out):.3f}m2")
    print(f"1순위 상자와 겹침 {len(ov1)} {ov1}, 2순위끼리 다른 동 겹침 {len(ovs)} {ovs}")
    print(f"높이 출처: 태그 {len({r['way_id'] for r in out if r['levels_tag']})}동, 기본값 {len({r['way_id'] for r in out if not r['levels_tag']})}동")

    fig, ax = plt.subplots(figsize=(11, 11))
    for rid, v in R.items():
        for p in b49.pieces(v["lane"]):
            ax.fill(*p.exterior.xy, color="#1baf7a" if rid in rroads else "0.7", alpha=0.6 if rid in rroads else 1, lw=0)
    for g in t1:
        ax.plot(*g.exterior.xy, color="0.35", lw=0.6)
    for P, Cc, boxes, r in draw:
        ax.plot(*P.exterior.xy, color="#eb6834", lw=1.0)
        for bx in boxes:
            ax.fill(*bx.exterior.xy, color="#2a78d6", alpha=0.5, lw=0)
        c = P.representative_point()
        ax.annotate((r["name"] or f"way{r['way_id']}").split(" (")[0], (c.x, c.y), fontsize=7, ha="center")
    for p in platforms:
        ax.plot(p["x"], p["y"], marker="^", color="#d55181", ms=7, ls="")
    ax.set_aspect("equal")
    ax.invert_yaxis()
    ax.set_title(f"10/23 2순위 {len(draw)}동(파랑 상자, 주황 OSM 윤곽), 회색 선 1순위 상자, 초록 경로 차선 (v3.1 214507af)", fontsize=9)
    fig.tight_layout()
    fig.savefig(FIG, dpi=110)
    print(f"-> {OUT.relative_to(BASE)}, {FIG.relative_to(BASE)}")


if __name__ == "__main__":
    main()
