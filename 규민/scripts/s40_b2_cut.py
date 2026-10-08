#!/usr/bin/env python3
# 맵세션40 A(B2): 세션25 1순위 30동 건물 초안 — OSM 폴리곤에서 도로 전체 폭 + 0.5m 와 겹친 부분을 깎는다(읽기전용, 서버 없음, 판정 없음)
# 대상: logs/s25_buildings.csv tier=1 (30동, 전부 way). 세션22 이름 목록 아님
# 도로 영역: 동결본 xodr 의 모든 road(917)를 s34_curvature.analyze 표본(0.02m)으로 띠 다각형으로 만든다
#   띠 = t 가 laneOffset(기준선 = 중앙선) 에서 laneOffset - (lane -1 폭 + lane -2 폭) 까지(차선 + 인도. 인도 없는 road 는 차선만)
#   이 지도는 모든 road 가 반쪽 도로라 왼쪽 차선이 없다(세션40 계수: 차선 구성 (-2,-1,0) 532 / (-1,0) 385)
#   합집합을 0.5m 넓힌 것이 깎는 영역. 건물 = OSM 폴리곤(Frame 으로 CARLA 좌표) - 깎는 영역
# 노선 차선 겹침: 5 노선(test_drive.py ROUTES_V2) road 의 lane -1 띠(0.5m 넓히지 않음)와 원래 폴리곤이 겹치는가
# 출력: data/processed/s40_b2_cut.csv, docs/figures/s40_b2_cut_grid.png(30칸, 전후), logs 는 stdout
# 사용: .venv-carla/bin/python map/scripts/s40_b2_cut.py
import csv
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from shapely.geometry import Polygon, box  # noqa: E402
from shapely.ops import unary_union  # noqa: E402

HERE = Path(__file__).resolve().parent
BASE = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(BASE / "tests"))
from s22_buildings import RAW_OSM, Frame, load_osm  # noqa: E402
from s34_curvature import analyze  # noqa: E402

sys.argv = sys.argv[:1]
import test_drive as td  # noqa: E402

XODR = BASE / "maps/cbnu_campus_frozen_v2.xodr"
S25 = BASE / "docs/logs/s25_buildings.csv"
OUT = BASE / "data/processed/s40_b2_cut.csv"
FIG = BASE / "docs/figures/s40_b2_cut_grid.png"
MARGIN = 0.5
STEP = 10                           # 표본 10개마다(0.2m) 한 점
for _f in ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",):
    if Path(_f).exists():
        matplotlib.font_manager.fontManager.addfont(_f)
plt.rcParams["font.family"] = ["Noto Sans CJK JP", "DejaVu Sans"]


def strip(rows, t_in, t_out):
    """t_in(s), t_out(s) 사이 띠 다각형(CARLA 좌표 = (x, -y))"""
    pick = rows[::STEP] + ([rows[-1]] if (len(rows) - 1) % STEP else [])
    left, right = [], []
    for r in pick:
        s, x, y, h, k, off, w, ws, tag = r
        for t, out in ((t_in(r), left), (t_out(r), right)):
            out.append((x - math.sin(h) * t, -(y + math.cos(h) * t)))
    p = Polygon(left + right[::-1])
    return p if p.is_valid else p.buffer(0)


def main():
    text = XODR.read_text()
    frame = Frame(text)
    root = ET.fromstring(text)
    route_roads = {rid for k, v in td.ROUTES_V2.items() for rid in v["roads"]}
    full, lane1 = [], []
    for road in root.iter("road"):
        rid, L, rows, *_ = analyze(road)
        if len(rows) < 2:
            continue
        full.append(strip(rows, lambda r: r[5], lambda r: r[5] - r[6] - r[7]))
        if rid in route_roads:
            lane1.append(strip(rows, lambda r: r[5], lambda r: r[5] - r[6]))
    road_area = unary_union(full)
    cut_area = road_area.buffer(MARGIN)
    route_lane = unary_union(lane1)
    print(f"도로 {len(full)} road 띠 합집합 면적 {road_area.area:.0f}m2, +{MARGIN}m 깎는 영역 {cut_area.area:.0f}m2, 노선 lane -1 띠 {len(lane1)} road")

    nodes, ways = load_osm(RAW_OSM)
    rows_out = []
    for b in csv.DictReader(open(S25)):
        if b["tier"] != "1":
            continue
        nds, tags = ways[b["osm_id"]]
        ll = [nodes[i] for i in nds if i in nodes]
        if ll[0] == ll[-1]:
            ll = ll[:-1]
        poly = Polygon([frame.carla(*p) for p in ll])
        if not poly.is_valid:
            poly = poly.buffer(0)
        after = poly.difference(cut_area)
        a0, a1 = poly.area, after.area
        ratio = max(0.0, 1 - a1 / a0) if a0 > 0 else 0.0   # 부동소수 오차로 -0 이 나오는 것 막음
        on_lane = poly.intersection(route_lane).area
        name = b["name"] or "(이름 없음)"
        rows_out.append(dict(name=name, way_id=b["osm_id"], code=b["code"], area_before_m2=round(a0, 1), area_after_m2=round(a1, 1),
                             cut_ratio=round(ratio, 4), pieces_after=len(getattr(after, "geoms", [after])) if a1 > 0 else 0,
                             route_lane_overlap_m2=round(on_lane, 2), s25_route_inside=b["route_inside"], s25_min_dist_m=b["min_dist_m"],
                             _poly=poly, _after=after))
    rows_out.sort(key=lambda r: (-(r["route_lane_overlap_m2"] > 0), -r["cut_ratio"], r["name"]))
    keys = [k for k in rows_out[0] if not k.startswith("_")]
    with open(OUT, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows({k: r[k] for k in keys} for r in rows_out)
    print("순서: 노선 lane -1 과 겹치는 동 먼저, 그다음 깎인 비율 큰 순")
    for r in rows_out:
        print(f"{r['name']} way{r['way_id']}: 면적 {r['area_before_m2']} -> {r['area_after_m2']}m2, 깎임 {r['cut_ratio']*100:.1f}%,"
              f" 조각 {r['pieces_after']}, 노선 lane -1 겹침 {r['route_lane_overlap_m2']}m2")
    over = [r for r in rows_out if r["cut_ratio"] > 0.5]
    print(f"깎인 비율 50% 초과 {len(over)}동: {[r['name'] + ' ' + r['way_id'] for r in over]}")
    print(f"깎임 0 인 동 {sum(r['cut_ratio'] == 0 for r in rows_out)} / 30")

    # 그림: 30칸. 회색 = 도로 영역(+0.5m 전), 빨간 선 = 원래 폴리곤, 파랑 채움 = 깎은 뒤
    n = len(rows_out)
    cols = 6
    fig, axes = plt.subplots(math.ceil(n / cols), cols, figsize=(cols * 3.2, math.ceil(n / cols) * 3.2))
    for ax, r in zip(axes.flat, rows_out):
        p = r["_poly"]
        x0, y0, x1, y1 = p.bounds
        pad = max(x1 - x0, y1 - y0) * 0.25 + 8
        win = box(x0 - pad, y0 - pad, x1 + pad, y1 + pad)
        for g in getattr(road_area.intersection(win), "geoms", [road_area.intersection(win)]):
            if g.geom_type == "Polygon" and not g.is_empty:
                ax.fill(*g.exterior.xy, color="0.8", lw=0)
        for g in getattr(r["_after"], "geoms", [r["_after"]]):
            if g.geom_type == "Polygon" and not g.is_empty:
                ax.fill(*g.exterior.xy, color="tab:blue", alpha=0.6, lw=0)
        ax.plot(*p.exterior.xy, color="tab:red", lw=1)
        ax.set_xlim(x0 - pad, x1 + pad)
        ax.set_ylim(y1 + pad, y0 - pad)        # CARLA y 는 화면 아래가 + -> 뒤집어 북쪽이 위
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        tag = " [노선]" if r["route_lane_overlap_m2"] > 0 else ""
        ax.set_title(f"{r['name'][:14]}{tag}\n깎임 {r['cut_ratio']*100:.1f}%", fontsize=8)
    for ax in list(axes.flat)[n:]:
        ax.axis("off")
    fig.suptitle("세션25 1순위 30동: 빨간 선 = OSM 폴리곤, 파랑 = 도로 전체 폭 +0.5m 를 깎은 뒤, 회색 = 도로(차선+인도)", fontsize=10)
    fig.tight_layout()
    fig.savefig(FIG, dpi=110)
    print(f"-> {OUT}, {FIG}")


if __name__ == "__main__":
    main()
