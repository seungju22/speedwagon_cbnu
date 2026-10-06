#!/usr/bin/env python3
# 도로망(cbnu_internal_only_localtm.xodr)을 OSM 배경(건물외곽선+보행로) 위에
# 겹쳐 그린다. 읽기전용(xodr/osm 미수정), 외부 타일 다운로드 없음
# (이미 가진 data/raw/cbnu_campus.osm만 사용).
#
# 좌표계: xodr 좌표를 geo_calibrate.py와 같은 방법(junction이름=원본OSM
# 노드ID를 이용한 Umeyama 유사변환)으로 ENU(동쪽,북쪽 단위:m)로 역변환한다.
# ENU는 정의상 +y=북쪽이므로, 이 좌표를 그대로 x=동, y=북으로 그리면
# 위쪽이 자동으로 진북이 된다(plot 원좌표를 그대로 쓰지 않음).
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

import matplotlib.font_manager as fm  # noqa: E402
fm.fontManager.addfont("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
matplotlib.rcParams["font.family"] = "Noto Sans CJK JP"
matplotlib.rcParams["axes.unicode_minus"] = False

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_geometry import XODR_PATH, parse_roads, parse_junctions, road_polyline
from geo_calibrate import (
    OSM_RAW_PATH, load_osm_nodes, junction_xodr_locations,
    latlon_to_enu, fit_similarity, apply_transform,
)

FIG_DIR = Path(__file__).resolve().parent.parent / "docs" / "figures"
OUT_PATH = FIG_DIR / "04_overlay.png"

# Phase C-1에서 확정한 정문(junction2)->도서관(junction11) 경로 6개 road
PATH_ROAD_IDS = ["300", "402", "319", "405", "404", "299"]
GATE_JUNCTION_OSM = "2261340221"   # junction2 = 정문
LIB_JUNCTION_OSM = "4402717742"    # junction11 = 중앙도서관 인근
MARGIN_M = 250


def fit_transform():
    root = ET.parse(XODR_PATH).getroot()
    roads = parse_roads(root)
    junctions = parse_junctions(root)
    osm_nodes = load_osm_nodes(OSM_RAW_PATH)
    j_xy = junction_xodr_locations(roads, junctions)

    pairs = []
    for jid, junction in junctions.items():
        name = junction["name"]
        if name in osm_nodes and jid in j_xy:
            lat, lon = osm_nodes[name]
            x, y = j_xy[jid]
            pairs.append((name, x, y, lat, lon))
    ref_lat = sum(p[3] for p in pairs) / len(pairs)
    ref_lon = sum(p[4] for p in pairs) / len(pairs)
    src_xy = [(p[1], p[2]) for p in pairs]
    dst_en = [latlon_to_enu(p[3], p[4], ref_lat, ref_lon) for p in pairs]
    R, s, t, angle_deg, residuals = fit_similarity(src_xy, dst_en)
    return roads, junctions, osm_nodes, R, s, t, ref_lat, ref_lon


def load_osm_ways(path):
    nodes = {}
    ways = []
    cur = None
    for ev, elem in ET.iterparse(path, events=("start", "end")):
        if ev == "start" and elem.tag == "way":
            cur = {"id": elem.get("id"), "nds": [], "tags": {}}
        elif ev == "end" and elem.tag == "node":
            nodes[elem.get("id")] = (float(elem.get("lat")), float(elem.get("lon")))
        elif ev == "end" and elem.tag == "nd" and cur is not None:
            cur["nds"].append(elem.get("ref"))
        elif ev == "end" and elem.tag == "tag" and cur is not None:
            cur["tags"][elem.get("k")] = elem.get("v")
        elif ev == "end" and elem.tag == "way":
            ways.append(cur)
            cur = None
            elem.clear()
    return nodes, ways


def main():
    roads, junctions, osm_nodes, R, s, t, ref_lat, ref_lon = fit_transform()

    # 도로망 전체를 ENU로 변환
    def road_enu_polyline(road):
        pts = road_polyline(road)
        return [apply_transform(R, s, t, x, y) for x, y in pts]

    all_pts = []
    plain_lines = []
    for rid, road in roads.items():
        if road["junction"] != "-1":
            continue
        pts = road_enu_polyline(road)
        plain_lines.append(pts)
        all_pts.extend(pts)

    path_lines = []
    for rid in PATH_ROAD_IDS:
        road = roads.get(rid)
        if road is None:
            continue
        path_lines.append(road_enu_polyline(road))

    xs = [p[0] for p in all_pts]
    ys = [p[1] for p in all_pts]
    xmin, xmax = min(xs) - MARGIN_M, max(xs) + MARGIN_M
    ymin, ymax = min(ys) - MARGIN_M, max(ys) + MARGIN_M

    # OSM 배경(건물 외곽선 + 보행로) - 같은 ref_lat/ref_lon로 ENU 변환
    osm_full_nodes, ways = load_osm_ways(OSM_RAW_PATH)

    def way_enu(way):
        pts = []
        for nid in way["nds"]:
            if nid in osm_full_nodes:
                lat, lon = osm_full_nodes[nid]
                pts.append(latlon_to_enu(lat, lon, ref_lat, ref_lon))
        return pts

    building_polys = []
    footway_lines = []
    for way in ways:
        tags = way["tags"]
        if "building" not in tags and tags.get("highway") != "footway":
            continue
        pts = way_enu(way)
        if not pts:
            continue
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        if not (xmin - 100 <= cx <= xmax + 100 and ymin - 100 <= cy <= ymax + 100):
            continue
        if "building" in tags:
            building_polys.append(pts)
        else:
            footway_lines.append(pts)

    print(f"배경 건물 {len(building_polys)}개, 보행로 {len(footway_lines)}개 (bbox+100m 필터)")

    # 정문/도서관 위경도(원본 OSM 직접조회, 오차0)
    gate_lat, gate_lon = osm_nodes[GATE_JUNCTION_OSM]
    lib_lat, lib_lon = osm_nodes[LIB_JUNCTION_OSM]
    gate_en = latlon_to_enu(gate_lat, gate_lon, ref_lat, ref_lon)
    lib_en = latlon_to_enu(lib_lat, lib_lon, ref_lat, ref_lon)

    fig, ax = plt.subplots(figsize=(11, 13))

    for pts in building_polys:
        poly = Polygon(pts, closed=True, facecolor="0.85", edgecolor="0.7",
                        linewidth=0.4, zorder=1)
        ax.add_patch(poly)
    for pts in footway_lines:
        xs_f = [p[0] for p in pts]
        ys_f = [p[1] for p in pts]
        ax.plot(xs_f, ys_f, color="#a8c8e8", linewidth=0.5, zorder=1, alpha=0.7)

    for pts in plain_lines:
        xs_r = [p[0] for p in pts]
        ys_r = [p[1] for p in pts]
        ax.plot(xs_r, ys_r, color="0.35", linewidth=1.0, zorder=2)

    for pts in path_lines:
        xs_p = [p[0] for p in pts]
        ys_p = [p[1] for p in pts]
        ax.plot(xs_p, ys_p, color="tab:red", linewidth=2.8, zorder=3)

    ax.scatter([gate_en[0]], [gate_en[1]], s=80, c="tab:green", marker="*",
               zorder=4, label="정문(junction2)")
    ax.annotate("정문", gate_en, textcoords="offset points", xytext=(8, 8), fontsize=10)
    ax.scatter([lib_en[0]], [lib_en[1]], s=80, c="tab:blue", marker="*",
               zorder=4, label="중앙도서관(junction11)")
    ax.annotate("중앙도서관", lib_en, textcoords="offset points", xytext=(8, 8), fontsize=10)

    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("동서 (m, ENU east)")
    ax.set_ylabel("남북 (m, ENU north) — 위쪽=진북")
    ax.grid(True, linewidth=0.3, alpha=0.4)
    ax.set_title(
        "cbnu_internal_only_localtm.xodr 도로망 + OSM 배경(건물/보행로)\n"
        "빨강=정문-도서관 경로(539.75m, road 300/402/319/405/404/299)"
    )
    ax.legend(loc="upper right", fontsize=9)

    # 북쪽 화살표
    ax.annotate("N", xy=(0.95, 0.90), xytext=(0.95, 0.80),
                xycoords="axes fraction", textcoords="axes fraction",
                arrowprops=dict(arrowstyle="-|>", color="black", lw=1.5),
                ha="center", fontsize=11)

    fig.tight_layout()
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT_PATH, dpi=150)
    plt.close(fig)
    print(f"저장: {OUT_PATH}")


if __name__ == "__main__":
    main()
