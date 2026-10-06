#!/usr/bin/env python3
# 맵세션5 Phase B-3. 확정 종점(사용자 승인, 2026-09-19)으로 오버레이
# 갱신 — road1278 s=100.38m 지점(구관-사회과학대 방향 출입구 앞,
# 출입구까지 16.68m). plot_session5_review.py의 계산 로직 재사용,
# 이번엔 A/B/C 3안 비교가 아니라 확정 경로 1개만 강조.
# 정문(junction2) -> 확정종점, 14road(마지막 road1278은 s=100.38까지
# 부분경로), 768.26m, junction9개 경유(1/2/10/35/36/42/60/84/107).
# 읽기전용(xodr/osm 미수정), 새 출력 파일(10_overlay_confirmed.png,
# 기존 06/07/08/09 유지).
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
from analyze_geometry import parse_roads, parse_junctions, road_polyline, geom_point
from geo_calibrate import (
    OSM_RAW_PATH, load_osm_nodes, junction_xodr_locations,
    latlon_to_enu, fit_similarity, apply_transform,
)

BASE_DIR = Path(__file__).resolve().parent.parent
OLD_XODR = BASE_DIR / "maps" / "cbnu_internal_only_localtm.xodr"
NEW_XODR = BASE_DIR / "maps" / "cbnu_internal_only_localtm_tags73.xodr"
FIG_DIR = BASE_DIR / "docs" / "figures"
OUT_PATH = FIG_DIR / "10_overlay_confirmed.png"

CONFIRMED_PATH_FULL = ["1247", "1914", "1356", "1917", "1355", "1446", "1172",
                        "1428", "1240", "1695", "1239", "1426", "1238"]
LAST_ROAD_ID = "1278"
LAST_ROAD_S_LIMIT = 100.38
CONFIRMED_LENGTH_M = 768.26
CONFIRMED_JUNCTIONS = ["1", "2", "10", "35", "36", "42", "60", "84", "107"]
GATE_JUNCTION_OSM = "2261340221"
LIB_BUILDING_WAY_ID = "442066135"
MARGIN_M = 250


def fit_transform_for(xodr_path):
    root = ET.parse(xodr_path).getroot()
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


def partial_polyline(road, s_limit, curve_samples=6):
    pts = []
    for geom in road["geoms"]:
        g_start = geom["s"]
        g_end = geom["s"] + geom["length"]
        if g_start >= s_limit:
            break
        local_limit = min(geom["length"], s_limit - g_start)
        p_limit = local_limit / geom["length"] if geom["length"] > 0 else 0.0
        if geom["type"] == "line":
            pts.append(geom_point(geom, 0.0))
            pts.append(geom_point(geom, p_limit))
        else:
            n = max(1, int(curve_samples * p_limit))
            for i in range(n + 1):
                pts.append(geom_point(geom, (i / n) * p_limit if n else 0.0))
    return pts


def old_axis_bounds():
    roads, junctions, osm_nodes, R, s, t, ref_lat, ref_lon = fit_transform_for(OLD_XODR)
    all_pts = []
    for rid, road in roads.items():
        if road["junction"] != "-1":
            continue
        pts = road_polyline(road)
        all_pts.extend(apply_transform(R, s, t, x, y) for x, y in pts)
    xs = [p[0] for p in all_pts]
    ys = [p[1] for p in all_pts]
    return (min(xs) - MARGIN_M, max(xs) + MARGIN_M,
            min(ys) - MARGIN_M, max(ys) + MARGIN_M)


def main():
    xmin, xmax, ymin, ymax = old_axis_bounds()
    roads, junctions, osm_nodes, R, s, t, ref_lat, ref_lon = fit_transform_for(NEW_XODR)

    def road_enu_polyline(road):
        return [apply_transform(R, s, t, x, y) for x, y in road_polyline(road)]

    plain_lines = []
    for rid, road in roads.items():
        if road["junction"] != "-1":
            continue
        plain_lines.append(road_enu_polyline(road))

    path_lines = []
    for rid in CONFIRMED_PATH_FULL:
        road = roads.get(rid)
        if road is None:
            print(f"경고: path road {rid} 없음")
            continue
        path_lines.append(road_enu_polyline(road))
    last_road = roads[LAST_ROAD_ID]
    last_partial = [apply_transform(R, s, t, x, y)
                     for x, y in partial_polyline(last_road, LAST_ROAD_S_LIMIT)]
    path_lines.append(last_partial)
    endpoint_en = last_partial[-1]

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
    lib_building_en = None
    for way in ways:
        tags = way["tags"]
        if way["id"] == LIB_BUILDING_WAY_ID:
            pts = way_enu(way)
            if pts:
                lib_building_en = (sum(p[0] for p in pts) / len(pts),
                                    sum(p[1] for p in pts) / len(pts))
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

    gate_lat, gate_lon = osm_nodes[GATE_JUNCTION_OSM]
    gate_en = latlon_to_enu(gate_lat, gate_lon, ref_lat, ref_lon)

    fig, ax = plt.subplots(figsize=(11, 13))
    for pts in building_polys:
        ax.add_patch(Polygon(pts, closed=True, facecolor="0.85", edgecolor="0.7",
                              linewidth=0.4, zorder=1))
    for pts in footway_lines:
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color="#a8c8e8",
                 linewidth=0.5, zorder=1, alpha=0.7)
    for pts in plain_lines:
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color="0.35",
                 linewidth=0.8, zorder=2)
    for pts in path_lines:
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color="tab:red",
                 linewidth=2.8, zorder=3)

    ax.scatter([gate_en[0]], [gate_en[1]], s=90, c="tab:green", marker="*",
               zorder=4, label="정문(junction2)")
    ax.annotate("정문", gate_en, textcoords="offset points", xytext=(8, 8), fontsize=10)

    ax.scatter([endpoint_en[0]], [endpoint_en[1]], s=100, c="tab:red", marker="*",
               zorder=5, label="확정 종점(road1278 s=100.38m)")
    ax.annotate("확정 종점\n(구관 사회과학대 방향\n출입구 앞, 16.68m)", endpoint_en,
                textcoords="offset points", xytext=(12, -30), fontsize=8, color="tab:red")

    if lib_building_en:
        ax.scatter([lib_building_en[0]], [lib_building_en[1]], s=90, c="crimson",
                   marker="s", zorder=4, label="중앙도서관 건물(N12 구관)")
        ax.annotate("도서관 건물", lib_building_en, textcoords="offset points",
                    xytext=(8, -14), fontsize=9, color="crimson")

    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("동서 (m, ENU east)")
    ax.set_ylabel("남북 (m, ENU north) - 위쪽=진북")
    ax.grid(True, linewidth=0.3, alpha=0.4)
    ax.set_title(
        "맵세션5 확정 종점(10) — cbnu_internal_only_localtm_tags73.xodr\n"
        f"빨강=정문-확정종점 경로({CONFIRMED_LENGTH_M}m, road14개, "
        f"junction{len(CONFIRMED_JUNCTIONS)}개 경유)\n"
        "종점=출입구 최근접(건물 중심점 기준 폐기, 세션5 변경)"
    )
    ax.legend(loc="upper right", fontsize=8)
    ax.annotate("N", xy=(0.95, 0.90), xytext=(0.95, 0.80),
                xycoords="axes fraction", textcoords="axes fraction",
                arrowprops=dict(arrowstyle="-|>", color="black", lw=1.5),
                ha="center", fontsize=11)
    fig.tight_layout()
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT_PATH, dpi=150)
    plt.close(fig)
    print(f"저장: {OUT_PATH}")
    print(f"확정종점 ENU: {endpoint_en}")


if __name__ == "__main__":
    main()
