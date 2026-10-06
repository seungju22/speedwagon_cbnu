#!/usr/bin/env python3
# 맵세션5 Phase A: 06_overlay_expanded.png 육안검증 후속.
# plot_overlay_expanded.py 사본+확장 — 원본은 손대지 않고 보존.
# 수정사항(세션5 지시서 A-1/A-2):
#  1) 범례 "중앙도서관(junction11)" -> 실제 종점 id(junction51)로 정정
#  2) 중앙도서관 "건물"(N12 구관, way442066135) 위치에 종점 junction과
#     구분되는 표식 추가 — 지금까지는 junction 표식만 있어 건물 실제
#     위치가 안 보였음(세션4 종점오류 발견의 원인)
#  3) 정문/중문 OSM 지물 표시(동문/남문은 data/raw/cbnu_campus.osm에
#     해당 태그 자체가 없어 표시 불가 — 아래 GATE_MARKERS 주석 참고)
#  4) 도서관 종점 3안(A=junction51/B=junction35/C=road1278 76.46m지점)
#     전부 표시
# 출력 3장: 07_overlay_review.png(전체), 08_zoom_gate.png(정문 300m),
#          09_zoom_library.png(도서관 300m, 3안 전부)
# 읽기전용(xodr/osm 미수정), 외부 타일 없음.
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

PATH_ROAD_IDS = ["1247", "1914", "1356", "1917", "1355", "1447", "1373",
                  "1673", "1374", "1591", "1375", "1689", "1376", "1625", "1377"]
PATH_LENGTH_M = 529.86
GATE_JUNCTION_OSM = "2261340221"   # junction2 = 정문
LIB_JUNCTION_OSM = "4402717742"    # junction51 = 도서관 종점(A안, 현재)
MARGIN_M = 250

FLAG_JUNCTION_XY_XODR = (517.53695993, 341.5124049225)
FLAG_LABEL = "신규 최대방향차 76.50°\n(junction20, 셔틀경로 아님)"
FLAG_LABEL_OFFSET = (40, -55)

# 도서관 종점 3안(맵세션4 조사, xodr 좌표). C안은 junction이 아니라
# road1278(junction36->35, pred=36/start, succ=35/end) 위의 한 점 —
# junction36쪽에서 76.46m 지점(setup_log.md 2026-09-18 기록).
LIB_BUILDING_WAY_ID = "442066135"  # N12 중앙도서관 구관
LIB_JUNCTION_A_ID = "51"
LIB_JUNCTION_B_ID = "35"
ROAD1278_ID = "1278"
ROAD1278_C_S = 76.46

# 정문/중문 OSM 지물(bus_stop, data/raw/cbnu_campus.osm 실측 확인).
# 동문/남문은 이 OSM 추출 범위 안에 해당 이름 태그의 게이트 지물이
# 없음(남문은 "남문 학생주차장" 주차장 이름만 존재, 게이트 자체 아님;
# 동문은 어떤 태그로도 검색되지 않음) — 다음 세션 재조사 필요 항목.
GATE_MARKERS = [
    ("정문", "6403450088", "tab:green"),
    ("중문", "12324240640", "tab:olive"),
]

BUILDING_LABELS = [
    "자연대 5호관", "충북대 천문대", "충북대학교 박물관",
    "경영대학 본관", "인문사회관", "사회과학대학 본관", "법학관",
    "공학관", "학연산공동기술연구원", "수의과대학 2호관",
    "농업생명과학대학 실험동", "농업과학 기술센터",
    "지선관", "신민관", "명덕관", "등용관",
    "NH관", "예지관",
]

# 확대 그림에서 도로 id를 표기할 주요 road(일반 road만, junction 내부
# connector 제외) - 전부 달면 안 읽히므로 최단경로 위 road와 각 zoom의
# 핵심 분기 road만 선별
LABEL_ROAD_IDS_GATE = {"1247", "1914", "1356"}
LABEL_ROAD_IDS_LIB = {"1591", "1375", "1689", "1376", "1625", "1377", "1278"}


def point_at_s(road, s_target):
    for geom in road["geoms"]:
        if geom["s"] - 1e-6 <= s_target <= geom["s"] + geom["length"] + 1e-6:
            p = (s_target - geom["s"]) / geom["length"] if geom["length"] > 0 else 0.0
            return geom_point(geom, p)
    geom = road["geoms"][-1]
    return geom_point(geom, 1.0)


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
    print(f"[{xodr_path.name}] 보정점 매칭 {len(pairs)}/{len(junctions)}, "
          f"회전={angle_deg:.3f}deg 스케일={s:.6f}")
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


def draw_background(ax, building_polys, footway_lines, plain_lines):
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
        ax.plot(xs_r, ys_r, color="0.35", linewidth=0.8, zorder=2)


def main():
    xmin, xmax, ymin, ymax = old_axis_bounds()
    print(f"세션3 축척·범위 재사용: x[{xmin:.1f},{xmax:.1f}] y[{ymin:.1f},{ymax:.1f}]")

    roads, junctions, osm_nodes, R, s, t, ref_lat, ref_lon = fit_transform_for(NEW_XODR)
    j_xy = junction_xodr_locations(roads, junctions)

    def road_enu_polyline(road):
        return [apply_transform(R, s, t, x, y) for x, y in road_polyline(road)]

    def xodr_to_enu(x, y):
        return apply_transform(R, s, t, x, y)

    plain_lines = {}
    for rid, road in roads.items():
        if road["junction"] != "-1":
            continue
        plain_lines[rid] = road_enu_polyline(road)

    path_lines = []
    for rid in PATH_ROAD_IDS:
        road = roads.get(rid)
        if road is None:
            print(f"경고: path road {rid} 신규판에 없음")
            continue
        path_lines.append(road_enu_polyline(road))

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
    label_targets = {name: [] for name in BUILDING_LABELS}
    lib_building_en = None
    for way in ways:
        tags = way["tags"]
        wname = tags.get("name", "")
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
            for label in BUILDING_LABELS:
                if label in wname:
                    label_targets[label].append((cx, cy))
        else:
            footway_lines.append(pts)

    found_labels = {k: v[0] for k, v in label_targets.items() if v}
    print(f"라벨 매칭 {len(found_labels)}/{len(BUILDING_LABELS)}건")
    print(f"도서관 건물(way{LIB_BUILDING_WAY_ID}) centroid ENU: {lib_building_en}")

    gate_lat, gate_lon = osm_nodes[GATE_JUNCTION_OSM]
    lib_lat, lib_lon = osm_nodes[LIB_JUNCTION_OSM]
    gate_en = latlon_to_enu(gate_lat, gate_lon, ref_lat, ref_lon)
    lib_a_en = latlon_to_enu(lib_lat, lib_lon, ref_lat, ref_lon)
    flag_en = xodr_to_enu(*FLAG_JUNCTION_XY_XODR)

    lib_b_en = xodr_to_enu(*j_xy[LIB_JUNCTION_B_ID])
    road1278 = roads[ROAD1278_ID]
    lib_c_xodr = point_at_s(road1278, ROAD1278_C_S)
    lib_c_en = xodr_to_enu(*lib_c_xodr)

    gate_markers_en = {}
    for label, nid, color in GATE_MARKERS:
        if nid in osm_full_nodes:
            lat, lon = osm_full_nodes[nid]
            gate_markers_en[label] = (latlon_to_enu(lat, lon, ref_lat, ref_lon), color)
        else:
            print(f"경고: {label} 노드 {nid} OSM에 없음")
    print("동문/남문: OSM 데이터에 해당 게이트 지물 없음(표시 불가, "
          "남문은 '남문 학생주차장' 주차장 이름만 존재)")

    # ---- 그림1: 전체 리뷰(07_overlay_review.png) ----
    fig, ax = plt.subplots(figsize=(11, 13))
    draw_background(ax, building_polys, footway_lines, list(plain_lines.values()))
    for pts in path_lines:
        ax.plot([p[0] for p in pts], [p[1] for p in pts],
                color="tab:red", linewidth=2.8, zorder=3)

    ax.scatter([gate_en[0]], [gate_en[1]], s=80, c="tab:green", marker="*",
               zorder=4, label="정문 junction(junction2)")
    ax.annotate("정문junction", gate_en, textcoords="offset points", xytext=(8, 8), fontsize=9)

    ax.scatter([lib_a_en[0]], [lib_a_en[1]], s=80, c="tab:blue", marker="*",
               zorder=4, label="종점A: junction51(현재)")
    ax.annotate("종점A(junction51)", lib_a_en, textcoords="offset points",
                xytext=(8, 8), fontsize=9)

    if lib_building_en:
        ax.scatter([lib_building_en[0]], [lib_building_en[1]], s=90, c="crimson",
                   marker="s", zorder=5, label="중앙도서관 건물(N12 구관)")
        ax.annotate("도서관 건물", lib_building_en, textcoords="offset points",
                    xytext=(8, -14), fontsize=9, color="crimson")

    ax.scatter([lib_b_en[0]], [lib_b_en[1]], s=60, c="tab:purple", marker="^",
               zorder=4, label="종점B: junction35")
    ax.scatter([lib_c_en[0]], [lib_c_en[1]], s=60, c="tab:cyan", marker="D",
               zorder=4, label="종점C: road1278 위 지점")

    for label, (en, color) in gate_markers_en.items():
        ax.scatter([en[0]], [en[1]], s=60, c=color, marker="P", zorder=4,
                   label=f"{label}(OSM)")
        ax.annotate(label, en, textcoords="offset points", xytext=(6, 6), fontsize=8)

    ax.scatter([flag_en[0]], [flag_en[1]], s=70, c="tab:orange", marker="X", zorder=4,
               label="신규 방향차 최댓값(76.50°)")
    ax.annotate(FLAG_LABEL, flag_en, textcoords="offset points", xytext=FLAG_LABEL_OFFSET,
                fontsize=8, color="tab:orange",
                arrowprops=dict(arrowstyle="-", color="tab:orange", lw=0.8))

    for name, (lx, ly) in found_labels.items():
        ax.plot(lx, ly, marker=".", markersize=2, color="0.3", zorder=3)
        ax.annotate(name, (lx, ly), textcoords="offset points", xytext=(3, 3),
                    fontsize=7, color="0.2", zorder=5)

    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("동서 (m, ENU east)")
    ax.set_ylabel("남북 (m, ENU north) - 위쪽=진북")
    ax.grid(True, linewidth=0.3, alpha=0.4)
    ax.set_title(
        "맵세션5 육안재확인용(07) — 73건 태그보정판 + OSM 배경\n"
        f"빨강=정문-종점A 경로({PATH_LENGTH_M}m) / 종점A·B·C·도서관건물 구분표시\n"
        "동문·남문은 OSM에 게이트 지물 없어 미표시"
    )
    ax.legend(loc="upper right", fontsize=7)
    ax.annotate("N", xy=(0.95, 0.90), xytext=(0.95, 0.80),
                xycoords="axes fraction", textcoords="axes fraction",
                arrowprops=dict(arrowstyle="-|>", color="black", lw=1.5),
                ha="center", fontsize=11)
    fig.tight_layout()
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    out1 = FIG_DIR / "07_overlay_review.png"
    fig.savefig(out1, dpi=150)
    plt.close(fig)
    print(f"저장: {out1}")

    # ---- 그림2: 정문 확대(08_zoom_gate.png, 반경 300m) ----
    zoom_gate(gate_en, roads, plain_lines, path_lines, building_polys, footway_lines,
              gate_markers_en, found_labels, R, s, t)

    # ---- 그림3: 도서관 확대(09_zoom_library.png, 반경 300m, 3안 전부) ----
    zoom_library(lib_a_en, lib_b_en, lib_c_en, lib_building_en, roads, plain_lines,
                 path_lines, building_polys, footway_lines, found_labels)


def bbox_filter_lines(lines_dict, cx, cy, radius, label_ids):
    kept = []
    labels = []
    for rid, pts in lines_dict.items():
        if any(abs(p[0] - cx) <= radius and abs(p[1] - cy) <= radius for p in pts):
            kept.append(pts)
            if rid in label_ids:
                mid = pts[len(pts) // 2]
                labels.append((rid, mid))
    return kept, labels


def bbox_filter_polys(polys, cx, cy, radius):
    return [pts for pts in polys
            if any(abs(p[0] - cx) <= radius and abs(p[1] - cy) <= radius for p in pts)]


def zoom_gate(gate_en, roads, plain_lines, path_lines, building_polys, footway_lines,
              gate_markers_en, found_labels, R, s, t):
    cx, cy = gate_en
    radius = 300
    kept_lines, road_labels = bbox_filter_lines(plain_lines, cx, cy, radius,
                                                 LABEL_ROAD_IDS_GATE)
    kept_buildings = bbox_filter_polys(building_polys, cx, cy, radius)
    kept_footways = bbox_filter_polys(footway_lines, cx, cy, radius)

    fig, ax = plt.subplots(figsize=(8, 8))
    draw_background(ax, kept_buildings, kept_footways, kept_lines)
    for pts in path_lines:
        if any(abs(p[0] - cx) <= radius and abs(p[1] - cy) <= radius for p in pts):
            ax.plot([p[0] for p in pts], [p[1] for p in pts],
                    color="tab:red", linewidth=2.8, zorder=3)

    ax.scatter([gate_en[0]], [gate_en[1]], s=100, c="tab:green", marker="*",
               zorder=5, label="정문 junction(junction2)")
    ax.annotate("정문junction", gate_en, textcoords="offset points", xytext=(8, 8), fontsize=9)
    for label, (en, color) in gate_markers_en.items():
        ax.scatter([en[0]], [en[1]], s=70, c=color, marker="P", zorder=5, label=f"{label}(OSM)")
        ax.annotate(label, en, textcoords="offset points", xytext=(6, 6), fontsize=8)

    for name, (lx, ly) in found_labels.items():
        if abs(lx - cx) <= radius and abs(ly - cy) <= radius:
            ax.plot(lx, ly, marker=".", markersize=2, color="0.3", zorder=3)
            ax.annotate(name, (lx, ly), textcoords="offset points", xytext=(3, 3),
                        fontsize=7, color="0.2", zorder=5)

    for rid, (mx, my) in road_labels:
        ax.annotate(f"road{rid}", (mx, my), fontsize=7, color="tab:blue", zorder=6,
                    weight="bold")

    ax.set_xlim(cx - radius, cx + radius)
    ax.set_ylim(cy - radius, cy + radius)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("동서 (m)")
    ax.set_ylabel("남북 (m) - 위쪽=진북")
    ax.grid(True, linewidth=0.3, alpha=0.4)
    ax.set_title("정문 주변 300m 확대(08)\n주요 road id 일부만 표기")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    out = FIG_DIR / "08_zoom_gate.png"
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"저장: {out}")


def zoom_library(lib_a_en, lib_b_en, lib_c_en, lib_building_en, roads, plain_lines,
                  path_lines, building_polys, footway_lines, found_labels):
    cx = (lib_a_en[0] + lib_b_en[0] + lib_c_en[0]) / 3
    cy = (lib_a_en[1] + lib_b_en[1] + lib_c_en[1]) / 3
    radius = 300
    kept_lines, road_labels = bbox_filter_lines(plain_lines, cx, cy, radius,
                                                 LABEL_ROAD_IDS_LIB)
    kept_buildings = bbox_filter_polys(building_polys, cx, cy, radius)
    kept_footways = bbox_filter_polys(footway_lines, cx, cy, radius)

    fig, ax = plt.subplots(figsize=(8, 8))
    draw_background(ax, kept_buildings, kept_footways, kept_lines)
    for pts in path_lines:
        if any(abs(p[0] - cx) <= radius and abs(p[1] - cy) <= radius for p in pts):
            ax.plot([p[0] for p in pts], [p[1] for p in pts],
                    color="tab:red", linewidth=2.8, zorder=3)

    if lib_building_en:
        ax.scatter([lib_building_en[0]], [lib_building_en[1]], s=110, c="crimson",
                   marker="s", zorder=5, label="중앙도서관 건물(N12 구관)")
        ax.annotate("도서관 건물", lib_building_en, textcoords="offset points",
                    xytext=(8, -14), fontsize=9, color="crimson")

    ax.scatter([lib_a_en[0]], [lib_a_en[1]], s=90, c="tab:blue", marker="*",
               zorder=4, label="종점A: junction51(현재, 118.7m)")
    ax.annotate("A", lib_a_en, textcoords="offset points", xytext=(8, 8), fontsize=10,
                color="tab:blue", weight="bold")
    ax.scatter([lib_b_en[0]], [lib_b_en[1]], s=90, c="tab:purple", marker="^",
               zorder=4, label="종점B: junction35(87.5m)")
    ax.annotate("B", lib_b_en, textcoords="offset points", xytext=(8, 8), fontsize=10,
                color="tab:purple", weight="bold")
    ax.scatter([lib_c_en[0]], [lib_c_en[1]], s=90, c="tab:cyan", marker="D",
               zorder=4, label="종점C: road1278 위(65.4m)")
    ax.annotate("C", lib_c_en, textcoords="offset points", xytext=(8, 8), fontsize=10,
                color="tab:cyan", weight="bold")

    for name, (lx, ly) in found_labels.items():
        if abs(lx - cx) <= radius and abs(ly - cy) <= radius:
            ax.plot(lx, ly, marker=".", markersize=2, color="0.3", zorder=3)
            ax.annotate(name, (lx, ly), textcoords="offset points", xytext=(3, 3),
                        fontsize=7, color="0.2", zorder=5)

    for rid, (mx, my) in road_labels:
        ax.annotate(f"road{rid}", (mx, my), fontsize=7, color="tab:blue", zorder=6,
                    weight="bold")

    ax.set_xlim(cx - radius, cx + radius)
    ax.set_ylim(cy - radius, cy + radius)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("동서 (m)")
    ax.set_ylabel("남북 (m) - 위쪽=진북")
    ax.grid(True, linewidth=0.3, alpha=0.4)
    ax.set_title("중앙도서관 주변 300m 확대(09)\n종점 3안(A/B/C) 전부 표시, road id 일부 표기")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    out = FIG_DIR / "09_zoom_library.png"
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"저장: {out}")


if __name__ == "__main__":
    main()
