#!/usr/bin/env python3
# Phase D(맵세션4, 2026-09-18): 73건 태그보정판
# (cbnu_internal_only_localtm_tags73.xodr) 오버레이. plot_overlay.py의
# 사본 — 원본은 19건판(대조군) 재현용으로 그대로 보존, 손대지 않음.
# 읽기전용(xodr/osm 미수정), 외부 타일 없음.
#
# 좌표계 주의(사용자 지시): 73건판은 osm2odr center_map 재중심으로
# 19건판과 (dx=86.50,dy=70.54) 평행이동돼 있다(맵세션4 확인 기록,
# setup_log.md 2026-09-18). 이 스크립트는 수동 오프셋 보정을 쓰지
# 않는다 — geo_calibrate.fit_similarity가 "이 xodr 파일 자신의
# junction 위치" <-> "원본 OSM 위경도"를 매번 새로 적합(Umeyama)하므로
# 어느 xodr을 넣어도 결과 ENU 좌표는 항상 같은 실세계 기준으로 나온다.
# 도로망(xodr 기준)과 배경(OSM 위경도 기준)이 같은 ref_lat/ref_lon로
# 변환되는 한 어긋나지 않는다 — 아래 main()에서 gate_en 마커와 도로망이
# 실제로 겹치는지 점으로 검증한다.
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
from analyze_geometry import parse_roads, parse_junctions, road_polyline
from geo_calibrate import (
    OSM_RAW_PATH, load_osm_nodes, junction_xodr_locations,
    latlon_to_enu, fit_similarity, apply_transform,
)

BASE_DIR = Path(__file__).resolve().parent.parent
OLD_XODR = BASE_DIR / "maps" / "cbnu_internal_only_localtm.xodr"
NEW_XODR = BASE_DIR / "maps" / "cbnu_internal_only_localtm_tags73.xodr"
FIG_DIR = BASE_DIR / "docs" / "figures"
OUT_PATH = FIG_DIR / "06_overlay_expanded.png"

# 맵세션4 확인(setup_log.md 2026-09-18)에서 다이크스트라로 재계산한
# 정문(junction2)->도서관(junction11) 경로, 73건판 road id 15개,
# 529.86m. 옛 6-road 경로(300/402/319/405/404/299, 539.75m)와
# 물리적으로 같은 구간이며 신규 junction 5개가 끼어들며 세분화됨
# (junction26=name4748296088을 포함해 총 9개 junction 통과).
PATH_ROAD_IDS = ["1247", "1914", "1356", "1917", "1355", "1447", "1373",
                  "1673", "1374", "1591", "1375", "1689", "1376", "1625", "1377"]
PATH_LENGTH_M = 529.86
GATE_JUNCTION_OSM = "2261340221"   # junction2 = 정문
LIB_JUNCTION_OSM = "4402717742"    # junction11 = 중앙도서관 인근
MARGIN_M = 250

# 맵세션4 heading_diff 최댓값(76.50°) 지점 — junction26과 별개,
# 신규id20(name=4397573144). way442065044(신규54건번호5, 자연대
# 5호관 인근)와 way392632036(기존16건) 교차. 셔틀경로(위 15road)
# 위에 없음(최단거리 402.59m) — setup_log.md 확인 기록.
FLAG_JUNCTION_XY_XODR = (517.53695993, 341.5124049225)
FLAG_LABEL = "신규 최대방향차 76.50°\n(junction20, 셔틀경로 아님)"
FLAG_LABEL_OFFSET = (40, -55)

# 주요 건물 라벨(20개 이내). road_candidates.md 등장 순 + 사용자가
# 언급한 농대권역/S17 기숙사권역을 반드시 포함하도록 큐레이션.
# 자연대 5호관은 위 FLAG 지점 바로 옆이라 맥락 확인용으로 포함.
BUILDING_LABELS = [
    "자연대 5호관", "충북대 천문대", "충북대학교 박물관",
    "경영대학 본관", "인문사회관", "사회과학대학 본관", "법학관",
    "공학관", "학연산공동기술연구원", "수의과대학 2호관",
    "농업생명과학대학 실험동", "농업과학 기술센터",
    "지선관", "신민관", "명덕관", "등용관",
    "NH관", "예지관",
]


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
          f"회전={angle_deg:.3f}deg 스케일={s:.6f} "
          f"잔차RMS={ (sum(r**2 for r in residuals)/len(residuals))**0.5 :.3f}m")
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
    # 세션3 04_overlay.png와 정확히 같은 축척·범위를 쓰기 위해
    # 옛(19건판) 도로망 bbox+MARGIN_M을 그대로 재현한다.
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
    print(f"세션3 축척·범위 재사용: x[{xmin:.1f},{xmax:.1f}] y[{ymin:.1f},{ymax:.1f}]")

    roads, junctions, osm_nodes, R, s, t, ref_lat, ref_lon = fit_transform_for(NEW_XODR)

    def road_enu_polyline(road):
        pts = road_polyline(road)
        return [apply_transform(R, s, t, x, y) for x, y in pts]

    plain_lines = []
    for rid, road in roads.items():
        if road["junction"] != "-1":
            continue
        plain_lines.append(road_enu_polyline(road))

    path_lines = []
    for rid in PATH_ROAD_IDS:
        road = roads.get(rid)
        if road is None:
            print(f"경고: path road {rid} 신규판에 없음")
            continue
        path_lines.append(road_enu_polyline(road))

    # OSM 배경(건물외곽선+보행로) - 같은 ref_lat/ref_lon로 ENU 변환
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
    for way in ways:
        tags = way["tags"]
        wname = tags.get("name", "")
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

    print(f"배경 건물 {len(building_polys)}개, 보행로 {len(footway_lines)}개 (bbox+100m 필터)")
    found_labels = {k: v[0] for k, v in label_targets.items() if v}
    print(f"라벨 매칭 {len(found_labels)}/{len(BUILDING_LABELS)}건: "
          f"{[k for k in BUILDING_LABELS if k not in found_labels]} 미매칭")

    gate_lat, gate_lon = osm_nodes[GATE_JUNCTION_OSM]
    lib_lat, lib_lon = osm_nodes[LIB_JUNCTION_OSM]
    gate_en = latlon_to_enu(gate_lat, gate_lon, ref_lat, ref_lon)
    lib_en = latlon_to_enu(lib_lat, lib_lon, ref_lat, ref_lon)
    flag_en = apply_transform(R, s, t, *FLAG_JUNCTION_XY_XODR)

    # 정합성 자체점검: gate_en(OSM 위경도 기반)이 실제 도로망(junction2)
    # 근처에 있는지 — 어긋나면(>5m) 배경/도로망 좌표계 불일치 경고
    gate_jid = [jid for jid, j in junctions.items() if j["name"] == GATE_JUNCTION_OSM]
    if gate_jid:
        gx, gy = junction_xodr_locations(roads, junctions)[gate_jid[0]]
        gate_road_en = apply_transform(R, s, t, gx, gy)
        d = ((gate_road_en[0]-gate_en[0])**2 + (gate_road_en[1]-gate_en[1])**2) ** 0.5
        print(f"정합성 점검: 정문 OSM위경도 vs 도로망junction2 거리={d:.3f}m "
              f"({'정상' if d < 5 else '경고: 어긋남'})")

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
        ax.plot(xs_r, ys_r, color="0.35", linewidth=0.8, zorder=2)

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
    ax.set_ylabel("남북 (m, ENU north) — 위쪽=진북")
    ax.grid(True, linewidth=0.3, alpha=0.4)
    ax.set_title(
        "cbnu_internal_only_localtm_tags73.xodr 도로망(73건 태그보정) + OSM 배경\n"
        f"빨강=정문-도서관 경로({PATH_LENGTH_M}m, road 15개, junction 9개 경유)\n"
        "축척·범위는 세션3 04_overlay.png(19건판)와 동일"
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


if __name__ == "__main__":
    main()
