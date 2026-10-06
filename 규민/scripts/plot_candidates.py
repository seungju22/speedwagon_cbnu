#!/usr/bin/env python3
# Phase B-2: road_candidates.get_candidates()의 64건 후보를 번호 라벨과
# highway태그별 색으로 그린다. 기존 도로망은 검은 굵은선, 배경은 건물
# 외곽선(진북 위쪽, 미터 축척) — plot_overlay.py와 같은 좌표계·배경
# 방식을 재사용한다. 64건을 한 장에 넣으면 번호가 겹쳐 안 보이므로
# 후보 중점 좌표 기준 4분면(중앙값 분할 아님 — 분면별 실제 후보
# bbox+여유로 자르므로 각 후보 전체 형상이 항상 온전히 보임)으로
# 나눠 05a~05d 4장을 만든다. 읽기전용(xodr/osm 미수정), PNG만 생성.
import sys
import statistics
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
import plot_overlay as po
from road_candidates import get_candidates
from geo_calibrate import latlon_to_enu

FIG_DIR = Path(__file__).resolve().parent.parent / "docs" / "figures"
MARGIN_M = 80

COLOR_BY_HIGHWAY = {
    "service": "tab:orange",
    "footway": "tab:purple",
    "path": "tab:green",
    "track": "tab:brown",
}
LABEL_BY_HIGHWAY = {
    "service": "service(parking_aisle 등)",
    "footway": "footway",
    "path": "path",
    "track": "track",
}


def candidate_enu_polyline(cand, nodes, ref_lat, ref_lon):
    pts = []
    for nid in cand["nds"]:
        if nid not in nodes:
            continue
        lat, lon = float(nodes[nid]["lat"]), float(nodes[nid]["lon"])
        pts.append(latlon_to_enu(lat, lon, ref_lat, ref_lon))
    return pts


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
    roads, junctions, osm_nodes, R, s, t, ref_lat, ref_lon = po.fit_transform()
    candidates, nodes = get_candidates()
    print(f"후보 총수: {len(candidates)}")

    cand_polys = []
    for c in candidates:
        pts = candidate_enu_polyline(c, nodes, ref_lat, ref_lon)
        if not pts:
            continue
        mx = sum(p[0] for p in pts) / len(pts)
        my = sum(p[1] for p in pts) / len(pts)
        cand_polys.append({**c, "pts": pts, "mx": mx, "my": my})

    mxs = [c["mx"] for c in cand_polys]
    mys = [c["my"] for c in cand_polys]
    med_x = statistics.median(mxs)
    med_y = statistics.median(mys)

    def quadrant(c):
        ew = "E" if c["mx"] >= med_x else "W"
        ns = "N" if c["my"] >= med_y else "S"
        return ns + ew  # NE, NW, SE, SW

    quads = {"NW": [], "NE": [], "SW": [], "SE": []}
    for c in cand_polys:
        quads[quadrant(c)].append(c)
    for q, lst in quads.items():
        print(f"{q}: {len(lst)}건, 번호 {sorted(x['number'] for x in lst)}")

    # 기존 도로망(xodr, junction==-1인 일반road) ENU 폴리라인
    def road_enu_polyline(road):
        pts = po.road_polyline(road)
        return [po.apply_transform(R, s, t, x, y) for x, y in pts]

    existing_lines = []
    for rid, road in roads.items():
        if road["junction"] != "-1":
            continue
        existing_lines.append(road_enu_polyline(road))

    # OSM 배경(건물 외곽선)
    osm_full_nodes, ways = load_osm_ways(po.OSM_RAW_PATH)

    def way_enu(way):
        pts = []
        for nid in way["nds"]:
            if nid in osm_full_nodes:
                lat, lon = osm_full_nodes[nid]
                pts.append(latlon_to_enu(lat, lon, ref_lat, ref_lon))
        return pts

    building_polys = []
    for way in ways:
        if "building" not in way["tags"]:
            continue
        pts = way_enu(way)
        if pts:
            building_polys.append(pts)

    label_suffix = {"NW": "a", "NE": "b", "SW": "c", "SE": "d"}
    for q in ("NW", "NE", "SW", "SE"):
        lst = quads[q]
        if not lst:
            continue
        allpts = [p for c in lst for p in c["pts"]]
        xs = [p[0] for p in allpts]
        ys = [p[1] for p in allpts]
        xmin, xmax = min(xs) - MARGIN_M, max(xs) + MARGIN_M
        ymin, ymax = min(ys) - MARGIN_M, max(ys) + MARGIN_M

        fig, ax = plt.subplots(figsize=(11, 11))

        for pts in building_polys:
            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
            if not (xmin - 50 <= cx <= xmax + 50 and ymin - 50 <= cy <= ymax + 50):
                continue
            ax.add_patch(Polygon(pts, closed=True, facecolor="0.85",
                                  edgecolor="0.7", linewidth=0.4, zorder=1))

        for pts in existing_lines:
            xs_r = [p[0] for p in pts]
            ys_r = [p[1] for p in pts]
            if max(xs_r) < xmin - 50 or min(xs_r) > xmax + 50:
                continue
            if max(ys_r) < ymin - 50 or min(ys_r) > ymax + 50:
                continue
            ax.plot(xs_r, ys_r, color="black", linewidth=2.2, zorder=2)

        seen_labels = set()
        for c in lst:
            color = COLOR_BY_HIGHWAY.get(c["highway"], "tab:gray")
            xs_c = [p[0] for p in c["pts"]]
            ys_c = [p[1] for p in c["pts"]]
            label = LABEL_BY_HIGHWAY.get(c["highway"], c["highway"])
            ax.plot(xs_c, ys_c, color=color, linewidth=2.0, zorder=3,
                     label=label if label not in seen_labels else None)
            seen_labels.add(label)
            ax.annotate(str(c["number"]), (c["mx"], c["my"]),
                        fontsize=8, fontweight="bold", color=color,
                        ha="center", va="center", zorder=4,
                        bbox=dict(boxstyle="circle,pad=0.15", fc="white",
                                  ec=color, lw=0.8, alpha=0.9))

        ax.set_xlim(xmin, xmax)
        ax.set_ylim(ymin, ymax)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("동서 (m, ENU east)")
        ax.set_ylabel("남북 (m, ENU north) — 위쪽=진북")
        ax.grid(True, linewidth=0.3, alpha=0.4)
        nums = sorted(x["number"] for x in lst)
        ax.set_title(
            f"도로 후보 {q}구역 ({len(lst)}건, 번호 {nums[0]}~{nums[-1]} 중 해당분)\n"
            "검은굵은선=기존 도로망(19way) / 색=highway태그"
        )
        ax.legend(loc="upper right", fontsize=9)
        ax.annotate("N", xy=(0.95, 0.90), xytext=(0.95, 0.80),
                    xycoords="axes fraction", textcoords="axes fraction",
                    arrowprops=dict(arrowstyle="-|>", color="black", lw=1.5),
                    ha="center", fontsize=11)

        fig.tight_layout()
        out_path = FIG_DIR / f"05{label_suffix[q]}_candidates_{q}.png"
        FIG_DIR.mkdir(parents=True, exist_ok=True)
        fig.savefig(out_path, dpi=150)
        plt.close(fig)
        print(f"저장: {out_path}")


if __name__ == "__main__":
    main()
