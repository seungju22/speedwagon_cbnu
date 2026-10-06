#!/usr/bin/env python3
# cbnu_internal_only.xodr 도로망 평면도 3장. 읽기전용(xodr 미수정).
# analyze_geometry.py의 파서/기하함수를 재사용한다.
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import matplotlib.font_manager as fm  # noqa: E402
fm.fontManager.addfont("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
matplotlib.rcParams["font.family"] = "Noto Sans CJK JP"  # 한글 글리프 포함(통합 CJK 폰트)
matplotlib.rcParams["axes.unicode_minus"] = False

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_geometry import (  # noqa: E402
    XODR_PATH,
    parse_roads,
    parse_junctions,
    road_polyline,
    road_end_state,
)
import xml.etree.ElementTree as ET  # noqa: E402

FIG_DIR = Path(__file__).resolve().parent.parent / "docs" / "figures"
ROUTE_ROAD_IDS = ["319", "299", "404", "405", "318"]
CAMPUS_NS_LENGTH_M = 1240


def junction_locations(roads, junctions):
    locs = {}
    for jid, junction in junctions.items():
        pts = []
        for conn in junction["connections"]:
            connecting = roads.get(conn["connectingRoad"])
            if connecting:
                pts.append(road_end_state(connecting, "start")[:2])
        if pts:
            locs[jid] = (
                sum(p[0] for p in pts) / len(pts),
                sum(p[1] for p in pts) / len(pts),
            )
    return locs


def draw_network(ax, roads, color="0.6", linewidth=0.8, exclude_ids=None):
    exclude_ids = exclude_ids or set()
    for rid, road in roads.items():
        if road["junction"] != "-1" or rid in exclude_ids:
            continue
        pts = road_polyline(road)
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ax.plot(xs, ys, color=color, linewidth=linewidth, zorder=1)


def draw_junction_markers(ax, locs, size=12):
    xs = [p[0] for p in locs.values()]
    ys = [p[1] for p in locs.values()]
    ax.scatter(xs, ys, s=size, c="tab:orange", marker="o", zorder=3,
               label="junction(25)")


def setup_axes(ax, title):
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True, linewidth=0.3, alpha=0.5)
    ax.set_title(title)


def plot_full_network(roads, junctions, out_path):
    fig, ax = plt.subplots(figsize=(10, 12))
    draw_network(ax, roads)
    locs = junction_locations(roads, junctions)
    draw_junction_markers(ax, locs)

    xs_all = [p[0] for r in roads.values() if r["junction"] == "-1"
              for p in road_polyline(r)]
    ys_all = [p[1] for r in roads.values() if r["junction"] == "-1"
              for p in road_polyline(r)]
    width = max(xs_all) - min(xs_all)
    height = max(ys_all) - min(ys_all)

    setup_axes(
        ax,
        f"cbnu_internal_only.xodr 전체 도로망 (road 134)\n"
        f"bbox {width:.0f}m x {height:.0f}m "
        f"(참고: 캠퍼스 남북 실측 약 {CAMPUS_NS_LENGTH_M}m)",
    )
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_gate_library_segment(roads, out_path):
    fig, ax = plt.subplots(figsize=(10, 12))
    draw_network(ax, roads, exclude_ids=set(ROUTE_ROAD_IDS))
    for rid in ROUTE_ROAD_IDS:
        road = roads.get(rid)
        if road is None:
            continue
        pts = road_polyline(road)
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ax.plot(xs, ys, color="tab:red", linewidth=2.5, zorder=2)
    setup_axes(
        ax,
        "정문-도서관 경로 중 확인된 구간 강조\n"
        f"(road {'/'.join(ROUTE_ROAD_IDS)} 만 — 전체 경로 아님)",
    )
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_junction26_detail(roads, out_path):
    fig, ax = plt.subplots(figsize=(9, 9))
    all_pts = []
    for rid in ROUTE_ROAD_IDS:
        road = roads.get(rid)
        if road is None:
            continue
        pts = road_polyline(road)
        all_pts.extend(pts)
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ax.plot(xs, ys, marker="o", markersize=2, linewidth=2, label=f"road {rid}")
        # 진행방향 화살표: 중간 두 점 사이에 표시
        mid = len(pts) // 2
        if mid + 1 < len(pts):
            x0, y0 = pts[mid]
            x1, y1 = pts[mid + 1]
            ax.annotate(
                "", xy=(x1, y1), xytext=(x0, y0),
                arrowprops=dict(arrowstyle="->", color="black", lw=1.5),
            )

    xs_all = [p[0] for p in all_pts]
    ys_all = [p[1] for p in all_pts]
    margin = 5
    ax.set_xlim(min(xs_all) - margin, max(xs_all) + margin)
    ax.set_ylim(min(ys_all) - margin, max(ys_all) + margin)
    setup_axes(ax, "junction 26(4748296088) 주변 확대\n(화살표=s 증가방향)")
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def main():
    root = ET.parse(XODR_PATH).getroot()
    roads = parse_roads(root)
    junctions = parse_junctions(root)

    FIG_DIR.mkdir(parents=True, exist_ok=True)
    p1 = FIG_DIR / "01_full_network.png"
    p2 = FIG_DIR / "02_gate_library_segment.png"
    p3 = FIG_DIR / "03_junction26_detail.png"

    plot_full_network(roads, junctions, p1)
    plot_gate_library_segment(roads, p2)
    plot_junction26_detail(roads, p3)

    print(p1)
    print(p2)
    print(p3)


if __name__ == "__main__":
    main()
