#!/usr/bin/env python3
# 맵세션16. 완주 경로를 실제 지도(OSM 표준 타일) 위에 그리고, 원본 OSM 도로망의
# 정문->종점 최단 경로와 비교한다. 읽기전용(xodr/osm/로그 미수정).
# 출력: map/docs/figures/22_route_realmap.png, map/docs/logs/session16_route_realmap.log
#
# 좌표: CARLA (x, y) -> xodr (x, -y) -> TM (xodr - header offset) -> 로컬 TM(lat_0/lon_0, xodr geoReference) 역변환.
#   TM 역변환은 원점 근처 국소 근사(WGS84 자오선·묘유선 곡률반경). 1km 안 오차 cm 수준.
# 비교 경로(원본 OSM, oneway 무시):
#   car = highway 중 footway/path/steps/cycleway/corridor/construction/track/rest_area 제외
#   all = highway 전체(보행로 포함)
# 타일: https://tile.openstreetmap.org, 캐시 scratchpad. (c) OpenStreetMap contributors
import csv
import math
import sys
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager as fm  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import networkx as nx  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

fm.fontManager.addfont("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
matplotlib.rcParams["font.family"] = "Noto Sans CJK JP"
matplotlib.rcParams["axes.unicode_minus"] = False

BASE = Path(__file__).resolve().parent.parent
OSM_RAW = BASE / "data" / "raw" / "cbnu_campus.osm"
OSM_USED = BASE / "data" / "processed" / "cbnu_internal_boundary_tags73_smooth.osm"
LOG_DIR = BASE / "docs" / "logs"
OUT_PNG = BASE / "docs" / "figures" / "22_route_realmap.png"
OUT_LOG = LOG_DIR / "session16_route_realmap.log"
DRIVE_LOG = LOG_DIR / (sys.argv[1] if len(sys.argv) > 1 else "drive_log_20260926_144047.csv")
TILE_DIR = Path(sys.argv[2]) if len(sys.argv) > 2 else Path("/tmp/osm_tiles")

LAT0, LON0 = 36.627298, 127.456394        # xodr geoReference +lat_0 +lon_0
OFF_X, OFF_Y = 521.51, 493.61             # xodr header <offset> (xodr = TM + offset)
LIB_WAY = "442066135"                     # 도서관 건물 way (세션5)
ZOOM = 17
UA = "campus_mobility_sim-study/1.0"
NOT_CAR = {"footway", "path", "steps", "cycleway", "corridor", "construction",
           "track", "rest_area"}
SHARP = {"r1446 +78.6°": 1446, "r1428 -68.5°": 1428, "r1564 -82.2°": 1564}

A, F = 6378137.0, 1 / 298.257223563
E2 = F * (2 - F)
_s = math.sin(math.radians(LAT0))
M_R = A * (1 - E2) / (1 - E2 * _s * _s) ** 1.5   # 자오선 곡률반경
N_R = A / math.sqrt(1 - E2 * _s * _s)             # 묘유선 곡률반경
COS0 = math.cos(math.radians(LAT0))


def tm_to_ll(x, y):
    return LAT0 + math.degrees(y / M_R), LON0 + math.degrees(x / (N_R * COS0))


def ll_to_tm(lat, lon):
    return math.radians(lon - LON0) * N_R * COS0, math.radians(lat - LAT0) * M_R


def ll_to_px(lat, lon, z=ZOOM):
    n = 256 * 2 ** z
    x = (lon + 180) / 360 * n
    y = (1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * n
    return x, y


def load_osm(path):
    root = ET.parse(path).getroot()
    nodes = {n.get("id"): (float(n.get("lat")), float(n.get("lon"))) for n in root.iter("node")}
    ways = {}
    for w in root.iter("way"):
        tags = {t.get("k"): t.get("v") for t in w.iter("tag")}
        ways[w.get("id")] = ([nd.get("ref") for nd in w.iter("nd")], tags)
    return nodes, ways


def build_graph(nodes, ways, car_only):
    g = nx.Graph()
    for wid, (nds, tags) in ways.items():
        hw = tags.get("highway")
        if not hw or (car_only and hw in NOT_CAR):
            continue
        for a, b in zip(nds, nds[1:]):
            if a in nodes and b in nodes:
                (xa, ya), (xb, yb) = ll_to_tm(*nodes[a]), ll_to_tm(*nodes[b])
                g.add_edge(a, b, w=math.hypot(xb - xa, yb - ya), way=wid)
    return g


def nearest(g, nodes, x, y):
    return min(g.nodes, key=lambda n: math.dist(ll_to_tm(*nodes[n]), (x, y)))


def fetch_tiles(px):
    x0, y0 = (px.min(0) // 256).astype(int)
    x1, y1 = (px.max(0) // 256).astype(int)
    TILE_DIR.mkdir(parents=True, exist_ok=True)
    mosaic = Image.new("RGB", ((x1 - x0 + 1) * 256, (y1 - y0 + 1) * 256))
    for tx in range(x0, x1 + 1):
        for ty in range(y0, y1 + 1):
            f = TILE_DIR / f"{ZOOM}_{tx}_{ty}.png"
            if not f.exists():
                req = urllib.request.Request(
                    f"https://tile.openstreetmap.org/{ZOOM}/{tx}/{ty}.png",
                    headers={"User-Agent": UA})
                f.write_bytes(urllib.request.urlopen(req, timeout=20).read())
            mosaic.paste(Image.open(f).convert("RGB"), ((tx - x0) * 256, (ty - y0) * 256))
    return mosaic, x0 * 256, y0 * 256, (x1 - x0 + 1) * (y1 - y0 + 1)


def main():
    out = []
    p = lambda s: (print(s), out.append(s))  # noqa: E731

    rows = list(csv.DictReader(open(DRIVE_LOG)))
    traj_tm = np.array([(float(r["x"]) - OFF_X, -float(r["y"]) - OFF_Y)
                        for r in rows])                   # CARLA -> xodr -> TM
    traj_ll = np.array([tm_to_ll(x, y) for x, y in traj_tm])
    roads = np.array([int(r["road_id"]) for r in rows])
    drive_len = np.hypot(*np.diff(traj_tm, axis=0).T).sum()

    nodes, ways = load_osm(OSM_RAW)
    _, used_ways = load_osm(OSM_USED)
    lib_nds = [nodes[n] for n in ways[LIB_WAY][0] if n in nodes]

    (sx, sy), (ex, ey) = traj_tm[0], traj_tm[-1]
    paths = {}
    for key, car in (("car", True), ("all", False)):
        g = build_graph(nodes, ways, car)
        a, b = nearest(g, nodes, sx, sy), nearest(g, nodes, ex, ey)
        path = nx.shortest_path(g, a, b, weight="w")
        length = nx.path_weight(g, path, weight="w")
        wids = []
        for u, v in zip(path, path[1:]):
            w = g[u][v]["way"]
            if not wids or wids[-1] != w:
                wids.append(w)
        missing = [w for w in dict.fromkeys(wids) if w not in used_ways]
        paths[key] = np.array([nodes[n] for n in path])
        p(f"[{key}] start node {a} ({math.dist(ll_to_tm(*nodes[a]), (sx, sy)):.1f}m from traj start)")
        p(f"[{key}] end node {b} ({math.dist(ll_to_tm(*nodes[b]), (ex, ey)):.1f}m from traj end)")
        p(f"[{key}] length {length:.1f}m, ways {len(dict.fromkeys(wids))}")
        for w in dict.fromkeys(wids):
            t = ways[w][1]
            p(f"  way{w} {t.get('highway')} name={t.get('name', '-')} "
              f"{'IN_XODR' if w in used_ways else 'NOT_IN_XODR'}")
        p(f"[{key}] ways not in xodr source: {len(missing)}")
    lib_c = np.mean([ll_to_tm(*q) for q in lib_nds], axis=0)
    p(f"[check] traj end - library centroid {math.dist(traj_tm[-1], lib_c):.1f}m, "
      f"traj start - OSM gate node 2261340221 {math.dist(traj_tm[0], ll_to_tm(*nodes['2261340221'])):.1f}m")
    p(f"[drive] trajectory {drive_len:.1f}m, start {traj_ll[0].round(6)}, end {traj_ll[-1].round(6)}")

    # --- 그림 ---
    all_ll = np.vstack([traj_ll, paths["car"], paths["all"], np.array(lib_nds)])
    px = np.array([ll_to_px(*q) for q in all_ll])
    pad = 120
    lo, hi = px.min(0) - pad, px.max(0) + pad
    mosaic, ox, oy, ntile = fetch_tiles(np.vstack([lo, hi]))
    p(f"[tiles] zoom {ZOOM}, {ntile} tiles, cache {TILE_DIR}")

    def to_ax(ll):
        q = np.array([ll_to_px(*v) for v in ll])
        return q[:, 0] - ox, q[:, 1] - oy

    fig, ax = plt.subplots(figsize=(12, 12), dpi=130)
    ax.imshow(np.asarray(mosaic))
    lx, ly = to_ax(lib_nds)
    ax.fill(lx, ly, color="#8e44ad", alpha=0.35, zorder=2)
    ax.plot(lx, ly, color="#8e44ad", lw=1.5, zorder=2)
    ax.text(lx.mean(), ly.mean(), "도서관", color="#4a235a", fontsize=11,
            ha="center", va="center", weight="bold", zorder=6)
    cx, cy = to_ax(paths["car"])
    ax.plot(cx, cy, color="#1f5fbf", lw=3, ls=(0, (4, 2)), zorder=4,
            label=f"원본 OSM 최단(차도) {nx_len(paths['car']):.0f}m")
    fx, fy = to_ax(paths["all"])
    ax.plot(fx, fy, color="#1e8449", lw=2.5, ls=(0, (1, 1.5)), zorder=4,
            label=f"원본 OSM 최단(보행로 포함) {nx_len(paths['all']):.0f}m")
    tx, ty = to_ax(traj_ll)
    ax.plot(tx, ty, color="#d62728", lw=4, alpha=0.85, zorder=3,
            label=f"세션16 주행 궤적 {drive_len:.0f}m")
    ax.plot(tx[0], ty[0], marker="*", ms=20, color="#2ca02c", mec="k", zorder=7)
    ax.text(tx[0] + 12, ty[0], "정문(출발)", fontsize=11, weight="bold", zorder=7,
            bbox=dict(fc="white", ec="none", alpha=0.8))
    ax.plot(tx[-1], ty[-1], marker="*", ms=20, color="#d62728", mec="k", zorder=7)
    ax.text(tx[-1] + 12, ty[-1] + 18, "종점(출입구1 앞)", fontsize=11, weight="bold",
            zorder=7, bbox=dict(fc="white", ec="none", alpha=0.8))
    for label, rid in SHARP.items():
        i = np.where(roads == rid)[0]
        if len(i):
            k = i[len(i) // 2]
            ax.annotate(label, (tx[k], ty[k]), (tx[k] + 40, ty[k] - 30), fontsize=10,
                        color="#7b241c", zorder=7, arrowprops=dict(arrowstyle="-", color="#7b241c"),
                        bbox=dict(fc="white", ec="#7b241c", alpha=0.85))
    # 축척 막대 100m
    mpp = 156543.03392 * COS0 / 2 ** ZOOM
    bx, by = 30, mosaic.size[1] - 40
    ax.plot([bx, bx + 100 / mpp], [by, by], color="k", lw=4, zorder=8)
    ax.text(bx + 50 / mpp, by - 12, "100m", ha="center", fontsize=10, zorder=8)
    ax.set_xlim(lo[0] - ox, hi[0] - ox)
    ax.set_ylim(hi[1] - oy, lo[1] - oy)
    ax.set_xticks([]), ax.set_yticks([])
    ax.legend(loc="upper right", fontsize=10, framealpha=0.9)
    ax.set_title("맵세션16 완주 경로 — 실제 지도(OSM 표준 타일, 위=북)\n"
                 f"궤적 {DRIVE_LOG.name}, CARLA(x,-y)->로컬TM 역변환", fontsize=12)
    ax.text(0.995, 0.003, "© OpenStreetMap contributors", transform=ax.transAxes,
            ha="right", va="bottom", fontsize=8, bbox=dict(fc="white", ec="none", alpha=0.8))
    fig.tight_layout()
    fig.savefig(OUT_PNG)
    p(f"[out] {OUT_PNG}")
    OUT_LOG.write_text("\n".join(out) + "\n")


def nx_len(ll):
    q = np.array([ll_to_tm(*v) for v in ll])
    return np.hypot(*np.diff(q, axis=0).T).sum()


if __name__ == "__main__":
    main()
