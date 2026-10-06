#!/usr/bin/env python3
# 맵세션16. 도서관 출입구 재검토(사용자: 서쪽 면에는 출입구 없음, 북·남(구관)과
# 동(신관, 제1학생회관 방향)에 있음). 면마다 가장 가까운 xodr 주행 차선 지점을
# 정차 후보로 뽑고, 정문(세션16 궤적 출발점)에서의 경로 길이를 GlobalRoutePlanner
# 로 계산한다. 서버 불필요(carla.Map(name, xodr) 클라이언트 지도). 읽기전용.
# 출력: map/docs/figures/23_library_entrances.png, map/docs/logs/session16_library_entrances.log
#
# 면 중점: 건물 외곽선 변 중 바깥 법선이 해당 방향과 45° 이내인 변들 가운데
#   가장 긴 변의 중점. 실제 출입구 위치가 아니라 "그 면" 의 대표점이다.
# 후보: 면 중점 80m 안 Driving 차선 중심 표본(1m) 중 road 별 최근접 1점, 가까운 순 3개.
import csv
import math
import sys
from pathlib import Path

import carla
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import plot_route_realmap_s16 as R  # noqa: E402

sys.path.insert(0, str(Path.home() / "carla/CARLA_0.9.15/PythonAPI/carla"))
from agents.navigation.global_route_planner import GlobalRoutePlanner  # noqa: E402

BASE = R.BASE
XODR = BASE / "maps" / "cbnu_internal_only_localtm_tags73_smooth.xodr"
OUT_PNG = BASE / "docs" / "figures" / "23_library_entrances.png"
OUT_LOG = BASE / "docs" / "logs" / "session16_library_entrances.log"
DRIVE_LOG = R.LOG_DIR / "drive_log_20260926_144047.csv"
OLD_WAY, NEW_WAY = "442066135", "648686778"          # 구관, 신관
FACES = [("북", OLD_WAY, (0, 1)), ("남", OLD_WAY, (0, -1)), ("동", NEW_WAY, (1, 0))]
CUR_END = (581.92, -675.04)                          # 현 종점 근처 궤적 끝(CARLA)
R.ZOOM = 18


def carla_to_tm(x, y):
    return x - R.OFF_X, -y - R.OFF_Y


def tm_to_carla(x, y):
    return x + R.OFF_X, -(y + R.OFF_Y)


def face_mid(poly, d):
    best = None
    ccw = sum(poly[i][0] * poly[i + 1][1] - poly[i + 1][0] * poly[i][1]
              for i in range(len(poly) - 1)) > 0
    for (x0, y0), (x1, y1) in zip(poly, poly[1:]):
        ln = math.hypot(x1 - x0, y1 - y0)
        if ln < 1e-6:
            continue
        nx_, ny_ = ((y1 - y0) / ln, -(x1 - x0) / ln) if ccw else (-(y1 - y0) / ln, (x1 - x0) / ln)
        if nx_ * d[0] + ny_ * d[1] >= math.cos(math.radians(45)) and (best is None or ln > best[0]):
            best = (ln, ((x0 + x1) / 2, (y0 + y1) / 2))
    return best[1]


def main():
    out = []
    p = lambda s: (print(s), out.append(s))  # noqa: E731
    nodes, ways = R.load_osm(R.OSM_RAW)
    polys = {w: [R.ll_to_tm(*nodes[n]) for n in ways[w][0] if n in nodes] for w in (OLD_WAY, NEW_WAY)}

    m = carla.Map("cbnu_s16", XODR.read_text())
    wps = [w for w in m.generate_waypoints(1.0) if w.lane_type == carla.LaneType.Driving]
    wtm = np.array([carla_to_tm(w.transform.location.x, w.transform.location.y) for w in wps])
    grp = GlobalRoutePlanner(m, 1.0)
    row = next(csv.DictReader(open(DRIVE_LOG)))
    start = carla.Location(float(row["x"]), float(row["y"]), 0.5)

    def route_len(wp):
        tr = grp.trace_route(start, wp.transform.location)
        pts = np.array([(t[0].transform.location.x, t[0].transform.location.y) for t in tr])
        return np.hypot(*np.diff(pts, axis=0).T).sum(), [t[0].road_id for t in tr], pts

    p(f"start CARLA ({start.x:.2f},{start.y:.2f}), driving samples {len(wps)}")
    cands = []
    cur = m.get_waypoint(carla.Location(*CUR_END, 0.5))
    ln, _, _ = route_len(cur)
    p(f"[현 종점] road{cur.road_id} lane{cur.lane_id} s={cur.s:.1f} route {ln:.1f}m")
    for name, way, d in FACES:
        fx, fy = face_mid(polys[way], d)
        dist = np.hypot(wtm[:, 0] - fx, wtm[:, 1] - fy)
        seen = {}
        for i in np.argsort(dist):
            if dist[i] > 80:
                break
            key = (wps[i].road_id, wps[i].lane_id)
            if key not in seen and wps[i].road_id not in {v[0].road_id for v in seen.values()}:
                seen[key] = (wps[i], dist[i])
            if len(seen) >= 3:
                break
        p(f"[{name}] 면 중점 TM ({fx:.1f},{fy:.1f}) ({'구관' if way == OLD_WAY else '신관'})")
        for k, (wp, dd) in enumerate(seen.values(), 1):
            ln, rds, pts = route_len(wp)
            seq = [r for j, r in enumerate(rds) if j == 0 or rds[j - 1] != r]
            tag = f"{name}{k}"
            p(f"  {tag}: road{wp.road_id} lane{wp.lane_id} s={wp.s:.1f} junction={wp.is_junction} "
              f"면까지 {dd:.1f}m, 정문부터 {ln:.1f}m, road {len(seq)}개 {seq}")
            cands.append((tag, wp, dd, ln, pts, (fx, fy)))

    # --- 그림: 도서관 주변 확대, xodr 주행 차선 중심선 + 후보 ---
    lib_ll = [R.tm_to_ll(*q) for w in polys for q in polys[w]]
    c = np.mean([R.ll_to_tm(*q) for q in lib_ll], axis=0)
    box = [R.tm_to_ll(c[0] + dx, c[1] + dy) for dx in (-230, 230) for dy in (-200, 200)]
    px = np.array([R.ll_to_px(*q, R.ZOOM) for q in box])
    mosaic, ox, oy, ntile = R.fetch_tiles(px)
    p(f"[tiles] zoom {R.ZOOM}, {ntile} tiles")

    def ax_xy(tm_pts):
        q = np.array([R.ll_to_px(*R.tm_to_ll(x, y), R.ZOOM) for x, y in tm_pts])
        return q[:, 0] - ox, q[:, 1] - oy

    fig, ax = plt.subplots(figsize=(12, 11), dpi=130)
    ax.imshow(np.asarray(mosaic))
    near = np.hypot(wtm[:, 0] - c[0], wtm[:, 1] - c[1]) < 320
    wx, wy = ax_xy(wtm[near])
    ax.scatter(wx, wy, s=1.5, color="#555", zorder=2, label="xodr 주행 차선 중심(1m 표본)")
    for w, col, lab in ((OLD_WAY, "#8e44ad", "구관"), (NEW_WAY, "#c0392b", "신관")):
        x, y = ax_xy(polys[w])
        ax.fill(x, y, color=col, alpha=0.3, zorder=3)
        ax.text(x.mean(), y.mean(), lab, fontsize=12, weight="bold", color=col, ha="center", zorder=6)
    colors = {"북": "#1f5fbf", "남": "#1e8449", "동": "#d35400"}
    for tag, wp, dd, ln, pts, fmid in cands:
        col = colors[tag[0]]
        mx, my = ax_xy([fmid])
        ax.plot(mx, my, marker="s", ms=10, color=col, mec="k", zorder=7)
        x, y = ax_xy([carla_to_tm(wp.transform.location.x, wp.transform.location.y)])
        ax.plot(x, y, marker="o", ms=9, color=col, mec="k", zorder=7)
        ax.plot([mx[0], x[0]], [my[0], y[0]], color=col, lw=1, ls=":", zorder=6)
        ax.annotate(f"{tag} {ln:.0f}m", (x[0], y[0]), (8, -8), textcoords="offset points",
                    fontsize=10, color=col, weight="bold", zorder=8,
                    bbox=dict(fc="white", ec=col, alpha=0.85))
    x, y = ax_xy([carla_to_tm(*CUR_END)])
    ax.plot(x, y, marker="*", ms=20, color="#d62728", mec="k", zorder=8)
    ax.annotate("현 종점(서쪽)", (x[0], y[0]), (10, 10), textcoords="offset points",
                fontsize=10, weight="bold", zorder=8, bbox=dict(fc="white", ec="none", alpha=0.85))
    ax.plot([], [], "s", color="gray", mec="k", label="면 중점(면 대표점, 실제 문 위치 아님)")
    ax.plot([], [], "o", color="gray", mec="k", label="정차 후보(최근접 주행 차선) + 정문부터 경로 길이")
    mpp = 156543.03392 * R.COS0 / 2 ** R.ZOOM
    bx, by = 30, mosaic.size[1] - 40
    x0, x1 = ax_xy([(c[0] - 230, c[1])])[0][0], ax_xy([(c[0] + 230, c[1])])[0][0]
    y0, y1 = ax_xy([(c[0], c[1] + 200)])[1][0], ax_xy([(c[0], c[1] - 200)])[1][0]
    ax.plot([x0 + 20, x0 + 20 + 50 / mpp], [y1 - 25] * 2, color="k", lw=4, zorder=8)
    ax.text(x0 + 20 + 25 / mpp, y1 - 35, "50m", ha="center", fontsize=10, zorder=8)
    ax.set_xlim(x0, x1)
    ax.set_ylim(y1, y0)
    ax.set_xticks([]), ax.set_yticks([])
    ax.legend(loc="upper left", fontsize=9, framealpha=0.9)
    ax.set_title("맵세션16 도서관 출입구 재검토 — 북·남(구관), 동(신관) 면별 정차 후보\n"
                 "OSM 표준 타일 zoom18, 위=북", fontsize=12)
    ax.text(0.995, 0.003, "© OpenStreetMap contributors", transform=ax.transAxes,
            ha="right", va="bottom", fontsize=8, bbox=dict(fc="white", ec="none", alpha=0.8))
    fig.tight_layout()
    fig.savefig(OUT_PNG)
    p(f"[out] {OUT_PNG}")
    OUT_LOG.write_text("\n".join(out) + "\n")


if __name__ == "__main__":
    main()
