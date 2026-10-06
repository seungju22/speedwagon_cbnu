#!/usr/bin/env python3
# 맵세션15 Phase D. 완주 궤적을 확정 경로 오버레이에 겹친다.
# 축척·범위·방향은 10_overlay_confirmed.png(plot_confirmed_endpoint.py)와 동일:
# old_axis_bounds() 범위, figsize (11,13), dpi 150, ENU 위쪽=진북.
# 변환은 평활화 xodr(세션11 재변환판, road id 동일) 자신의 junction 으로 재적합.
# CARLA 좌표 -> xodr 좌표: y 부호 반전. 읽기전용, 새 출력 11_drive_trajectory.png
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import plot_confirmed_endpoint as P  # noqa: E402  (폰트 설정 포함)
from geo_calibrate import apply_transform  # noqa: E402

BASE_DIR = Path(__file__).resolve().parent.parent
SMOOTH_XODR = BASE_DIR / "maps" / "cbnu_internal_only_localtm_tags73_smooth.xodr"
LOG_DIR = BASE_DIR / "docs" / "logs"
OUT_PATH = P.FIG_DIR / "11_drive_trajectory.png"
SHARP = {"1446": "r1446 +78.6°", "1428": "r1428 -68.5°", "1564": "r1564 -82.2°"}


def main(csv_name):
    xmin, xmax, ymin, ymax = P.old_axis_bounds()
    roads, junctions, osm_nodes, R, s, t, ref_lat, ref_lon = P.fit_transform_for(SMOOTH_XODR)

    def enu(x, y):
        return apply_transform(R, s, t, x, y)

    rows = list(csv.DictReader(open(LOG_DIR / csv_name)))
    traj = [enu(float(r["x"]), -float(r["y"])) for r in rows]
    sw = [enu(float(r["x"]), -float(r["y"])) for r in rows if r["sidewalk"] == "True"]

    fig, ax = plt.subplots(figsize=(11, 13))
    for rid, road in roads.items():
        if road["junction"] != "-1":
            continue
        pts = [enu(x, y) for x, y in P.road_polyline(road)]
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color="0.35", lw=0.8, zorder=2)
    for rid in P.CONFIRMED_PATH_FULL:
        pts = [enu(x, y) for x, y in P.road_polyline(roads[rid])]
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color="tab:red", lw=4.0,
                alpha=0.45, zorder=3)
    last = [enu(x, y) for x, y in P.partial_polyline(roads[P.LAST_ROAD_ID], P.LAST_ROAD_S_LIMIT)]
    ax.plot([p[0] for p in last], [p[1] for p in last], color="tab:red", lw=4.0, alpha=0.45,
            zorder=3, label="계획 경로(15 road)")
    ax.plot([p[0] for p in traj], [p[1] for p in traj], color="tab:blue", lw=1.2, zorder=4,
            label=f"세션15 완주 궤적({rows[-1]['t']}s)")
    if sw:
        ax.scatter([p[0] for p in sw], [p[1] for p in sw], s=6, c="orange", zorder=5,
                   label=f"인도 침범 tick({len(sw)})")
    ax.scatter([traj[0][0]], [traj[0][1]], s=90, c="tab:green", marker="*", zorder=6,
               label="출발(road1247)")
    ax.scatter([traj[-1][0]], [traj[-1][1]], s=100, c="tab:red", marker="*", zorder=6,
               label="도착(road1278, 종점 4.97m)")
    for rid, label in SHARP.items():
        c = [enu(x, y) for x, y in P.road_polyline(roads[rid])]
        mid = c[len(c) // 2]
        ax.annotate(label, mid, textcoords="offset points", xytext=(10, -12), fontsize=8,
                    color="purple", arrowprops=dict(arrowstyle="-", color="purple", lw=0.6))

    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("동서 (m, ENU east)")
    ax.set_ylabel("남북 (m, ENU north) - 위쪽=진북")
    ax.grid(True, linewidth=0.3, alpha=0.4)
    ax.set_title("맵세션15 완주 궤적(11) — cbnu_internal_only_localtm_tags73_smooth.xodr\n"
                 "BasicAgent 20km/h, 동기 20Hz, wall_height 0, conn_spacing 4m\n"
                 f"축척·범위 = 10_overlay_confirmed.png, 로그 {csv_name}")
    ax.legend(loc="upper right", fontsize=8)
    ax.annotate("N", xy=(0.95, 0.90), xytext=(0.95, 0.80), xycoords="axes fraction",
                textcoords="axes fraction",
                arrowprops=dict(arrowstyle="-|>", color="black", lw=1.5),
                ha="center", fontsize=11)
    fig.tight_layout()
    fig.savefig(OUT_PATH, dpi=150)
    plt.close(fig)
    print(f"저장: {OUT_PATH}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "drive_log_20260925_113316.csv")
