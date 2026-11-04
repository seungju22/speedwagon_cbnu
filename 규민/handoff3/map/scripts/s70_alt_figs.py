#!/usr/bin/env python3
# 10/25 3단계 그림: 승강장마다 원래·대체 정차 구역(지도 좌표). 차로가 오른쪽을 향하게 돌려 그림
# 입력 map/docs/logs/s70_alt_stops.json, 차로 조각(s56_shuttle·place_campus_layers 함수), v32a 횡단보도, 건물 상자
# 출력 map/docs/figs/s70_alt_stops.png, docs/midterm_material/fig16_alt_stops.png(같은 그림)
# 사용: .venv-carla/bin/python map/scripts/s70_alt_figs.py
import json
import math
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager as fm  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from shapely import affinity  # noqa: E402
from shapely.geometry import Polygon, box  # noqa: E402

HERE = Path(__file__).resolve().parent
BASE = HERE.parent
ROOT = BASE.parent
sys.path.insert(0, str(HERE))
import s56_shuttle as s56  # noqa: E402
import place_campus_layers as pcl  # noqa: E402
import s70_alt_stops as s70  # noqa: E402
from s22_buildings import Frame  # noqa: E402

FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
fm.fontManager.addfont(FONT); plt.rcParams["font.family"] = fm.FontProperties(fname=FONT).get_name()
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
C_ORIG, C_ALT = "#2a78d6", "#1baf7a"
plt.rcParams.update({"axes.edgecolor": INK2, "figure.facecolor": SURF, "axes.facecolor": SURF, "font.size": 9})
OUT = BASE / "docs/figs/s70_alt_stops.png"
MID = ROOT / "docs/midterm_material/fig16_alt_stops.png"


def main():
    import carla
    d = json.loads(s70.OUT.read_text())
    text = s70.XODR.read_text()
    cmap = carla.Map("OpenDriveMap", text)
    root = ET.fromstring(text)
    quads = [s56.flip(q) for _, q in pcl.lane_quads(cmap, root)]
    frame = Frame(text)
    blds = []
    for r in pcl.read_layout(pcl.LAYOUT):
        cx, cy = frame.carla(float(r["center_lat"]), float(r["center_lon"]))
        blds.append(s56.flip(pcl.rect(cx, cy, float(r["yaw_deg"]), float(r["length_m"]), float(r["width_m"]))))
    blds += [s70.sq(b["cx"], b["cy"], b["yaw_deg"], b["side"]) for b in json.loads(s70.BRECT.read_text())["rects"]]
    cws = [g for _, g in s70.crosswalks(s70.V32A)]
    import subprocess
    plats = {p["id"]: p for p in json.loads(subprocess.run(["/usr/bin/python3", "-c", "import json,sys,yaml;print(json.dumps(yaml.safe_load(open(sys.argv[1]))))",
                                                            str(s70.PLAT)], capture_output=True, text=True, check=True).stdout)["platforms"]}
    fig, axs = plt.subplots(4, 3, figsize=(12, 12.5))
    for ax, rec in zip(axs.flat, d["platforms"]):
        p = plats[rec["id"]]
        o = p["base_link_goal"]
        yaw = o["yaw_deg"]
        cx, cy = (o["x"], o["y"])
        if rec["status"] == "있음":
            a = rec["alt"]["base_link_goal"]
            cx, cy = (o["x"] + a["x"]) / 2, (o["y"] + a["y"]) / 2
        rot = lambda g: affinity.rotate(affinity.translate(g, -cx, -cy), -yaw, origin=(0, 0))
        view = affinity.rotate(box(-14, -9, 14, 9), yaw, origin=(0, 0))
        view = affinity.translate(view, cx, cy)

        def draw(g, **kw):
            if g.is_empty or not g.intersects(view):
                return
            g = rot(g.intersection(view.buffer(2)))
            for gg in (g.geoms if hasattr(g, "geoms") else [g]):
                if gg.geom_type == "Polygon":
                    x, y = gg.exterior.xy
                    ax.fill(x, y, **kw)
        for q in quads:
            draw(q, fc=GRID, ec="none")
        for b in blds:
            draw(b, fc="#cfc6b8", ec="none")
        for c in cws:
            draw(c, fc="none", ec=INK2, hatch="////", lw=0.5)
        if p.get("facility"):
            draw(Polygon(p["facility"]), fc=INK2, ec="none")
        draw(Polygon(p["stop_zone"]), fc="none", ec=C_ORIG, lw=1.8)
        if len(p.get("door_zone", [])) >= 3 and p.get("door_zone_area_m2", 0) > 0:
            draw(Polygon(p["door_zone"]), fc=C_ORIG, ec="none", alpha=0.35)
        title = f"{p['name']}({p['side']}) {rec['id']}"
        if rec["status"] == "있음":
            draw(Polygon(rec["alt"]["stop_zone"]), fc="none", ec=C_ALT, lw=1.8, ls="--")
            draw(Polygon(rec["alt"]["door_zone"]), fc=C_ALT, ec="none", alpha=0.35)
            title += f"\n대체: 뒤로 {rec['alt']['shift_back_m']}m"
        else:
            title += "\n대체 없음(원래 정차 불가)"
        ax.annotate("", xy=(12, -7.5), xytext=(6, -7.5), arrowprops=dict(arrowstyle="->", color=INK2))
        ax.text(6, -8.6, "진행 방향", color=INK2, fontsize=8)
        ax.set_xlim(-14, 14); ax.set_ylim(-9, 9); ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
        ax.set_title(title, fontsize=9, color=INK)
    lg = axs.flat[-1]
    lg.axis("off")
    from matplotlib.patches import Patch
    from matplotlib.lines import Line2D
    lg.legend(handles=[Line2D([], [], color=C_ORIG, lw=1.8, label="원래 정차 구역(차 + 앞뒤 1m)"),
                       Patch(fc=C_ORIG, alpha=0.35, label="원래 문 구역"),
                       Line2D([], [], color=C_ALT, lw=1.8, ls="--", label="대체 정차 구역"),
                       Patch(fc=C_ALT, alpha=0.35, label="대체 문 구역"),
                       Patch(fc=GRID, label="차로(xodr)"), Patch(fc="#cfc6b8", label="건물 상자·정육면체"),
                       Patch(fc=INK2, label="정류장 시설"), Patch(fc="none", ec=INK2, hatch="////", label="횡단보도(v32a)")],
              loc="center", frameon=False, fontsize=9)
    fig.suptitle("승강장별 원래·대체 정차 구역(세션70, 지도 좌표를 진행 방향 기준으로 돌림, 칸 28 x 18m)", fontsize=11, color=INK)
    fig.tight_layout()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=130)
    shutil.copy(OUT, MID)
    print(OUT, MID)


if __name__ == "__main__":
    main()
