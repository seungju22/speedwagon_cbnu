#!/usr/bin/env python3
# 11/03: 북문 길 끝 유턴 자리(개신문화관 앞) 확인 그림. 서버 없음, 읽기만
#   회색 = CARLA 도로면(동결 xodr 214507af 를 carla.Map 으로 오프라인, get_waypoint(project_to_road=False, Driving|Sidewalk)
#          가 None 이 아닌 곳 = s4_track.py 바퀴 이탈 판정과 같은 기준), 진회색 = 그중 Driving
#   파랑 선 = v3.3b 유턴 레인렛(relation 52799) 경계, 빨강 점 = s74 프리우스 바퀴 이탈 틱 자리
#   OSM(받은 그대로) 건물·주차장·길, 고를 칸(5m 격자, 열 A.. 행 1..)
# 사용: .venv-carla/bin/python map/scripts/s75_uturn_fig.py [x0 x1 y0 y1] [--out 경로] [--json 경로]
import argparse, csv, json, re, sys
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

HERE = Path(__file__).resolve().parent; C = HERE.parents[1]
sys.path.insert(0, str(HERE))
import s72_common as K  # noqa: E402
import carla  # noqa: E402

ap = argparse.ArgumentParser(); ap.add_argument("box", nargs="*", type=float, default=[720, 790, 670, 735])
ap.add_argument("--rel", default="52799"); ap.add_argument("--osm", default=str(C / "map/maps/v33b/lanelet2_map.osm"))
ap.add_argument("--track", default=str(C / "integration/logs/i5_track_s74loop_prius.csv"))
ap.add_argument("--out", default=str(C / "integration/logs/s75/figs/north_uturn_ask.png"))
ap.add_argument("--json", default=str(C / "integration/logs/s75/north_uturn_surface.json"))
ap.add_argument("--cell", type=float, default=5.0); ap.add_argument("--step", type=float, default=0.5); ap.add_argument("--mark", action="append", default=[]); ap.add_argument("--title", default="")
a = ap.parse_args(); x0, x1, y0, y1 = a.box
for f in font_manager.findSystemFonts():
    if "NanumGothic" in f or "NotoSansCJK" in f:
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=f).get_name(); break

m = K.cmap(); ROAD = carla.LaneType.Driving | carla.LaneType.Sidewalk
st = a.step; grid = {}
y = y0
while y <= y1:
    x = x0
    while x <= x1:
        w = m.get_waypoint(carla.Location(x, -y, 0.0), project_to_road=False, lane_type=ROAD)
        if w is not None:
            grid[(round(x, 2), round(y, 2))] = (str(w.lane_type), w.road_id, w.lane_id)
        x += st
    y += st

fig, ax = plt.subplots(figsize=(10, 9.3), dpi=110)
for (x, y), (t, r, l) in grid.items():
    ax.add_patch(plt.Rectangle((x - st / 2, y - st / 2), st, st, color="#6b6b6b" if t == "Driving" else "#bdbdbd", lw=0))

# OSM 건물·면·길
names = []
for wid, pts, t in K.osm_ways(lambda t: True):
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    if max(xs) < x0 or min(xs) > x1 or max(ys) < y0 or min(ys) > y1: continue
    if "building" in t:
        ax.fill(xs, ys, fc="#f3d9a4", ec="#b38b3c", lw=1, alpha=0.7)
        if t.get("name"): names.append((sum(xs) / len(xs), sum(ys) / len(ys), t["name"]))
    elif t.get("amenity") == "parking" or t.get("area") == "yes":
        ax.fill(xs, ys, fc="none", ec="#7a3fb0", lw=1.4, ls="--")
        ax.text(sum(xs) / len(xs), sum(ys) / len(ys), f"OSM {t.get('amenity') or t.get('highway')}\n{wid}", color="#7a3fb0", fontsize=7, ha="center")
    elif "highway" in t:
        ax.plot(xs, ys, color="#2e8b57" if t["highway"] in ("footway", "pedestrian", "path", "steps") else "#d2691e", lw=1, ls=":")
for cx, cy, n in names:
    if x0 < cx < x1 and y0 < cy < y1: ax.text(cx, cy, n, fontsize=10, ha="center", weight="bold")

# 유턴 레인렛 경계(v3.3b)
txt = Path(a.osm).read_text()
NODE_RE = r'<node id="(\d+)"[^>]*>\s*<tag k="ele" v="[-\d.]+" />\s*<tag k="local_x" v="([-\d.]+)" />\s*<tag k="local_y" v="([-\d.]+)" />'
nodes = {i: (float(x), float(y)) for i, x, y in re.findall(NODE_RE, txt)}
rel = re.search(rf'<relation id="{a.rel}"[^>]*>(.*?)</relation>', txt, re.S).group(1)
uturn = {}
for ref, role in re.findall(r'<member type="way" ref="(\d+)" role="(\w+)" />', rel):
    nd = re.findall(r'<nd ref="(\d+)" />', re.search(rf'<way id="{ref}"[^>]*>(.*?)</way>', txt, re.S).group(1))
    pts = [nodes[n] for n in nd]; uturn[role] = pts
    ax.plot([p[0] for p in pts], [p[1] for p in pts], color="#1f5fd1", lw=2.2, label=f"유턴 레인렛 {a.rel} 경계" if role == "left" else None)

# 상자 안 레인렛 경계(가는 선)
for wid_, body in re.findall(r'<way id="(\d+)"[^>]*>(.*?)</way>', txt, re.S):
    nd = re.findall(r'<nd ref="(\d+)" />', body); pts = [nodes[n] for n in nd if n in nodes]
    if pts and any(x0 < p[0] < x1 and y0 < p[1] < y1 for p in pts):
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color="#00a0a0", lw=0.6, alpha=0.8)
# s74 바퀴 이탈 자리
off = []
for r in csv.DictReader(open(a.track)):
    x, y = float(r["x"]), -float(r["y"])
    if int(r["wheels_off"]) > 0 and x0 < x < x1 and y0 < y < y1: off.append((x, y))
if off: ax.scatter([p[0] for p in off[::20]], [p[1] for p in off[::20]], s=6, c="red", label=f"바퀴 이탈 틱({Path(a.track).stem}, {len(off)}, 20개마다 1점)", zorder=5)

# 고를 칸
cs = a.cell; cols = []; xx = x0; i = 0
while xx < x1:
    ax.axvline(xx, color="#999", lw=0.4); cols.append(xx); xx += cs
yy = y0
while yy < y1:
    ax.axhline(yy, color="#999", lw=0.4); yy += cs
nc = int((x1 - x0) / cs); nr = int((y1 - y0) / cs)
for ci in range(nc):
    ax.text(x0 + cs * (ci + 0.5), y1 + 0.6, chr(65 + ci), ha="center", fontsize=9, color="#333")
for ri in range(nr):
    ax.text(x0 - 1.0, y1 - cs * (ri + 0.5), str(ri + 1), va="center", ha="right", fontsize=9, color="#333")
ax.plot([], [], "s", color="#6b6b6b", label="CARLA 차도(Driving)"); ax.plot([], [], "s", color="#bdbdbd", label="CARLA 인도 면(Sidewalk, 바퀴 이탈 아님)")
ax.plot([], [], color="#7a3fb0", ls="--", label="OSM 주차장·면"); ax.plot([], [], color="#2e8b57", ls=":", label="OSM 보행로")
for mk in a.mark:   # "이름:x:y"
    nm, mx, my = mk.split(":"); ax.scatter([float(mx)], [float(my)], s=500, facecolors="none", edgecolors="#e00000", lw=3, zorder=6)
    ax.text(float(mx) + 6, float(my) + 6, nm, color="#e00000", fontsize=16, weight="bold", zorder=7)
ax.set_xlim(x0 - 2, x1); ax.set_ylim(y0, y1 + 2); ax.set_aspect("equal")
ax.set_title(a.title or "북문 길 끝 유턴 자리: 실제 포장이 어디까지? (칸 이름으로 골라 주세요, 5m 칸)", fontsize=11)
ax.legend(loc="lower left", fontsize=8, framealpha=0.9)
ax.set_xlabel("지도 x (m)"); ax.set_ylabel("지도 y (m), 위 = 북")
Path(a.out).parent.mkdir(parents=True, exist_ok=True); fig.tight_layout(); fig.savefig(a.out)

# 칸별 CARLA 면 비율(나중에 답과 맞춤)
cells = {}
for ci in range(nc):
    for ri in range(nr):
        cx0 = x0 + cs * ci; cy1 = y1 - cs * ri
        pts = [(x, y) for (x, y) in grid if cx0 <= x < cx0 + cs and cy1 - cs < y <= cy1]
        cells[f"{chr(65 + ci)}{ri + 1}"] = round(len(pts) / (cs / st) ** 2, 2)
J = dict(box=a.box, cell=cs, step=st, surface_cells=len(grid), off_ticks=len(off), uturn_rel=a.rel,
         uturn_pts_outside=sum(1 for role in uturn for (x, y) in uturn[role]
                               if m.get_waypoint(carla.Location(x, -y, 0), project_to_road=False, lane_type=ROAD) is None),
         uturn_pts=sum(len(v) for v in uturn.values()), road_ids=sorted({v[1] for v in grid.values()}), cell_surface_ratio=cells,
         osm=a.osm)
Path(a.json).write_text(json.dumps(J, ensure_ascii=False, indent=1))
print(a.out, "surface", len(grid), "uturn outside", J["uturn_pts_outside"], "/", J["uturn_pts"], "roads", J["road_ids"])
