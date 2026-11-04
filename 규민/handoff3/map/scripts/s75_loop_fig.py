#!/usr/bin/env python3
# 11/03: v3.3c 순환 노선 그림(Lanelet2 경로 레인렛 열 = s75_check_v33c.py 결과의 circulation ids). 서버 없음, 읽기만
#   회색 = 모든 레인렛, 주황 = 순환 경로, 초록 점 = 정류장(순환 파일 goal), 원 = 유턴 두 곳·N14 모서리
# 사용: .venv-carla/bin/python map/scripts/s75_loop_fig.py [out.png]
import json, math, subprocess, sys, xml.etree.ElementTree as ET
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

C = Path(__file__).resolve().parents[2]
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else C / "map/docs/figs/v33c_loop.png"
for f in font_manager.findSystemFonts():
    if "NanumGothic" in f or "NotoSansCJK" in f:
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=f).get_name(); break
r = ET.parse(C / "map/maps/v33c/lanelet2_map.osm").getroot()
nodes = {}
for n in r.iter("node"):
    t = {x.get("k"): x.get("v") for x in n.findall("tag")}; nodes[n.get("id")] = (float(t["local_x"]), float(t["local_y"]))
ways = {w.get("id"): [nd.get("ref") for nd in w.findall("nd")] for w in r.iter("way")}
CL = {}
for rel in r.iter("relation"):
    t = {x.get("k"): x.get("v") for x in rel.findall("tag")}
    if t.get("type") != "lanelet": continue
    mem = {m.get("role"): m.get("ref") for m in rel.findall("member")}
    if "centerline" in mem: CL[int(rel.get("id"))] = [nodes[q] for q in ways[mem["centerline"]]]
    else:
        L = [nodes[q] for q in ways[mem["left"]]]; R = [nodes[q] for q in ways[mem["right"]]]
        n = max(len(L), len(R)); f = lambda P, i: P[min(len(P) - 1, round(i * (len(P) - 1) / max(1, n - 1)))]
        CL[int(rel.get("id"))] = [((f(L, i)[0] + f(R, i)[0]) / 2, (f(L, i)[1] + f(R, i)[1]) / 2) for i in range(n)]
chk = json.loads((C / "integration/logs/s75/v33c_route_check.json").read_text())
RT = json.loads(subprocess.check_output(["/usr/bin/python3", "-c", "import sys,json,yaml;print(json.dumps(yaml.safe_load(open(sys.argv[1]))))",
                                         str(C / "map/data/routes/circulation_v33c_r1.yaml")]))
fig, ax = plt.subplots(figsize=(11, 9), dpi=100)
for k, P in CL.items(): ax.plot([p[0] for p in P], [p[1] for p in P], color="#cfcfcf", lw=0.8, zorder=1)
tot = 0.0
for sg in chk["circulation"]:
    for k in sg["ids"] or []:
        P = CL[k]; ax.plot([p[0] for p in P], [p[1] for p in P], color="#eb6834", lw=2.6, zorder=3)
for i, s in enumerate(RT["stops"]):
    g = s["goal"]; ax.plot(g["x"], g["y"], "o", color="#1a9e5f", ms=9, zorder=5)
    ax.text(g["x"] + 8, g["y"] + 6, f"{i + 1}. {s['name']}", fontsize=10, color="#0b5d36", zorder=6)
for (x, y), nm in (((722, 641), "모서리 유턴(가)"), ((970, 280), "후문 앞 유턴"), ((537, 941), "N14 모서리")):
    ax.plot(x, y, "o", ms=22, mfc="none", mec="#1f5fd1", mew=2.2, zorder=7); ax.text(x + 14, y - 12, nm, fontsize=9, color="#1f5fd1", zorder=7)
ax.set_aspect("equal"); ax.set_title(f"순환 노선 v3.3c(11/03) 약 {RT['length_m']:.0f}m, 정류장 {len(RT['stops'])}곳", fontsize=12)
ax.set_xlabel("지도 x (m)"); ax.set_ylabel("지도 y (m)")
OUT.parent.mkdir(parents=True, exist_ok=True); fig.tight_layout(); fig.savefig(OUT); print(OUT)
