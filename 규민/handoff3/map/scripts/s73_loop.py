#!/usr/bin/env python3
# 11/02 4단계: v3.3a 순환 노선(road 그래프, s72_net.py 를 v3.3a 로: S72_BAN = v3.3a 에서 뺀 것, S72_RESTORE=1318,1188, S72_UTURN=1415:1409)
#   정류장 승강장 = integration/logs/s73/platforms_<veh>.json 의 새 승강장 + 승강장 1판의 북문·남문(건너편)·양성재
#   순서·방향(이쪽/건너편)을 다 바꿔 한 바퀴가 가장 짧은 고리. 연결로 반경 3.77m 밑(스타렉스)·접힌 연결로 안 씀. U턴은 북문 길 끝 유턴 레인렛 하나만
#   큰 도로(안 1, 11/02 '맞음') 마다 v3.3a 에서 양방향인지·고리 안인지, 고리가 지나는지
# 출력 integration/logs/s73/loop_v33a.json, figs/loop_v33a.png
import itertools, json, math, os, subprocess
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
import s72_net as N
import s72_common as K

OUT = K.C / "integration/logs/s73"
VEH = os.environ.get("VEH", "sample")
P = json.loads((OUT / f"platforms_{VEH}.json").read_text())["platforms"]
PL1 = {p["id"]: p for p in json.loads(subprocess.check_output(["/usr/bin/python3", "-c", "import sys,json,yaml;print(json.dumps(yaml.safe_load(open(sys.argv[1]))))",
                                                                str(K.BASE / "data/shuttle_platforms_v31_r2.yaml")]))["platforms"]}


def new(pid):
    b = P[pid]["best"]; return (pid, b["base_link_ref"]["road_id"], b["base_link_ref"]["s"] + 1.345)   # 문 점 s 근사(뒷축 s + 문 앞뒤)


def old(pid):
    p = PL1[pid]; return (pid, p["stops_v3"]["road_id"], float(p["stops_v3"]["s"]))


STOP = {"main": [new("main_gate_inner_se"), new("main_gate_inner_nw")], "gaeseong": [new("middle_gate_s")], "north": [old("lib_north")],
        "edu": [new("edu_front")], "south": [new("lib_south_se"), old("lib_south_nw")], "yang": [old("yangseong_se"), old("yangseong_nw")],
        "back": [new("back_gate_inner_se"), new("back_gate_inner_nw")]}
NAME = {"main": "정문 안쪽", "gaeseong": "개성재", "north": "도서관 북문", "edu": "사범대 앞", "south": "도서관 남문", "yang": "양성재", "back": "후문 안쪽"}
TIGHT = {r for r in K.RLEN if K.RJUNC.get(r, -1) != -1 and r in N.KEPT and N.min_rad(r) < 3.77}
FOLD = {r for r in N.KEPT if K.RJUNC.get(r, -1) != -1 and N.min_rad(r) < 1.5}
ALLOW = {int(x) for x in os.environ.get("ALLOW", "").split(",") if x}   # 반경 기준 밑이지만 허용해 보는 연결로(참고 판)
BANNED = frozenset((TIGHT | FOLD) - ALLOW)


def seg(a, b):
    ra, sa = a[1], a[2]; rb, sb = b[1], b[2]
    if ra == rb and sb > sa: return [ra], sb - sa
    if ra == rb:
        best = None
        for v in N.G[ra]:
            if v in BANNED: continue
            p = N.sp(v, rb, BANNED)
            if p and (best is None or N.plen([ra] + p) < N.plen(best)): best = [ra] + p
        if best is None: return None, None
        return best, N.LEN[ra] - sa + sum(N.LEN[r] for r in best[1:-1]) + sb
    p = N.sp(ra, rb, BANNED)
    if p is None: return None, None
    return p, N.LEN[ra] - sa + sum(N.LEN[r] for r in p[1:-1]) + sb


def tour(stops):
    roads, L, segs = [], 0.0, []
    for x, y in zip(stops, stops[1:] + stops[:1]):
        p, l = seg(x, y)
        if p is None: return None
        segs.append(dict(frm=x[0], to=y[0], roads=p, start_s=round(x[2], 2), end_s=round(y[2], 2), length=round(l, 1)))
        roads += p if not roads else p[1:]; L += l
    return dict(stops=[s[0] for s in stops], length=round(L, 1), segs=segs, roads=roads)


keys = [k for k in ["main", "gaeseong", "north", "edu", "south", "yang", "back"] if not (k == "back" and os.environ.get("NO_BACK"))]
best = []
for perm in itertools.permutations(keys[1:]):
    order = ["main"] + list(perm)
    for sides in itertools.product(*[STOP[k] for k in order]):
        t = tour(list(sides))
        if t: best.append(t)
best.sort(key=lambda t: t["length"])
T = best[0]
rs = set(T["roads"])
uturns = [r for i, r in enumerate(T["roads"][:-1]) if r == 1415 and T["roads"][i + 1] == 1409]
# 큰 도로 상태
def reach(start, G):
    seen = {start}; st = [start]
    while st:
        u = st.pop()
        for v in G[u]:
            if v in N.KEPT and v not in seen: seen.add(v); st.append(v)
    return seen
scc = reach(1409, N.G) & reach(1409, N.GR)
big = {}
for w in sorted(N.big_w):
    roads = [r for r in N.W2R[w] if r in K.RLEN]
    kept = [r for r in roads if r in N.KEPT]
    di = N.dirinfo.get(w, {})
    fk = [r for r in di.get("fwd", []) if r in N.KEPT]; bk = [r for r in di.get("bwd", []) if r in N.KEPT]
    Lk = sum(K.RLEN[r] for r in kept) or 1
    frac = sum(K.RLEN[r] for r in kept if r in scc) / Lk
    big[w] = dict(roads=roads, kept=kept, two_way=bool(fk and bk), in_loop=bool(kept) and frac >= 0.8, loop_frac=round(frac, 2),
                  tails_out=[r for r in kept if r not in scc], on_route=any(r in rs for r in kept),
                  oneway_osm=N.wayinfo[w]["oneway"], length=N.wayinfo[w]["length"])
okw = [w for w, v in big.items() if v["two_way"] and v["in_loop"]]
res = dict(veh=VEH, best=T, top5=[dict(stops=t["stops"], length=t["length"]) for t in best[:5]], uturn_count=len(uturns), big=big, big_ok=okw,
           banned_tight=len(TIGHT), stops=STOP)
res["allow"] = sorted(ALLOW); res["no_back"] = bool(os.environ.get("NO_BACK"))
(OUT / os.environ.get("LOOP_OUT", "loop_v33a.json")).write_text(json.dumps(res, ensure_ascii=False, indent=1))
print("가장 짧은 고리", T["length"], " > ".join(T["stops"]), "유턴", len(uturns))
for s_ in T["segs"]: print("  ", s_["frm"], "->", s_["to"], s_["length"])
print("큰 도로", len(big), "양방향·고리 안", len(okw), "노선이 지남", sum(1 for v in big.values() if v["on_route"]))
for w, v in big.items():
    if w not in okw: print("  안 됨", w, v["length"], "양방향" if v["two_way"] else "한 방향", f"고리 안 {v['loop_frac']}", "OSM oneway" if v["oneway_osm"] else "", "끝 조각", v["tails_out"])
print("고리 안이지만 끝 조각(막다른 끝)이 밖인 것", {w: v["tails_out"] for w, v in big.items() if w in okw and v["tails_out"]})
# 그림
FONT = subprocess.run(["fc-match", "-f", "%{file}", "Noto Sans CJK KR"], capture_output=True, text=True).stdout
fm.fontManager.addfont(FONT); plt.rcParams["font.family"] = fm.FontProperties(fname=FONT).get_name()
fig, ax = plt.subplots(figsize=(13, 12)); N.draw_base(ax, alpha=0.8)
for w, v in big.items():
    col = "#f3c9b5" if w in okw else "#c4b5fd"
    for r in v["kept"]:
        Q = K.road_line(r, step=3.0)
        if len(Q) > 1: xs, ys = zip(*Q); ax.plot(xs, ys, color=col, lw=4, zorder=2)
for r in T["roads"]:
    Q = K.road_line(r, step=2.0)
    if len(Q) > 1: xs, ys = zip(*Q); ax.plot(xs, ys, color="#eb6834", lw=2.6, zorder=4)
for r in T["roads"][::5]:
    Q = K.road_line(r, step=2.0)
    if len(Q) > 3: i = len(Q) // 2; ax.annotate("", xy=Q[i + 1], xytext=Q[i - 1], arrowprops=dict(arrowstyle="-|>", color="#111"), zorder=6)
for k, sel in STOP.items():
    used = [s for s in sel if s[0] in T["stops"]]
    for pid, r, s in used:
        b = P[pid]["best"]["door_pose"] if pid in P else PL1[pid]["door_pose"]
        ax.plot(b["x"], b["y"], "o", color="#1a9e5f", ms=10, zorder=8)
        ax.text(b["x"] + 6, b["y"] + 6, f"{T['stops'].index(pid) + 1}. {NAME[k]}", fontsize=10, color="#0b5d36", zorder=9,
                bbox=dict(fc="white", ec="none", alpha=0.7, pad=0.5))
Q = K.road_line(1415, step=1.0)[-1]; ax.plot(*Q, "o", ms=20, mfc="none", mec="#0b5d36", mew=2.5, zorder=9); ax.text(Q[0] + 12, Q[1] - 6, "유턴(북문 길 끝)", fontsize=9)
from matplotlib.lines import Line2D
ax.legend(handles=[Line2D([], [], color="#eb6834", lw=2.6, label="순환 노선(한 바퀴)"), Line2D([], [], color="#f3c9b5", lw=4, label="큰 도로: 양방향·고리 안"),
                   Line2D([], [], color="#c4b5fd", lw=4, label="큰 도로: 한 방향이거나 막다른 곳")], loc="lower left", fontsize=10)
ax.set_aspect("equal"); ax.set_xlim(150, 1180); ax.set_ylim(80, 1080); ax.set_xticks([]); ax.set_yticks([])
ax.set_title(f"v3.3a 순환 노선 후보: 한 바퀴 {T['length']:.0f}m, 정류장 {len(T['stops'])}곳, 유턴 {len(uturns)}번(북문 길 끝), 연결로 반경 3.77m 밑 안 씀", fontsize=11)
fig.tight_layout(); fig.savefig(OUT / ("figs/" + os.environ.get("LOOP_FIG", "loop_v33a.png")), dpi=105); print(OUT / "figs" / os.environ.get("LOOP_FIG", "loop_v33a.png"))


# ---- 덮기 노선: 양방향·고리 안 큰 도로를 모두 한 번은 지나게 경유점을 끼움(경유점 = 그 way 의 road 가운데) ----
def cover(stops):
    stops = list(stops); added = []
    while True:
        t = tour(stops); rs_ = set(t["roads"])
        miss = [w for w in okw if not any(r in rs_ for r in big[w]["kept"])]
        if not miss: break
        cand = None
        for w in miss:
            for r in big[w]["kept"]:
                if r not in scc or r in BANNED: continue
                for i in range(1, len(stops) + 1):
                    ss = stops[:i] + [(f"via_r{r}", r, N.LEN[r] / 2)] + stops[i:]
                    tt = tour(ss)
                    if tt and (cand is None or tt["length"] < cand[0]): cand = (tt["length"], ss, w)
        if cand is None: break
        stops = cand[1]; added.append(cand[2])
    t = tour(stops); t["via_ways"] = added
    t["missed"] = [w for w in okw if not any(r in set(t["roads"]) for r in big[w]["kept"])]
    return t


base = [next(s for v in STOP.values() for s in v if s[0] == pid) for pid in T["stops"]]
TC = cover(base)
res["cover"] = TC
(OUT / os.environ.get("LOOP_OUT", "loop_v33a.json")).write_text(json.dumps(res, ensure_ascii=False, indent=1))
print("덮기 노선", TC["length"], "경유점", len([s for s in TC["stops"] if s.startswith("via")]), "못 덮은 큰 도로", TC["missed"])
fig, ax = plt.subplots(figsize=(13, 12)); N.draw_base(ax, alpha=0.8)
for w, v in big.items():
    col = "#f3c9b5" if w in okw else "#c4b5fd"
    for r in v["kept"]:
        Q = K.road_line(r, step=3.0)
        if len(Q) > 1: xs, ys = zip(*Q); ax.plot(xs, ys, color=col, lw=4, zorder=2)
for r in TC["roads"]:
    Q = K.road_line(r, step=2.0)
    if len(Q) > 1: xs, ys = zip(*Q); ax.plot(xs, ys, color="#7c3aed", lw=2.4, zorder=4)
for r in TC["roads"][::5]:
    Q = K.road_line(r, step=2.0)
    if len(Q) > 3: i = len(Q) // 2; ax.annotate("", xy=Q[i + 1], xytext=Q[i - 1], arrowprops=dict(arrowstyle="-|>", color="#111"), zorder=6)
ax.set_aspect("equal"); ax.set_xlim(150, 1180); ax.set_ylim(80, 1080); ax.set_xticks([]); ax.set_yticks([])
ax.set_title(f"v3.3a 덮기 노선(양방향·고리 안 큰 도로 {len(okw)}개를 모두 지나게 경유점): 한 바퀴 {TC['length']:.0f}m, 못 덮은 것 {len(TC['missed'])}", fontsize=11)
fig.tight_layout(); fig.savefig(OUT / ("figs/" + os.environ.get("LOOP_FIG", "loop_v33a.png").replace(".png", "_cover.png")), dpi=100)
