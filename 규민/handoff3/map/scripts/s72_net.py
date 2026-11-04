#!/usr/bin/env python3
# 11/02 나(순환, 계획): 큰 도로 기준·목록, 양방향·막다른 끝·U턴·급커브, 순환 노선 후보. 서버 없음, 지도 파일은 읽기만
# 그래프: Lanelet2 28a57711 빌드 메타(map/build_s57_regen/v31/l1_meta.json)의 road 단위 연결(xodr road 하나 = 한 방향 차로 하나)
#   남은 road = meta lanelet 키(U턴 연결로·s55 r1188·r1318 뺌). 비용 = 차로 길이(lane_len). Lanelet2 최단 경로와 같은 road 열이 나오는지 30쌍으로 확인
# 큰 도로 기준(안): OSM 원래 태그는 캠퍼스 안이 거의 다 service/parking_aisle 이라 등급으로는 못 가름. 그래서
#   (1) OSM way 가 주차장 면(amenity=parking) 안에 70% 넘게 있지 않음 (2) way 길이 100m 이상 (3) 막다른 way 가 아님(양 끝이 다른 차도 way 와 이어짐)
#   비교용 '30쌍이 쓰는 road' 도 같이 냄
# 출력: integration/logs/s72/net.json, figs/big_roads.png, figs/loop_*.png
# 사용: .venv-carla/bin/python map/scripts/s72_net.py
import collections, heapq, json, math, subprocess
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager as fm  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from shapely.geometry import LineString, Polygon  # noqa: E402
from shapely.ops import unary_union  # noqa: E402
import s72_common as K  # noqa: E402

OUT = K.C / "integration/logs/s72"; (OUT / "figs").mkdir(parents=True, exist_ok=True)
FONT = subprocess.run(["fc-match", "-f", "%{file}", "Noto Sans CJK KR"], capture_output=True, text=True).stdout
fm.fontManager.addfont(FONT); plt.rcParams["font.family"] = fm.FontProperties(fname=FONT).get_name()
META = json.loads((K.C / "map/build_s57_regen/v31/l1_meta.json").read_text())
PAIRS = json.loads((K.BASE / "build_v31_s54/pairs30.json").read_text())
key = lambda r: f"{r},0,-1"
KEPT = {int(k.split(",")[0]) for k in META["lanelet"]} - set(META["s55_dropped_roads"])
import os
S72_BAN = {int(x) for x in os.environ.get("S72_BAN", "").split(",") if x}   # 11/02 보충: 볼라드·개성재 안쪽 등 뺄 road(지도는 안 바꿈, 그래프에서만)
KEPT -= S72_BAN
KEPT |= {int(x) for x in os.environ.get("S72_RESTORE", "").split(",") if x}   # 11/02: v3.3a 에서 되살린 road(r1318·r1188)
LEN = {int(k.split(",")[0]): v for k, v in META["lane_len"].items()}
G = collections.defaultdict(set); GR = collections.defaultdict(set)
for a, b in META["edges"]:
    ra, rb = int(a.split(",")[0]), int(b.split(",")[0])
    if ra in KEPT and rb in KEPT:
        G[ra].add(rb); GR[rb].add(ra)
UTURN = {u[0] for u in META["uturn"]}
# 11/02 보충: 현장에서 차를 돌리는 자리(만들어야 하는 유턴 레인렛). 예: S72_UTURN=1415:1409 (북문 길 끝, 사범대 앞)
ADDED_UTURN = [tuple(int(v) for v in x.split(":")) for x in os.environ.get("S72_UTURN", "").split(",") if x]
for a, b in ADDED_UTURN:
    G[a].add(b); GR[b].add(a)


def sp(a, b, banned=frozenset()):
    """road a(출발, 처음부터) -> road b 최단 road 열. a == b 면 [a]"""
    if a == b: return [a]
    dist = {a: 0.0}; prev = {}; pq = [(0.0, a)]
    while pq:
        d, u = heapq.heappop(pq)
        if u == b: break
        if d > dist.get(u, 1e18): continue
        for v in G[u]:
            if v in banned: continue
            nd = d + LEN.get(u, 0)
            if nd < dist.get(v, 1e18): dist[v] = nd; prev[v] = u; heapq.heappush(pq, (nd, v))
    if b not in dist: return None
    out = [b]
    while out[-1] != a: out.append(prev[out[-1]])
    return out[::-1]


def plen(path, s0=0.0, s1=None):
    if not path: return None
    L = sum(LEN[r] for r in path) - s0
    if s1 is not None: L -= LEN[path[-1]] - s1
    return L


# 30쌍 확인(원래 road 열과 같은지)
same30 = sum(1 for p in PAIRS.values() if sp(p["roads"][0], p["roads"][-1]) == p["roads"])
# OSM way: 길이, 주차장 면 안 비율, 막다른 끝
park = unary_union([Polygon(pts).buffer(0) for _, pts, t in K.osm_ways(lambda t: t.get("amenity") == "parking") if len(pts) >= 4 and pts[0] == pts[-1]])
hw_ways = {wid: (pts, t) for wid, pts, t in K.osm_ways(lambda t: t.get("highway") in ("service", "unclassified", "residential", "tertiary", "secondary", "primary", "track", "living_street"))}
node_use = collections.Counter()
for wid in hw_ways:
    for n in K.WAYS[wid][0]: node_use[n] += 1
W2R = collections.defaultdict(list)
for r, w in K.R2W.items():
    if r in K.RLEN: W2R[w].append(r)
wayinfo = {}
for w, rs in W2R.items():
    if w not in hw_ways: continue
    pts, t = hw_ways[w]; L = LineString(pts)
    nd = K.WAYS[w][0]
    dead = sum(1 for n in (nd[0], nd[-1]) if node_use[n] <= 1 and nd[0] != nd[-1])
    inpark = L.intersection(park).length / L.length if L.length else 0
    wayinfo[w] = dict(length=round(L.length, 1), in_parking=round(inpark, 2), dead_ends=dead, roads=sorted(rs), highway=t.get("highway"), service=t.get("service", ""),
                      oneway=t.get("oneway", ""))
big_w = {w for w, v in wayinfo.items() if v["in_parking"] <= 0.7 and v["length"] >= 100 and v["dead_ends"] == 0}
used30 = {r for p in PAIRS.values() for r in p["roads"]}
big_r = {r for w in big_w for r in W2R[w] if r in K.RLEN}
# 큰 도로마다: 양방향(그 way 의 fwd·bwd road 가 모두 Lanelet2 에 있는지), 막다른 끝(남은 그래프에서 나가는 연결 없음), U턴만 남는 끝
dirinfo = {}
import csv
WD = {r["way"]: r for r in csv.DictReader(open(K.BASE / "docs/logs/s42_way_dir.csv"))}
for w in sorted(big_w):
    d = WD.get(w, {}); f = [int(x) for x in d.get("roads_fwd", "").split()]; b = [int(x) for x in d.get("roads_bwd", "").split()]
    fk = [r for r in f if r in KEPT]; bk = [r for r in b if r in KEPT]
    dirinfo[w] = dict(fwd=f, bwd=b, fwd_kept=fk, bwd_kept=bk, both=bool(fk) and bool(bk), oneway_osm=wayinfo[w]["oneway"])
dead_roads = sorted(r for r in KEPT if K.RJUNC.get(r, -1) == -1 and not G[r])
dead_big = [r for r in dead_roads if r in big_r]
dead_in = sorted(r for r in KEPT if K.RJUNC.get(r, -1) == -1 and not GR[r])


# 급커브: 남은 교차로 연결로 lanelet 중심선 최소 반경(xodr 차로 중심선 1m 간격, 세 점 원)
def min_rad(rid):
    P = K.road_line(rid, step=0.5)
    best = 1e9
    for i in range(2, len(P) - 2):
        (x1, y1), (x2, y2), (x3, y3) = P[i - 2], P[i], P[i + 2]
        a = math.dist((x1, y1), (x2, y2)); b = math.dist((x2, y2), (x3, y3)); c = math.dist((x1, y1), (x3, y3))
        area = abs((x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1)) / 2
        if area > 1e-6: best = min(best, a * b * c / (4 * area))
    return best


junc_on_big = sorted({v for r in big_r for v in G[r] if K.RJUNC.get(v, -1) != -1 and any(x in big_r for x in G[v])})
radii = {r: round(min_rad(r), 2) for r in junc_on_big}
sharp = {r: v for r, v in radii.items() if v < 6.0}
json.dump(dict(same30=same30, n_kept=len(KEPT), big_ways=sorted(big_w), big_roads=sorted(big_r), big_len_m=round(sum(K.RLEN[r] for r in big_r), 1),
               used30_len_m=round(sum(K.RLEN[r] for r in used30 if K.RJUNC.get(r, -1) == -1), 1), wayinfo=wayinfo, dirinfo=dirinfo, dead_roads=dead_roads,
               dead_big=dead_big, dead_in=dead_in, junc_radii=radii, sharp=sharp, uturn_dropped=sorted(UTURN)),
          open(OUT / "net.json", "w"), ensure_ascii=False, indent=1)
print("30쌍 같음", same30, "/30, 남은 road", len(KEPT))
print("큰 도로 way", len(big_w), "road", len(big_r), "길이", round(sum(K.RLEN[r] for r in big_r)), "m(양방향 합)")
print("한쪽만", [w for w, v in dirinfo.items() if not v["both"]])
print("막다른 끝 road(큰 도로)", dead_big, "/ 전체", len(dead_roads))
print("급커브(<6m) 연결로", sharp)


# ---- 그림: 큰 도로 기준 결과 ----
def draw_base(ax, alpha=1.0):
    for r in K.RLEN:
        if r not in KEPT: continue
        P = K.road_line(r, step=3.0)
        if len(P) < 2: continue
        xs, ys = zip(*P); ax.plot(xs, ys, color="#d6d5d0", lw=1.0, zorder=1, alpha=alpha)
    for wid, pts, t in K.osm_ways(lambda t: "building" in t):
        xs, ys = zip(*pts); ax.fill(xs, ys, fc="none", ec="#bdb6aa", lw=0.4, zorder=0)


STOPS = {"정문": (263, 966), **({"개성재(자리 정할 것)": (640, 905)} if S72_BAN else {"중문(지금)": (620, 906)}), "도서관 북문": (657, 684), "도서관 남문": (570, 564), "양성재": (214, 551), "후문": (1100, 222)}
if __name__ == "__main__":
    fig, ax = plt.subplots(figsize=(13, 13)); draw_base(ax)
    for r in used30:
        if r not in K.RLEN: continue
        P = K.road_line(r, step=3.0)
        if len(P) > 1: xs, ys = zip(*P); ax.plot(xs, ys, color="#9cc3f0", lw=6, zorder=2, solid_capstyle="round")
    for w in big_w:
        col = "#eb6834" if dirinfo[w]["both"] else "#7c3aed"
        for r in W2R[w]:
            P = K.road_line(r, step=3.0)
            if len(P) > 1: xs, ys = zip(*P); ax.plot(xs, ys, color=col, lw=2.2, zorder=3)
        pts, _ = hw_ways[w]; m = pts[len(pts) // 2]
        ax.text(m[0], m[1], f"w{w}", fontsize=6, color="#5a2a0a", zorder=6, bbox=dict(fc="white", ec="none", alpha=0.6, pad=0.3))
    for r in dead_big:
        P = K.road_line(r, step=1.0); ax.plot(*P[-1], "X", color="#c00", ms=9, zorder=7); ax.text(P[-1][0] + 4, P[-1][1], f"r{r} 끝", fontsize=7, color="#c00")
    # 11/02 보충: 정문 쪽 본선(r1392)에서 들어가 정문 쪽(r1255)으로 나올 수 있는지. 못 하면 굵은 점선(막다른 곳)
    trap = sorted(r for r in big_r if r in KEPT and not (sp(1392, r) and sp(r, 1255)))
    for r in trap:
        P = K.road_line(r, step=3.0)
        if len(P) > 1: xs, ys = zip(*P); ax.plot(xs, ys, color="#c00", lw=4, ls=(0, (1, 1.2)), zorder=6, alpha=0.8)
    print("들어가면 못 나오는(또는 못 들어가는) 큰 도로 road", len(trap), trap)
    for r, v in sharp.items():
        P = K.road_line(r, step=1.0); m = P[len(P) // 2]
        if v < 1.5:
            ax.plot(*m, "D", color="#000", ms=7, zorder=8); ax.text(m[0] + 3, m[1] - 6, f"r{r} 접힘 {v}m", fontsize=7)
    for n, (x, y) in STOPS.items():
        ax.plot(x, y, "o", color="#1a9e5f", ms=9, zorder=9); ax.text(x + 6, y + 6, n, fontsize=10, color="#0b5d36", zorder=9)
    from matplotlib.lines import Line2D
    ax.legend(handles=[Line2D([], [], color="#eb6834", lw=2.2, label="큰 도로(안) 양방향 모두 Lanelet2 에 있음"),
                       Line2D([], [], color="#7c3aed", lw=2.2, label="큰 도로(안) 한 방향만 있음"),
                       Line2D([], [], color="#9cc3f0", lw=6, label="30쌍이 쓰는 road(비교)"),
                       Line2D([], [], color="#c00", marker="X", ls="", label="큰 도로 막다른 끝(남은 연결 없음)"),
                       Line2D([], [], color="#000", marker="D", ls="", label="접힌 연결로(반경 1.5m 밑)"),
                       Line2D([], [], color="#c00", lw=4, ls=(0, (1, 1.2)), label="본선에서 들어갔다 나올 수 없는 road(U턴 필요)")]
              + ([Line2D([], [], color="#000", lw=2.5, ls=(0, (2, 1.5)), label="뺀 road(볼라드·개성재 게이트 안·북쪽 고리, 그래프에서만)")] if S72_BAN else []), loc="lower left", fontsize=9)
    ax.set_aspect("equal"); ax.set_xlim(-30, 1180); ax.set_ylim(60, 1120); ax.grid(color="#eee", lw=0.5)
    if S72_BAN:
        for r in S72_BAN:
            P = K.road_line(r, step=1.0)
            if len(P) > 1: xs, ys = zip(*P); ax.plot(xs, ys, color="#000", lw=2.5, ls=(0, (2, 1.5)), zorder=7)
        P = K.road_line(1320, step=1.0); ax.plot(*P[6], "X", color="#000", ms=11, mew=2.5, zorder=9)
        for a, b in ADDED_UTURN:
            Q = K.road_line(a, step=1.0)[-1]
            ax.plot(*Q, "o", ms=22, mfc="none", mec="#0b5d36", mew=2.5, zorder=9)
            ax.text(Q[0] + 12, Q[1] - 4, "유턴 자리(만들어야 함)", fontsize=9, color="#0b5d36", zorder=9)
        ax.text(P[6][0] + 8, P[6][1] + 8, "볼라드", fontsize=9, zorder=9)
    ax.set_title(f"큰 도로 기준 안(주차장 면 안 70% 이하·way 100m 이상·막다른 way 아님): way {len(big_w)}개, road {len(big_r)}개"
                 + (" / 11/02 보충 S-E 반영" if S72_BAN else ""), fontsize=11)
    fig.tight_layout(); fig.savefig(OUT / "figs/big_roads.png", dpi=110); print(OUT / "figs/big_roads.png")
