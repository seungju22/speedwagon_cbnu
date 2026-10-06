#!/usr/bin/env python3
# 맵세션19 Phase 1: 재변환 범위 확정용 분석 (읽기전용, 서버 없음, xodr/OSM 미수정).
# 1-1 양성재 도달 가능성(frozen_v1), 1-2 회차·루프(방향 그래프, U턴 커넥터 제외 여부),
# 후문은 현재 맵에서 도달 불가라 세션18 시험 재변환 맵(_trial_s18_gates, 채택 안 된 산출물)으로 조사.
# 방향 그래프: 각 road 의 lane -1 끝에서 next(0.05) 로 도착하는 road (역방향 불가).
# 실행: .venv-carla/bin/python map/scripts/s19_scope.py > map/docs/logs/s19_scope.log
import csv
import math
import sys

sys.path.insert(0, "/home/gyumin/campus_mobility_sim/map/scripts")
from s19_scope_common import (BASE, RAW_OSM, XODR_V1, XODR_TRIAL, load_osm, load_map,
                              road_lengths, build_graph, dijkstra, path_to, ll_to_carla,
                              turn_deg, nearest_lane)

UTURN_DEG = 135.0   # 앞뒤 접선 회전 |deg| 이 이상이면 U턴형 커넥터(j43 1612/1613 의 ±141 포함)
ORIGIN = 1247       # 정문 진입 road (frozen_v1)
STOPS_V1 = {"north": (1378, 78.0), "south": (1237, 47.0), "middle": (1327, 70.0)}


def seg_d(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy or 1)))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def poly_d(p, poly):
    return min(seg_d(p, poly[i], poly[i + 1]) for i in range(len(poly) - 1))


class Net:
    def __init__(self, path, label):
        self.label = label
        self.cmap = load_map(path)
        self.lens = road_lengths(path)
        self.g = build_graph(self.cmap, self.lens)
        self.pred = {r: set() for r in self.lens}
        for u, vs in self.g.items():
            for v in vs:
                self.pred[v].add(u)
        self.turn = {r: turn_deg(self.cmap, r, L) for r, (L, _) in self.lens.items()}
        self.nb = {}
        for r, (L, j) in self.lens.items():
            if j == -1 or not self.pred[r] or not self.g[r]:
                continue
            p, s = min(self.pred[r]), min(self.g[r])
            a = self.cmap.get_waypoint_xodr(p, -1, max(self.lens[p][0] - 1.0, self.lens[p][0] / 2))
            b = self.cmap.get_waypoint_xodr(s, -1, min(1.0, self.lens[s][0] / 2))
            if a is None or b is None:
                continue
            d = b.transform.rotation.yaw - a.transform.rotation.yaw
            self.nb[r] = (d + 180) % 360 - 180
        self.uturn = {r for r, d in self.nb.items() if abs(d) >= UTURN_DEG}
        self.r2 = {}

    def radius(self, r):
        """평균R = 길이/|끝점 회전(rad)|, R2min = 2m 창 곡률 최소 반경(세션11 정의, 0.5m 간격, 양끝 1m 제외)."""
        if r in self.r2:
            return self.r2[r]
        L = self.lens[r][0]
        tr = abs(math.radians(self.turn[r]))
        avg = L / tr if tr > 1e-6 else 1e6
        r2min, s = 1e6, 1.0
        while s <= L - 1.0:
            a = self.cmap.get_waypoint_xodr(r, -1, s - 1.0)
            b = self.cmap.get_waypoint_xodr(r, -1, s + 1.0)
            k = abs(((b.transform.rotation.yaw - a.transform.rotation.yaw + 180) % 360 - 180)) / 2.0
            if k > 1e-6:
                r2min = min(r2min, 1.0 / math.radians(k))
            s += 0.5
        self.r2[r] = (avg, r2min)
        return self.r2[r]

    def route_len(self, path, end_s):
        return sum(self.lens[r][0] for r in path[:-1]) + end_s

    def curves(self, path, end_s=None):
        out = []
        for i, r in enumerate(path):
            avg, r2 = self.radius(r)
            ang = self.nb.get(r, self.turn[r])
            if abs(ang) >= 30 or r2 < 9:
                if i == len(path) - 1 and end_s is not None and r2 < 9:
                    note = "(종점 road)"
                else:
                    note = ""
                out.append(f"r{r} {'j' + str(self.lens[r][1]) if self.lens[r][1] != -1 else '일반'} "
                           f"{ang:+.1f}° 평균R {avg:.2f} R2min {r2:.2f}{note}")
        return out

    def min_radius(self, path):
        """경로 위 커브(회전 20° 이상)의 최소 평균R 과 그 road."""
        best = None
        for r in path:
            avg, r2 = self.radius(r)
            if abs(self.nb.get(r, self.turn[r])) >= 20 and (best is None or avg < best[1]):
                best = (r, avg, r2)
        return best

    def shortest(self, src, dst, banned=frozenset(), src_s=0.0):
        rest = self.lens[src][0] - src_s
        dist, prev = dijkstra(self.g, self.lens, src, banned, rest)
        if dst not in dist:
            return None, None
        return path_to(prev, src, dst), dist[dst]


def twin_of(net, rid, s):
    """rid 의 s 지점과 같은 자리에서 반대 방향인 반쪽 도로(road, s)."""
    w = net.cmap.get_waypoint_xodr(rid, -1, s)
    loc, yaw = w.transform.location, w.transform.rotation.yaw
    best = None
    for r, (L, j) in net.lens.items():
        if j != -1 or r == rid:
            continue
        t = 0.0
        while t <= L:
            q = net.cmap.get_waypoint_xodr(r, -1, t)
            d = math.hypot(q.transform.location.x - loc.x, q.transform.location.y - loc.y)
            dy = abs((q.transform.rotation.yaw - yaw + 180) % 360 - 180)
            if d < 8 and dy > 150 and (best is None or d < best[2]):
                best = (r, t, d)
            t += 1.0
    return best


def reverse_reach(net, dst):
    seen, st = {dst}, [dst]
    while st:
        u = st.pop()
        for p in net.pred[u]:
            if p not in seen:
                seen.add(p)
                st.append(p)
    return seen


def stop_matrix(net, stops):
    """정류장 사이 이동: U턴 금지 최단거리(없으면 U턴 허용 거리와 쓴 U턴 커넥터)."""
    print(f"  [{net.label}] 정류장 간 이동 (행=출발, U턴금지 / 없으면 U턴허용)")
    for a, (ra, sa) in stops.items():
        cells = []
        for b, (rb, sb) in stops.items():
            if a == b:
                continue
            if ra == rb and sb > sa:
                cells.append(f"{b} {sb - sa:.0f}m")
                continue
            pb, db = net.shortest(ra, rb, banned=frozenset(net.uturn), src_s=sa)
            if pb:
                cells.append(f"{b} {db - net.lens[rb][0] + sb:.0f}m")
                continue
            pa, da = net.shortest(ra, rb, src_s=sa)
            if pa:
                cells.append(f"{b} U턴필요 {da - net.lens[rb][0] + sb:.0f}m "
                             f"{[x for x in pa if x in net.uturn]}")
            else:
                cells.append(f"{b} 불가")
        print(f"    {a}: " + " | ".join(cells))


def osm_graph(cw, nodes, extra):
    """변환되는 way(unclassified 73) + extra 로 방향 간선 목록. oneway=yes 는 한 방향."""
    edges = {}
    for wid, w in cw.items():
        if w["tags"].get("highway") != "unclassified" and wid not in extra:
            continue
        nd = w["nds"]
        ow = w["tags"].get("oneway") == "yes"
        for a, b in zip(nd, nd[1:]):
            pa, pb = ll_to_carla(*nodes[a]), ll_to_carla(*nodes[b])
            d = math.hypot(pa[0] - pb[0], pa[1] - pb[1])
            edges.setdefault(a, []).append((b, d, wid))
            if not ow:
                edges.setdefault(b, []).append((a, d, wid))
    return edges


def _hd(a, b, nodes):
    pa, pb = ll_to_carla(*nodes[a]), ll_to_carla(*nodes[b])
    return math.degrees(math.atan2(pb[1] - pa[1], pb[0] - pa[0]))


def osm_path(edges, src, dst, allow_reverse=False, nodes=None, start_yaw=None):
    """간선 상태 Dijkstra. allow_reverse=False 면 U턴 금지: 바로 되돌아가기(u->v->u)와
    진행 방향이 UTURN_DEG 이상 꺾이는 연결(평행 일방통행 way 끼리 갈아타기 포함)."""
    import heapq
    pq, best = [], {}
    for v, d, wid in edges.get(src, ()):
        if start_yaw is not None and nodes is not None:
            if abs((_hd(src, v, nodes) - start_yaw + 180) % 360 - 180) >= 90:
                continue   # 정차 방향과 반대로 출발 = 제자리 U턴
        heapq.heappush(pq, (d, src, v, wid, (src, v)))
    while pq:
        d, u, v, wid, trail = heapq.heappop(pq)
        if (u, v) in best:
            continue
        best[(u, v)] = d
        if v == dst:
            return d, trail
        for w, dd, wid2 in edges.get(v, ()):
            if not allow_reverse:
                if w == u:
                    continue
                if nodes is not None:
                    dh = abs((_hd(v, w, nodes) - _hd(u, v, nodes) + 180) % 360 - 180)
                    if dh >= UTURN_DEG:
                        continue
            if (v, w) not in best:
                heapq.heappush(pq, (d + dd, v, w, wid2, trail + (w,)))
    return None, None


def core_scc(net):
    """U턴형 커넥터를 뺀 방향 그래프의 가장 큰 강연결 성분(= U턴 없이 서로 오갈 수 있는 도로망)."""
    ban = net.uturn
    g = {u: [v for v in vs if v not in ban] for u, vs in net.g.items() if u not in ban}
    idx, low, on, st, comps, c = {}, {}, set(), [], [], [0]
    sys.setrecursionlimit(20000)

    def dfs(u):
        idx[u] = low[u] = c[0]
        c[0] += 1
        st.append(u)
        on.add(u)
        for v in g.get(u, ()):
            if v not in idx:
                dfs(v)
                low[u] = min(low[u], low[v])
            elif v in on:
                low[u] = min(low[u], idx[v])
        if low[u] == idx[u]:
            comp = []
            while True:
                w = st.pop()
                on.discard(w)
                comp.append(w)
                if w == u:
                    break
            comps.append(comp)
    for u in g:
        if u not in idx:
            dfs(u)
    return set(max(comps, key=len))


def fmt_path(p):
    return ",".join(str(r) for r in p)


def main():
    nodes, ways = load_osm(RAW_OSM)
    v1 = Net(XODR_V1, "frozen_v1")
    print(f"# s19 scope 로그. UTURN_DEG={UTURN_DEG}")
    print(f"[v1] road {len(v1.lens)} / edge {sum(len(v) for v in v1.g.values())} / "
          f"U턴형 커넥터 {len(v1.uturn)}")
    for r in (1915, 1449, 1429, 1697, 1612, 1613, 1770, 1827):
        print(f"  r{r} j{v1.lens[r][1]} 길이 {v1.lens[r][0]:.2f} 자체회전 {v1.turn[r]:+.1f} "
              f"앞뒤접선 {v1.nb.get(r, float('nan')):+.1f} U턴형={r in v1.uturn}")

    # ---------- 1-1 양성재 ----------
    print("\n## 1-1 양성재 (frozen_v1)")
    Y = [ll_to_carla(*nodes[n]) for n in ways["442760311"]["nds"]]
    cy = (sum(p[0] for p in Y[:-1]) / (len(Y) - 1), sum(p[1] for p in Y[:-1]) / (len(Y) - 1))
    print(f"양성재 way442760311 building=university, 중심 CARLA ({cy[0]:.1f},{cy[1]:.1f}) "
          f"TM ({cy[0] - 521.51:.1f},{-cy[1] - 493.61:.1f})")
    dist, prev = dijkstra(v1.g, v1.lens, ORIGIN, src_rest=v1.lens[ORIGIN][0])
    cands = []
    for r, (L, j) in v1.lens.items():
        if j != -1:
            continue
        s, best = 0.0, None
        while s <= L:
            q = v1.cmap.get_waypoint_xodr(r, -1, s).transform.location
            d = poly_d((q.x, q.y), Y)
            if best is None or d < best[0]:
                best = (d, s)
            s += 1.0
        if best[0] < 60:
            cands.append((best[0], r, best[1]))
    for d, r, s in sorted(cands):
        if r in dist:
            p = path_to(prev, ORIGIN, r)
            banned_hit = [x for x in p if x in v1.uturn]
            print(f"  후보 r{r} s={s:.0f} 건물까지 {d:.1f}m 정문부터 {v1.route_len(p, s):.1f}m "
                  f"{len(p)} road U턴커넥터 {banned_hit or '없음'}")
        else:
            print(f"  후보 r{r} s={s:.0f} 건물까지 {d:.1f}m 정문에서 도달 불가")
    STOP_Y = (1123, 22.0)
    p, _ = v1.shortest(ORIGIN, STOP_Y[0])
    q = v1.cmap.get_waypoint_xodr(STOP_Y[0], -1, STOP_Y[1])
    print(f"채택 후보 r{STOP_Y[0]} s={STOP_Y[1]} junction={q.is_junction} "
          f"건물까지 {poly_d((q.transform.location.x, q.transform.location.y), Y):.1f}m "
          f"정문부터 {v1.route_len(p, STOP_Y[1]):.1f}m {len(p)} road")
    print(f"  경로 {fmt_path(p)}")
    for c in v1.curves(p):
        print(f"  커브 {c}")
    print(f"  최소 반경 커브 {v1.min_radius(p)}")
    s_path, _ = v1.shortest(ORIGIN, STOPS_V1['south'][0])
    common = 0
    while common < min(len(p), len(s_path)) and p[common] == s_path[common]:
        common += 1
    print(f"  south 와 공통 앞부분 {common} road (~r{p[common - 1]}), 이후 j10 에서 "
          f"south 는 r{s_path[common]} / 양성재는 r{p[common]} (회전 {v1.nb.get(p[common], 0):+.1f}°)")

    # ---------- 1-2 회차·루프 (v1) ----------
    print("\n## 1-2 루프 (frozen_v1)")
    tw = twin_of(v1, ORIGIN, 5.0)
    print(f"정문 진입 r{ORIGIN} 의 반대 방향 반쪽(나가는 쪽): r{tw[0]} s={tw[1]:.0f} ({tw[2]:.2f}m)")
    print(f"r{ORIGIN} 선행 road: {sorted(v1.pred[ORIGIN])} "
          f"(각 앞뒤접선 {[round(v1.nb.get(r, v1.turn[r]), 1) for r in sorted(v1.pred[ORIGIN])]})")
    stops = dict(STOPS_V1)
    stops["yangseong"] = STOP_Y
    for name, (r, s) in stops.items():
        for target, tlabel in ((tw[0], "정문 나가는 쪽"), (ORIGIN, "정문 진입 r1247(순환 완성)")):
            pa, da = v1.shortest(r, target, src_s=s)
            pb, db = v1.shortest(r, target, banned=frozenset(v1.uturn), src_s=s)
            ua = [x for x in (pa or []) if x in v1.uturn]
            print(f"  {name} r{r} s={s:.0f} -> {tlabel}: U턴허용 "
                  f"{f'{da:.0f}m U턴 {ua}' if pa else '경로없음'} / U턴금지 "
                  f"{f'{db:.0f}m' if pb else '경로없음'}")
            if pb:
                print(f"    U턴금지 경로 {fmt_path(pb)}")
                for c in v1.curves(pb):
                    print(f"    커브 {c}")
    back = {r for r in v1.lens if r in reverse_reach(v1, ORIGIN)}
    print(f"r{ORIGIN} 에 도달할 수 있는 road 전체: {sorted(back)} -> 정문 구간은 들어오기만 가능")
    for r in (1914, 1915, 1380):
        print(f"  r{r} j{v1.lens[r][1]} 선행 {sorted(v1.pred[r])} 후속 {sorted(v1.g[r])} "
              f"앞뒤접선 {v1.nb.get(r, v1.turn[r]):+.1f}")
    stop_matrix(v1, stops)
    core = core_scc(v1)
    print(f"  U턴 없이 서로 오갈 수 있는 핵심 도로망(최대 강연결 성분): {len(core)} road / 전체 {len(v1.lens)}")
    for name, (r, s) in stops.items():
        print(f"    {name} r{r} 핵심망 안={r in core}")
    print(f"    정문 r{ORIGIN} 핵심망 안={ORIGIN in core}, 나가는 쪽 r{tw[0]} 핵심망 안={tw[0] in core}")

    # ---------- 1-2 후문 (시험 맵) ----------
    print("\n## 1-2 후문 루프 (시험 재변환 맵 _trial_s18_gates, 채택 안 됨)")
    tr = Net(XODR_TRIAL, "trial")
    idmap = {int(r["old_id"]): int(r["trial_id"]) for r in csv.DictReader(
        open(BASE / "docs/logs/s18_D_road_id_map.csv")) if r["trial_id"] and float(r["match_cost"]) < 0.05}
    t_origin = idmap[ORIGIN]
    print(f"[trial] road {len(tr.lens)} / edge {sum(len(v) for v in tr.g.values())} / U턴형 {len(tr.uturn)}; "
          f"정문 r{ORIGIN} -> r{t_origin}")
    gate_b = ll_to_carla(36.624701, 127.463656)
    tdist, tprev = dijkstra(tr.g, tr.lens, t_origin, src_rest=tr.lens[t_origin][0])
    best = None
    for r, (L, j) in tr.lens.items():
        if r not in tdist:
            continue
        s = 0.0
        while s <= L:
            q = tr.cmap.get_waypoint_xodr(r, -1, s).transform.location
            d = math.hypot(q.x - gate_b[0], q.y - gate_b[1])
            if best is None or d < best[0]:
                best = (d, r, s)
            s += 1.0
    print(f"후문B 경계점 최근접 도달 차선 r{best[1]} s={best[2]:.0f} ({best[0]:.1f}m) "
          f"junction={tr.lens[best[1]][1]} U턴형={best[1] in tr.uturn}")
    # 정차점: U턴 커넥터 앞의 일반 road 끝 쪽
    stop_r = best[1]
    if tr.lens[stop_r][1] != -1:
        pp = path_to(tprev, t_origin, stop_r)
        stop_r = [x for x in pp if tr.lens[x][1] == -1][-1]
    L = tr.lens[stop_r][0]
    stop_s = max(L - 5.0, L / 2)
    q = tr.cmap.get_waypoint_xodr(stop_r, -1, stop_s).transform.location
    pb_, _ = tr.shortest(t_origin, stop_r)
    print(f"후문 정차 후보 r{stop_r} s={stop_s:.1f} 경계점까지 {math.hypot(q.x - gate_b[0], q.y - gate_b[1]):.1f}m "
          f"정문부터 {tr.route_len(pb_, stop_s):.0f}m {len(pb_)} road")
    print(f"  경로 {fmt_path(pb_)}")
    for c in tr.curves(pb_):
        print(f"  커브 {c}")
    print(f"  r{stop_r} 후속 {sorted(tr.g[stop_r])} (앞뒤접선 "
          f"{[round(tr.nb.get(x, tr.turn[x]), 1) for x in sorted(tr.g[stop_r])]})")
    t_tw = twin_of(tr, t_origin, 5.0)
    for target, tlabel in ((t_tw[0], "정문 나가는 쪽"), (t_origin, "정문 진입(순환 완성)")):
        pa, da = tr.shortest(stop_r, target, src_s=stop_s)
        pb, db = tr.shortest(stop_r, target, banned=frozenset(tr.uturn), src_s=stop_s)
        ua = [x for x in (pa or []) if x in tr.uturn]
        print(f"  후문 -> {tlabel}: U턴허용 {f'{da:.0f}m U턴 {ua}' if pa else '경로없음'} / "
              f"U턴금지 {f'{db:.0f}m' if pb else '경로없음'}")
        for lab, pth in (("U턴허용", pa), ("U턴금지", pb)):
            if pth:
                print(f"    {lab} 경로 {fmt_path(pth)}")
                for c in tr.curves(pth):
                    print(f"    커브 {c}")
    t_stops = {"north": (idmap[1378], 78.0), "south": (idmap[1237], 47.0), "middle": (1382, 70.0),
               "yangseong": (idmap[STOP_Y[0]], STOP_Y[1]), "back": (stop_r, stop_s)}
    print(f"  시험 맵 정류장 번호: {t_stops} (middle 은 형상 변경 road, s18 D-3 대응 r1327->r1382)")
    stop_matrix(tr, t_stops)
    tcore = core_scc(tr)
    print(f"  시험 맵 핵심 도로망 {len(tcore)} road. 정류장 핵심망 안: "
          f"{ {k: r in tcore for k, (r, s) in t_stops.items()} }")
    bc = None
    for r in tcore:
        L = tr.lens[r][0]
        s = 0.0
        while s <= L:
            q = tr.cmap.get_waypoint_xodr(r, -1, s).transform.location
            d = math.hypot(q.x - gate_b[0], q.y - gate_b[1])
            if bc is None or d < bc[0]:
                bc = (d, r, s)
            s += 1.0
    print(f"  핵심망 위 후문 최근접 r{bc[1]} s={bc[2]:.0f} 경계점까지 {bc[0]:.1f}m junction={tr.lens[bc[1]][1]}")

    # ---------- OSM 수준 예측 (재변환 전, oneway 반영) ----------
    print("\n## 1-2 OSM 수준 예측 (변환 대상 way 방향 그래프, 되돌아가기=U턴 금지)")
    from s19_scope_common import CONV_OSM
    cn, cw = load_osm(CONV_OSM)
    BACK7 = ["452870644", "392632034", "481945510", "481943505", "481475019", "481943503", "481945509"]
    MID2 = ["446440352", "446440353"]
    EXIT = ["481950060"]
    GATE_PUBLIC, GATE_PLAZA, BACK_IN = "2261340221", "4748296080", "4727599760"
    stop_tf = {k: v1.cmap.get_waypoint_xodr(r, -1, s).transform for k, (r, s) in stops.items()}
    stop_xy = {k: tf.location for k, tf in stop_tf.items()}
    syaw = {k: tf.rotation.yaw for k, tf in stop_tf.items()}
    syaw["back"] = _hd(BACK_IN, "3958352335", cn)   # 후문은 게이트(밖) 쪽을 보고 정차

    def near_node(edges, loc):
        return min(edges, key=lambda n: math.hypot(ll_to_carla(*cn[n])[0] - loc.x,
                                                    ll_to_carla(*cn[n])[1] - loc.y))
    for label, extra in (("S0 현재(73)", []), ("S1 +후문7+중문2", BACK7 + MID2),
                         ("S2 S1+정문출구1", BACK7 + MID2 + EXIT)):
        e = osm_graph(cw, cn, set(extra))
        sn = {k: near_node(e, loc) for k, loc in stop_xy.items()}
        if BACK_IN in e:
            sn["back"] = BACK_IN
        print(f"  [{label}] 간선 {sum(len(v) for v in e.values())}")
        for a in sn:
            for tgt, tl in ((GATE_PUBLIC, "정문 입구(공도 접점)"),):
                d0, _ = osm_path(e, sn[a], tgt, nodes=cn, start_yaw=syaw[a])
                d1, _ = osm_path(e, sn[a], tgt, allow_reverse=True)
                print(f"    ({a} 시작 노드 {sn[a]}, 정차 방향 {syaw[a]:.0f}°)")
                print(f"    {a} -> {tl}: U턴금지 {f'{d0:.0f}m' if d0 else '없음'} / "
                      f"U턴허용 {f'{d1:.0f}m' if d1 else '없음'}")
        d0, _ = osm_path(e, GATE_PUBLIC, GATE_PUBLIC, nodes=cn)
        print(f"    정문 입구 -> 캠퍼스 -> 정문 입구 순환(U턴금지): {f'{d0:.0f}m' if d0 else '없음'}")
        d0, tr0 = osm_path(e, GATE_PLAZA, GATE_PLAZA, nodes=cn)
        print(f"    정문 광장 j1 노드 출발 -> 다시 j1 순환(U턴금지): {f'{d0:.0f}m' if d0 else '없음'}")
        if "back" in sn:
            d0, _ = osm_path(e, sn["back"], sn["back"], nodes=cn, start_yaw=syaw["back"])
            print(f"    후문 출발 -> 다시 후문 순환(U턴금지): {f'{d0:.0f}m' if d0 else '없음'}")

    # 종점형 확인: S2 에 미변환 service way 를 하나씩 더해도 U턴 없이 정문에 가는 길이 생기는가
    base = set(BACK7 + MID2 + EXIT)
    svc = [w for w, x in cw.items() if x["tags"].get("highway") == "service" and w not in base]
    e2 = osm_graph(cw, cn, base)
    sn2 = {k: near_node(e2, stop_xy[k]) for k in ("north", "middle")}
    sn2["back"] = BACK_IN
    print(f"  [종점형 전수] 미변환 service way {len(svc)}개를 S2 에 하나씩 추가")
    for k, n in sn2.items():
        hits = [w for w in svc if osm_path(osm_graph(cw, cn, base | {w}), n, GATE_PUBLIC,
                                           nodes=cn, start_yaw=syaw[k])[0]]
        print(f"    {k}: U턴 없이 정문 가는 길이 생기는 way {len(hits)}개 {hits}")

    # 후문 부근 새 도로의 방향 연결 구조
    near = sorted(r for r, (L, j) in tr.lens.items()
                  if r not in idmap.values() and j == -1)
    print(f"  시험 맵에서 새로 생긴 일반 road(대응표에 없는 번호) {len(near)}개: {near}")
    for r in near:
        a = tr.cmap.get_waypoint_xodr(r, -1, 0.0).transform.location
        print(f"    r{r} 길이 {tr.lens[r][0]:.1f} 시작({a.x:.0f},{a.y:.0f}) 선행 {sorted(tr.pred[r])} "
              f"후속 {sorted(tr.g[r])} 정문도달={r in tdist}")


if __name__ == "__main__":
    main()
