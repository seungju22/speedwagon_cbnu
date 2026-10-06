#!/usr/bin/env python3
# 맵세션22 Phase 0: 후문 방향 노선 후보 비교 (읽기전용, 서버 없음, xodr/OSM 미수정).
# - frozen_v1 에서 정문(road1247 s=0) -> 후문 최근접 도달점(road1898 s=5.0)
#   후보 A 남문 경유 / B 양성재 경유 / C 그 밖(k-최단, 북문·중문 회차 경유, 정류장 두 곳 경유)
# - 0-4 예측: 세션18 시험 재변환 맵(_trial_s18_gates, 채택 안 됨)에서 같은 비교, 종점 = 후문 정차 후보
# 방향 그래프: lane -1 끝에서 next() 로 도착하는 road (s19_scope_common). 역방향은 구조상 못 들어간다.
# 교차로 U턴형 커넥터(|회전| >= 135°)는 금지. 북문·중문 막다른 끝 회차 커넥터만 그 후보에서 허용.
# 실행: .venv-carla/bin/python map/scripts/s22_routes.py > map/docs/logs/s22_routes.log
import csv
import math
import sys

sys.path.insert(0, "/home/gyumin/campus_mobility_sim/map/scripts")
from s19_scope_common import BASE, XODR_V1, XODR_TRIAL  # noqa: E402
from s19_scope import Net  # noqa: E402

K_SHORTEST = 12
TURN_MIN_DEG = 20.0      # 이 이상 꺾이는 road 를 "회전" 으로 센다
R9 = 9.0                 # 세션18 잠정 경계(세션21: 통과/실패 경계 아님, 비교용)

V1 = {
    "origin": (1247, 0.0),
    "target": (1898, 5.0),
    "stops": {"북문": (1378, 78.0), "남문": (1237, 47.0), "양성재": (1123, 22.0), "중문": (1327, 70.0)},
    "turnback": {"북문": 1665, "중문": 1840},   # 막다른 끝 회차 커넥터(s19_scope.md)
}


def trial_cfg():
    m = {}
    with open(BASE / "docs/logs/s18_D_road_id_map.csv") as f:
        for row in csv.DictReader(f):
            if row["trial_id"]:
                m[int(row["old_id"])] = (int(row["trial_id"]), float(row["match_cost"]))
    st = {k: (m[r][0], s) for k, (r, s) in V1["stops"].items()}
    cost = {k: m[r][1] for k, (r, s) in V1["stops"].items()}
    return {
        "origin": (m[1247][0], 0.0),
        "target": (1171, 91.0),     # 시험 맵 후문 정차 후보(stops_status, s19_scope)
        "stops": st,
        "turnback": {"북문": m[1665][0], "중문": m[1840][0]},
        "match_cost": cost,
    }


class Router:
    def __init__(self, net):
        self.n = net

    def shortest(self, a, b, banned_nodes=frozenset(), banned_edges=frozenset(), allow=frozenset()):
        """a road 끝 -> b road 끝 까지 road 시퀀스(a,b 포함). U턴 커넥터 금지(allow 제외)."""
        import heapq
        # 종점 b 는 금지에서 뺀다: frozen_v1 종점 road1898 은 j106 막다른 끝 회차 커넥터(U턴형) 자체다
        ban = ((self.n.uturn - set(allow)) | set(banned_nodes)) - {b}
        dist, prev = {a: 0.0}, {}
        pq = [(0.0, a)]
        while pq:
            d, u = heapq.heappop(pq)
            if u == b and u != a:
                break
            if d > dist.get(u, 1e18):
                continue
            for v in self.n.g.get(u, ()):
                if v in ban or (u, v) in banned_edges:
                    continue
                nd = d + self.n.lens[v][0]
                if nd < dist.get(v, 1e18):
                    dist[v], prev[v] = nd, u
                    heapq.heappush(pq, (nd, v))
        if b not in prev and b != a:
            return None
        p = [b]
        while p[-1] != a:
            p.append(prev[p[-1]])
        return p[::-1]

    def yen(self, a, b, k, allow=frozenset()):
        first = self.shortest(a, b, allow=allow)
        if not first:
            return []
        A, B = [first], []
        for _ in range(1, k):
            last = A[-1]
            for i in range(len(last) - 1):
                spur, root = last[i], last[:i + 1]
                be = {(p[i], p[i + 1]) for p in A if p[:i + 1] == root and len(p) > i + 1}
                bn = set(root[:-1])
                sp = self.shortest(spur, b, banned_nodes=bn, banned_edges=be, allow=allow)
                if sp:
                    cand = root[:-1] + sp
                    if cand not in A and cand not in B:
                        B.append(cand)
            if not B:
                break
            B.sort(key=self.plen)
            A.append(B.pop(0))
        return A

    def plen(self, p, start_s=0.0, end_s=None):
        L = self.n.lens
        if len(p) == 1:
            return (end_s if end_s is not None else L[p[0]][0]) - start_s
        tot = L[p[0]][0] - start_s + sum(L[r][0] for r in p[1:-1])
        return tot + (end_s if end_s is not None else L[p[-1]][0])

    def chain(self, legs, allow=frozenset()):
        """legs = road 목록 [a, x, y, b]. 각 구간 최단으로 이어 붙인다."""
        out = [legs[0]]
        for u, v in zip(legs, legs[1:]):
            seg = self.shortest(u, v, allow=allow)
            if not seg:
                return None
            out += seg[1:]
        return out


def r2_partial(net, r, s0, s1):
    """road r 의 [s0,s1] 구간 2m 창 R2 최소(세션11 정의, 0.5m 간격, 구간 양끝 1m 제외)."""
    best, best_s, s = 1e6, None, max(1.0, s0 + 1.0)
    while s <= min(net.lens[r][0], s1) - 1.0:
        a = net.cmap.get_waypoint_xodr(r, -1, s - 1.0)
        b = net.cmap.get_waypoint_xodr(r, -1, s + 1.0)
        k = abs(((b.transform.rotation.yaw - a.transform.rotation.yaw + 180) % 360 - 180)) / 2.0
        if k > 1e-6 and 1.0 / math.radians(k) < best:
            best, best_s = 1.0 / math.radians(k), s
        s += 0.5
    return best, best_s


def describe(name, rt, cfg, p):
    n = rt.n
    o_r, o_s = cfg["origin"]
    t_r, t_s = cfg["target"]
    total = rt.plen(p, o_s, t_s)
    print(f"\n## {name}")
    print(f"road 수 {len(p)}, 총 거리 {total:.1f}m")
    print(f"seq {p}")
    # 방향 검사: 연속 쌍이 lane -1 next() 그래프의 간선인가
    bad = [(u, v) for u, v in zip(p, p[1:]) if v not in n.g.get(u, ())]
    used_ut = [r for r in p if r in n.uturn]
    print(f"방향: 연속 {len(p) - 1}쌍 중 next() 간선 아님 {len(bad)} {bad if bad else ''}"
          f" / U턴형 커넥터 {used_ut if used_ut else '없음'}")
    # 정류장 통과
    cum, pos = 0.0, {}
    for i, r in enumerate(p):
        start = o_s if i == 0 else 0.0
        end = t_s if i == len(p) - 1 else n.lens[r][0]
        pos[r] = (cum - start, start, end)
        cum += end - start
    seen = []
    for k, (r, s) in cfg["stops"].items():
        if r in pos:
            base, st, en = pos[r]
            if st <= s <= en:
                seen.append((base + s, k, r, s))
    seen.sort()
    print("정류장 통과(정문부터 거리): " + (", ".join(f"{k} r{r} s={s:.0f} @{d:.0f}m" for d, k, r, s in seen)
                                        if seen else "없음"))
    # 커브·회전
    turns, r2min, r2at, avgmin, avgat = [], 1e6, None, 1e6, None
    r2x, r2xat = 1e6, None   # 종점 road 가 회차 커넥터일 때 그것을 뺀 최소 R2
    for i, r in enumerate(p):
        st = o_s if i == 0 else 0.0
        en = t_s if i == len(p) - 1 else n.lens[r][0]
        r2, r2s = r2_partial(n, r, st, en)
        if r2 < r2min:
            r2min, r2at = r2, (r, r2s)
        if not (i == len(p) - 1 and r in n.uturn) and r2 < r2x:
            r2x, r2xat = r2, (r, r2s)
        deg = n.turn[r]
        full = (st == 0.0 and en >= n.lens[r][0] - 1e-6)
        if abs(deg) >= TURN_MIN_DEG and full:
            avg = n.lens[r][0] / abs(math.radians(deg))
            if avg < avgmin:
                avgmin, avgat = avg, r
            j = n.lens[r][1]
            kind = "우" if deg > 0 else "좌"
            turns.append(f"r{r}({'j' + str(j) if j != -1 else '일반'}) {deg:+.1f}° {kind} 평균R {avg:.2f} R2min {r2:.2f}")
    print(f"회전(|각| >= {TURN_MIN_DEG:.0f}°, road 전체 통과분) {len(turns)}회")
    for t in turns:
        print(f"  {t}")
    print(f"최소 R2(2m 창, 경로 구간) {r2min:.2f}m @ r{r2at[0]} s={r2at[1]:.1f}")
    if r2xat != r2at:
        print(f"최소 R2(종점 회차 커넥터 제외) {r2x:.2f}m @ r{r2xat[0]} s={r2xat[1]:.1f}")
    print(f"최소 평균R(회전 road) {avgmin:.2f}m @ r{avgat} -> 9m 이상: {'예' if avgmin >= R9 else '아니오'}")
    return {"name": name, "len": total, "n": len(p), "stops": [k for _, k, _, _ in seen],
            "r2min": r2min, "r2x": r2x, "avgmin": avgmin, "turns": len(turns), "bad": len(bad), "ut": used_ut}


def run(label, path, cfg):
    print(f"\n# {label}")
    net = Net(path, label)
    rt = Router(net)
    o, t = cfg["origin"][0], cfg["target"][0]
    st = cfg["stops"]
    print(f"road {len(net.lens)}, U턴형 커넥터 {len(net.uturn)}, 출발 r{o} s={cfg['origin'][1]}, "
          f"종점 r{t} s={cfg['target'][1]}")
    if "match_cost" in cfg:
        print("정류장 번호 대응(옛->시험, 형상 일치 비용 m): " +
              ", ".join(f"{k} r{V1['stops'][k][0]}->r{st[k][0]} ({cfg['match_cost'][k]:.2f})" for k in st))
    res, seen = [], set()

    def add(name, p):
        if p is None:
            print(f"\n## {name}\n경로 없음")
            return
        key = tuple(p)
        dup = key in seen
        seen.add(key)
        r = describe(name + (" (앞 후보와 같은 경로)" if dup else ""), rt, cfg, p)
        if not dup:
            res.append(r)

    add("후보 A 정문 -> 남문 -> 후문 방향", rt.chain([o, st["남문"][0], t]))
    add("후보 B 정문 -> 양성재 -> 후문 방향", rt.chain([o, st["양성재"][0], t]))
    add("후보 C1 정문 -> 양성재 -> 남문 -> 후문 방향", rt.chain([o, st["양성재"][0], st["남문"][0], t]))
    add("후보 C2 정문 -> 남문 -> 양성재 -> 후문 방향", rt.chain([o, st["남문"][0], st["양성재"][0], t]))
    for k in ("북문", "중문"):
        tb = cfg["turnback"][k]
        add(f"후보 C3-{k} 정문 -> {k} -> 회차(r{tb}) -> 후문 방향",
            rt.chain([o, st[k][0], tb, t], allow=frozenset({tb})))
    # 정류장 순서 조합 전수(1~4곳, 순열 64개). 북문·중문 뒤에는 그 막다른 끝 회차 커넥터를 넣는다
    import itertools
    tb_all = frozenset(cfg["turnback"].values())
    combos = []
    for k in range(1, 5):
        for order in itertools.permutations(st.keys(), k):
            legs = [o]
            for name in order:
                legs.append(st[name][0])
                if name in cfg["turnback"]:
                    legs.append(cfg["turnback"][name])
            legs.append(t)
            p = rt.chain(legs, allow=tb_all)
            if p is None:
                continue
            # 실제로 지나는 정류장(순서 지정 밖에서 덤으로 지나는 것 포함)
            names = [nm for nm, (r, s) in st.items() if r in p]
            combos.append((len(set(names)), rt.plen(p, cfg["origin"][1], cfg["target"][1]), order, names, p))
    print(f"\n# 정류장 순서 조합 {len(combos)}개 성립. 지나는 정류장 수별 최단")
    best = {}
    for c in combos:
        if c[0] not in best or c[1] < best[c[0]][1]:
            best[c[0]] = c
    for nst in sorted(best):
        c = best[nst]
        print(f"{nst}곳: {c[1]:.0f}m 지정순서 {'->'.join(c[2])} 실제통과 {'+'.join(c[3])}")
    top = best[max(best)]
    add(f"후보 C4 정류장 최다 {top[0]}곳 ({'->'.join(top[2])})", top[4])

    ks = rt.yen(o, t, K_SHORTEST)
    print(f"\n# k-최단(U턴 금지) {len(ks)}개, 길이: " +
          ", ".join(f"{rt.plen(p, cfg['origin'][1], cfg['target'][1]):.0f}" for p in ks))
    for i, p in enumerate(ks, 1):
        add(f"후보 C-k{i} (k-최단 {i}번째)", p)
    print(f"\n# {label} 요약 (거리 / road / 정류장 / 회전 / 최소 R2(종점 회차 제외) / 최소 평균R)")
    for r in res:
        print(f"{r['name'][:28]} | {r['len']:.0f}m | {r['n']} | {'+'.join(r['stops']) or '-'} | {r['turns']} | "
              f"{r['r2min']:.2f}({r['r2x']:.2f}) | {r['avgmin']:.2f} | 방향오류 {r['bad']} | U턴 {r['ut'] or '-'}")


if __name__ == "__main__":
    run("frozen_v1 (bf835cdfad0cea65), 종점 road1898 s=5.0", XODR_V1, V1)
    run("시험 재변환 맵 _trial_s18_gates (채택 안 됨, 예측용), 종점 후문 후보", XODR_TRIAL, trial_cfg())
