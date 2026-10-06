#!/usr/bin/env python3
# 맵세션23 Phase 0: 연결성 분석 (읽기전용, 서버 없음, xodr/OSM 미수정).
# 0-1 정문(road1247 lane -1 s=0)에서 방향을 지켜 도달 가능한 road
# 0-2 정문으로 돌아올 수 있는 road, 막다른 길과 회차 커넥터
# 0-3 도달 불가 road 를 덩어리로 묶고, 본망에 붙이는 데 필요한 OSM way 탐색
# 방향 그래프: lane -1 끝에서 next() 로 도착하는 road (s19_scope_common.successors). 무방향 계산은
# "덩어리 묶기"에만 쓰고 도달·왕복 판정에는 쓰지 않는다.
# 실행: .venv-carla/bin/python map/scripts/s23_connectivity.py [xodr] > map/docs/logs/s23_connectivity.log
import csv
import heapq
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, "/home/gyumin/campus_mobility_sim/map/scripts")
from s19_scope_common import BASE, RAW_OSM, CONV_OSM, XODR_V1, load_osm, ll_to_carla  # noqa: E402
from s19_scope import Net  # noqa: E402
from classify_internal import load_polygon_rings, point_in_polygon  # noqa: E402

XODR = Path(sys.argv[1]) if len(sys.argv) > 1 else XODR_V1
ORIGIN = int(sys.argv[2]) if len(sys.argv) > 2 else 1247
BLDG = BASE / "data/processed/s22_buildings.csv"
POLY = BASE / "data/processed/cbnu_relation_polygon.json"
NODE_SNAP = 3.0          # road 끝점 -> OSM 노드 대응 허용 거리(m)
FOOT_W = 3.0             # 보행로(footway/path/pedestrian) 비용 가중
OUT_W = 5.0              # 캠퍼스 폴리곤 밖 way 비용 가중
NO_DRIVE = {"steps", "corridor", "rest_area", "construction", "cycleway"}


def reach(g, src):
    seen, st = {src}, [src]
    while st:
        u = st.pop()
        for v in g.get(u, ()):
            if v not in seen:
                seen.add(v)
                st.append(v)
    return seen


def sccs(g, nodes):
    sys.setrecursionlimit(50000)
    idx, low, on, st, comps, c = {}, {}, set(), [], [], [0]

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
    for u in nodes:
        if u not in idx:
            dfs(u)
    return comps


def endpoints(net, r):
    L = net.lens[r][0]
    a = net.cmap.get_waypoint_xodr(r, -1, 0.0)
    b = net.cmap.get_waypoint_xodr(r, -1, max(L - 1e-3, 0.0))
    return ((a.transform.location.x, a.transform.location.y),
            (b.transform.location.x, b.transform.location.y))


def mid(net, r):
    w = net.cmap.get_waypoint_xodr(r, -1, net.lens[r][0] / 2)
    return w.transform.location.x, w.transform.location.y


def main():
    print(f"# s23 연결성 분석 — {XODR.name}, 출발 r{ORIGIN} lane -1 s=0")
    net = Net(XODR, XODR.stem)
    allr = set(net.lens)
    N = len(allr)
    nedge = sum(len(v) for v in net.g.values())
    jcount = {}
    for r, (L, j) in net.lens.items():
        if j != -1:
            jcount[j] = jcount.get(j, 0) + 1
    deadend_ut = {r for r in net.uturn if jcount.get(net.lens[r][1]) == 1}
    inter_ut = net.uturn - deadend_ut
    print(f"road {N}, 방향 간선 {nedge}, junction {len(jcount)}, U턴형 커넥터 {len(net.uturn)}"
          f" (막다른 끝 회차 {len(deadend_ut)} / 교차로 안 {len(inter_ut)})")
    print(f"막다른 끝 회차 커넥터: {sorted(deadend_ut)}")

    # 0-1 도달
    R = reach(net.g, ORIGIN)
    g_noU = {u: [v for v in vs if v not in inter_ut] for u, vs in net.g.items() if u not in inter_ut}
    R_noU = reach(g_noU, ORIGIN)
    U = allr - R
    print("\n## 0-1 도달 (방향 준수)")
    print(f"도달 가능 {len(R)} / {N} ({100 * len(R) / N:.1f}%) — 모든 커넥터 허용")
    print(f"도달 가능 {len(R_noU)} / {N} ({100 * len(R_noU) / N:.1f}%) — 교차로 안 U턴 금지"
          f"(막다른 끝 회차는 허용)")
    print(f"U턴 금지 시 추가로 못 가는 road {len(R - R_noU)}: {sorted(R - R_noU)}")
    print(f"도달 불가 {len(U)}")

    # 무방향 연결(덩어리 묶기·분류용): 그래프 간선 + 반쪽 도로 쌍(lane -1 끝점이 서로 엇갈려 4.0m 이내,
    # 두 반쪽의 차선 중심 간격 = 차선폭 3.35m)
    ep = {r: endpoints(net, r) for r in allr}
    und = {r: set() for r in allr}
    for u, vs in net.g.items():
        for v in vs:
            und[u].add(v)
            und[v].add(u)
    twin = {}
    plain = [r for r in allr if net.lens[r][1] == -1]
    for i, a in enumerate(plain):
        (as_, ae) = ep[a]
        for b in plain[i + 1:]:
            (bs, be) = ep[b]
            if math.dist(as_, be) < 4.0 and math.dist(ae, bs) < 4.0:
                twin[a], twin[b] = b, a
                und[a].add(b)
                und[b].add(a)
    print(f"반쪽 도로 쌍(양방향 길) {len(twin) // 2}쌍, 짝 없는 일반 road {len(plain) - len(twin)}")

    main_wcc = reach({k: list(v) for k, v in und.items()}, ORIGIN)
    can_exit = set()   # 도달 불가지만 도달 가능 망으로 나갈 수 있는 road
    for r in U:
        if reach(net.g, r) & R:
            can_exit.add(r)
    cls = {}
    for r in U:
        indeg, outdeg = len(net.pred[r]), len(net.g[r])
        if indeg == 0 and outdeg == 0 and r not in twin:
            cls[r] = "고립(앞뒤 연결 0, 짝 없음)"
        elif r not in main_wcc:
            cls[r] = "연결 끊김(본망과 무방향으로도 안 닿음)"
        elif r in can_exit:
            cls[r] = "역방향만 존재(본망으로 나가는 쪽만 연결)"
        else:
            cls[r] = "본망에 닿으나 진입·진출 모두 불가"
    from collections import Counter
    print("도달 불가 사유:")
    for k, v in Counter(cls.values()).most_common():
        print(f"  {k}: {v}")
    print("  보도·주차장 등 비주행 lane: 0 (878 road 전부 lane -1 = driving, 보도는 lane -2 로 같은 road 안)")

    # 0-2 왕복
    back = set()
    rev = {r: net.pred[r] for r in allr}
    back = reach(rev, ORIGIN)
    RT = R & back
    print("\n## 0-2 왕복")
    print(f"정문 진입 road r{ORIGIN} 로 돌아올 수 있는 road(도달 가능 중) {len(RT)} / {N}"
          f" ({100 * len(RT) / N:.1f}%): {sorted(RT)}")
    print(f"r{ORIGIN} 에 도달할 수 있는 road 전체(도달 여부 무관) {len(back)}: {sorted(back)}")
    print(f"r{ORIGIN} 앞 road(pred): {sorted(net.pred[ORIGIN])}")
    comps = sccs(net.g, sorted(allr))
    big = max(comps, key=len)
    bigset = set(big)
    print(f"강연결 성분 {len(comps)}개, 가장 큰 것 {len(big)} road (정문 r{ORIGIN} 포함: {ORIGIN in bigset})")
    to_big = reach(rev, next(iter(bigset)))  # 큰 SCC 로 들어올 수 있는 road
    trap = sorted(R - to_big)
    print(f"[참고] 정문 복귀 대신 '주 순환망(가장 큰 SCC)으로 복귀' 기준: 도달 {len(R)} 중 복귀 가능 {len(R & to_big)},"
          f" 갇힘 {len(trap)}")
    print(f"  갇히는 road(주 순환망으로 못 돌아옴): {trap}")
    dead = sorted(r for r in R if not net.g[r])
    print(f"막다른 road(도달 가능, 다음 road 0) {len(dead)}: {dead}")
    import xml.etree.ElementTree as ET
    root = ET.parse(str(XODR)).getroot()
    jdef = {j.get("id") for j in root.iter("junction")}
    jref = {}
    for rd in root.iter("road"):
        for e in rd.iter():
            if e.tag in ("predecessor", "successor") and e.get("elementType") == "junction":
                jref.setdefault(e.get("elementId"), []).append(f"r{rd.get('id')}.{e.tag[:4]}")
    undef = sorted(set(jref) - jdef, key=int)
    print(f"정의 없는 junction 참조(커넥터 0 = 회차 없는 막다른 끝) {len(undef)}:")
    for j in undef:
        print(f"  j{j}: {jref[j]}")
    for r in dead:
        if r in twin:
            print(f"  막다른 r{r} -> 짝 r{twin[r]} 도달 {'예' if twin[r] in R else '아니오'}")
        else:
            print(f"  막다른 r{r} -> 짝 없음(일방통행)")
    de_reached = sorted(deadend_ut & R)
    print(f"회차 연결로(막다른 끝 회차 커넥터)가 있는 막다른 길 {len(deadend_ut)}개, 그중 도달 가능 {len(de_reached)}:"
          f" {de_reached}")
    for r in sorted(deadend_ut):
        j = net.lens[r][1]
        print(f"  r{r} j{j} 길이 {net.lens[r][0]:.2f} 앞 {sorted(net.pred[r])} 뒤 {sorted(net.g[r])}"
              f" 위치 CARLA ({mid(net, r)[0]:.0f},{mid(net, r)[1]:.0f}) 도달 {'예' if r in R else '아니오'}")

    # 0-3 덩어리
    print("\n## 0-3 도달 불가 덩어리")
    seen, lumps = set(), []
    for r in sorted(U):
        if r in seen:
            continue
        comp, st = {r}, [r]
        while st:
            u = st.pop()
            for v in und[u]:
                if v in U and v not in comp:
                    comp.add(v)
                    st.append(v)
        seen |= comp
        lumps.append(sorted(comp))
    lumps.sort(key=lambda c: -sum(net.lens[r][0] for r in c))
    print(f"덩어리 {len(lumps)}개, 가장 큰 덩어리 road 수 {max(len(c) for c in lumps)}")

    blds = []
    if BLDG.exists():
        with open(BLDG) as f:
            for row in csv.DictReader(f):
                blds.append(row)
    nodes, ways = load_osm(RAW_OSM)
    conv_nodes, conv_ways = load_osm(CONV_OSM)
    conv_ids = {w for w, d in conv_ways.items() if d["tags"].get("highway") == "unclassified"}
    rings = load_polygon_rings(POLY)
    nxy = {n: ll_to_carla(*ll) for n, ll in nodes.items()}
    used_nodes = set()
    for w in ways.values():
        if "highway" in w["tags"]:
            used_nodes.update(w["nds"])
    grid = {}
    for n in used_nodes:
        x, y = nxy[n]
        grid.setdefault((int(x // 10), int(y // 10)), []).append(n)

    def snap(p):
        out = []
        gx, gy = int(p[0] // 10), int(p[1] // 10)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for n in grid.get((gx + dx, gy + dy), ()):
                    if math.dist(nxy[n], p) <= NODE_SNAP:
                        out.append(n)
        return out

    # OSM 방향 그래프: 변환된 way 는 비용 0(이미 road), 미변환 highway way 는 길이 x 가중
    def way_info(wid):
        w = ways[wid]
        t = w["tags"]
        inside = sum(point_in_polygon(nodes[n][1], nodes[n][0], rings) for n in w["nds"]) / len(w["nds"])
        L = sum(math.dist(nxy[a], nxy[b]) for a, b in zip(w["nds"], w["nds"][1:]))
        return t, inside, L

    edges = {}
    winfo = {}
    for wid, w in ways.items():
        t = w["tags"]
        hw = t.get("highway")
        if not hw or hw in NO_DRIVE:
            continue
        tt, inside, L = way_info(wid)
        winfo[wid] = (hw, t.get("service", ""), t.get("oneway", ""), inside, L, t.get("name", ""))
        conv = wid in conv_ids
        mult = 0.0 if conv else (FOOT_W if hw in ("footway", "path", "pedestrian", "track") else 1.0)
        if not conv and inside < 0.5:
            mult *= OUT_W
        ow = t.get("oneway") in ("yes", "1")
        for a, b in zip(w["nds"], w["nds"][1:]):
            d = math.dist(nxy[a], nxy[b]) * mult + (0 if conv else 0.01)
            edges.setdefault(a, []).append((b, d, wid))
            if not ow:
                edges.setdefault(b, []).append((a, d, wid))

    def osm_search(src_nodes, dst_nodes):
        pq = [(0.0, n, None) for n in src_nodes]
        heapq.heapify(pq)
        best, prev = {}, {}
        for _, n, _ in pq:
            best[n] = 0.0
        while pq:
            d, u, _ = heapq.heappop(pq)
            if d > best.get(u, 1e18):
                continue
            if u in dst_nodes:
                ws, x = [], u
                while x in prev:
                    x, wid = prev[x]
                    if wid not in conv_ids and (not ws or ws[-1] != wid):
                        ws.append(wid)
                return d, ws[::-1]
            for v, dd, wid in edges.get(u, ()):
                nd = d + dd
                if nd < best.get(v, 1e18):
                    best[v], prev[v] = nd, (u, wid)
                    heapq.heappush(pq, (nd, v, wid))
        return None, None

    R_nodes = set()
    for r in R:
        for p in ep[r]:
            R_nodes.update(snap(p))
    summary = []
    for i, c in enumerate(lumps, 1):
        L = sum(net.lens[r][0] for r in c)
        pts = [mid(net, r) for r in c]
        cx, cy = sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)
        near = ""
        if blds:
            b = min(blds, key=lambda b: math.dist((float(b["carla_x"]), float(b["carla_y"])), (cx, cy)))
            near = f"{b['name']} {math.dist((float(b['carla_x']), float(b['carla_y'])), (cx, cy)):.0f}m"
        kinds = Counter(cls[r] for r in c)
        inside = sum(point_in_polygon(*_ll_from_carla(p), rings) for p in pts)
        lump_nodes = set()
        for r in c:
            for p in ep[r]:
                lump_nodes.update(snap(p))
        tag = " <- road1333 포함(후문 쪽)" if 1333 in c else ""
        print(f"\n### 덩어리 {i}{tag}")
        print(f"road {len(c)}, 길이 합 {L:.0f}m, 중심 CARLA ({cx:.0f},{cy:.0f}), 가까운 건물 {near or '-'}")
        print(f"캠퍼스 폴리곤 안 road {inside}/{len(c)}")
        print(f"사유: {dict(kinds)}")
        print(f"road: {c}")
        jset = sorted({net.lens[r][1] for r in c if net.lens[r][1] != -1})
        print(f"junction: {jset if jset else '없음'}")
        d_in, w_in = osm_search(R_nodes - lump_nodes, lump_nodes)
        d_out, w_out = osm_search(lump_nodes, R_nodes - lump_nodes)
        for lab, d, ws in (("본망 -> 덩어리(도달)", d_in, w_in), ("덩어리 -> 본망(복귀)", d_out, w_out)):
            if d is None:
                print(f"{lab}: OSM 에서도 경로 없음")
                continue
            print(f"{lab}: 미변환 way {len(ws)}개, 가중 비용 {d:.0f}")
            for wid in ws:
                hw, sv, ow, ins, wl, nm = winfo[wid]
                print(f"  way{wid} {hw}{'/' + sv if sv else ''}{' oneway=' + ow if ow else ''} {wl:.0f}m"
                      f" 폴리곤안 {ins:.0%}{' ' + nm if nm else ''}")
        summary.append((i, len(c), L, (cx, cy), near, w_in, w_out, 1333 in c))

    print("\n## 0-4 요약")
    print(f"도달 가능 {len(R)} / {N} ({100 * len(R) / N:.1f}%)")
    print(f"왕복 가능 {len(RT)} / {N} ({100 * len(RT) / N:.1f}%)")
    print(f"고립 덩어리 {len(lumps)}개")
    print(f"가장 큰 덩어리의 도로 수 {max(len(c) for c in lumps)}")
    print(f"road1333 덩어리 포함: {any(s[7] for s in summary)}")
    json.dump({"R": sorted(R), "RT": sorted(RT), "lumps": lumps, "trap": trap},
              open(BASE / f"docs/logs/s23_connectivity_{XODR.stem}.json", "w"))


def _ll_from_carla(p):
    """CARLA (x,y) -> (lon, lat) 근사 역변환(폴리곤 안 판정용, 1km 범위 오차 무시 가능)."""
    from s19_scope_common import tm, OFF_X, OFF_Y, LAT0, LON0
    tx, ty = p[0] - OFF_X, -p[1] - OFF_Y
    lat, lon = LAT0, LON0
    for _ in range(5):
        x, y = tm(lat, lon)
        lat += (ty - y) / 111000.0
        lon += (tx - x) / (111000.0 * math.cos(math.radians(lat)))
    return lon, lat


if __name__ == "__main__":
    main()
