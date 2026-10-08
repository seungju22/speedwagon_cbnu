#!/usr/bin/env python3
# 맵세션28 Phase 1·2 오프라인 확인 (읽기전용, 서버 없음, 맵·OSM 수정 없음).
# Phase 1: OSM 에서 "후문" 으로 볼 수 있는 지점 전부 찾기
#   (가) 이름에 '후문' 이 들어간 요소 (나) entrance·gate·barrier 노드
#   (다) 차량 도로가 캠퍼스 폴리곤 경계를 넘는 점(차량 출입구 후보) -> 50m 안 묶음
#   각 묶음: 좌표, 경계 넘는 way, 주변 이름 있는 건물, 고가차도(way293211019) 거리, 후문 정류장 거리
# Phase 2: way392632034 양 끝·노드별 안/밖·태그, 추정 게이트 좌표와의 거리,
#   후문 일대 경계 넘는 차량 way 의 방향(들어옴/나감)과 우회 가능성
# 실행: ~/campus_mobility_sim/.venv-carla/bin/python map/scripts/s28_gate_review.py > map/docs/logs/s28_gate_review.log
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from s19_scope_common import BASE, tm  # noqa: E402
from classify_internal import load_polygon_rings, point_in_polygon  # noqa: E402
from s27_route_checks import load_raw, inv_tm, seg_dist, poly_tm, dist_to_line, area, centroid, fmt, compass  # noqa: E402

POLY = BASE / "data/processed/cbnu_relation_polygon.json"
CONV = BASE / "data/processed/cbnu_internal_boundary_tags73_smooth.osm"
OVERPASS = "293211019"
GATE_EST = (36.624844, 127.462860)      # 세션27 추정 게이트(s27_route_checks.md 2-2, 미확인)
LOOP_JOIN = "4496854212"                 # 고리 접점(392632034 끝)
STOP_NAMES = ("충북대학교후문",)
BACK6 = ["481945510", "481943505", "481475019", "452870644", "481945509", "481943503"]
LOOP = ["452870641", "452870648"]        # road1333
NON_VEH = {"footway", "path", "cycleway", "steps", "pedestrian", "corridor", "bridleway", "elevator", "platform"}


def inside(nodes, rings, nid):
    lat, lon = nodes[nid]
    return point_in_polygon(lon, lat, rings)


def ll_dist(a, b):
    ax, ay = tm(*a)
    bx, by = tm(*b)
    return math.hypot(ax - bx, ay - by)


def named_buildings(nodes, ways):
    out = []
    for wid, w in ways.items():
        t = w["tags"]
        if "building" not in t:
            continue
        q = poly_tm(nodes, w)
        if len(q) >= 3:
            out.append((wid, t.get("name", ""), t.get("building"), q))
    return out


def main():
    nodes, ways, ntags = load_raw()
    rings = load_polygon_rings(POLY)
    conv_ways = set()
    import xml.etree.ElementTree as ET
    for _, el in ET.iterparse(str(CONV)):
        if el.tag == "way":
            t = {x.get("k"): x.get("v") for x in el.findall("tag")}
            if t.get("highway") and t.get("highway") != "service":
                conv_ways.add(el.get("id"))
            el.clear()
    ov = poly_tm(nodes, ways[OVERPASS])
    print(f"고가: way{OVERPASS} {ways[OVERPASS]['tags']}")
    print(f"  노드 {len(ways[OVERPASS]['nds'])}, 시작 {fmt(nodes[ways[OVERPASS]['nds'][0]])}, 끝 {fmt(nodes[ways[OVERPASS]['nds'][-1]])}")
    bridges = [(wid, w) for wid, w in ways.items() if w["tags"].get("bridge") and w["tags"].get("highway")
               and w["tags"]["highway"] not in NON_VEH]
    stops = [(nid, t) for nid, t in ntags.items() if t.get("name") in STOP_NAMES]
    blds = named_buildings(nodes, ways)

    print("\n## Phase 1-가. 이름에 '후문' 이 들어간 OSM 요소")
    for nid, t in ntags.items():
        if any("후문" in v for v in t.values()):
            print(f"  node{nid} {fmt(nodes[nid])} {t}")
    for wid, w in ways.items():
        if any("후문" in v for v in w["tags"].values()):
            print(f"  way{wid} {w['tags']}")

    print("\n## Phase 1-나. entrance·gate·barrier·guard 태그 노드/way (캠퍼스 전체)")
    for nid, t in ntags.items():
        if "entrance" in t or "barrier" in t or t.get("amenity") in ("security", "guard") or "gate" in t.get("name", ""):
            p = nodes[nid]
            print(f"  node{nid} {fmt(p)} 안={point_in_polygon(p[1], p[0], rings)} {t}")
    for wid, w in ways.items():
        t = w["tags"]
        if "barrier" in t or "entrance" in t or t.get("building") in ("guardhouse", "gatehouse"):
            print(f"  way{wid} {t}")

    print("\n## Phase 1-다. 차량 도로가 캠퍼스 경계를 넘는 점")
    cross = []
    for wid, w in ways.items():
        hw = w["tags"].get("highway")
        if not hw or hw in NON_VEH:
            continue
        nds = [n for n in w["nds"] if n in nodes]
        for i in range(len(nds) - 1):
            a, b = nds[i], nds[i + 1]
            ia, ib = inside(nodes, rings, a), inside(nodes, rings, b)
            if ia != ib:
                o = a if not ia else b
                cross.append((wid, hw, o, nodes[o], "들어옴" if ib else "나감"))
    print(f"경계 넘는 차량 way 구간 {len(cross)}개 (점 = 밖쪽 노드)")
    # 50m 묶음
    groups = []
    for c in cross:
        for g in groups:
            if ll_dist(g[0][3], c[3]) < 50:
                g.append(c)
                break
        else:
            groups.append([c])
    outer = max(rings, key=len)
    cen = centroid([tm(la, lo) for lo, la in outer])
    print(f"묶음(50m) {len(groups)}개. 방위는 캠퍼스 폴리곤 중심 {fmt(inv_tm(*cen))} 기준")
    for k, g in enumerate(sorted(groups, key=lambda g: compass(*[tm(*g[0][3])[i] - cen[i] for i in (0, 1)])[0])):
        p = g[0][3]
        pt = tm(*p)
        b, nm = compass(pt[0] - cen[0], pt[1] - cen[1])
        d_ov = dist_to_line(pt, ov)
        d_br = min((dist_to_line(pt, poly_tm(nodes, w)), wid, w["tags"].get("name", "")) for wid, w in bridges)
        d_st = min((ll_dist(p, nodes[n]), n) for n, _ in stops) if stops else None
        print(f"\n[G{k + 1}] {fmt(p)} 방위 {b:.0f}도({nm}) 중심에서 {math.hypot(pt[0]-cen[0], pt[1]-cen[1]):.0f}m")
        for wid, hw, o, q, dr in g:
            t = ways[wid]["tags"]
            print(f"  way{wid} {hw} {t.get('service', '')} {t.get('name', '')} oneway={t.get('oneway', '-')}"
                  f" 경계구간 {dr}(way 방향) 밖노드 {o} 변환입력={'예' if wid in conv_ways else '아니오'}")
        print(f"  고가 way{OVERPASS} 까지 {d_ov:.0f}m / 가장 가까운 차량 bridge way{d_br[1]} {d_br[2]} {d_br[0]:.0f}m")
        if d_st:
            print(f"  '충북대학교후문' 정류장 최근접 node{d_st[1]} {d_st[0]:.0f}m")
        near = sorted((dist_to_line(pt, q), n or "(이름없음)", bt, wid, area(q)) for wid, n, bt, q in blds)
        named = [x for x in near if x[1] != "(이름없음)"][:4]
        print("  가까운 이름 있는 건물: " + "; ".join(f"{n} {d:.0f}m" for d, n, *_ in named))
        small = [x for x in near if x[0] < 120][:3]
        print("  120m 안 건물(가까운 순 3): " + "; ".join(f"way{w} {n} {bt} {a:.0f}m2 {d:.0f}m" for d, n, bt, w, a in small))

    print("\n## Phase 1-라. 정류장·고가 상호 거리")
    for n, t in stops:
        print(f"  node{n} {t.get('name')} {fmt(nodes[n])} 고가까지 {dist_to_line(tm(*nodes[n]), ov):.0f}m")

    print("\n## Phase 2-가. way392632034 노드별")
    w = ways["392632034"]
    print(f"  태그 {w['tags']}")
    acc = 0.0
    prev = None
    for i, n in enumerate(w["nds"]):
        p = nodes[n]
        if prev:
            acc += ll_dist(prev, p)
        prev = p
        users = sorted(wid for wid, ww in ways.items() if n in ww["nds"] and wid != "392632034")
        ut = [f"{u}({ways[u]['tags'].get('highway', ways[u]['tags'].get('building', '?'))})" for u in users]
        print(f"  [{i}] node{n} {fmt(p)} 안={inside(nodes, rings, n)} 누적 {acc:.1f}m 태그 {ntags.get(n, {})} 공유 {', '.join(ut)}")
    pts = poly_tm(nodes, w)
    g = tm(*GATE_EST)
    print(f"  추정 게이트 {fmt(GATE_EST)} -> way 까지 {dist_to_line(g, pts):.2f}m, 안={point_in_polygon(GATE_EST[1], GATE_EST[0], rings)}")
    for oid in BACK6 + LOOP:
        print(f"  추정 게이트 -> way{oid} {dist_to_line(g, poly_tm(nodes, ways[oid])):.1f}m")

    print("\n## Phase 2-나. 후문B 일대(경계점 300m) 경계 넘는 차량 way: 밖에서 고리로 들어오는 다른 길이 있는가")
    bpt = tm(36.624701, 127.463656)
    for wid, hw, o, q, dr in cross:
        if dist_to_line(bpt, [tm(*q)]) < 300:
            t = ways[wid]["tags"]
            print(f"  way{wid} {hw} {t.get('service', '')} {t.get('name', '')} oneway={t.get('oneway', '-')} "
                  f"경계구간 {dr} 밖노드 {fmt(q)} 후문B 경계점에서 {ll_dist(q, (36.624701, 127.463656)):.0f}m")
    print("고리(road1333) way 노드 안/밖")
    for lid in LOOP:
        lw = ways[lid]
        flags = "".join("I" if inside(nodes, rings, n) else "O" for n in lw["nds"])
        print(f"  way{lid} {lw['tags']} 노드 {len(lw['nds'])} {flags}")
        for end in (lw["nds"][0], lw["nds"][-1]):
            users = sorted(x for x, ww in ways.items() if end in ww["nds"] and x != lid and ww["tags"].get("highway"))
            print(f"    끝 node{end} {fmt(nodes[end])} 안={inside(nodes, rings, end)} 공유 "
                  + ", ".join(f"{u}({ways[u]['tags'].get('highway')},{ways[u]['tags'].get('oneway', '-')})" for u in users))


if __name__ == "__main__":
    main()


# ---- Phase 1-마 / 2-다: 그래프 확인 (2차 추가) ----
PUBLIC = {"motorway", "trunk", "primary", "secondary", "tertiary", "residential",
          "motorway_link", "trunk_link", "primary_link", "secondary_link", "tertiary_link", "living_street"}


def veh_graph(ways, nodes, banned=frozenset(), directed=True):
    import collections
    g = collections.defaultdict(list)
    for wid, w in ways.items():
        hw = w["tags"].get("highway")
        if not hw or hw in NON_VEH or wid in banned:
            continue
        nds = [n for n in w["nds"] if n in nodes]
        ow = w["tags"].get("oneway") in ("yes", "1", "true")
        for a, b in zip(nds, nds[1:]):
            d = ll_dist(nodes[a], nodes[b])
            g[a].append((b, d, wid))
            if not ow or not directed:
                g[b].append((a, d, wid))
    return g


def sp(g, srcs, dst):
    import heapq
    dist, prev = {}, {}
    h = [(0.0, s, None, None) for s in srcs]
    while h:
        d, u, p, w = heapq.heappop(h)
        if u in dist:
            continue
        dist[u], prev[u] = d, (p, w)
        if u == dst:
            break
        for v, dd, wid in g.get(u, []):
            if v not in dist:
                heapq.heappush(h, (d + dd, v, u, wid))
    if dst not in dist:
        return None, []
    seq, u = [], dst
    while prev[u][0] is not None:
        if not seq or seq[-1] != prev[u][1]:
            seq.append(prev[u][1])
        u = prev[u][0]
    return dist[dst], seq[::-1]


def public_nodes(ways, nodes, rings):
    s = set()
    for wid, w in ways.items():
        if w["tags"].get("highway") in PUBLIC:
            for n in w["nds"]:
                if n in nodes and not inside(nodes, rings, n):
                    s.add(n)
    return s


def extra():
    nodes, ways, ntags = load_raw()
    rings = load_polygon_rings(POLY)
    pub = public_nodes(ways, nodes, rings)
    print("\n## Phase 1-마. 동쪽 경계 횡단점(G3·G4·G5) 밖 노드 -> 공도 연결과 주변 주차장")
    east = {"G3": ("665834148", "6232825685"), "G4": ("665834145", "2967848544"), "G5": ("392632034", "3958352335")}
    g_und = veh_graph(ways, nodes, directed=False)
    parks = [(wid, w) for wid, w in ways.items() if w["tags"].get("amenity") == "parking"]
    for k, (wid, onode) in east.items():
        d, seq = sp(g_und, [onode], None) if False else (None, [])
        # 밖 노드에서 가장 가까운 공도 노드(무방향, 차량 way)
        best = None
        import heapq
        dist = {onode: 0.0}
        h = [(0.0, onode)]
        while h:
            dd, u = heapq.heappop(h)
            if dd > dist.get(u, 1e18):
                continue
            if u in pub:
                best = (dd, u)
                break
            for v, l, w2 in g_und.get(u, []):
                if dd + l < dist.get(v, 1e18):
                    dist[v] = dd + l
                    heapq.heappush(h, (dd + l, v))
        users = sorted(x for x, ww in ways.items() if best and best[1] in ww["nds"] and ww["tags"].get("highway") in PUBLIC)
        p = tm(*nodes[onode])
        pk = sorted((dist_to_line(p, poly_tm(nodes, w)), area(poly_tm(nodes, w)), pid, w["tags"].get("name", ""))
                    for pid, w in parks if len(w["nds"]) > 3)
        print(f"  {k} way{wid} 밖노드 {onode}: 공도까지 {best[0]:.0f}m "
              + ", ".join(f"way{u} {ways[u]['tags'].get('highway')} {ways[u]['tags'].get('name', '')}" for u in users))
        print("    100m 안 주차장: " + "; ".join(f"way{pid} {nm or '(이름없음)'} {a:.0f}m2 {dd:.0f}m" for dd, a, pid, nm in pk if dd < 100))
    ow = ways[OVERPASS]
    print(f"  고가 way{OVERPASS} 길이 {sum(ll_dist(nodes[a], nodes[b]) for a, b in zip(ow['nds'], ow['nds'][1:])):.1f}m"
          f" (노드 {len(ow['nds'])}) -> 동쪽 경계를 따라 길게 지나 G3·G4·G5 모두 10m 안")
    for n in ("6232825686", "6339086688"):
        on = [f"way{x}[{ww['nds'].index(n)}/{len(ww['nds'])}]" for x, ww in ways.items() if n in ww["nds"]]
        print(f"  node{n} {ntags.get(n)} {fmt(nodes[n])} 소속 {', '.join(on)} G4 밖노드에서 {ll_dist(nodes[n], nodes['2967848544']):.1f}m")
    print("  G3-G4 거리 %.0fm, G4-G5 %.0fm, G3-G5 %.0fm" % (
        ll_dist(nodes["6232825685"], nodes["2967848544"]), ll_dist(nodes["2967848544"], nodes["3958352335"]),
        ll_dist(nodes["6232825685"], nodes["3958352335"])))
    for wid in ("665834148", "665834145", "676959808"):
        w = ways[wid]
        print(f"  way{wid} {w['tags']} 길이 {sum(ll_dist(nodes[a], nodes[b]) for a, b in zip(w['nds'], w['nds'][1:])):.0f}m "
              f"안/밖 {''.join('I' if inside(nodes, rings, n) else 'O' for n in w['nds'])}")
    e87 = [(wid, w) for wid, w in ways.items() if "E8-7" in w["tags"].get("name", "")]
    for wid, w in e87:
        c = centroid(poly_tm(nodes, w))
        print(f"  E8-7 way{wid} {w['tags'].get('name')} 중심 {fmt(inv_tm(*c))} -> G4 {math.hypot(c[0]-tm(*nodes['2967848544'])[0], c[1]-tm(*nodes['2967848544'])[1]):.0f}m"
              f", G5 {math.hypot(c[0]-tm(*nodes['3958352335'])[0], c[1]-tm(*nodes['3958352335'])[1]):.0f}m")

    print("  동남쪽 다른 묶음(G6~G9) 밖 노드 -> 공도 최단(무방향, 캠퍼스 안 경유 포함). G3~G5 는 0m")
    for k, ns in {"G6": ["4631208531", "4658272638", "4658272654"], "G7": ["4658272617", "4658272624"],
                  "G8": ["4496854215", "4496854272"], "G9": ["4658272626", "4658272629"]}.items():
        ds = []
        for o in ns:
            dist = {o: 0.0}
            h = [(0.0, o)]
            while h:
                dd, u = heapq.heappop(h)
                if dd > dist[u]:
                    continue
                if u in pub:
                    ds.append(dd)
                    break
                for v, l, w2 in g_und.get(u, []):
                    if dd + l < dist.get(v, 1e18):
                        dist[v] = dd + l
                        heapq.heappush(h, (dd + l, v))
        print(f"    {k}: " + ", ".join(f"{d:.0f}m" for d in ds))

    print("\n## Phase 2-다. 공도에서 고리 접점(node4496854212)으로 들어오는 길 (방향 준수)")
    inner6 = set(BACK6)
    for label, banned in (("제한 없음(후문 6 way 제외: 캠퍼스 안쪽 경로 배제)", inner6),
                          ("392632034 도 제외", inner6 | {"392632034"})):
        g = veh_graph(ways, nodes, banned)
        d, seq = sp(g, pub, LOOP_JOIN)
        print(f"  [{label}] 공도 -> 고리 접점: " + (f"{d:.0f}m via " + " -> ".join(seq) if d is not None else "경로 없음"))
        # 반대 방향
        gr = {}
        for u, lst in g.items():
            for v, l, w2 in lst:
                gr.setdefault(v, []).append((u, l, w2))
        d2, seq2 = sp(gr, [LOOP_JOIN], None) if False else (None, [])
        import heapq
        dist = {LOOP_JOIN: 0.0}
        prev = {}
        h = [(0.0, LOOP_JOIN)]
        hit = None
        while h:
            dd, u = heapq.heappop(h)
            if dd > dist.get(u, 1e18):
                continue
            if u in pub:
                hit = (dd, u)
                break
            for v, l, w2 in g.get(u, []):
                if dd + l < dist.get(v, 1e18):
                    dist[v] = dd + l
                    prev[v] = (u, w2)
                    heapq.heappush(h, (dd + l, v))
        if hit:
            s, u = [], hit[1]
            while u in prev:
                if not s or s[-1] != prev[u][1]:
                    s.append(prev[u][1])
                u = prev[u][0]
            print(f"  [{label}] 고리 접점 -> 공도: {hit[0]:.0f}m via " + " -> ".join(s[::-1]))
        else:
            print(f"  [{label}] 고리 접점 -> 공도: 경로 없음")
    for wid in ("662321292", "468616601", "471667654", "452870642", "452870647"):
        w = ways[wid]
        print(f"  way{wid} {w['tags']} 안/밖 {''.join('I' if inside(nodes, rings, n) else 'O' for n in w['nds'])}"
              f" 시작 {fmt(nodes[w['nds'][0]])} 끝 {fmt(nodes[w['nds'][-1]])}")


if __name__ == "__main__":
    extra()
