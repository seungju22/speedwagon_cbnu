#!/usr/bin/env python3
# 맵세션7 추가 조사: 중문·후문 연결 후보 (읽기전용).
# 태그 보정·재변환 없음(재변환하면 road id 전면 재부여 -> test_drive.py 15-road 경로 무효).
# CARLA 불필요. 산출: docs/gate_candidates.md, docs/figures/12_gate_candidates.png
#
# 방법
#  - OSM(cbnu_campus_fixed_tags73.osm)의 highway way 전수를 캠퍼스 폴리곤(relation 6705106)
#    기준으로 안/밖/횡단 판정, 878 road 도로망(xodr) 포함 여부는 ENU 로 환산해 8m 간격
#    표본점이 xodr 도로에서 15m 이내인 비율로 판정(보정 잔차 최대 10.6m 라 기하 근사).
#  - 878 road 는 driving lane 이 전부 id<0 이라 단방향. OSM 후보의 방향은 oneway 태그로 검사.
#  - 경로 탐색은 OSM 그래프(차량 통행 가능 highway 만) 위 Dijkstra, 방향 검사 포함.
import heapq
import math
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from matplotlib.patches import Polygon as MplPolygon

fm.fontManager.addfont("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
matplotlib.rcParams["font.family"] = "Noto Sans CJK JP"
matplotlib.rcParams["axes.unicode_minus"] = False

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_geometry import parse_roads, parse_junctions, road_polyline
from classify_internal import load_polygon_rings, point_in_polygon
from geo_calibrate import (OSM_RAW_PATH, load_osm_nodes, junction_xodr_locations,
                           latlon_to_enu, fit_similarity, apply_transform)
from phase_e_route_check import lane_dirs, build

BASE = Path(__file__).resolve().parent.parent
XODR = BASE / "maps" / "cbnu_internal_only_localtm_tags73.xodr"
OSM_FIXED = BASE / "data" / "processed" / "cbnu_campus_fixed_tags73.osm"
POLY = BASE / "data" / "processed" / "cbnu_relation_polygon.json"
OUT_MD = BASE / "docs" / "gate_candidates.md"
OUT_PNG = BASE / "docs" / "figures" / "12_gate_candidates.png"

STOP_JUNGMUN = "12324240640"          # 충북대학교중문
STOPS_HUMUN = ["12324240641", "12324240642"]   # 충북대학교후문(동문 추정) 2개
STOPS_HOSP = ["4662426709", "12324240644", "12324240645"]  # 충북대학교병원
STOP_JEONGMUN = "6403450088"
NON_VEHICLE = {"footway", "path", "steps", "cycleway", "pedestrian", "bridleway", "corridor"}
TOL = 15.0     # 878 road 포함 판정 허용 거리(m)
SAMPLE = 8.0
# Osm2Odr 기본 변환 대상 highway(추정, 아래 검증 출력으로 확인): service/footway 등은 제외되므로
# 세션4 fix_tags 가 highway=unclassified 로 바꾼 73건만 변환됐다.
CONV_TYPES = {"motorway", "trunk", "primary", "secondary", "tertiary", "unclassified", "residential",
              "motorway_link", "trunk_link", "primary_link", "secondary_link", "tertiary_link"}
S4_CANDS = {"392632034": "번호1", "452870644": "번호48"}
JM_WAYS = {"792211896": "1순환로", "169523944": "내수동로", "293271139": "내수동로102번길"}


def load_osm(path):
    nodes, ways = {}, {}
    for ev, el in ET.iterparse(path, events=("end",)):
        if el.tag == "node":
            nodes[el.get("id")] = (float(el.get("lat")), float(el.get("lon")))
        elif el.tag == "way":
            ways[el.get("id")] = {
                "id": el.get("id"),
                "nds": [n.get("ref") for n in el.findall("nd")],
                "tags": {t.get("k"): t.get("v") for t in el.findall("tag")},
            }
            el.clear()
    return nodes, ways


def main():
    # ---------- xodr -> ENU ----------
    root = ET.parse(XODR).getroot()
    roads, juncs, ld = parse_roads(root), parse_junctions(root), lane_dirs(root)
    adj = build(roads, juncs, ld)
    osm_ids = load_osm_nodes(OSM_RAW_PATH)
    j_xy = junction_xodr_locations(roads, juncs)
    pairs = [(*j_xy[j], *osm_ids[juncs[j]["name"]]) for j in juncs
             if juncs[j]["name"] in osm_ids and j in j_xy]
    rl = sum(p[2] for p in pairs) / len(pairs)
    ro = sum(p[3] for p in pairs) / len(pairs)
    R, s, t, ang, res = fit_similarity([(p[0], p[1]) for p in pairs],
                                       [latlon_to_enu(p[2], p[3], rl, ro) for p in pairs])
    print(f"보정점 {len(pairs)}, 회전 {ang:.2f}deg, 잔차 최대 {max(res):.1f}m 평균 {sum(res)/len(res):.1f}m")

    road_pts = {}       # rid -> Nx2 ENU
    for rid, r in roads.items():
        pts = road_polyline(r, curve_samples=12)
        road_pts[rid] = np.array([apply_transform(R, s, t, x, y) for x, y in pts])
    # 표본화(2.5m 간격)
    all_xy, all_rid = [], []
    for rid, P in road_pts.items():
        for a, b in zip(P, P[1:]):
            n = max(1, int(np.hypot(*(b - a)) // 2.5))
            for k in range(n):
                all_xy.append(a + (b - a) * k / n)
                all_rid.append(rid)
        all_xy.append(P[-1]); all_rid.append(rid)
    all_xy = np.array(all_xy)

    def nearest_road(pt):
        d = np.hypot(all_xy[:, 0] - pt[0], all_xy[:, 1] - pt[1])
        k = int(d.argmin())
        return float(d[k]), all_rid[k]

    # ---------- xodr 연결 성분(무방향) + 종점 도달 집합(방향) ----------
    parent = {r: r for r in roads}

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        if a in parent and b in parent:
            parent[find(a)] = find(b)
    for rid, r in roads.items():
        for e in (r["pred"], r["succ"]):
            if e and e.get("elementType") == "road":
                union(rid, e["elementId"])
    for jid, j in juncs.items():
        for c in j["connections"]:
            union(c["incomingRoad"], c["connectingRoad"])
    comp_members = defaultdict(list)
    for r in roads:
        comp_members[find(r)].append(r)
    main_comp = find("1278")
    radj = defaultdict(list)
    for u, vs in adj.items():
        for v in vs:
            radj[v].append(u)
    can = {("1278", 1)}
    stack = [("1278", 1)]
    while stack:
        v = stack.pop()
        for u in radj.get(v, []):
            if u not in can:
                can.add(u); stack.append(u)
    can_roads = {r for r, _ in can}
    for rid in ("1333",):
        if rid in roads:
            e = road_pts[rid]
            print(f"road{rid} 길이 {roads[rid]['length']:.1f}m 양끝 ENU {e[0].round(0)} {e[-1].round(0)}")
    print(f"road {len(roads)}개, 무방향 성분 {len(comp_members)}개, 종점 성분 크기 {len(comp_members[main_comp])}, 종점 방향도달 road {len(can_roads)}개")
    comp_info = []
    for c, mem in sorted(comp_members.items(), key=lambda kv: -len(kv[1])):
        pts = np.vstack([road_pts[m] for m in mem])
        comp_info.append((c, len(mem), pts.mean(axis=0)))

    # ---------- OSM ----------
    nodes, ways = load_osm(OSM_FIXED)
    raw_nodes, raw_ways = load_osm(OSM_RAW_PATH)
    rings = load_polygon_rings(POLY)
    enu = {n: latlon_to_enu(la, lo, rl, ro) for n, (la, lo) in nodes.items()}
    inside = {n: point_in_polygon(lo, la, rings) for n, (la, lo) in nodes.items()}

    def seglen(a, b):
        return math.hypot(enu[a][0] - enu[b][0], enu[a][1] - enu[b][1])

    info = {}
    for wid, w in ways.items():
        hw = w["tags"].get("highway")
        if not hw:
            continue
        nds = [n for n in w["nds"] if n in enu]
        if len(nds) < 2:
            continue
        L = sum(seglen(a, b) for a, b in zip(nds, nds[1:]))
        Lin = sum(seglen(a, b) for a, b in zip(nds, nds[1:]) if inside[a] and inside[b])
        flags = {inside[n] for n in nds}
        cls = "안" if flags == {True} else ("밖" if flags == {False} else "횡단")
        # 표본점 -> 878 road 포함 판정
        samp = []
        for a, b in zip(nds, nds[1:]):
            A, B = np.array(enu[a]), np.array(enu[b])
            n = max(1, int(np.hypot(*(B - A)) // SAMPLE))
            samp.extend(A + (B - A) * k / n for k in range(n))
        samp.append(np.array(enu[nds[-1]]))
        hit, rset = 0, defaultdict(int)
        for p in samp:
            d, rid = nearest_road(p)
            if d <= TOL:
                hit += 1
                rset[rid] += 1
        frac = hit / len(samp)
        status = "포함" if frac >= 0.8 else ("일부" if frac > 0.2 else "없음")
        roads_hit = sorted(rset, key=lambda r: -rset[r])[:4]
        comps = {("종점성분" if find(r) == main_comp else f"고립{len(comp_members[find(r)])}") for r in rset}
        raw_tags = raw_ways.get(wid, {}).get("tags", {})
        info[wid] = dict(id=wid, nds=nds, tags=w["tags"], raw_tags=raw_tags, L=L, Lin=Lin, cls=cls,
                         frac=frac, status=status, roads=roads_hit, comps=comps,
                         vehicle=hw not in NON_VEHICLE,
                         fixed=bool(raw_tags) and raw_tags.get("highway") != hw,
                         raw_hw=raw_tags.get("highway"),
                         conv=(cls != "밖") and (bool(raw_tags) and raw_tags.get("highway") != hw
                                                or raw_tags.get("highway") in CONV_TYPES))
    stat = defaultdict(lambda: defaultdict(int))
    for v in info.values():
        if v["cls"] != "밖":
            key = (v["raw_hw"] or "?") + ("+parking_aisle" if v["raw_tags"].get("service") == "parking_aisle" else "")
            stat[key][("보정" if v["fixed"] else "원본") + "/" + v["status"]] += 1
    print("안·횡단 way 의 원본 highway 별 878 포함 현황:")
    for k, d in sorted(stat.items(), key=lambda kv: -sum(kv[1].values())):
        print("  ", k, dict(d))
    agree = defaultdict(int)
    for v in info.values():
        if v["cls"] != "밖":
            agree[("변환됨" if v["conv"] else "미변환") + "/" + v["status"]] += 1
    print("변환 판정(보정 또는 변환대상 highway) vs 근접 판정 일치도:", dict(agree))
    print(f"highway way {len(info)}개, 878 포함 {sum(1 for v in info.values() if v['status']=='포함')}, 일부 {sum(1 for v in info.values() if v['status']=='일부')}")

    # ---------- OSM 그래프 ----------
    def oneway_dir(tags):
        ow = tags.get("oneway")
        if ow in ("yes", "true", "1") or tags.get("junction") == "roundabout":
            return 1
        if ow == "-1":
            return -1
        return 0

    G = defaultdict(list)        # 무방향 (a,b,len,wid, 순방향허용, 역방향허용)
    for wid, v in info.items():
        if not v["vehicle"]:
            continue
        od = oneway_dir(v["tags"])
        for a, b in zip(v["nds"], v["nds"][1:]):
            d = seglen(a, b)
            G[a].append((b, d, wid, od in (0, 1)))     # a->b 가능?
            G[b].append((a, d, wid, od in (0, -1)))    # b->a 가능?

    main_nodes = {n for v in info.values() if v["conv"] and v["status"] in ("포함", "일부") and "종점성분" in v["comps"] for n in v["nds"]}

    def dijkstra(sources, targets, ext_pen=1.0, directed=True, forbid_ext=False):
        dist, prev = {}, {}
        pq = []
        for sn in sources:
            if sn in G:
                dist[sn] = 0.0
                heapq.heappush(pq, (0.0, sn))
        while pq:
            c, u = heapq.heappop(pq)
            if c > dist[u]:
                continue
            if u in targets:
                chain = []
                while u in prev:
                    chain.append(prev[u])
                    u = prev[u][0]
                return c, chain[::-1]
            for v, d, wid, ok in G[u]:
                if directed and not ok:
                    continue
                cls = info[wid]["cls"]
                if forbid_ext and cls == "밖":
                    continue
                w = d * (ext_pen if cls == "밖" else 1.0)
                if c + w < dist.get(v, 1e18):
                    dist[v] = c + w
                    prev[v] = (u, wid, d)
                    heapq.heappush(pq, (c + w, v))
        return None, None

    def nearest_gnode(xy_or_id):
        p = enu[xy_or_id]
        return min((n for n in G), key=lambda n: math.hypot(enu[n][0] - p[0], enu[n][1] - p[1]))

    def chain_ways(chain):
        seq = []
        for _, wid, d in chain:
            if not seq or seq[-1][0] != wid:
                seq.append([wid, 0.0])
            seq[-1][1] += d
        return seq

    results = {}

    def run(label, src_nodes, tgt_nodes, ext_pen, directed):
        c, ch = dijkstra(src_nodes, set(tgt_nodes), ext_pen=ext_pen, directed=directed)
        if ch is None:
            return None
        seq = chain_ways(ch)
        # 878 도로망에 이미 포함된 앞뒤 구간은 후보 제외, 없음/일부만 "추가 필요 구간"
        return dict(label=label, cost=c, seq=seq, start=ch[0][0] if ch else None, end=ch[-1][0] if ch else None)

    # 중문: 도로망 -> 중문 정류장 (진출) 과 반대(진입). directed 는 검색 방향 그대로.
    jm_node = nearest_gnode(STOP_JUNGMUN)
    print("중문 정류장 최근접 그래프 노드", jm_node, f"{math.hypot(*(np.array(enu[jm_node]) - np.array(enu[STOP_JUNGMUN]))):.0f}m")
    for tag, ep in (("최단(공도 가중 1)", 1.0), ("캠퍼스 우선(공도 가중 5)", 5.0)):
        results[("중문", tag, "진출")] = run(tag, main_nodes, {jm_node}, ep, True)
        # 진입: 정류장 -> 도로망
        rr = dijkstra({jm_node}, main_nodes, ext_pen=ep, directed=True)
        if rr[1] is not None:
            results[("중문", tag, "진입")] = dict(label=tag, cost=rr[0], seq=chain_ways(rr[1]))
        else:
            results[("중문", tag, "진입")] = None

    # 후문: 병원 고립 성분 도로 노드
    hosp_pt = np.array(latlon_to_enu(36.6238, 127.4625, rl, ro))
    hosp_comp = None
    for c, size, ctr in comp_info:
        if find(c) != main_comp and np.hypot(*(ctr - hosp_pt)) < 300:
            hosp_comp = (c, size, ctr) if hosp_comp is None or size > hosp_comp[1] else hosp_comp
    hosp_nodes = set()
    if hosp_comp:
        for wid, v in info.items():
            if v["conv"] and v["status"] in ("포함", "일부") and any(find(r) == hosp_comp[0] for r in v["roads"]):
                hosp_nodes |= set(v["nds"])
    hm_nodes = {nearest_gnode(n) for n in STOPS_HUMUN}
    hp_nodes = {nearest_gnode(n) for n in STOPS_HOSP}
    for tag, ep in (("최단(공도 가중 1)", 1.0), ("캠퍼스 우선(공도 가중 5)", 5.0)):
        for name, tg in (("후문 정류장", hm_nodes), ("병원 정류장", hp_nodes), ("병원 고립성분", hosp_nodes)):
            if not tg:
                continue
            for dirn, (src, dst) in (("진출", (main_nodes, tg)), ("진입", (tg, main_nodes))):
                cst, ch = dijkstra(src, set(dst), ext_pen=ep, directed=True)
                results[("후문:" + name, tag, dirn)] = None if ch is None else dict(label=tag, cost=cst, seq=chain_ways(ch))

    # ---------- 출력 ----------
    lines = []
    P = lines.append
    P("# 중문·후문 연결 후보 조사 (맵세션7 추가, 읽기전용)\n")
    P("재변환 금지(road id 전면 재부여 -> test_drive.py 15-road 경로 무효) 조건 하의 조사·후보 선정만. 태그 보정·변환 없음.\n")
    P("- 판정 기준: 캠퍼스 폴리곤(relation 6705106) 기준 way 노드가 전부 안=안, 전부 밖=밖, 섞임=횡단")
    P(f"- 878 road 포함 판정: way 를 {SAMPLE:.0f}m 간격 표본화, xodr 도로 {TOL:.0f}m 이내 비율 80% 이상=포함 / 20% 이하=없음 / 사이=일부 (보정 잔차 최대 {max(res):.1f}m 라 근사)")
    P("- 방향: 878 road 는 전부 단방향. OSM 후보 방향은 oneway 태그로 검사(탐색은 방향 준수)\n")

    def conv_label(v):
        if v["cls"] == "밖":
            return "미변환(폴리곤 밖)"
        if v["conv"]:
            comp = "/".join(sorted(v["comps"])) if v["comps"] else "-"
            return f"변환됨 road{','.join(v['roads'][:2])} [{comp}]"
        return f"미변환(원본 {v['raw_hw']}, 보정 73건 아님; 근접 {v['status']} {v['frac']*100:.0f}%)"

    def wrow(v, use):
        t = v["tags"]
        tg = ",".join(f"{k}={t[k]}" for k in ("highway", "service", "name", "oneway", "surface") if k in t)
        chg = " (태그보정됨: %s->%s)" % (v["raw_hw"], t.get("highway")) if v["fixed"] else ""
        pct = 100 * v["Lin"] / v["L"] if v["L"] else 0
        return (f"| {v['id']} | {tg}{chg} | {v['L']:.0f} | {v['cls']}(안쪽 {pct:.0f}%) | "
                f"{conv_label(v)} | {use} |")

    G_JM, G_A, G_B = "중문", "후문A(동측 정류장 진입로)", "후문B(병원측: 번호1·48 체인)"
    cand = {}     # wid -> use set
    for key, r in results.items():
        if not r:
            continue
        if key[0] == "중문":
            grp = G_JM
        elif key[0] == "후문:후문 정류장":
            if key[2] != "진입":      # 진출은 일방통행 1순환로 우회(공도 900m)라 후보에서 제외
                continue
            grp = G_A
        else:
            grp = G_B
        for wid, d in r["seq"]:
            cand.setdefault(wid, set()).add(grp)
    for wid in S4_CANDS:
        cand.setdefault(wid, set()).add(G_B)
    for wid in JM_WAYS:
        cand.setdefault(wid, set()).add(G_JM)

    P("## 0. 878 road 도로망 성분")
    P(f"- 종점(road1278)을 포함한 성분: road {len(comp_members[main_comp])}개. 종점 방향도달 road {len(can_roads)}개")
    for c, size, ctr in comp_info[:6]:
        la, lo = None, None
        e, n = ctr
        la = rl + n / 111000.0
        lo = ro + e / (111000.0 * math.cos(math.radians(rl)))
        tagc = "종점 성분" if c == main_comp else "고립"
        P(f"- 성분 대표 {c}: road {size}개, 중심 약 ({la:.4f},{lo:.4f}) {tagc}")
    P(f"- 병원 권역 고립 성분(세션3, 약 36.6238,127.4625 300m 이내): " + (f"road {hosp_comp[1]}개" if hosp_comp else "없음(878 road 에 병원 권역이 없거나 종점 성분에 포함)"))
    P("")

    def dump(title, key_prefix):
        P(f"## {title}")
        for key, r in results.items():
            if not key[0].startswith(key_prefix):
                continue
            if r is None:
                P(f"- [{key[0]} / {key[1]} / {key[2]}] 방향 준수 경로 없음")
                continue
            steps = " -> ".join(f"way{w}({info[w]['tags'].get('name') or info[w]['tags'].get('highway')},{d:.0f}m,{info[w]['cls']},{info[w]['status']})" for w, d in r["seq"])
            P(f"- [{key[0]} / {key[1]} / {key[2]}] 총 {sum(d for _, d in r['seq']):.0f}m: {steps}")
        P("")

    dump("1. 중문 연결 경로 탐색(OSM, 방향 준수)", "중문")
    dump("2. 후문 연결 경로 탐색(OSM, 방향 준수)", "후문")

    # 접속점: 후보 way 양끝 노드를 공유하는 "변환된" way / 그 way 가 대응하는 xodr road
    node_conv = defaultdict(list)
    for v in info.values():
        if v["conv"]:
            for n in v["nds"]:
                node_conv[n].append(v)
    hw1333 = [v for v in info.values() if v["conv"] and "1333" in v["roads"]]
    P("## 3-0. road1333(병원 고립 성분) 대응 way")
    P(f"- xodr road1333 길이 {roads['1333']['length']:.1f}m. 변환된 way 중 road1333 근접: " +
      (", ".join(f"way{v['id']}({v['L']:.0f}m,{v['tags'].get('highway')})" for v in hw1333) or "없음"))
    P("")
    P("## 3-1. 후보 way 양끝의 접속 상태 (끝 노드를 공유하는 변환된 way = 도로망에 실제로 붙는 지점)")
    for wid in sorted(cand, key=lambda w: (sorted(cand[w])[0], -info[w]["L"])):
        v = info[wid]
        if v["cls"] == "밖":
            continue
        ends = []
        for nm, n in (("시작", v["nds"][0]), ("끝", v["nds"][-1])):
            att = [f"way{a['id']}->road{','.join(a['roads'][:2])}" for a in node_conv.get(n, []) if a["id"] != wid]
            d, rid = nearest_road(np.array(enu[n]))
            ends.append(f"{nm}노드 {'변환way 공유: ' + ';'.join(att[:3]) if att else '변환way 공유 없음'} (xodr 최근접 road{rid} {d:.0f}m)")
        P(f"- way{wid}: " + " / ".join(ends))
    P("")
    P("## 3. 후보 목록 표 (way id, 현재 태그, 길이, 폴리곤 판정, 878 road 포함/성분, 노선)")
    P("| way id | 현재 태그 | 길이(m) | 폴리곤 | 878 road 변환 여부 | 노선 |")
    P("|---|---|---|---|---|---|")
    for wid in sorted(cand, key=lambda w: (sorted(cand[w])[0], -info[w]["L"])):
        v = info[wid]
        P(wrow(v, "/".join(sorted(cand[wid])) + (f" [세션4 {S4_CANDS[wid]}]" if wid in S4_CANDS else "")))
    P("")
    out = "\n".join(lines)
    OUT_MD.write_text(out + "\n")
    print(out)

    # ---------- 그림 ----------
    bld = []
    for wid, w in ways.items():
        if "building" in w["tags"] and all(n in enu for n in w["nds"]):
            bld.append([enu[n] for n in w["nds"]])
    ring = np.array([latlon_to_enu(y, x, rl, ro) for x, y in rings[0]])

    def draw(ax, xlim, ylim, lw_c=3.0, labels=True):
        for pts in bld:
            ax.add_patch(MplPolygon(pts, closed=True, fc="0.88", ec="0.72", lw=0.3, zorder=1))
        ax.plot(ring[:, 0], ring[:, 1], ls="--", color="tab:green", lw=1.0, zorder=2, label="캠퍼스 폴리곤(relation 6705106)")
        for rid, Pn in road_pts.items():
            ax.plot(Pn[:, 0], Pn[:, 1], color="black", lw=1.0, zorder=3)
        colors = {G_JM: "tab:orange", G_A: "tab:purple", G_B: "tab:brown"}
        drawn = set()
        for wid, uses in cand.items():
            v = info[wid]
            xy = np.array([enu[n] for n in v["nds"]])
            use = sorted(uses)[0] if len(uses) == 1 else (G_B if G_B in uses else sorted(uses)[0])
            ls = "-" if v["cls"] != "밖" else ":"
            ax.plot(xy[:, 0], xy[:, 1], color=colors[use], lw=lw_c, ls=ls, zorder=4, solid_capstyle="round")
            if labels and (xlim[1] - xlim[0]) < 1500:
                m = xy[len(xy) // 2]
                if xlim[0] < m[0] < xlim[1] and ylim[0] < m[1] < ylim[1]:
                    nm = v["tags"].get("name") or ""
                    ax.annotate(f"{wid[-5:]} {nm}", m, fontsize=6.5, color=colors[use], zorder=6,
                                xytext=(3, 3), textcoords="offset points")
        for sid, mk, col, lab in ([(STOP_JUNGMUN, "*", "tab:red", "중문 정류장")] +
                                  [(x, "P", "tab:blue", "후문(동문) 추정: OSM '충북대학교후문' 정류장") for x in STOPS_HUMUN] +
                                  [(x, "^", "tab:cyan", "충북대학교병원 정류장") for x in STOPS_HOSP] +
                                  [(STOP_JEONGMUN, "s", "tab:green", "정문 정류장")]):
            p = enu[sid]
            if lab not in drawn:
                ax.scatter(*p, marker=mk, s=130, color=col, edgecolors="k", zorder=7, label=lab)
                drawn.add(lab)
            else:
                ax.scatter(*p, marker=mk, s=130, color=col, edgecolors="k", zorder=7)
        ax.set_xlim(*xlim); ax.set_ylim(*ylim); ax.set_aspect("equal")
        ax.grid(True, lw=0.3, alpha=0.5)
        ax.set_xlabel("동쪽 (m)"); ax.set_ylabel("북쪽 (m)")

    allx = np.concatenate([P_[:, 0] for P_ in road_pts.values()] + [ring[:, 0]])
    ally = np.concatenate([P_[:, 1] for P_ in road_pts.values()] + [ring[:, 1]])
    pad = 150
    ovx = (allx.min() - pad, allx.max() + pad)
    ovy = (ally.min() - pad, ally.max() + pad)
    jm, hm = np.array(enu[STOP_JUNGMUN]), np.mean([enu[x] for x in STOPS_HUMUN], axis=0)
    fig = plt.figure(figsize=(20, 12), dpi=110)
    gs = fig.add_gridspec(2, 2, width_ratios=[1.25, 1], hspace=0.28, wspace=0.12)
    ax0 = fig.add_subplot(gs[:, 0]); ax1 = fig.add_subplot(gs[0, 1]); ax2 = fig.add_subplot(gs[1, 1])
    draw(ax0, ovx, ovy, lw_c=2.5, labels=False)
    ax0.set_title("전체 (진북 위쪽) 검정=878 road / 주황=중문 / 보라=후문A(동측) / 갈색=후문B(병원측) / 점선=폴리곤 밖 way", fontsize=10)
    ax0.legend(loc="lower left", fontsize=8, framealpha=0.9)
    ax0.plot([ovx[1] - 600, ovx[1] - 100], [ovy[0] + 60] * 2, color="k", lw=3)
    ax0.text(ovx[1] - 600, ovy[0] + 80, "500 m", fontsize=9)
    ax0.annotate("N", (ovx[1] - 120, ovy[1] - 200), (ovx[1] - 120, ovy[1] - 380),
                 arrowprops=dict(arrowstyle="-|>", lw=2), ha="center", fontsize=14)
    draw(ax1, (jm[0] - 500, jm[0] + 250), (jm[1] - 350, jm[1] + 300))
    ax1.set_title("중문 확대")
    draw(ax2, (hm[0] - 450, hm[0] + 450), (min(hm[1], enu[STOPS_HOSP[0]][1]) - 330, hm[1] + 250))
    ax2.set_title("후문·병원 권역 확대")
    fig.suptitle("12 중문·후문 연결 후보 (읽기전용 조사, 채택 전)", fontsize=14)
    fig.savefig(OUT_PNG, bbox_inches="tight")
    print("그림:", OUT_PNG)


if __name__ == "__main__":
    main()
