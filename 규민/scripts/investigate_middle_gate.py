#!/usr/bin/env python3
# 맵세션7 추가 조사 2: (1) 후문B 확정 표(번호1 밖 구간 분석), (2) 중문 직전 도로망 끝점(dead end) 5개.
# 읽기전용. 태그 보정·재변환 없음. CARLA 불필요.
# 산출: docs/humun_b_confirmed.md, docs/middle_gate_deadends.md, docs/figures/13_middle_gate_deadends.png
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
from investigate_gates import load_osm, CONV_TYPES, NON_VEHICLE

BASE = Path(__file__).resolve().parent.parent
XODR = BASE / "maps" / "cbnu_internal_only_localtm_tags73.xodr"
OSM_FIXED = BASE / "data" / "processed" / "cbnu_campus_fixed_tags73.osm"
POLY = BASE / "data" / "processed" / "cbnu_relation_polygon.json"
OUT_B = BASE / "docs" / "humun_b_confirmed.md"
OUT_MD = BASE / "docs" / "middle_gate_deadends.md"
OUT_PNG = BASE / "docs" / "figures" / "13_middle_gate_deadends.png"

STOP_JUNGMUN = "12324240640"
HUMUN_B_WAYS = ["452870644", "392632034", "481945510", "481943505", "481475019", "481943503", "481945509"]
HUMUN_B_EXCLUDED = ["662321292", "792211884", "774924320"]   # 폴리곤 밖 연결/공도
COLORS = ["tab:red", "tab:blue", "tab:green", "tab:purple", "tab:orange"]


def main():
    root = ET.parse(XODR).getroot()
    roads, juncs = parse_roads(root), parse_junctions(root)
    osm_ids = load_osm_nodes(OSM_RAW_PATH)
    j_xy = junction_xodr_locations(roads, juncs)
    pairs = [(*j_xy[j], *osm_ids[juncs[j]["name"]]) for j in juncs
             if juncs[j]["name"] in osm_ids and j in j_xy]
    rl = sum(p[2] for p in pairs) / len(pairs)
    ro = sum(p[3] for p in pairs) / len(pairs)
    R, s, t, ang, res = fit_similarity([(p[0], p[1]) for p in pairs],
                                       [latlon_to_enu(p[2], p[3], rl, ro) for p in pairs])
    road_pts = {rid: np.array([apply_transform(R, s, t, x, y) for x, y in road_polyline(r, curve_samples=12)])
                for rid, r in roads.items()}

    nodes, ways = load_osm(OSM_FIXED)
    _, raw_ways = load_osm(OSM_RAW_PATH)
    rings = load_polygon_rings(POLY)
    enu = {n: latlon_to_enu(la, lo, rl, ro) for n, (la, lo) in nodes.items()}
    inside = {n: point_in_polygon(lo, la, rings) for n, (la, lo) in nodes.items()}
    ring = np.array([latlon_to_enu(y, x, rl, ro) for x, y in rings[0]])
    ntags = {}
    for ev, el in ET.iterparse(OSM_FIXED, events=("end",)):
        if el.tag == "node":
            tg = {c.get("k"): c.get("v") for c in el.findall("tag")}
            if tg:
                ntags[el.get("id")] = tg
            el.clear()

    def d2(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def wlen(nds):
        return sum(d2(enu[a], enu[b]) for a, b in zip(nds, nds[1:]))

    winfo = {}
    for wid, w in ways.items():
        hw = w["tags"].get("highway")
        if not hw:
            continue
        nds = [n for n in w["nds"] if n in enu]
        if len(nds) < 2:
            continue
        fl = {inside[n] for n in nds}
        cls = "안" if fl == {True} else ("밖" if fl == {False} else "횡단")
        rt = raw_ways.get(wid, {}).get("tags", {})
        fixed = bool(rt) and rt.get("highway") != hw
        conv = cls != "밖" and (fixed or rt.get("highway") in CONV_TYPES)
        Lin = sum(d2(enu[a], enu[b]) for a, b in zip(nds, nds[1:]) if inside[a] and inside[b])
        winfo[wid] = dict(id=wid, nds=nds, tags=w["tags"], L=wlen(nds), Lin=Lin, cls=cls, conv=conv, fixed=fixed,
                          raw_hw=rt.get("highway"))

    def tagstr(v):
        tg = v["tags"]
        return ",".join(f"{k}={tg[k]}" for k in ("highway", "service", "name", "oneway") if k in tg)

    # =====================================================================
    # (1) 후문B 확정 표
    # =====================================================================
    B = []
    B.append("# 후문B(병원측) 채택 후보 확정 표 (맵세션7 추가)\n")
    B.append("사용자 판정: 후문=B(병원측). 캠퍼스 안쪽 구간만 채택, 폴리곤 밖 공도(1순환로 등) 제외. 재변환은 보류(이 표는 후보 확정까지).\n")
    B.append("| 구분 | way id | 태그 | 길이(m) | 폴리곤 판정 | 안쪽/밖 길이(m) | 변환 여부 | 방향 | 채택 |")
    B.append("|---|---|---|---|---|---|---|---|---|")
    role = {"452870644": "번호48", "392632034": "번호1",
            "481945510": "서비스way(도로망 접속)", "481943505": "서비스way", "481475019": "서비스way",
            "481943503": "진입 쌍", "481945509": "진입 쌍"}
    for wid in HUMUN_B_WAYS + HUMUN_B_EXCLUDED:
        v = winfo[wid]
        ow = v["tags"].get("oneway")
        conv = "변환됨" if v["conv"] else ("미변환(폴리곤 밖)" if v["cls"] == "밖" else "미변환(원본 service)")
        adopt = "채택(안쪽 구간)" if wid in HUMUN_B_WAYS else "제외(폴리곤 밖)"
        if wid == "392632034":
            adopt = "채택(way 전체 105m 중 안쪽 %.0fm; 밖 %.0fm 처리는 아래)" % (v["Lin"], v["L"] - v["Lin"])
        B.append(f"| {role.get(wid, '연결/공도')} | {wid} | {tagstr(v)} | {v['L']:.0f} | {v['cls']} | "
                 f"{v['Lin']:.0f}/{v['L'] - v['Lin']:.0f} | {conv} | {'일방통행(way 방향)' if ow=='yes' else '양방향'} | {adopt} |")
    B.append("")
    B.append("이미 변환된 이웃: way452870641+way452870648 = road1333(485.6m, 고립 성분). 번호48 시작 노드와 번호1 끝 노드가 이 way들과 노드를 공유한다.\n")

    # 번호1 밖 구간
    v = winfo["392632034"]
    B.append("## 번호1(way392632034) 밖으로 나가는 부분 분석")
    seq = []
    for i, n in enumerate(v["nds"]):
        seq.append((i, n, inside[n], enu[n]))
    B.append(f"- 노드 {len(seq)}개, 순서대로 안/밖: " + "".join("I" if x[2] else "O" for x in seq))
    out_nodes = [x for x in seq if not x[2]]
    cross = None
    for a, b in zip(seq, seq[1:]):
        if a[2] != b[2]:
            cross = (a, b)
    if cross:
        a, b = cross
        la, lo = nodes[a[1]]
        B.append(f"- 경계 횡단 구간: 노드{a[1]}({'안' if a[2] else '밖'}) -> 노드{b[1]}({'안' if b[2] else '밖'}), 횡단 부근 ({la:.6f},{lo:.6f})")
    endn = v["nds"][0] if not inside[v["nds"][0]] else v["nds"][-1]
    B.append(f"- 밖쪽 끝 노드 {endn} ({nodes[endn][0]:.6f},{nodes[endn][1]:.6f}) 태그: {ntags.get(endn) or '없음'}")
    outside_len = v["L"] - v["Lin"]
    B.append(f"- 밖 구간 길이(경계 횡단 세그먼트 포함 개략): {outside_len:.0f}m ; 이 way 자체에는 name 태그 없음, service=parking_aisle")
    B.append("- 밖 끝 노드 주변 way(공유 노드):")
    for wid2, w2 in winfo.items():
        if wid2 != "392632034" and endn in w2["nds"]:
            B.append(f"  - way{wid2} {tagstr(w2)} {w2['L']:.0f}m {w2['cls']}")
    for n in v["nds"]:
        if n in ntags:
            B.append(f"  - 노드{n} 태그 {ntags[n]} ({'안' if inside[n] else '밖'})")
    shared = [w2 for wid2, w2 in winfo.items() if wid2 != "392632034" and endn in w2["nds"]]
    on_public = [w2 for w2 in shared if w2["tags"].get("name") == "1순환로" or str(w2["tags"].get("highway", "")).endswith("_link")]
    B.append(f"- 판정: 밖 끝 노드가 곧바로 공도({len(on_public)}개 way: " + ", ".join(f"way{w2['id']}" for w2 in on_public) +
             f")와 접속한다. 즉 way392632034 의 밖 부분은 폴리곤 경계를 넘어 1순환로에 닿는 마지막 한 세그먼트(최대 {outside_len:.0f}m, 이름 없음)뿐으로, 게이트 통과(접속)부다. 공도 구간은 그 너머(1순환로 계열 way)다.")
    B.append("- 제안: 번호1은 way 단위로만 변환되므로 way 전체(105m, 밖 최대 %.0fm)를 채택하고 1순환로 계열은 제외. 접속 세그먼트를 잘라내려면 OSM way 분할 편집이 필요(재변환 단계에서 사용자 결정)." % outside_len)
    OUT_B.write_text("\n".join(B) + "\n")
    print("\n".join(B))

    # =====================================================================
    # (2) 중문 직전 도로망 끝점
    # =====================================================================
    stop = np.array(enu[STOP_JUNGMUN])
    # 끝점 정의: 변환된 way 의 첫/끝 노드 중 다른 변환 way 와 접점이 없는 노드(OSM 기준 진짜 막다른 끝).
    # xodr 는 이런 끝을 U턴 junction 으로 처리해 pred/succ 링크가 있는 것처럼 보이므로 링크 유무로는 못 찾는다.
    cnt = defaultdict(int)
    for v in winfo.values():
        if v["conv"]:
            for n in v["nds"]:
                cnt[n] += 1
    name2j = {j["name"]: jid for jid, j in juncs.items()}
    ends = []
    for v in winfo.values():
        if not v["conv"]:
            continue
        for n in (v["nds"][0], v["nds"][-1]):
            if cnt[n] == 1 and inside[n] is not None:
                jid = name2j.get(n)
                inc = sorted({c["incomingRoad"] for c in juncs[jid]["connections"]}) if jid else []
                ends.append(dict(node=n, own=v["id"], pt=np.array(enu[n]), dist=float(d2(enu[n], stop)), jid=jid, inc=inc))
    ends.sort(key=lambda e: e["dist"])
    unlinked = sum(1 for r in roads.values() if r["junction"] == "-1" and (r["pred"] is None or r["succ"] is None))
    print(f"\n끝점(변환 way 의 막다른 끝) 총 {len(ends)}개, (참고) xodr 링크 없는 road 끝 {unlinked}개")
    top = ends[:5]

    # OSM 그래프
    G = defaultdict(list)
    for wid, v in winfo.items():
        od = v["tags"].get("oneway")
        vehicle = v["raw_hw"] not in NON_VEHICLE and v["tags"].get("highway") not in NON_VEHICLE
        for a, b in zip(v["nds"], v["nds"][1:]):
            d = d2(enu[a], enu[b])
            G[a].append((b, d, wid, od in (None, "no", "yes", "true", "1") and od != "-1", vehicle))
            G[b].append((a, d, wid, od in (None, "no", "-1"), vehicle))
    stop_node = min(G, key=lambda n: d2(enu[n], stop))

    def dijkstra(src, dst, vehicle_only, directed):
        dist, prev = {src: 0.0}, {}
        pq = [(0.0, src)]
        while pq:
            c, u = heapq.heappop(pq)
            if c > dist[u]:
                continue
            if u == dst:
                ch = []
                while u in prev:
                    ch.append(prev[u])
                    u = prev[u][0]
                return c, ch[::-1]
            for v_, d, wid, ok, veh in G[u]:
                if directed and not ok:
                    continue
                if vehicle_only and not veh:
                    continue
                if c + d < dist.get(v_, 1e18):
                    dist[v_] = c + d
                    prev[v_] = (u, wid, d)
                    heapq.heappush(pq, (c + d, v_))
        return None, None

    M = ["# 중문 직전 878 road 끝점(dead end) 조사 (맵세션7 추가, 읽기전용)\n",
         "- 끝점 = 변환된 way 의 첫/끝 노드 중 다른 변환 way 와 접점이 없는 막다른 OSM 노드(xodr 에서는 U턴 junction 으로 표현되어 링크 유무로는 안 잡힘). 중문 정류장(36.633602,127.460506) 직선거리 순 5개",
         f"- (참고) xodr 에서 pred/succ 링크 자체가 없는 road 끝은 {unlinked}개이며 중문 정류장에서 1000m 이상 떨어진 병원 권역·서남측뿐이라 이번 대상이 아님",
         "- 좌표는 OSM 노드 위경도 기준. xodr road 는 보정 잔차 최대 10.6m 오차. 이어지는 way = 끝 노드에서 30m 이내에 닿는 다른 way",
         f"- 중문 정류장 최근접 OSM 그래프 노드 {stop_node} ({d2(enu[stop_node], stop):.0f}m)\n"]
    # 중문 게이트 추정점 = 캠퍼스 폴리곤 경계에서 중문 정류장에 가장 가까운 점(OSM 에 게이트 지물이 없어 이 방식으로 추정)
    best_g = None
    for a_, b_ in zip(ring, ring[1:]):
        ab = b_ - a_
        tt = max(0.0, min(1.0, float(np.dot(stop - a_, ab) / (np.dot(ab, ab) + 1e-9))))
        q = a_ + tt * ab
        dd = d2(q, stop)
        if best_g is None or dd < best_g[0]:
            best_g = (dd, q)
    gate_pt = best_g[1]
    glat = rl + gate_pt[1] / 111000.0
    glon = ro + gate_pt[0] / (111000.0 * math.cos(math.radians(rl)))
    M.append(f"- 중문 게이트 추정점(폴리곤 경계에서 정류장에 가장 가까운 점): ({glat:.6f},{glon:.6f}) ENU ({gate_pt[0]:.0f},{gate_pt[1]:.0f}), 정류장까지 {best_g[0]:.0f}m. OSM 에 게이트 지물이 없어 추정")
    near_g = []
    for wid_, v_ in winfo.items():
        dm = min(d2(enu[n_], gate_pt) for n_ in v_["nds"])
        if dm <= 60:
            near_g.append((dm, wid_, v_))
    near_g.sort(key=lambda x: x[0])
    M.append("- 게이트 추정점 60m 내 way: " + (", ".join(f"way{w_}({tagstr(v_)},{v_['L']:.0f}m,{v_['cls']},{'변환됨' if v_['conv'] else '미변환'},{dm:.0f}m)" for dm, w_, v_ in near_g[:8]) or "없음"))
    M.append("")
    found = []
    for k, e in enumerate(top):
        pt = e["pt"]
        la, lo = nodes[e["node"]]
        own, n0 = e["own"], e["node"]
        M.append(f"## 끝점 {k + 1}: OSM 노드 {n0}" + (f" (junction{e['jid']}, 들어오는 road {','.join(e['inc'])})" if e["jid"] else " (junction 없음)"))
        M.append(f"- 좌표 ({la:.6f},{lo:.6f}) ENU ({pt[0]:.1f},{pt[1]:.1f}), 중문 정류장까지 {e['dist']:.0f}m, 폴리곤 {'안' if inside[n0] else '밖'}")
        ov = winfo[own]
        M.append(f"- 대응 자기 way: way{own} {tagstr(ov)} {ov['L']:.0f}m {ov['cls']} (이 끝에서 끝나는 way)")
        cont = []
        for wid, v in winfo.items():
            if wid == own:
                continue
            dmin = min(d2(enu[n], enu[n0]) for n in v["nds"])
            if dmin <= 30:
                cont.append((dmin, wid, v))
        cont.sort(key=lambda x: x[0])
        if not cont:
            M.append("- 이어지는 OSM way: 없음(30m 내)")
        for dmin, wid, v in cont[:6]:
            share = "노드 공유" if n0 in v["nds"] else f"{dmin:.0f}m 이격"
            M.append(f"- 이어지는 way{wid}: {tagstr(v)} {v['L']:.0f}m {v['cls']}(안쪽 {v['Lin']:.0f}m) {'변환됨' if v['conv'] else '미변환'} [{share}]")
        found.append(dict(k=k, e=e, own=own, n0=n0, cont=[c[1] for c in cont[:6]]))
        # 게이트 도달 여부
        srcs = [n0] + [n for _, wid, v in cont[:3] for n in v["nds"] if d2(enu[n], enu[n0]) <= 30][:1]
        for label, veh, dr in (("차량 way 만·방향 준수(진출)", True, True), ("모든 highway·무방향(보행로 포함)", False, False)):
            res_ = None
            for sn in srcs:
                c_, ch_ = dijkstra(sn, stop_node, veh, dr)
                if ch_ is not None and (res_ is None or c_ < res_[0]):
                    res_ = (c_, ch_)
            if res_ is None:
                M.append(f"- 중문 정류장 도달({label}): 도달 못함")
                continue
            c_, ch_ = res_
            segs = []
            for _, wid, d in ch_:
                if not segs or segs[-1][0] != wid:
                    segs.append([wid, 0.0])
                segs[-1][1] += d
            cross_pt = None
            for a_, wid, d in ch_:
                pass
            # 폴리곤 경계 횡단 지점: 경로 노드열에서 안->밖 첫 전이
            path_nodes = [ch_[0][0]] + [x[0] for x in ch_[1:]]
            ends_n = ch_[-1][0]
            for a_, b_ in zip(path_nodes, path_nodes[1:] + [stop_node]):
                if inside[a_] and not inside[b_]:
                    cross_pt = b_
                    break
            gd = min(d2(enu[n], gate_pt) for n in path_nodes + [stop_node])
            gates = [n for n in path_nodes if n in ntags and any(kk in ntags[n] for kk in ("barrier", "entrance"))]
            M.append(f"- 중문 정류장 도달({label}): 총 {c_:.0f}m, way {len(segs)}개: " +
                     " -> ".join(f"way{w}({winfo[w]['tags'].get('name') or winfo[w]['tags'].get('highway')},{d:.0f}m,{winfo[w]['cls']})" for w, d in segs))
            if cross_pt:
                cl = nodes[cross_pt]
                M.append(f"  - 폴리곤 경계 통과 지점 약 ({cl[0]:.6f},{cl[1]:.6f}), 중문 정류장까지 직선 {d2(enu[cross_pt], stop):.0f}m")
            else:
                M.append("  - 경로 중 폴리곤 안->밖 전이 없음(시작부터 밖이거나 끝까지 안)")
            M.append(f"  - 경로가 중문 게이트 추정점에 가장 가까이 지나는 거리: {gd:.0f}m")
            if gates:
                M.append("  - 경로상 게이트/출입구 노드: " + ", ".join(f"{g}{ntags[g]}" for g in gates))
        M.append("")
    M.append("## 공통 관찰")
    M.append("- 5개 끝점 모두 이어지는 OSM way 가 원본 service(parking_aisle) 등 미변환 way 다. 끊긴 원인은 '연결 실패'가 아니라 세션4 fix_tags 73건에 들어가지 않아 변환기에서 빠진 것이다(중문 주변 도로 전체가 같은 패턴).")
    M.append("- 끝점 5는 앞서 주황 후보(way442595018)의 시작점과 같다. 사용자 판정으로 주황은 중문이 아니라 별개 출구다.")
    OUT_MD.write_text("\n".join(M) + "\n")
    print("\n".join(M))

    # =====================================================================
    # 그림 13
    # =====================================================================
    pts_all = [e["pt"] for e in top] + [stop]
    xs, ys = [p[0] for p in pts_all], [p[1] for p in pts_all]
    pad = 120
    x0, x1, y0, y1 = min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    half = max(x1 - x0, y1 - y0) / 2
    x0, x1, y0, y1 = cx - half, cx + half, cy - half, cy + half
    fig, ax = plt.subplots(figsize=(13, 13), dpi=110)
    for wid, w in ways.items():
        if "building" in w["tags"]:
            ax.add_patch(MplPolygon([enu[n] for n in w["nds"] if n in enu], closed=True, fc="0.9", ec="0.75", lw=0.3, zorder=1))
    # 모든 OSM highway(참고, 연한 회색)
    for wid, v in winfo.items():
        xy = np.array([enu[n] for n in v["nds"]])
        if xy[:, 0].max() < x0 or xy[:, 0].min() > x1 or xy[:, 1].max() < y0 or xy[:, 1].min() > y1:
            continue
        ax.plot(xy[:, 0], xy[:, 1], color="0.6", lw=1.0, ls="-" if v["conv"] else ":", zorder=2)
    ax.plot(ring[:, 0], ring[:, 1], ls="--", color="tab:green", lw=1.3, zorder=3, label="캠퍼스 폴리곤")
    for rid, P in road_pts.items():
        ax.plot(P[:, 0], P[:, 1], color="black", lw=1.8, zorder=4)
    for f in found:
        col = COLORS[f["k"]]
        for wid in f["cont"][:4]:
            xy = np.array([enu[n] for n in winfo[wid]["nds"]])
            ax.plot(xy[:, 0], xy[:, 1], color=col, lw=3.5, alpha=0.85, zorder=5, solid_capstyle="round")
            m = xy[len(xy) // 2]
            if x0 < m[0] < x1 and y0 < m[1] < y1:
                ax.annotate(f"{wid[-5:]}", m, fontsize=8, color=col, zorder=8, xytext=(4, 4), textcoords="offset points")
        e = f["e"]
        ax.scatter(*e["pt"], s=420, color="white", edgecolors=col, linewidths=3, zorder=9)
        ax.text(e["pt"][0], e["pt"][1], str(f["k"] + 1), ha="center", va="center", fontsize=13, fontweight="bold", color=col, zorder=10)
    ax.scatter(*stop, marker="*", s=500, color="gold", edgecolors="k", zorder=10, label="중문 정류장(OSM)")
    ax.set_xlim(x0, x1); ax.set_ylim(y0, y1); ax.set_aspect("equal"); ax.grid(True, lw=0.3, alpha=0.5)
    ax.set_xlabel("동쪽 (m)"); ax.set_ylabel("북쪽 (m)")
    ax.plot([x1 - 220, x1 - 20], [y0 + 25] * 2, color="k", lw=3); ax.text(x1 - 220, y0 + 35, "200 m", fontsize=10)
    ax.annotate("N", (x0 + 35, y1 - 40), (x0 + 35, y1 - 100), arrowprops=dict(arrowstyle="-|>", lw=2), ha="center", fontsize=14)
    ax.set_title("13 중문 직전 878 road 끝점(번호=중문 정류장 가까운 순) / 색선=끝점에서 이어지는 OSM way\n"
                 "검은 실선=878 road, 회색 실선=변환된 way, 회색 점선=미변환 way, 녹색 점선=캠퍼스 폴리곤 (재변환 보류, 채택 전)", fontsize=11)
    ax.legend(loc="lower left", fontsize=9)
    fig.savefig(OUT_PNG, bbox_inches="tight")
    print("그림:", OUT_PNG)


if __name__ == "__main__":
    main()
