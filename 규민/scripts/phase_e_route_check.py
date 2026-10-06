#!/usr/bin/env python3
# 맵세션7 Phase E: 중문 -> 중앙도서관 종점(road1278 s=100.38) 경로 존재 확인.
# 읽기전용(xodr/osm 미수정), CARLA 불필요. 방향 검사 포함(driving lane 부호 기준).
# 검증: 같은 그래프로 정문(road1247 s=0) -> 종점이 확정 15road/781.22m 와 일치해야 함.
import heapq
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_geometry import parse_roads, parse_junctions, road_polyline
from geo_calibrate import (OSM_RAW_PATH, load_osm_nodes, junction_xodr_locations,
                           latlon_to_enu, fit_similarity)

BASE = Path(__file__).resolve().parent.parent
XODR = BASE / "maps" / "cbnu_internal_only_localtm_tags73.xodr"
TARGET_ROAD, TARGET_S = "1278", 100.38
STOPS = {"정문": "6403450088", "중문": "12324240640",
         "중문쪽 내부망 접점(OSM node 4402717708)": (36.632993, 127.457385)}


def lane_dirs(root):
    d = {}
    for road in root.iter("road"):
        ids = set()
        for lane in road.iter("lane"):
            if lane.get("type") == "driving":
                ids.add(int(lane.get("id")))
        d[road.get("id")] = (any(i < 0 for i in ids), any(i > 0 for i in ids))
    return d


def build(roads, juncs, ld):
    # 상태 = (road, dir), dir +1: start->end(우측차선 id<0), -1: end->start(id>0)
    inc = {}
    for jid, j in juncs.items():
        for c in j["connections"]:
            inc.setdefault((jid, c["incomingRoad"]), []).append(c)
    ok = lambda r, d: ld.get(r, (False, False))[0 if d == 1 else 1]
    adj = {}
    for rid, r in roads.items():
        for d in (1, -1):
            if not ok(rid, d):
                continue
            end = r["succ"] if d == 1 else r["pred"]
            nxt = []
            if end:
                et, eid = end.get("elementType"), end.get("elementId")
                if et == "road":
                    cp = end.get("contactPoint")
                    nd = 1 if cp == "start" else -1
                    if ok(eid, nd):
                        nxt.append((eid, nd))
                elif et == "junction":
                    for c in inc.get((eid, rid), []):
                        nd = 1 if c["contactPoint"] == "start" else -1
                        if ok(c["connectingRoad"], nd):
                            nxt.append((c["connectingRoad"], nd))
            adj[(rid, d)] = nxt
    return adj


def dijkstra(adj, roads, src, s0):
    # 비용 = src 도로에서 s0 이후 남은 길이 + 이후 도로 전체 길이
    rid, d = src
    L = roads[rid]["length"]
    c0 = (L - s0) if d == 1 else s0
    dist = {src: c0}
    prev = {}
    pq = [(c0, src)]
    while pq:
        c, u = heapq.heappop(pq)
        if c > dist[u]:
            continue
        for v in adj.get(u, []):
            nc = c + roads[v[0]]["length"]
            if nc < dist.get(v, 1e18):
                dist[v] = nc
                prev[v] = u
                heapq.heappush(pq, (nc, v))
    return dist, prev


def path_to(prev, t):
    p = [t]
    while p[-1] in prev:
        p.append(prev[p[-1]])
    return p[::-1]


def main():
    root = ET.parse(XODR).getroot()
    roads, juncs, ld = parse_roads(root), parse_junctions(root), lane_dirs(root)
    print(f"road {len(roads)}개, 양방향 도로(driving 양쪽 부호): "
          f"{sum(1 for v in ld.values() if v[0] and v[1])}개, "
          f"역방향만: {sum(1 for v in ld.values() if v[1] and not v[0])}개")
    adj = build(roads, juncs, ld)

    osm = load_osm_nodes(OSM_RAW_PATH)
    j_xy = junction_xodr_locations(roads, juncs)
    pairs = [(n, *j_xy[j], *osm[juncs[j]["name"]]) for j in juncs
             if juncs[j]["name"] in osm and j in j_xy for n in [juncs[j]["name"]]]
    rl = sum(p[3] for p in pairs) / len(pairs)
    ro = sum(p[4] for p in pairs) / len(pairs)
    R, s, t, ang, _ = fit_similarity([(p[1], p[2]) for p in pairs],
                                     [latlon_to_enu(p[3], p[4], rl, ro) for p in pairs])
    print(f"보정점 {len(pairs)}, 회전 {ang:.2f}deg, 스케일 {s:.5f}")

    def to_xodr(lat, lon):
        e, n = latlon_to_enu(lat, lon, rl, ro)
        return tuple(np.linalg.inv(s * R) @ (np.array([e, n]) - t))

    # 일반 road 폴리라인(s 누적)
    polys = {}
    for rid, r in roads.items():
        if r["junction"] != "-1":
            continue
        pts = road_polyline(r, curve_samples=20)
        cum = [0.0]
        for a, b in zip(pts, pts[1:]):
            cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        polys[rid] = (pts, cum)

    def nearest(x, y):
        best = []
        for rid, (pts, cum) in polys.items():
            k = min(range(len(pts)), key=lambda i: math.hypot(pts[i][0] - x, pts[i][1] - y))
            dd = math.hypot(pts[k][0] - x, pts[k][1] - y)
            sc = cum[k] * roads[rid]["length"] / cum[-1] if cum[-1] else 0
            best.append((dd, rid, sc))
        return sorted(best)[:6]

    tgt_states = [(TARGET_ROAD, 1)]

    # 역방향 도달 가능 집합(목표 road 에 갈 수 있는 상태)
    radj = {}
    for u, vs in adj.items():
        for v in vs:
            radj.setdefault(v, []).append(u)
    can = set(tgt_states)
    st = list(tgt_states)
    while st:
        v = st.pop()
        for u in radj.get(v, []):
            if u not in can:
                can.add(u)
                st.append(u)
    print(f"목표({TARGET_ROAD})로 갈 수 있는 상태 {len(can)}개 / 전체 {len(adj)}개")

    def cost_to_target(dist, prev, src):
        tg = (TARGET_ROAD, 1)
        if tg not in dist:
            return None
        # 목표 road 는 s=TARGET_S 까지만 주행
        if src[0] == TARGET_ROAD:
            return None
        return dist[tg] - roads[TARGET_ROAD]["length"] + TARGET_S

    for name, oid in STOPS.items():
        lat, lon = osm[oid] if isinstance(oid, str) else oid
        x, y = to_xodr(lat, lon)
        print(f"\n=== {name} ({oid if isinstance(oid, str) else '직접좌표'}, {lat:.6f},{lon:.6f}) -> xodr ({x:.1f},{y:.1f}) ===")
        near = nearest(x, y)
        for dd, rid, sc in near:
            ds = [d for d in (1, -1) if (rid, d) in adj]
            print(f" 근접 road{rid} 거리 {dd:.1f}m s~{sc:.1f}/{roads[rid]['length']:.1f}m "
                  f"방향 {ds} 목표도달 {[(rid,d) in can for d in ds]}")
        # 시작 후보: 근접 6개 road 의 각 방향
        done = False
        for dd, rid, sc in near:
            for d in (1, -1):
                if (rid, d) not in adj:
                    continue
                dist, prev = dijkstra(adj, roads, (rid, d), sc)
                c = cost_to_target(dist, prev, (rid, d))
                if c is not None:
                    seq = path_to(prev, (TARGET_ROAD, 1))
                    print(f" -> 경로 존재: 시작 road{rid} dir{d} (s~{sc:.1f}, 중문에서 {dd:.1f}m), "
                          f"road {len(seq)}개, {c:.1f}m")
                    print("    시퀀스:", " ".join(f"{a}{'+' if b==1 else '-'}" for a, b in seq))
                    done = True
                    break
            if done:
                break
        if not done:
            # 시작 후보에서 도달 가능한 집합의 크기와 도달 불가 원인
            rid = near[0][1]
            for d in (1, -1):
                if (rid, d) in adj:
                    dist, _ = dijkstra(adj, roads, (rid, d), near[0][2])
                    print(f" -> 경로 없음. road{rid} dir{d} 에서 도달 가능 상태 {len(dist)}개 "
                          f"(목표 포함 여부 {(TARGET_ROAD,1) in dist}), 막다른 지점 예: "
                          f"{[u for u in dist if not adj.get(u)][:5]}")
            # 목표로 갈 수 있는 road 중 이 지점에서 가장 가까운 것
            cand = []
            for rid2, (pts, cum) in polys.items():
                if any((rid2, d) in can for d in (1, -1)):
                    dd2 = min(math.hypot(px - x, py - y) for px, py in pts)
                    cand.append((dd2, rid2))
            cand.sort()
            print(" 목표 도달 가능 road 중 가장 가까운 것:", [(f"{a:.0f}m", b) for a, b in cand[:4]])


if __name__ == "__main__":
    main()
