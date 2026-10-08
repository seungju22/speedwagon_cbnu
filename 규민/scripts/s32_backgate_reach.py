#!/usr/bin/env python3
# 맵세션32 Phase 0-5: 후문 종점 후보가 v2 에서 정문(r1278 lane -1)으로부터 도달 가능한가 (읽기전용, 서버 없음)
# 후보(좌표 출처): 추정 게이트·고리 접점·경계 횡단점 = s28_gate_route.log / 고리 위 경계 최근접 노드 = s23_scope_final.md
#   (way452870648 노드 중 경계 횡단점 최근접, 38.2m 로 기록됨 -> 다시 계산해 대조)
# 도달 = s19_scope_common.successors 그래프(s23_connectivity 와 같은 정의)에서 정문 road 로부터 BFS.
# 후보가 도달 불가면 도달 가능한 road 의 lane -1 중심선 위 최근접 점(0.5m 표본)을 찾는다.
# 사용: .venv-carla/bin/python s32_backgate_reach.py <v2 xodr> <정문 road id>
import math
import sys
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from s19_scope_common import RAW_OSM, load_osm, ll_to_carla, load_map, road_lengths, build_graph, \
    nearest_lane, dijkstra, path_to  # noqa: E402

BOUNDARY = (36.624701, 127.463656)
CANDS = [("추정 게이트", 36.624844, 127.462860), ("고리 접점", 36.624909, 127.462514),
         ("경계 횡단점", 36.624701, 127.463656)]
LOOP_WAY = "452870648"


def xy_to_ll_dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def main():
    xodr, origin = Path(sys.argv[1]), int(sys.argv[2])
    cmap, lengths = load_map(xodr), road_lengths(xodr)
    g = build_graph(cmap, lengths)
    seen, q = {origin}, deque([origin])
    while q:
        u = q.popleft()
        for v in g.get(u, ()):
            if v not in seen:
                seen.add(v)
                q.append(v)
    print(f"맵 {xodr.name}: road {len(lengths)}, 정문 r{origin} 도달 {len(seen)}, 불가 {sorted(set(lengths) - seen)}")
    nodes, ways = load_osm(RAW_OSM)
    bxy = ll_to_carla(*BOUNDARY)
    if LOOP_WAY in ways:
        best = min(ways[LOOP_WAY]["nds"], key=lambda n: xy_to_ll_dist(ll_to_carla(*nodes[n]), bxy))
        d = xy_to_ll_dist(ll_to_carla(*nodes[best]), bxy)
        CANDS.append((f"고리 위 경계 최근접 node{best}(경계 {d:.1f}m)", *nodes[best]))
    else:
        print(f"way{LOOP_WAY} 원본 OSM 에 없음")
    dist, prev = dijkstra(g, lengths, origin)
    # 도달 가능 road 의 lane -1 중심선 표본
    samples = []
    for rid in seen:
        L = lengths[rid][0]
        s = 0.0
        while s <= L:
            wp = cmap.get_waypoint_xodr(rid, -1, min(s, max(L - 0.01, 0.0)))
            if wp is not None:
                loc = wp.transform.location
                samples.append((rid, s, loc.x, loc.y, lengths[rid][1]))
            s += 0.5
    for name, lat, lon in CANDS:
        x, y = ll_to_carla(lat, lon)
        wp, d = nearest_lane(cmap, x, y)
        rid = wp.road_id if wp else None
        ok = rid in seen
        print(f"\n[{name}] {lat:.6f},{lon:.6f} carla ({x:.2f},{y:.2f})")
        print(f"  최근접 주행 차선 r{rid} lane{wp.lane_id if wp else None} s={wp.s if wp else 0:.1f} "
              f"junction={lengths[rid][1] if rid else None} 거리 {d:.2f}m -> 정문에서 {'도달 가능' if ok else '도달 불가'}")
        if ok:
            p = path_to(prev, origin, rid)
            run = dist[rid] - lengths[rid][0] + wp.s
            print(f"  정문 -> {len(p)} road, 약 {run:.0f}m (r{origin} 길이 포함, 마지막 road s={wp.s:.1f} 까지)")
        else:
            br = min(samples, key=lambda t: math.hypot(t[2] - x, t[3] - y))
            bd = math.hypot(br[2] - x, br[3] - y)
            p = path_to(prev, origin, br[0])
            run = dist[br[0]] - lengths[br[0]][0] + br[1]
            print(f"  대체(도달 가능 최근접): r{br[0]} s={br[1]:.1f} junction={br[4]} ({br[2]:.2f},{br[3]:.2f}) "
                  f"원래 점까지 {bd:.2f}m, 정문 -> {len(p)} road 약 {run:.0f}m")


if __name__ == "__main__":
    main()
