#!/usr/bin/env python3
# 맵세션29 Phase 3-4: 반쪽 도로(단방향 2개) 쌍이 Lanelet2 에서 어떻게 표현되는가 (읽기전용, 시험 venv)
# 경계 linestring 공유 관계를 센다: 차량 lanelet 의 왼쪽 경계가 다른 차량 lanelet 의 왼쪽 경계와 같은 선(뒤집힘 포함)이면 맞은편 쌍으로 묶인 것
# 같은 자리(중간점 0.05m 안)에 따로 그려진 왼쪽 경계가 있으면 "쌍이 묶이지 않음"
# routing graph 의 adjacentLeft / lefts(차선 변경 가능 이웃) 수도 센다
# 사용: python s29_halfroad.py <osm>
import collections
import math
import sys

import lanelet2
from lanelet2.io import Origin
from lanelet2.projection import UtmProjector

proj = UtmProjector(Origin(36.627298, 127.456394))
m, _ = lanelet2.io.loadRobust(sys.argv[1], proj)
rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany, lanelet2.traffic_rules.Participants.Vehicle)
g = lanelet2.routing.RoutingGraph(m, rules)
veh = [ll for ll in m.laneletLayer if rules.canPass(ll)]
walk = [ll for ll in m.laneletLayer if not rules.canPass(ll)]
use = collections.defaultdict(list)
for ll in m.laneletLayer:
    use[ll.leftBound.id].append((ll.id, "L", ll.attributes["subtype"]))
    use[ll.rightBound.id].append((ll.id, "R", ll.attributes["subtype"]))
shared_left_left = sum(1 for v in use.values() if sum(1 for x in v if x[1] == "L" and x[2] == "road") >= 2)
road_walk = sum(1 for v in use.values() if {x[2] for x in v} == {"road", "walkway"})
print(f"차량 lanelet {len(veh)}, 보행 lanelet {len(walk)}")
print(f"차량 lanelet 두 개가 왼쪽 경계를 같은 선으로 공유(맞은편 쌍) {shared_left_left}")
print(f"차량-보행 lanelet 이 경계 선 공유 {road_walk}")
mid = {}
for ll in veh:
    lb = ll.leftBound
    p = lb[len(lb) // 2]
    mid[ll.id] = (p.x, p.y, lb.id)
grid = collections.defaultdict(list)
for k, (x, y, lid) in mid.items():
    grid[(int(x // 5), int(y // 5))].append(k)
twin_sep = 0
for k, (x, y, lid) in mid.items():
    gx, gy = int(x // 5), int(y // 5)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for o in grid[(gx + dx, gy + dy)]:
                if o == k:
                    continue
                ll_o = m.laneletLayer[o]
                if ll_o.leftBound.id == lid:
                    continue
                d = min(math.hypot(x - pt.x, y - pt.y) for pt in ll_o.leftBound)
                if d < 0.05:
                    twin_sep += 1
print(f"왼쪽 경계가 다른 차량 lanelet 왼쪽 경계 위 0.05m 안(같은 자리, 다른 선) 인 lanelet 수(쌍이면 2씩) {twin_sep}")
print(f"routing graph 왼쪽 이웃(lefts, 같은 방향 차선 변경) 있는 차량 lanelet {sum(1 for ll in veh if g.lefts(ll))},"
      f" adjacentLeft(통행 불가 이웃 포함) {sum(1 for ll in veh if g.adjacentLeft(ll))}")
