#!/usr/bin/env python3
# 맵세션29 Phase 3 보조: Lanelet2 routing graph 에서 뒤 lanelet 이 없는 차량 lanelet 의 원인 분류 (읽기전용, 시험 venv)
# 끝점(왼·오른 경계 마지막 점) 근처에 다른 lanelet 시작점이 있는데 점 id 가 달라 연결이 안 된 것인지 본다
# 또 CommonRoad 중간 파일(.cr.xml)에서는 같은 lanelet 이 successor 를 가졌는지 (CR id 는 Lanelet2 id 와 달라 위치로 맞춘다)
# 사용: python s29_lanelet2_gaps.py <osm>
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
lls = [ll for ll in m.laneletLayer if rules.canPass(ll)]
starts = [(ll.id, ll.leftBound[0], ll.rightBound[0]) for ll in lls]
dead = [ll for ll in lls if not g.following(ll)]
nodead = [ll for ll in lls if not g.previous(ll)]
print(f"차량 lanelet {len(lls)}, 뒤 없음 {len(dead)}, 앞 없음 {len(nodead)}")
bins = collections.Counter()
same_xy_diff_id = 0
examples = []
for ll in dead:
    le, re_ = ll.leftBound[-1], ll.rightBound[-1]
    best = None
    for sid, ls, rs in starts:
        if sid == ll.id:
            continue
        d = max(math.hypot(le.x - ls.x, le.y - ls.y), math.hypot(re_.x - rs.x, re_.y - rs.y))
        if best is None or d < best[0]:
            best = (d, sid, ls.id == le.id, rs.id == re_.id)
    d = best[0]
    b = "<0.001m" if d < 0.001 else "<0.05m" if d < 0.05 else "<0.5m" if d < 0.5 else "<3m" if d < 3 else ">=3m"
    bins[b] += 1
    if d < 0.05 and not (best[2] and best[3]):
        same_xy_diff_id += 1
        if len(examples) < 5:
            examples.append((ll.id, best[1], round(d, 4), best[2], best[3]))
print(f"뒤 없는 lanelet 의 끝 -> 가장 가까운 다른 lanelet 시작(두 경계 끝점 중 큰 거리) 분포 {dict(bins)}")
print(f"0.05m 안인데 점 id 가 달라 연결 안 됨 {same_xy_diff_id}, 예(lanelet, 후보, 거리, 왼쪽 id 같음, 오른쪽 id 같음) {examples}")
