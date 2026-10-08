#!/usr/bin/env python3
# 맵세션29 Phase 3 보조: 공식 Lanelet2 파이썬 라이브러리(lanelet2 1.2.3, PyPI)로 변환 결과를 읽어 본다 (읽기전용)
# Autoware 검증기(autoware_lanelet2_map_validator)는 호스트에 없다. 이것은 그 대신이 아니라 "Lanelet2 로 읽히는가" 만 본다
# 시험 venv 로 실행: ~/lanelet2_trial_venv/bin/python
# 1 load 오류 수 2 lanelet 수 3 차량 교통 규칙(germany vehicle) 기준 routing graph, checkValidity
# 4 정문 lanelet(r1247 lane -1 중간점 최근접)에서 도달 가능한 차량 lanelet 수
# 사용: python s29_lanelet2_load.py <osm> [<osm> ...]
import sys

import lanelet2
from lanelet2.core import BasicPoint2d, GPSPoint
from lanelet2.io import Origin
from lanelet2.projection import UtmProjector

LAT0, LON0 = 36.627298, 127.456394
GATE = (36.631961, 127.453056)       # 정문 r1247 lane -1 중간 실제 위경도(s29_eval.log D)


def main():
    proj = UtmProjector(Origin(LAT0, LON0))
    for path in sys.argv[1:]:
        print(f"## {path.rsplit('/', 1)[-1]}")
        m, errs = lanelet2.io.loadRobust(path, proj)
        print(f"load 오류 {len(errs)}" + (f" 예: {errs[:3]}" if errs else ""))
        lls = list(m.laneletLayer)
        sub = {}
        for ll in lls:
            s = ll.attributes["subtype"] if "subtype" in ll.attributes else "-"
            sub[s] = sub.get(s, 0) + 1
        print(f"lanelet {len(lls)} {sub}, point {len(m.pointLayer)}, linestring {len(m.lineStringLayer)}, regulatory {len(m.regulatoryElementLayer)}")
        rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany, lanelet2.traffic_rules.Participants.Vehicle)
        passable = [ll for ll in lls if rules.canPass(ll)]
        print(f"차량 통행 가능 lanelet {len(passable)}")
        g = lanelet2.routing.RoutingGraph(m, rules)
        bad = g.checkValidity()
        print(f"routing graph checkValidity 문제 {len(bad)}" + (f" 예: {list(bad)[:3]}" if bad else ""))
        p = proj.forward(GPSPoint(GATE[0], GATE[1], 0))
        start = min(passable, key=lambda ll: lanelet2.geometry.distance(lanelet2.geometry.to2D(ll), BasicPoint2d(p.x, p.y)))
        d = lanelet2.geometry.distance(lanelet2.geometry.to2D(start), BasicPoint2d(p.x, p.y))
        reach = g.reachableSet(start, 1e9, 0, True)
        print(f"정문 lanelet {start.id}(거리 {d:.2f}m)에서 도달 가능 {len(reach)}/{len(passable)} (차선 변경 포함)")
        reach2 = g.reachableSet(start, 1e9, 0, False)
        print(f"  차선 변경 없이 {len(reach2)}/{len(passable)}")
        zs = {round(pt.z, 3) for pt in m.pointLayer}
        print(f"point z 값 종류 {sorted(zs)[:5]}{' ...' if len(zs) > 5 else ''}")


if __name__ == "__main__":
    main()
