#!/usr/bin/env python3
# 10/23(s66_route_check.py 사본): v3.1(28a57711) 과 v32a·v32b 에서 30쌍 Lanelet2 최단 경로(레인렛 id 열)가 같은지(서버 없음)
# s55_l2_cost.py 와 같은 로더·규칙(lanelet2 1.2.3, Germany Vehicle, local_x/local_y). 출발·도착 레인렛은 l1_meta.json(28a57711 빌드)
# 사용: ~/lanelet2_trial_venv/bin/python map/scripts/s66_route_check.py
import json
from pathlib import Path

import lanelet2
from lanelet2.io import Origin
from lanelet2.projection import UtmProjector

C = Path(__file__).resolve().parents[2]
META = json.loads((C / "map/build_s57_regen/v31/l1_meta.json").read_text())
PAIRS = json.loads((C / "map/build_v31_s54/pairs30.json").read_text())
MAPS = {"v31": C / "autoware_map/cbnu_campus_v31/lanelet2_map.osm", "v32a": C / "map/maps/v32a/lanelet2_map.osm", "v32b": C / "map/maps/v32b/lanelet2_map.osm"}


def load(p):
    m, err = lanelet2.io.loadRobust(str(p), UtmProjector(Origin(0, 0)))
    for q in m.pointLayer:
        q.x, q.y, q.z = float(q.attributes["local_x"]), float(q.attributes["local_y"]), 0.0
    rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany, lanelet2.traffic_rules.Participants.Vehicle)
    return m, lanelet2.routing.RoutingGraph(m, rules), err


res = {}
for k, p in MAPS.items():
    m, g, err = load(p)
    sub = {}
    for ll in m.laneletLayer:
        sub[ll.attributes["subtype"]] = sub.get(ll.attributes["subtype"], 0) + 1
    regs = {}
    for r in m.regulatoryElementLayer:
        s = r.attributes["subtype"] if "subtype" in r.attributes else "?"
        regs[s] = regs.get(s, 0) + 1
    print(f"{k}: 오류 {len(err)}, 레인렛 {dict(sorted(sub.items()))}, 규제 요소 {regs}, 경로 그래프 오류 {len(g.checkValidity())}")
    for e in err[:5]:
        print("  ", e)
    out = {}
    for name, pr in PAIRS.items():
        a = m.laneletLayer[META["lanelet"][f"{pr['roads'][0]},0,-1"]]
        b = m.laneletLayer[META["lanelet_last"][f"{pr['roads'][-1]},0,-1"]]
        sp = g.shortestPath(a, b)
        out[name] = [ll.id for ll in sp] if sp else None
    res[k] = out
for k in ("v32a", "v32b"):
    same = sum(1 for n in PAIRS if res["v31"][n] is not None and res["v31"][n] == res[k][n])
    print(f"30쌍 레인렛 id 열 v31 == {k} {same}/30")
road_ok = 0
m31, _, _ = load(MAPS["v31"])
for n, pr in PAIRS.items():
    rs = []
    for i in res["v31"][n] or []:
        r = int(m31.laneletLayer[i].attributes["xodr_road"])
        if not rs or rs[-1] != r:
            rs.append(r)
    road_ok += rs == pr["roads"]
print(f"v3.1 경로 road 열 == pairs30 roads {road_ok}/30")
for n in PAIRS:
    for k in ("v32a", "v32b"):
        if res["v31"][n] != res[k][n]:
            print("다름", k, n)
