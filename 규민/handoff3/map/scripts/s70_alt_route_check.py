#!/usr/bin/env python3
# 10/25 3단계: 대체 정차점 경로 검사(s66_route_check.py 와 같은 로더·규칙, lanelet2 1.2.3 Germany Vehicle). 서버 없음
# 지도 둘: v3.1(28a57711, 팀 지도) 와 v32a. 승강장마다
#   1) 들어오는 30쌍: 출발 레인렛 -> 대체 정차점 레인렛 최단 경로가 원래 경로(-> 원래 정차점 레인렛)의 앞부분과 같은지(같은 길로 와서 먼저 섬)
#   2) 원래 정차점 -> 대체 정차점: 대체가 뒤에 있으므로 한 바퀴 돌아오는 길. 같은 레인렛이면 다음 레인렛들에서 다시 찾음
#   3) 대체 정차점 -> 원래 정차점(앞으로 조금 감)
#   경로 길이 = 레인렛 중심선 길이 합(출발·도착 레인렛 전체 포함, 대략값)
# 입력: map/docs/logs/s70_alt_stops.json(s70_alt_stops.py). 출력: map/docs/logs/s70_alt_route_check.json·.log
# 사용: ~/lanelet2_trial_venv/bin/python map/scripts/s70_alt_route_check.py
import json
import math
from pathlib import Path

import lanelet2
from lanelet2.core import BasicPoint2d
from lanelet2.io import Origin
from lanelet2.projection import UtmProjector

C = Path(__file__).resolve().parents[2]
META = json.loads((C / "map/build_s57_regen/v31/l1_meta.json").read_text())
PAIRS = json.loads((C / "map/build_v31_s54/pairs30.json").read_text())
ALT = json.loads((C / "map/docs/logs/s70_alt_stops.json").read_text())
MAPS = {"v31": C / "autoware_map/cbnu_campus_v31/lanelet2_map.osm", "v32a": C / "map/maps/v32a/lanelet2_map.osm"}
OUT = C / "map/docs/logs/s70_alt_route_check.json"


def load(p):
    m, err = lanelet2.io.loadRobust(str(p), UtmProjector(Origin(0, 0)))
    for q in m.pointLayer:
        q.x, q.y, q.z = float(q.attributes["local_x"]), float(q.attributes["local_y"]), 0.0
    rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany, lanelet2.traffic_rules.Participants.Vehicle)
    return m, lanelet2.routing.RoutingGraph(m, rules), err


def heading(ll, x, y):
    cl = ll.centerline
    best = None
    for a, b in zip(cl, list(cl)[1:]):
        mx, my = (a.x + b.x) / 2, (a.y + b.y) / 2
        d = math.hypot(mx - x, my - y)
        if best is None or d < best[0]:
            best = (d, math.degrees(math.atan2(b.y - a.y, b.x - a.x)))
    return best[1]


def find_ll(m, x, y, yaw):
    p = BasicPoint2d(x, y)
    c = [ll for ll in m.laneletLayer.search(lanelet2.core.BoundingBox2d(BasicPoint2d(x - 1, y - 1), BasicPoint2d(x + 1, y + 1)))
         if lanelet2.geometry.inside(ll, p) and abs((heading(ll, x, y) - yaw + 180) % 360 - 180) < 60]
    return c[0] if c else None


def length(path):
    return sum(lanelet2.geometry.length2d(ll) for ll in path)


def loop_path(g, a, b):
    """a 에서 출발해 b 로(같은 레인렛이면 한 바퀴). 반환 레인렛 열"""
    if a.id != b.id:
        sp = g.shortestPath(a, b)
        return [ll for ll in sp] if sp else None
    best = None
    for f in g.following(a):
        sp = g.shortestPath(f, b)
        if sp:
            cand = [a] + [ll for ll in sp]
            if best is None or length(cand) < length(best):
                best = cand
    return best


res, log = {}, []


def P(*a):
    s = " ".join(str(x) for x in a)
    print(s)
    log.append(s)


for k, mp in MAPS.items():
    m, g, err = load(mp)
    P(f"{k}: 읽기 오류 {len(err)}")
    out = {}
    for pl in ALT["platforms"]:
        if pl["status"] != "있음":
            continue
        o, a = pl["original_base_link_goal"], pl["alt"]["base_link_goal"]
        lo, la = find_ll(m, o["x"], o["y"], o["yaw_deg"]), find_ll(m, a["x"], a["y"], a["yaw_deg"])
        r = dict(orig_ll=lo.id if lo else None, alt_ll=la.id if la else None, same_ll=bool(lo and la and lo.id == la.id))
        inc = {}
        for n in pl["incoming_pairs"]:
            pr = PAIRS[n]
            s = m.laneletLayer[META["lanelet"][f"{pr['roads'][0]},0,-1"]]
            sp_o, sp_a = g.shortestPath(s, lo), g.shortestPath(s, la)
            po = [ll.id for ll in sp_o] if sp_o else None
            pa = [ll.id for ll in sp_a] if sp_a else None
            inc[n] = dict(found=pa is not None, prefix_of_original=bool(pa and po and po[:len(pa)] == pa),
                          n_alt=len(pa) if pa else None, n_orig=len(po) if po else None)
        r["incoming"] = inc
        oa = loop_path(g, lo, la)
        ao = loop_path(g, la, lo) if not r["same_ll"] else [la]
        r["orig_to_alt"] = dict(found=oa is not None, n=len(oa) if oa else None, length_m=round(length(oa), 1) if oa else None)
        r["alt_to_orig"] = dict(found=ao is not None, n=len(ao) if ao else None, length_m=round(length(ao), 1) if ao else None)
        out[pl["id"]] = r
        ok_inc = sum(1 for v in inc.values() if v["found"] and v["prefix_of_original"])
        P(f"  {pl['id']}: 레인렛 원래 {r['orig_ll']} 대체 {r['alt_ll']}(같음 {r['same_ll']}) 들어오는 쌍 {ok_inc}/{len(inc)} 앞부분 같음, "
          f"원래->대체 {r['orig_to_alt']}, 대체->원래 {r['alt_to_orig']}")
    res[k] = out
OUT.write_text(json.dumps(res, ensure_ascii=False, indent=1))
(C / "map/docs/logs/s70_alt_route_check.log").write_text("\n".join(log) + "\n")
P(f"-> {OUT}")
