#!/usr/bin/env python3
# 11/04: s75_check_v33c.py 사본. 지도 v3.3d(후문 앞 섬 둘레 유턴), 순환 파일은 세 번째 인자(기본 v33d_r1). 모서리 유턴 나눔(r1415·r1409)은 v3.3c 그대로라 v33c_changes 를 씀
# (s75 머리말) s74_check_v33b.py 사본. v3.3c, 순환 v33c_r1, 유턴 52803
#   사용: ~/lanelet2_trial_venv/bin/python map/scripts/s75_check_v33c.py map/maps/v33c/lanelet2_map.osm integration/logs/s75/v33c_route_check.json
# (s74 머리말) s73_check_v33.py 사본(순환 노선 파일만 v3.3b 1판으로)
# 11/02: v3.3a Lanelet2 확인(s66_route_check.py 와 같은 로더·규칙, lanelet2 1.2.3 Germany Vehicle). 서버 없음
#   읽기 오류, 레인렛 수, 30쌍 경로(v3.1 과 레인렛 id 열이 같은지 / 바뀌었는지 / 없는지, 출발·도착 레인렛이 빠진 쌍),
#   북문 길 끝 유턴: r1415 마지막 레인렛 -> r1409 첫 레인렛 경로가 유턴 레인렛 하나로 이어지는지
# 사용: ~/lanelet2_trial_venv/bin/python map/scripts/s73_check_v33.py [지도 osm] [출력 json]
import json, sys
from pathlib import Path
import lanelet2
from lanelet2.io import Origin
from lanelet2.projection import UtmProjector

C = Path(__file__).resolve().parents[2]
OSM = Path(sys.argv[1]) if len(sys.argv) > 1 else C / "map/maps/v33a/lanelet2_map.osm"
OUTJ = Path(sys.argv[2]) if len(sys.argv) > 2 else C / "integration/logs/s73/v33a_route_check.json"
META = json.loads((C / "map/build_s57_regen/build/l1_meta.json").read_text())
import copy
META0 = copy.deepcopy(META)
CH = json.loads((C / "map/maps/v33c/v33c_changes.json").read_text())
META["lanelet_last"]["1415,0,-1"] = int(CH["split"]["r1415"]["back"]); META["lanelet"]["1409,0,-1"] = int(CH["split"]["r1409"]["front"])
PAIRS = json.loads((C / "map/build_v31_s54/pairs30.json").read_text())
V31 = C / "autoware_map/cbnu_campus_v31/lanelet2_map.osm"


def load(p):
    m, err = lanelet2.io.loadRobust(str(p), UtmProjector(Origin(0, 0)))
    for q in m.pointLayer:
        q.x, q.y, q.z = float(q.attributes["local_x"]), float(q.attributes["local_y"]), 0.0
    rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany, lanelet2.traffic_rules.Participants.Vehicle)
    return m, lanelet2.routing.RoutingGraph(m, rules), err


def roads(m, path):
    out = []
    for i in path:
        ll = m.laneletLayer[i]
        r = int(ll.attributes["xodr_road"]) if "xodr_road" in ll.attributes else -1
        if not out or out[-1] != r: out.append(r)
    return out


m3, g3, e3 = load(V31); m, g, e = load(OSM)
res = {"osm": str(OSM), "errors": len(e), "lanelets": len(m.laneletLayer), "routing_errors": len(g.checkValidity()), "pairs": {}}
ids = {ll.id for ll in m.laneletLayer}
for name, pr in PAIRS.items():
    a = META["lanelet"].get(f"{pr['roads'][0]},0,-1"); b = META["lanelet_last"].get(f"{pr['roads'][-1]},0,-1")
    if a not in ids or b not in ids:
        res["pairs"][name] = dict(status="승강장 레인렛 빠짐"); continue
    a3 = META0['lanelet'].get(f"{pr['roads'][0]},0,-1"); b3 = META0['lanelet_last'].get(f"{pr['roads'][-1]},0,-1")
    sp3 = g3.shortestPath(m3.laneletLayer[a3], m3.laneletLayer[b3]); sp = g.shortestPath(m.laneletLayer[a], m.laneletLayer[b])
    p3 = [ll.id for ll in sp3] if sp3 else None; p = [ll.id for ll in sp] if sp else None
    if p is None: st = "경로 없음"
    elif p == p3: st = "같음"
    else: st = "바뀜"
    L = lambda pp: round(sum(lanelet2.geometry.length2d(m.laneletLayer[i]) for i in pp), 1) if pp else None
    L3 = lambda pp: round(sum(lanelet2.geometry.length2d(m3.laneletLayer[i]) for i in pp), 1) if pp else None
    res["pairs"][name] = dict(status=st, len=L(p), len_v31=L3(p3), roads=roads(m, p) if p else None)
u = [ll for ll in m.laneletLayer if "s75_uturn" in ll.attributes]
a = m.laneletLayer[int(CH["uturn"]["from_lanelet"])]; b = m.laneletLayer[int(CH["uturn"]["to_lanelet"])]
sp = g.shortestPath(a, b)
res["uturn"] = dict(n=len(u), ids=[x.id for x in u], following_of_1415=[x.id for x in g.following(a)], path=[ll.id for ll in sp] if sp else None,
                    uturn_len=round(lanelet2.geometry.length2d(u[0]), 1) if u else None)
from collections import Counter
res["count"] = dict(Counter(v["status"] for v in res["pairs"].values()))
OUTJ.parent.mkdir(parents=True, exist_ok=True)
OUTJ.write_text(json.dumps(res, ensure_ascii=False, indent=1))
print("오류", res["errors"], "레인렛", res["lanelets"], "경로 그래프 오류", res["routing_errors"], res["count"])
print("유턴", res["uturn"])
for k, v in res["pairs"].items():
    if v["status"] != "같음": print(" ", k, v["status"], v.get("len_v31"), "->", v.get("len"))


# ---- 11/02: 중문 쌍을 개성재 자리(r1316)로 바꾼 30쌍, 순환 노선 구간 ----
SUB = {1320: 1316, 1191: 1316}
res["pairs_v33a"] = {}
for name, pr in PAIRS.items():
    r0, r1 = SUB.get(pr["roads"][0], pr["roads"][0]), SUB.get(pr["roads"][-1], pr["roads"][-1])
    a = META["lanelet"].get(f"{r0},0,-1"); b = META["lanelet_last"].get(f"{r1},0,-1")
    if a not in ids or b not in ids: res["pairs_v33a"][name] = dict(status="레인렛 없음"); continue
    sp = g.shortestPath(m.laneletLayer[a], m.laneletLayer[b]) if a != b else None
    if a == b:   # 같은 레인렛: 한 바퀴(개성재 -> 개성재 는 없음)
        res["pairs_v33a"][name] = dict(status="같은 레인렛"); continue
    res["pairs_v33a"][name] = dict(status="있음" if sp else "경로 없음", ids=[ll.id for ll in sp] if sp else None, roads=[r for r in roads(m, [ll.id for ll in sp]) if r != -1] if sp else None,
                                   len=round(sum(lanelet2.geometry.length2d(ll) for ll in sp), 1) if sp else None)
import subprocess
RT = json.loads(subprocess.check_output(["/usr/bin/python3", "-c", "import sys,json,yaml;print(json.dumps(yaml.safe_load(open(sys.argv[1]))))",
                                          sys.argv[3] if len(sys.argv) > 3 else str(C / "map/data/routes/circulation_v33d_r1.yaml")]))
res["circulation"] = []
for sg in RT["segments"]:
    rs = sg["roads"]
    a = META["lanelet"].get(f"{rs[0]},0,-1"); b = META["lanelet_last"].get(f"{rs[-1]},0,-1")
    if sg.get("via_uturn") or sg["frm"] == "lib_north": a = 37801   # 도서관 북문 승강장은 r1415 앞 조각
    if sg["to"] == "lib_north": b = 37801
    if a == b and len(rs) == 1:   # 같은 레인렛 안에서 앞으로(예: 북문 -> 사범대 앞)
        res["circulation"].append(dict(frm=sg["frm"], to=sg["to"], same=True, got=rs, want=rs, note="같은 레인렛 안 앞으로")); continue
    if a == b:
        # 같은 레인렛에서 출발·도착(한 바퀴): 다음 레인렛들에서 찾음
        best = None
        for f in g.following(m.laneletLayer[a]):
            sp = g.shortestPath(f, m.laneletLayer[b])
            if sp and (best is None or len(sp) < len(best)): best = [m.laneletLayer[a]] + list(sp)
        sp = best
    else:
        sp = g.shortestPath(m.laneletLayer[a], m.laneletLayer[b])
    got = [r for r in roads(m, [ll.id for ll in sp]) if r != -1] if sp else None
    want = [r for r in rs]
    fix = None
    if got != want and sp:
        # 고정용 경유점: 원하는 열에만 있는 첫 road 의 레인렛 가운데. shortestPathWithVia 로 다시 봄
        for r in want:
            if r in (got or []) or f"{r},0,-1" not in META["lanelet"]: continue
            vl = m.laneletLayer[META["lanelet"][f"{r},0,-1"]]
            sp2 = g.shortestPathWithVia(m.laneletLayer[a], [vl], m.laneletLayer[b])
            got2 = [x for x in roads(m, [ll.id for ll in sp2]) if x != -1] if sp2 else None
            cl = vl.centerline; mid = cl[len(cl) // 2]; nx = cl[min(len(cl) - 1, len(cl) // 2 + 1)]
            import math as _m
            fix = dict(road=r, lanelet=vl.id, x=round(mid.x, 3), y=round(mid.y, 3), yaw_deg=round(_m.degrees(_m.atan2(nx.y - mid.y, nx.x - mid.x)), 2), same_after=(got2 == want))
            break
    res["circulation"].append(dict(frm=sg["frm"], to=sg["to"], same=(got == want), got=got, want=want, force_via=fix, ids=[ll.id for ll in sp] if sp else None))
from collections import Counter as _C
print("30쌍(개성재 자리)", dict(_C(v["status"] for v in res["pairs_v33a"].values())))
print("순환 구간 road 열 같음", sum(x["same"] for x in res["circulation"]), "/", len(res["circulation"]))
for x in res["circulation"]:
    if not x["same"]: print("  다름", x["frm"], x["to"], "고정 경유점", x["force_via"])
OUTJ.write_text(json.dumps(res, ensure_ascii=False, indent=1))

# ---- 11/03: 도서관 북문 -> 개성재 구간 길이(v3.3b 와 레인렛 길이 차이로 고침) ----
mb, gb, _ = load(C / "map/maps/v33b/lanelet2_map.osm")
mid = META["lanelet_last"]["1316,0,-1"]
spb = gb.shortestPath(mb.laneletLayer[37801], mb.laneletLayer[mid]); spc = g.shortestPath(m.laneletLayer[37801], m.laneletLayer[mid])
Lb = sum(lanelet2.geometry.length2d(x) for x in spb); Lc = sum(lanelet2.geometry.length2d(x) for x in spc)
res["seg_lib_mid"] = dict(v33b_ids=[x.id for x in spb], v33c_ids=[x.id for x in spc], Lb=round(Lb, 1), Lc=round(Lc, 1), new_len=round(127.0 + 483.9 + Lc - Lb, 1))
print("도서관 북문 -> 개성재", res["seg_lib_mid"]["new_len"], "m (v33b 북문->사범대->유턴->개성재 610.9)", res["seg_lib_mid"]["v33c_ids"][:5])
OUTJ.write_text(json.dumps(res, ensure_ascii=False, indent=1))

# ---- 11/04: 후문 섬 둘레 유턴(v33d_changes) ----
CD = json.loads((C / "map/maps/v33d/v33d_changes.json").read_text())
a = m.laneletLayer[int(CD["uturn"]["from_lanelet"])]; b = m.laneletLayer[int(CD["uturn"]["to_lanelet"])]; sp = g.shortestPath(a, b)
res["uturn_back"] = dict(id=int(CD["uturn"]["relation"]), following=[x.id for x in g.following(a)], path=[ll.id for ll in sp] if sp else None,
                         len=round(lanelet2.geometry.length2d(m.laneletLayer[int(CD["uturn"]["relation"])]), 1))
print("후문 유턴", res["uturn_back"])
OUTJ.write_text(json.dumps(res, ensure_ascii=False, indent=1))
