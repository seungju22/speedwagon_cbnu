#!/usr/bin/env python3
# 11/03(s73_m20.py 사본: v3.3b 순환 노선, 후문 유턴 9002) 11/02 5단계: v3.3a 차체·곡률 검사(s59_m20.py 의 centerline·body_check·curvature 그대로, 지도만 v3.3a). 서버 없음
#   유턴 레인렛(s73_uturn 태그, xodr_road 없음)은 가짜 road 9001 로 읽음
#   대상: 30쌍(중문 쌍은 개성재 자리 r1316 s=70, 경로 = s73_check_v33.py 의 Lanelet2 경로), 순환 노선 구간, 유턴(r1415 s=180 -> 유턴 -> r1409 s=30)
#   차: sample_vehicle(지금), 스타렉스(설계 차: 5.15 x 1.92, 축거 3.2, 오버행 0.877/1.073 가정, 뒷축 R 3.77), sprinter(CARLA 실측 5.92 x 1.99, 축거 3.662, R 은 모름 -> 3.77 로 둠)
# 출력 integration/logs/s73/m20_v33a.json
import json, math, subprocess, xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path
import s59_m20 as m20

C = Path(__file__).resolve().parents[2]
import os
OSM = Path(os.environ.get("S73_OSM", str(C / "map/maps/v33a/lanelet2_map.osm")))
OUT = C / "integration/logs/s73"
UT = 9001


def load(osm):
    r = ET.parse(osm).getroot()
    nodes = {}
    for n in r.iter("node"):
        t = {x.get("k"): x.get("v") for x in n.findall("tag")}
        nodes[n.get("id")] = (float(t["local_x"]), float(t["local_y"]))
    ways = {w.get("id"): [nd.get("ref") for nd in w.findall("nd")] for w in r.iter("way")}
    lls = defaultdict(list)
    for rel in r.iter("relation"):
        t = {x.get("k"): x.get("v") for x in rel.findall("tag")}
        if t.get("type") != "lanelet": continue
        mem = {m.get("role"): m.get("ref") for m in rel.findall("member")}
        L = [nodes[q] for q in ways[mem["left"]]]; R = [nodes[q] for q in ways[mem["right"]]]
        k = (UT + 1 if t.get("s73_uturn") == "back_gate_front" else UT) if "s73_uturn" in t else int(t["xodr_road"])
        lls[k].append((int(t.get("xodr_piece", 0)), L, R))
    for k in lls: lls[k].sort()
    return lls


def ins_ut(rs):
    out = []
    for i, r in enumerate(rs):
        out.append(r)
        if r == 1415 and i + 1 < len(rs) and rs[i + 1] == 1409: out.append(UT)
        if r == 1251 and i + 1 < len(rs) and rs[i + 1] == 1384: out.append(UT + 1)
    return out


VEH = {"sample": m20.VEH["sample"],
       "starex": dict(name="스타렉스(설계 차)", WB=3.2, FO=0.877, RO=1.073, W=1.92, TREAD=1.66, R=math.sqrt(5.6 ** 2 - 3.2 ** 2) - 1.66 / 2),
       "sprinter": dict(name="CARLA sprinter", WB=3.662, FO=1.015, RO=1.240, W=1.988, TREAD=1.676, R=math.sqrt(5.6 ** 2 - 3.2 ** 2) - 1.66 / 2)}
PAIRS = json.loads((C / "map/build_v31_s54/pairs30.json").read_text())
CHK = json.loads((OUT / "v33a_route_check.json").read_text())
RT = json.loads(subprocess.check_output(["/usr/bin/python3", "-c", "import sys,json,yaml;print(json.dumps(yaml.safe_load(open(sys.argv[1]))))",
                                         str(C / "map/data/routes/circulation_v33b_r1.yaml")]))
MID_S = next(s for s in RT["stops"] if s["platform"] == "middle_gate_s")
items = {}
for n, pr in PAIRS.items():
    rs = CHK["pairs_v33a"][n].get("roads")
    if not rs: continue
    s0 = 70.0 if pr["roads"][0] in (1320, 1191) else pr["start_s"]; s1 = 70.0 if pr["roads"][-1] in (1320, 1191) else pr["end_s"]
    items[n] = dict(roads=ins_ut(rs), start_s=s0, end_s=s1)
stop_s = {"main_gate_inner_nw": 52.0, "yangseong_se": 22.0, "lib_south_nw": 83.6, "lib_north": 78.0, "edu_front": 205.0, "middle_gate_s": 70.0, "back_gate_inner_se": 130.0}
for sg in RT["segments"]:
    items[f"circ_{sg['frm']}_{sg['to']}"] = dict(roads=ins_ut(sg["roads"]), start_s=stop_s[sg["frm"]], end_s=stop_s[sg["to"]])
items["uturn"] = dict(roads=[1415, UT, 1409], start_s=180.0, end_s=30.0)
items["uturn_back"] = dict(roads=[1251, UT + 1, 1384], start_s=110.0, end_s=30.0)
lls = load(OSM)
cache = {n: m20.centerline(lls, p) for n, p in items.items()}
out = {"osm_sha16": __import__("hashlib").sha256(OSM.read_bytes()).hexdigest()[:16], "vehicles": {}}
for key, v in VEH.items():
    cs = m20.corners(v); rv = {}
    for n in items:
        c, o, a = cache[n]
        bad, emax, omax = m20.body_check(c, o, a, cs)
        rlow, tight = m20.curvature(c, o, v["R"])
        rv[n] = dict(bad=len(bad), bad_roads=sorted({b[0] for b in bad}), out_max=round(omax, 2), r_min=round(rlow, 2), tight_roads=sorted(int(k) for k in tight),
                     bad_pts=[b[1:] for b in bad][:30])
    sm = lambda pre: dict(items=sum(1 for k in rv if k.startswith(pre)), bad_items=[k for k in rv if k.startswith(pre) and rv[k]["bad"]],
                          bad_pts=sum(rv[k]["bad"] for k in rv if k.startswith(pre)), tight_items=[k for k in rv if k.startswith(pre) and rv[k]["tight_roads"]],
                          bad_roads=sorted({r for k in rv if k.startswith(pre) for r in rv[k]["bad_roads"]}))
    out["vehicles"][key] = dict(spec={k: (round(x, 3) if isinstance(x, float) else x) for k, x in v.items()}, items=rv,
                                summary={g: sm(g) for g in ("p_", "circ_", "uturn", "uturn_back")})
    s = out["vehicles"][key]["summary"]
    print(key, f"R {v['R']:.2f}", {g: (x["bad_pts"], len(x["bad_items"]), x["bad_roads"], len(x["tight_items"])) for g, x in s.items()})
    print("   유턴", rv["uturn"])
(OUT / os.environ.get("S73_M20_OUT", "m20_v33a.json")).write_text(json.dumps(out, ensure_ascii=False, indent=1))
