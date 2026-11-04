#!/usr/bin/env python3
# 11/04: s75_m20.py 사본. 지도 v3.3d, 후문 앞 유턴 = 섬 둘레 52806(34923 -> 52806 -> 34884). 출력 integration/logs/s77/m20_v33d.json
# (s75 머리말) 11/03: v3.3c 차체 검사(레인렛 id 열 판). s74_m20.py 는 road 번호로 중심선을 이어서 v3.3c(r1415·r1409 를 나눔)에 못 씀
#   경로 = s75_check_v33c.py 가 Lanelet2 로 찾은 레인렛 id 열(30쌍 개성재 자리, 순환 구간) + 유턴 둘(앞뒤 12m)
#   중심선 = 레인렛마다 왼·오 경계를 0.5m 로 다시 나눈 가운데(s59_m20.centerline 과 같은 식), 첫·끝 레인렛은 start_s·end_s(레인렛 안 길이)로 자름
#   (가) 차체: 네 모서리가 경로 레인렛 합친 면 안에 드는지(옆으로 OFFS 만큼 비켜서라도), s59_m20.body_check 그대로
#   (나) CARLA 바퀴: 중심선(뒷축)을 그대로 따라갈 때 바퀴 4개(타이어 반폭 0.12m 더함)가 CARLA 면(동결 xodr, Driving|Sidewalk) 밖인 점
#       (차로 경계의 가는 틈은 면으로 침: 0.2m 맞은편 두 점이 둘 다 면)
#   (다) 곡률: 중심선 반경이 차 최소 반경 R 밑인 점
#   차: sample(지금), starex(설계 차), sprinter(CARLA 실측 5.915 x 1.988, 축거 3.661, 오버행 1.015/1.239, max_steer 0.77 -> R 3.77)
# 사용: .venv-carla/bin/python map/scripts/s75_m20.py (shapely 는 시스템 python 에도 있음, carla 는 .venv-carla)
# 출력 integration/logs/s75/m20_v33c.json
import json, math, sys, xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent; C = HERE.parents[1]; sys.path.insert(0, str(HERE))
import s59_m20 as m20
import s53_footprint as fp
from shapely.geometry import Polygon
from shapely.ops import unary_union
import s72_common as K
import carla

OSM = C / "map/maps/v33d/lanelet2_map.osm"; OUT = C / "integration/logs/s77/m20_v33d.json"
CHK = json.loads((C / "integration/logs/s77/v33d_route_check.json").read_text())
CH = json.loads((C / "map/maps/v33c/v33c_changes.json").read_text())
PAIRS = json.loads((C / "map/build_v31_s54/pairs30.json").read_text())
r = ET.parse(OSM).getroot()
nodes = {}
for n in r.iter("node"):
    t = {x.get("k"): x.get("v") for x in n.findall("tag")}; nodes[n.get("id")] = (float(t["local_x"]), float(t["local_y"]))
ways = {w.get("id"): [nd.get("ref") for nd in w.findall("nd")] for w in r.iter("way")}
LL = {}; CLN = {}   # CLN: role=centerline 이 있는 레인렛(11/03 유턴)은 그 점을 가운데선으로
for rel in r.iter("relation"):
    t = {x.get("k"): x.get("v") for x in rel.findall("tag")}
    if t.get("type") != "lanelet": continue
    mem = {m.get("role"): m.get("ref") for m in rel.findall("member")}
    LL[int(rel.get("id"))] = ([nodes[q] for q in ways[mem["left"]]], [nodes[q] for q in ways[mem["right"]]], t)
    if "centerline" in mem: CLN[int(rel.get("id"))] = [nodes[q] for q in ways[mem["centerline"]]]


def centerline(ids, s0, s1):
    polys, center, owner = [], [], []
    for k, lid in enumerate(ids):
        L, R, t = LL[lid]; polys.append(Polygon(L + R[::-1]).buffer(0))
        n = max(3, int(sum(math.dist(a, b) for a, b in zip(L, L[1:])) / 0.5) + 1)
        if lid in CLN:
            cl, _ = fp.resample(CLN[lid], max(3, int(sum(math.dist(p, q) for p, q in zip(CLN[lid], CLN[lid][1:])) / 0.5) + 1))
        else:
            a, _ = fp.resample(L, n); b, _ = fp.resample(R, n)
            cl = [((p[0] + q[0]) / 2, (p[1] + q[1]) / 2) for p, q in zip(a, b)]
        acc = [0.0]
        for p, q in zip(cl, cl[1:]): acc.append(acc[-1] + math.dist(p, q))
        for p, s in zip(cl, acc):
            if k == 0 and s < max(s0, fp.MIN_START): continue
            if k == len(ids) - 1 and s > s1: continue
            if center and math.dist(center[-1], p) < 0.25: continue
            center.append(p); owner.append(lid)
    return center, owner, unary_union([p.buffer(0.02) for p in polys]).buffer(-0.02)


M = K.cmap(); RD = carla.LaneType.Driving | carla.LaneType.Sidewalk; _c = {}


def on0(x, y):
    k = (round(x * 20), round(y * 20))
    if k not in _c: _c[k] = M.get_waypoint(carla.Location(k[0] / 20, -k[1] / 20, 0.0), project_to_road=False, lane_type=RD) is not None
    return _c[k]


def on(x, y):
    """CARLA 면 위인지. 차로 경계의 가는 틈(0.2m 양옆 둘 다 면, 또는 위아래 둘 다 면, 또는 대각 한 쌍이 면)은 면으로 침"""
    if on0(x, y): return True
    d = 0.2
    return any(on0(x + dx, y + dy) and on0(x - dx, y - dy) for dx, dy in ((d, 0), (0, d), (d, d), (d, -d)))


def wheel_check(center, owner, v):
    bad = []
    for i, p in enumerate(center):
        a = center[max(0, i - 2)]; b = center[min(len(center) - 1, i + 2)]; h = math.atan2(b[1] - a[1], b[0] - a[0]); c, s = math.cos(h), math.sin(h)
        off = [(x_, y_) for x_ in (0.0, v["WB"]) for y_ in (-v["TREAD"] / 2 - 0.12, v["TREAD"] / 2 + 0.12)
               if not on(p[0] + x_ * c - y_ * s, p[1] + x_ * s + y_ * c)]
        if off: bad.append((owner[i], round(p[0], 2), round(p[1], 2), len(off)))
    return bad


VEH = {"sample": m20.VEH["sample"],
       "starex": dict(name="스타렉스(설계 차)", WB=3.2, FO=0.877, RO=1.073, W=1.92, TREAD=1.66, R=3.2 / math.tan(0.704)),
       "sprinter": dict(name="CARLA sprinter(s75 vehicle_info)", WB=3.661, FO=1.015, RO=1.239, W=1.988, TREAD=1.676, R=3.661 / math.tan(0.77))}
stop_s = {"main_gate_inner_nw": 52.0, "yangseong_se": 22.0, "lib_south_nw": 83.6, "lib_north": 78.0, "middle_gate_s": 70.0, "back_gate_inner_se": 130.0}
items = {}
for n, pr in PAIRS.items():
    ids = CHK["pairs_v33a"][n].get("ids")
    if not ids: continue
    s0 = 70.0 if pr["roads"][0] in (1320, 1191) else pr["start_s"]; s1 = 70.0 if pr["roads"][-1] in (1320, 1191) else pr["end_s"]
    items[n] = (ids, s0, s1)
for sg in CHK["circulation"]:
    items[f"circ_{sg['frm']}_{sg['to']}"] = (sg["ids"], stop_s[sg["frm"]], stop_s[sg["to"]])
ut = int(CH["uturn"]["relation"]); fa, tb = int(CH["uturn"]["from_lanelet"]), int(CH["uturn"]["to_lanelet"])
lenA = sum(math.dist(p, q) for p, q in zip(LL[fa][0], LL[fa][0][1:]))
items["uturn_lib"] = ([fa, ut, tb], lenA - 12, 12.0)
CD = json.loads((C / "map/maps/v33d/v33d_changes.json").read_text())
bu = int(CD["uturn"]["relation"]); pre, nxt = int(CD["uturn"]["from_lanelet"]), int(CD["uturn"]["to_lanelet"])
lenP = sum(math.dist(p, q) for p, q in zip(LL[pre][0], LL[pre][0][1:]))
nx2 = next(k for k, (L, R, t) in LL.items() if k != nxt and math.dist(L[0], LL[nxt][0][-1]) < 0.01 and math.dist(R[0], LL[nxt][1][-1]) < 0.01)   # 34884 뒤 조각이 짧아서(약 3m) 다음 레인렛(49817)까지
items["uturn_back"] = ([pre, bu, nxt, nx2], max(0.0, lenP - 12), 8.0)
cache = {n: centerline(*it) for n, it in items.items()}
out = {"osm_sha16": __import__("hashlib").sha256(OSM.read_bytes()).hexdigest()[:16], "vehicles": {}}
for key, v in VEH.items():
    cs = m20.corners(v); rv = {}
    for n in items:
        c, o, a = cache[n]
        bad, emax, omax = m20.body_check(c, o, a, cs); rlow, tight = m20.curvature(c, o, v["R"]); wb = wheel_check(c, o, v)
        rv[n] = dict(bad=len(bad), bad_lanelets=sorted({b[0] for b in bad}), out_max=round(omax, 2), r_min=round(rlow, 2), tight=sorted(int(k) for k in tight),
                     wheel_off=len(wb), wheel_off_lanelets=sorted({b[0] for b in wb}), bad_pts=[b[1:] for b in bad][:30], wheel_pts=[b[1:] for b in wb][:40])
    sm = lambda pre_: dict(items=sum(1 for k in rv if k.startswith(pre_)), bad_items=[k for k in rv if k.startswith(pre_) and rv[k]["bad"]],
                           bad_pts=sum(rv[k]["bad"] for k in rv if k.startswith(pre_)), wheel_items=[k for k in rv if k.startswith(pre_) and rv[k]["wheel_off"]],
                           wheel_pts=sum(rv[k]["wheel_off"] for k in rv if k.startswith(pre_)), tight_items=[k for k in rv if k.startswith(pre_) and rv[k]["tight"]])
    out["vehicles"][key] = dict(spec={k: (round(x, 3) if isinstance(x, float) else x) for k, x in v.items()}, items=rv,
                                summary={g: sm(g) for g in ("p_", "circ_", "uturn_lib", "uturn_back")})
    print(key, f"R {v['R']:.2f}")
    for g, x in out["vehicles"][key]["summary"].items():
        print(f"  {g}: 차체 {x['bad_pts']}점 {len(x['bad_items'])}건, 바퀴 {x['wheel_pts']}점 {len(x['wheel_items'])}건, 급곡 {len(x['tight_items'])}건")
OUT.write_text(json.dumps(out, ensure_ascii=False, indent=1))
