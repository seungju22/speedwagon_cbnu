#!/usr/bin/env python3
# 맵세션30 Phase 3 보조: CommonRoad 중간(.cr.xml) lanelet 경계와 Lanelet2(.osm) lanelet 경계를 점 단위로 비교 (읽기전용, 시험 venv)
# 왜: 병합 뒤 남은 끊김이 "점 id 문제" 가 아니라 CR -> Lanelet2 단계에서 경계 기하가 바뀐 것인지 가른다
# 대응: cr2lanelet 은 scenario lanelet 순서대로 relation id 를 늘려 만든다. cr.xml 의 k 번째 lanelet <-> osm relation id 순 k 번째
#   이 가정은 "경계가 같은 쌍 수" 로 확인한다(가정이 틀리면 거의 0 이 나온다)
# 분류(좌·우 경계 각각): same(점 수 같고 모든 점 0.01m 안) / ends(안쪽 점은 같고 첫·끝 점만 0.05m 넘게 다름)
#   / reversed(순서를 뒤집으면 같음) / other
# 원인 A: CR -> Lanelet2 변환이 경계 첫·끝 점을 다른 lanelet 의 점으로 바꿈 / 원인 B: 공식 lanelet2 가 읽을 때 경계 순서를 뒤집음
# 사용: python s30_lanelet2_cr_compare.py <cr.xml> <osm> <xodr_lanes.csv> <xodr 복사본> [<남은 뒤 없음 id json> <병합본 osm>]
import collections
import csv
import json
import math
import sys
import xml.etree.ElementTree as ET

from pyproj import Transformer

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from s30_lanelet2_merge import GEO, OFF_X, OFF_Y, poly_d  # noqa: E402

t3857 = Transformer.from_crs("EPSG:3857", GEO, always_xy=True)
t4326 = Transformer.from_crs("EPSG:4326", GEO, always_xy=True)
txodr = lambda x, y: (x - OFF_X, y - OFF_Y)  # noqa: E731  xodr -> TM


def cr_lanelets(path):
    out = []
    for ll in ET.parse(path).getroot().findall("lanelet"):
        b = {}
        for side in ("leftBound", "rightBound"):
            b[side] = [t3857.transform(float(p.find("x").text), float(p.find("y").text)) for p in ll.find(side).findall("point")]
        out.append(dict(id=ll.get("id"), L=b["leftBound"], R=b["rightBound"], types=[x.text for x in ll.findall("laneletType")],
                        succ=[s.get("ref") for s in ll.findall("successor")]))
    return out


def osm_lanelets(path):
    root = ET.parse(path).getroot()
    nodes = {n.get("id"): t4326.transform(float(n.get("lon")), float(n.get("lat"))) for n in root.findall("node")}
    ways = {w.get("id"): [nd.get("ref") for nd in w.findall("nd")] for w in root.findall("way")}
    out = []
    for r in root.findall("relation"):
        tg = {t.get("k"): t.get("v") for t in r.findall("tag")}
        if tg.get("type") != "lanelet":
            continue
        mem = {x.get("role"): x.get("ref") for x in r.findall("member")}
        out.append(dict(id=int(r.get("id")), sub=tg.get("subtype"), L=[nodes[n] for n in ways[mem["left"]]],
                        R=[nodes[n] for n in ways[mem["right"]]], Lw=mem["left"], Rw=mem["right"]))
    out.sort(key=lambda x: x["id"])
    return out


def cls(a, b):
    d = lambda p, q: math.hypot(p[0] - q[0], p[1] - q[1])  # noqa: E731
    if len(a) == len(b) and all(d(p, q) < 0.01 for p, q in zip(a, b)):
        return "same"
    if len(a) == len(b) and len(a) > 2 and all(d(p, q) < 0.01 for p, q in zip(a[1:-1], b[1:-1])):
        return "ends"
    if len(a) == len(b) and all(d(p, q) < 0.01 for p, q in zip(a, b[::-1])):
        return "reversed"
    return "other"


def cr_lane_map(crp, osmp, lanes_csv):
    """xodr (road, lane) -> Lanelet2 lanelet id. CR 중심선으로 맞추고(CR 은 878/878 대응) 순서 대응으로 L2 id 를 얻는다.
    Lanelet2 쪽 중심선은 경계가 뒤집혀 읽힌 lanelet 에서 망가지므로 그쪽으로 맞추지 않는다."""
    cr, l2 = cr_lanelets(crp), osm_lanelets(osmp)
    cr2l2 = {a["id"]: b["id"] for a, b in zip(cr, l2)}
    cens = []
    for a in cr:
        cen = [((p[0] + q[0]) / 2, (p[1] + q[1]) / 2) for p, q in zip(a["L"], a["R"])]
        if len(cen) >= 2:
            cens.append((a["id"], cen, cen[len(cen) // 2]))
    out = {}
    for row in csv.DictReader(open(lanes_csv)):
        if not row["x0"]:
            continue
        pts = [txodr(float(row[f"x{k}"]), float(row[f"y{k}"])) for k in "0m1"]
        best = None
        for cid, cen, mid in cens:
            if abs(mid[0] - pts[1][0]) > 300 or abs(mid[1] - pts[1][1]) > 300:
                continue
            dd = max(poly_d(p, cen) for p in pts)
            if best is None or dd < best[0]:
                best = (dd, cid)
        if best and best[0] < 0.5:
            out[(int(row["road"]), int(row["lane"]))] = cr2l2[best[1]]
    return out


def flipped(osmp, m):
    """Lanelet2 가 읽은 경계 순서가 OSM way 순서와 반대인 lanelet id (쪽별)"""
    root = ET.parse(osmp).getroot()
    first = {int(w.get("id")): int(w.find("nd").get("ref")) for w in root.findall("way")}
    out = collections.defaultdict(set)
    for ll in m.laneletLayer:
        for s, b in (("L", ll.leftBound), ("R", ll.rightBound)):
            if b[0].id != first[b.id]:
                out[s].add(ll.id)
    return out


def main():
    crp, osmp, lanes_csv, xodr = sys.argv[1:5]
    left_dead = set(json.loads(open(sys.argv[5]).read())) if len(sys.argv) > 5 else None
    cr, l2 = cr_lanelets(crp), osm_lanelets(osmp)
    print(f"CR lanelet {len(cr)}, Lanelet2 lanelet {len(l2)}")
    pairs = list(zip(cr, l2))
    c = collections.Counter((cls(a["L"], b["L"]), cls(a["R"], b["R"])) for a, b in pairs)
    print(f"순서 대응 확인: 좌우 모두 same {c[('same', 'same')]}/{len(pairs)}")
    # xodr 차선 -> CR lanelet (표본 3점 0.5m 안)
    rows = list(csv.DictReader(open(lanes_csv)))
    lo = set()
    for rd in ET.parse(xodr).getroot().iter("road"):
        if any(abs(float(x.get(k))) > 1e-9 for x in rd.findall("lanes/laneOffset") for k in "abcd"):
            lo.add(int(rd.get("id")))
    road_of = collections.defaultdict(set)
    jn = {}
    for row in rows:
        if not row["x0"]:
            continue
        jn[int(row["road"])] = int(row["junction"])
        pts = [txodr(float(row[f"x{k}"]), float(row[f"y{k}"])) for k in "0m1"]
        best = None
        for a in cr:
            cen = [((p[0] + q[0]) / 2, (p[1] + q[1]) / 2) for p, q in zip(a["L"], a["R"])]
            if len(cen) < 2 or abs(cen[len(cen) // 2][0] - pts[1][0]) > 300 or abs(cen[len(cen) // 2][1] - pts[1][1]) > 300:
                continue
            dd = max(poly_d(p, cen) for p in pts)
            if best is None or dd < best[0]:
                best = (dd, a["id"])
        if best and best[0] < 0.5:
            road_of[best[1]].add((int(row["road"]), int(row["lane"])))
    veh = [(a, b) for a, b in pairs if b["sub"] != "walkway"]
    cv = collections.Counter((cls(a["L"], b["L"]), cls(a["R"], b["R"])) for a, b in veh)
    print(f"차량 lanelet {len(veh)}: (왼쪽, 오른쪽) 분류 {dict(cv)}")
    bad = [(a, b) for a, b in veh if (cls(a["L"], b["L"]), cls(a["R"], b["R"])) != ("same", "same")]
    print(f"경계가 CR 과 다른 차량 lanelet {len(bad)}")
    roads = [r for a, _ in bad for r, ln in road_of[a["id"]] if ln == -1]
    print(f"  그 xodr road(lane -1) {len(roads)}: laneOffset 있음 {sum(r in lo for r in roads)}, junction 커넥터 {sum(jn[r] != -1 for r in roads)},"
          f" 일반 road {sum(jn[r] == -1 for r in roads)}")
    print(f"  laneOffset 있는 road 전체 {len(lo)} 중 경계 바뀜 {len(set(roads) & lo)}")
    print(f"  경계 바뀐 쪽 수 왼쪽만 {sum(1 for a, b in bad if cls(a['R'], b['R']) == 'same')},"
          f" 오른쪽만 {sum(1 for a, b in bad if cls(a['L'], b['L']) == 'same')}, 둘 다 {sum(1 for a, b in bad if 'same' not in (cls(a['L'], b['L']), cls(a['R'], b['R'])))}")
    ends_far = []
    for a, b in bad:
        for s in "LR":
            if cls(a[s], b[s]) == "ends":
                ends_far.append(max(math.hypot(a[s][0][0] - b[s][0][0], a[s][0][1] - b[s][0][1]),
                                    math.hypot(a[s][-1][0] - b[s][-1][0], a[s][-1][1] - b[s][-1][1])))
    if ends_far:
        print(f"  ends 분류의 끝점 이동 거리 최소 {min(ends_far):.3f} 중앙 {sorted(ends_far)[len(ends_far) // 2]:.3f} 최대 {max(ends_far):.3f}m")
    ex = [(b["id"], a["id"], sorted(r for r, ln in road_of[a["id"]] if ln == -1), cls(a["L"], b["L"]), cls(a["R"], b["R"])) for a, b in bad[:12]]
    print(f"  예(L2 id, CR id, road, 왼, 오) {ex}")
    for r in (1917, 1286, 1143, 1380):
        hit = [(b["id"], cls(a["L"], b["L"]), cls(a["R"], b["R"])) for a, b in veh if (r, -1) in road_of[a["id"]]]
        print(f"  r{r}: {hit}")
    bad_ids = {b["id"] for _, b in bad}
    json.dump(sorted(bad_ids), open(osmp.rsplit("/", 1)[0] + "/s30_bad_bounds.json", "w"))
    # 원인 B: 공식 lanelet2 가 읽을 때 경계를 뒤집음(OSM way 순서는 CR 과 같은데 읽은 순서가 반대)
    import lanelet2
    from s30_lanelet2_merge import proj, rules
    m, _ = lanelet2.io.loadRobust(osmp, proj)
    fl = flipped(osmp, m)
    vf = {s: {i for i in v if rules.canPass(m.laneletLayer[i])} for s, v in fl.items()}
    print(f"[B] lanelet2 가 뒤집어 읽은 경계: 차량 왼쪽 {len(vf.get('L', ()))}, 차량 오른쪽 {len(vf.get('R', ()))},"
          f" 보행 왼쪽 {len(fl['L']) - len(vf.get('L', ()))}, 보행 오른쪽 {len(fl['R']) - len(vf.get('R', ()))} (원본 osm)")
    l2road = collections.defaultdict(set)
    for (r, ln), lid in cr_lane_map(crp, osmp, lanes_csv).items():
        if ln == -1:
            l2road[lid].add(r)
    uturn = {t["road"] for t in json.loads(open("/home/gyumin/campus_mobility_sim/map/docs/logs/s28_turnaround_removed.json").read())["templates"]}
    if left_dead is not None:
        mm, _ = lanelet2.io.loadRobust(sys.argv[6], proj) if len(sys.argv) > 6 else (m, None)
        flm = flipped(sys.argv[6], mm) if len(sys.argv) > 6 else fl
        flv = {i for v in flm.values() for i in v if rules.canPass(mm.laneletLayer[i])}
        print(f"[B] 병합본에서 뒤집혀 읽힌 차량 lanelet {len(flv)}: road {sorted(r for i in flv for r in l2road[i])}")
        print(f"    그중 junction 커넥터 {sum(1 for i in flv for r in l2road[i] if jn[r] != -1)}, 일반 road {sum(1 for i in flv for r in l2road[i] if jn[r] == -1)}, 회차 틀 {sum(1 for i in flv for r in l2road[i] if r in uturn)}")
        # 남은 뒤 없음 lanelet 분류
        cr2l2 = {a["id"]: b["id"] for a, b in pairs}
        l22cr = {b["id"]: a for a, b in pairs}
        k = collections.Counter()
        for lid in left_dead:
            a = l22cr[lid]
            rs = l2road[lid]
            kind = "회차 틀" if rs & uturn else "junction 커넥터" if any(jn[r] != -1 for r in rs) else "일반 road" if rs else "road 대응 없음"
            if not a["succ"]:
                why = "CR 에서도 뒤 없음(xodr 막다른 끝)"
            elif lid in flv:
                why = "자신 경계 뒤집혀 읽힘[B]" + ("+끝점 바뀜[A]" if lid in bad_ids else "")
            elif lid in bad_ids:
                why = "자신 끝점 바뀜[A]"
            elif any(cr2l2[s] in flv for s in a["succ"]):
                why = "CR 뒤 lanelet 이 뒤집혀 읽힘[B]"
            elif any(cr2l2[s] in bad_ids for s in a["succ"]):
                why = "CR 뒤 lanelet 끝점 바뀜[A]"
            else:
                why = "원인 못 찾음"
            k[f"{kind} / {why}"] += 1
        print(f"남은 뒤 없음 {len(left_dead)} 분류:")
        for kk, v in sorted(k.items(), key=lambda x: -x[1]):
            print(f"  {kk}: {v}")


if __name__ == "__main__":
    main()
