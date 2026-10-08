#!/usr/bin/env python3
# 맵세션30: Lanelet2 점 병합 후처리 시험 (오프라인, 서버·재변환 없음). 시험 venv 로 실행(~/lanelet2_trial_venv/bin/python)
# 입력: s29_lanelet2_trial.py 변환 결과(.osm, 읽기만 함), xodr 복사본(링크·junction 읽기만), xodr_lanes.csv(s29_xodr_lanes.py)
# 출력: <작업 디렉터리>/s30_merged_<mode>_<tol>.osm (새 파일. 입력 osm 은 덮어쓰지 않는다)
# 병합: 허용오차 tol 안의 점을 대표점 하나로 바꾼다(way 의 nd 참조만 바꿈, 대표점 좌표 그대로)
#   대표점 고르기: 점 id 순서로 보며 이미 정한 대표점 tol 안이면 그것에 붙인다(사슬처럼 번지는 단일연결 병합 방지)
#   mode all = 모든 점 / ends = linestring 끝점만 (안쪽 점 병합이 과병합의 원인인지 가르기 위해 둘 다 본다)
# 평가(공식 lanelet2 1.2.3, germany vehicle 규칙): 뒤 없음 수, 정문 도달 수, 노선 4개 따라가기, 과병합 지표
# 사용: python s30_lanelet2_merge.py <작업 디렉터리> <입력 osm> <xodr 복사본> <xodr_lanes.csv> [--tols 0.01,0.05,...] [--modes all,ends]
import argparse
import collections
import csv
import hashlib
import json
import math
import sys
import xml.etree.ElementTree as XET
from pathlib import Path

import lanelet2
from lanelet2.core import BasicPoint2d, GPSPoint
from lanelet2.io import Origin
from lanelet2.projection import UtmProjector
from lxml import etree
from pyproj import Transformer

FROZEN = Path("/home/gyumin/campus_mobility_sim/map/maps/cbnu_internal_only_localtm_tags73_smooth.xodr")
LOGS = Path("/home/gyumin/campus_mobility_sim/map/docs/logs")
LAT0, LON0 = 36.627298, 127.456394
GEO = f"+proj=tmerc +lat_0={LAT0} +lon_0={LON0} +ellps=WGS84"   # frozen_v1 header geoReference
OFF_X, OFF_Y = 521.51, 493.61       # xodr = TM + 오프셋 (s19_scope_common)
GATE = (36.631961, 127.453056)       # 정문 r1247 lane -1 중간(s29_lanelet2_load.py 와 같은 시작점)
MATCH_TOL = 0.5                      # xodr 차선 표본 -> lanelet 중심선 대응 허용치(s29 와 같음)
# 주행 검증 끝난 4개 노선(tests/test_drive.py ROUTES, 세션20). 모두 lane -1
ROUTES = {
    "북문": [1247, 1914, 1356, 1917, 1355, 1447, 1373, 1673, 1374, 1591, 1375, 1689, 1376, 1625, 1377, 1640, 1378],
    "남문": [1247, 1914, 1356, 1917, 1355, 1446, 1172, 1428, 1240, 1695, 1239, 1426, 1238, 1563, 1237],
    "중문": [1247, 1914, 1356, 1917, 1355, 1447, 1373, 1673, 1374, 1592, 1283, 1617, 1284, 1606, 1285, 1597, 1286,
           1838, 1327],
    "양성재": [1247, 1914, 1356, 1917, 1355, 1446, 1172, 1427, 1127, 1699, 1126, 1726, 1125, 1733, 1124, 1826, 1123],
}

proj = UtmProjector(Origin(LAT0, LON0))
rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                      lanelet2.traffic_rules.Participants.Vehicle)


def sha16(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()[:16]


def seg_d(p, a, b):
    L2 = (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2
    t = 0 if L2 == 0 else max(0, min(1, ((p[0] - a[0]) * (b[0] - a[0]) + (p[1] - a[1]) * (b[1] - a[1])) / L2))
    return math.hypot(p[0] - a[0] - t * (b[0] - a[0]), p[1] - a[1] - t * (b[1] - a[1]))


def poly_d(p, pts):
    return min(seg_d(p, pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def heading(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def ang_diff(a, b):
    d = abs(a - b) % (2 * math.pi)
    return math.degrees(min(d, 2 * math.pi - d))


def area(ll):
    pts = [(p.x, p.y) for p in ll.leftBound] + [(p.x, p.y) for p in reversed(list(ll.rightBound))]
    return abs(sum(pts[i][0] * pts[(i + 1) % len(pts)][1] - pts[(i + 1) % len(pts)][0] * pts[i][1]
                   for i in range(len(pts)))) / 2


# ---------- xodr 링크 ----------
def xodr_links(xodr):
    root = XET.parse(str(xodr)).getroot()
    junc_of, succ_type = {}, {}
    adj = collections.defaultdict(set)
    jroads = collections.defaultdict(set)
    for r in root.iter("road"):
        rid, j = int(r.get("id")), int(r.get("junction"))
        junc_of[rid] = j
        if j != -1:
            jroads[j].add(rid)
        lk = r.find("link")
        for kind in ("predecessor", "successor"):
            e = lk.find(kind) if lk is not None else None
            if kind == "successor":
                succ_type[rid] = None if e is None else (e.get("elementType"), int(e.get("elementId")))
            if e is not None and e.get("elementType") == "road":
                adj[rid].add(int(e.get("elementId")))
                adj[int(e.get("elementId"))].add(rid)
    for jn in root.iter("junction"):
        j = int(jn.get("id"))
        for c in jn.iter("connection"):
            a, b = int(c.get("incomingRoad")), int(c.get("connectingRoad"))
            adj[a].add(b)
            adj[b].add(a)
            jroads[j].add(a)
    return junc_of, succ_type, adj, jroads


def linked(a, b, adj, junc_of, jroads):
    if a == b or b in adj[a]:
        return True
    ja, jb = junc_of.get(a, -1), junc_of.get(b, -1)
    # 같은 junction 안(커넥터끼리, 또는 그 junction 에 들어오는 road)이면 끝점 공유가 정상
    for j in {ja, jb} - {-1}:
        if a in jroads[j] and b in jroads[j]:
            return True
    return False


# ---------- lanelet -> xodr road 대응 ----------
def lanelet_roads(m, lanes_csv):
    to_ll = Transformer.from_crs(GEO, "EPSG:4326", always_xy=True)
    rows = list(csv.DictReader(open(lanes_csv)))
    cen = {}
    for ll in m.laneletLayer:
        c = [(p.x, p.y) for p in ll.centerline]
        if len(c) >= 2:
            cen[ll.id] = (c, min(p[0] for p in c), max(p[0] for p in c), min(p[1] for p in c), max(p[1] for p in c))
    road_of = collections.defaultdict(set)
    lane_ll = {}
    samples = {}
    for row in rows:
        if not row["x0"]:
            continue
        pts = []
        for k in ("0", "m", "1"):
            lon, lat = to_ll.transform(float(row[f"x{k}"]) - OFF_X, float(row[f"y{k}"]) - OFF_Y)
            q = proj.forward(GPSPoint(lat, lon, 0))
            pts.append((q.x, q.y))
        best = None
        for lid, (c, x0, x1, y0, y1) in cen.items():
            if pts[1][0] < x0 - 30 or pts[1][0] > x1 + 30 or pts[1][1] < y0 - 30 or pts[1][1] > y1 + 30:
                continue
            d = max(poly_d(p, c) for p in pts)
            if best is None or d < best[0]:
                best = (d, lid)
        key = (int(row["road"]), int(row["lane"]))
        samples[key] = pts
        if best and best[0] <= MATCH_TOL:
            lane_ll[key] = best[1]
            road_of[best[1]].add(key[0])
    return road_of, lane_ll, samples


# ---------- 병합 ----------
def build_merge(m, tol, mode, endpoint_ids):
    pts = sorted(((p.id, p.x, p.y) for p in m.pointLayer if mode == "all" or p.id in endpoint_ids))
    grid = collections.defaultdict(list)
    rep = {}
    cs = max(tol, 1e-6)
    for pid, x, y in pts:
        gx, gy = int(math.floor(x / cs)), int(math.floor(y / cs))
        hit = None
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for rid, rx, ry in grid[(gx + dx, gy + dy)]:
                    if math.hypot(rx - x, ry - y) <= tol and (hit is None or rid < hit[0]):
                        hit = (rid, rx, ry)
        if hit:
            rep[pid] = hit[0]
        else:
            grid[(gx, gy)].append((pid, x, y))
    return rep


def write_merged(src_tree, rep, out):
    root = src_tree.getroot()
    degenerate = 0
    used = set()
    for w in root.iter("way"):
        nds = w.findall("nd")
        prev = None
        for nd in nds:
            r = int(nd.get("ref"))
            r = rep.get(r, r)
            if r == prev:
                w.remove(nd)          # 연속 중복 제거(같은 linestring 안 두 점이 합쳐짐)
                continue
            nd.set("ref", str(r))
            used.add(r)
            prev = r
        if len(w.findall("nd")) < 2:
            degenerate += 1
    removed = 0
    for n in list(root.iter("node")):
        if int(n.get("id")) not in used and n.get("id") is not None and int(n.get("id")) in rep:
            root.remove(n)
            removed += 1
    out.write_bytes(etree.tostring(root, xml_declaration=True, encoding="UTF-8", pretty_print=True))
    return degenerate, removed


# ---------- 평가 ----------
def graph_of(m):
    return lanelet2.routing.RoutingGraph(m, rules)


def gate_start(m, passable):
    p = proj.forward(GPSPoint(GATE[0], GATE[1], 0))
    return min(passable, key=lambda ll: lanelet2.geometry.distance(lanelet2.geometry.to2D(ll), BasicPoint2d(p.x, p.y)))


def route_check(m, g, lane_ll):
    out = {}
    for name, roads in ROUTES.items():
        seq = []
        miss = [r for r in roads if (r, -1) not in lane_ll]
        for r in roads:
            lid = lane_ll.get((r, -1))
            if lid is not None and (not seq or seq[-1][0] != lid):
                seq.append((lid, r))
        lid_seq = [s[0] for s in seq]
        brks = []
        for (a, ra), (b, rb) in zip(seq, seq[1:]):
            fol = {x.id for x in g.following(m.laneletLayer[a])}
            if b not in fol:
                brks.append((ra, a, rb, b))
        brk = brks[0] if brks else None
        sp = g.shortestPath(m.laneletLayer[lid_seq[0]], m.laneletLayer[lid_seq[-1]])
        sp_ids = [ll.id for ll in sp] if sp else None
        out[name] = dict(n_route_ll=len(lid_seq), miss=miss, brk=brk, sp_len=len(sp_ids) if sp_ids else None,
                         sp_same=(sp_ids == lid_seq) if sp_ids else None, lid_seq=lid_seq, brks=brks)
    return out


def point_info(m, road_of):
    # 점 id -> [(lanelet id, 방향(rad), 끝점인가, 차량인가)]
    info = collections.defaultdict(list)
    for ll in m.laneletLayer:
        veh = rules.canPass(ll)
        for b in (ll.leftBound, ll.rightBound):
            ps = list(b)
            for i, p in enumerate(ps):
                a, c = ps[max(i - 1, 0)], ps[min(i + 1, len(ps) - 1)]
                h = heading((a.x, a.y), (c.x, c.y)) if (a.x, a.y) != (c.x, c.y) else None
                info[p.id].append((ll.id, h, i in (0, len(ps) - 1), veh))
    return info


def overmerge(rep, info, road_of, adj, junc_of, jroads, xy):
    # 반대 방향 지표 opp 는 "그 점을 쓰는 lanelet 중 반대 방향 쌍이 있음" 이라 half-road 중앙선 공유점이면 다 걸린다
    # 그래서 opp_only(같은 방향 쌍이 하나도 없고 반대 방향 쌍만 있음)와 이동 거리(moved, 1mm 넘게 옮겨진 점)를 따로 센다
    diff_road = unlinked = opp = interior = opp_only = moved = moved_unlinked = moved_opp_only = 0
    ex_unlinked, ex_opp = [], []
    dist = collections.Counter()
    for pid, r in rep.items():
        d = math.hypot(xy[pid][0] - xy[r][0], xy[pid][1] - xy[r][1])
        dist["<1mm" if d < 0.001 else "<1cm" if d < 0.01 else "<5cm" if d < 0.05 else "<10cm" if d < 0.1 else
             "<30cm" if d < 0.3 else "<50cm" if d < 0.5 else "<=1m"] += 1
        mv = d >= 0.001
        moved += mv
        A, B = info.get(pid, []), info.get(r, [])
        ra = set().union(*[road_of.get(x[0], set()) for x in A]) if A else set()
        rb = set().union(*[road_of.get(x[0], set()) for x in B]) if B else set()
        if not any(x[2] for x in A) or not any(x[2] for x in B):
            interior += 1
        if ra and rb and not (ra & rb):
            diff_road += 1
            if not any(linked(a, b, adj, junc_of, jroads) for a in ra for b in rb):
                unlinked += 1
                moved_unlinked += mv
                if mv and len(ex_unlinked) < 6:
                    ex_unlinked.append((pid, r, sorted(ra), sorted(rb), round(d, 3)))
        # 반대 방향: 차량 lanelet 끼리, 서로 다른 lanelet, 방향 차 135도 넘음
        o = same = False
        for la, ha, _, va in A:
            for lb, hb, _, vb in B:
                if va and vb and la != lb and ha is not None and hb is not None:
                    ad = ang_diff(ha, hb)
                    o |= ad > 135
                    same |= ad <= 45
        if o:
            opp += 1
        if o and not same:
            opp_only += 1
            moved_opp_only += mv
            if len(ex_opp) < 6:
                ex_opp.append((pid, r, sorted(ra), sorted(rb), round(d, 3)))
    return dict(diff_road=diff_road, unlinked=unlinked, opp=opp, interior=interior, opp_only=opp_only,
                moved=moved, moved_unlinked=moved_unlinked, moved_opp_only=moved_opp_only, dist=dict(dist),
                ex_unlinked=ex_unlinked, ex_opp=ex_opp)


def evaluate(path, base=None):
    m, errs = lanelet2.io.loadRobust(str(path), proj)
    lls = list(m.laneletLayer)
    passable = [ll for ll in lls if rules.canPass(ll)]
    g = graph_of(m)
    dead = [ll.id for ll in passable if not g.following(ll)]
    nopred = [ll.id for ll in passable if not g.previous(ll)]
    start = gate_start(m, passable)
    reach = g.reachableSet(start, 1e9, 0, True)
    reach_nolc = g.reachableSet(start, 1e9, 0, False)
    edges = {(ll.id, f.id) for ll in passable for f in g.following(ll)}
    lc = sum(len(g.lefts(ll)) + len(g.rights(ll)) for ll in passable)
    ar = {ll.id: area(ll) for ll in lls}
    return dict(m=m, g=g, errs=len(errs), n_ll=len(lls), n_pass=len(passable), dead=dead, nopred=nopred,
                start=start.id, reach=len(reach), reach_nolc=len(reach_nolc), edges=edges, lc=lc, area=ar,
                n_pt=len(m.pointLayer), invalid=len(g.checkValidity()))


def edge_turn(m, a, b):
    la, lb = m.laneletLayer[a], m.laneletLayer[b]
    ca, cb = [(p.x, p.y) for p in la.centerline], [(p.x, p.y) for p in lb.centerline]
    if len(ca) < 2 or len(cb) < 2 or ca[-2] == ca[-1] or cb[0] == cb[1]:
        return 999.0                  # 중심선이 점 하나로 무너짐(과병합 신호). 꺾임 지표에 999 로 남긴다
    return ang_diff(heading(ca[-2], ca[-1]), heading(cb[0], cb[1]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("work")
    ap.add_argument("osm")
    ap.add_argument("xodr")
    ap.add_argument("lanes")
    ap.add_argument("--cr", help="CommonRoad 중간 .cr.xml. 주면 road -> lanelet 대응을 CR 경유로(권장)")
    ap.add_argument("--idmap", help="옛 -> 새 road id 대응표 csv(s27_id_map). 주면 ROUTES 를 번역(세션31, 재변환 맵용)")
    ap.add_argument("--tols", default="0.01,0.05,0.1,0.3,0.5,1.0")
    ap.add_argument("--modes", default="ends,all")
    a = ap.parse_args()
    work, src = Path(a.work), Path(a.osm)
    if Path(a.xodr).resolve() == FROZEN.resolve():
        sys.exit("원본을 직접 입력으로 쓰지 않는다. 복사본을 주라")
    print(f"frozen_v1 sha {sha16(FROZEN)} / 입력 osm {src.name} sha {sha16(src)}")
    junc_of, succ_type, adj, jroads = xodr_links(a.xodr)
    if a.idmap:
        mp = {int(r["old_id"]): int(r["new_id"]) for r in csv.DictReader(open(a.idmap))}
        for k in ROUTES:
            ROUTES[k] = [mp[r] for r in ROUTES[k]]
        print(f"노선 번역(--idmap {Path(a.idmap).name}): {ROUTES}")

    base = evaluate(src)
    m0 = base["m"]
    road_of, lane_ll, samples = lanelet_roads(m0, a.lanes)
    drive = [k for k in lane_ll if k[1] == -1]
    print(f"(L2 중심선 기준) xodr 주행 차선 -> lanelet 대응 {len(drive)}/{sum(1 for k in samples if k[1] == -1)}")
    if a.cr:
        from s30_lanelet2_cr_compare import cr_lane_map
        lane_ll = cr_lane_map(a.cr, str(src), a.lanes)
        road_of = collections.defaultdict(set)
        for (r, ln), lid in lane_ll.items():
            road_of[lid].add(r)
        drive = [k for k in lane_ll if k[1] == -1]
    print(f"xodr 주행 차선 -> lanelet 대응 {len(drive)}/{sum(1 for k in samples if k[1] == -1)}")
    info = point_info(m0, road_of)
    endpoint_ids = {p for p, v in info.items() if any(x[2] for x in v)}
    xy = {p.id: (p.x, p.y) for p in m0.pointLayer}
    print(f"point {base['n_pt']}, linestring 끝점 {len(endpoint_ids)}")
    uturn = {t["road"] for t in json.loads((LOGS / "s28_turnaround_removed.json").read_text())["templates"]}

    def summary(tag, ev, rt):
        print(f"[{tag}] load 오류 {ev['errs']}, lanelet {ev['n_ll']}(차량 {ev['n_pass']}), point {ev['n_pt']},"
              f" checkValidity {ev['invalid']}, 뒤 없음 {len(ev['dead'])}, 앞 없음 {len(ev['nopred'])},"
              f" 정문({ev['start']}) 도달 {ev['reach']}/{ev['n_pass']} (차선변경 없이 {ev['reach_nolc']}),"
              f" 연결 간선 {len(ev['edges'])}, 차선변경 이웃 {ev['lc']}")
        for name, r in rt.items():
            ok = r["brk"] is None
            print(f"  노선 {name}: 노선 lanelet {r['n_route_ll']}개, " +
                  ("road->lanelet 대응 없음 " + str(r["miss"]) + ", " if r["miss"] else "") +
                  ("순서대로 다 이어짐" if ok else f"끊김 {len(r['brks'])}곳 " + ", ".join(f"r{x[0]}(ll {x[1]})->r{x[2]}(ll {x[3]})" for x in r['brks'])) +
                  f", 최단경로 {'없음' if r['sp_len'] is None else str(r['sp_len']) + '개'}" +
                  ("" if r["sp_same"] is None else f"(노선과 같은 lanelet 열 {r['sp_same']})"))

    rt0 = route_check(m0, base["g"], lane_ll)
    summary("원본", base, rt0)
    results = {}
    for mode in a.modes.split(","):
        for tol in [float(t) for t in a.tols.split(",")]:
            rep = build_merge(m0, tol, mode, endpoint_ids)
            out = work / f"s30_merged_{mode}_{tol:g}.osm"
            deg, removed = write_merged(etree.parse(str(src)), rep, out)
            ev = evaluate(out)
            rt = route_check(ev["m"], ev["g"], lane_ll)
            om = overmerge(rep, info, road_of, adj, junc_of, jroads, xy)
            new_edges = ev["edges"] - base["edges"]
            lost_edges = base["edges"] - ev["edges"]
            sharp = [(x, y, round(edge_turn(ev["m"], x, y))) for x, y in new_edges if edge_turn(ev["m"], x, y) > 90]
            ach = sum(1 for k, v in base["area"].items() if v > 0 and k in ev["area"] and abs(ev["area"][k] - v) / v > 0.05)
            small = sum(1 for k, v in ev["area"].items() if v < 1.0 and base["area"].get(k, 0) >= 1.0)
            collapsed = sum(1 for ll in ev["m"].laneletLayer if len({(round(p.x, 3), round(p.y, 3)) for p in ll.centerline}) < 2)
            print(f"\n### mode={mode} tol={tol:g}m -> {out.name}")
            print(f"병합된 점 {len(rep)}(대표점으로 바뀐 점 수), 지운 node {removed}, 2점 미만 linestring {deg}")
            summary(f"{mode} {tol:g}", ev, rt)
            print(f"새 연결 간선 {len(new_edges)}, 사라진 간선 {len(lost_edges)}, 새 간선 중 방향 꺾임 90도 넘음 {len(sharp)}"
                  + (f" 예 {sharp[:5]}" if sharp else ""))
            print(f"병합 이동 거리 분포 {om['dist']}, 1mm 넘게 옮겨진 점 {om['moved']}")
            print(f"과병합: 다른 road 끼리 병합 {om['diff_road']}, 그중 xodr 링크·같은 junction 아닌 것 {om['unlinked']}"
                  f"(1mm 넘게 옮김 {om['moved_unlinked']})"
                  + (f" 옮긴 것 예(점, 대표, road, road, 거리) {om['ex_unlinked']}" if om['ex_unlinked'] else ""))
            print(f"과병합: 반대 방향 쌍 있는 병합 {om['opp']}, 같은 방향 쌍 없이 반대 방향만 {om['opp_only']}"
                  f"(1mm 넘게 옮김 {om['moved_opp_only']})"
                  + (f" 예 {om['ex_opp']}" if om['ex_opp'] else "") + f", 끝점 아닌 점이 낀 병합 {om['interior']}")
            print(f"과병합: lanelet 면적 5% 넘게 변함 {ach}, 면적 1m^2 미만으로 줄어듦 {small}, 중심선이 점 하나로 무너짐 {collapsed}")
            results[(mode, tol)] = (ev, rt, om, len(rep), len(sharp), ach, small, deg)
            (work / f"s30_dead_{mode}_{tol:g}.json").write_text(json.dumps(sorted(ev["dead"])))

    # Phase 3: 남은 끊김 분류 (원본의 뒤 없음 201 과 비교)
    print("\n## 남은 끊김 분류")
    starts0 = [(ll.id, ll.leftBound[0], ll.rightBound[0]) for ll in m0.laneletLayer if rules.canPass(ll)]

    def near_start(ll):
        le, re_ = ll.leftBound[-1], ll.rightBound[-1]
        return min((max(math.hypot(le.x - ls.x, le.y - ls.y), math.hypot(re_.x - rs.x, re_.y - rs.y)), sid)
                   for sid, ls, rs in starts0 if sid != ll.id)

    cat0 = {}
    for lid in base["dead"]:
        d, _ = near_start(m0.laneletLayer[lid])
        cat0[lid] = "same" if d < 0.001 else "mid" if d < 3 else "far"
    s29_47 = {k for k, v in cat0.items() if v != "same"}
    print(f"원본 뒤 없음 {len(cat0)}: 같은 자리(<0.001m) {sum(v == 'same' for v in cat0.values())},"
          f" 0.001~3m {sum(v == 'mid' for v in cat0.values())}, 3m 이상 {sum(v == 'far' for v in cat0.values())}")

    def describe(lid):
        ll = m0.laneletLayer[lid]
        rs = sorted(road_of.get(lid, ()))
        end = ll.centerline[len(ll.centerline) - 1]
        last = min(rs, key=lambda r: math.hypot(samples[(r, -1)][2][0] - end.x, samples[(r, -1)][2][1] - end.y)) if rs and all((r, -1) in samples for r in rs) else None
        d, cand = near_start(ll)
        return dict(roads=rs, last=last, junc=junc_of.get(last, None) if last else None,
                    succ=succ_type.get(last) if last else None, uturn=bool(set(rs) & uturn), d=round(d, 3), cand=cand)

    for (mode, tol), (ev, *_rest) in results.items():
        left = set(ev["dead"])
        print(f"\n### mode={mode} tol={tol:g}: 남은 뒤 없음 {len(left)}, 원본 47 중 남음 {len(left & s29_47)},"
              f" 원본 같은 자리 154 중 남음 {len(left & (set(cat0) - s29_47))}, 새로 생김 {len(left - set(cat0))}")
        if tol not in (0.05, 0.3, 1.0):
            continue
        cnt = collections.Counter()
        for lid in sorted(left):
            dsc = describe(lid)
            in_j = dsc["junc"] not in (None, -1)
            k = ("xodr 끝 successor 없음" if dsc["succ"] is None else
                 "회차 연결로" if dsc["uturn"] else
                 "교차로 커넥터" if in_j else "일반 road") + (" / 원본 47" if lid in s29_47 else " / 원본 154" if lid in cat0 else " / 새로")
            cnt[k] += 1
            if tol == 0.05 and mode == "ends":
                print(f"  ll {lid} roads {dsc['roads']} 끝 road r{dsc['last']} junction {dsc['junc']} successor {dsc['succ']}"
                      f" 회차 {dsc['uturn']} 다음 시작까지 {dsc['d']}m(ll {dsc['cand']}) [{k}]")
        print(f"  분류 {dict(cnt)}")
    print(f"\nfrozen_v1 sha {sha16(FROZEN)}")


if __name__ == "__main__":
    main()
