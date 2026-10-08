#!/usr/bin/env python3
# 맵세션29 Phase 3: crdesigner 변환 결과 평가 (읽기전용). 시험 venv 로 실행(~/lanelet2_trial_venv/bin/python, pyproj 필요)
# 입력: <작업 디렉터리> 안 s29_<꼬리>.cr.xml / s29_<꼬리>.osm, xodr_lanes.csv(s29_xodr_lanes.py), 회차 목록(s28_turnaround_removed.json)
# A Lanelet2 OSM 집계: 요소 수, 태그 키, ele, local_x, turn_direction, 규제 요소, 위경도 범위
# B Lanelet2 연결: 경계 끝 노드 공유로 앞뒤 lanelet 찾기, 앞·뒤 없는 lanelet
# C xodr 차선 <-> CommonRoad lanelet 대응(중심선 표본 3점 거리), 반쪽 도로 쌍, 회차 연결로, 교차로
# D 위치: Lanelet2 노드 위경도와 실제 위경도(xodr - 오프셋 -> TM 역변환) 차
# 사용: python s29_lanelet2_eval.py <작업 디렉터리> <꼬리> [<꼬리> ...]
import collections
import csv
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from pyproj import Transformer

GEO = "+proj=tmerc +lat_0=36.627298 +lon_0=127.456394 +ellps=WGS84"   # frozen_v1 header geoReference
OFF_X, OFF_Y = 521.51, 493.61       # xodr = TM + 오프셋 (s19_scope_common)
LOGS = Path("/home/gyumin/campus_mobility_sim/map/docs/logs")
TOL = 0.5                           # 차선 중심 표본 -> lanelet 중심선 허용 거리(3857 m)


def seg_d(p, a, b):
    ax, ay = a
    bx, by = b
    L2 = (bx - ax) ** 2 + (by - ay) ** 2
    t = 0 if L2 == 0 else max(0, min(1, ((p[0] - ax) * (bx - ax) + (p[1] - ay) * (by - ay)) / L2))
    return math.hypot(p[0] - ax - t * (bx - ax), p[1] - ay - t * (by - ay))


def poly_d(p, pts):
    return min(seg_d(p, pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def osm_eval(path):
    print(f"\n## A. {path.name} ({path.stat().st_size}B)")
    root = ET.parse(str(path)).getroot()
    nodes, ways, rels = root.findall("node"), root.findall("way"), root.findall("relation")
    print(f"node {len(nodes)} / way {len(ways)} / relation {len(rels)}")
    tg = lambda e: {t.get("k"): t.get("v") for t in e.findall("tag")}  # noqa: E731
    nk = collections.Counter(k for n in nodes for k in tg(n))
    print(f"node 태그 키: {dict(nk)}")
    ele = [tg(n).get("ele") for n in nodes]
    have = [e for e in ele if e is not None]
    print(f"ele 태그 있는 node {len(have)}/{len(nodes)}, 값 종류 {collections.Counter(have).most_common(5)}")
    print(f"way 태그: {collections.Counter((k, v) for w in ways for k, v in tg(w).items() if k in ('type', 'subtype', 'lane_change')).most_common(10)}")
    rt = collections.Counter((tg(r).get("type"), tg(r).get("subtype")) for r in rels)
    print(f"relation type/subtype: {dict(rt)}")
    rk = collections.Counter(k for r in rels for k in tg(r))
    print(f"relation 태그 키: {dict(rk)}")
    lan = [r for r in rels if tg(r).get("type") == "lanelet"]
    print(f"lanelet {len(lan)}, turn_direction {sum('turn_direction' in tg(r) for r in lan)}, speed_limit {sum('speed_limit' in tg(r) for r in lan)},"
          f" one_way 값 {collections.Counter(tg(r).get('one_way') for r in lan)}")
    print(f"regulatory_element {sum(tg(r).get('type') == 'regulatory_element' for r in rels)}")
    lat = [float(n.get("lat")) for n in nodes]
    lon = [float(n.get("lon")) for n in nodes]
    print(f"위도 {min(lat):.6f}~{max(lat):.6f} / 경도 {min(lon):.6f}~{max(lon):.6f}")
    # 연결: lanelet 의 left/right way 끝 노드
    wn = {w.get("id"): [nd.get("ref") for nd in w.findall("nd")] for w in ways}
    ends = {}
    for r in lan:
        m = {x.get("role"): x.get("ref") for x in r.findall("member") if x.get("type") == "way"}
        if "left" not in m or "right" not in m:
            continue
        L, R = wn[m["left"]], wn[m["right"]]
        ends[r.get("id")] = ((L[0], R[0]), (L[-1], R[-1]), tg(r).get("subtype"))
    start_of = collections.defaultdict(list)
    for k, (s, e, st) in ends.items():
        start_of[s].append(k)
    succ = {k: [x for x in start_of.get(e, []) if x != k] for k, (s, e, st) in ends.items()}
    pred = collections.Counter(x for v in succ.values() for x in v)
    road = [k for k, v in ends.items() if v[2] != "walkway"]
    print(f"[B] 경계 끝 노드 공유 기준 연결 (walkway 제외 {len(road)} lanelet)")
    print(f"  뒤 lanelet 있음 {sum(bool(succ[k]) for k in road)}, 없음 {sum(not succ[k] for k in road)}")
    print(f"  앞 lanelet 있음 {sum(pred[k] > 0 for k in road)}, 없음 {sum(pred[k] == 0 for k in road)}")
    print(f"  뒤 lanelet 수 분포 {collections.Counter(len(succ[k]) for k in road)}")
    return nodes


def cr_eval(path, lanes, uturn, j_uturn):
    print(f"\n## C. {path.name}")
    root = ET.parse(str(path)).getroot()
    to3857 = Transformer.from_crs(GEO, "EPSG:3857", always_xy=True)
    L = {}
    for ll in root.findall("lanelet"):
        lb = [(float(p.find("x").text), float(p.find("y").text)) for p in ll.find("leftBound").findall("point")]
        rb = [(float(p.find("x").text), float(p.find("y").text)) for p in ll.find("rightBound").findall("point")]
        n = min(len(lb), len(rb))
        cen = [((lb[i][0] + rb[i][0]) / 2, (lb[i][1] + rb[i][1]) / 2) for i in range(n)]
        types = [t.text for t in ll.findall("laneletType")]
        adj = [(a.tag, a.get("drivingDir")) for a in ll if a.tag in ("adjacentLeft", "adjacentRight")]
        L[ll.get("id")] = dict(cen=cen, types=types, succ=[s.get("ref") for s in ll.findall("successor")],
                               pred=[s.get("ref") for s in ll.findall("predecessor")], adj=adj,
                               bx=(min(p[0] for p in cen), max(p[0] for p in cen), min(p[1] for p in cen), max(p[1] for p in cen)))
    tc = collections.Counter(tuple(sorted(v["types"])) for v in L.values())
    print(f"CommonRoad lanelet {len(L)}, 유형 조합 {dict(tc)}")
    print(f"앞 없음 {sum(not v['pred'] for v in L.values())}, 뒤 없음 {sum(not v['succ'] for v in L.values())}"
          f" (sidewalk 제외: 앞 없음 {sum(not v['pred'] for v in L.values() if 'sidewalk' not in v['types'])},"
          f" 뒤 없음 {sum(not v['succ'] for v in L.values() if 'sidewalk' not in v['types'])})")
    print(f"인접 표기 {collections.Counter(a for v in L.values() for a in v['adj'])}")
    # 대응
    match = {}
    for row in lanes:
        pts = [to3857.transform(float(row[f"x{k}"]) - OFF_X, float(row[f"y{k}"]) - OFF_Y) for k in ("0", "m", "1")]
        best = None
        for lid, v in L.items():
            bx = v["bx"]
            if pts[1][0] < bx[0] - 30 or pts[1][0] > bx[1] + 30 or pts[1][1] < bx[2] - 30 or pts[1][1] > bx[3] + 30:
                continue
            if len(v["cen"]) < 2:
                continue
            d = max(poly_d(p, v["cen"]) for p in pts)
            if best is None or d < best[0]:
                best = (d, lid)
        match[(int(row["road"]), int(row["lane"]))] = best
    for lane in (-1, -2):
        ks = [k for k in match if k[1] == lane]
        ok = [k for k in ks if match[k] and match[k][0] <= TOL]
        tgt = collections.Counter(match[k][1] for k in ok)
        print(f"xodr lane {lane}: {len(ks)} 중 lanelet 대응(표본 3점 모두 {TOL}m 안) {len(ok)}, 대응 안 됨 {len(ks) - len(ok)},"
              f" lanelet 하나에 xodr 차선 여러 개 {sum(1 for c in tgt.values() if c > 1)}")
        bad = sorted((k[0], round(match[k][0], 2) if match[k] else None) for k in ks if k not in ok)
        print(f"  대응 안 된 road(최근접 거리) {bad[:40]}{' ...' if len(bad) > 40 else ''}")
        if lane == -1:
            jn = {int(r["road"]): int(r["junction"]) for r in lanes}
            print(f"  대응 안 된 것 중 교차로 커넥터 {sum(jn[b[0]] != -1 for b in bad)}, 일반 road {sum(jn[b[0]] == -1 for b in bad)}")
    u_ok = [r for r in uturn if match.get((r, -1)) and match[(r, -1)][0] <= TOL]
    print(f"회차 연결로(틀 24): lanelet 대응 {len(u_ok)}/{len(uturn)}; 대응 lanelet 유형 "
          f"{collections.Counter(tuple(L[match[(r, -1)][1]]['types']) for r in u_ok)}")
    print(f"  최근접 거리 {sorted(round(match[(r, -1)][0], 2) for r in uturn if match.get((r, -1)))}")
    for r in j_uturn:
        m = match.get((r, -1))
        print(f"정문 junction 회차 r{r}: " + (f"lanelet {m[1]} 거리 {m[0]:.2f}m 유형 {L[m[1]]['types']}" if m else "없음"))
    # 교차로
    inter = root.findall("intersection")
    jn = {int(r["road"]): int(r["junction"]) for r in lanes}
    lid2road = collections.defaultdict(set)
    for (rd, ln), m in match.items():
        if m and m[0] <= TOL:
            lid2road[m[1]].add(rd)
    jcover = collections.Counter()
    for it in inter:
        ids = {x.get("ref") for x in it.iter() if x.get("ref")}
        js = {jn[rd] for i in ids for rd in lid2road.get(i, ()) if jn[rd] != -1}
        jcover[len(js)] += 1
    print(f"intersection {len(inter)}: 관련 xodr junction 수 분포 {dict(jcover)}")
    used_j = set()
    for it in inter:
        ids = {x.get("ref") for x in it.iter() if x.get("ref")}
        used_j |= {jn[rd] for i in ids for rd in lid2road.get(i, ()) if jn[rd] != -1}
    alljs = {j for j in jn.values() if j != -1}
    print(f"xodr junction(커넥터 있는) {len(alljs)} 중 intersection 에 잡힌 것 {len(used_j)}, 안 잡힌 것 {sorted(alljs - used_j)}")
    return match, L


def pos_eval(nodes_osm, lanes):
    print("\n## D. 위치 (Lanelet2 node 위경도 vs 실제)")
    tm2ll = Transformer.from_crs(GEO, "EPSG:4326", always_xy=True)
    pts = [(float(n.get("lon")), float(n.get("lat"))) for n in nodes_osm]
    row = [r for r in lanes if r["road"] == "1247" and r["lane"] == "-1"][0]
    x, y = float(row["xm"]), float(row["ym"])
    true_lon, true_lat = tm2ll.transform(x - OFF_X, y - OFF_Y)
    asif_lon, asif_lat = tm2ll.transform(x, y)   # 오프셋을 빼지 않고 geoReference 로 해석한 경우
    to_tm = Transformer.from_crs("EPSG:4326", GEO, always_xy=True)
    tx, ty = x - OFF_X, y - OFF_Y
    best = min(pts, key=lambda p: math.hypot(*[a - b for a, b in zip(to_tm.transform(*p), (tx, ty))]) if abs(p[0] - true_lon) < 3e-4 and abs(p[1] - true_lat) < 3e-4 else 1e9)
    bx, by = to_tm.transform(*best)
    print(f"정문 r1247 lane -1 중간: 실제 {true_lat:.6f}, {true_lon:.6f} / 오프셋 무시 해석 {asif_lat:.6f}, {asif_lon:.6f} (두 해석 차 {math.hypot(OFF_X, OFF_Y):.1f}m)")
    print(f"  Lanelet2 최근접 node 와 실제 점 거리 {math.hypot(bx - tx, by - ty):.2f}m (차선 중심 -> 경계 node 라 반폭 1.675m 안팎이 정상)")


def main():
    work = Path(sys.argv[1])
    lanes = list(csv.DictReader(open(work / "xodr_lanes.csv")))
    tj = json.loads((LOGS / "s28_turnaround_removed.json").read_text())
    uturn = [t["road"] for t in tj["templates"]]
    for tag in sys.argv[2:]:
        nodes = osm_eval(work / f"s29_{tag}.osm")
    cr_eval(work / f"s29_{sys.argv[2]}.cr.xml", lanes, uturn, [1915, 1916])
    pos_eval(nodes, lanes)


if __name__ == "__main__":
    main()
