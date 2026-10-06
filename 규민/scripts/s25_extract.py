#!/usr/bin/env python3
# 맵세션25: 결정용 좌표(도서관 교차로 후보, 정문 평행 way 4개)와 건물 배치 후보 목록 (읽기전용, 서버 없음).
# 원본 OSM 과 frozen_v1 xodr(carla.Map 오프라인)만 읽는다. xodr/OSM 수정 없음, 새 xodr 없음.
# 좌표 사슬: 위경도 -> 지역 TM(동 x, 북 y) -> xodr = TM + offset(521.51, 493.61) -> CARLA = (x, -y)
# 출력: 표준출력(로그), map/docs/logs/s25_buildings.csv
# 실행: .venv-carla/bin/python map/scripts/s25_extract.py > map/docs/logs/s25_extract.log
import csv
import math
import re
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, "/home/gyumin/campus_mobility_sim/map/scripts")
from s19_scope_common import (BASE, RAW_OSM, XODR_V1, load_map, road_lengths, tm,  # noqa
                              OFF_X, OFF_Y, dijkstra, path_to)
from s24_offline_checks import connectors_of, road_meta  # noqa
import carla  # noqa

LOGS = BASE / "docs/logs"
OUT_CSV = LOGS / "s25_buildings.csv"
JUNCS = {"j105": (105, "3958352374"), "j36": (36, "4400580244")}
GATE_WAYS = ["481950064", "481950063", "481950060", "481950061"]
J1, J107 = "4748296080", "4748296088"
# 주행 궤적(실제 주행 로그). 각 노선 완주 기록 1개
TRAJ = {"north": "drive_log_20260928_224708.csv", "south": "drive_log_20260928_224907.csv",
        "middle": "drive_log_20261002_012601.csv", "yangseong": "drive_log_20261002_012739.csv",
        "legacy": "drive_log_20261002_012938.csv"}
FIELD_CODES = {"N2": "법학전문대학원", "N11": "공동실험실습관", "N13": "경영대학", "N14": "인문사회관(추정, 미술관 맞은편)",
               "N15": "사회과학대학 본관", "N16-2": "미술관", "S9": "박물관"}
CODE_RE = re.compile(r"\b([NSE]\d{1,2}(?:-\d{1,2})?)\b")


def load_raw():
    nodes, ways, rels, ntags = {}, {}, {}, {}
    for _, el in ET.iterparse(str(RAW_OSM)):
        if el.tag == "node":
            nodes[el.get("id")] = (float(el.get("lat")), float(el.get("lon")))
        elif el.tag == "way":
            ways[el.get("id")] = {"nds": [n.get("ref") for n in el.findall("nd")],
                                  "tags": {t.get("k"): t.get("v") for t in el.findall("tag")}}
            el.clear()
        elif el.tag == "relation":
            rels[el.get("id")] = {"members": [(m.get("type"), m.get("ref"), m.get("role")) for m in el.findall("member")],
                                  "tags": {t.get("k"): t.get("v") for t in el.findall("tag")}}
    return nodes, ways, rels


def bearing(a, b):
    ax, ay = tm(*a)
    bx, by = tm(*b)
    return math.degrees(math.atan2(bx - ax, by - ay)) % 360


def plen(pts):
    return sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]) for i in range(len(pts) - 1))


def seg_dist(p, a, b):
    vx, vy = b[0] - a[0], b[1] - a[1]
    L2 = vx * vx + vy * vy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / L2))
    return math.hypot(a[0] + t * vx - p[0], a[1] + t * vy - p[1])


def inside(p, ring):
    x, y = p
    c = False
    for i in range(len(ring) - 1):
        (x1, y1), (x2, y2) = ring[i], ring[i + 1]
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            c = not c
    return c


def phase2(nodes, ways, cmap, meta):
    print("## 2-1. 도서관 교차로 후보")
    names = []
    for wid, w in ways.items():
        t = w["tags"]
        if "building" in t and t.get("name") and all(n in nodes for n in w["nds"]):
            ring = [tm(*nodes[n]) for n in w["nds"]]
            names.append((t["name"], wid, ring))
    for lab, (jid, node) in JUNCS.items():
        p = nodes[node]
        P = tm(*p)
        rows = connectors_of(cmap, meta, jid)
        ins = {r[2] for r in rows if r[2] is not None}
        outs = {r[3] for r in rows if r[3] is not None}
        left = [r for r in rows if -120 < r[4] <= -30]
        uturn = [r for r in rows if abs(r[4]) >= 120]
        # 커넥터 시작점 중심과 OSM 노드 투영 위치 차(역변환 검산)
        sx = sy = 0
        for r in rows:
            wp = cmap.get_waypoint_xodr(r[0], r[1], 0.01)
            sx += wp.transform.location.x
            sy += -wp.transform.location.y
        cx, cy = sx / len(rows) - OFF_X, sy / len(rows) - OFF_Y
        print(f"{lab} junction {jid} OSM 노드 {node} 위경도 {p[0]:.6f}, {p[1]:.6f}")
        print(f"  검산: 커넥터 시작점 중심과 노드 투영 위치 차 {math.hypot(cx - P[0], cy - P[1]):.1f}m")
        print(f"  연결 road: 들어오는 {len(ins)} {sorted(ins)} / 나가는 {len(outs)} {sorted(outs)} / 고유 {len(ins | outs)}")
        print(f"  커넥터 {len(rows)}: 좌회전 {len(left)} {[f'r{r[0]} {r[4]:+.1f}' for r in left]} / "
              f"유턴 {len(uturn)} {[f'r{r[0]}' for r in uturn]}")
        near = []
        for nm, wid, ring in names:
            d = 0.0 if inside(P, ring) else min(seg_dist(P, ring[i], ring[i + 1]) for i in range(len(ring) - 1))
            near.append((d, nm, wid))
        for d, nm, wid in sorted(near)[:3]:
            print(f"  가까운 건물(외곽선 거리) {nm} (way{wid}) {d:.0f}m")
    # j36 대체 경로: legacy 종점 r1278 s=100.38 을 r1564 없이
    print("\n## 2-1b. legacy 가 j36 좌회전 r1564 없이 종점(r1278 s=100.38)에 가는 경로")
    lengths = road_lengths(XODR_V1)
    from s19_scope_common import build_graph
    g = build_graph(cmap, lengths)
    uturn_all = set()
    for rid, m in meta.items():
        if m["junction"] < 0:
            continue
        a = cmap.get_waypoint_xodr(rid, -1, 0.01)
        b = cmap.get_waypoint_xodr(rid, -1, max(m["len"] - 0.01, 0))
        if a and b and abs((b.transform.rotation.yaw - a.transform.rotation.yaw + 180) % 360 - 180) >= 120:
            uturn_all.add(rid)
    with open(LOGS / "drive_plan_dry.csv") as fh:
        seq = list(dict.fromkeys(int(r["road_id"]) for r in csv.DictReader(fh)))
    print(f"legacy 계획 road 순서: {seq}")
    src = 1247
    base_d, base_p = dijkstra(g, lengths, src, banned=frozenset(uturn_all), src_rest=lengths[src][0])
    p0 = path_to(base_p, src, 1278)
    d0 = base_d.get(1278, 0) - lengths[1278][0] + 100.38
    print(f"유턴 금지 최단(정문 r1247 -> r1278 s=100.38): {d0:.1f}m {p0}")
    alt_d, alt_p = dijkstra(g, lengths, src, banned=frozenset(uturn_all | {1564}), src_rest=lengths[src][0])
    p1 = path_to(alt_p, src, 1278)
    if p1:
        d1 = alt_d[1278] - lengths[1278][0] + 100.38
        print(f"r1564 금지 + 유턴 금지: {d1:.1f}m (+{d1 - d0:.1f}m) {p1}")
        turns = []
        for r in p1:
            if meta[r]["junction"] >= 0:
                a = cmap.get_waypoint_xodr(r, -1, 0.01)
                b = cmap.get_waypoint_xodr(r, -1, meta[r]["len"] - 0.01)
                d = (b.transform.rotation.yaw - a.transform.rotation.yaw + 180) % 360 - 180
                if abs(d) >= 30:
                    turns.append(f"r{r} j{meta[r]['junction']} {d:+.1f}도 R{meta[r]['len'] / math.radians(abs(d)):.2f}")
        print(f"  회전(30도 이상): {turns}")
    else:
        print("r1564 금지 + 유턴 금지: 경로 없음")
    # j36 의 다른 좌회전 r1558 도 금지할 때(교차로 전체 좌회전 금지 가정)
    alt2_d, alt2_p = dijkstra(g, lengths, src, banned=frozenset(uturn_all | {1564, 1558}), src_rest=lengths[src][0])
    p2 = path_to(alt2_p, src, 1278)
    print(f"j36 좌회전 2개 모두 금지: {'경로 없음' if not p2 else f'{alt2_d[1278] - lengths[1278][0] + 100.38:.1f}m'}")
    print("\n## 2-2. 정문 평행 way 4개")
    A, B = tm(*nodes[J1]), tm(*nodes[J107])
    L = math.hypot(B[0] - A[0], B[1] - A[1])
    ux, uy = (B[0] - A[0]) / L, (B[1] - A[1]) / L
    print(f"기준선 j1 {nodes[J1][0]:.6f}, {nodes[J1][1]:.6f} -> j107 {nodes[J107][0]:.6f}, {nodes[J107][1]:.6f} "
          f"{L:.1f}m 방위 {bearing(nodes[J1], nodes[J107]):.1f}도")

    def offs_at(wid, s):
        pts = [tm(*nodes[n]) for n in ways[wid]["nds"]]
        sd = [((p[0] - A[0]) * ux + (p[1] - A[1]) * uy, -(p[0] - A[0]) * uy + (p[1] - A[1]) * ux) for p in pts]
        if sd[0][0] > sd[-1][0]:
            sd = sd[::-1]
        for i in range(len(sd) - 1):
            if sd[i][0] <= s <= sd[i + 1][0]:
                t = (s - sd[i][0]) / ((sd[i + 1][0] - sd[i][0]) or 1)
                return sd[i][1] + t * (sd[i + 1][1] - sd[i][1])
        return None
    S = [10 + i * 2 for i in range(15)]   # s 10~38m
    prof = {}
    for wid in GATE_WAYS:
        w = ways[wid]
        a, b = nodes[w["nds"][0]], nodes[w["nds"][-1]]
        pts = [tm(*nodes[n]) for n in w["nds"]]
        prof[wid] = [offs_at(wid, s) for s in S]
        print(f"way {wid} 노드 {len(w['nds'])}개 길이 {plen(pts):.1f}m 시작 {a[0]:.6f}, {a[1]:.6f} 끝 {b[0]:.6f}, {b[1]:.6f} "
              f"방위(시작->끝) {bearing(a, b):.1f}도")
        print(f"  태그 전부: {w['tags']}")
        print(f"  횡오프셋 s=10~38m(2m 간격, j1->j107 왼쪽 +): {[round(x, 1) for x in prof[wid]]}")
    print("쌍별 수직 거리(s=10~38m 평균 / 최소 / 최대):")
    for i in range(4):
        for j in range(i + 1, 4):
            a, b = GATE_WAYS[i], GATE_WAYS[j]
            d = [abs(x - y) for x, y in zip(prof[a], prof[b]) if x is not None and y is not None]
            print(f"  {a} - {b}: {sum(d) / len(d):.1f} / {min(d):.1f} / {max(d):.1f} m")
    for n in (J1, J107):
        print(f"노드 {n} 에 접속하는 way: " + ", ".join(
            f"{k}({w['tags'].get('highway')},{w['tags'].get('oneway', '-')})" for k, w in ways.items() if n in w["nds"]))


def building_rings(nodes, ways, rels):
    out = []
    for wid, w in ways.items():
        if "building" in w["tags"] and len(w["nds"]) >= 4 and all(n in nodes for n in w["nds"]):
            out.append(("way", wid, w["tags"], [[tm(*nodes[n]) for n in w["nds"]]]))
    for rid, r in rels.items():
        if "building" in r["tags"]:
            rings = []
            for typ, ref, role in r["members"]:
                if typ == "way" and role == "outer" and ref in ways and all(n in nodes for n in ways[ref]["nds"]):
                    rings.append([tm(*nodes[n]) for n in ways[ref]["nds"]])
            if rings:
                out.append(("relation", rid, r["tags"], rings))
    return out


def phase3(nodes, ways, rels):
    print("\n## 3. 건물 배치 후보")
    traj = {}
    for k, f in TRAJ.items():
        pts = []
        with open(LOGS / f) as fh:
            for i, r in enumerate(csv.DictReader(fh)):
                if i % 4 == 0:   # 20Hz -> 5Hz (약 1.1m 간격)
                    pts.append((float(r["x"]) - OFF_X, -float(r["y"]) - OFF_Y))
        traj[k] = pts
        print(f"궤적 {k}: {f} 점 {len(pts)}개(5Hz 표본)")
    blds = building_rings(nodes, ways, rels)
    print(f"원본 OSM building: way {sum(1 for b in blds if b[0] == 'way')}, relation {sum(1 for b in blds if b[0] == 'relation')}")
    rows = []
    for typ, oid, t, rings in blds:
        allp = [p for rg in rings for p in rg[:-1]]
        xs, ys = [p[0] for p in allp], [p[1] for p in allp]
        cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
        # 빠른 거르기: 중심이 모든 궤적에서 400m 넘으면 생략
        best = (1e9, None)
        for k, pts in traj.items():
            for q in pts:
                if abs(q[0] - cx) > 400 or abs(q[1] - cy) > 400:
                    continue
                for rg in rings:
                    if inside(q, rg):
                        d = 0.0
                    else:
                        d = min(seg_dist(q, rg[i], rg[i + 1]) for i in range(len(rg) - 1))
                    if d < best[0]:
                        best = (d, k)
        near_routes = []
        for k, pts in traj.items():
            dk = min((min(seg_dist(q, rg[i], rg[i + 1]) for rg in rings for i in range(len(rg) - 1))
                      for q in pts if abs(q[0] - cx) < 200 and abs(q[1] - cy) < 200), default=1e9)
            if dk <= 100:
                near_routes.append(f"{k}:{dk:.0f}")
        name = t.get("name", "")
        code = ""
        for src in (t.get("ref", ""), name, t.get("addr:housenumber", ""), t.get("name:en", "")):
            m = CODE_RE.search(src or "")
            if m:
                code = m.group(1)
                break
        d = best[0]
        tier = 1 if d <= 50 else (2 if d <= 100 else 0)
        rows.append({"osm_type": typ, "osm_id": oid, "name": name, "code": code, "ref": t.get("ref", ""),
                     "building": t.get("building", ""), "levels": t.get("building:levels", ""), "height_m": t.get("height", ""), "layer": t.get("layer", ""),
                     "tm_x": f"{cx:.1f}", "tm_y": f"{cy:.1f}",
                     "xodr_x": f"{cx + OFF_X:.1f}", "xodr_y": f"{cy + OFF_Y:.1f}",
                     "carla_x": f"{cx + OFF_X:.1f}", "carla_y": f"{-(cy + OFF_Y):.1f}",
                     "bbox_ew_m": f"{max(xs) - min(xs):.1f}", "bbox_ns_m": f"{max(ys) - min(ys):.1f}",
                     "min_dist_m": f"{d:.1f}" if d < 1e8 else "", "nearest_route": best[1] or "",
                     "routes_within_100m": " ".join(near_routes), "tier": tier,
                     "route_inside": " ".join(sorted({k for k, pts in traj.items() for q in pts
                                                      if any(inside(q, rg) for rg in rings)}))})
    rows.sort(key=lambda r: (r["tier"] == 0, r["tier"], float(r["min_dist_m"] or 1e9)))
    with open(OUT_CSV, "w", newline="") as fh:
        wr = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        wr.writeheader()
        wr.writerows(rows)
    t1 = [r for r in rows if r["tier"] == 1]
    t2 = [r for r in rows if r["tier"] == 2]
    print(f"전체 {len(rows)}, 1순위(0~50m) {len(t1)}, 2순위(50~100m) {len(t2)}, 제외 {len(rows) - len(t1) - len(t2)}")
    print(f"1순위 이름 있음 {sum(1 for r in t1 if r['name'])} / 없음 {sum(1 for r in t1 if not r['name'])}")
    print(f"2순위 이름 있음 {sum(1 for r in t2 if r['name'])} / 없음 {sum(1 for r in t2 if not r['name'])}")
    print(f"1순위 levels 태그 있음 {sum(1 for r in t1 if r['levels'])}")
    for lab, lst in (("1순위", t1), ("2순위", t2)):
        print(f"### {lab}")
        for r in lst:
            print(f"  {r['osm_type']}{r['osm_id']} {r['name'] or '(이름없음)'} code={r['code'] or '-'} "
                  f"{r['bbox_ew_m']}x{r['bbox_ns_m']}m 층={r['levels'] or '-'} 거리 {r['min_dist_m']}m({r['nearest_route']}) "
                  f"TM {r['tm_x']},{r['tm_y']} 노선 {r['routes_within_100m']}")
    for r in rows:
        if r["route_inside"]:
            print(f"주의: 궤적이 건물 폴리곤 안을 지남 {r['osm_type']}{r['osm_id']} {r['name'] or '(이름없음)'} "
                  f"노선 {r['route_inside']} layer={r['layer'] or '-'}")
    codes = {}
    for r in rows:
        if r["code"]:
            codes.setdefault(r["code"], []).append(r["name"])
    for c, nm in codes.items():
        if len(nm) > 1:
            print(f"주의: 같은 코드 {c} 건물 {len(nm)}개 {nm}")
    print("\n## 3-4. 현장 확인 코드 대조")
    for code, desc in FIELD_CODES.items():
        hit = [r for r in rows if r["code"] == code]
        if hit:
            for r in hit:
                print(f"{code} 현장 '{desc}' -> OSM {r['osm_type']}{r['osm_id']} '{r['name']}' 순위 {r['tier']} 거리 {r['min_dist_m']}m")
        else:
            print(f"{code} 현장 '{desc}' -> OSM 에 코드 {code} 없음")
    return rows


def main():
    nodes, ways, rels = load_raw()
    cmap = load_map(XODR_V1)
    meta = road_meta()
    print(f"map: {XODR_V1.name}")
    phase2(nodes, ways, cmap, meta)
    phase3(nodes, ways, rels)


if __name__ == "__main__":
    main()
