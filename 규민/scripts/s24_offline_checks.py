#!/usr/bin/env python3
# 맵세션24: 현장 답사 반영 오프라인 검증 (읽기전용, 서버 없음, xodr/OSM 미수정, 새 xodr 생성 없음).
# 원본 OSM / 변환 입력 OSM / frozen_v1 xodr(carla.Map 오프라인)만 읽는다.
# 출력: 표준출력(key: value 줄). s24_reconv_list.md, s24_offline_checks.md 의 근거 자료
# 실행: .venv-carla/bin/python map/scripts/s24_offline_checks.py > map/docs/logs/s24_offline_checks.log
import csv
import math
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, "/home/gyumin/campus_mobility_sim/map/scripts")
from s19_scope_common import (BASE, RAW_OSM, CONV_OSM, XODR_V1, load_osm, load_map,  # noqa
                              ll_to_carla, tm)
from classify_internal import load_polygon_rings, point_in_polygon  # noqa
import carla  # noqa

POLY = BASE / "data/processed/cbnu_relation_polygon.json"
LOGS = BASE / "docs/logs"
ROUTES = {"legacy": "drive_plan_dry.csv", "north": "drive_plan_dry_north.csv",
          "south": "drive_plan_dry_south.csv", "middle": "drive_plan_dry_middle.csv",
          "yangseong": "drive_plan_dry_yangseong.csv", "uturn": "drive_plan_dry_uturn.csv"}
# 세션4 제외 8 + s23 재변환 9 (s23_scope_final.md)
EXCL8 = ["481950064", "481950061", "442710666", "442710667", "442595035", "471667693",
         "442595850", "442709215"]
RECONV9 = ["481945510", "481943505", "481475019", "452870644", "481945509", "481943503",
           "446440352", "446440353", "481950060"]
GATE_PAR = ["481950063", "481950064", "481950060", "481950061"]   # j1 - j107 사이 일방통행 4개
J1, J107 = "4748296080", "4748296088"
MIDDLE_NODE = "4437127634"
MIDDLE_GATE_EST = (36.633680, 127.458398)   # middle_gate_deadends.md 추정점(OSM 문 지물 없음)
STOPS = {"북문": (1378, 78.0), "남문": (1237, 47.0), "양성재": (1123, 22.0), "중문": (1327, 70.0)}
BLD = {"중앙도서관 신관": (36.628238, 127.458035), "중앙도서관 구관": (36.628527, 127.457535),
       "개신문화관": (36.628217, 127.459391), "양성재": (36.627604, 127.452295),
       "인문사회관 N14": (36.631082, 127.456839)}
UNPAVED = {"ground", "unpaved", "dirt", "gravel", "fine_gravel", "compacted", "earth", "mud", "sand", "grass"}


def dist_ll(a, b):
    ax, ay = tm(*a)
    bx, by = tm(*b)
    return math.hypot(ax - bx, ay - by)


def way_len(nodes, w):
    p = [tm(*nodes[n]) for n in w["nds"] if n in nodes]
    return sum(math.hypot(p[i + 1][0] - p[i][0], p[i + 1][1] - p[i][1]) for i in range(len(p) - 1))


def inside_frac(nodes, w, rings):
    ns = [n for n in w["nds"] if n in nodes]
    return sum(point_in_polygon(nodes[n][1], nodes[n][0], rings) for n in ns) / max(len(ns), 1)


def fmt(p):
    return f"{p[0]:.6f}, {p[1]:.6f}"


def tags_s(t):
    keys = ["highway", "service", "oneway", "surface", "footway", "crossing", "bicycle",
            "motor_vehicle", "access", "barrier", "bridge", "lanes", "width", "name"]
    return " ".join(f"{k}={t[k]}" for k in keys if k in t) or "(태그 없음)"


def road_meta():
    root = ET.parse(str(XODR_V1)).getroot()
    out = {}
    for r in root.iter("road"):
        link = r.find("link")
        pre = suc = None
        if link is not None:
            if link.find("predecessor") is not None:
                pre = (link.find("predecessor").get("elementType"), link.find("predecessor").get("elementId"))
            if link.find("successor") is not None:
                suc = (link.find("successor").get("elementType"), link.find("successor").get("elementId"))
        out[int(r.get("id"))] = {"len": float(r.get("length")), "junction": int(r.get("junction")),
                                 "name": r.get("name"), "pre": pre, "suc": suc}
    return out


def route_roads():
    res = {}
    for k, f in ROUTES.items():
        with open(LOGS / f) as fh:
            res[k] = list(dict.fromkeys(int(r["road_id"]) for r in csv.DictReader(fh)))
    return res


def yaw_diff(a, b):
    return (b - a + 180) % 360 - 180


def connectors_of(cmap, meta, jid):
    """junction jid 의 커넥터 (road, lane) 별 진입 road, 진출 road, 회전각, 길이, 평균R."""
    rows = []
    for rid, m in meta.items():
        if m["junction"] != jid:
            continue
        for lane in (-1, 1, -2, 2):
            a = cmap.get_waypoint_xodr(rid, lane, 0.01)
            b = cmap.get_waypoint_xodr(rid, lane, max(m["len"] - 0.01, 0.0))
            if a is None or b is None or a.lane_type != carla.LaneType.Driving:
                continue
            if lane > 0:   # 양수 lane 은 s 역방향 주행
                a, b = b, a
            prv = [p.road_id for p in a.previous(0.5) if p.road_id != rid]
            nxt = [p.road_id for p in b.next(0.5) if p.road_id != rid]
            d = yaw_diff(a.transform.rotation.yaw, b.transform.rotation.yaw)
            ad = abs(d)
            kind = "U턴" if ad >= 135 else ("직진" if ad < 30 else ("우회전" if d > 0 else "좌회전"))
            R = m["len"] / math.radians(ad) if ad >= 20 else float("inf")
            rows.append((rid, lane, prv[0] if prv else None, nxt[0] if nxt else None, d, kind, m["len"], R))
    return rows


def main():
    nodes, ways = load_osm(RAW_OSM)
    cnodes, cways = load_osm(CONV_OSM)
    rings = load_polygon_rings(POLY)
    meta = road_meta()
    routes = route_roads()
    cmap = load_map(XODR_V1)
    print(f"map: {XODR_V1.name}")
    print(f"roads: {len(meta)} junction_connectors: {sum(1 for m in meta.values() if m['junction'] >= 0)}")
    print("routes: " + ", ".join(f"{k}={len(v)}road" for k, v in routes.items()))

    def conv_status(wid):
        if wid not in cways:
            return "변환입력에 없음"
        return f"변환입력 highway={cways[wid]['tags'].get('highway')}"

    # ---- 변환되는 highway 종류(기하로 확인) ----
    print("\n## T. 변환 입력 highway 종류별 맵 포함 여부 (way 점이 Driving 차선 2.5m 안 + 방향 30도 안 비율)")

    def on_road_frac(nds, nd_src):
        pts = [nd_src[n] for n in nds if n in nd_src]
        samp = []
        for i in range(len(pts) - 1):
            a, b = ll_to_carla(*pts[i]), ll_to_carla(*pts[i + 1])
            seg = math.hypot(b[0] - a[0], b[1] - a[1])
            k = max(int(seg // 2.0), 1)
            hd = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
            for j in range(k):
                t = (j + 0.5) / k
                samp.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), hd))
        if not samp:
            return 0.0, 0
        ok = 0
        for x, y, hd in samp:
            wp = cmap.get_waypoint(carla.Location(x=x, y=y, z=0), project_to_road=True,
                                   lane_type=carla.LaneType.Driving)
            if wp is None:
                continue
            loc = wp.transform.location
            dd = abs(yaw_diff(wp.transform.rotation.yaw, hd)) % 180
            dd = min(dd, 180 - dd)
            if math.hypot(loc.x - x, loc.y - y) <= 2.5 and dd <= 30:
                ok += 1
        return ok / len(samp), len(samp)

    by_type = {}
    for wid, w in cways.items():
        h = w["tags"].get("highway")
        if h is None:
            continue
        f, n = on_road_frac(w["nds"], cnodes)
        by_type.setdefault(h, []).append((wid, f, n))
    for h, lst in sorted(by_type.items()):
        hi = sum(1 for _, f, _ in lst if f >= 0.8)
        print(f"type {h}: way {len(lst)}, 80%이상 겹침 {hi}, 평균 {sum(f for _, f, _ in lst) / len(lst):.2f}")
    onroad = {wid: f for lst in by_type.values() for wid, f, _ in lst}

    # ---- Phase 2 목록 ----
    print("\n## P2. 세션4 제외 8 + s23 재변환 9 대조")
    allw = list(dict.fromkeys(EXCL8 + RECONV9))
    print(f"합집합 {len(allw)} (중복 ID {len(EXCL8) + len(RECONV9) - len(allw)})")
    for wid in allw:
        w = ways[wid]
        a, b = nodes[w["nds"][0]], nodes[w["nds"][-1]]
        grp = "세션4제외8" if wid in EXCL8 else "s23재변환9"
        print(f"way {wid} [{grp}] {tags_s(w['tags'])} | {way_len(nodes, w):.1f}m | 안쪽 {inside_frac(nodes, w, rings) * 100:.0f}%"
              f" | 시작 {fmt(a)} 끝 {fmt(b)} | {conv_status(wid)} | 맵겹침 {onroad.get(wid, 0):.2f}")

    print("\n## P2-정문. j1 - j107 사이 평행 way (j1->j107 방향 기준 왼쪽 +, 오른쪽 -)")
    A, B = tm(*nodes[J1]), tm(*nodes[J107])
    L = math.hypot(B[0] - A[0], B[1] - A[1])
    ux, uy = (B[0] - A[0]) / L, (B[1] - A[1]) / L
    print(f"j1 {fmt(nodes[J1])} -> j107 {fmt(nodes[J107])} 직선 {L:.1f}m")
    for wid in GATE_PAR:
        w = ways[wid]
        offs = []
        for n in w["nds"]:
            p = tm(*nodes[n])
            s = (p[0] - A[0]) * ux + (p[1] - A[1]) * uy
            d = -(p[0] - A[0]) * uy + (p[1] - A[1]) * ux
            if 10 <= s <= 38:
                offs.append(d)
        dirn = "진입(j1->j107)" if w["nds"][0] == J1 else "진출(j107->j1)"
        print(f"way {wid} {dirn} oneway={w['tags'].get('oneway')} {way_len(nodes, w):.1f}m 중간부 횡오프셋 평균 "
              f"{sum(offs) / len(offs):+.1f}m | {conv_status(wid)} | 맵겹침 {onroad.get(wid, 0):.2f}")
    for wid in ways:
        if J1 in ways[wid]["nds"] and wid not in GATE_PAR:
            w = ways[wid]
            print(f"j1 접속 way {wid} {tags_s(w['tags'])} {way_len(nodes, w):.1f}m {conv_status(wid)}")

    # ---- 회전 제한 관계 ----
    print("\n## R. OSM 회전 제한 relation (원본)")
    root = ET.parse(str(RAW_OSM)).getroot()
    nrel = 0
    for rel in root.iter("relation"):
        t = {x.get("k"): x.get("v") for x in rel.findall("tag")}
        if t.get("type") != "restriction":
            continue
        nrel += 1
        mem = [(m.get("type"), m.get("ref"), m.get("role")) for m in rel.findall("member")]
        via = [m for m in mem if m[2] == "via"]
        vtxt = ""
        if via and via[0][0] == "node" and via[0][1] in nodes:
            p = nodes[via[0][1]]
            ins = point_in_polygon(p[1], p[0], rings)
            nearest = min(BLD.items(), key=lambda kv: dist_ll(p, kv[1]))
            vtxt = f"via {fmt(p)} 안쪽={ins} 가장가까운 건물 {nearest[0]} {dist_ll(p, nearest[1]):.0f}m"
        fr = [m[1] for m in mem if m[2] == "from"]
        to = [m[1] for m in mem if m[2] == "to"]
        fn = ways.get(fr[0], {}).get("tags", {}).get("name") if fr else None
        tn = ways.get(to[0], {}).get("tags", {}).get("name") if to else None
        print(f"relation {rel.get('id')} {t.get('restriction')} from {fr}({fn}) to {to}({tn}) {vtxt}")
    print(f"원본 restriction relation {nrel}개, 변환 입력 relation "
          f"{sum(1 for _ in ET.parse(str(CONV_OSM)).getroot().iter('relation'))}개")

    # ---- 3-1 도서관 부근 교차로 ----
    print("\n## 3-1. 도서관 부근 junction 의 연결로")
    jpos = {}
    for rid, m in meta.items():
        if m["junction"] >= 0:
            wp = cmap.get_waypoint_xodr(rid, -1, m["len"] / 2) or cmap.get_waypoint_xodr(rid, 1, m["len"] / 2)
            if wp:
                jpos.setdefault(m["junction"], []).append((wp.transform.location.x, wp.transform.location.y))
    jc = {j: (sum(p[0] for p in v) / len(v), sum(p[1] for p in v) / len(v)) for j, v in jpos.items()}
    lib = BLD["중앙도서관 신관"]
    mid = ((lib[0] + BLD["개신문화관"][0]) / 2, (lib[1] + BLD["개신문화관"][1]) / 2)
    print(f"도서관 신관-개신문화관 중점 {fmt(mid)} (두 건물 중심 간 {dist_ll(lib, BLD['개신문화관']):.0f}m)")
    cand = set()
    for nm, (rid, s) in list(STOPS.items())[:2]:
        wp = cmap.get_waypoint_xodr(rid, -1, s)
        x, y = wp.transform.location.x, wp.transform.location.y
        near = sorted(jc.items(), key=lambda kv: math.hypot(kv[1][0] - x, kv[1][1] - y))[:2]
        print(f"정차점 {nm} r{rid} s={s} -> 가까운 junction: " +
              ", ".join(f"j{j} {math.hypot(c[0] - x, c[1] - y):.0f}m" for j, c in near))
        cand.update(j for j, _ in near[:1])
    mx, my = ll_to_carla(*mid)
    near = sorted(jc.items(), key=lambda kv: math.hypot(kv[1][0] - mx, kv[1][1] - my))[:3]
    print("두 건물 중점 -> 가까운 junction: " + ", ".join(f"j{j} {math.hypot(c[0] - mx, c[1] - my):.0f}m" for j, c in near))
    cand.update(j for j, _ in near[:2])
    for j in sorted(cand):
        rows = connectors_of(cmap, meta, j)
        names = sorted({meta[r[0]]["name"] for r in rows})
        used = {k: [r[0] for r in rows if r[0] in v] for k, v in routes.items()}
        used = {k: v for k, v in used.items() if v}
        print(f"junction j{j} 커넥터 {len(rows)} (이름 {names}) 사용 노선 {used}")
        for r in sorted(rows, key=lambda r: (r[2] or 0, r[3] or 0)):
            Rt = f"{r[7]:.2f}" if r[7] != float("inf") else "-"
            print(f"  r{r[0]} lane{r[1]:+d} r{r[2]} -> r{r[3]} {r[5]} {r[4]:+.1f}도 길이 {r[6]:.2f}m 평균R {Rt}")

    # ---- 3-1b 도서관·개신문화관 250m 안 junction 전체와 only_u_turn via 노드 ----
    print("\n## 3-1b. 도서관 신관·개신문화관 250m 안 junction (U턴 = 회전 120도 이상)")
    VIA = "3958352374"
    vp = nodes[VIA]
    print(f"only_u_turn via 노드 {VIA} {fmt(vp)} 접속 way: " + ", ".join(
        f"{k}({tags_s(x['tags'])}; {conv_status(k)})" for k, x in ways.items() if VIA in x["nds"]))
    vx, vy = ll_to_carla(*vp)
    gx, gy = ll_to_carla(*BLD["개신문화관"])
    lx, ly = ll_to_carla(*lib)
    for j, (x, y) in sorted(jc.items(), key=lambda kv: math.hypot(kv[1][0] - vx, kv[1][1] - vy)):
        dl, dg, dv = math.hypot(x - lx, y - ly), math.hypot(x - gx, y - gy), math.hypot(x - vx, y - vy)
        if min(dl, dg) > 250:
            continue
        rows = connectors_of(cmap, meta, j)
        ins = {r[2] for r in rows}
        n_left = sum(1 for r in rows if -120 < r[4] <= -30)
        n_right = sum(1 for r in rows if 30 <= r[4] < 120)
        n_u = sum(1 for r in rows if abs(r[4]) >= 120)
        used = sorted(k for k, v in routes.items() if any(r[0] in v for r in rows))
        nm = sorted({meta[r[0]]["name"].lstrip(":").split("_")[0] for r in rows})
        print(f"j{j} 노드 {nm} 도서관신관 {dl:.0f}m 개신문화관 {dg:.0f}m via {dv:.0f}m | 진입 road {len(ins)} "
              f"커넥터 {len(rows)} 좌 {n_left} 우 {n_right} U {n_u} | 노선 {used}")
        for r in sorted(rows, key=lambda r: r[0]):
            if abs(r[4]) >= 30:
                print(f"    r{r[0]} r{r[2]} -> r{r[3]} {r[4]:+.1f}도 {r[6]:.2f}m R {r[7]:.2f}"
                      f"{' [노선:' + ','.join(k for k, v in routes.items() if r[0] in v) + ']' if any(r[0] in v for v in routes.values()) else ''}")

    # ---- 3-2 r1786 ----
    print("\n## 3-2. 평균R 5.5m road")
    m = meta[1786]
    print(f"r1786 name={m['name']} junction={m['junction']} length={m['len']:.3f}")
    rows = connectors_of(cmap, meta, m["junction"])
    for r in rows:
        if r[0] == 1786:
            print(f"  r1786 lane{r[1]:+d} r{r[2]} -> r{r[3]} {r[5]} {r[4]:+.1f}도 길이 {r[6]:.2f}m 평균R {r[7]:.2f}")
    node = m["name"].lstrip(":").split("_")[0]
    if node in nodes:
        p = nodes[node]
        deg = sum(1 for w in ways.values() if node in w["nds"])
        print(f"  junction 노드 {node} {fmt(p)} 원본 OSM 에서 접속 way {deg}개")
        for wid, w in ways.items():
            if node in w["nds"]:
                print(f"    way {wid} {tags_s(w['tags'])} {conv_status(wid)}")
    print(f"j{m['junction']} 커넥터 {len(rows)}개:")
    for r in sorted(rows, key=lambda r: r[0]):
        Rt = f"{r[7]:.2f}" if r[7] != float("inf") else "-"
        print(f"  r{r[0]} lane{r[1]:+d} r{r[2]} -> r{r[3]} {r[5]} {r[4]:+.1f}도 {r[6]:.2f}m R {Rt}")
    print("r1786 사용 노선: " + (", ".join(k for k, v in routes.items() if 1786 in v) or "없음"))
    # 평균R 9m 미만 일반 road(junction 아님) 가 있는가 — 같은 지표로
    sharp_plain = []
    for rid, mm in meta.items():
        if mm["junction"] >= 0 or mm["len"] < 3:
            continue
        a = cmap.get_waypoint_xodr(rid, -1, 0.01) or cmap.get_waypoint_xodr(rid, 1, 0.01)
        b = cmap.get_waypoint_xodr(rid, -1, mm["len"] - 0.01) or cmap.get_waypoint_xodr(rid, 1, mm["len"] - 0.01)
        if a is None or b is None:
            continue
        d = abs(yaw_diff(a.transform.rotation.yaw, b.transform.rotation.yaw))
        if d >= 20 and mm["len"] / math.radians(d) < 9.0:
            sharp_plain.append((rid, mm["len"], d, mm["len"] / math.radians(d)))
    print(f"일반 road 중 평균R 9m 미만(회전 20도 이상): {len(sharp_plain)}개 " +
          ", ".join(f"r{r} {L:.1f}m {d:.0f}도 R{R:.2f}" for r, L, d, R in sorted(sharp_plain, key=lambda x: x[3])))

    # ---- 3-3 비포장 ----
    print("\n## 3-3. 비포장 surface")
    for src, nd, ws in (("원본", nodes, ways), ("변환입력", cnodes, cways)):
        hits = [(wid, w) for wid, w in ws.items() if w["tags"].get("surface") in UNPAVED]
        print(f"{src}: 비포장 surface way {len(hits)}")
        for wid, w in hits:
            ps = [nd[n] for n in w["nds"] if n in nd]
            c = (sum(p[0] for p in ps) / len(ps), sum(p[1] for p in ps) / len(ps)) if ps else (0, 0)
            print(f"  way {wid} {tags_s(w['tags'])} 중심 {fmt(c)} 양성재까지 {dist_ll(c, BLD['양성재']):.0f}m "
                  f"| {conv_status(wid)} | 맵겹침 {onroad.get(wid, 0):.2f}")
    surf_unc = [wid for wid, w in cways.items() if w["tags"].get("highway") == "unclassified" and "surface" in w["tags"]]
    print(f"변환입력 unclassified 중 surface 태그 있는 way: {len(surf_unc)}")
    print("양성재 부근 테니스장(원본 leisure=pitch sport=tennis):")
    for wid, w in ways.items():
        if w["tags"].get("sport") != "tennis":
            continue
        ps = [nodes[n] for n in w["nds"] if n in nodes]
        c = (sum(p[0] for p in ps) / len(ps), sum(p[1] for p in ps) / len(ps))
        dy = dist_ll(c, BLD["양성재"])
        if dy > 400:
            continue
        cx, cy = ll_to_carla(*c)
        wp = cmap.get_waypoint(carla.Location(x=cx, y=cy, z=0), project_to_road=True, lane_type=carla.LaneType.Driving)
        loc = wp.transform.location
        ondr = [k for k, v in routes.items() if wp.road_id in v]
        print(f"  pitch {wid} 중심 {fmt(c)} 양성재까지 {dy:.0f}m 최근접 차선 r{wp.road_id} "
              f"{math.hypot(loc.x - cx, loc.y - cy):.0f}m (그 road 지나는 노선: {ondr or '없음'})")
        # 테니스장 30m 안 원본 way 중 차량용 태그
        for wid2, w2 in ways.items():
            h = w2["tags"].get("highway")
            if h is None or wid2 == wid:
                continue
            if any(n in nodes and dist_ll(nodes[n], c) < 40 for n in w2["nds"]):
                print(f"    40m 안 way {wid2} {tags_s(w2['tags'])} | {conv_status(wid2)} | 맵겹침 {onroad.get(wid2, 0):.2f}")

    print("테니스장 40m 안 변환 way 가 맵에서 어느 road 이고 노선이 지나는가:")
    for wid in ["442708402", "442708405", "472252989", "472252990"]:
        rids = set()
        for n in cways[wid]["nds"]:
            if n not in cnodes:
                continue
            x, y = ll_to_carla(*cnodes[n])
            wp = cmap.get_waypoint(carla.Location(x=x, y=y, z=0), project_to_road=True, lane_type=carla.LaneType.Driving)
            if wp and math.hypot(wp.transform.location.x - x, wp.transform.location.y - y) < 3.0:
                rids.add(wp.road_id)
        ondr = sorted(k for k, v in routes.items() if rids & set(v))
        print(f"  way {wid} {tags_s(ways[wid]['tags'])} -> road {sorted(rids)} 노선 {ondr or '없음'}")

    # ---- 3-4 자전거도로 ----
    print("\n## 3-4. 자전거도로")
    for wid, w in ways.items():
        t = w["tags"]
        if t.get("highway") == "cycleway" or any(k.startswith("cycleway") for k in t):
            ps = [nodes[n] for n in w["nds"] if n in nodes]
            c = (sum(p[0] for p in ps) / len(ps), sum(p[1] for p in ps) / len(ps))
            ins = inside_frac(nodes, w, rings)
            print(f"way {wid} {tags_s(t)} {' '.join(f'{k}={v}' for k, v in t.items() if k.startswith('cycleway'))} "
                  f"{way_len(nodes, w):.0f}m 중심 {fmt(c)} 안쪽 {ins * 100:.0f}% j1까지 {dist_ll(c, nodes[J1]):.0f}m "
                  f"| {conv_status(wid)} | 맵겹침 {onroad.get(wid, 0):.2f}")

    # ---- 계단·육교 ----
    print("\n## P2-보행구조. 변환 입력의 steps / bridge")
    for wid, w in cways.items():
        t = w["tags"]
        if t.get("highway") == "steps" or t.get("bridge"):
            print(f"way {wid} {tags_s(t)} | 맵겹침 {onroad.get(wid, 0):.2f}")

    # ---- 3-5 중문 way ----
    print("\n## 3-5. 중문 way 2개")
    node_tags = {}
    for el in ET.parse(str(RAW_OSM)).getroot().iter("node"):
        t = {x.get("k"): x.get("v") for x in el.findall("tag")}
        if any(k in t for k in ("barrier", "entrance")):
            node_tags[el.get("id")] = {k: v for k, v in t.items() if k in ("barrier", "entrance", "name", "access")}
    print(f"원본 barrier/entrance 태그 노드 {len(node_tags)}개 (캠퍼스 전체)")
    mn = nodes[MIDDLE_NODE]
    print(f"중문 정차점 쪽 끝 노드 {MIDDLE_NODE} {fmt(mn)} 안쪽={point_in_polygon(mn[1], mn[0], rings)}")
    print(f"중문 게이트 추정점 {fmt(MIDDLE_GATE_EST)} 안쪽={point_in_polygon(MIDDLE_GATE_EST[1], MIDDLE_GATE_EST[0], rings)}"
          f" 끝 노드에서 {dist_ll(mn, MIDDLE_GATE_EST):.0f}m")
    n14 = BLD["인문사회관 N14"]
    print(f"인문사회관 N14 중심 {fmt(n14)} 끝 노드에서 {dist_ll(mn, n14):.0f}m")
    # 경계까지 최소 거리
    ring_pts = [(la, lo) for ring in rings for lo, la in ring]

    def to_boundary(p):
        px, py = tm(*p)
        best = 1e9
        for ring in rings:
            for i in range(len(ring) - 1):
                ax, ay = tm(ring[i][1], ring[i][0])
                bx, by = tm(ring[i + 1][1], ring[i + 1][0])
                vx, vy = bx - ax, by - ay
                t = max(0, min(1, ((px - ax) * vx + (py - ay) * vy) / (vx * vx + vy * vy or 1)))
                best = min(best, math.hypot(ax + t * vx - px, ay + t * vy - py))
        return best
    _ = ring_pts
    for wid in ["446440352", "446440353", "446440351"]:
        w = ways[wid]
        print(f"way {wid} {tags_s(w['tags'])} {way_len(nodes, w):.1f}m 안쪽 {inside_frac(nodes, w, rings) * 100:.0f}% | {conv_status(wid)}")
        for idx in (0, -1):
            nid = w["nds"][idx]
            p = nodes[nid]
            deg = [k for k, x in ways.items() if nid in x["nds"] and k != wid]
            print(f"  {'시작' if idx == 0 else '끝'} 노드 {nid} {fmt(p)} 안쪽={point_in_polygon(p[1], p[0], rings)} "
                  f"경계까지 {to_boundary(p):.1f}m 게이트추정점까지 {dist_ll(p, MIDDLE_GATE_EST):.0f}m N14까지 {dist_ll(p, n14):.0f}m "
                  f"접속 way {[(k, ways[k]['tags'].get('highway'), ways[k]['tags'].get('service'), 'conv' if k in cways and cways[k]['tags'].get('highway') == 'unclassified' else '-') for k in deg]}")
            if idx == -1:
                near = []
                for k, x in ways.items():
                    if "highway" not in x["tags"] and "barrier" not in x["tags"] and "entrance" not in x["tags"]:
                        continue
                    dmin = min((dist_ll(p, nodes[n]) for n in x["nds"] if n in nodes), default=1e9)
                    if dmin < 40 and k != wid:
                        near.append((dmin, k, tags_s(x["tags"]), inside_frac(nodes, x, rings)))
                for dmin, k, t, f in sorted(near)[:8]:
                    print(f"    끝 40m 안 way {k} {t} 최근접 {dmin:.0f}m 안쪽 {f * 100:.0f}%")
                gates = [(dist_ll(p, nodes[n]), n, nt) for n, nt in node_tags.items()
                         if n in nodes and dist_ll(p, nodes[n]) < 60]
                for dg, n, nt in sorted(gates):
                    print(f"    끝 60m 안 태그 노드 {n} {nt} {dg:.0f}m")

    # ---- 4. OSM traffic_calming / barrier 노드 (캠퍼스 안) ----
    print("\n## 4-N. 원본 OSM 캠퍼스 안 traffic_calming / barrier 노드와 노선")
    for el in ET.parse(str(RAW_OSM)).getroot().iter("node"):
        t = {x.get("k"): x.get("v") for x in el.findall("tag")}
        if not ("traffic_calming" in t or "barrier" in t):
            continue
        la, lo = float(el.get("lat")), float(el.get("lon"))
        if not point_in_polygon(lo, la, rings):
            continue
        x, y = ll_to_carla(la, lo)
        wp = cmap.get_waypoint(carla.Location(x=x, y=y, z=0), project_to_road=True, lane_type=carla.LaneType.Driving)
        d = math.hypot(wp.transform.location.x - x, wp.transform.location.y - y)
        wl = [k for k, w in ways.items() if el.get("id") in w["nds"]]
        ondr = sorted(k for k, v in routes.items() if wp.road_id in v) if d < 3.5 else []
        print(f"node {el.get('id')} {la:.6f}, {lo:.6f} {t} way {wl} 최근접 차선 r{wp.road_id} lane{wp.lane_id} s={wp.s:.1f} {d:.1f}m 노선(road 기준) {ondr or '없음'}")


if __name__ == "__main__":
    main()
