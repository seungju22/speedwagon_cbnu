#!/usr/bin/env python3
# 맵세션27 Phase 2·3 오프라인 확인 (읽기전용, 서버 없음, 맵·OSM 수정 없음).
# A 양성재 정차점 방향(주행 CSV 종점 -> 위경도, OSM 양성재 건물 대조)
# B 양성재 노선과 후문 노선의 겹침(세션18 시험 맵 _trial_s18_gates.xodr 를 읽기만 함, 채택 안 된 산출물)
# C 후문 게이트 위치(OSM barrier·작은 건물·주차장·고가 확인)
# D 후문 연결 6 way 개별 확인(태그·좌표·끝 노드 공유·주차장 폴리곤 안 여부)
# 실행: ~/campus_mobility_sim/.venv-carla/bin/python map/scripts/s27_route_checks.py > map/docs/logs/s27_route_checks.log
import csv
import glob
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from s19_scope_common import (BASE, RAW_OSM, XODR_V1, XODR_TRIAL, OFF_X, OFF_Y, LAT0, LON0, tm,  # noqa: E402
                              load_map, road_lengths, build_graph, dijkstra, path_to, nearest_lane)
from classify_internal import load_polygon_rings, point_in_polygon  # noqa: E402

POLY = BASE / "data/processed/cbnu_relation_polygon.json"
LOGS = BASE / "docs/logs"
YS_ROADS = [1247, 1914, 1356, 1917, 1355, 1446, 1172, 1427, 1127, 1699, 1126, 1726, 1125, 1733, 1124, 1826, 1123]
SOUTH_ROADS = [1247, 1914, 1356, 1917, 1355, 1446, 1172, 1428, 1240, 1695, 1239, 1426, 1238, 1563, 1237]
YS_STOP = (1123, 22.0)
START_LL = (36.632650, 127.453018)        # 출발점 road1247 s=0 (field_survey 0번)
BACK_B_CROSS = (36.624701, 127.463656)    # 후문B 경계 횡단점(392632034, humun_b_confirmed.md)
BACK_B_LOOP = (36.624909, 127.462514)     # road1333 고리 접점(field_survey 20번)
BACK6 = ["481945510", "481943505", "481475019", "452870644", "481945509", "481943503"]
YS_BLD = "442760311"


def load_raw():
    nodes, ways, ntags = {}, {}, {}
    for _, el in ET.iterparse(str(RAW_OSM)):
        if el.tag == "node":
            nodes[el.get("id")] = (float(el.get("lat")), float(el.get("lon")))
            t = {x.get("k"): x.get("v") for x in el.findall("tag")}
            if t:
                ntags[el.get("id")] = t
        elif el.tag == "way":
            ways[el.get("id")] = {"nds": [n.get("ref") for n in el.findall("nd")],
                                  "tags": {t.get("k"): t.get("v") for t in el.findall("tag")}}
            el.clear()
    return nodes, ways, ntags


def d_ll(a, b):
    ax, ay = tm(*a)
    bx, by = tm(*b)
    return math.hypot(ax - bx, ay - by)


def inv_tm(x, y):
    """TM(동, 북) -> 위경도. 뉴턴 반복(수치 야코비안). 왕복 오차는 로그에 찍는다."""
    lat, lon = LAT0, LON0
    for _ in range(20):
        fx, fy = tm(lat, lon)
        ex, ey = x - fx, y - fy
        if math.hypot(ex, ey) < 1e-4:
            break
        h = 1e-6
        ax, ay = tm(lat + h, lon)
        bx, by = tm(lat, lon + h)
        j11, j21 = (ax - fx) / h, (ay - fy) / h
        j12, j22 = (bx - fx) / h, (by - fy) / h
        det = j11 * j22 - j12 * j21
        lat += (j22 * ex - j12 * ey) / det
        lon += (-j21 * ex + j11 * ey) / det
    return lat, lon


def carla_to_ll(cx, cy):
    return inv_tm(cx - OFF_X, -cy - OFF_Y)


def compass(dx, dy):
    b = math.degrees(math.atan2(dx, dy)) % 360
    names = ["북", "북동", "동", "남동", "남", "남서", "서", "북서"]
    return b, names[int((b + 22.5) // 45) % 8]


def seg_dist(p, a, b):
    ax, ay = a
    bx, by = b
    px, py = p
    L2 = (bx - ax) ** 2 + (by - ay) ** 2
    t = 0 if L2 == 0 else max(0, min(1, ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / L2))
    return math.hypot(px - ax - t * (bx - ax), py - ay - t * (by - ay))


def poly_tm(nodes, w):
    return [tm(*nodes[n]) for n in w["nds"] if n in nodes]


def dist_to_line(p, pts):
    return min(seg_dist(p, pts[i], pts[i + 1]) for i in range(len(pts) - 1)) if len(pts) > 1 else math.hypot(
        p[0] - pts[0][0], p[1] - pts[0][1])


def area(pts):
    return abs(sum(pts[i][0] * pts[(i + 1) % len(pts)][1] - pts[(i + 1) % len(pts)][0] * pts[i][1]
                   for i in range(len(pts)))) / 2


def centroid(pts):
    return sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)


def in_ring_tm(p, ring):
    x, y = p
    inside = False
    for i in range(len(ring)):
        x1, y1 = ring[i]
        x2, y2 = ring[(i + 1) % len(ring)]
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


def fmt(p):
    return f"{p[0]:.6f}, {p[1]:.6f}"


def part_a(nodes, ways, rings):
    print("## A. 양성재 정차점 방향")
    rows = []
    for f in sorted(glob.glob(str(LOGS / "drive_log_*.csv"))):
        with open(f) as fh:
            r = list(csv.DictReader(fh))
        if r and r[-1].get("road_id") == "1123":
            rows.append((Path(f).name, r[-1]))
    print(f"종점 road1123 주행 CSV {len(rows)}개")
    ends = []
    for name, r in rows:
        cx, cy = float(r["x"]), float(r["y"])
        ll = carla_to_ll(cx, cy)
        back = tm(*ll)
        err = math.hypot(back[0] - (cx - OFF_X), back[1] - (-cy - OFF_Y))
        ends.append(ll)
        print(f"  {name} t={r['t']} s={r['s']} CARLA ({cx:.2f}, {cy:.2f}) -> {fmt(ll)} (왕복오차 {err:.4f}m)")
    end = (sum(e[0] for e in ends) / len(ends), sum(e[1] for e in ends) / len(ends))
    spread = max(d_ll(end, e) for e in ends)
    print(f"  종점 평균 {fmt(end)} (개별 종점 최대 편차 {spread:.2f}m)")
    outer = max(rings, key=len)
    otm = [tm(lat, lon) for lon, lat in outer]
    cen = centroid(otm)
    cen_ll = inv_tm(*cen)
    print(f"캠퍼스 폴리곤 외곽 링(점 {len(outer)}) 꼭짓점 평균 중심 {fmt(cen_ll)}")
    e = tm(*end)
    b, nm = compass(e[0] - cen[0], e[1] - cen[1])
    print(f"  중심 -> 양성재 종점: {math.hypot(e[0]-cen[0], e[1]-cen[1]):.0f}m, 방위 {b:.0f}도 ({nm})")
    s = tm(*START_LL)
    b, nm = compass(s[0] - cen[0], s[1] - cen[1])
    print(f"  중심 -> 출발점(정문 밖): 방위 {b:.0f}도 ({nm})")
    k = tm(*BACK_B_CROSS)
    b, nm = compass(k[0] - cen[0], k[1] - cen[1])
    print(f"  중심 -> 후문B 경계 횡단점: 방위 {b:.0f}도 ({nm})")
    print(f"  양성재 종점 -> 후문B 경계 횡단점 직선 {d_ll(end, BACK_B_CROSS):.0f}m")
    w = ways[YS_BLD]
    pts = poly_tm(nodes, w)
    c = inv_tm(*centroid(pts[:-1] if w["nds"][0] == w["nds"][-1] else pts))
    print(f"OSM way{YS_BLD} 태그 {w['tags']}")
    print(f"  꼭짓점 평균 {fmt(c)}, 종점에서 외곽선까지 {dist_to_line(e, pts):.1f}m, 종점이 건물 안: {in_ring_tm(e, pts)}")
    print("이름에 '양성재' 또는 '생활관' 이 들어간 OSM 요소:")
    for wid, ww in ways.items():
        n = ww["tags"].get("name", "")
        if "양성재" in n or "생활관" in n:
            q = poly_tm(nodes, ww)
            print(f"  way{wid} {n} {ww['tags'].get('building', '')} 중심 {fmt(inv_tm(*centroid(q)))}")
    print("종점 150m 안 이름 있는 건물:")
    near = []
    for wid, ww in ways.items():
        if "building" not in ww["tags"] or "name" not in ww["tags"]:
            continue
        q = poly_tm(nodes, ww)
        if len(q) < 3:
            continue
        d = dist_to_line(e, q)
        if d < 150:
            near.append((d, ww["tags"]["name"], wid))
    for d, n, wid in sorted(near):
        print(f"  {d:6.1f}m {n} (way{wid})")
    print()
    return end


def path_len(lengths, p, end_s=None):
    tot = sum(lengths[r][0] for r in p[:-1])
    return tot + (end_s if end_s is not None else lengths[p[-1]][0])


def part_b(nodes, ways):
    print("## B. 양성재 노선과 후문 노선 겹침 (세션18 시험 맵, 채택 안 된 산출물을 읽기만 함)")
    idmap = {}
    with open(LOGS / "s18_D_road_id_map.csv") as fh:
        for r in csv.DictReader(fh):
            idmap[int(r["old_id"])] = (int(r["trial_id"]), float(r["match_cost"]))
    Lv1 = road_lengths(XODR_V1)
    # frozen_v1 기준 공통 앞부분(양성재 vs 남문)
    common = []
    for a, b in zip(YS_ROADS, SOUTH_ROADS):
        if a != b:
            break
        common.append(a)
    print(f"frozen_v1: 양성재와 남문 공통 앞 road {len(common)}개 {common}, 길이 합 {sum(Lv1[r][0] for r in common):.1f}m")
    print(f"frozen_v1: 양성재 경로 합 {path_len(Lv1, YS_ROADS, YS_STOP[1]):.1f}m (s26 기록 637.4m 대조)")
    tmap = load_map(XODR_TRIAL)
    Lt = road_lengths(XODR_TRIAL)
    g = build_graph(tmap, Lt)
    ys_t = [idmap[r][0] for r in YS_ROADS]
    print(f"양성재 루트 시험 맵 번호 {ys_t} (형상 비용 최대 {max(idmap[r][1] for r in YS_ROADS):.3f})")
    src = idmap[1247][0]
    dist, prev = dijkstra(g, Lt, src, src_rest=Lt[src][0])
    results = {}
    for label, ll in (("고리 접점(20번)", BACK_B_LOOP), ("경계 횡단점", BACK_B_CROSS)):
        cx, cy = ll_carla(ll)
        best = None
        for rid in dist:
            if Lt[rid][1] != -1:
                continue
            L = Lt[rid][0]
            ss = 0.0
            while ss <= L:
                wp = tmap.get_waypoint_xodr(rid, -1, ss)
                if wp is not None:
                    loc = wp.transform.location
                    d = math.hypot(loc.x - cx, loc.y - cy)
                    if best is None or d < best[0]:
                        best = (d, rid, ss)
                ss += 1.0
        d, rid, ss = best
        p = path_to(prev, src, rid)
        plen = path_len(Lt, p, ss)
        pre = []
        for a, b in zip(ys_t, p):
            if a != b:
                break
            pre.append(a)
        pre_len = sum(Lt[r][0] for r in pre)
        inv = {v[0]: k for k, v in idmap.items()}
        print(f"[{label} {fmt(ll)}] 정문에서 도달 가능한 일반 road 최근접: 시험 r{rid} s={ss:.0f} 거리 {d:.1f}m")
        print(f"  최단 경로 {len(p)} road, {plen:.0f}m: {[str(r) + '(' + str(inv.get(r, 'new')) + ')' for r in p]}")
        print(f"  양성재 루트와 공통 앞부분 {len(pre)} road, {pre_len:.1f}m (옛 번호 {[inv.get(r) for r in pre]})")
        shared = set(ys_t) & set(p)
        print(f"  순서 무관 공통 road {len(shared)}개, 길이 합 {sum(Lt[r][0] for r in shared):.1f}m")
        results[label] = (p, rid, ss)
    # 양성재 정차점을 거쳐 후문으로 가는 경우
    ys_end = ys_t[-1]
    d2, prev2 = dijkstra(g, Lt, ys_end, src_rest=Lt[ys_end][0] - YS_STOP[1])
    for label, (p, rid, ss) in results.items():
        if rid in d2:
            q = path_to(prev2, ys_end, rid)
            via = path_len(Lt, ys_t, YS_STOP[1]) + (d2[rid] - Lt[rid][0] + ss)
            print(f"[{label}] 양성재 정차점 경유: 양성재까지 {path_len(Lt, ys_t, YS_STOP[1]):.0f}m + 이후 "
                  f"{d2[rid] - Lt[rid][0] + ss:.0f}m = {via:.0f}m ({len(q)} road), 최단 대비 +{via - path_len(Lt, p, ss):.0f}m")
        else:
            print(f"[{label}] 양성재 정차점에서 도달 불가")
    print()


def ll_carla(ll):
    x, y = tm(*ll)
    return x + OFF_X, -(y + OFF_Y)


def part_c(nodes, ways, ntags, rings):
    print("## C. 후문 게이트 위치 (OSM)")
    k = tm(*BACK_B_CROSS)
    print("barrier 태그 전부(노드·way), 후문B 경계 횡단점까지 거리:")
    for nid, t in ntags.items():
        if "barrier" in t:
            print(f"  node{nid} barrier={t['barrier']} {fmt(nodes[nid])} {d_ll(nodes[nid], BACK_B_CROSS):.0f}m")
    for wid, w in ways.items():
        if "barrier" in w["tags"]:
            q = poly_tm(nodes, w)
            print(f"  way{wid} barrier={w['tags']['barrier']} 중심 {fmt(inv_tm(*centroid(q)))} "
                  f"최근접 {dist_to_line(k, q):.0f}m")
    print("그 밖의 출입 관련 태그(entrance, access=private/no, amenity=parking_entrance/toll, toll, gate) 후문B 300m 안:")
    keys = {"entrance", "toll", "gate", "parking:entrance"}
    for nid, t in ntags.items():
        hit = (keys & set(t)) or t.get("amenity") in ("parking_entrance", "toll_booth") or \
            t.get("access") in ("private", "no", "permit", "customers") or t.get("barrier")
        if hit and d_ll(nodes[nid], BACK_B_CROSS) < 300:
            print(f"  node{nid} {t} {fmt(nodes[nid])} {d_ll(nodes[nid], BACK_B_CROSS):.0f}m")
    w34 = ways["392632034"]
    print(f"way392632034 노드(경계 밖 -> 안): 태그 {w34['tags']}")
    nds = w34["nds"]
    cum = 0.0
    prevp = None
    for i, n in enumerate(nds):
        p = tm(*nodes[n])
        if prevp:
            cum += math.hypot(p[0] - prevp[0], p[1] - prevp[1])
        prevp = p
        inside = point_in_polygon(nodes[n][1], nodes[n][0], rings)
        share = [wid for wid, w in ways.items() if n in w["nds"] and wid != "392632034" and "highway" in w["tags"]]
        print(f"  [{i}] node{n} {fmt(nodes[n])} 안={inside} 누적 {cum:.1f}m 공유 highway way {share} "
              f"태그 {ntags.get(n, {})}")
    print("후문B 경계 횡단점 200m 안 건물(면적 작은 순 10개 + 이름 있는 것):")
    rows = []
    q34 = poly_tm(nodes, w34)
    for wid, w in ways.items():
        if "building" not in w["tags"]:
            continue
        q = poly_tm(nodes, w)
        if len(q) < 3:
            continue
        d = dist_to_line(k, q)
        if d < 200:
            rows.append((area(q), d, dist_to_line(centroid(q), q34), wid, w["tags"]))
    for a, d, d34, wid, t in sorted(rows)[:10]:
        print(f"  way{wid} 면적 {a:.0f}m2 경계점 {d:.0f}m way392632034 까지 {d34:.0f}m 중심 "
              f"{fmt(inv_tm(*centroid(poly_tm(nodes, ways[wid]))))} {t}")
    for a, d, d34, wid, t in sorted(rows, key=lambda r: r[1]):
        if "name" in t:
            print(f"  (이름) way{wid} {t.get('name')} 면적 {a:.0f}m2 경계점 {d:.0f}m")
    print("amenity=parking 폴리곤 후문B 300m 안:")
    for wid, w in ways.items():
        if w["tags"].get("amenity") == "parking":
            q = poly_tm(nodes, w)
            if len(q) >= 3 and dist_to_line(k, q) < 300:
                print(f"  way{wid} 면적 {area(q):.0f}m2 경계점 {dist_to_line(k, q):.0f}m 중심 "
                      f"{fmt(inv_tm(*centroid(q)))} {w['tags']}")
    print("고가(bridge=yes 또는 layer>=1) highway 후문B 300m 안:")
    for wid, w in ways.items():
        t = w["tags"]
        if "highway" in t and (t.get("bridge") in ("yes", "viaduct") or t.get("layer", "0").lstrip("-").isdigit()
                               and int(t.get("layer", "0")) >= 1):
            q = poly_tm(nodes, w)
            if q and dist_to_line(k, q) < 300:
                print(f"  way{wid} {t.get('highway')} {t.get('name', '')} bridge={t.get('bridge')} "
                      f"layer={t.get('layer')} 최근접 {dist_to_line(k, q):.0f}m")
    print("OSM 버스정류장 이름 '후문'·'중문' 위치(참고):")
    for nid, t in ntags.items():
        n = t.get("name", "")
        if t.get("highway") == "bus_stop" and ("후문" in n or "중문" in n):
            inside = point_in_polygon(nodes[nid][1], nodes[nid][0], rings)
            print(f"  node{nid} {n} {fmt(nodes[nid])} 후문B 경계점에서 {d_ll(nodes[nid], BACK_B_CROSS):.0f}m 폴리곤 안={inside}")
    print()


def part_d(nodes, ways, rings):
    print("## D. 후문 연결 6 way 개별 확인 (원본 OSM)")
    parking = []
    for wid, w in ways.items():
        if w["tags"].get("amenity") == "parking":
            q = poly_tm(nodes, w)
            if len(q) >= 3:
                parking.append((wid, q))
    for wid in BACK6:
        w = ways[wid]
        q = poly_tm(nodes, w)
        L = sum(math.hypot(q[i + 1][0] - q[i][0], q[i + 1][1] - q[i][1]) for i in range(len(q) - 1))
        fr = sum(point_in_polygon(nodes[n][1], nodes[n][0], rings) for n in w["nds"]) / len(w["nds"])
        a, b = w["nds"][0], w["nds"][-1]
        sa = [x for x, ww in ways.items() if a in ww["nds"] and x != wid and "highway" in ww["tags"]]
        sb = [x for x, ww in ways.items() if b in ww["nds"] and x != wid and "highway" in ww["tags"]]
        mid = centroid(q)
        inpk = [pid for pid, pq in parking if in_ring_tm(mid, pq)]
        print(f"way{wid} {L:.1f}m 안쪽 {fr:.0%} 태그 {w['tags']}")
        print(f"  시작 node{a} {fmt(nodes[a])} 공유 {sa}")
        print(f"  끝   node{b} {fmt(nodes[b])} 공유 {sb}")
        print(f"  중간점이 amenity=parking 폴리곤 안: {inpk or '아님'}")
    print()


def main():
    print(f"원본 OSM {RAW_OSM.name}, 시험 맵 {XODR_TRIAL.name}(읽기만)")
    nodes, ways, ntags = load_raw()
    rings = load_polygon_rings(POLY)
    part_a(nodes, ways, rings)
    part_b(nodes, ways)
    part_c(nodes, ways, ntags, rings)
    part_d(nodes, ways, rings)


if __name__ == "__main__":
    main()
