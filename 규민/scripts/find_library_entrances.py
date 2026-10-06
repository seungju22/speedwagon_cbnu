#!/usr/bin/env python3
# 맵세션5 종점 재검토(사용자 요청, 2026-09-19). 읽기전용 분석.
# 세션4 A/B/C안이 건물 중심점 기준이라 실제 출입구를 반영 못한다는
# 사용자 지적에 따라, 도서관 구관/신관 건물 외곽선에서 각각
# 사회과학대학본관(N15)/자연과학대학본관(S1-1)/제1학생회관 후보
# (E3 신학생회관 또는 S14 구학생회관, OSM에 "제1학생회관" 명칭 자체는
# 없어 둘 다 후보로 남김) 방향으로 가장 가까운 외곽선 점을 출입구
# 추정 위치로 삼는다. amenity=entrance 태그 노드는 도서관 관련 way에
# 없음을 먼저 확인(전수조사, grep 결과 무관한 parking_entrance 1건뿐).
# xodr/osm 파일 미수정, 콘솔 출력만 — 실행: .venv-carla 파이썬 필요
# (numpy/matplotlib 의존, geo_calibrate.py와 동일).
import sys, math, heapq
sys.path.insert(0, "/home/gyumin/campus_mobility_sim/map/scripts")
import xml.etree.ElementTree as ET
from analyze_geometry import parse_roads, parse_junctions, geom_point
from geo_calibrate import (OSM_RAW_PATH, load_osm_nodes, junction_xodr_locations,
                            latlon_to_enu, fit_similarity, apply_transform)
import verify_session4 as vs

NEW_XODR = "/home/gyumin/campus_mobility_sim/map/maps/cbnu_internal_only_localtm_tags73.xodr"
root = ET.parse(NEW_XODR).getroot()
roads = parse_roads(root)
junctions = parse_junctions(root)
osm_nodes = load_osm_nodes(str(OSM_RAW_PATH))
j_xy = junction_xodr_locations(roads, junctions)

pairs = []
for jid, junction in junctions.items():
    name = junction["name"]
    if name in osm_nodes and jid in j_xy:
        lat, lon = osm_nodes[name]
        x, y = j_xy[jid]
        pairs.append((name, x, y, lat, lon))
ref_lat = sum(p[3] for p in pairs) / len(pairs)
ref_lon = sum(p[4] for p in pairs) / len(pairs)
src_xy = [(p[1], p[2]) for p in pairs]
dst_en = [latlon_to_enu(p[3], p[4], ref_lat, ref_lon) for p in pairs]
R, s_scale, t, angle_deg, residuals = fit_similarity(src_xy, dst_en)


def xodr_to_enu(x, y):
    return apply_transform(R, s_scale, t, x, y)


def enu_to_xodr(ex, ey):
    import numpy as np
    v = (np.array([ex, ey]) - t) / s_scale
    Rinv = np.linalg.inv(R)
    xy = Rinv @ v
    return xy[0], xy[1]


def load_osm_ways(path):
    nodes = {}
    ways = []
    cur = None
    for ev, elem in ET.iterparse(path, events=("start", "end")):
        if ev == "start" and elem.tag == "way":
            cur = {"id": elem.get("id"), "nds": [], "tags": {}}
        elif ev == "end" and elem.tag == "node":
            nodes[elem.get("id")] = (float(elem.get("lat")), float(elem.get("lon")))
        elif ev == "end" and elem.tag == "nd" and cur is not None:
            cur["nds"].append(elem.get("ref"))
        elif ev == "end" and elem.tag == "tag" and cur is not None:
            cur["tags"][elem.get("k")] = elem.get("v")
        elif ev == "end" and elem.tag == "way":
            ways.append(cur)
            cur = None
            elem.clear()
    return nodes, ways


osm_full_nodes, ways = load_osm_ways(str(OSM_RAW_PATH))
way_by_id = {w["id"]: w for w in ways}


def way_enu(w):
    return [latlon_to_enu(*osm_full_nodes[n], ref_lat, ref_lon) for n in w["nds"] if n in osm_full_nodes]


def centroid(pts):
    return (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))


def nearest_point_on_polygon(poly_pts, target):
    best = None
    best_d = math.inf
    for i in range(len(poly_pts) - 1):
        ax, ay = poly_pts[i]
        bx, by = poly_pts[i + 1]
        dx, dy = bx - ax, by - ay
        seglen2 = dx * dx + dy * dy
        if seglen2 == 0:
            px, py = ax, ay
        else:
            tt = ((target[0] - ax) * dx + (target[1] - ay) * dy) / seglen2
            tt = max(0.0, min(1.0, tt))
            px, py = ax + tt * dx, ay + tt * dy
        d = math.hypot(target[0] - px, target[1] - py)
        if d < best_d:
            best_d = d
            best = (px, py)
    return best, best_d


lib_gu = way_enu(way_by_id["442066135"])
lib_sin = way_enu(way_by_id["648686778"])

target_ways = {
    "N15": way_by_id["446440345"],
    "S1-1": way_by_id["452270376"],
    "E3": way_by_id["452264484"],
    "S14": way_by_id["452267229"],
}
tc = {k: centroid(way_enu(w)) for k, w in target_ways.items()}

entrances = {
    "출입구1_구관_사회과학대방향": nearest_point_on_polygon(lib_gu, tc["N15"]),
    "출입구2_구관_자연과학대방향": nearest_point_on_polygon(lib_gu, tc["S1-1"]),
    "출입구3_신관_제1학생회관방향(E3후보)": nearest_point_on_polygon(lib_sin, tc["E3"]),
}

# ---- 도로망 전체(plain road, junction==-1) s-샘플링, 1m 간격 ----
samples = []  # (rid, s, x, y)
for rid, road in roads.items():
    if road["junction"] != "-1":
        continue
    for geom in road["geoms"]:
        n = max(2, int(geom["length"]) + 1)
        for i in range(n + 1):
            p = i / n
            s_val = geom["s"] + p * geom["length"]
            x, y = geom_point(geom, p)
            samples.append((rid, s_val, x, y))

adj = vs.build_road_graph(roads)
name2jid = vs.name_to_jid(junctions)
gate_jid = name2jid.get("2261340221")
gate_starts = vs.roads_touching_junction(roads, gate_jid)


def nearest_network_point(target_en):
    target_xodr = enu_to_xodr(*target_en)
    best = None
    best_d = math.inf
    for rid, s_val, x, y in samples:
        d = math.hypot(x - target_xodr[0], y - target_xodr[1])
        if d < best_d:
            best_d = d
            best = (rid, s_val, x, y)
    return best, best_d


def route_length_to_point(rid, s_val):
    road = roads[rid]
    length = road["length"]
    options = []
    for end, link, partial in (
        ("start", road["pred"], s_val),
        ("end", road["succ"], length - s_val),
    ):
        if link and link.get("elementType") == "junction":
            jid = link["elementId"]
            goal_roads = vs.roads_touching_junction(roads, jid) - {rid}
            if not goal_roads:
                continue
            path, dist = vs.dijkstra_shortest(adj, roads, gate_starts, goal_roads)
            if path is not None:
                options.append((dist + partial, end, jid, path + [rid]))
    if not options:
        return None
    return min(options, key=lambda o: o[0])


print(f"{'entrance':40s} {'ENU':>28s} {'nearest_road':>12s} {'dist_to_road_m':>14s} {'route_from_gate_m':>18s}")
for label, (pt, d_to_bldg) in entrances.items():
    (rid, s_val, x, y), d_net = nearest_network_point(pt)
    pt_en = xodr_to_enu(x, y)
    route = route_length_to_point(rid, s_val)
    if route:
        total, end, jid, path = route
        print(f"{label:40s} ENU={pt} 건물centroid까지={d_to_bldg:.2f}m")
        print(f"  -> 도로망 최근접: road{rid} s={s_val:.2f}m 지점(도로전체길이{roads[rid]['length']:.2f}m), "
              f"거리={d_net:.2f}m, ENU={pt_en}")
        print(f"  -> 정문 경로: junction{jid}({end}쪽)까지 경유 + 잔여, 총 {total:.2f}m, "
              f"road수={len(path)}, path={path}")
    else:
        print(f"{label}: 경로계산 실패")
    print()
