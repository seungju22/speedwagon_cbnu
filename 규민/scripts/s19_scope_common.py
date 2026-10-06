#!/usr/bin/env python3
# 맵세션19 Phase 1 공용 도구 (읽기전용, 서버 없음).
# - OSM 읽기, TM 투영(osm_to_xodr.py PROJ_STRING 과 같은 중심), xodr 좌표 변환
# - 오프라인 carla.Map 으로 방향 그래프(lane -1 next()) 구성, 경로 탐색
# 좌표계: TM(동 x, 북 y) -> xodr = TM + offset(521.51, 493.61) -> CARLA = (x, -y)
import heapq
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import carla

BASE = Path("/home/gyumin/campus_mobility_sim/map")
RAW_OSM = BASE / "data/raw/cbnu_campus.osm"
CONV_OSM = BASE / "data/processed/cbnu_internal_boundary_tags73_smooth.osm"  # 변환 입력
XODR_V1 = BASE / "maps/cbnu_internal_only_localtm_tags73_smooth.xodr"
XODR_TRIAL = BASE / "maps/cbnu_internal_only_localtm_tags73_smooth_trial_s18_gates.xodr"
LAT0, LON0 = 36.627298, 127.456394
OFF_X, OFF_Y = 521.51, 493.61
LANE = -1

# WGS84 횡메르카토르(Snyder 식 8-9~8-10, k0=1). 캠퍼스 1km 범위라 오차 mm 수준
_A = 6378137.0
_F = 1 / 298.257223563
_E2 = _F * (2 - _F)
_EP2 = _E2 / (1 - _E2)


def _mer(phi):
    e2, e4, e6 = _E2, _E2 ** 2, _E2 ** 3
    return _A * ((1 - e2 / 4 - 3 * e4 / 64 - 5 * e6 / 256) * phi
                 - (3 * e2 / 8 + 3 * e4 / 32 + 45 * e6 / 1024) * math.sin(2 * phi)
                 + (15 * e4 / 256 + 45 * e6 / 1024) * math.sin(4 * phi)
                 - (35 * e6 / 3072) * math.sin(6 * phi))


_M0 = _mer(math.radians(LAT0))


def tm(lat, lon):
    phi, lam = math.radians(lat), math.radians(lon - LON0)
    n = _A / math.sqrt(1 - _E2 * math.sin(phi) ** 2)
    t = math.tan(phi) ** 2
    c = _EP2 * math.cos(phi) ** 2
    a = lam * math.cos(phi)
    x = n * (a + (1 - t + c) * a ** 3 / 6 + (5 - 18 * t + t * t + 72 * c - 58 * _EP2) * a ** 5 / 120)
    y = (_mer(phi) - _M0 + n * math.tan(phi) * (a * a / 2 + (5 - t + 9 * c + 4 * c * c) * a ** 4 / 24
                                              + (61 - 58 * t + t * t + 600 * c - 330 * _EP2) * a ** 6 / 720))
    return x, y


def ll_to_carla(lat, lon):
    x, y = tm(lat, lon)
    return x + OFF_X, -(y + OFF_Y)


def load_osm(path):
    nodes, ways = {}, {}
    for _, el in ET.iterparse(str(path)):
        if el.tag == "node":
            nodes[el.get("id")] = (float(el.get("lat")), float(el.get("lon")))
        elif el.tag == "way":
            ways[el.get("id")] = {"nds": [n.get("ref") for n in el.findall("nd")],
                                  "tags": {t.get("k"): t.get("v") for t in el.findall("tag")}}
            el.clear()
    return nodes, ways


def load_map(path):
    return carla.Map(Path(path).stem, Path(path).read_text())


def road_lengths(path):
    root = ET.parse(str(path)).getroot()
    return {int(r.get("id")): (float(r.get("length")), int(r.get("junction")))
            for r in root.iter("road")}


def successors(cmap, rid, length):
    """lane -1 끝에서 next() 로 도착하는 road id 집합."""
    wp = cmap.get_waypoint_xodr(rid, LANE, max(length - 0.02, 0.0))
    out = set()
    if wp is None:
        return out
    for step in (0.05, 0.3, 1.0):
        for n in wp.next(step):
            if n.road_id != rid:
                out.add(n.road_id)
        if out:
            break
    return out


def turn_deg(cmap, rid, length):
    """끝점 기준 회전각(+우 / -좌, CARLA yaw 차). s18 A-3 의 '끝점 기준'."""
    a = cmap.get_waypoint_xodr(rid, LANE, 0.001)
    b = cmap.get_waypoint_xodr(rid, LANE, max(length - 0.001, 0.0))
    if a is None or b is None:
        return 0.0
    d = b.transform.rotation.yaw - a.transform.rotation.yaw
    return (d + 180) % 360 - 180


def build_graph(cmap, lengths):
    g = {}
    for rid, (L, _) in lengths.items():
        g[rid] = successors(cmap, rid, L)
    return g


def dijkstra(g, lengths, src, banned=frozenset(), src_rest=None):
    """src road 끝까지를 시작 비용(src_rest)으로, 이후 road 길이 합. 반환 dist, prev."""
    start = src_rest if src_rest is not None else lengths[src][0]
    dist, prev = {src: start}, {}
    pq = [(start, src)]
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist.get(u, 1e18):
            continue
        for v in g.get(u, ()):
            if v in banned:
                continue
            nd = d + lengths[v][0]
            if nd < dist.get(v, 1e18):
                dist[v], prev[v] = nd, u
                heapq.heappush(pq, (nd, v))
    return dist, prev


def path_to(prev, src, dst):
    p = [dst]
    while p[-1] != src:
        if p[-1] not in prev:
            return None
        p.append(prev[p[-1]])
    return p[::-1]


def nearest_lane(cmap, x, y):
    """CARLA 좌표 점에서 가장 가까운 Driving 차선 waypoint 와 거리."""
    wp = cmap.get_waypoint(carla.Location(x=x, y=y, z=0.0), project_to_road=True,
                           lane_type=carla.LaneType.Driving)
    if wp is None:
        return None, 1e9
    loc = wp.transform.location
    return wp, math.hypot(loc.x - x, loc.y - y)
