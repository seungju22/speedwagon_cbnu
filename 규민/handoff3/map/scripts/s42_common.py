#!/usr/bin/env python3
# 10/09 공통: 동결본 v2 road <-> OSM way 대응, 방향, 연결 그래프 (읽기전용, 서버 없음)
# carla.Map 은 오프라인으로 만든다(서버 접속 없음). 모든 road 는 lane -1 하나(반쪽 도로), 진행 방향 = s 증가
# way 대응: 일반 road(junction 아님)를 1m 간격으로 샘플해 변환 입력 OSM(tags81_B_smooth) highway way 와 거리 중앙값 최소인 것.
#   방향: road 첫·끝 샘플을 way 선에 투영한 위치가 커지면 way 정방향(fwd), 작아지면 역방향(bwd)
# 좌표는 CARLA (x, y). 위경도 -> CARLA 는 s22_buildings.Frame(xodr 머리말 투영 + offset)
import math
import re
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict, deque
from pathlib import Path

import carla
from shapely.geometry import LineString, Point, Polygon
from shapely.strtree import STRtree

HERE = Path(__file__).resolve().parent
BASE = HERE.parent
sys.path.insert(0, str(HERE))
from classify_internal import load_polygon_rings  # noqa: E402
from s22_buildings import Frame  # noqa: E402

XODR = BASE / "maps/cbnu_campus_frozen_v2.xodr"
OSM_IN = BASE / "data/processed/cbnu_internal_boundary_tags81_B_smooth.osm"   # v2 변환 입력(평활화 후)
OSM_RAW = BASE / "data/raw/cbnu_campus.osm"
POLY = BASE / "data/processed/cbnu_relation_polygon.json"
STOPS = BASE / "data/stops_v2.yaml"
MATCH_MAX = 6.0     # road 샘플과 way 거리 중앙값 상한(m). 반쪽 도로 중심은 way 에서 약 1.7m

DRIVE_HW = {"motorway", "trunk", "primary", "secondary", "tertiary", "unclassified", "residential",
            "service", "living_street", "track", "road", "motorway_link", "trunk_link", "primary_link",
            "secondary_link", "tertiary_link"}


def parse_osm(path):
    nodes, ntags, ways, rels = {}, {}, {}, []
    for _, el in ET.iterparse(str(path)):
        if el.tag == "node":
            nodes[el.get("id")] = (float(el.get("lat")), float(el.get("lon")))
            t = {x.get("k"): x.get("v") for x in el.findall("tag")}
            if t:
                ntags[el.get("id")] = t
            el.clear()
        elif el.tag == "way":
            ways[el.get("id")] = ([n.get("ref") for n in el.findall("nd")],
                                  {t.get("k"): t.get("v") for t in el.findall("tag")})
            el.clear()
    return nodes, ntags, ways


def load_stops():
    st = {}
    txt = STOPS.read_text().split("depot_candidates")[0]
    for m in re.finditer(r"id: (\w+)\n    name: ([^\n]+)\n.*?road_id: (\d+)\n    s: ([\d.]+)\n    lane_id: (-?\d+)\n"
                         r"    carla_xyz: \[([-\d.]+), ([-\d.]+)", txt, re.S):
        st[m.group(1)] = dict(name=m.group(2), road=int(m.group(3)), s=float(m.group(4)), lane=int(m.group(5)),
                              xy=(float(m.group(6)), float(m.group(7))))
    return st


class V2:
    def __init__(self, xodr=XODR):
        self.text = Path(xodr).read_text()
        self.frame = Frame(self.text)
        self.cmap = carla.Map(Path(xodr).stem, self.text)
        root = ET.fromstring(self.text.encode())
        self.length, self.junc, self.name = {}, {}, {}
        self.pred, self.succ = {}, {}
        for r in root.iter("road"):
            rid = int(r.get("id"))
            self.length[rid] = float(r.get("length"))
            self.junc[rid] = int(r.get("junction"))
            self.name[rid] = r.get("name")
            lk = r.find("link")
            for tag, d in (("predecessor", self.pred), ("successor", self.succ)):
                e = lk.find(tag) if lk is not None else None
                if e is not None:
                    d[rid] = (e.get("elementType"), int(e.get("elementId")))
        # (road, lane) 연결 그래프
        self.G = defaultdict(set)
        for a, b in self.cmap.get_topology():
            self.G[a.road_id].add(b.road_id)
        self.Grev = defaultdict(set)
        for u, vs in self.G.items():
            for v in vs:
                self.Grev[v].add(u)
        self._pts = {}
        rings = load_polygon_rings(POLY)
        self.poly = Polygon([self.frame.carla(lat, lon) for lon, lat in rings[0]])

    def pts(self, rid, step=1.0):
        """road 중심선(lane -1) 샘플, s 순서. (x, y) 목록. 캐시 키는 (road, 간격)"""
        key = (rid, step)
        if key not in self._pts:
            L = self.length[rid]
            n = max(2, int(L / step) + 1)
            out = []
            for i in range(n):
                s = min(L - 1e-3, max(1e-3, L * i / (n - 1)))
                w = self.cmap.get_waypoint_xodr(rid, -1, s)
                if w is not None:
                    loc = w.transform.location
                    out.append((loc.x, loc.y))
            self._pts[key] = out
        return self._pts[key]

    def reach_from(self, start):
        q, seen = deque([start]), {start}
        while q:
            u = q.popleft()
            for v in self.G[u]:
                if v not in seen:
                    seen.add(v)
                    q.append(v)
        return seen

    def reach_to(self, goal):
        q, seen = deque([goal]), {goal}
        while q:
            u = q.popleft()
            for v in self.Grev[u]:
                if v not in seen:
                    seen.add(v)
                    q.append(v)
        return seen

    def path(self, a, b):
        """road a 에서 road b 까지 최단(road 수) 경로. 없으면 None"""
        q, prev = deque([a]), {a: None}
        while q:
            u = q.popleft()
            if u == b:
                out = []
                while u is not None:
                    out.append(u)
                    u = prev[u]
                return out[::-1]
            for v in self.G[u]:
                if v not in prev:
                    prev[v] = u
                    q.append(v)
        return None

    def stop_reach(self, src, dst):
        if src["road"] == dst["road"] and dst["s"] >= src["s"]:
            return True
        if src["road"] == dst["road"]:
            # 같은 road 뒤쪽: 한 바퀴 돌아와야 한다
            return any(dst["road"] in self.reach_from(v) for v in self.G[src["road"]])
        return dst["road"] in self.reach_from(src["road"])


def way_lines(frame, nodes, ways, keep):
    out = {}
    for wid, (nds, tags) in ways.items():
        if not keep(tags):
            continue
        pts = [frame.carla(*nodes[n]) for n in nds if n in nodes]
        if len(pts) >= 2:
            out[wid] = LineString(pts)
    return out


def match_roads(v2, lines):
    """일반 road -> (way, 방향, 거리 중앙값). 못 맞추면 way None"""
    ids = list(lines)
    tree = STRtree([lines[i] for i in ids])
    res = {}
    for rid in v2.length:
        if v2.junc[rid] != -1:
            continue
        P = v2.pts(rid)
        if len(P) < 2:
            res[rid] = (None, None, None)
            continue
        ls = LineString(P)
        cand = tree.query(ls.buffer(MATCH_MAX))
        best = None
        for ci in cand:
            wid = ids[int(ci)]
            L = lines[wid]
            ds = sorted(L.distance(Point(p)) for p in P)
            med = ds[len(ds) // 2]
            if best is None or med < best[2]:
                best = (wid, L, med)
        if best is None or best[2] > MATCH_MAX:
            res[rid] = (None, None, best[2] if best else None)
            continue
        wid, L, med = best
        t0, t1 = L.project(Point(P[0])), L.project(Point(P[-1]))
        if L.is_ring or L.coords[0] == L.coords[-1]:
            # 고리: 가운데 점 두 개의 투영 순서로 본다
            k = len(P) // 3
            t0, t1 = L.project(Point(P[k])), L.project(Point(P[min(len(P) - 1, 2 * k)]))
        res[rid] = (wid, "fwd" if t1 >= t0 else "bwd", med)
    return res


def min_radius(P):
    """3점 외접원 반경의 최솟값(m), 점 간격 약 1m. 직선이면 inf"""
    best = math.inf
    for i in range(1, len(P) - 1):
        (ax, ay), (bx, by), (cx, cy) = P[i - 1], P[i], P[i + 1]
        a, b, c = math.dist(P[i - 1], P[i]), math.dist(P[i], P[i + 1]), math.dist(P[i - 1], P[i + 1])
        area2 = abs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax))
        if area2 > 1e-9 and min(a, b) > 0.3:
            best = min(best, a * b * c / (2 * area2))
    return best


def heading_change(P):
    """road 전체 방향 변화(도). 첫 2점과 끝 2점 방위 차"""
    if len(P) < 3:
        return 0.0
    h0 = math.atan2(P[1][1] - P[0][1], P[1][0] - P[0][0])
    h1 = math.atan2(P[-1][1] - P[-2][1], P[-1][0] - P[-2][0])
    return math.degrees((h1 - h0 + math.pi) % (2 * math.pi) - math.pi)


WAY_HIT = 2.4       # way 샘플에서 road 중심선까지 거리 상한(m). 반쪽 도로 1.68m, 정문 평행 일방 way 간격 3.4m


def road_segments(v2):
    """일반 road 중심선 조각 STRtree. (트리, 조각 목록[(road, 방위 단위벡터, LineString)])"""
    segs = []
    for rid in v2.length:
        if v2.junc[rid] != -1:
            continue
        P = v2.pts(rid)
        for a, b in zip(P, P[1:]):
            d = math.dist(a, b)
            if d < 1e-6:
                continue
            segs.append((rid, ((b[0] - a[0]) / d, (b[1] - a[1]) / d), LineString([a, b])))
    return STRtree([s[2] for s in segs]), segs


def way_coverage(lines, tree, segs, step=2.0):
    """way 마다 샘플(step m) 중 같은 방향(fwd)·반대 방향(bwd) road 가 WAY_HIT 안에 있는 비율과 road 목록"""
    out = {}
    for wid, L in lines.items():
        n = max(2, int(L.length / step) + 1)
        f = b = 0
        rf, rb = set(), set()
        for i in range(n):
            t = L.length * i / (n - 1)
            p = L.interpolate(t)
            q = L.interpolate(min(L.length, t + 0.5)) if t + 0.5 <= L.length else p
            q0 = L.interpolate(max(0.0, t - 0.5))
            dx, dy = q.x - q0.x, q.y - q0.y
            dn = math.hypot(dx, dy) or 1.0
            ux, uy = dx / dn, dy / dn
            hf = hb = False
            for ci in tree.query(p.buffer(WAY_HIT)):
                rid, (sx, sy), seg = segs[int(ci)]
                if seg.distance(p) > WAY_HIT:
                    continue
                dot = ux * sx + uy * sy
                if dot > 0.7:
                    hf = True
                    rf.add(rid)
                elif dot < -0.7:
                    hb = True
                    rb.add(rid)
            f += hf
            b += hb
        out[wid] = dict(fwd=f / n, bwd=b / n, roads_fwd=sorted(rf), roads_bwd=sorted(rb), length=L.length)
    return out


def all_way_lines(frame):
    """raw OSM 의 모든 highway way. 변환 입력(평활화)에 같은 id 가 있으면 그 형상을 쓴다"""
    rn, rnt, rw = parse_osm(OSM_RAW)
    n, nt, w = parse_osm(OSM_IN)
    lines, tags, intags = {}, {}, {}
    for wid, (nds, t) in rw.items():
        if "highway" not in t:
            continue
        src_n, src_nds = (n, w[wid][0]) if wid in w else (rn, nds)
        pts = [frame.carla(*src_n[x]) for x in src_nds if x in src_n]
        if len(pts) >= 2:
            lines[wid] = LineString(pts)
            tags[wid] = t
            intags[wid] = w[wid][1] if wid in w else None
    return lines, tags, intags, (rn, rnt, rw)


def dijkstra(v2, a, b, banned=frozenset()):
    """road a 끝에서 road b 까지 길이 합 최소 경로(road 목록). 없으면 None. banned road 는 지나지 않는다"""
    import heapq
    dist, prev = {a: 0.0}, {a: None}
    h = [(0.0, a)]
    while h:
        d, u = heapq.heappop(h)
        if u == b:
            out = []
            while u is not None:
                out.append(u)
                u = prev[u]
            return out[::-1], d
        if d > dist.get(u, math.inf):
            continue
        for v in v2.G[u]:
            if v in banned:
                continue
            nd = d + v2.length[v]
            if nd < dist.get(v, math.inf):
                dist[v], prev[v] = nd, u
                heapq.heappush(h, (nd, v))
    return None, None


def stop_path(v2, src, dst, banned=frozenset()):
    """정류장 사이 경로(road 목록, m). 같은 road 앞쪽이면 그 road 하나"""
    if src["road"] == dst["road"] and dst["s"] >= src["s"]:
        return [src["road"]], dst["s"] - src["s"]
    best = None
    for v in v2.G[src["road"]]:
        if v in banned:
            continue
        p, d = dijkstra(v2, v, dst["road"], banned)
        if p is not None:
            tot = (v2.length[src["road"]] - src["s"]) + v2.length[v] + d - v2.length[dst["road"]] + dst["s"]
            if v == dst["road"]:
                tot = (v2.length[src["road"]] - src["s"]) + dst["s"]
            if best is None or tot < best[1]:
                best = ([src["road"]] + p, tot)
    return best if best else (None, None)


def lane_radius(v2, rid):
    """xodr planView(line·arc·paramPoly3)로 기준선과 lane -1 중심선의 최소 반경(m)을 해석적으로 계산.
    lane -1 중심 = 기준선 왼쪽으로 laneOffset - 폭/2 (laneOffset·폭은 s=0 의 a 값, 이 맵의 연결로는 상수).
    10/09 처음 보고의 '반경 2.44m'는 1m 간격 waypoint 3점 외접원 값이라 점 간격에 따라 0.76~4.45m 로 흔들렸다"""
    import xml.etree.ElementTree as _ET
    if not hasattr(v2, "_roads_xml"):
        v2._roads_xml = {int(r.get("id")): r for r in _ET.fromstring(v2.text.encode()).iter("road")}
    r = v2._roads_xml[rid]
    lo = r.find("lanes/laneOffset")
    off_a = float(lo.get("a")) if lo is not None else 0.0
    w = r.find("lanes/laneSection/right/lane[@id='-1']/width")
    off = off_a - float(w.get("a")) / 2 if w is not None else off_a
    rref, rlane = math.inf, math.inf
    for g in r.findall("planView/geometry"):
        ks = []
        if g.find("arc") is not None:
            ks = [float(g.find("arc").get("curvature"))]
        elif g.find("paramPoly3") is not None:
            pp = {k: float(v) for k, v in g.find("paramPoly3").attrib.items() if k != "pRange"}
            for i in range(2001):
                p = i / 2000
                du = pp["bU"] + 2 * pp["cU"] * p + 3 * pp["dU"] * p * p
                dv = pp["bV"] + 2 * pp["cV"] * p + 3 * pp["dV"] * p * p
                ddu, ddv = 2 * pp["cU"] + 6 * pp["dU"] * p, 2 * pp["cV"] + 6 * pp["dV"] * p
                sp = math.hypot(du, dv)
                if sp > 1e-9:
                    ks.append((du * ddv - dv * ddu) / sp ** 3)
        for k in ks:
            if abs(k) > 1e-12:
                rref = min(rref, 1 / abs(k))
                den = 1 - k * off
                rlane = min(rlane, abs(den / k))
    return rref, rlane
