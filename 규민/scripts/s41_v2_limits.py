#!/usr/bin/env python3
# 맵세션41: README "v2 의 한계" 문장 확인(읽기전용, 서버 없음)
# 1) 캠퍼스 경계(cbnu_relation_polygon.json)를 지나는 주행 차선: 들어오는 쪽 / 나가는 쪽, 가장 가까운 게이트(정문·후문 정류장)
#    lane -1 은 road s 증가 방향으로 달린다(모든 road 가 반쪽 도로). road 시작·끝이 경계 안/밖으로 갈리면 그 방향
# 2) 정류장 6지점 30쌍 도달 가능 여부: carla.Map(동결본) topology 로 (road, lane) 그래프. 같은 road 면 s 순서까지 본다
# 사용: .venv-carla/bin/python map/scripts/s41_v2_limits.py
import math
import re
import sys
from collections import defaultdict, deque
from pathlib import Path

import carla
from shapely.geometry import LineString, Point, Polygon

HERE = Path(__file__).resolve().parent
BASE = HERE.parent
sys.path.insert(0, str(HERE))
from classify_internal import load_polygon_rings  # noqa: E402
from s22_buildings import Frame  # noqa: E402

XODR = BASE / "maps/cbnu_campus_frozen_v2.xodr"
text = XODR.read_text()
frame = Frame(text)
cmap = carla.Map(XODR.stem, text)
rings = load_polygon_rings(BASE / "data/processed/cbnu_relation_polygon.json")
poly = Polygon([frame.carla(lat, lon) for lon, lat in rings[0]])

st = {}
for m in re.finditer(r"id: (\w+)\n    name: ([^\n]+)\n.*?road_id: (\d+)\n    s: ([\d.]+)\n    lane_id: (-?\d+)\n    carla_xyz: \[([-\d.]+), ([-\d.]+)",
                     (BASE / "data/stops_v2.yaml").read_text().split("depot_candidates")[0], re.S):
    st[m.group(1)] = dict(name=m.group(2), road=int(m.group(3)), s=float(m.group(4)), lane=int(m.group(5)),
                          xy=(float(m.group(6)), float(m.group(7))))

# 1) 경계를 지나는 주행 차선
print("[1] 캠퍼스 경계를 지나는 주행 차선(road 단위, 진행 방향 = s 증가)")
seen = set()
cross = []
for a, b in cmap.get_topology():
    key = (a.road_id, a.section_id, a.lane_id)
    if key in seen:
        continue
    seen.add(key)
    pts = []
    w = a
    pts.append((w.transform.location.x, w.transform.location.y))
    for _ in range(400):
        n = w.next(1.0)
        if not n or n[0].road_id != a.road_id or n[0].lane_id != a.lane_id:
            break
        w = n[0]
        pts.append((w.transform.location.x, w.transform.location.y))
    pts.append((b.transform.location.x, b.transform.location.y))
    i0, i1 = poly.contains(Point(pts[0])), poly.contains(Point(pts[-1]))
    if i0 != i1:
        kind = "나감(안->밖)" if i0 else "들어옴(밖->안)"
        hit = LineString(pts).intersection(poly.exterior)
        hp = hit if hit.geom_type == "Point" else list(getattr(hit, "geoms", [hit]))[0]
        near = min(("main_gate", "back_gate"), key=lambda k: math.dist(st[k]["xy"], (hp.x, hp.y)))
        d = math.dist(st[near]["xy"], (hp.x, hp.y))
        cross.append((near, d, a.road_id, a.lane_id, kind, a.is_junction))
G = defaultdict(set)
for a, b in cmap.get_topology():
    G[(a.road_id, a.lane_id)].add((b.road_id, b.lane_id))


def bfs(start):
    q, s = deque([start]), {start}
    while q:
        u = q.popleft()
        for v in G[u]:
            if v not in s:
                s.add(v)
                q.append(v)
    return s
inner = (st["middle_gate"]["road"], -1)   # 캠퍼스 안 기준점: 중문 정류장 road
from_inner = bfs(inner)
for near, d, rid, lid, kind, j in sorted(cross, key=lambda c: (c[0], c[1])):
    k = (rid, lid)
    if "나감" in kind:
        note = f"캠퍼스 안(중문 road)에서 도달 {'가능' if k in from_inner else '불가'}"
    else:
        note = f"이 차선에서 캠퍼스 안(중문 road) 도달 {'가능' if inner in bfs(k) else '불가'}"
    print(f"  r{rid} lane{lid}{' (junction)' if j else ''}: {kind}, 경계 교차점에서 {near} 정류장까지 {d:.1f}m, {note}")
print(f"  합계 {len(cross)}: 들어옴 {sum('들어옴' in c[4] for c in cross)}, 나감 {sum('나감' in c[4] for c in cross)}")

# 2) 정류장 30쌍
g = defaultdict(set)
for a, b in cmap.get_topology():
    g[(a.road_id, a.lane_id)].add((b.road_id, b.lane_id))
# 같은 (road, lane) 안의 다음 section 연결은 topology 에 이미 있다. road 안 s 순서는 아래에서 처리
def reach(src, dst):
    s0, d0 = (src["road"], src["lane"]), (dst["road"], dst["lane"])
    if s0 == d0 and dst["s"] >= src["s"]:
        return True
    q, seen2 = deque([s0]), {s0}
    while q:
        u = q.popleft()
        for v in g[u]:
            if v == d0:
                return True
            if v not in seen2:
                seen2.add(v)
                q.append(v)
    return False

ids = list(st)
print("\n[2] 정류장 30쌍 도달(출발 -> 도착)")
ok = 0
for i in ids:
    row = []
    for j in ids:
        if i == j:
            continue
        r = reach(st[i], st[j])
        ok += r
        row.append(f"{st[j]['name']}:{'O' if r else 'X'}")
    print(f"  {st[i]['name']} -> " + " ".join(row))
print(f"  도달 가능 {ok}/30")
