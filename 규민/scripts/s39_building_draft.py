#!/usr/bin/env python3
# 맵세션39: 건물 배치 초안(읽기전용, 서버 없음, 소환 없음). 좌표·크기·검사 결과를 소환 전에 확정해 CSV 로 남긴다
# 원칙(세션38 2부 결과 반영)
#   - 크기·방향: OSM 건물 폴리곤 기준(최소 외접 사각형 길이·폭·방향). API 바운딩 박스는 쓰지 않는다(축·크기 불일치, s38)
#   - 좌표: 소환 전에 확정한다(static.prop.mesh 는 소환 뒤 set_transform 으로 안 움직임, s38)
#   - 위치는 OSM 위경도에서 매번 xodr 머리말로 변환(s22_buildings.py 의 Frame). 직선 연장·순서 배치(커서) 안 함
#     (s38 House02·Apartment03 불합격 사유: r1392 끝 너머로 직선 연장해 커서로 늘어놓은 자리가 다른 road r1334·r1202 와 겹침)
#   - 검사 대상: 노선 road 만이 아니라 맵 전체 차선(s38 과 같음)
#   - 검사점: 실제 OSM 폴리곤 안쪽 1m 격자 + 둘레 1m 간격(외접 사각형보다 덜 보수적, 사각형 결과도 같이 낸다)
#   - 불합격 건물은 자동으로 옮기지 않는다. 겹친 road·깊이를 남기고 사용자가 정한다
# 검사 규칙: R1 어떤 차선(Any)도 안 잡힘 / R2 주행 차선 가장자리까지 >= 5.0m (s38 2-2 조건) / 참고 R2' >= 0m
# 사용: .venv-carla/bin/python map/scripts/s39_building_draft.py [--xodr PATH] [--all]
import argparse
import csv
import hashlib
import math
import sys
from pathlib import Path

import carla

sys.path.insert(0, str(Path(__file__).parent))
from s22_buildings import KEY_NAMES, RAW_OSM, Frame, load_osm, poly_stats  # noqa: E402

BASE = Path(__file__).resolve().parent.parent
V2 = BASE / "maps/cbnu_internal_only_localtm_tags81_B_smooth_turnfix.xodr"
OUT = BASE / "data/processed/s39_building_draft.csv"
EDGE_GAP = 5.0


def inside(pt, poly):
    x, y = pt
    c = False
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        if (y0 > y) != (y1 > y) and x < (x1 - x0) * (y - y0) / (y1 - y0) + x0:
            c = not c
    return c


def check_points(poly, step=1.0):
    pts = []
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        k = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
        pts += [(x0 + (x1 - x0) * j / k, y0 + (y1 - y0) * j / k) for j in range(k)]
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    x = math.floor(min(xs))
    while x <= max(xs):
        y = math.floor(min(ys))
        while y <= max(ys):
            if inside((x, y), poly):
                pts.append((x, y))
            y += step
        x += step
    return pts


def check(cmap, pts):
    hit, roads, best = 0, set(), (float("inf"), None)
    for x, y in pts:
        loc = carla.Location(x, y, 0.0)
        wp = cmap.get_waypoint(loc, project_to_road=False, lane_type=carla.LaneType.Any)
        if wp is not None:
            hit += 1
            roads.add(wp.road_id)
        wd = cmap.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
        d = wd.transform.location.distance(loc) - wd.lane_width / 2
        if d < best[0]:
            best = (d, wd.road_id)
    return hit, sorted(roads), best


def min_rect(poly):
    """CARLA 좌표 폴리곤의 최소 넓이 외접 사각형 꼭짓점(변 방향 후보 = 폴리곤 변). 메시 크기 맞춤용 사각형"""
    best = None
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        h = math.atan2(y1 - y0, x1 - x0)
        c, s = math.cos(h), math.sin(h)
        u = [x * c + y * s for x, y in poly]
        v = [-x * s + y * c for x, y in poly]
        area = (max(u) - min(u)) * (max(v) - min(v))
        if best is None or area < best[0]:
            best = (area, c, s, min(u), max(u), min(v), max(v))
    _, c, s, u0, u1, v0, v1 = best
    return [(u * c - v * s, u * s + v * c) for u, v in ((u0, v0), (u1, v0), (u1, v1), (u0, v1))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xodr", type=Path, default=V2)
    ap.add_argument("--all", action="store_true")
    a = ap.parse_args()
    text = a.xodr.read_text()
    sha = hashlib.sha256(text.encode()).hexdigest()[:16]
    frame = Frame(text)
    cmap = carla.Map(a.xodr.stem, text)
    print(f"xodr {a.xodr.name} sha16 {sha}, 오프라인 carla.Map, 규칙 R1 차선 겹침 0 / R2 주행 차선 가장자리 >= {EDGE_GAP}m")
    nodes, ways = load_osm(RAW_OSM)
    rows = []
    for wid, (nds, tags) in ways.items():
        name = tags.get("name")
        if "building" not in tags or not name or (not a.all and not any(k in name for k in KEY_NAMES)):
            continue
        ll = [nodes[i] for i in nds if i in nodes]
        if len(ll) < 3:
            continue
        if ll[0] == ll[-1]:
            ll = ll[:-1]
        poly = [frame.carla(*p) for p in ll]
        (cx, cy), area, L, W, yaw_tm = poly_stats([frame.tm.fwd(*p) for p in ll])
        (lon, lat), *_ = poly_stats([(p[1], p[0]) for p in ll])
        x, y = frame.carla(lat, lon)
        yaw = (-yaw_tm) % 180
        hp, rp, (dp, dpr) = check(cmap, check_points(poly))
        hr, rr, (dr, drr) = check(cmap, check_points(min_rect(poly)))
        ok = hp == 0 and dp >= EDGE_GAP
        rows.append(dict(name=name, way_id=wid, lat=f"{lat:.6f}", lon=f"{lon:.6f}", carla_x=f"{x:.2f}", carla_y=f"{y:.2f}",
                         yaw_deg=f"{yaw:.1f}", len_m=f"{L:.1f}", wid_m=f"{W:.1f}", area_m2=f"{area:.0f}",
                         poly_lane_hits=hp, poly_hit_roads=" ".join(map(str, rp)), poly_min_edge_m=f"{dp:.2f}",
                         poly_nearest_road=dpr, rect_lane_hits=hr, rect_hit_roads=" ".join(map(str, rr)),
                         rect_min_edge_m=f"{dr:.2f}", rect_nearest_road=drr,
                         draft="통과" if ok else ("차선 겹침" if hp else f"여유 {dp:.2f}m < {EDGE_GAP}m"),
                         mesh="미정", scale="미정(메시 실제 크기 미측정)", map_sha=sha))
    rows.sort(key=lambda r: r["name"])
    with open(OUT, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    for r in rows:
        print(f"{r['name']}: ({r['carla_x']},{r['carla_y']}) yaw {r['yaw_deg']} {r['len_m']}x{r['wid_m']}m | 폴리곤 겹침 {r['poly_lane_hits']}"
              f" {r['poly_hit_roads']} 여유 {r['poly_min_edge_m']}m(r{r['poly_nearest_road']}) | 사각형 겹침 {r['rect_lane_hits']}"
              f" 여유 {r['rect_min_edge_m']}m | {r['draft']}")
    print(f"건물 {len(rows)}동, 통과 {sum(r['draft'] == '통과' for r in rows)} -> {OUT}")


if __name__ == "__main__":
    main()
