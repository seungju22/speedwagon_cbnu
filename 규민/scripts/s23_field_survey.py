#!/usr/bin/env python3
# 맵세션23: 현장 답사 지점 좌표 추출 (읽기전용, 서버 없음, xodr/OSM 미수정).
# 원본 OSM(위경도 그대로)과 frozen_v1(road 위치 -> 위경도 역변환)에서 지점을 뽑고,
# 정문에서 출발해 가까운 지점 순(최근접 이웃)으로 도보 동선을 만든다.
# 출력: 표준출력(key: value 줄). field_survey_2026-10-04.md 작성의 근거 자료
# 실행: .venv-carla/bin/python map/scripts/s23_field_survey.py > map/docs/logs/s23_field_survey.log
import math
import sys

sys.path.insert(0, "/home/gyumin/campus_mobility_sim/map/scripts")
from s19_scope_common import RAW_OSM, XODR_V1, load_osm, load_map, ll_to_carla, tm, OFF_X, OFF_Y, LAT0, LON0  # noqa

EXCL8 = ["442595035", "442595850", "442709215", "442710666", "442710667", "471667693",
         "481950061", "481950064"]
BACK6 = ["481945510", "481943505", "481475019", "452870644", "481945509", "481943503"]
LOOP = ["452870648", "452870641"]
GATE_EXIT = "481950060"
MIDDLE_NODE = "4437127634"
MIDDLE_WAYS = ["446440352", "446440353"]
# 회차 없는 막다른 끝 14곳: (junction, OSM 노드, 끝 way) — s23_deadend_cause.log
DEAD14 = [("j4", "3958352377", "392632035"), ("j15", "4397573124", "442065041"),
          ("j17", "4397573135", "442065042"), ("j26", "4397573198", "442065062"),
          ("j55", "4402717760", "442595040"), ("j56", "4658272859", "442595852"),
          ("j75", "4403766708", "442709216"), ("j83", "4404203144", "442761502"),
          ("j89", "4492202733", "452421704"), ("j92", "4492202737", "452421706"),
          ("j97", "4631209792", "468616610"), ("j98", "4655490507", "471400786"),
          ("j99", "4655490541", "471400793"), ("j104", "4784199633", "472252990")]
GATE_NODE = "4748296080"     # 정문 안 j1
SLOPE = {"정문->큰 사거리(j42)": [1247, 1914, 1356, 1917, 1355],
         "큰 사거리->j10 (남문·양성재 공통)": [1446, 1172],
         "j10->양성재 정차점(사용자 지적 오르막)": [1427, 1127, 1126, 1125, 1124, 1123],
         "큰 사거리->북문 정차점": [1447, 1373, 1374, 1375, 1376, 1377, 1378],
         "j10->남문 정차점": [1428, 1240, 1239, 1238, 1237],
         "북문 갈림->중문 정차점": [1283, 1284, 1285, 1286, 1327]}
CURVE_F = 1786               # 양성재 경유 경로 최소 평균R 5.54m (s22_routes.log)
DIRS = ["북", "북동", "동", "남동", "남", "남서", "서", "북서"]


def carla_to_ll(x, y):
    tx, ty = x - OFF_X, -y - OFF_Y
    lat, lon = LAT0, LON0
    for _ in range(8):
        ex, ey = tm(lat, lon)
        lat += (ty - ey) / 110950.0
        lon += (tx - ex) / (110950.0 * math.cos(math.radians(lat)))
    return lat, lon


def bearing(a, b):
    """위경도 a -> b 방위(도, 북=0 시계방향)."""
    dy = (b[0] - a[0]) * 110950.0
    dx = (b[1] - a[1]) * 110950.0 * math.cos(math.radians(a[0]))
    return math.degrees(math.atan2(dx, dy)) % 360


def compass(deg):
    return DIRS[int((deg + 22.5) // 45) % 8]


def dist(a, b):
    dy = (b[0] - a[0]) * 110950.0
    dx = (b[1] - a[1]) * 110950.0 * math.cos(math.radians(a[0]))
    return math.hypot(dx, dy)


def fmt(p):
    return f"{p[0]:.6f}, {p[1]:.6f}"


def main():
    nodes, ways = load_osm(RAW_OSM)
    names = []
    for wid, w in ways.items():
        t = w["tags"]
        if "building" in t and t.get("name") and all(n in nodes for n in w["nds"]):
            pts = [nodes[n] for n in w["nds"]]
            names.append((t["name"], (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))))

    def near_bld(p):
        n, c = min(names, key=lambda x: dist(p, x[1]))
        return f"{n} {dist(p, c):.0f}m {compass(bearing(c, p))}쪽"

    def wlen(w):
        return sum(dist(nodes[a], nodes[b]) for a, b in zip(w["nds"], w["nds"][1:]))

    pts = []   # (key, 분류, 위경도, 설명 dict)
    for wid in EXCL8:
        w = ways[wid]
        a, b = nodes[w["nds"][0]], nodes[w["nds"][-1]]
        mid = nodes[w["nds"][len(w["nds"]) // 2]]
        pts.append((f"A-{wid}", "A", mid, dict(start=fmt(a), end=fmt(b), tags=w["tags"], len=wlen(w),
                                                  run=compass(bearing(a, b)), bld=near_bld(mid))))
    # B: 고리 범위, 고리-본망 접점(후문 6 way 가 붙는 노드), 경계 최근접점
    loop_nodes = [nodes[n] for wid in LOOP for n in ways[wid]["nds"]]
    lat = [p[0] for p in loop_nodes]
    lon = [p[1] for p in loop_nodes]
    print(f"B 고리 범위: 위도 {min(lat):.6f}~{max(lat):.6f} 경도 {min(lon):.6f}~{max(lon):.6f}")
    loop_set = {n for wid in LOOP for n in ways[wid]["nds"]}
    for wid in BACK6:
        nd = ways[wid]["nds"]
        print(f"B way{wid} {ways[wid]['tags'].get('oneway', '양방향')} {wlen(ways[wid]):.0f}m 시작 {fmt(nodes[nd[0]])}"
              f" 끝 {fmt(nodes[nd[-1]])} 고리노드공유 {[n for n in nd if n in loop_set]}")
    j106 = nodes["4658272907"]   # 본망 쪽 접점(road1220 끝 j106, 회차 r1898)
    near_loop = min(loop_nodes, key=lambda p: dist(p, j106))
    pts.append(("B-1 본망 끝(j106)", "B", j106, dict(other=fmt(near_loop), gap=dist(j106, near_loop),
                                                   bld=near_bld(j106))))
    jn = nodes["4496854212"]     # 고리와 452870644·392632034 만나는 노드
    pts.append(("B-2 고리 접점(452870644)", "B", jn, dict(bld=near_bld(jn))))
    # C
    w = ways[GATE_EXIT]
    a, b = nodes[w["nds"][0]], nodes[w["nds"][-1]]
    pts.append(("C 정문 출구", "C", b, dict(start=fmt(a), end=fmt(b), len=wlen(w), run=compass(bearing(a, b)),
                                         tags=w["tags"])))
    # D
    m = nodes[MIDDLE_NODE]
    far = []
    for wid in MIDDLE_WAYS:
        nd = ways[wid]["nds"]
        e = nd[-1] if nd[0] == MIDDLE_NODE else nd[0]
        far.append((wid, nodes[e], wlen(ways[wid])))
    pts.append(("D 중문 끝점", "D", m, dict(far=[(wid, fmt(p), f"{L:.0f}m", f"직선 {dist(m, p):.0f}m {compass(bearing(m, p))}")
                                              for wid, p, L in far],
                                          bus=f"충북대학교중문 정류장 직선 {dist(m, (36.6336022, 127.4605061)):.0f}m")))
    # E
    for j, n, wid in DEAD14:
        nd = ways[wid]["nds"]
        i = nd.index(n)
        prev = nd[1] if i == 0 else nd[i - 1]
        into = compass(bearing(nodes[prev], nodes[n]))
        others = [x for x, ww in ways.items() if n in ww["nds"] and x != wid and "highway" in ww["tags"]]
        pts.append((f"E-{j}", "E", nodes[n], dict(into=into, back=compass(bearing(nodes[n], nodes[prev])),
                                                  bld=near_bld(nodes[n]),
                                                  cont=[(x, ways[x]["tags"].get("highway"),
                                                         ways[x]["tags"].get("service", "")) for x in others])))
    # F
    cm = load_map(XODR_V1)
    L = float([r for r in __import__("s19_scope_common").road_lengths(XODR_V1).items() if r[0] == CURVE_F][0][1][0])
    wps = [cm.get_waypoint_xodr(CURVE_F, -1, s) for s in (0.0, L / 2, L - 0.01)]
    lls = [carla_to_ll(w.transform.location.x, w.transform.location.y) for w in wps]
    pts.append(("F r1786 커브", "F", lls[1], dict(enter=compass(bearing(lls[0], lls[1])), exit=compass(bearing(lls[1], lls[2])),
                                                bld=near_bld(lls[1]), start=fmt(lls[0]), end=fmt(lls[2]))))
    # 검산: 역변환 왕복 오차
    x, y = ll_to_carla(*lls[1])
    print(f"검산 역변환 왕복 오차 {math.hypot(x - wps[1].transform.location.x, y - wps[1].transform.location.y):.3f}m")
    # G
    lens = __import__("s19_scope_common").road_lengths(XODR_V1)
    for k, rs in SLOPE.items():
        a = cm.get_waypoint_xodr(rs[0], -1, 0.0).transform.location
        b = cm.get_waypoint_xodr(rs[-1], -1, lens[rs[-1]][0] - 0.01).transform.location
        la, lb = carla_to_ll(a.x, a.y), carla_to_ll(b.x, b.y)
        print(f"G {k}: 시작 {fmt(la)} 끝 {fmt(lb)} 직선 {dist(la, lb):.0f}m")

    # 도보 동선: 정문에서 최근접 이웃
    cur = nodes[GATE_NODE]
    todo = list(pts)
    order, total = [], 0.0
    while todo:
        nxt = min(todo, key=lambda p: dist(cur, p[2]))
        d = dist(cur, nxt[2])
        total += d
        order.append((nxt, d))
        todo.remove(nxt)
        cur = nxt[2]
    total += dist(cur, nodes[GATE_NODE])
    print(f"\n동선 {len(order)}지점, 직선 합 {total:.0f}m (정문 복귀 포함)")
    for i, ((key, cat, p, info), d) in enumerate(order, 1):
        print(f"\n#{i} [{cat}] {key} | {fmt(p)} | 앞 지점에서 직선 {d:.0f}m")
        for k, v in info.items():
            print(f"  {k}: {v}")


if __name__ == "__main__":
    main()
