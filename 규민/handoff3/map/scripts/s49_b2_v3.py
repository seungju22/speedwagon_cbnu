#!/usr/bin/env python3
# 10/10 B2 v3: 10/09 B2(s40_b2_cut.py) 방법을 v3 동결본(a17ab012…)에 다시 돌리고 배치 상자·높이·표현 후보를 붙인다(서버 없음)
# 대상: 10/04 1순위 30동(docs/logs/s25_buildings.csv tier=1). 깎는 영역 = 모든 road 의 차선+인도 띠 합집합 + 0.5m (s40 과 같음)
# 좌표: OSM 위경도 -> 지역 TM(xodr 머리말 lat_0/lon_0, s22 TM) -> + <offset> = xodr 평면 -> CARLA (x, -y). CARLA 위경도(GeoReference)는 안 씀
# 상자: 깎은 폴리곤 안에 주축(가장 큰 조각의 최소 외접 사각형 긴 축) 방향 직사각형을 최대 4개(깎은 면적 90% 덮으면 멈춤).
#   폭 MIN_W(4m) 미만 조각은 버림. 외접 사각형 하나는 오목한 동에서 도로·이웃 동을 덮어서(첫 시도 기록 logs/s49_b2_v3_try1.log) 쓰지 않음. layout 은 상자 1개 = 1행
#   중문 노선 2동(way765358663, 경영대학 본관)은 원래 폴리곤을 주축 차선(모든 road)+0.5m 밖으로 평행 이동(물리기)한 뒤 같은 깎기
# 높이: building:levels x 층고 4.0m (OSM 에 height·levels 둘 다 있는 건물의 height/levels 중앙값, 이 스크립트가 다시 셈)
#   levels 없으면 기본 층수 = OSM 건물 levels 태그 중앙값(이 스크립트가 다시 셈). 출처 열에 적음
# v2 대조: 같은 방법으로 v2 동결본(5240ca8b…)을 다시 깎아 s40 결과와 맞는지 보고, 차이를 바뀐 road(삭제·추가·형상 변경)로 나눈다
# 출력: data/buildings_layout_v3.csv, docs/logs/s49_b2_v3.log(stdout), docs/logs/s49_unnamed_buildings.md, docs/figures/s49_plan_*.png
# 사용: .venv-carla/bin/python map/scripts/s49_b2_v3.py
import csv
import hashlib
import math
import statistics
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
import numpy as np  # noqa: E402
import shapely  # noqa: E402
from shapely.affinity import rotate, translate  # noqa: E402
from shapely.geometry import Point, Polygon, box  # noqa: E402
from shapely.ops import unary_union  # noqa: E402

HERE = Path(__file__).resolve().parent
BASE = HERE.parent
sys.path.insert(0, str(HERE))
from s22_buildings import RAW_OSM, Frame, load_osm  # noqa: E402
from s34_curvature import analyze  # noqa: E402
import place_campus_layers as pcl  # noqa: E402

XODR3 = BASE / "maps/cbnu_campus_frozen_v3.xodr"
XODR2 = BASE / "maps/cbnu_campus_frozen_v2.xodr"
S25 = BASE / "docs/logs/s25_buildings.csv"
S40 = BASE / "data/processed/s40_b2_cut.csv"
OUT = BASE / "data/buildings_layout_v3.csv"
UNNAMED = BASE / "docs/logs/s49_unnamed_buildings.md"
FIG = BASE / "docs/figures"
MARGIN = 0.5
STEP = 10                       # s34 표본 0.02m 의 10개마다(0.2m), s40 과 같음
ROUTE_BLDG = {"765358663", "446440341"}   # 중문 노선이 지나는 2동(물리기)
MIN_W = 4.0                     # 상자 최소 폭. 이보다 가는 조각(분해 부스러기)은 버린다. 나 방식 정육면체 한 변 = 최소 치수라 가는 상자는 개수가 수백으로 늘어남
PUSH_MAX = 5.0                  # 물리기 한도. 넘거나 다른 동(깎은 폴리곤)과 겹치면 물리지 않고 깎기만(필요 거리는 기록)
for _f in ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",):
    if Path(_f).exists():
        matplotlib.font_manager.fontManager.addfont(_f)
plt.rcParams["font.family"] = ["Noto Sans CJK JP", "DejaVu Sans"]


def strip(rows, t_in, t_out):
    """s40 과 같은 띠 다각형(CARLA 좌표 = (x, -y))"""
    pick = rows[::STEP] + ([rows[-1]] if (len(rows) - 1) % STEP else [])
    left, right = [], []
    for r in pick:
        s, x, y, h, k, off, w, ws, tag = r
        for t, out in ((t_in(r), left), (t_out(r), right)):
            out.append((x - math.sin(h) * t, -(y + math.cos(h) * t)))
    p = Polygon(left + right[::-1])
    return p if p.is_valid else p.buffer(0)


def road_polys(text):
    """road id -> dict(full, lane, side, xml). full = 차선+인도, lane = lane -1, side = lane -2(있으면)"""
    root = ET.fromstring(text)
    out = {}
    for road in root.iter("road"):
        rid, L, rows, *_ = analyze(road)
        if len(rows) < 2:
            continue
        has2 = any(r[7] > 0 for r in rows)
        out[rid] = dict(full=strip(rows, lambda r: r[5], lambda r: r[5] - r[6] - r[7]),
                        lane=strip(rows, lambda r: r[5], lambda r: r[5] - r[6]),
                        side=strip(rows, lambda r: r[5] - r[6], lambda r: r[5] - r[6] - r[7]) if has2 else None,
                        xml=ET.tostring(road))
    return out


def inverse(frame, x, y, lat=36.63, lon=127.46):
    """CARLA (x, y) -> 위경도. Frame.carla 를 수치 미분 뉴턴 반복(s40_stops.inverse 와 같은 방법)"""
    for _ in range(20):
        fx, fy = frame.carla(lat, lon)
        ex, ey = x - fx, y - fy
        if math.hypot(ex, ey) < 1e-6:
            break
        d = 1e-6
        a = [(frame.carla(lat + d, lon)[i] - (fx, fy)[i]) / d for i in (0, 1)]
        b = [(frame.carla(lat, lon + d)[i] - (fx, fy)[i]) / d for i in (0, 1)]
        det = a[0] * b[1] - a[1] * b[0]
        lat += (ex * b[1] - ey * b[0]) / det
        lon += (a[0] * ey - a[1] * ex) / det
    return lat, lon


def rect_params(r):
    """최소 외접 사각형 -> (cx, cy, yaw 긴 축 deg [-90, 90), L, W)"""
    pts = list(r.exterior.coords)[:4]
    e1 = (pts[1][0] - pts[0][0], pts[1][1] - pts[0][1])
    e2 = (pts[2][0] - pts[1][0], pts[2][1] - pts[1][1])
    l1, l2 = math.hypot(*e1), math.hypot(*e2)
    e, L, W = (e1, l1, l2) if l1 >= l2 else (e2, l2, l1)
    yaw = math.degrees(math.atan2(e[1], e[0]))
    yaw = (yaw + 90) % 180 - 90
    c = r.centroid
    return c.x, c.y, yaw, L, W


def pieces(g):
    return [p for p in getattr(g, "geoms", [g]) if p.geom_type == "Polygon" and not p.is_empty]


def maxrect(m):
    """불 행렬에서 참으로만 된 가장 큰 직사각형 (넓이, 위, 왼, 아래, 오른) — 히스토그램 스택"""
    ny, nx = m.shape
    h = np.zeros(nx, int)
    best = (0, 0, 0, 0, 0)
    for j in range(ny):
        h = np.where(m[j], h + 1, 0)
        st = []
        for i in range(nx + 1):
            cur = h[i] if i < nx else 0
            start = i
            while st and st[-1][1] >= cur:
                k, hk = st.pop()
                if hk * (i - k) > best[0]:
                    best = (hk * (i - k), j - hk + 1, k, j, i - 1)
                start = k
            st.append((start, cur))
    return best


def decompose(C, yaw, cell=0.5, max_parts=4, cover=0.9, min_frac=0.05):
    """깎은 폴리곤 C 를 주축(yaw) 방향 직사각형 최대 max_parts 개로(모두 C 안). 반환 [(cx, cy, yaw, L, W)], 덮은 비율"""
    R = rotate(C, -yaw, origin=(0, 0))
    Rin = R.buffer(1e-6)
    x0, y0, x1, y1 = R.bounds
    nx, ny = math.ceil((x1 - x0) / cell), math.ceil((y1 - y0) / cell)
    X, Y = np.meshgrid(x0 + np.arange(nx + 1) * cell, y0 + np.arange(ny + 1) * cell)
    c = shapely.contains_xy(R.buffer(0.02), X, Y)
    avail = c[:-1, :-1] & c[1:, :-1] & c[:-1, 1:] & c[1:, 1:]
    parts, tries = [], 0
    while len(parts) < max_parts and tries < 20:
        tries += 1
        area, top, left, bot, right = maxrect(avail)
        if area == 0 or area * cell * cell < min_frac * C.area:
            break
        avail[top:bot + 1, left:right + 1] = False
        if min(bot - top + 1, right - left + 1) * cell < MIN_W:      # 가는 조각은 버린다(정육면체 수 폭증 방지)
            continue
        r = [x0 + left * cell, y0 + top * cell, x0 + (right + 1) * cell, y0 + (bot + 1) * cell]
        if not Rin.contains(box(*r)):          # 꼭짓점 검사로 들어온 칸이 오목한 홈을 걸치면 안쪽으로 줄인다
            for _ in range(10):
                r = [r[0] + 0.05, r[1] + 0.05, r[2] - 0.05, r[3] - 0.05]
                if Rin.contains(box(*r)):
                    break
        for k, dlt in ((0, -0.05), (1, -0.05), (2, 0.05), (3, 0.05)):   # 칸 격자로 잃은 가장자리를 0.5m 까지 되살림
            for _ in range(10):
                t = list(r)
                t[k] += dlt
                if not Rin.contains(box(*t)):
                    break
                r = t
        parts.append(r)
        if unary_union([box(*q) for q in parts]).area >= cover * C.area:
            break
    out = []
    ca, sa = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    for r in parts:
        mx, my = (r[0] + r[2]) / 2, (r[1] + r[3]) / 2
        L, W = r[2] - r[0], r[3] - r[1]
        cx, cy = mx * ca - my * sa, mx * sa + my * ca
        out.append((cx, cy, yaw, L, W) if L >= W else (cx, cy, (yaw + 90 + 90) % 180 - 90, W, L))
    cov = unary_union([pcl.rect(*p) for p in out]).area / C.area if out else 0.0
    return out, cov


def push_out(P, drive_cut, dmax=30.0):
    """물리기: P 를 평행 이동해 주행 차선 + 여유와 안 겹치는 가장 짧은 거리(0.1m, 5도 간격). 반환 (거리, 방위, dx, dy)"""
    local = drive_cut.intersection(P.buffer(dmax + 1))
    if P.intersection(local).area < 1e-4:
        return 0.0, 0, 0.0, 0.0
    d = 0.1
    while d <= dmax + 1e-9:
        for ang in range(0, 360, 5):
            dx, dy = d * math.cos(math.radians(ang)), d * math.sin(math.radians(ang))
            if translate(P, dx, dy).intersection(local).area < 1e-4:
                return round(d, 1), ang, dx, dy
        d += 0.1
    return None


def main():
    t3, t2 = XODR3.read_text(), XODR2.read_text()
    sha3, sha2 = (hashlib.sha256(t.encode()).hexdigest()[:16] for t in (t3, t2))
    f3, f2 = Frame(t3), Frame(t2)
    print(f"v3 {XODR3.name} sha {sha3} / v2 {XODR2.name} sha {sha2}")
    print(f"투영 v3: {f3.proj} offset ({f3.ox}, {f3.oy}) / v2 같음: {(f2.proj, f2.ox, f2.oy) == (f3.proj, f3.ox, f3.oy)}")
    print("좌표식: (lat, lon) -> TM(lat_0, lon_0, k0=1, WGS84)(E, N) -> xodr (E + ox, N + oy) -> CARLA (E + ox, -(N + oy))")

    R3, R2 = road_polys(t3), road_polys(t2)
    only2 = sorted(set(R2) - set(R3))
    only3 = sorted(set(R3) - set(R2))
    changed = sorted(r for r in set(R2) & set(R3) if R2[r]["xml"] != R3[r]["xml"])
    print(f"road v2 {len(R2)} / v3 {len(R3)}: v2 에만 {len(only2)}, v3 에만 {len(only3)} {only3}, 같은 id 바뀜 {len(changed)} {changed}")
    cut3 = unary_union([v["full"] for v in R3.values()]).buffer(MARGIN)
    cut2 = unary_union([v["full"] for v in R2.values()]).buffer(MARGIN)
    rroads, nroutes = pcl.route_roads()
    route_lane = unary_union([R3[r]["lane"] for r in rroads if r in R3])
    drive_cut = unary_union([v["lane"] for v in R3.values()]).buffer(MARGIN)      # 물리기 기준: 주행 차선(모든 road) + 여유
    print(f"깎는 영역 v3 {cut3.area:.0f}m2 / v2 {cut2.area:.0f}m2, 경로 {nroutes}개 road {len(rroads)}")
    # 바뀐 road 별 영역(차이 원인 찾기용)
    gone_by = {r: R2[r]["full"].buffer(MARGIN) for r in only2 + changed}
    new_by = {r: R3[r]["full"].buffer(MARGIN) for r in only3 + changed}

    nodes, ways = load_osm(RAW_OSM)
    # 층고·기본 층수 근거(OSM 전체 건물)
    ratio, levels_all = [], []
    for nds, tags in ways.values():
        if "building" not in tags:
            continue
        try:
            lv = float(tags.get("building:levels", ""))
        except ValueError:
            lv = None
        try:
            h = float(tags.get("height", "").replace("m", "").strip())
        except ValueError:
            h = None
        if lv:
            levels_all.append(lv)
        if lv and h:
            ratio.append(h / lv)
    floor_h = statistics.median(ratio)
    def_lv = statistics.median(levels_all)
    print(f"층고 {floor_h:.2f}m = OSM height/levels 중앙값({len(ratio)}동, 범위 {min(ratio):.2f}~{max(ratio):.2f}), "
          f"기본 층수 {def_lv:g} = OSM levels 중앙값({len(levels_all)}동)")

    s40 = {r["way_id"]: r for r in csv.DictReader(open(S40))}
    platforms = pcl.parse_platforms()
    out, draw_rows = [], []
    for b in csv.DictReader(open(S25)):
        if b["tier"] != "1":
            continue
        wid = b["osm_id"]
        nds, tags = ways[wid]
        ll = [nodes[i] for i in nds if i in nodes]
        if ll[0] == ll[-1]:
            ll = ll[:-1]
        P = Polygon([f3.carla(*p) for p in ll])
        if not P.is_valid:
            P = P.buffer(0)
        Pc = Polygon([(lon, lat) for lat, lon in ll]).centroid      # OSM 원래 위경도 면적 중심(링크용)
        C3, C2 = P.difference(cut3), P.difference(cut2)
        a0, a3, a2 = P.area, C3.area, C2.area
        r3 = max(0.0, 1 - a3 / a0)
        r2 = max(0.0, 1 - a2 / a0)
        reasons = []
        if abs(a3 - a2) > 0.05:
            for r, g in gone_by.items():
                x = P.intersection(g).difference(cut3).area
                if x > 0.05:
                    reasons.append(f"r{r} {'삭제' if r in only2 else '형상 변경'} +{x:.1f}m2 덜 깎임")
            for r, g in new_by.items():
                x = P.intersection(g).difference(cut2).area
                if x > 0.05:
                    reasons.append(f"r{r} {'추가' if r in only3 else '형상 변경'} -{x:.1f}m2 더 깎임")
        on_lane3 = P.intersection(route_lane).area
        push = ""
        Pp = P
        if wid in ROUTE_BLDG:                       # 차도 밖으로 물린다(원래 폴리곤 평행 이동 뒤 같은 깎기)
            pr = push_out(P, drive_cut)
            # 다른 동은 깎은 폴리곤 기준(그 동도 도로에 깎이므로 원래 폴리곤으로 막으면 이미 없는 부분에 막힌다)
            others = unary_union([Polygon([f3.carla(*nodes[i]) for i in ways[o["osm_id"]][0] if i in nodes]).buffer(0).difference(cut3)
                                  for o in csv.DictReader(open(S25)) if o["tier"] == "1" and o["osm_id"] != wid])
            hit = pr is not None and translate(P, pr[2], pr[3]).intersection(others).area > 0.01
            if pr is None or pr[0] > PUSH_MAX or hit:
                need = "30m 넘음" if pr is None else f"{pr[0]}m 방위 {pr[1]}도"
                why = "한도 밖" if pr is None or pr[0] > PUSH_MAX else ""
                why += (", " if why else "") + ("다른 동과 겹침 " + f"{translate(P, pr[2], pr[3]).intersection(others).area:.1f}m2" if hit else "")
                push = f"안 물림(필요 {need}, {why}, 한도 {PUSH_MAX:g}m) -> 깎기만"
            else:
                Pp = translate(P, pr[2], pr[3])
                push = f"{pr[0]}m CARLA 방위 {pr[1]}도(dx {pr[2]:+.2f}, dy {pr[3]:+.2f})"
                C3 = Pp.difference(cut3)
        ps = sorted(pieces(C3), key=lambda p: -p.area)
        yaw = rect_params(ps[0].minimum_rotated_rectangle)[2]
        parts, cov = decompose(C3, yaw)
        lv_tag = tags.get("building:levels", "")
        try:
            lv = float(lv_tag)
            src = f"OSM building:levels {lv_tag} x {floor_h:g}m"
        except ValueError:
            lv = def_lv
            src = f"기본 {def_lv:g}층 x {floor_h:g}m (levels 태그 없음)"
        H = lv * floor_h
        name = tags.get("name", "")
        boxes = [pcl.rect(*q) for q in parts]
        bu = unary_union(boxes)
        brow = []
        for k, (cx, cy, pyaw, L, W) in enumerate(parts, 1):
            lat_c, lon_c = inverse(f3, cx, cy)
            rt = f3.carla(lat_c, lon_c)
            fit = pcl.fit_mesh(L, H)
            a, nl, nw, nh = pcl.cube_plan(L, W, H)
            brow.append(dict(name=name, code=b["code"], way_id=wid, part=k, parts=len(parts),
                       center_carla_x=round(cx, 3), center_carla_y=round(cy, 3),
                       center_lat=f"{lat_c:.9f}", center_lon=f"{lon_c:.9f}", latlon_roundtrip_m=round(math.dist(rt, (cx, cy)), 4),
                       yaw_deg=round(pyaw, 2), length_m=round(L, 2), width_m=round(W, 2), height_m=round(H, 1),
                       levels_tag=lv_tag, osm_height_tag=tags.get("height", ""), height_src=src,
                       area_osm_m2=round(a0, 1), area_cut_m2=round(C3.area, 1), cut_ratio=round(max(0.0, 1 - C3.area / a0), 4),
                       pieces=len(ps), boxes_cover_cut=round(cov, 3), boxes_cover_osm=round(bu.intersection(P).area / a0, 3),
                       push_out=push, route_lane_overlap_osm_m2=round(on_lane3, 2),
                       box_route_lane_overlap_m2=round(bu.intersection(route_lane).area, 3),
                       v2_cut_ratio=round(r2, 4), v2_s40_cut_ratio=s40[wid]["cut_ratio"] if wid in s40 else "",
                       v3_minus_v2_cut_m2=round(a2 - a3, 1), change_reason="; ".join(reasons),
                       repr_a=f"{fit['mesh']} x{fit['scale']} (길이 {fit['len_err_m']:+.1f}m, 높이 {fit['h_err_m']:+.1f}m, 폭 미측정)",
                       repr_b=f"정육면체 {a:.2f}m x {nl * nw * nh}개({nl}x{nw}x{nh})", repr_c="", repr_c_why="",
                       osm_lat=f"{Pc.y:.7f}", osm_lon=f"{Pc.x:.7f}"))
        cs = [pcl.choose_c(r, platforms) for r in brow]
        pick = min(cs, key=lambda c: (c[0] != "가", c[1]))
        for r in brow:
            r["repr_c"], r["repr_c_why"] = pick
        out += brow
        draw_rows.append(dict(row=brow[0], P=P, Pp=Pp, C=C3, B=boxes))

    out.sort(key=lambda r: (r["way_id"] not in ROUTE_BLDG, -r["cut_ratio"], r["name"] or "~", r["part"]))
    with open(OUT, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(out[0]))
        w.writeheader()
        w.writerows(out)
    bl = [r for r in out if r["part"] == 1]

    print("\n== 30동 (노선 2동 먼저, 그다음 깎인 비율 큰 순). 상자 = 깎은 폴리곤 안 직사각형(최대 4) ==")
    for r in bl:
        ps = [q for q in out if q["way_id"] == r["way_id"]]
        dims = ", ".join(f"{q['length_m']}x{q['width_m']}" for q in ps)
        print(f"{r['name'] or '(이름 없음)'} way{r['way_id']}: 깎임 {r['cut_ratio']*100:.1f}% (v2 {r['v2_cut_ratio']*100:.1f}%, s40 기록 "
              f"{float(r['v2_s40_cut_ratio'] or 0)*100:.1f}%), 조각 {r['pieces']}, 상자 {r['parts']}개 [{dims}] 높이 {r['height_m']}m "
              f"yaw {r['yaw_deg']}, 덮음 깎은것의 {r['boxes_cover_cut']*100:.0f}%/OSM의 {r['boxes_cover_osm']*100:.0f}%, "
              f"물림 {r['push_out'] or '-'}, 경로 차선 겹침 원래 {r['route_lane_overlap_osm_m2']}m2 -> 상자 {r['box_route_lane_overlap_m2']}m2, "
              f"다={r['repr_c']}({r['repr_c_why']})")
    rep = [r for r in bl if abs(r["v2_cut_ratio"] - float(r["v2_s40_cut_ratio"] or 0)) > 1e-4]
    print(f"\nv2 재현(s40 기록과 깎인 비율 차 > 0.01%p): {len(rep)}동")
    ch = [r for r in bl if abs(r["v3_minus_v2_cut_m2"]) > 0.05]
    print(f"v2 -> v3 깎인 면적이 바뀐 동 {len(ch)} (물리기 전 원래 폴리곤 기준)")
    for r in ch:
        print(f"  {r['name'] or '(이름 없음)'} way{r['way_id']}: {r['v2_cut_ratio']*100:.1f}% -> 원래 위치 v3 깎임 "
              f"(남은 면적 차 v2-v3 {r['v3_minus_v2_cut_m2']:+.1f}m2) 원인: {r['change_reason']}")
    print(f"깎임 0: {sum(r['cut_ratio'] == 0 for r in bl)}/30, 최대 {max(r['cut_ratio'] for r in bl)*100:.1f}%, 50% 초과 {sum(r['cut_ratio'] > 0.5 for r in bl)}")
    print(f"상자 수: 합 {len(out)}, 1개 {sum(r['parts'] == 1 for r in bl)}동, 2개 {sum(r['parts'] == 2 for r in bl)}, "
          f"3개 {sum(r['parts'] == 3 for r in bl)}, 4개 {sum(r['parts'] == 4 for r in bl)}; 깎은 폴리곤 덮음 최소 {min(r['boxes_cover_cut'] for r in bl)*100:.0f}%")
    print(f"높이 출처: 태그 {sum(1 for r in bl if r['levels_tag'])}, 기본값 {sum(1 for r in bl if not r['levels_tag'])}")
    print(f"위경도 왕복 최대 {max(r['latlon_roundtrip_m'] for r in out):.4f}m")
    allb = [(d["row"]["way_id"], b) for d in draw_rows for b in d["B"]]
    ov = sorted({(a[0], c[0], round(a[1].intersection(c[1]).area, 1)) for i, a in enumerate(allb) for c in allb[i + 1:]
                 if a[0] != c[0] and a[1].intersection(c[1]).area > 0.01})
    print(f"다른 동 상자끼리 겹침 {len(ov)} {ov}")
    cutc = unary_union([v["full"] for v in R3.values()])
    bad = [(w, round(b.intersection(cut3).area, 3)) for w, b in allb if b.intersection(cut3).area > 1e-3]
    print(f"상자 vs 도로(차선+인도)+0.5m 겹침 {len(bad)} {bad}, 상자 vs 도로 자체 겹침 {sum(1 for w, b in allb if b.intersection(cutc).area > 1e-3)}")

    # 이름 없는 동 목록(OSM 원래 위경도 면적 중심)
    lines = ["# 10/10 이름 없는 동 (10/04 1순위 30동 중) — 이름은 직접 채운다", "",
             "위경도 = OSM 폴리곤 원래 위경도의 면적 중심(WGS84). CARLA 위경도 아님", ""]
    for r in bl:
        if r["name"]:
            continue
        lab = f"way{r['way_id']}"
        lines += [f"- {lab} (OSM building={ways[r['way_id']][1].get('building')}, 바닥 {r['area_osm_m2']}m2, "
                  f"상자 {r['length_m']}x{r['width_m']}m, 높이 {r['height_m']}m {r['height_src']})",
                  f"  - 지도: https://map.kakao.com/link/map/{lab},{r['osm_lat']},{r['osm_lon']}",
                  f"  - 로드뷰: https://map.kakao.com/link/roadview/{r['osm_lat']},{r['osm_lon']}",
                  "  - 이름: "]
    UNNAMED.write_text("\n".join(lines) + "\n")
    print(f"이름 없는 동 {sum(1 for r in bl if not r['name'])} -> {UNNAMED.name}")

    draw_plans(draw_rows, R3, rroads, platforms)
    print(f"-> {OUT}")


def zones(draw_rows, platforms, span_max=330.0):
    """건물(원래 폴리곤 중심)·승강장 점을 넓이가 span_max 를 넘으면 긴 쪽 중앙값으로 반씩 나눈다"""
    pts = [(d["P"].centroid.x, d["P"].centroid.y, ("b", i)) for i, d in enumerate(draw_rows)] + \
          [(p["x"], p["y"], ("p", i)) for i, p in enumerate(platforms)]
    def split(g):
        xs, ys = [p[0] for p in g], [p[1] for p in g]
        if max(max(xs) - min(xs), max(ys) - min(ys)) <= span_max or len(g) < 2:
            return [g]
        k = 0 if max(xs) - min(xs) >= max(ys) - min(ys) else 1
        g = sorted(g, key=lambda p: p[k])
        h = len(g) // 2
        return split(g[:h]) + split(g[h:])
    return sorted(split(pts), key=lambda g: (min(p[1] for p in g), min(p[0] for p in g)))


def draw_plans(draw_rows, R3, rroads, platforms):
    gs = zones(draw_rows, platforms)
    names = []
    for zi, g in enumerate(gs, 1):
        bi = [p[2][1] for p in g if p[2][0] == "b"]
        geo = [draw_rows[i]["P"].union(draw_rows[i]["Pp"]) for i in bi] + [Point(p[0], p[1]).buffer(15) for p in g if p[2][0] == "p"]
        x0, y0, x1, y1 = unary_union(geo).bounds
        pad = 25
        x0, y0, x1, y1 = x0 - pad, y0 - pad, x1 + pad, y1 + pad
        win = box(x0, y0, x1, y1)
        span = max(x1 - x0, y1 - y0)
        fig, ax = plt.subplots(figsize=(10 * (x1 - x0) / span + 3.2, 10 * (y1 - y0) / span + 1.0))
        for rid, v in R3.items():
            if not v["full"].intersects(win):
                continue
            if v["side"] is not None:
                for p in pieces(v["side"].intersection(win)):
                    ax.fill(*p.exterior.xy, color="0.86", lw=0)
            for p in pieces(v["lane"].intersection(win)):
                ax.fill(*p.exterior.xy, color="#1baf7a" if rid in rroads else "0.62", alpha=0.6 if rid in rroads else 1, lw=0)
        for i in bi:
            d = draw_rows[i]
            for p in pieces(d["C"]):
                ax.fill(*p.exterior.xy, color="#2a78d6", alpha=0.35, lw=0)
            ax.plot(*d["P"].exterior.xy, color="#eb6834", lw=1.2)
            if not d["Pp"].equals(d["P"]):
                ax.plot(*d["Pp"].exterior.xy, color="#eb6834", lw=1.0, ls=":")
            for bx in d["B"]:
                ax.plot(*bx.exterior.xy, color="0.1", lw=1.0, ls="--")
            r = d["row"]
            lab = (r["name"] or f"way{r['way_id']}").split(" (")[0]
            extra = ("\n물림 " + r["push_out"].split(" CARLA")[0]) if r["push_out"] and not r["push_out"].startswith("안") else ("\n안 물림" if r["push_out"] else "")
            c = d["P"].representative_point()
            ax.annotate(f"{lab}\n깎임 {r['cut_ratio']*100:.1f}%{extra}", (c.x, c.y), fontsize=7, ha="center", va="center", color="0.1")
        for p in g:
            if p[2][0] == "p":
                pl = platforms[p[2][1]]
                ax.plot(pl["x"], pl["y"], marker="^", color="#d55181", ms=9, mec="white", mew=1.5, ls="")
                ax.annotate(pl["label"], (pl["x"], pl["y"]), xytext=(5, -10), textcoords="offset points", fontsize=7, color="0.1")
        ax.set_xlim(x0, x1)
        ax.set_ylim(y1, y0)                 # 위 = 북
        ax.set_aspect("equal")
        ax.set_xlabel("CARLA x (m)")
        ax.set_ylabel("CARLA y (m), 위 = 북")
        ax.legend(handles=[Line2D([], [], color="#1baf7a", lw=6, alpha=0.6, label="경로 차선(5노선+30쌍)"),
                           Line2D([], [], color="0.62", lw=6, label="그 밖 차선"),
                           Line2D([], [], color="0.86", lw=6, label="인도"),
                           Line2D([], [], color="#eb6834", lw=1.2, label="OSM 원래 폴리곤"),
                           Line2D([], [], color="#eb6834", lw=1.0, ls=":", label="물린 폴리곤(중문 2동)"),
                           Line2D([], [], color="#2a78d6", lw=6, alpha=0.35, label="깎은 폴리곤(차선+인도+0.5m)"),
                           Line2D([], [], color="0.1", lw=1, ls="--", label="배치 상자"),
                           Line2D([], [], color="#d55181", marker="^", ls="", label="승강장")],
                  loc="upper left", bbox_to_anchor=(1.01, 1), fontsize=8, borderaxespad=0)
        nb, npf = len(bi), sum(1 for p in g if p[2][0] == "p")
        ax.set_title(f"구역 {zi}: 건물 {nb}동, 승강장 {npf}곳 (v3 a17ab012)", fontsize=10)
        fig.tight_layout()
        path = FIG / f"s49_plan_zone{zi}.png"
        fig.savefig(path, dpi=120)
        plt.close(fig)
        names.append((path.name, nb, npf, (round(x0), round(y0), round(x1), round(y1))))
    for n in names:
        print(f"그림 {n[0]}: 건물 {n[1]}, 승강장 {n[2]}, 범위 {n[3]}")


if __name__ == "__main__":
    main()
