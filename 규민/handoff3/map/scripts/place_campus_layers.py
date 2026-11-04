#!/usr/bin/env python3
# 캠퍼스 장식 레이어 배치 (B3 초안, 맵세션49, 세션50 결정 반영). xodr·stops·test_drive.py 는 읽기만 한다
# 레이어: buildings, busstop, ground, trees, parked, props. buildings(가/나/다)·busstop·ground 구현, 나머지는 빈 레이어
# 세션50 결정: 건물 표현 기본값 나(가·다는 --repr 옵션). 도서관 남문(이쪽) 정류장 시설은 뺀다(BUSSTOP_SKIP)
# 세션52: 정류장 시설 180도(열린 면이 차도 쪽, BUS_YAW_OFF), --repr 가·다 에서 경로 침범 동은 나(GA_TO_NA)
#   ground(지면 판)는 기본 끔(--layers 에 넣어야 켜짐). 켜면 차도 이탈이 낙하로 안 보이므로 이탈 판정은 차선 중심 옆 벗어남으로 한다
#   --mesh-sizes: layers_bench.py 가 잰 메시 실제 크기(JSON)로 MESHES·CUBE·BUS·GROUND 값을 바꾼다(가 의 폭 정사각형 가정이 풀림)
# 데이터 흐름
#   buildings: data/buildings_layout_v3.csv(s49_b2_v3.py) 의 상자 중심 위경도 -> xodr 머리말 투영(s22 Frame) -> CARLA (x, -y)
#              --repr 가 = 패키지 건물 메시 1개(배율 하나), 나 = 정육면체 메시를 겹쳐 쌓은 덩어리, 다 = 행마다 layout 의 repr_c 열
#   busstop:   data/stops_v3.yaml 승강장 11곳. carla.Map 의 lane -1 중심에서 오른쪽(인도 쪽)으로 띄워 차로와 나란히
# 침범 검사: 모든 배치물 바닥 사각형 vs 주행 차선(carla.Map, 차선 폭 3.35m) + 여유 0.5m
#   경로 차로 = test_drive.py ROUTES_V3S46(5노선 + 30쌍) 의 road lane -1. 기준은 경로 차로 침범 0
# 서버 없이(--dry-run) 차선 모양은 carla.Map(이름, xodr 문자열)로 읽는다
# 사용
#   --dry-run : 서버 없이 배치 목록·침범 검사·그림(map/docs/logs/campus_layers_dry_<repr>.*, figures/campus_layers_dry_<repr>*.png)
#   (기본)    : 서버에 소환. 서버 맵 sha 가 --xodr 과 다르면 멈춤. 경로 차로 침범이 있는 배치물은 소환하지 않음
#   --remove  : 상태 파일(--state)에 적힌 actor 만 지운다(type_id 가 같을 때만)
#   --measure : 서버 소환 때 소환 전·레이어마다 동기 100틱 ms/틱(렌더 켬·끔)과 avail 을 상태 파일에 남김(B1 과 같은 방법)
# 서버 쪽(소환·지우기·측정)은 이 세션에서 실행해 보지 않았다(서버 없음). 문법·--help·--dry-run 만 확인
# 예: .venv-carla/bin/python map/scripts/place_campus_layers.py --layers buildings,busstop --repr 나 --dry-run
import argparse
import csv
import hashlib
import json
import math
import random
import re
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
BASE = HERE.parent
sys.path.insert(0, str(HERE))
from s22_buildings import Frame  # noqa: E402

XODR_V3 = BASE / "maps/cbnu_campus_frozen_v3.xodr"
LAYOUT = BASE / "data/buildings_layout_v3.csv"
STOPS = BASE / "data/stops_v3.yaml"
STATE = BASE / "docs/logs/campus_layers_state.json"
LOGS = BASE / "docs/logs"
FIGS = BASE / "docs/figures"
LAYERS = ["buildings", "busstop", "ground", "trees", "parked", "props"]
SEED = 49
MARGIN = 0.5            # 차선 가장자리 여유. 근거: fusorosa 폭 3.944m - 차선 3.35m = 양쪽 0.297m 차체 돌출 + 0.2m
                        # (B2 깎기 여유 0.5m 와 같은 값. r1391 추종 이탈 0.626m 같은 급커브 구간은 이 여유로 못 덮는다)
LANE_STEP = 0.5         # 차선 표본 간격(m)

# 가: 패키지 건물 메시. 크기는 세션38(B1) API bbox(2*extent) 값. x 축은 화면과 안 맞아 믿지 않는다 -> 폭 미측정
#   length = bbox y, height = bbox z, pivot 오프셋 = bbox 중심(x, y). 피벗 높이는 바닥(z 오프셋 = 높이/2 로 확인)
MESHES = {
    "House01": dict(path="/Game/Carla/Static/Building/SM_House01.SM_House01", length=15.436, height=12.646, off=(0.0, -0.001)),
    "House02": dict(path="/Game/Carla/Static/Building/SM_House02.SM_House02", length=11.690, height=8.476, off=(0.008, -0.137)),
    "Apartment03": dict(path="/Game/Carla/Static/Building/Apartment03/SM_Apartment03_v01_merged.SM_Apartment03_v01_merged",
                        length=29.870, height=32.256, off=(-7.934, 5.996)),
}
# 나: 정육면체 메시. B1 bbox 0.222x1.397x1.000(축 불신), 화면상 약 1m -> 한 변 1.0m 는 가정(미측정). 피벗은 중심(B1 오프셋 0,0,0)
CUBE = dict(path="/Game/Carla/Static/Ground/GroundCube.GroundCube", side=1.0)
BUS_GAP = MARGIN + 0.3  # 정류장 시설 가까운 변 ~ 차선 가장자리. 여유 0.5m 에 딱 맞추면 곡선·부동소수로 0.487~0.5m 가 되어(세션49 첫 dry-run) 0.3m 더함
# busstop: B1 API bbox 2.024(x) x 3.809(y) x 2.739(z), 중심 오프셋 (0.367, 0.177). 축 불신이라 바닥은 오프셋만큼 넓힌 사각형
BUS = dict(bp="static.prop.busstop", along=3.809 + 2 * 0.177, depth=2.024 + 2 * 0.367)
# 세션51 LiDAR: 긴 축은 메시 x(3.598m), 깊이 y(1.884m), 피벗->중심 (0.235, 0.175). 그래서 yaw = 차로 방향 + BUS_YAW_OFF
#   (세션49 의 '차로 - 90' 은 긴 축을 차로에 직각으로 세웠다. 세션51 정문 B 화면으로 확인). 열린 면 방향은 화면으로 따로 본다
# 세션52 사용자 결정: 180. 세션51 retest_busstop_*_B(인도에서 차로를 봄)에 뒷유리 없이 기둥·의자가 보여 열린 면이 인도 쪽이었다.
#   180 돌려 열린 면이 차도를 보게. 바닥 사각형은 대칭이라 침범 검사 결과는 같고, 피벗 오프셋은 myaw 로 같이 돈다. 화면 확인은 통합 S4
BUS_YAW_OFF = 180.0
BUS_OFF = (0.0, 0.0)    # 피벗->바닥 중심(메시 좌표). --mesh-sizes 로 채움
# 정류장 시설을 놓지 않는 승강장(세션50 사용자 결정). 승강장 자체(stops_v3.yaml)는 그대로
BUSSTOP_SKIP = {
    "도서관 남문(이쪽)": "±20m 안에 자리 없음(원래 자리 + 앞뒤 2m 간격 20곳 모두 걸림). 원래 자리는 경로 밖 차로 r1369 와 1.379m2 겹침(세션49 dry-run)",
}
# 가 메시가 경로 차로에 걸리는 동(세션51 잰 메시 크기 dry-run, s51_bench/dry_가·다_measured.log). --repr 가·다 에서 이 동은 나(상자)로 바꾼다
#   way765358663(r1316·r1644, 가만 해당. 다에서는 원래 나), 미술관 N16-2(r1320·r2041), 명덕관 S17-2(r1363·r1881)
#   이 목록 밖이라도 가 메시가 경로 차로에 걸리면 같이 나로 바꾼다(--mesh-sizes 없이 정사각형 가정일 때 등)
GA_TO_NA = {"765358663": "세션51 실측 가 경로 침범", "446440342": "세션51 실측 가·다 경로 침범(미술관)",
            "442760314": "세션51 실측 가·다 경로 침범(명덕관)"}
# ground: 캠퍼스 전체를 덮는 평면 메시 하나(기본 끔). 윗면을 도로면(z 0) 보다 GROUND_DROP 아래에 둔다(깜빡임 방지 0.1~0.2m)
#   메시 후보(세션50 파일 대조, 서버 소환은 세션51 layers_bench.py 에서 확인):
#   GroundCube = B1 소환 확인 경로. uexp 경계 50x50x50cm 반폭 = 1m 정육면체·중심 피벗 [파일 바이트, 서버 미확인]
#   Plane      = /Engine/BasicShapes/Plane, 경계 50x50x0cm 반폭 = 1m 판·중심 피벗, 충돌 BoxElems [파일, B1 밖]
#   SideWalkCube = 경계 1m 정육면체·중심 피벗, 충돌 ConvexElems [파일, B1 밖]
#   scale 은 세 축 같은 값이라 정육면체 후보는 아래로 같은 깊이만큼 내려간다(땅속이라 보이지 않음)
GROUND_DROP = 0.15
GROUND_PAD = 100.0      # 차선 외곽 사각형 바깥 여유(m)
GROUND_MESHES = {
    "GroundCube": dict(path="/Game/Carla/Static/Ground/GroundCube.GroundCube", side=1.0, thick=1.0),
    "Plane": dict(path="/Engine/BasicShapes/Plane.Plane", side=1.0, thick=0.0),
    "SideWalkCube": dict(path="/Game/Carla/Static/SideWalk/SideWalkCube.SideWalkCube", side=1.0, thick=1.0),
}
GROUND = "GroundCube"   # 기본은 B1 에서 소환 확인된 경로. 세션51 측정 뒤 --mesh-sizes 의 ground 로 바꿀 수 있음
# 다: 정류장(승강장)에서 보이는 동 = 상자까지 수평 거리 이 값 이하, 또는 이름 있는 큰 동(바닥 이 값 이상)
VISIBLE_M = 50.0
BIG_M2 = 2000.0


# ---------------- 표현 계산(서버 불필요, s49_b2_v3.py 도 씀) ----------------

def fit_mesh(L, H):
    """가: 상자 길이 L·높이 H 에 가장 맞는 메시와 배율(세 축 같은 값). 배율 = 길이비·높이비의 기하평균(두 상대오차 크기 같게)"""
    best = None
    for name, m in MESHES.items():
        s = math.sqrt((L / m["length"]) * (H / m["height"]))
        el, eh = s * m["length"] - L, s * m["height"] - H
        score = max(abs(el) / L, abs(eh) / H)
        if best is None or score < best["score"]:
            best = dict(mesh=name, scale=round(s, 3), len_m=round(s * m["length"], 2), h_m=round(s * m["height"], 2),
                        len_err_m=round(el, 2), h_err_m=round(eh, 2), score=round(score, 3))
    return best


def cube_plan(L, W, H):
    """나: 한 변 a = min(L, W, H) 정육면체를 겹쳐 놓아 L x W x H 상자의 바깥 치수를 맞춘다. 반환 (a, nL, nW, nH)"""
    a = min(L, W, H)
    return a, math.ceil(L / a - 1e-9), math.ceil(W / a - 1e-9), math.ceil(H / a - 1e-9)


def spread(D, a, n):
    """길이 D 를 한 변 a 정육면체 n 개로 덮을 때 중심 좌표(-D/2..D/2 안, 양끝 맞춤, 겹침 허용)"""
    if n == 1:
        return [0.0]
    return [-D / 2 + a / 2 + i * (D - a) / (n - 1) for i in range(n)]


def rect(cx, cy, yaw_deg, L, W):
    """CARLA 평면 사각형(길이 L 이 yaw 방향)"""
    from shapely.geometry import Polygon
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    pts = []
    for u, v in ((L / 2, W / 2), (-L / 2, W / 2), (-L / 2, -W / 2), (L / 2, -W / 2)):
        pts.append((cx + u * c - v * s, cy + u * s + v * c))
    return Polygon(pts)


def read_layout(path=LAYOUT):
    return list(csv.DictReader(open(path)))


def parse_platforms(path=STOPS):
    """stops_v3.yaml 의 승강장만 읽는다(yaml 모듈 없음 -> 이 파일 형식 전용 줄 파서)"""
    out, stop, cur, in_pf = [], None, None, False
    for line in open(path):
        if line.startswith("depot_candidates"):
            break
        m = re.match(r"  - id: (\S+)", line)
        if m:
            stop, in_pf = {"id": m.group(1)}, False
            continue
        m = re.match(r"    name: (.+)", line)
        if m and stop is not None and not in_pf:
            stop["name"] = m.group(1).strip()
        if line.startswith("    platforms:"):
            in_pf = True
            continue
        m = re.match(r"      - side: (\S+)", line)
        if m and in_pf:
            cur = dict(stop_id=stop["id"], stop_name=stop["name"], side=m.group(1))
            out.append(cur)
            continue
        m = re.match(r"        (road_id|s|lane_id|heading_carla_yaw_deg|lat|lon): ([-\d.]+)", line)
        if m and in_pf and cur is not None:
            cur[m.group(1)] = float(m.group(2))
        m = re.match(r"        carla_xyz: \[([-\d.]+), ([-\d.]+)", line)
        if m and in_pf and cur is not None:
            cur["x"], cur["y"] = float(m.group(1)), float(m.group(2))
    for p in out:
        p["road_id"], p["lane_id"] = int(p["road_id"]), int(p["lane_id"])
        p["label"] = f"{p['stop_name']}({p['side']})"
    return out


def choose_c(row, platforms):
    """다: 정류장에서 보이는 동(상자 거리 <= VISIBLE_M) 또는 이름 있는 큰 동이면 가, 나머지 나. 반환 (선택, 근거)"""
    from shapely.geometry import Point
    b = rect(float(row["center_carla_x"]), float(row["center_carla_y"]), float(row["yaw_deg"]),
             float(row["length_m"]), float(row["width_m"]))
    d, who = min((b.distance(Point(p["x"], p["y"])), p["label"]) for p in platforms)
    if d <= VISIBLE_M:
        return "가", f"{who} {d:.1f}m"
    if row["name"] and float(row["area_osm_m2"]) >= BIG_M2:
        return "가", f"이름 있는 큰 동 {float(row['area_osm_m2']):.0f}m2"
    return "나", f"가장 가까운 승강장 {who} {d:.1f}m"


# ---------------- 배치물 목록 ----------------

def building_actors(row, repr_, frame):
    """layout 한 행 -> 배치물 목록. 위치는 위경도에서 다시 변환(서버 맵이 달라도 그 머리말 기준)"""
    x, y = frame.carla(float(row["center_lat"]), float(row["center_lon"]))
    yaw, L, W, H = float(row["yaw_deg"]), float(row["length_m"]), float(row["width_m"]), float(row["height_m"])
    tag = row["name"] or f"way{row['way_id']}"
    acts = []
    if repr_ == "가":
        f = fit_mesh(L, H)
        m = MESHES[f["mesh"]]
        s = f["scale"]
        myaw = yaw - 90.0                       # bbox 가 긴 축 = 메시 y 축(가정). 메시 y 를 건물 긴 축에 맞춤
        c, si = math.cos(math.radians(myaw)), math.sin(math.radians(myaw))
        ox, oy = m["off"][0] * s, m["off"][1] * s
        ax, ay = x - (ox * c - oy * si), y - (ox * si + oy * c)     # 피벗 오프셋 보정 -> 메시 중심이 상자 중심
        side = s * m["length"]
        if m.get("width"):                      # --mesh-sizes 로 잰 폭이 있으면 실제 바닥 사각형
            foot, note = rect(x, y, yaw, side, s * m["width"]), "잰 메시 길이 x 폭(--mesh-sizes)"
        else:
            foot, note = rect(x, y, yaw, side, side), "폭 미측정 -> 길이 x 길이 정사각형(보수)"
        acts.append(dict(layer="buildings", owner=tag, kind="mesh", bp="static.prop.mesh", mesh_path=m["path"],
                         scale=s, x=ax, y=ay, z=-s * m.get("zmin", 0.0), yaw=myaw, foot=foot,
                         foot_note=note, foot_box=rect(x, y, yaw, side, min(W, side)), fit=f))
    else:
        a, nL, nW, nH = cube_plan(L, W, H)
        s = a / CUBE["side"]
        c, si = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        for u in spread(L, a, nL):
            for v in spread(W, a, nW):
                for zc in spread(H, a, nH):
                    cx, cy = x + u * c - v * si, y + u * si + v * c
                    acts.append(dict(layer="buildings", owner=tag, kind="cube", bp="static.prop.mesh", mesh_path=CUBE["path"],
                                     scale=round(s, 3), x=cx, y=cy, z=zc + H / 2 - CUBE.get("zc", 0.0) * s, yaw=yaw,
                                     foot=rect(cx, cy, yaw, a, a),
                                     foot_note="정육면체 한 변 a"))
    return acts


def busstop_actors(platforms, cmap, xodr_root, quads, rroads, blocks):
    """승강장마다 lane -1 중심에서 오른쪽(인도 쪽)으로 차선 반폭 + BUS_GAP + 바닥 깊이/2 만큼 띄워 차로와 나란히.
    그 자리가 어떤 차로(여유 MARGIN 안)나 건물 상자(blocks)에 걸리면 승강장 앞뒤로 2m 씩 최대 20m 옮겨 본다"""
    import carla
    lanes2 = {}
    for rd in xodr_root.iter("road"):
        for ln in rd.iter("lane"):
            if ln.get("id") == "-2":
                lanes2[int(rd.get("id"))] = ln.get("type")
    acts, notes = [], []
    for p in platforms:
        if p["label"] in BUSSTOP_SKIP:
            notes.append(dict(platform=p["label"], skipped=True, reason=BUSSTOP_SKIP[p["label"]]))
            continue
        base = cmap.get_waypoint_xodr(p["road_id"], p["lane_id"], p["s"])
        if base is None:
            notes.append(dict(platform=p["label"], error="get_waypoint_xodr None"))
            continue
        chosen = None
        for ds in [0] + [k * sg for k in range(2, 21, 2) for sg in (1, -1)]:
            if ds == 0:
                wp = base
            else:
                nx = base.next(ds) if ds > 0 else base.previous(-ds)
                if not nx:
                    continue
                wp = nx[0]
            tf = wp.transform
            r = tf.get_right_vector()
            off = wp.lane_width / 2 + BUS_GAP + BUS["depth"] / 2
            x, y = tf.location.x + r.x * off, tf.location.y + r.y * off
            yaw = tf.rotation.yaw
            myaw = yaw + BUS_YAW_OFF
            c, si = math.cos(math.radians(myaw)), math.sin(math.radians(myaw))
            ax, ay = x - (BUS_OFF[0] * c - BUS_OFF[1] * si), y - (BUS_OFF[0] * si + BUS_OFF[1] * c)   # 메시 중심이 바닥 사각형 중심
            a = dict(layer="busstop", owner=p["label"], kind="busstop", bp=BUS["bp"], mesh_path=None, scale=None,
                     x=ax, y=ay, z=0.0, yaw=myaw, foot=rect(x, y, yaw, BUS["along"], BUS["depth"]),
                     foot_note="bbox+중심오프셋 사각형")
            v = check([a], quads, rroads)
            b = [n for n, g in blocks if g.intersects(a["foot"]) and g.intersection(a["foot"]).area > 1e-3]
            if chosen is None:
                chosen = (ds, wp, a, off, v, b)            # 다 안 되면 원래 자리(0) 를 남기고 위반으로 보고
            if not v and not b:
                chosen = (ds, wp, a, off, v, b)
                break
        ds, wp, a, off, v, b = chosen
        acts.append(a)
        sw = cmap.get_waypoint_xodr(wp.road_id, -2, wp.s)
        sw_w = sw.lane_width if (sw is not None and sw.lane_type == carla.LaneType.Sidewalk) else 0.0
        notes.append(dict(platform=p["label"], road=wp.road_id, s=round(wp.s, 2), moved_along_m=ds,
                          side="오른쪽(진행 방향, 인도 쪽)", lane_center_to_bus_center_m=round(off, 3),
                          lane_edge_to_bus_near_m=BUS_GAP, lane_edge_to_bus_far_m=round(BUS_GAP + BUS["depth"], 3),
                          sidewalk_lane_w_m=round(sw_w, 2), xodr_lane_m2_type=lanes2.get(wp.road_id, "없음"),
                          beyond_sidewalk_m=round(max(0.0, BUS_GAP + BUS["depth"] - sw_w), 2),
                          yaw_lane=round(wp.transform.rotation.yaw, 1), x=round(a["x"], 2), y=round(a["y"], 2),
                          ok=not v and not b, left_violations=len(v), left_building_clash=b))
    return acts, notes


def ground_actors(quads, name=None):
    """ground: 모든 차선 조각 외곽 사각형 + GROUND_PAD 를 덮는 정사각형 판 하나(scale 세 축 같음 -> 긴 변 기준).
    윗면 = z -GROUND_DROP. 침범 검사 대상 아님(차로 아래)"""
    from shapely.geometry import box
    name = name or GROUND
    g = GROUND_MESHES[name]
    xs0, ys0, xs1, ys1 = zip(*[q.bounds for _, q in quads])
    x0, y0, x1, y1 = min(xs0) - GROUND_PAD, min(ys0) - GROUND_PAD, max(xs1) + GROUND_PAD, max(ys1) + GROUND_PAD
    span = max(x1 - x0, y1 - y0)
    s = math.ceil(span / g["side"])
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    z = -GROUND_DROP - s * (g["thick"] / 2 + g.get("zc", 0.0))      # 중심 피벗: 윗면이 -GROUND_DROP 이 되게
    return [dict(layer="ground", owner=f"ground_{name}", kind="ground", bp="static.prop.mesh", mesh_path=g["path"],
                 scale=float(s), x=cx, y=cy, z=z, yaw=0.0, foot=box(cx - s * g["side"] / 2, cy - s * g["side"] / 2,
                                                                    cx + s * g["side"] / 2, cy + s * g["side"] / 2),
                 foot_note=f"{name} 한 변 {s * g['side']:.0f}m, 윗면 z {-GROUND_DROP}m")]


def load_mesh_sizes(path):
    """layers_bench.py 의 mesh_sizes JSON -> MESHES·CUBE·BUS·GROUND_MESHES 값 덮어쓰기.
    형식: {"meshes": {이름: {path, size_x, size_y, size_z, center_x, center_y, center_z, min_z, ok}}, "cube": 이름, "ground_choice": 이름}
    크기는 scale 1.0, yaw 0 에서 피벗 기준 메시 좌표(m). 이름은 MESHES·GROUND_MESHES 키, "busstop", 그리고 정육면체 후보"""
    global GROUND, BUS_OFF
    d = json.loads(Path(path).read_text())
    ms = {k: v for k, v in d.get("meshes", {}).items() if v.get("ok")}
    bad = [k for k, v in d.get("meshes", {}).items() if not v.get("ok") and k in MESHES]
    if bad and len(bad) < len(MESHES):
        for k in bad:
            del MESHES[k]                   # 소환 안 된 가 메시는 고르지 않는다
        print(f"--mesh-sizes: 가 메시에서 뺌 {bad}")
    used = []
    for name, m in ms.items():
        if name in MESHES:
            # 바닥 z 는 LiDAR 가 밑변을 덜 맞혀 0.003~0.088m 로 나왔다(세션51). 윗면이 API 높이와 같으므로 0.1m 미만은 0 으로 본다
            MESHES[name].update(length=m["size_y"], width=m["size_x"], height=m["min_z"] + m["size_z"],   # = 윗면 z(바닥 0 기준)
                                off=(m["center_x"], m["center_y"]), zmin=0.0 if abs(m["min_z"]) < 0.1 else m["min_z"])
            used.append(name)
        if name in GROUND_MESHES:
            GROUND_MESHES[name].update(side=max(m["size_x"], m["size_y"]), thick=m["size_z"], zc=m["center_z"])
            used.append(f"ground:{name}")
        if name == "busstop":
            BUS.update(along=m["size_x"], depth=m["size_y"])         # 긴 축 x 를 차로와 나란히(BUS_YAW_OFF 0 또는 180)
            BUS_OFF = (m["center_x"], m["center_y"])
            used.append("busstop")
    cube = d.get("cube")
    if cube in ms:
        m = ms[cube]
        side = max(m["size_x"], m["size_y"], m["size_z"])         # z 는 밑변 덜 맞혀 0.989(세션51), x·y 1.000 -> 가장 긴 축
        CUBE.update(path=m["path"], side=side, zc=round(m["min_z"] + side / 2, 3) if abs(m["min_z"] + side / 2) > 0.02 else 0.0)
        used.append(f"cube:{cube}")
    if d.get("ground_choice") in GROUND_MESHES:
        GROUND = d["ground_choice"]
    print(f"--mesh-sizes {Path(path).name}: {used}, ground {GROUND}, cube {CUBE}")


# ---------------- 침범 검사 ----------------

def lane_quads(cmap, xodr_root):
    """주행 차선 lane -1 을 LANE_STEP 간격 사각형 조각으로(차선 폭 그대로). [(road_id, Polygon)]"""
    from shapely.geometry import Polygon
    out = []
    for rd in xodr_root.iter("road"):
        rid, L = int(rd.get("id")), float(rd.get("length"))
        n = max(1, math.ceil(L / LANE_STEP))
        prev = None
        for i in range(n + 1):
            s = min(i * LANE_STEP, L - 1e-3)
            wp = cmap.get_waypoint_xodr(rid, -1, s)
            if wp is None:
                prev = None
                continue
            loc, r, h = wp.transform.location, wp.transform.get_right_vector(), wp.lane_width / 2
            edge = ((loc.x - r.x * h, loc.y - r.y * h), (loc.x + r.x * h, loc.y + r.y * h))
            if prev is not None:
                q = Polygon([prev[0], prev[1], edge[1], edge[0]])
                if not q.is_valid:
                    q = q.buffer(0)
                if not q.is_empty:
                    out.append((rid, q))
            prev = edge
    return out


def route_roads():
    """test_drive.py ROUTES_V3S46(5노선 + 30쌍) road 집합. 파일은 고치지 않고 읽기만"""
    sys.path.insert(0, str(BASE / "tests"))
    argv, sys.argv = sys.argv, sys.argv[:1]
    try:
        import test_drive as td
    finally:
        sys.argv = argv
    routes = td.ROUTES_V3S46
    return {rid for v in routes.values() for rid in v["roads"]}, len(routes)


def check(acts, quads, rroads):
    """바닥 사각형과 차선 사이 거리 < MARGIN 이면 위반. 경로 차로 / 그 밖 차로 나눠서"""
    from shapely.strtree import STRtree
    tree = STRtree([q for _, q in quads])
    viol = []
    for i, a in enumerate(acts):
        if a["layer"] == "ground":
            continue                         # 지면 판은 차로 아래(윗면 z -GROUND_DROP)
        f = a["foot"]
        hits = {}
        for j in tree.query(f.buffer(MARGIN)):
            rid, q = quads[j]
            d = f.distance(q)
            if d < MARGIN:
                ov = f.intersection(q).area
                k = hits.get(rid)
                if k is None or d < k[0] or (d == 0 and ov > k[1]):
                    hits[rid] = (d, ov)
        for rid, (d, ov) in sorted(hits.items()):
            viol.append(dict(idx=i, layer=a["layer"], owner=a["owner"], kind=a["kind"], road=rid, route=rid in rroads,
                             clearance_m=round(d, 3), overlap_m2=round(ov, 3)))
    return viol


# ---------------- 그림 ----------------

def draw(acts, viol, quads, rroads, platforms, path, title, layout_rows=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.font_manager
    import matplotlib.pyplot as plt
    f = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
    if Path(f).exists():
        matplotlib.font_manager.fontManager.addfont(f)
    plt.rcParams["font.family"] = ["Noto Sans CJK JP", "DejaVu Sans"]
    bad = {v["idx"] for v in viol}
    bad_route = {v["idx"] for v in viol if v["route"]}
    fig, ax = plt.subplots(figsize=(14, 14))
    for rid, q in quads:
        ax.fill(*q.exterior.xy, color="#2a78d6" if rid in rroads else "0.75", lw=0, alpha=0.55 if rid in rroads else 1)
    for i, a in enumerate(acts):
        if a["layer"] == "ground":
            continue
        col = "#d55181" if i in bad_route else ("#eda100" if i in bad else ("#1baf7a" if a["layer"] == "busstop" else "0.25"))
        ax.plot(*a["foot"].exterior.xy, color=col, lw=0.8 if a["kind"] == "cube" else 1.2)
    for p in platforms:
        ax.plot(p["x"], p["y"], marker="^", color="#eb6834", ms=8, mec="white", mew=1.5)
        ax.annotate(p["label"], (p["x"], p["y"]), xytext=(4, 4), textcoords="offset points", fontsize=7, color="0.15")
    if layout_rows:
        for r in layout_rows:
            ax.annotate(r["name"] or f"way{r['way_id']}", (float(r["center_carla_x"]), float(r["center_carla_y"])),
                        fontsize=6, color="0.35", ha="center")
    ax.set_aspect("equal")
    ax.invert_yaxis()                       # CARLA y 는 화면 아래가 + -> 뒤집어 북쪽이 위
    ax.set_title(title, fontsize=10)
    ax.set_xlabel("CARLA x (m)")
    ax.set_ylabel("CARLA y (m), 위 = 북")
    from matplotlib.lines import Line2D
    ax.legend(handles=[Line2D([], [], color="#2a78d6", lw=6, alpha=0.55, label="경로 차로(5노선+30쌍)"),
                       Line2D([], [], color="0.75", lw=6, label="그 밖 주행 차로"),
                       Line2D([], [], color="0.25", lw=1.2, label="건물 바닥"),
                       Line2D([], [], color="#1baf7a", lw=1.2, label="정류장 시설 바닥"),
                       Line2D([], [], color="#d55181", lw=1.2, label="경로 차로 침범(여유 0.5m 안)"),
                       Line2D([], [], color="#eda100", lw=1.2, label="그 밖 차로 침범"),
                       Line2D([], [], color="#eb6834", marker="^", lw=0, label="승강장")], loc="lower left", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def draw_busstops(acts, viol, quads, rroads, platforms, path):
    import matplotlib.pyplot as plt
    from shapely.geometry import box
    bs = [(i, a) for i, a in enumerate(acts) if a["layer"] == "busstop"]
    bad = {v["idx"] for v in viol}
    cols = 4
    rows = math.ceil(len(bs) / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(cols * 3.4, rows * 3.4))
    for ax, (i, a) in zip(axes.flat, bs):
        win = box(a["x"] - 12, a["y"] - 12, a["x"] + 12, a["y"] + 12)
        for rid, q in quads:
            if q.intersects(win):
                ax.fill(*q.exterior.xy, color="#2a78d6" if rid in rroads else "0.75", lw=0, alpha=0.55 if rid in rroads else 1)
        ax.plot(*a["foot"].exterior.xy, color="#d55181" if i in bad else "#1baf7a", lw=1.5)
        for p in platforms:
            if p["label"] == a["owner"]:
                ax.plot(p["x"], p["y"], marker="^", color="#eb6834", ms=8, mec="white", mew=1.5)
        ax.set_xlim(a["x"] - 12, a["x"] + 12)
        ax.set_ylim(a["y"] + 12, a["y"] - 12)
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_title(a["owner"], fontsize=8)
    for ax in list(axes.flat)[len(bs):]:
        ax.axis("off")
    fig.suptitle("정류장 시설 11곳(24m 창): 청록 = 바닥, 파랑 = 경로 차로, 회색 = 그 밖 차로, 주황 삼각형 = 승강장", fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


# ---------------- 서버 ----------------

def mem_avail_mib():
    for line in open("/proc/meminfo"):
        if line.startswith("MemAvailable"):
            return int(line.split()[1]) // 1024
    return -1


def tick_ms(world, n=100):
    """B1(s38) 과 같은 측정: 동기 0.05s 로 n 틱, 렌더 켬/끔 각각 ms/틱. 원래 설정은 되돌린다"""
    orig = world.get_settings()
    out = {}
    try:
        for render in (True, False):
            st = world.get_settings()
            st.synchronous_mode, st.fixed_delta_seconds, st.no_rendering_mode = True, 0.05, not render
            world.apply_settings(st)
            for _ in range(5):
                world.tick()
            t0 = time.perf_counter()
            for _ in range(n):
                world.tick()
            out["render_on" if render else "render_off"] = round((time.perf_counter() - t0) * 1000 / n, 2)
    finally:
        world.apply_settings(orig)
    out["avail_mib"] = mem_avail_mib()
    return out


def spawn(args, acts, viol, sha):
    import carla
    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    world = client.get_world()
    srv = hashlib.sha256(world.get_map().to_opendrive().encode()).hexdigest()[:16]
    if srv != sha:
        raise SystemExit(f"서버 맵 sha {srv} != --xodr {sha}. 소환하지 않고 종료(로더로 맞는 맵을 먼저 띄울 것)")
    if args.state.exists():
        raise SystemExit(f"상태 파일 {args.state} 가 이미 있다. 먼저 --remove 로 지우고 다시 실행")
    skip = {v["idx"] for v in viol if v["route"]}
    lib = world.get_blueprint_library()
    state = dict(map_sha16=sha, repr=args.repr, layers=args.layers, seed=args.seed, time=time.strftime("%F %T"), actors=[], measure=[])
    if args.measure:
        state["measure"].append(dict(after="소환 전", **tick_ms(world)))
        print(f"측정 {state['measure'][-1]}")
    for layer in args.layers:
        todo = [(i, a) for i, a in enumerate(acts) if a["layer"] == layer and i not in skip]
        m0 = mem_avail_mib()
        batch = []
        for i, a in todo:
            bp = lib.find(a["bp"])
            if a["mesh_path"]:
                bp.set_attribute("mesh_path", a["mesh_path"])
            if a["scale"] is not None and bp.has_attribute("scale"):
                bp.set_attribute("scale", str(a["scale"]))
            tf = carla.Transform(carla.Location(x=a["x"], y=a["y"], z=a["z"]), carla.Rotation(yaw=a["yaw"]))
            batch.append(carla.command.SpawnActor(bp, tf))
        res = client.apply_batch_sync(batch, False) if batch else []
        ok = 0
        for (i, a), r in zip(todo, res):
            if r.error:
                print(f"[{layer}] 소환 실패 {a['owner']}: {r.error}")
                continue
            ok += 1
            state["actors"].append(dict(id=r.actor_id, type_id=a["bp"], layer=layer, owner=a["owner"]))
        time.sleep(2.0)
        print(f"[{layer}] 소환 {ok}/{len(todo)}(경로 침범으로 뺀 것 {sum(1 for i in skip if acts[i]['layer'] == layer)}), "
              f"avail {m0} -> {mem_avail_mib()} MiB")
        args.state.write_text(json.dumps(state, ensure_ascii=False, indent=1))      # 중간에 멈춰도 --remove 가 되게 매 레이어 저장
        if args.measure:
            state["measure"].append(dict(after=f"{layer} {ok}개", **tick_ms(world)))
            print(f"측정 {state['measure'][-1]}")
    args.state.write_text(json.dumps(state, ensure_ascii=False, indent=1))
    print(f"상태 파일 {args.state} (actor {len(state['actors'])})")


def remove(args):
    import carla
    if not args.state.exists():
        raise SystemExit(f"상태 파일 없음: {args.state}. 지울 것 없음")
    st = json.loads(args.state.read_text())
    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    world = client.get_world()
    gone = miss = other = 0
    ids = []
    for rec in st["actors"]:
        a = world.get_actor(rec["id"])
        if a is None:
            miss += 1
        elif a.type_id != rec["type_id"]:
            other += 1                       # 같은 id 를 다른 actor 가 쓰면 건드리지 않는다
        else:
            ids.append(rec["id"])
    res = client.apply_batch_sync([carla.command.DestroyActor(i) for i in ids], False) if ids else []
    gone = sum(1 for r in res if not r.error)
    done = args.state.with_suffix(f".removed_{time.strftime('%Y%m%d_%H%M%S')}.json")
    args.state.rename(done)
    print(f"지움 {gone}/{len(st['actors'])}, 이미 없음 {miss}, type 다름(안 지움) {other}. 상태 파일 -> {done.name}")


# ---------------- main ----------------

def main():
    ap = argparse.ArgumentParser(description="캠퍼스 장식 레이어 배치(B3 초안)")
    ap.add_argument("--layers", default="buildings,busstop", help=f"쉼표 목록 {LAYERS}")
    ap.add_argument("--repr", choices=["가", "나", "다"], default="나",
                    help="건물 표현(기본 나, 세션50 결정). 다 는 메시 실제 크기를 잰 뒤 다시 비교")
    ap.add_argument("--mesh-sizes", type=Path, default=None, help="layers_bench.py 가 잰 메시 크기 JSON")
    ap.add_argument("--tag", default=None, help="--dry-run 출력 이름(기본: 레이어·표현에서 만듦. 세션49 파일을 덮지 않게 줄 것)")
    ap.add_argument("--ground-mesh", choices=sorted(GROUND_MESHES), default=None, help=f"ground 레이어 메시(기본 {GROUND})")
    ap.add_argument("--seed", type=int, default=SEED, help="무작위 배치 레이어용(이번 구현 레이어는 쓰지 않음)")
    ap.add_argument("--xodr", type=Path, default=XODR_V3)
    ap.add_argument("--layout", type=Path, default=LAYOUT)
    ap.add_argument("--stops", type=Path, default=STOPS)
    ap.add_argument("--state", type=Path, default=STATE)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--remove", action="store_true")
    ap.add_argument("--measure", action="store_true", help="서버: 소환 전과 레이어마다 100틱 ms/틱(렌더 켬·끔)·avail 기록")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    args = ap.parse_args()
    if args.remove:
        return remove(args)
    args.layers = [s.strip() for s in args.layers.split(",") if s.strip()]
    bad = [s for s in args.layers if s not in LAYERS]
    if bad:
        raise SystemExit(f"모르는 레이어 {bad}. 가능: {LAYERS}")
    if "buildings" in args.layers and not args.repr:
        raise SystemExit("buildings 레이어는 --repr 가|나|다 가 필요")
    random.seed(args.seed)
    if args.mesh_sizes:
        load_mesh_sizes(args.mesh_sizes)

    import carla
    text = args.xodr.read_text()
    sha = hashlib.sha256(text.encode()).hexdigest()[:16]
    print(f"xodr {args.xodr.name} sha256 {sha} / layers {args.layers} / repr {args.repr} / seed {args.seed} / 여유 {MARGIN}m")
    frame = Frame(text)
    cmap = carla.Map(args.xodr.stem, text)
    root = ET.fromstring(text)
    platforms = parse_platforms(args.stops)
    rows = read_layout(args.layout)
    quads = lane_quads(cmap, root)
    rroads, nroutes = route_roads()
    # 정류장 자리 고를 때 피할 건물 상자: layout 상자(표현과 무관) + 가 일 때 정사각형 바닥
    acts, notes = [], []
    for layer in args.layers:
        if layer == "buildings":
            to_na = []                                 # (동, 이유) 가 -> 나 로 바꾼 것
            for r in rows:
                rp = r["repr_c"] if args.repr == "다" else args.repr
                btag = r["name"] or f"way{r['way_id']}"
                if rp == "가" and r["way_id"] in GA_TO_NA:
                    rp = "나"
                    to_na.append((f"{btag} part{r['part']}", GA_TO_NA[r["way_id"]]))
                elif rp == "가":
                    hit = [v for v in check(building_actors(r, "가", frame), quads, rroads) if v["route"]]
                    if hit:
                        rp = "나"
                        to_na.append((f"{btag} part{r['part']}", "자동: 가 메시 경로 침범 r" + "·r".join(str(v["road"]) for v in hit)))
                acts += building_actors(r, rp, frame)
            if args.repr in ("가", "다"):
                print(f"[buildings] --repr {args.repr}: 가 -> 나 로 바꾼 동 {len(to_na)}")
                for n, why in to_na:
                    print(f"  나로 {n}: {why}")
    blocks = [(a["owner"], a["foot"]) for a in acts] + \
             [(r["name"] or r["way_id"], rect(*frame.carla(float(r["center_lat"]), float(r["center_lon"])), float(r["yaw_deg"]),
                                             float(r["length_m"]), float(r["width_m"]))) for r in rows]
    for layer in args.layers:
        if layer == "busstop":
            a, notes = busstop_actors(platforms, cmap, root, quads, rroads, blocks)
            acts += a
        elif layer == "ground":
            g = ground_actors(quads, args.ground_mesh)
            print(f"[ground] {g[0]['owner']} scale {g[0]['scale']:g} 중심 ({g[0]['x']:.1f}, {g[0]['y']:.1f}, {g[0]['z']:.2f}) "
                  f"{g[0]['foot_note']}")
            acts += g
        elif layer != "buildings":
            print(f"[{layer}] 빈 레이어(이번 초안 미구현) 0개")
    viol = check(acts, quads, rroads)
    n_route = sum(1 for v in viol if v["route"])
    print(f"차선 조각 {len(quads)}, 경로 {nroutes}개 road {len(rroads)}")
    for layer in args.layers:
        n = sum(1 for a in acts if a["layer"] == layer)
        vr = len({v["idx"] for v in viol if v["layer"] == layer and v["route"]})
        vo = len({v["idx"] for v in viol if v["layer"] == layer and not v["route"]})
        print(f"[{layer}] 배치물 {n}, 경로 차로 침범 배치물 {vr}, 그 밖 차로만 침범 {len({v['idx'] for v in viol if v['layer'] == layer}) - vr} (그 밖 포함 {vo})")
    print(f"경로 차로 침범 건수 {n_route} -> {'기준 통과(0)' if n_route == 0 else '기준 미달'}")
    alt = [dict(a, foot=a["foot_box"]) for a in acts if "foot_box" in a]
    if alt:
        va = check(alt, quads, rroads)
        print(f"[가 참고] 메시 폭 = 상자 폭으로 가정하면: 경로 차로 침범 배치물 {len({v['idx'] for v in va if v['route']})}, "
              f"건수 {sum(1 for v in va if v['route'])}, 그 밖 포함 배치물 {len({v['idx'] for v in va})}")
    for v in viol:
        print(f"  위반 {'경로' if v['route'] else '그밖'} {v['layer']} {v['owner']} {v['kind']} r{v['road']} "
              f"거리 {v['clearance_m']}m 겹침 {v['overlap_m2']}m2")
    # 다른 배치물끼리 겹침(건물 상자 vs 정류장)
    bus = [a for a in acts if a["layer"] == "busstop"]
    blds = [a for a in acts if a["layer"] == "buildings"]
    clash = [(b["owner"], s["owner"], round(b["foot"].intersection(s["foot"]).area, 2)) for s in bus for b in blds
             if b["foot"].intersects(s["foot"]) and b["foot"].intersection(s["foot"]).area > 1e-3]
    print(f"정류장 시설 vs 건물 바닥 겹침 {len(clash)} {clash[:10]}")
    for n in notes:
        print(f"  busstop {n}")

    if args.dry_run:
        # 세션49 이름(campus_layers_dry_<repr>) 은 buildings,busstop 일 때만. 다른 조합은 레이어 이름을 붙여 덮어쓰지 않게
        tag = args.tag or (args.repr if args.layers == ["buildings", "busstop"] else
                           "-".join(([args.repr] if "buildings" in args.layers else []) + [l for l in args.layers if l != "buildings"]))
        out = LOGS / f"campus_layers_dry_{tag}.json"
        out.write_text(json.dumps(dict(
            map_sha16=sha, layers=args.layers, repr=args.repr, seed=args.seed, margin_m=MARGIN, routes=nroutes,
            route_roads=len(rroads), lane_pieces=len(quads),
            actors=[{k: (round(v, 3) if isinstance(v, float) else v) for k, v in a.items() if k not in ("foot", "foot_box")}
                    for a in acts],
            violations=viol, busstop_notes=notes, busstop_building_clash=clash), ensure_ascii=False, indent=1))
        with open(LOGS / f"campus_layers_dry_{tag}_actors.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["layer", "owner", "kind", "blueprint", "mesh_path", "scale", "x", "y", "z", "yaw"])
            for a in acts:
                w.writerow([a["layer"], a["owner"], a["kind"], a["bp"], a["mesh_path"] or "", a["scale"] if a["scale"] is not None else "",
                            f"{a['x']:.3f}", f"{a['y']:.3f}", f"{a['z']:.3f}", f"{a['yaw']:.2f}"])
        draw(acts, viol, quads, rroads, platforms, FIGS / f"campus_layers_dry_{tag}.png",
             f"place_campus_layers --dry-run --repr {tag}: 배치물 {len(acts)}, 경로 차로 침범 {n_route} (여유 {MARGIN}m)",
             rows if "buildings" in args.layers else None)
        if "busstop" in args.layers:
            draw_busstops(acts, viol, quads, rroads, platforms, FIGS / (f"campus_layers_dry_busstop_{args.tag}.png" if args.tag else "campus_layers_dry_busstop.png"))
        print(f"-> {out.name}, campus_layers_dry_{tag}_actors.csv, figures/campus_layers_dry_{tag}.png")
        return
    spawn(args, acts, viol, sha)


if __name__ == "__main__":
    main()
