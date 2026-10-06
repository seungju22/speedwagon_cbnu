#!/usr/bin/env python3
# 맵세션22 Phase 3-3/4: 건물 자리에 소품 배치 / 디버그 표시 / 소품 목록 추출. 설계 map/docs/props_plan.md
# xodr 은 건드리지 않는다. 서버에 접속해 실행 중에만 actor 를 만든다(서버 재시작·지도 재생성 때 사라짐).
# 좌표: s22_buildings.csv 의 위경도 -> 서버가 지금 가진 지도(to_opendrive) 머리말의 투영으로 매번 변환(재변환 후에도 그대로).
# 기본은 아무것도 하지 않는다. 동작은 인자로만:
#   --list OUT        static.prop.* / vehicle.* 블루프린트 목록을 파일로 (Phase 4-2)
#   --dry-run         침범 검사만 (소환 안 함)
#   --apply           검사 통과분만 소환, actor id 를 상태 파일에 기록
#   --clear           상태 파일 actor + 남은 static.prop.mesh 제거
#   --draw [--life S] 건물 자리에 디버그 상자·이름 (센서 영상에는 안 나옴)
#   --only 이름일부   그 건물만
# 근거: https://carla.readthedocs.io/en/0.9.15/python_api/ (carla.Map.get_waypoint, DebugHelper, static.prop.mesh)
import argparse
import csv
import hashlib
import json
import math
import re
import subprocess
import sys
import time
from pathlib import Path

import carla

BASE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(BASE / "scripts"))
from s22_buildings import Frame  # noqa: E402

CSV_IN = BASE / "data/processed/s22_buildings.csv"
STATE = BASE / "docs/logs/props_state.json"
HOST, PORT, TIMEOUT = "localhost", 2000, 20.0
MARGIN_M = 3.0          # 주행 차선 끝에서 최소 여유 (= 인도 2.80m 바깥)
ROUTE_CLEAR_M = 0.9 + 3.0  # 루트 계획선에서 차체 반폭 + 여유
STOP_CLEAR_M = 10.0
SHIFT_STEP_M, SHIFT_MAX_M = 1.0, 20.0
ON_ROAD_TYPES = (carla.LaneType.Driving | carla.LaneType.Sidewalk | carla.LaneType.Shoulder
                 | carla.LaneType.Parking | carla.LaneType.Biking)
MESH = "/Game/Carla/Static/{0}/{1}.{1}"

# 시험 배치 표 (Phase 4-3, 가능성 확인용. 전면 배치 아님)
# kind: mesh = static.prop.mesh + mesh_path / bp = 등록 블루프린트 id
# where: center = 폴리곤 중심, ends = 긴 축 양 끝 바깥 gap m, size = 발자국(길이, 폭) m. None 이면 폴리곤 외접 사각형
PLAN = [
    dict(building="중앙도서관 구관", kind="mesh", ref=MESH.format("Building", "SM_Apartment01_1"), scale=1.0,
         where="center", size=None),
    dict(building="중앙도서관 구관", kind="mesh", ref=MESH.format("Vegetation/Trees", "SM_Acer_02"), scale=1.0,
         where="ends", gap=8.0, size=(9.0, 9.0)),
    dict(building="자연과학대학 본관", kind="mesh", ref=MESH.format("Building", "SM_Block05"), scale=1.0,
         where="center", size=None),
    dict(building="자연과학대학 본관", kind="bp", ref="static.prop.bench01", scale=None,
         where="ends", gap=4.0, size=(2.0, 1.0)),
]


def mem(tag):
    out = subprocess.run(["free", "-h"], capture_output=True, text=True).stdout.splitlines()
    gtt = None
    for p in ("/sys/class/drm/card0/device/mem_info_gtt_used", "/sys/class/drm/card1/device/mem_info_gtt_used"):
        try:
            gtt = int(open(p).read()) / 2 ** 30
            break
        except OSError:
            pass
    print(f"[mem {tag}] {time.strftime('%H:%M:%S')} {out[1].split()[1:]} swap {out[2].split()[1:3]} "
          f"gtt {gtt:.2f}GiB" if gtt is not None else f"[mem {tag}] {out[1]}")


def load_buildings(only):
    rows = list(csv.DictReader(open(CSV_IN)))
    return [r for r in rows if not only or only in r["name"]]


def route_points(cmap, xodr_text):
    """정류장 루트 계획선 근사: 각 루트 road 의 lane -1 을 2m 간격으로 샘플. 정차점도 반환."""
    sys.path.insert(0, str(BASE / "tests"))
    try:
        import test_drive
        routes = test_drive.ROUTES
    except Exception as e:
        print(f"루트 불러오기 실패({e}) -> 루트 검사 생략")
        return [], []
    lens = {int(i): float(L) for L, i in re.findall(r'<road[^>]*length="([\d.]+)" id="(\d+)"', xodr_text)}
    pts, stops = [], []
    for name, r in routes.items():
        for rid in r["roads"]:
            s = 0.0
            while s <= lens.get(rid, 0.0):
                wp = cmap.get_waypoint_xodr(rid, -1, s)
                if wp:
                    pts.append(wp.transform.location)
                s += 2.0
        wp = cmap.get_waypoint_xodr(r["roads"][-1], -1, r["end_s"])
        if wp:
            stops.append((name, wp.transform.location))
    return pts, stops


def footprint(cx, cy, yaw_deg, L, W, step=1.0):
    """CARLA 평면 사각형 검사점: 중심, 네 모서리, 둘레 step 간격."""
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    hl, hw = L / 2, W / 2
    out = [(cx, cy)]
    corners = [(hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)]
    for (u0, v0), (u1, v1) in zip(corners, corners[1:] + corners[:1]):
        n = max(1, int(math.hypot(u1 - u0, v1 - v0) / step))
        for k in range(n):
            u, v = u0 + (u1 - u0) * k / n, v0 + (v1 - v0) * k / n
            out.append((cx + u * c - v * s, cy + u * s + v * c))
    return out


def check(cmap, pts_xy, route_pts, stops):
    """침범 검사. 반환 (통과 여부, 이유, 최소 차선 여유 m)."""
    min_clear = 1e9
    for x, y in pts_xy:
        loc = carla.Location(x=x, y=y, z=0.0)
        if cmap.get_waypoint(loc, project_to_road=False, lane_type=ON_ROAD_TYPES) is not None:
            return False, f"도로·인도 위 점 ({x:.1f},{y:.1f})", 0.0
        wp = cmap.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
        if wp:
            d = wp.transform.location.distance(loc) - wp.lane_width / 2
            min_clear = min(min_clear, d)
            if d < MARGIN_M:
                return False, f"차선 끝까지 {d:.1f}m < {MARGIN_M}", d
        for p in route_pts:
            if abs(p.x - x) < ROUTE_CLEAR_M and abs(p.y - y) < ROUTE_CLEAR_M and p.distance(loc) < ROUTE_CLEAR_M:
                return False, "루트 계획선 근접", min_clear
        for name, p in stops:
            if p.distance(loc) < STOP_CLEAR_M:
                return False, f"정차점({name}) {STOP_CLEAR_M}m 안", min_clear
    return True, "통과", min_clear


def candidates(b, item, frame):
    """배치 위치 후보 (cx, cy, yaw, L, W) 목록. 위경도에서 서버 맵 기준으로 다시 변환."""
    cx, cy = frame.carla(float(b["lat"]), float(b["lon"]))
    yaw = float(b["yaw_deg"])
    L, W = item["size"] or (float(b["len_m"]), float(b["wid_m"]))
    if item["where"] == "center":
        return [(cx, cy, yaw, L, W)]
    half = float(b["len_m"]) / 2 + item.get("gap", 5.0) + L / 2
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    return [(cx + k * half * c, cy + k * half * s, yaw, L, W) for k in (1, -1)]


def place(cmap, frame, rows, route_pts, stops):
    """각 PLAN 항목의 최종 위치(도로 반대쪽 이동 포함). 반환 [(item, building, (x,y,yaw,L,W), clear)]."""
    out = []
    for item in PLAN:
        for b in rows:
            if item["building"] not in b["name"]:
                continue
            for (x, y, yaw, L, W) in candidates(b, item, frame):
                ok, why, clr = check(cmap, footprint(x, y, yaw, L, W), route_pts, stops)
                shift = 0.0
                if not ok:
                    # 가장 가까운 주행 차선에서 멀어지는 방향으로 1m 씩
                    wp = cmap.get_waypoint(carla.Location(x=x, y=y), project_to_road=True,
                                           lane_type=carla.LaneType.Driving)
                    dx, dy = x - wp.transform.location.x, y - wp.transform.location.y
                    n = math.hypot(dx, dy) or 1.0
                    while not ok and shift < SHIFT_MAX_M:
                        shift += SHIFT_STEP_M
                        nx, ny = x + dx / n * shift, y + dy / n * shift
                        ok, why2, clr = check(cmap, footprint(nx, ny, yaw, L, W), route_pts, stops)
                    if ok:
                        x, y = nx, ny
                    else:
                        why = f"{why} (이동 {SHIFT_MAX_M:.0f}m 로도 실패: {why2})"
                tag = f"{b['name']} {item['ref'].split('/')[-1]} ({x:.1f},{y:.1f}) yaw{yaw:.0f} 발자국{L:.0f}x{W:.0f}"
                if ok:
                    print(f"[통과] {tag} 이동{shift:.0f}m 차선여유 {clr:.1f}m")
                    out.append((item, b, (x, y, yaw, L, W), clr))
                else:
                    print(f"[제외] {tag} {why}")
    return out


def spawn(world, placed):
    lib = world.get_blueprint_library()
    ids = []
    for item, b, (x, y, yaw, L, W), _ in placed:
        if item["kind"] == "mesh":
            bp = lib.find("static.prop.mesh")
            bp.set_attribute("mesh_path", item["ref"])
            if item["scale"] is not None and bp.has_attribute("scale"):
                bp.set_attribute("scale", str(item["scale"]))
        else:
            found = lib.filter(item["ref"])
            if not found:
                print(f"[소환 실패] 블루프린트 없음 {item['ref']}")
                continue
            bp = found[0]
        tf = carla.Transform(carla.Location(x=x, y=y, z=0.0), carla.Rotation(yaw=yaw))
        a = world.try_spawn_actor(bp, tf)
        if a is None:
            print(f"[소환 실패] {b['name']} {item['ref']}")
            continue
        ids.append(a.id)
    if world.get_settings().synchronous_mode:
        world.tick()
    else:
        world.wait_for_tick()
    for i in ids:
        a = world.get_actor(i)
        print(f"[소환] id {i} {a.type_id} {a.attributes.get('mesh_path', '')} loc ({a.get_location().x:.1f},"
              f"{a.get_location().y:.1f},{a.get_location().z:.2f})")
    return ids


def list_blueprints(world, out):
    lib = world.get_blueprint_library()
    props = sorted(bp.id for bp in lib.filter("static.prop.*"))
    vehicles = sorted(bp.id for bp in lib.filter("vehicle.*"))
    groups = {
        "식생(나무·화분·풀)": r"tree|plant|bush|grass|flower|pot|vegetation",
        "거리 시설(가로등·벤치·쓰레기통·정류장 등)": r"bench|bin|trash|streetlight|light|lamp|busstop|bus_stop|"
                                       r"mailbox|vending|kiosk|bike|fountain|clock|table|chair|advert|atm|"
                                       r"guardrail|haybale|swing|slide|plasticchair|gnome|doghouse",
        "공사 자재(배리어·콘 등)": r"barrier|cone|construction|warning|barrel|streetbarrier|ironplank|"
                             r"trafficwarning|creased|box|pallet|calibrator|dirtdebris|brokentile",
        "큰 구조물(건물 비슷한 크기 후보)": r"container|shelter|gate|house|building|kiosk|warehouse|garage",
    }
    with open(out, "w") as f:
        f.write(f"# s22 블루프린트 목록 {time.strftime('%Y-%m-%d %H:%M:%S')} 서버 맵 {world.get_map().name}\n")
        f.write(f"# static.prop.* {len(props)}개 / vehicle.* {len(vehicles)}개\n\n")
        for g, pat in groups.items():
            m = [p for p in props if re.search(pat, p, re.I)]
            f.write(f"## {g} {len(m)}개\n" + "\n".join(m) + "\n\n")
        f.write(f"## static.prop.* 전체 {len(props)}개\n" + "\n".join(props) + "\n\n")
        f.write(f"## vehicle.* 전체 {len(vehicles)}개\n")
        for v in vehicles:
            bp = lib.find(v)
            f.write(f"{v} wheels={bp.get_attribute('number_of_wheels').as_int() if bp.has_attribute('number_of_wheels') else '?'}"
                    f" base_type={bp.get_attribute('base_type').as_str() if bp.has_attribute('base_type') else '?'}\n")
    print(f"목록 -> {out} (static.prop {len(props)}, vehicle {len(vehicles)})")


def draw(world, frame, rows, life):
    for b in rows:
        x, y = frame.carla(float(b["lat"]), float(b["lon"]))
        L, W, yaw = float(b["len_m"]), float(b["wid_m"]), float(b["yaw_deg"])
        bb = carla.BoundingBox(carla.Location(x=x, y=y, z=5.0), carla.Vector3D(L / 2, W / 2, 5.0))
        world.debug.draw_box(bb, carla.Rotation(yaw=yaw), 0.3, carla.Color(255, 140, 0), life)
        world.debug.draw_string(carla.Location(x=x, y=y, z=12.0), b["name"], False, carla.Color(255, 255, 255), life)
    print(f"디버그 상자 {len(rows)}개, life_time {life}s")


def clear(world):
    ids = []
    try:
        ids = json.loads(STATE.read_text()).get("ids", [])
    except (OSError, ValueError):
        pass
    extra = [a.id for a in world.get_actors().filter("static.prop.*") if a.id not in ids]
    n = 0
    for i in ids + extra:
        a = world.get_actor(i)
        if a is not None and a.destroy():
            n += 1
    STATE.write_text(json.dumps({"ids": [], "cleared": time.strftime("%Y-%m-%d %H:%M:%S")}))
    left = len(world.get_actors().filter("static.prop.*"))
    print(f"제거 {n}개 (상태 파일 {len(ids)}, 그 밖 static.prop {len(extra)}), 남은 static.prop {left}")


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--list", type=Path)
    g.add_argument("--dry-run", action="store_true")
    g.add_argument("--apply", action="store_true")
    g.add_argument("--clear", action="store_true")
    g.add_argument("--draw", action="store_true")
    ap.add_argument("--only", default="")
    ap.add_argument("--life", type=float, default=300.0)
    args = ap.parse_args()

    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)
    world = client.get_world()
    if args.list:
        list_blueprints(world, args.list)
        return
    if args.clear:
        clear(world)
        return
    cmap = world.get_map()
    text = cmap.to_opendrive()
    print(f"server_map_sha256={hashlib.sha256(text.encode()).hexdigest()[:16]} ({cmap.name})")
    frame = Frame(text)
    rows = load_buildings(args.only)
    if args.draw:
        draw(world, frame, rows, args.life)
        return
    route_pts, stops = route_points(cmap, text)
    print(f"루트 검사점 {len(route_pts)}, 정차점 {len(stops)}")
    placed = place(cmap, frame, rows, route_pts, stops)
    if args.dry_run:
        return
    mem("소환 전")
    ids = spawn(world, placed)
    try:
        prev = json.loads(STATE.read_text()).get("ids", [])
    except (OSError, ValueError):
        prev = []
    STATE.write_text(json.dumps({"ids": prev + ids, "applied": time.strftime("%Y-%m-%d %H:%M:%S")}))
    time.sleep(5.0)
    mem("소환 5s 후")


if __name__ == "__main__":
    main()
