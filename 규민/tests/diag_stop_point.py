"""진단 2·3: 세션7 주행 정지 지점 주변 조회 + 맵 전체 신호/표지 전수.

서버에 읽기 전용으로만 접근한다(액터 스폰·맵 로드 없음).
실행 전에 서버에 878 road 맵이 로드돼 있어야 한다.
"""
import csv
import math
from collections import Counter
from pathlib import Path

import carla

HOST, PORT, TIMEOUT = "localhost", 2000, 30.0
STOP_XY = (228.3, -995.8)
RADIUS = 100.0
PLAN_CSV = Path(__file__).resolve().parents[1] / "docs/logs/drive_plan_20260919_231354.csv"


def fmt_wp(wp):
    return (f"road{wp.road_id} lane{wp.lane_id} s={wp.s:.2f} junc={wp.is_junction} "
            f"lane_change={wp.lane_change} ({wp.transform.location.x:.1f},"
            f"{wp.transform.location.y:.1f}) yaw={wp.transform.rotation.yaw:.1f}")


def main():
    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)
    world = client.get_world()
    cmap = world.get_map()
    print(f"맵: {cmap.name}")
    if "OpenDriveMap" not in cmap.name:
        print("878 road 맵이 아니다. 중단.")
        return

    plan = list(csv.DictReader(open(PLAN_CSV)))
    plan_roads = {int(p["road_id"]) for p in plan}
    loc = carla.Location(x=STOP_XY[0], y=STOP_XY[1], z=0.5)

    print("\n=== [진단2] 정지 지점 waypoint ===")
    wp = cmap.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Any)
    print("지점:", fmt_wp(wp))
    dx = math.hypot(wp.transform.location.x - loc.x, wp.transform.location.y - loc.y)
    print(f"지점-waypoint 거리 {dx:.2f}m")
    for d in (5, 20, 50):
        nxt = wp.next(d)
        print(f"next({d}m): " + (" | ".join(fmt_wp(n) for n in nxt) if nxt else "없음"))

    print("\n=== [진단2] 가장 가까운 set_path 경로점과 차량 앞/뒤 ===")
    yaw = math.radians(wp.transform.rotation.yaw)
    fx, fy = math.cos(yaw), math.sin(yaw)
    rows = []
    for p in plan:
        px, py = float(p["x"]), float(p["y"])
        rx, ry = px - loc.x, py - loc.y
        rows.append((math.hypot(rx, ry), rx * fx + ry * fy, p))
    rows.sort(key=lambda r: r[0])
    for dist, along, p in rows[:4]:
        side = "앞" if along > 0 else "뒤"
        print(f"idx{p['idx']} road{p['road_id']} s={p['s']} 거리 {dist:.2f}m 진행방향 {side}({along:+.2f}m)")

    print(f"\n=== [진단2] 반경 {RADIUS:.0f}m 내 traffic.* 액터 ===")
    found = 0
    for a in world.get_actors().filter("traffic.*"):
        d = a.get_location().distance(loc)
        if d <= RADIUS:
            found += 1
            l = a.get_location()
            print(f"{a.type_id} id={a.id} ({l.x:.1f},{l.y:.1f}) d={d:.1f}m")
    print(f"반경 내 traffic.* 액터: {found}개")

    print("\n=== [진단2] 정지 지점 근처 그 외 액터(15m) ===")
    for a in world.get_actors():
        if a.type_id.startswith(("traffic.", "sensor.")):
            continue
        d = a.get_location().distance(loc)
        if d <= 15:
            print(f"{a.type_id} id={a.id} d={d:.1f}m")

    print("\n=== [진단2] 정지 지점 15m 내 환경 오브젝트(라벨별) ===")
    try:
        objs = world.get_environment_objects(carla.CityObjectLabel.Any)
        near = [o for o in objs if o.transform.location.distance(loc) <= 15]
        print(f"전체 {len(objs)}개 중 15m 내 {len(near)}개:",
              dict(Counter(str(o.type) for o in near)))
    except Exception as e:  # noqa: BLE001
        print("환경 오브젝트 조회 실패:", e)

    print("\n=== [진단3] 맵 전체 신호/표지 전수 ===")
    types = Counter(a.type_id for a in world.get_actors().filter("traffic.*"))
    print("traffic.* 액터:", dict(types) if types else "없음")
    lms = cmap.get_all_landmarks()
    print(f"OpenDRIVE landmark(signal) 총 {len(lms)}개")
    print("landmark 타입:", dict(Counter(f"{l.type}/{l.name}" for l in lms)))
    for l in lms:
        t = l.transform.location
        on_plan = l.road_id in plan_roads
        print(f"lm id={l.id} type={l.type} name={l.name} road{l.road_id} s={l.s:.1f} "
              f"({t.x:.1f},{t.y:.1f}) 확정15road={'예' if on_plan else '아니오'}")
    tl = world.get_traffic_lights_in_junction if False else None  # noqa: F841
    print("\n확정 경로 15 road 에 걸린 landmark:",
          sum(1 for l in lms if l.road_id in plan_roads), "개")


if __name__ == "__main__":
    main()
