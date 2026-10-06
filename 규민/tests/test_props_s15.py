#!/usr/bin/env python3
# 맵세션15 Phase E. static.prop.mesh 소품 시험(나무 먼저, 그다음 건물, 합계 5개).
# 세션4 조사: Content/ 경로 = /Game/ 경로 1:1, 나무 167종, 건물 SM_House 계열.
# 확인 항목: 스폰 성공 여부, 단계별 메모리 증가량, scale 속성(블루프린트에 'scale'
# Float, 기본 1.0 - 세션15 서버 조회) 효과.
# 배치: road1247(출발 road) 오른쪽 도로변. 위치 = lane -1 중심 + 오른쪽 단위벡터 x 오프셋.
# 비동기 모드 전제(주행 후 복귀 확인됨). 액터는 스크립트 종료 후에도 남는다 -> --destroy.
#   python map/tests/test_props_s15.py            소환 + 측정 + spectator 이동
#   python map/tests/test_props_s15.py --destroy  static.prop.mesh 액터 전부 제거
import argparse
import subprocess
import time

import carla

HOST, PORT, TIMEOUT = "localhost", 2000, 20.0
ROAD, LANE = 1247, -1
SETTLE_S = 5.0                      # 소환 후 메모리 측정까지 대기
TREE = "/Game/Carla/Static/Vegetation/Trees/{0}.{0}"
BUILDING = "/Game/Carla/Static/Building/{0}.{0}"
# (단계, 메시 경로, s, 오른쪽 오프셋 m(차선 중심 기준), scale)
PROPS = [
    ("tree", TREE.format("SM_Acer_02"), 30.0, 7.0, 1.0),
    ("tree", TREE.format("SM_Ash_01"), 45.0, 7.0, 1.0),
    ("tree", TREE.format("SM_Acer_02"), 60.0, 7.0, 2.0),   # scale 비교(1번과 같은 수종)
    ("building", BUILDING.format("SM_House01"), 90.0, 22.0, 1.0),
    ("building", BUILDING.format("SM_House02"), 125.0, 22.0, 1.0),
]
VIEW_S, VIEW_UP_M, VIEW_BACK_M = 60.0, 35.0, 40.0


def mem(tag):
    out = subprocess.run(["free", "-h"], capture_output=True, text=True).stdout
    gtt = None
    for p in ("/sys/class/drm/card0/device/mem_info_gtt_used",
              "/sys/class/drm/card1/device/mem_info_gtt_used"):
        try:
            gtt = int(open(p).read()) / 2 ** 30
            break
        except OSError:
            pass
    lines = out.splitlines()
    print(f"--- 메모리 [{tag}] {time.strftime('%H:%M:%S')} ---")
    print(lines[1])
    print(lines[2])
    print(f"gtt_used: {gtt:.2f}GiB" if gtt is not None else "gtt_used: 없음")


def spawn_props(world, carla_map):
    bp_lib = world.get_blueprint_library()
    spawned = []
    mem("소환 전")
    for stage in ("tree", "building"):
        for kind, path, s, off, scale in PROPS:
            if kind != stage:
                continue
            wp = carla_map.get_waypoint_xodr(ROAD, LANE, s)
            rv = wp.transform.get_right_vector()
            base = wp.transform.location
            loc = carla.Location(base.x + rv.x * off, base.y + rv.y * off, base.z)
            tf = carla.Transform(loc, carla.Rotation(yaw=wp.transform.rotation.yaw))
            bp = bp_lib.find("static.prop.mesh")
            bp.set_attribute("mesh_path", path)
            bp.set_attribute("scale", str(scale))
            actor = world.try_spawn_actor(bp, tf)
            name = path.rsplit(".", 1)[-1]
            if actor is None:
                print(f"[실패] {kind} {name} scale={scale} s={s} off={off}")
                continue
            ext = actor.bounding_box.extent
            print(f"[소환] {kind} {name} scale={scale} actor_id={actor.id} "
                  f"loc=({loc.x:.1f},{loc.y:.1f},{loc.z:.2f}) "
                  f"bbox_extent=({ext.x:.2f},{ext.y:.2f},{ext.z:.2f}) "
                  f"attr_scale={actor.attributes.get('scale')}")
            spawned.append(actor)
        time.sleep(SETTLE_S)
        mem(f"{stage} 소환 {SETTLE_S:g}s 후")

    wp = carla_map.get_waypoint_xodr(ROAD, LANE, VIEW_S)
    f = wp.transform.get_forward_vector()
    rv = wp.transform.get_right_vector()
    c = wp.transform.location
    cam = carla.Location(c.x - f.x * VIEW_BACK_M - rv.x * 10.0,
                         c.y - f.y * VIEW_BACK_M - rv.y * 10.0, c.z + VIEW_UP_M)
    world.get_spectator().set_transform(carla.Transform(
        cam, carla.Rotation(pitch=-35.0, yaw=wp.transform.rotation.yaw + 20.0)))
    print(f"spectator: ({cam.x:.1f},{cam.y:.1f},{cam.z:.1f}) pitch -35")
    print(f"소환 {len(spawned)}/{len(PROPS)}: {[a.id for a in spawned]}")


def destroy_props(world):
    props = [a for a in world.get_actors() if a.type_id == "static.prop.mesh"]
    for a in props:
        a.destroy()
    print(f"static.prop.mesh 제거: {len(props)}개 {[a.id for a in props]}")
    time.sleep(SETTLE_S)
    mem(f"제거 {SETTLE_S:g}s 후")


def main():
    ap = argparse.ArgumentParser(description="세션15 소품 시험(static.prop.mesh)")
    ap.add_argument("--destroy", action="store_true", help="static.prop.mesh 전부 제거")
    args = ap.parse_args()
    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)
    world = client.get_world()
    s = world.get_settings()
    print(f"synchronous_mode={s.synchronous_mode}")
    if s.synchronous_mode:
        raise SystemExit("동기 모드 상태. --restore-async 먼저")
    if args.destroy:
        destroy_props(world)
    else:
        spawn_props(world, world.get_map())


if __name__ == "__main__":
    main()
