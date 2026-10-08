#!/usr/bin/env python3
# 맵세션39 준비(다음 서버 세션용, 이번 세션 미실행): 셔틀 후보 차량 바퀴 위치 원값 덤프. 주행 없음, 소환 직후 기록 후 지움
# 세션38 s38_spawn_trial.py 는 계산값(축거)만 남겼다. fusorosa 축거 5.630m 가 Rosa 공개 축거 3.490~4.550m 보다 커서 원값을 다시 본다
# 출력: 바퀴 순서(physics_control.wheels 인덱스) 별 position 원값(문서상 cm, 세계 좌표), 차 위치(m)와의 거리(cm 가정·m 가정 둘 다),
#       차 전방축 투영(앞뒤), 측방 투영(좌우), 축거 = 앞 두 바퀴 중점 - 뒤 두 바퀴 중점, bbox 길이·폭·높이와 bbox 중심 오프셋
# 전제: 서버 v2 로드 상태, 비동기 모드. r1392 s=28 lane -1 위에 하나씩 소환
# 사용: .venv-carla/bin/python map/scripts/s39_vehicle_raw.py
import math
import time

import carla

VEHICLES = ["vehicle.mitsubishi.fusorosa", "vehicle.mercedes.sprinter", "vehicle.toyota.prius", "vehicle.audi.a2"]


def main():
    c = carla.Client("localhost", 2000)
    c.set_timeout(30.0)
    w = c.get_world()
    if w.get_settings().synchronous_mode:
        raise SystemExit("동기 모드 상태 - 중단")
    wp = w.get_map().get_waypoint_xodr(1392, -1, 28.0)
    tf = carla.Transform(wp.transform.location + carla.Location(z=0.5), wp.transform.rotation)
    lib = w.get_blueprint_library()
    for vid in VEHICLES:
        found = lib.filter(vid)
        if not found:
            print(f"{vid}: 블루프린트 없음")
            continue
        v = w.try_spawn_actor(found[0], tf)
        if v is None:
            print(f"{vid}: 소환 실패")
            continue
        try:
            time.sleep(2.0)
            vt = v.get_transform()
            f, r = vt.get_forward_vector(), vt.get_right_vector()
            bb = v.bounding_box
            print(f"{vid}: 차 위치 m ({vt.location.x:.3f},{vt.location.y:.3f},{vt.location.z:.3f}) yaw {vt.rotation.yaw:.2f}")
            print(f"  bbox extent m ({bb.extent.x:.3f},{bb.extent.y:.3f},{bb.extent.z:.3f}) -> 길이 {2*bb.extent.x:.3f} 폭 {2*bb.extent.y:.3f}"
                  f" 높이 {2*bb.extent.z:.3f}, bbox 중심 오프셋 ({bb.location.x:.3f},{bb.location.y:.3f},{bb.location.z:.3f})")
            pts = []
            for i, wh in enumerate(v.get_physics_control().wheels):
                p = wh.position
                pm = (p.x / 100, p.y / 100, p.z / 100)
                d_cm = math.dist(pm[:2], (vt.location.x, vt.location.y))
                d_m = math.dist((p.x, p.y), (vt.location.x, vt.location.y))
                lon = (pm[0] - vt.location.x) * f.x + (pm[1] - vt.location.y) * f.y
                lat = (pm[0] - vt.location.x) * r.x + (pm[1] - vt.location.y) * r.y
                pts.append((i, pm, lon))
                print(f"  wheel[{i}] 원값 ({p.x:.2f},{p.y:.2f},{p.z:.2f}) | cm 가정 차까지 {d_cm:.3f}m, m 가정 {d_m:.1f}m |"
                      f" 앞뒤 {lon:+.3f}m 좌우 {lat:+.3f}m | max_steer {wh.max_steer_angle:.2f} radius {wh.radius:.2f}")
            s = sorted(pts, key=lambda q: q[2])
            fm = [(s[-1][1][k] + s[-2][1][k]) / 2 for k in range(2)]
            rm = [(s[0][1][k] + s[1][1][k]) / 2 for k in range(2)]
            print(f"  앞 = wheel{[s[-1][0], s[-2][0]]}, 뒤 = wheel{[s[0][0], s[1][0]]}, 축거 {math.dist(fm, rm):.3f}m")
        finally:
            v.destroy()
    print("남은 차량", [a.id for a in w.get_actors().filter("vehicle.*")])


if __name__ == "__main__":
    main()
