#!/usr/bin/env python3
# 맵세션13 A-4. 벽이 실제로 사라졌는지 서버에 직접 묻는다(읽기 전용, 액터 생성 없음).
# road1240 끝(세션12 첫 충돌, 중앙선 벽 끝 (421.9,-662.5)) 앞 20m 구간에서
# 차선 중심 -> 중앙선 너머 1m 로 노면 위 0.5m 높이 수평 광선을 쏜다(world.cast_ray).
# 벽(높이 1.0)이 있으면 중앙선(t=0) 근처에서 맞는다. 없으면 중앙선 부근 충돌점이 없다.
# 대조로 바깥쪽(인도 바깥 t=-6.15 너머 1m)도 같은 높이로 쏜다(바깥 벽도 같이 사라져야 함).
# 근거: https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.cast_ray
import math
import carla

WALL_END = carla.Location(x=421.9, y=-662.5, z=0.0)
ROAD_ID = 1240
H = 0.5          # 노면 위 광선 높이(m)
BEYOND = 1.0     # 경계 너머 연장(m)
OUTER_T = 6.15   # 기준선~인도 바깥(세션11 실측)


def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(30.0)
    world = client.get_world()
    m = world.get_map()
    wp_end = m.get_waypoint(WALL_END, project_to_road=True)
    print(f"end wp road{wp_end.road_id} lane{wp_end.lane_id} s={wp_end.s:.2f}")
    if wp_end.road_id != ROAD_ID:
        print("경고: 투영 road 가 1240 아님")
    # 끝에서 2m 간격으로 뒤로 10점
    wps = []
    w = wp_end
    for _ in range(10):
        wps.append(w)
        prev = w.previous(2.0)
        if not prev or prev[0].road_id != ROAD_ID:
            break
        w = prev[0]
    for w in wps:
        tf = w.transform
        c = tf.location
        r = tf.get_right_vector()
        half = w.lane_width / 2.0
        z = c.z + H
        start = carla.Location(c.x, c.y, z)
        # 왼쪽(중앙선) = -right 방향, 차선 중심에서 half 만큼이 t=0
        left_end = carla.Location(c.x - r.x * (half + BEYOND),
                                  c.y - r.y * (half + BEYOND), z)
        # 오른쪽 바깥: 기준선에서 OUTER_T -> 중심에서 OUTER_T - half
        d = OUTER_T - half + BEYOND
        right_end = carla.Location(c.x + r.x * d, c.y + r.y * d, z)
        for name, end, edge in (("L", left_end, half), ("R", right_end, OUTER_T - half)):
            hits = world.cast_ray(start, end)
            desc = []
            for h in hits:
                p = h.location
                dist = math.hypot(p.x - c.x, p.y - c.y)
                desc.append(f"{dist:.2f}m z{p.z - c.z:+.2f} {h.label}")
            print(f"s={w.s:7.2f} {name} edge={edge:.2f} hits={len(hits)} "
                  + ("; ".join(desc) if desc else "-"))


if __name__ == "__main__":
    main()
