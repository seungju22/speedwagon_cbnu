"""I6-2(i6_route.py 사본, s54 xodr·s54 pairs30, 2026-10-12). I6(i5_route.py 사본, v3.1 후보 xodr·pairs, 2026-10-11): 쌍 이름 -> 출발 spawn_point(CARLA 좌표, 브리지 인자 순서 x,y,z,roll,pitch,yaw) 와 목표(ROS map 좌표, 뒷바퀴 축 자세)
출발 = 첫 road lane -1 start_s 차선 중심·차선 방향, 목표 = 마지막 road lane -1 end_s 차선 중심·차선 방향(I2 방식: 차선 중심 위 뒷바퀴 축)
오프라인 carla.Map(동결본 v3). 사용: python i5_route.py <쌍> -> 셸 변수 줄 출력
"""
import json, math, sys
from pathlib import Path
import carla
X = Path.home() / "campus_mobility_sim/map/maps/cbnu_campus_frozen_v31.xodr"
P = json.loads((Path.home() / "campus_mobility_sim/map/build_v31_s54/pairs30.json").read_text())[sys.argv[1]]
m = carla.Map("OpenDriveMap", X.read_text())
s = m.get_waypoint_xodr(P["roads"][0], -1, P["start_s"]).transform
g = m.get_waypoint_xodr(P["roads"][-1], -1, P["end_s"]).transform
gyaw = -g.rotation.yaw
print(f'SPAWN={s.location.x:.2f},{s.location.y:.2f},0.30,0.0,0.0,{s.rotation.yaw:.2f}')
print(f'GX={g.location.x:.3f}; GY={-g.location.y:.3f}; GYAW={gyaw:.2f}; QZ={math.sin(math.radians(gyaw) / 2):.6f}; QW={math.cos(math.radians(gyaw) / 2):.6f}')
