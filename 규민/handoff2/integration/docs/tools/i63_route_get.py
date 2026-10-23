#!/usr/bin/env python3
# I6-3(2026-10-13): 컨테이너 안에서 Autoware 경로(/planning/mission_planning/route)를 한 번 받아 레인렛 id 를 줄마다 쓴다
# 사용: i63_route_get.py <출력 파일> [대기 s]
import sys, time
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from autoware_planning_msgs.msg import LaneletRoute

rclpy.init(); n = rclpy.create_node('i63_route_get'); got = []
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(LaneletRoute, '/planning/mission_planning/route', lambda m: got.append(m), qos)
t0 = time.time(); to = float(sys.argv[2]) if len(sys.argv) > 2 else 30
while not got and time.time() - t0 < to:
    rclpy.spin_once(n, timeout_sec=0.5)
with open(sys.argv[1], 'w') as f:
    if got:
        for s in got[-1].segments:
            f.write('%d\n' % s.preferred_primitive.id)
print('segments', len(got[-1].segments) if got else 0)
