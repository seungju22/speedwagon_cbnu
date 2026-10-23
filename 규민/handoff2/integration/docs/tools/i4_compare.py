"""I4: Autoware 가 받는 자차 위치·방향(/localization/kinematic_state) vs CARLA 실제, 매 초(실제 시간) 기록
컨테이너 안에서 실행(rclpy + carla). CARLA 쪽 뒷바퀴 축은 매번 physics_control 바퀴 위치(cm)로 다시 잰다.
(정답 측위 노드는 시작할 때 한 번 잰 offset 을 쓰므로 서로 독립은 아니지만 offset 변화는 드러난다)
사용: python3 i4_compare.py <out.csv> <timeout_wall_s> [--role ego_vehicle]
열: wall, clock(sim), ks_stamp, carla_frame, aw_x, aw_y, aw_yaw, ca_x, ca_y, ca_yaw, d_lon, d_lat, d_yaw_deg, v
d_lon/d_lat 는 CARLA 기준 Autoware 위치 차이(CARLA 진행 방향 앞+, 왼쪽+)
"""
import sys, math, time, carla, rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
out, TO = sys.argv[1], float(sys.argv[2])
role = sys.argv[sys.argv.index("--role") + 1] if "--role" in sys.argv else "ego_vehicle"
c = carla.Client("localhost", 2000); c.set_timeout(20); w = c.get_world()


def ego():
    for v in w.get_actors().filter("vehicle.*"):
        if v.attributes.get("role_name") == role:
            return v


class N(Node):
    def __init__(s):
        super().__init__("i4_compare")
        s.ks = None; s.clock = None
        s.create_subscription(Odometry, "/localization/kinematic_state", lambda m: setattr(s, "ks", m), 10)
        s.create_subscription(Clock, "/clock", lambda m: setattr(s, "clock", m.clock.sec + m.clock.nanosec * 1e-9), 10)


rclpy.init(); n = N(); f = open(out, "w")
f.write("wall,clock,ks_stamp,carla_frame,aw_x,aw_y,aw_yaw,ca_x,ca_y,ca_yaw,d_lon,d_lat,d_yaw_deg,v\n")
t0 = time.time(); nxt = t0; e = None
while rclpy.ok() and time.time() - t0 < TO:
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.time() < nxt: continue
    nxt += 1.0
    if e is None: e = ego()
    if e is None or n.ks is None: continue
    snap = w.get_snapshot()
    t = e.get_transform(); wh = e.get_physics_control().wheels
    cx, cy = (wh[2].position.x + wh[3].position.x) / 200.0, -(wh[2].position.y + wh[3].position.y) / 200.0
    cyaw = -math.radians(t.rotation.yaw)
    p = n.ks.pose.pose; q = p.orientation
    ayaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
    dx, dy = p.position.x - cx, p.position.y - cy
    dlon = dx * math.cos(cyaw) + dy * math.sin(cyaw); dlat = -dx * math.sin(cyaw) + dy * math.cos(cyaw)
    dyaw = math.degrees((ayaw - cyaw + math.pi) % (2 * math.pi) - math.pi)
    st = n.ks.header.stamp.sec + n.ks.header.stamp.nanosec * 1e-9
    f.write("%.3f,%.3f,%.3f,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n" % (
        time.time(), n.clock or -1, st, snap.frame, p.position.x, p.position.y, math.degrees(ayaw),
        cx, cy, math.degrees(cyaw), dlon, dlat, dyaw, n.ks.twist.twist.linear.x)); f.flush()
