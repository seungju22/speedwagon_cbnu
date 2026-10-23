"""I2: 컨테이너 안에서 주행 1회 감시 (rclpy, 코드 수정 없이 토픽만 구독)
기록: routing state 변화, 후진 명령(control_cmd velocity<0) 첫 시각,
      kinematic_state 위치·yaw, 목표 기준 앞뒤·옆 오차, 정지 위치
사용: python3 i2_watch.py <out.csv> <gx> <gy> <gyaw_deg> <timeout_wall_s>
"""
import sys, math, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry
from autoware_control_msgs.msg import Control
from autoware_adapi_v1_msgs.msg import RouteState, OperationModeState
out, gx, gy, gyaw, TO = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), math.radians(float(sys.argv[4])), float(sys.argv[5])
class W(Node):
    def __init__(s):
        super().__init__("i2_watch")
        s.f = open(out, "w"); s.t0 = time.time()
        s.f.write("wall,sim,event,x,y,yaw_deg,v,lon_err,lat_err,info\n")
        s.pose = None; s.v = 0.0; s.route = None; s.mode = None; s.rev = 0; s.last = 0
        tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        s.create_subscription(Odometry, "/localization/kinematic_state", s.odom, 10)
        s.create_subscription(Control, "/control/command/control_cmd", s.ctl, 10)
        s.create_subscription(RouteState, "/api/routing/state", s.rs, tl)
        s.create_subscription(OperationModeState, "/api/operation_mode/state", s.om, tl)
    def sim(s): return s.get_clock().now().nanoseconds * 1e-9
    def err(s):
        x, y, yaw = s.pose; dx, dy = x - gx, y - gy
        return dx * math.cos(gyaw) + dy * math.sin(gyaw), -dx * math.sin(gyaw) + dy * math.cos(gyaw)
    def log(s, ev, info=""):
        if s.pose is None: x = y = yaw = lo = la = float("nan")
        else: x, y, yaw = s.pose; lo, la = s.err()
        s.f.write("%.3f,%.3f,%s,%.3f,%.3f,%.2f,%.3f,%.3f,%.3f,%s\n" % (time.time(), s.sim(), ev, x, y, math.degrees(yaw), s.v, lo, la, info)); s.f.flush()
    def odom(s, m):
        q = m.pose.pose.orientation
        s.pose = (m.pose.pose.position.x, m.pose.pose.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))
        s.v = m.twist.twist.linear.x
        if time.time() - s.last > 5: s.last = time.time(); s.log("tick")
    def ctl(s, m):
        if m.longitudinal.velocity < -1e-3:
            s.rev += 1
            if s.rev == 1 or s.rev % 50 == 0: s.log("reverse_cmd", "n=%d vel=%.3f acc=%.3f" % (s.rev, m.longitudinal.velocity, m.longitudinal.acceleration))
    def rs(s, m):
        if m.state != s.route: s.route = m.state; s.log("route_state", str(m.state))
    def om(s, m):
        if m.mode != s.mode: s.mode = m.mode; s.log("op_mode", str(m.mode))
rclpy.init(); n = W()
while rclpy.ok() and time.time() - n.t0 < TO and n.route != 3:
    rclpy.spin_once(n, timeout_sec=0.5)
n.log("end", "route=%s rev=%d" % (n.route, n.rev))
# 도착 뒤 2초(wall) 더 받아서 정지 위치 기록
t = time.time()
while time.time() - t < 2: rclpy.spin_once(n, timeout_sec=0.2)
n.log("final")
