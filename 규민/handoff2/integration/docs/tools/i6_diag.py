"""I6 정지 진단(컨테이너 안, 2026-10-11): 4초 동안 virtual_wall 류 MarkerArray 의 정지 사유 글자와 벽 위치,
API velocity/steering factor, MRM·운전 모드, 최종 궤적 속도 0 점 수, 자차 위치를 한 번 적는다. 토픽 구독만 한다"""
import time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import MarkerArray
from autoware_planning_msgs.msg import Trajectory
from autoware_adapi_v1_msgs.msg import VelocityFactorArray, SteeringFactorArray, MrmState, OperationModeState
from nav_msgs.msg import Odometry
rclpy.init(); n = Node("i6_diag"); out = {}
tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
walls = [t for t, ty in n.get_topic_names_and_types() if "virtual_wall" in t and "visualization_msgs/msg/MarkerArray" in ty]
def wall_cb(t):
    def f(m):
        for mk in m.markers:
            if mk.text or "stop" in mk.ns or "slow" in mk.ns or "dead" in mk.ns:
                p = mk.pose.position
                out.setdefault(("wall", t), set()).add(f"ns={mk.ns} text='{mk.text}' ({p.x:.2f},{p.y:.2f}) action={mk.action}")
    return f
time.sleep(1.0)
walls = [t for t, ty in n.get_topic_names_and_types() if "virtual_wall" in t]
for t in walls: n.create_subscription(MarkerArray, t, wall_cb(t), 10)
def put(k): return lambda m: out.__setitem__(k, m)
n.create_subscription(VelocityFactorArray, "/api/planning/velocity_factors", put("vf"), 10)
n.create_subscription(SteeringFactorArray, "/api/planning/steering_factors", put("sf"), 10)
n.create_subscription(MrmState, "/api/fail_safe/mrm_state", put("mrm"), tl)
n.create_subscription(OperationModeState, "/api/operation_mode/state", put("om"), tl)
n.create_subscription(Trajectory, "/planning/scenario_planning/trajectory", put("traj"), 10)
n.create_subscription(Odometry, "/localization/kinematic_state", put("odom"), 10)
t0 = time.time()
while time.time() - t0 < 4.0: rclpy.spin_once(n, timeout_sec=0.1)
print("virtual_wall 토픽", len(walls))
for k, v in out.items():
    if isinstance(k, tuple):
        print("==", k[1]); [print("  ", x) for x in sorted(v)]
if "vf" in out:
    print("== velocity_factors", len(out["vf"].factors))
    for f in out["vf"].factors: print("  ", f.behavior, f.status, f"dist {f.distance:.1f}", f"({f.pose.position.x:.2f},{f.pose.position.y:.2f})", f.detail)
if "sf" in out:
    print("== steering_factors", len(out["sf"].factors))
    for f in out["sf"].factors: print("  ", f.behavior, f.status, f.detail)
if "mrm" in out: print("mrm state", out["mrm"].state, "behavior", out["mrm"].behavior)
if "om" in out: print("op mode", out["om"].mode, "auto_avail", out["om"].is_autonomous_mode_available)
if "traj" in out:
    pts = out["traj"].points; z = [i for i, p in enumerate(pts) if abs(p.longitudinal_velocity_mps) < 1e-3]
    print("trajectory 점", len(pts), "속도 0 점", len(z), "첫 0", z[0] if z else None,
          f"({pts[z[0]].pose.position.x:.2f},{pts[z[0]].pose.position.y:.2f})" if z else "")
if "odom" in out:
    p = out["odom"].pose.pose.position; print(f"ego map ({p.x:.2f},{p.y:.2f})")
