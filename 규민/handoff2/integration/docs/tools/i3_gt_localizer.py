"""I3: 정답 측위(ground truth). NDT·EKF 대신 CARLA 자차 상태를 그대로 측위 출력으로 낸다.
캠퍼스 지도는 점군(pcd)이 없어 NDT 를 못 쓴다. 그래서 localization:=false 로 띄우고 이 노드를 쓴다.
컨테이너 안에서 실행한다(rclpy + 마운트된 carla 0.9.15, PYTHONPATH=/opt/carla-pyclient).
사용: python3 i3_gt_localizer.py [--host localhost] [--port 2000] [--role ego_vehicle] [--check N]
  --check N: ROS 없이 CARLA 쪽 변환만 N 틱 출력하고 끝낸다(호스트 .venv-carla 에서도 됨)

내는 토픽 (원래 내던 노드)
  /localization/kinematic_state         nav_msgs/Odometry          (ekf_localizer 뒤 stop_filter)
  /localization/pose_with_covariance    PoseWithCovarianceStamped  (ekf_localizer)
  /localization/pose_twist_fusion_filter/pose  PoseStamped        (ekf_localizer, component_state_monitor 가 봄)
  /localization/acceleration            AccelWithCovarianceStamped (twist2accel)
  /localization/initialization_state    LocalizationInitializationState=INITIALIZED (pose_initializer)
  /tf map -> base_link                                             (ekf_localizer)
base_link 는 뒷바퀴 축 가운데. CARLA 뒷바퀴 두 개 위치(physics_control, cm)의 평균에서 구한다.
좌표: CARLA(왼손, y 반전) -> ROS map. x 같음, y·yaw·pitch 부호 반대. 브리지 carla_utils 와 같은 규칙.
시각: 브리지 /clock 은 브리지 시작부터 센 게임 시간이다(carla_data_provider GameTime).
  CARLA elapsed_seconds 와 차이가 일정하므로, /clock 을 받을 때마다 차이를 모아 틱 단위 최빈값 묶음의 중앙값을 쓴다.
"""
import argparse, collections, math, sys, threading, time
import carla

ap = argparse.ArgumentParser()
ap.add_argument("--host", default="localhost")
ap.add_argument("--port", type=int, default=2000)
ap.add_argument("--role", default="ego_vehicle")
ap.add_argument("--check", type=int, default=0)
a = ap.parse_args()


def quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def find_ego(w):
    for v in w.get_actors().filter("vehicle.*"):
        if v.attributes.get("role_name") == a.role:
            return v
    return None


def rear_axle_offset(ego):
    """뒷바퀴 축 가운데의 차 좌표(m). wheels 순서 FL FR RL RR, position 은 월드 cm."""
    wh = ego.get_physics_control().wheels
    t = ego.get_transform()
    p = carla.Location(x=(wh[2].position.x + wh[3].position.x) / 200.0,
                       y=(wh[2].position.y + wh[3].position.y) / 200.0,
                       z=(wh[2].position.z + wh[3].position.z) / 200.0)
    inv = t.get_inverse_matrix()
    lx = inv[0][0] * p.x + inv[0][1] * p.y + inv[0][2] * p.z + inv[0][3]
    ly = inv[1][0] * p.x + inv[1][1] * p.y + inv[1][2] * p.z + inv[1][3]
    return lx, ly


def state(snap_actor, off):
    """CARLA 스냅샷 한 대 -> ROS map 기준 base_link 상태"""
    t = snap_actor.get_transform()
    yaw = -math.radians(t.rotation.yaw)
    q = quat(math.radians(t.rotation.roll), -math.radians(t.rotation.pitch), yaw)
    b = t.transform(carla.Location(x=off[0], y=off[1], z=0.0))
    x, y, z = b.x, -b.y, b.z
    v, w, acc = snap_actor.get_velocity(), snap_actor.get_angular_velocity(), snap_actor.get_acceleration()
    wz = -math.radians(w.z)  # CARLA deg/s, 왼손 -> ROS rad/s
    rx, ry = x - t.location.x, y - (-t.location.y)
    vx, vy = v.x - wz * ry, -v.y + wz * rx  # 차 원점 속도 + w x r
    c, s = math.cos(yaw), math.sin(yaw)
    lon, lat = c * vx + s * vy, -s * vx + c * vy
    ax, ay = acc.x, -acc.y
    alon, alat = c * ax + s * ay, -s * ax + c * ay
    return dict(x=x, y=y, z=z, q=q, yaw=yaw, lon=lon, lat=lat, vz=v.z, wz=wz, alon=alon, alat=alat, az=acc.z)


c = carla.Client(a.host, a.port); c.set_timeout(20)
world = c.get_world()
ego = None
while ego is None:
    ego = find_ego(world)
    if ego is None:
        print("ego(role_name=%s) 기다리는 중" % a.role, flush=True)
        time.sleep(1.0)
OFF = rear_axle_offset(ego)
print("ego", ego.id, ego.type_id, "base_link offset x %.3f y %.3f" % OFF, flush=True)

if a.check:
    for _ in range(a.check):
        snap = world.wait_for_tick(10)
        sa = snap.find(ego.id)
        st = state(sa, OFF)
        t = sa.get_transform()
        print("frame %d center %.3f %.3f yaw %.2f -> base %.3f %.3f yaw %.2f v %.3f/%.3f wz %.4f" % (
            snap.frame, t.location.x, -t.location.y, -t.rotation.yaw, st["x"], st["y"],
            math.degrees(st["yaw"]), st["lon"], st["lat"], st["wz"]), flush=True)
    sys.exit(0)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, AccelWithCovarianceStamped, TransformStamped
from tf2_msgs.msg import TFMessage
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState

COV6 = [0.0] * 36
for i in range(6):
    COV6[i * 7] = 1e-4


class GT(Node):
    def __init__(s):
        super().__init__("i3_gt_localizer", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        s.p_odom = s.create_publisher(Odometry, "/localization/kinematic_state", 10)
        s.p_pose = s.create_publisher(PoseWithCovarianceStamped, "/localization/pose_with_covariance", 10)
        s.p_fuse = s.create_publisher(PoseStamped, "/localization/pose_twist_fusion_filter/pose", 10)
        s.p_acc = s.create_publisher(AccelWithCovarianceStamped, "/localization/acceleration", 10)
        s.p_init = s.create_publisher(LocalizationInitializationState, "/localization/initialization_state", tl)
        s.p_tf = s.create_publisher(TFMessage, "/tf", 100)
        s.create_subscription(Clock, "/clock", s.on_clock, 10)
        s.diffs = collections.deque(maxlen=40)
        s.offset = None; s.last_frame = -1; s.n = 0; s.lock = threading.Lock()
        s.create_timer(1.0, s.pub_init)

    def pub_init(s):
        m = LocalizationInitializationState(); m.stamp = s.get_clock().now().to_msg()
        m.state = LocalizationInitializationState.INITIALIZED
        s.p_init.publish(m)

    def on_clock(s, m):
        sim = m.clock.sec + m.clock.nanosec * 1e-9
        snap = world.get_snapshot()
        with s.lock:
            raw = snap.timestamp.elapsed_seconds - sim
            s.diffs.append((round(raw / 0.05), raw))
            if len(s.diffs) >= 10:
                # 틱 단위 최빈값으로 한 틱 어긋난 표본을 버리고, 남은 표본의 중앙값을 쓴다
                # (CARLA elapsed 는 0.05 격자에 딱 맞지 않아 반올림만 하면 0.03s 어긋났다, I4 b1)
                k = collections.Counter(d[0] for d in s.diffs).most_common(1)[0][0]
                r = sorted(d[1] for d in s.diffs if d[0] == k)
                s.offset = r[len(r) // 2]

    def on_tick(s, snap):
        with s.lock:
            off = s.offset
        if off is None or snap.frame <= s.last_frame:
            return
        sa = snap.find(ego.id)
        if sa is None:
            return
        s.last_frame = snap.frame
        st = state(sa, OFF)
        t = max(0.0, snap.timestamp.elapsed_seconds - off)
        stamp = Time(sec=int(t), nanosec=int((t - int(t)) * 1e9))
        od = Odometry(); od.header.stamp = stamp; od.header.frame_id = "map"; od.child_frame_id = "base_link"
        p = od.pose.pose
        p.position.x, p.position.y, p.position.z = st["x"], st["y"], st["z"]
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = st["q"]
        od.pose.covariance = COV6
        od.twist.twist.linear.x, od.twist.twist.linear.y = st["lon"], st["lat"]
        od.twist.twist.angular.z = st["wz"]
        od.twist.covariance = COV6
        pc = PoseWithCovarianceStamped(); pc.header = od.header; pc.pose = od.pose
        ps = PoseStamped(); ps.header = od.header; ps.pose = p
        ac = AccelWithCovarianceStamped(); ac.header.stamp = stamp; ac.header.frame_id = "base_link"
        ac.accel.accel.linear.x, ac.accel.accel.linear.y, ac.accel.accel.linear.z = st["alon"], st["alat"], st["az"]
        ac.accel.covariance = COV6
        tf = TransformStamped(); tf.header = od.header; tf.child_frame_id = "base_link"
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = st["x"], st["y"], st["z"]
        tf.transform.rotation = p.orientation
        s.p_tf.publish(TFMessage(transforms=[tf]))
        s.p_odom.publish(od); s.p_pose.publish(pc); s.p_fuse.publish(ps); s.p_acc.publish(ac)
        s.n += 1
        if s.n % 200 == 1:
            print("pub", s.n, "frame", snap.frame, "t %.2f" % t, "base %.2f %.2f yaw %.2f v %.2f" % (
                st["x"], st["y"], math.degrees(st["yaw"]), st["lon"]), flush=True)


rclpy.init()
node = GT()
node.pub_init()
cb = world.on_tick(node.on_tick)
try:
    rclpy.spin(node)
finally:
    world.remove_on_tick(cb)
