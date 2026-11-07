#!/usr/bin/env python3
"""캠퍼스 셔틀 제어 노드 (v0.6)

Autoware AD API만 사용하므로 planning_simulator / AWSIM 어디서든 동일하게 동작한다.

명령은 /shuttle/command (std_msgs/String) 로 받는다.
  call <픽업지> <목적지>   예) call library front_gate
  boarded                  승객 탑승 완료 -> 목적지로 출발
  pause / resume           일시정지 / 재개
  estop / estop_clear      비상정지 / 비상정지 해제
  speed <km/h>             최대 속도 제한 변경 (0~30, 예: speed 5)
  cancel                   현재 임무 취소 후 정지
  status                   현재 상태 로그 출력

정차 장소 이름: library, front_gate, back_gate

이벤트 감시: 자율주행 중인데 일정 시간 이상 서 있으면 경고 로그를 남기고,
다시 움직이면 재개 로그를 남긴다 (장애물, 보행로 정지, 속도 제한 등).
"""
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger

from autoware_internal_planning_msgs.msg import VelocityLimit
from autoware_adapi_v1_msgs.msg import RouteState, OperationModeState
from autoware_adapi_v1_msgs.srv import SetRoutePoints, ClearRoute, ChangeOperationMode
from tier4_control_msgs.srv import SetPause

# 니시신주쿠 샘플 맵 위의 임의 지점 3곳 (map 프레임, 단위 m, yaw는 라디안).
# 차량을 실제로 세워 /api/vehicle/kinematics 에서 읽은 값이다.
# 캠퍼스 맵으로 바꿀 때 이 좌표만 교체하면 된다.
STOPS = {
    "library":    {"x": 81824.24, "y": 50354.96, "yaw": -1.3593},   # 1번 지점
    "front_gate": {"x": 81684.07, "y": 50597.71, "yaw": -2.9550},   # 2번 지점
    "back_gate":  {"x": 81650.66, "y": 50303.88, "yaw": 1.6968},    # 3번 지점
}

# 호출받은 픽업지에 이미 이 거리(m) 안에 있으면 이동 없이 바로 탑승 대기로 넘어간다
NEAR_STOP_M = 5.0

# 자율주행 중인데 이 시간(초) 이상 서 있으면 "정지 이벤트"로 경고한다
STALL_SEC = 8.0
STALL_REPEAT_SEC = 15.0
SPEED_LIMIT_MAX_KMH = 30.0

IDLE, TO_PICKUP, WAIT_BOARDING, TO_DEST = "IDLE", "TO_PICKUP", "WAIT_BOARDING", "TO_DEST"


def make_pose(x, y, yaw):
    p = Pose()
    p.position.x, p.position.y = float(x), float(y)
    p.orientation.z = math.sin(yaw / 2.0)
    p.orientation.w = math.cos(yaw / 2.0)
    return p


class ShuttleNode(Node):
    def __init__(self):
        super().__init__("shuttle_node")

        # AD API 상태 토픽은 transient_local 이므로 QoS를 맞춘다
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(RouteState, "/api/routing/state", self.on_route, qos)
        self.create_subscription(OperationModeState, "/api/operation_mode/state", self.on_mode, qos)
        self.create_subscription(String, "/shuttle/command", self.on_command, 10)
        qos_be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, "/localization/kinematic_state", self.on_odom, qos_be)
        self.pose_xy = None
        self.speed = None

        # 속도 제한 토픽의 구독자가 transient_local 이므로 발행자도 맞춘다
        qos_latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 history=HistoryPolicy.KEEP_LAST)
        self.pub_limit = self.create_publisher(
            VelocityLimit, "/planning/scenario_planning/max_velocity", qos_latched)
        self.cli_estop = self.create_client(
            Trigger, "/control/vehicle_cmd_gate/external_emergency_stop")
        self.cli_estop_clear = self.create_client(
            Trigger, "/control/vehicle_cmd_gate/clear_external_emergency_stop")

        self.cli_route = self.create_client(SetRoutePoints, "/api/routing/set_route_points")
        self.cli_clear = self.create_client(ClearRoute, "/api/routing/clear_route")
        self.cli_auto = self.create_client(ChangeOperationMode, "/api/operation_mode/change_to_autonomous")
        self.cli_stop = self.create_client(ChangeOperationMode, "/api/operation_mode/change_to_stop")
        self.cli_pause = self.create_client(SetPause, "/control/vehicle_cmd_gate/set_pause")

        self.state = IDLE
        self.queue = deque()          # (pickup, dest)
        self.job = None               # 현재 임무 (pickup, dest)
        self.route_state = RouteState.UNKNOWN
        self.mode = OperationModeState.UNKNOWN
        self.route_seen_set = False   # 새 경로가 SET 이 된 것을 본 뒤에만 도착 판정
        self.route_requested = False
        self.auto_requested = False
        self.paused = False
        self.last_auto_try = 0.0
        self.estopped = False
        self.speed_limit_kmh = None
        self.stall_since = None
        self.stall_reported = False
        self.stall_last_log = 0.0

        self.create_timer(0.2, self.tick)
        self.get_logger().info("셔틀 노드 시작. /shuttle/command 로 명령을 보내세요.")

    # ---------- 입력 ----------
    def on_route(self, msg):
        self.route_state = msg.state
        if msg.state == RouteState.SET:
            self.route_seen_set = True

    def on_mode(self, msg):
        self.mode = msg.mode

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose_xy = (p.x, p.y)
        self.speed = msg.twist.twist.linear.x

    def near_stop(self, stop_name):
        if self.pose_xy is None:
            return False
        s = STOPS[stop_name]
        return math.hypot(self.pose_xy[0] - s["x"], self.pose_xy[1] - s["y"]) < NEAR_STOP_M

    def on_command(self, msg):
        args = msg.data.strip().split()
        if not args:
            return
        cmd = args[0].lower()
        if cmd == "call" and len(args) == 3:
            pickup, dest = args[1], args[2]
            if pickup not in STOPS or dest not in STOPS or pickup == dest:
                self.get_logger().warn(f"잘못된 호출: {pickup} -> {dest} (가능: {list(STOPS)})")
                return
            self.queue.append((pickup, dest))
            self.get_logger().info(f"호출 접수: {pickup} -> {dest} (대기열 {len(self.queue)})")
        elif cmd == "boarded":
            if self.state == WAIT_BOARDING:
                self.get_logger().info("탑승 완료, 목적지로 출발")
                self.start_leg(TO_DEST, self.job[1])
            else:
                self.get_logger().warn("지금은 탑승 대기 상태가 아닙니다")
        elif cmd == "pause":
            self.set_pause(True)
        elif cmd == "resume":
            self.set_pause(False)
        elif cmd == "estop":
            self.call(self.cli_estop, Trigger.Request(), self.on_estop_done)
        elif cmd == "estop_clear":
            self.call(self.cli_estop_clear, Trigger.Request(), self.on_estop_clear_done)
        elif cmd == "speed" and len(args) == 2:
            try:
                kmh = float(args[1])
            except ValueError:
                self.get_logger().warn(f"속도 값이 숫자가 아닙니다: {args[1]}")
                return
            if not 0.0 <= kmh <= SPEED_LIMIT_MAX_KMH:
                self.get_logger().warn(f"속도는 0~{SPEED_LIMIT_MAX_KMH:.0f} km/h 범위로 입력하세요")
                return
            self.set_speed_limit(kmh)
        elif cmd == "cancel":
            self.cancel_job()
        elif cmd == "status":
            spd = "알 수 없음" if self.speed is None else f"{self.speed * 3.6:.1f} km/h"
            self.get_logger().info(
                f"state={self.state} job={self.job} queue={list(self.queue)} "
                f"route={self.route_state} mode={self.mode} paused={self.paused} "
                f"estop={self.estopped} 속도={spd} 속도제한(내가 건 값)={self.speed_limit_kmh}")
        else:
            self.get_logger().warn(f"알 수 없는 명령: {msg.data}")

    # ---------- 상태머신 ----------
    def tick(self):
        self.check_stall()
        if self.state == IDLE:
            if self.queue:
                self.job = self.queue.popleft()
                self.get_logger().info(f"임무 시작: {self.job[0]} -> {self.job[1]}")
                self.start_leg(TO_PICKUP, self.job[0])
            return

        if self.state in (TO_PICKUP, TO_DEST):
            # 경로가 SET 이 되고, 운행 모드가 STOP 이면 자율주행 전환
            # 도착 직후에는 Autoware 가 스스로 STOP 으로 바꾸므로, 경로가 SET 상태일 때만 요청한다
            if (self.route_seen_set and self.route_state == RouteState.SET
                    and self.mode == OperationModeState.STOP
                    and not self.auto_requested
                    and time.monotonic() - self.last_auto_try > 1.0):
                self.auto_requested = True
                self.last_auto_try = time.monotonic()
                self.call(self.cli_auto, ChangeOperationMode.Request(), self.on_auto_done)

            # 도착 판정
            if self.route_seen_set and self.route_state == RouteState.ARRIVED:
                if self.state == TO_PICKUP:
                    self.get_logger().info(f"픽업지({self.job[0]}) 도착. 'boarded' 명령을 기다립니다.")
                    self.state = WAIT_BOARDING
                else:
                    self.get_logger().info(f"목적지({self.job[1]}) 도착. 임무 완료.")
                    self.job = None
                    self.state = IDLE

    def start_leg(self, new_state, stop_name):
        if new_state == TO_PICKUP and self.near_stop(stop_name):
            # 이미 픽업지에 있으면 같은 위치로 경로를 만들 수 없으므로 이동 없이 탑승 대기로 전환
            self.get_logger().info(
                f"이미 {stop_name} 에 있습니다. 이동 없이 탑승 대기로 전환 ('boarded' 를 기다립니다)")
            self.state = WAIT_BOARDING
            return
        self.state = new_state
        self.route_seen_set = False
        self.auto_requested = False
        self.pending_stop = stop_name
        # 이전 경로가 남아 있으면 set_route_points 가 'The route is already set.' 으로
        # 실패하므로, 먼저 기존 경로를 지운 뒤 새 경로를 설정한다.
        self.get_logger().info(f"기존 경로 정리 후 경로 설정 -> {stop_name}")
        self.call(self.cli_clear, ClearRoute.Request(), self.on_clear_done)

    def on_clear_done(self, fut):
        # 지우기가 실패해도(이미 비어 있는 경우 등) 무시하고 경로 설정을 시도한다
        res = fut.result()
        if res is not None and not res.status.success:
            self.get_logger().info(f"경로 지우기 응답: {res.status.message} (무시하고 진행)")
        s = STOPS[self.pending_stop]
        req = SetRoutePoints.Request()
        req.header.frame_id = "map"
        req.header.stamp = self.get_clock().now().to_msg()
        req.option.allow_goal_modification = False
        req.goal = make_pose(s["x"], s["y"], s["yaw"])
        self.get_logger().info(f"경로 설정 요청 -> {self.pending_stop}")
        self.call(self.cli_route, req, self.on_route_done)

    def cancel_job(self):
        self.get_logger().warn("임무 취소")
        self.queue.clear()
        self.job = None
        self.state = IDLE
        self.call(self.cli_clear, ClearRoute.Request(), None)
        self.call(self.cli_stop, ChangeOperationMode.Request(), None)

    def set_pause(self, value):
        req = SetPause.Request()
        req.pause = value
        self.paused = value
        self.get_logger().info("일시정지" if value else "재개")
        self.call(self.cli_pause, req, None)

    # ---------- 이벤트 대응 ----------
    def set_speed_limit(self, kmh):
        msg = VelocityLimit()
        msg.stamp = self.get_clock().now().to_msg()
        msg.max_velocity = float(kmh) / 3.6
        msg.use_constraints = False
        msg.sender = "shuttle_node"
        self.pub_limit.publish(msg)
        self.speed_limit_kmh = kmh
        self.get_logger().info(f"속도 제한 설정: {kmh:.1f} km/h ({msg.max_velocity:.2f} m/s)")

    def on_estop_done(self, fut):
        res = fut.result()
        if res is not None and res.success:
            self.estopped = True
            self.get_logger().warn("비상정지 작동. 해제하려면 'estop_clear' 를 보내세요.")
        else:
            msg = res.message if res else "응답 없음"
            self.get_logger().error(f"비상정지 요청 실패: {msg}")

    def on_estop_clear_done(self, fut):
        res = fut.result()
        if res is not None and res.success:
            self.estopped = False
            self.get_logger().info("비상정지 해제")
        else:
            msg = res.message if res else "응답 없음"
            self.get_logger().error(f"비상정지 해제 실패: {msg}")

    def check_stall(self):
        """자율주행 중인데 서 있는 상태(장애물, 보행로 정지 등)를 감지해 로그로 알린다."""
        if self.speed is None or self.state not in (TO_PICKUP, TO_DEST):
            self.stall_since = None
            self.stall_reported = False
            return
        driving_expected = (self.mode == OperationModeState.AUTONOMOUS
                            and self.route_state == RouteState.SET
                            and not self.paused and not self.estopped)
        moving = abs(self.speed) > 0.05
        now = time.monotonic()
        if moving or not driving_expected:
            if self.stall_reported and moving:
                self.get_logger().info("정지 해제: 주행을 재개했습니다.")
            self.stall_since = None
            self.stall_reported = False
            return
        if self.stall_since is None:
            self.stall_since = now
            return
        stalled = now - self.stall_since
        if stalled > STALL_SEC and (not self.stall_reported
                                    or now - self.stall_last_log > STALL_REPEAT_SEC):
            self.stall_reported = True
            self.stall_last_log = now
            self.get_logger().warn(
                f"주행 중이어야 하는데 {stalled:.0f}초째 정지 중 "
                "(장애물, 보행로 정지, 속도 제한 등을 확인하세요)")

    # ---------- 서비스 호출 헬퍼 ----------
    def call(self, client, req, done_cb):
        if not client.service_is_ready():
            self.get_logger().error(f"서비스 준비 안 됨: {client.srv_name}")
            return
        fut = client.call_async(req)
        if done_cb:
            fut.add_done_callback(done_cb)

    def on_route_done(self, fut):
        res = fut.result()
        if res is None or not res.status.success:
            msg = res.status.message if res else "응답 없음"
            self.get_logger().error(f"경로 설정 실패: {msg} -> 임무 중단")
            if "empty" in msg:
                self.get_logger().error(
                    "힌트: 현재 차량 위치에서 목표까지 이어지는 경로가 없습니다 "
                    "(일방통행으로 되돌아갈 수 없거나, 목표가 차로 위가 아닐 수 있음)")
            self.job = None
            self.state = IDLE

    def on_auto_done(self, fut):
        res = fut.result()
        if res is None or not res.status.success:
            msg = res.status.message if res else "응답 없음"
            if "in progress" in msg:
                self.get_logger().info("모드 전환이 이미 진행 중입니다 (정상, 잠시 후 상태 확인)")
            else:
                self.get_logger().warn(f"자율주행 전환 실패: {msg} (재시도)")
            self.auto_requested = False
        else:
            self.get_logger().info("자율주행 시작")


def main():
    rclpy.init()
    node = ShuttleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
