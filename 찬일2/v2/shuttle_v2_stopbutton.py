#!/usr/bin/env python3
"""셔틀 v2 + 정차 버튼

호출(출발지 도착지) -> 출발지로 이동 -> 정차 10초(탑승) -> 도착지로 이동 -> 정차 10초(하차) -> 다음 호출

정차 버튼 (버스 하차벨과 비슷한 동작)
- 주행 중에 누르면: 가는 길에 있는 다음 정류장에 정차했다가(10초) 원래 목적지로 계속 간다.
    예) 1 -> 3 이동 중 2 이전에 누르면 2에서 정차, 2를 지난 뒤에 누르면 3(원래 정차 예정)
        1 -> 4 이동 중 2 이전이면 2, 2와 3 사이면 3, 3 이후면 4(원래 정차 예정)
    정류장이 너무 가까워 설 수 없으면(MIN_STOP_DIST_M) 그다음 정류장에 정차한다.
- 정차 중에 누르면: 남은 정차 시간이 다시 10초로 늘어난다.
- 누르고 있는 동안(hold ~ release)은 출발하지 않고, 떼면 10초 뒤에 출발한다.
- 아무도 버튼을 누르지 않으면 정차 10초 뒤 알아서 다음 호출로 출발한다.

가정: 정류장은 LANE_ORDER(1->2->3->4->5) 순서로 길 위에 놓여 있고, 반대 방향도 같은 정류장을
역순으로 지나서 갈 수 있다. 이동 중 위치는 localization pose 토픽으로 받는다.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from autoware_adapi_v1_msgs.srv import ChangeOperationMode
from autoware_adapi_v1_msgs.msg import RouteState
import math
import queue
import time
import datetime
import threading
from collections import deque


LOCATIONS = {
    "1": {"name": "정문", "x": 3768.640380859375, "y": 73697.9296875,
          "qz": 0.21878969872785992, "qw": 0.9757720367640037},
    "2": {"name": "기타", "x": 3770.7509765625,   "y": 73730.203125,
          "qz": 0.8567721406194471,  "qw": 0.5156951609801769},
    "3": {"name": "남문", "x": 3752.751220703125, "y": 73736.265625,
          "qz": -0.9729766704593605, "qw": 0.23090344029879056},
    "4": {"name": "중앙", "x": 3718.184326171875, "y": 73718.1640625,
          "qz": -0.9738443370537525, "qw": 0.22721621242406376},
    "5": {"name": "동문", "x": 3716.48681640625,  "y": 73695.390625,
          "qz": -0.5392124002151566, "qw": 0.8421698091562115},
}

LANE_ORDER = ["1", "2", "3", "4", "5"]   # 정류장이 길 위에 놓인 순서
DWELL_SEC = 10.0              # 정차 시간 (버튼을 누르면 이만큼 다시 늘어남)
PASS_RADIUS_M = 6.0           # 정류장에서 이 거리 안을 지나가면 "지나쳤다"고 본다
MIN_STOP_DIST_M = 10.0        # 이보다 가까운 정류장은 정차 요청에 못 맞춰서 건너뜀
ROUTE_SET_TIMEOUT_SEC = 8.0   # 목표를 보낸 뒤 경로가 SET 되기를 기다리는 한계
ENGAGE_RETRIES = 3            # 자율주행 전환 시도 횟수
ENGAGE_RETRY_SEC = 2.0        # 전환 재시도 간격
PROMPT = ('\n입력 (호출: "1 5" / 정차 버튼: b, 누르기 hold, 떼기 release / '
          '현황: status / 재시도: retry): ')

# 상태
#   IDLE      대기 (정차 중, 호출 없음)
#   STARTING  목표 전송과 자율주행 전환 중
#   MOVING    이동 중
#   DWELL     정류장에 정차 중 (탑승 / 하차 / 정차 요청)
#   HALTED    목표 전송이나 자율주행 전환에 실패해 멈춘 상태 -> retry 입력


def ro(name):
    """'기타'+'로', '정문'+'으로'처럼 받침에 맞는 조사를 붙인다."""
    last = name[-1]
    if '가' <= last <= '힣':
        jong = (ord(last) - 0xAC00) % 28
        return name + ('로' if jong in (0, 8) else '으로')
    return name + '(으)로'


def stops_between(a, b):
    """a에서 b로 갈 때 중간에 지나는 정류장을 지나는 순서대로 돌려준다 (a, b 제외)."""
    if a not in LANE_ORDER or b not in LANE_ORDER:
        return []
    ia, ib = LANE_ORDER.index(a), LANE_ORDER.index(b)
    if ia < ib:
        return LANE_ORDER[ia + 1:ib]
    return LANE_ORDER[ib + 1:ia][::-1]


class ShuttleNode(Node):
    def __init__(self):
        super().__init__('shuttle_node')

        self.goal_pub = self.create_publisher(
            PoseStamped, '/planning/mission_planning/goal', 10
        )
        self.autonomous_client = self.create_client(
            ChangeOperationMode, '/api/operation_mode/change_to_autonomous'
        )
        self.route_state_sub = self.create_subscription(
            RouteState, '/api/routing/state', self.route_state_callback, 10
        )
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization/pose_twist_fusion_filter/pose_with_covariance',
            self.pose_callback, 10
        )

        self.lock = threading.Lock()       # 상태 전환만 짧게 보호 (오래 걸리는 작업은 락 밖에서)
        self.state = "IDLE"
        self.current_route_state = None
        self.route_changes = 0             # 경로 상태가 바뀐 횟수 (오래된 ARRIVED를 새 도착으로 오인하지 않기 위함)
        self.current_pose = None

        # 호출
        self.request_queue = deque()       # (출발지, 도착지) 튜플 큐
        self.pending_destination = None    # 출발지에서 태운 승객의 도착지

        # 현재 구간
        self.cur_key = None                # 마지막으로 정차한 정류장
        self.leg_final = None              # 이번 구간의 최종 목표
        self.leg_role = None               # "pickup" / "dest"
        self.goal_key = None               # 지금 차량에 내려 둔 목표 (정차 요청 시 중간 정류장)
        self.leg_stops = []                # 이번 구간에서 중간에 지나는 정류장 (순서대로)
        self.passed = set()                # 이미 지나친 정류장
        self.via_active = False            # 중간 정류장 정차를 위해 이동 중인지
        self.in_leg = False                # 구간 이동 중인지 (지나친 정류장 추적용)

        # 정차
        self.dwell_key = None
        self.dwell_role = None             # "pickup" / "dest" / "via"
        self.dwell_until = 0.0

        # 버튼
        self.button_held = False
        self.stop_requested = False

        self.retry_job = None
        self.log = []

        # 목표 전송/자율주행 전환처럼 오래 걸리는 작업은 별도 스레드에서 순서대로 처리
        self.jobs = queue.Queue()
        threading.Thread(target=self.worker_loop, daemon=True).start()

        self.get_logger().info('셔틀 대기 중...')

    # ---------- 콜백 / 유틸 ----------
    def route_state_callback(self, msg):
        if msg.state != self.current_route_state:
            self.route_changes += 1
        self.current_route_state = msg.state

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose.position

    def pose_xy(self):
        if self.current_pose is None:
            return None
        return (self.current_pose.x, self.current_pose.y)

    @staticmethod
    def dist_xy(xy, key):
        loc = LOCATIONS[key]
        return math.hypot(loc["x"] - xy[0], loc["y"] - xy[1])

    def name(self, key):
        return LOCATIONS[key]["name"]

    def log_event(self, event, key):
        self.log.append({
            'time': datetime.datetime.now().isoformat(),
            'event': event,
            'destination': LOCATIONS[key]['name']
        })

    def worker_loop(self):
        while True:
            job = self.jobs.get()
            try:
                job()
            except Exception as e:      # 작업 하나가 실패해도 스레드는 유지
                self.get_logger().error(f'작업 처리 중 오류: {e}')
                with self.lock:
                    self.state = "HALTED"

    # ---------- 호출 ----------
    def call_shuttle(self, origin_key, dest_key):
        if origin_key not in LOCATIONS or dest_key not in LOCATIONS:
            self.get_logger().error('알 수 없는 지점 번호입니다.')
            return

        if origin_key == dest_key:
            self.get_logger().error('출발지와 목적지가 같습니다.')
            return

        with self.lock:
            idle = self.state == "IDLE"
            if idle:
                self.state = "STARTING"
            else:
                self.request_queue.append((origin_key, dest_key))
        if not idle:
            self.get_logger().info(
                f'{self.name(origin_key)} -> {self.name(dest_key)} '
                f'요청을 대기열에 추가 (대기 {len(self.request_queue)}건)')
            return

        self.begin_trip(origin_key, dest_key)

    def begin_trip(self, origin_key, dest_key):
        """state는 이미 STARTING 이다. 출발지로 이동하거나, 이미 출발지에 있으면 바로 탑승 정차."""
        self.pending_destination = dest_key
        if origin_key == self.cur_key:
            self.get_logger().info(f'{self.name(origin_key)}에 이미 있습니다. 바로 탑승 정차합니다.')
            self.enter_dwell(origin_key, "pickup")
        else:
            self.log_event('depart_to_pickup', origin_key)
            self.jobs.put(lambda: self.job_leg(origin_key, "pickup"))

    # ---------- 정차 버튼 ----------
    def button_press(self):
        with self.lock:
            self.button_held = True
            st = self.state
            if st == "DWELL":
                self.dwell_until = time.time() + DWELL_SEC
            elif st in ("MOVING", "STARTING"):
                if self.via_active:
                    st = "VIA"
                else:
                    self.stop_requested = True
        if st == "DWELL":
            self.get_logger().info(f'정차 버튼: 정차 시간을 {DWELL_SEC:.0f}초로 연장합니다.')
        elif st == "VIA":
            self.get_logger().info('정차 버튼: 이미 다음 정류장에 정차하러 가는 중입니다.')
        elif st in ("MOVING", "STARTING"):
            self.get_logger().info('정차 버튼: 다음 정류장에 정차를 요청했습니다.')
        else:
            self.get_logger().info('정차 버튼: 지금은 주행 중이 아니라서 아무 일도 하지 않습니다.')

    def button_release(self):
        with self.lock:
            self.button_held = False

    def button_tap(self):
        self.button_press()
        self.button_release()

    # ---------- 1초가 아니라 0.5초마다 도는 메인 루프 ----------
    def tick(self):
        now = time.time()
        with self.lock:
            state = self.state
        if self.in_leg and state in ("MOVING", "STARTING"):
            self.track_passes()

        if state == "MOVING":
            if self.current_route_state == RouteState.ARRIVED:
                self.on_arrival()
            elif self.stop_requested and not self.via_active:
                self.plan_stop()
        elif state == "DWELL":
            if self.button_held:
                self.dwell_until = now + DWELL_SEC     # 누르고 있는 동안은 정차 시간이 줄지 않음
            if now >= self.dwell_until:
                self.finish_dwell()

    def track_passes(self):
        """차량이 정류장 옆을 지나갔는지 순서대로 확인한다."""
        xy = self.pose_xy()
        if xy is None:
            return
        for k in self.leg_stops:
            if k in self.passed:
                continue
            if self.dist_xy(xy, k) <= PASS_RADIUS_M:
                self.passed.add(k)
            break       # 순서대로 첫 번째 미통과 정류장만 확인

    def plan_stop(self):
        """정차 요청: 아직 지나치지 않았고 설 수 있을 만큼 먼 첫 번째 중간 정류장에 정차."""
        xy = self.pose_xy()
        if xy is None:
            self.stop_requested = False
            self.get_logger().warn('위치 정보를 받지 못해 정차 요청을 처리할 수 없습니다.')
            return

        for k in self.leg_stops:
            if k in self.passed:
                continue
            if self.dist_xy(xy, k) < MIN_STOP_DIST_M:
                continue        # 너무 가까워서 못 섬 -> 그다음 정류장
            with self.lock:
                if self.state != "MOVING":
                    return
                self.state = "STARTING"
                self.stop_requested = False
            self.get_logger().info(f'정차 요청: {self.name(k)}에 정차합니다.')
            self.jobs.put(lambda k=k: self.job_via(k))
            return

        self.stop_requested = False
        self.get_logger().info(
            f'중간에 정차할 수 있는 정류장이 없어서, 다음 정류장 {self.name(self.leg_final)}'
            f'(원래 정차 예정)에서 정차합니다.')

    # ---------- 정차 ----------
    def on_arrival(self):
        with self.lock:
            if self.state != "MOVING":
                return
            key = self.goal_key
            role = "via" if self.via_active else self.leg_role
        self.enter_dwell(key, role)

    def enter_dwell(self, key, role):
        now = time.time()
        with self.lock:
            self.state = "DWELL"
            self.dwell_key = key
            self.dwell_role = role
            self.dwell_until = now + DWELL_SEC
            self.cur_key = key
            self.in_leg = False
            self.via_active = False
            self.stop_requested = False if role == "via" else self.stop_requested
            if role == "via" and key in self.leg_stops:
                # 이 정류장까지는 지나친 것으로 처리
                for k in self.leg_stops[:self.leg_stops.index(key) + 1]:
                    self.passed.add(k)
        label = {"pickup": "탑승", "dest": "하차", "via": "정차 요청"}[role]
        self.log_event({"pickup": "arrive_pickup", "dest": "arrive", "via": "arrive_via"}[role], key)
        self.get_logger().info(
            f'{self.name(key)} 도착, 정차 ({label}). '
            f'{DWELL_SEC:.0f}초 뒤 출발합니다 (정차 버튼을 누르고 있으면 출발하지 않음)')

    def finish_dwell(self):
        with self.lock:
            if self.state != "DWELL":
                return
            role = self.dwell_role
            self.state = "STARTING"

        if role == "pickup":
            dest = self.pending_destination
            self.log_event('pickup_done_depart', dest)
            self.jobs.put(lambda: self.job_leg(dest, "dest"))
        elif role == "via":
            self.jobs.put(self.job_resume)
        else:
            with self.lock:
                nxt = self.request_queue.popleft() if self.request_queue else None
                if nxt is None:
                    self.state = "IDLE"
                    self.pending_destination = None
            if nxt is None:
                self.get_logger().info('대기 중인 호출이 없습니다.')
            else:
                self.get_logger().info(
                    f'다음 호출 {self.name(nxt[0])} -> {self.name(nxt[1])}(으)로 출발합니다.')
                self.begin_trip(nxt[0], nxt[1])

    # ---------- 이동 작업 (작업 스레드에서 실행) ----------
    def job_leg(self, final_key, role):
        """현재 위치에서 final_key까지 한 구간을 시작"""
        with self.lock:
            self.state = "STARTING"
        if self.depart(final_key):
            with self.lock:
                self.leg_final = final_key
                self.leg_role = role
                self.goal_key = final_key
                self.leg_stops = stops_between(self.cur_key, final_key)
                self.passed = set()
                self.via_active = False
                self.in_leg = True
                self.state = "MOVING"
        else:
            self.halt(lambda: self.job_leg(final_key, role))

    def job_via(self, stop_key):
        """이동 중에 목표를 중간 정류장으로 바꿈"""
        if self.depart(stop_key, midleg=True):
            with self.lock:
                self.goal_key = stop_key
                self.via_active = True
                self.state = "MOVING"
            return

        self.get_logger().error('정차 요청에 실패했습니다. 원래 목적지로 계속 갑니다.')
        if self.depart(self.leg_final, midleg=True):
            with self.lock:
                self.goal_key = self.leg_final
                self.via_active = False
                self.state = "MOVING"
        else:
            self.halt(self.job_resume)

    def job_resume(self):
        """중간 정류장 정차가 끝난 뒤 원래 목적지로 다시 출발"""
        with self.lock:
            self.state = "STARTING"
        self.log_event('resume', self.leg_final)
        if self.depart(self.leg_final):
            with self.lock:
                self.goal_key = self.leg_final
                self.via_active = False
                self.in_leg = True
                self.state = "MOVING"
        else:
            self.halt(self.job_resume)

    def halt(self, retry_job):
        with self.lock:
            self.state = "HALTED"
            self.retry_job = retry_job
        self.get_logger().error(
            '출발하지 못해 멈췄습니다. Autoware 상태를 확인한 뒤 "retry"를 입력하세요.')

    def retry(self):
        with self.lock:
            if self.state != "HALTED" or self.retry_job is None:
                ok = False
            else:
                ok = True
                job = self.retry_job
                self.retry_job = None
                self.state = "STARTING"
        if not ok:
            self.get_logger().info('다시 시도할 작업이 없습니다.')
            return
        self.jobs.put(job)

    # ---------- 목표 전송 / 자율주행 전환 ----------
    def depart(self, key, midleg=False):
        """목표를 보내고 경로가 SET 되면 자율주행으로 전환. 성공하면 True

        midleg=True 는 이미 이동 중(경로가 SET 상태)에 목표만 바꾸는 경우라서
        경로 상태가 바뀌지 않아도 이어서 진행한다.
        """
        loc = LOCATIONS[key]
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = loc["x"]
        msg.pose.position.y = loc["y"]
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = loc["qz"]
        msg.pose.orientation.w = loc["qw"]

        self.get_logger().info(f'목적지 전송: {loc["name"]}')
        mark = self.route_changes - (1 if midleg else 0)
        self.goal_pub.publish(msg)

        if not self.wait_route_set(mark):
            self.get_logger().error(
                f'{loc["name"]} 경로가 설정되지 않았습니다. 지점이 차선 위인지, 시뮬레이터 상태를 확인하세요.')
            return False
        return self.switch_to_autonomous()

    def wait_route_set(self, mark):
        """목표를 보낸 뒤 경로 상태가 실제로 바뀌고 SET(또는 곧바로 도착이면 ARRIVED)이 될 때까지 기다린다."""
        t0 = time.time()
        while time.time() - t0 < ROUTE_SET_TIMEOUT_SEC:
            if (self.route_changes > mark
                    and self.current_route_state in (RouteState.SET, RouteState.ARRIVED)):
                return True
            time.sleep(0.1)
        return False

    def switch_to_autonomous(self):
        if not self.autonomous_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('operation_mode 서비스를 찾을 수 없습니다.')
            return False

        for attempt in range(1, ENGAGE_RETRIES + 1):
            future = self.autonomous_client.call_async(ChangeOperationMode.Request())
            t0 = time.time()
            while not future.done() and time.time() - t0 < 5.0:
                time.sleep(0.05)
            result = future.result() if future.done() else None

            if result is not None and result.status.success:
                self.get_logger().info('자율주행 모드 전환 성공, 이동 시작')
                return True

            msg = result.status.message if result else '응답 없음'
            self.get_logger().warn(
                f'자율주행 모드 전환 실패 ({attempt}/{ENGAGE_RETRIES}): {msg}')
            if attempt < ENGAGE_RETRIES:
                time.sleep(ENGAGE_RETRY_SEC)
        return False

    # ---------- 현황 ----------
    def print_status(self):
        with self.lock:
            state = self.state
            queue_txt = ', '.join(f'{self.name(o)}->{self.name(d)}' for o, d in self.request_queue) or '없음'
        cur = self.name(self.cur_key) if self.cur_key else '-'
        goal = self.name(self.goal_key) if self.goal_key else '-'
        extra = ''
        if state == "DWELL":
            extra = f', 남은 정차 {max(0.0, self.dwell_until - time.time()):.0f}초'
        self.get_logger().info(
            f'[현황] 상태={state} 마지막 정차={cur} 현재 목표={goal}{extra}\n'
            f'  버튼: {"누르는 중" if self.button_held else "뗌"}, '
            f'정차 요청: {"있음" if self.stop_requested else "없음"}\n'
            f'  대기 호출: {queue_txt}')


def main(args=None):
    rclpy.init(args=args)
    node = ShuttleNode()
    node.create_timer(0.5, node.tick)

    def input_loop():
        while rclpy.ok():
            print(PROMPT, end="", flush=True)
            raw = input().strip().lower()

            if raw in ("b", "정차"):
                node.button_tap()
            elif raw == "hold":
                node.button_press()
            elif raw == "release":
                node.button_release()
            elif raw == "retry":
                node.retry()
            elif raw == "status":
                node.print_status()
            else:
                parts = raw.split()
                if len(parts) != 2:
                    print('형식이 맞지 않습니다. 예: 1 5')
                    continue
                node.call_shuttle(parts[0], parts[1])

    threading.Thread(target=input_loop, daemon=True).start()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
