#!/usr/bin/env python3
"""셔틀 v2 loop6: 순환 노선 + 정류장 호출 버튼 + 차 안 도착지 버튼 (엘리베이터 방식)

지도: OpenDriveMap_v33d_loop (순환 노선 v3.3d), 정류장 좌표는 info/circulation_v33d_r1.yaml

운행 방식
- 셔틀은 1 정문 -> 2 양성재 -> 3 후문 -> 4 도서관남문 -> 5 도서관북문 -> 1 ... 한 방향으로만 돈다.
- 버튼
    정류장 호출 버튼 (입력 1~5)   : 정류장에서 기다리는 사람이 누름 -> 그 정류장에 선다
    차 안 도착지 버튼 (입력 d 1~5): 탄 사람이 내릴 정류장을 누름 -> 그 정류장에 선다
    차 안 하차벨 (입력 b)         : 가는 길의 바로 다음 정류장에 선다 (도착지 버튼을 깜빡했을 때)
- 눌린 정류장(호출 + 도착지)에만 서고, 나머지는 경유점으로 그냥 지나간다.
  도착지 버튼이 남아 있다 = 아직 탄 사람이 있다 -> 센서 없이도 내릴 사람이 다 내릴 때까지 계속 간다.
- 주행 중에 가는 길 앞쪽 정류장이 새로 눌리면 멈추지 않고 그 정류장으로 목표를 앞당긴다.
  방금 지나친 정류장이면 다음 바퀴에 들른다.
- 눌린 버튼이 하나도 없으면 그 자리에서 대기 (IDLE_HOME 을 정하면 그 정류장으로 가서 대기).

안전장치: 호출 정류장에서 문을 열었는데 문이 닫힐 때까지 도착지 버튼이 하나도 안 눌리면
  "탄 사람이 도착지를 안 눌렀을 수 있다"고 보고, 그 정류장으로 돌아올 때까지 한 바퀴는 돈다.
  그 사이에 도착지 버튼이 눌리면 한 바퀴 규칙은 풀린다.

문 (엘리베이터처럼)
- 정류장에 서면 문이 DWELL_SEC 동안 열린다. 그 정류장 호출 버튼이나 b 를 누르면 다시 연장.
- hold 하는 동안은 문이 계속 열려 있고, release 하면 DWELL_SEC 뒤 출발.

RViz 표시 (/shuttle/markers, MarkerArray): Add -> By topic -> /shuttle/markers -> MarkerArray
    초록 = 지금 정차 중(문 열림)   파랑 = 다음 정차   노랑 = 눌린 정류장 (WAITING = 호출, GET OFF = 내릴 사람)
    어두운 회색 = 이번 구간에서 이미 지남   회색 = 요청 없음
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from autoware_adapi_v1_msgs.srv import ChangeOperationMode, ClearRoute, SetRoutePoints
from autoware_adapi_v1_msgs.msg import RouteState, OperationModeState
import math
import queue
import time
import datetime
import threading


def yaw_to_q(yaw_deg):
    h = math.radians(yaw_deg) / 2.0
    return math.sin(h), math.cos(h)


# goal = 정차 자리, via = 지나갈 때 경유점(차선 가운데), en = RViz 표시용 이름
LOCATIONS = {
    "1": {"name": "정문", "en": "MAIN GATE",
          "goal": (379.257, 886.011, 0.951546, 0.307506), "via": (378.166, 886.798, 144.18)},
    "2": {"name": "양성재", "en": "YANGSEONG",
          "goal": (213.999, 550.838, -0.54348, 0.839422), "via": (214.552, 549.608, -65.75)},
    "3": {"name": "후문", "en": "BACK GATE",
          "goal": (959.052, 289.852, -0.318379, 0.947963), "via": (960.124, 289.041, -37.13)},
    "4": {"name": "도서관남문", "en": "LIB SOUTH",
          "goal": (572.721, 566.217, 0.972186, 0.234212), "via": (571.524, 566.829, 152.91)},
    "5": {"name": "도서관북문", "en": "LIB NORTH",
          "goal": (656.162, 684.456, -0.304989, 0.952356), "via": (657.257, 683.675, -35.52)},
}
LANE_ORDER = ["1", "2", "3", "4", "5"]   # 순환 순서 (끝 다음은 다시 처음)
# 다음 정류장까지 차선 거리(m). 맵 차선 연결로 계산한 값, 도착 예상 시간 안내용
LEG_M = {"1": 426, "2": 943, "3": 605, "4": 492, "5": 539}

INITIAL_KEY = "1"             # 시작할 때 차가 서 있는 정류장 (초기 위치를 정문 안쪽에 둠)
IDLE_HOME = None              # 할 일이 없을 때 돌아가 기다릴 정류장 (예: "1"). None 이면 그 자리에서 대기
DWELL_SEC = 3.0               # 정차 시간 (테스트용 3초)
EST_SPEED = 4.2               # m/s, 도착 예상 시간 계산용 평균 속도 (약 15 km/h)
PASS_RADIUS_M = 6.0           # 정류장 경유점에서 이 거리 안을 지나가면 "지나쳤다"고 본다
MIN_STOP_DIST_M = 10.0        # 이보다 가까운 정류장은 새로 세우기 어려워서 건너뜀 (다음 바퀴에 처리)
ROUTE_SET_TIMEOUT_SEC = 8.0
ENGAGE_RETRIES = 5
ENGAGE_RETRY_SEC = 2.0
STATIONARY_WAIT_SEC = 5.0
STATIONARY_SPEED = 0.05
PROMPT = ('\n입력 (정류장 호출: 1~5 / 차 안 도착지: d 1~5 / 하차벨: b / 문 잡기: hold, release / '
          '현황: status / 재시도: retry): ')

# 셔틀 상태
#   IDLE      할 일 없음, 정류장에서 대기
#   STARTING  경로 설정과 자율주행 전환 중
#   MOVING    이동 중
#   DWELL     정류장에 정차 중 (문 열림)
#   HALTED    경로 설정이나 전환 실패 -> retry


def stops_between(a, b):
    """a에서 b로 갈 때 중간에 지나는 정류장 (a, b 제외). 순환이라 끝을 넘어 돈다."""
    if a not in LANE_ORDER or b not in LANE_ORDER or a == b:
        return []
    ia, ib = LANE_ORDER.index(a), LANE_ORDER.index(b)
    if ia < ib:
        return LANE_ORDER[ia + 1:ib]
    return LANE_ORDER[ia + 1:] + LANE_ORDER[:ib]


def loop_after(a):
    """a 다음 정류장부터 한 바퀴 돌아 a 까지 (a 를 마지막에 포함)"""
    i = LANE_ORDER.index(a)
    return LANE_ORDER[i + 1:] + LANE_ORDER[:i + 1]


def loop_dist(a, b):
    """a 에서 b 까지 순환 방향 차선 거리(m)"""
    d, k = 0, a
    while k != b:
        d += LEG_M[k]
        k = LANE_ORDER[(LANE_ORDER.index(k) + 1) % len(LANE_ORDER)]
    return d


def goal_pose(key):
    x, y, qz, qw = LOCATIONS[key]["goal"]
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, 0.0
    p.orientation.z, p.orientation.w = qz, qw
    return p


def via_pose(key):
    x, y, yaw = LOCATIONS[key]["via"]
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, 0.0
    p.orientation.z, p.orientation.w = yaw_to_q(yaw)
    return p


class ShuttleNode(Node):
    def __init__(self):
        super().__init__('shuttle_node')

        self.cli_set = self.create_client(SetRoutePoints, '/api/routing/set_route_points')
        self.cli_change = self.create_client(SetRoutePoints, '/api/routing/change_route_points')
        self.cli_clear = self.create_client(ClearRoute, '/api/routing/clear_route')
        self.cli_auto = self.create_client(ChangeOperationMode, '/api/operation_mode/change_to_autonomous')
        self.cli_stop = self.create_client(ChangeOperationMode, '/api/operation_mode/change_to_stop')

        # AD API 상태 토픽은 마지막 값을 남겨 두는(transient_local) 방식이라 같은 QoS 로 받는다
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(RouteState, '/api/routing/state', self.route_state_callback, latched)
        self.create_subscription(OperationModeState, '/api/operation_mode/state', self.op_mode_callback, latched)
        self.create_subscription(Odometry, '/localization/kinematic_state', self.pose_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/shuttle/markers', 10)

        self.lock = threading.RLock()
        self.state = "IDLE"
        self.current_route_state = None
        self.op_mode = None
        self.current_pose = None
        self.speed = 0.0

        # 버튼이 눌린 정류장: {정류장: {"station": 호출, "dest": 도착지 버튼, "inside": 하차벨, "t": 처음 눌린 시각}}
        self.requests = {}
        self.run_until = None              # 안전장치: 여기로 돌아올 때까지는 버튼이 없어도 계속 돈다
        self.boarded_at = None             # 이번 정차에서 호출 승객을 태운 정류장 (도착지 버튼 확인용)
        self.dest_since_board = False      # 태운 뒤 도착지 버튼이 눌렸는지

        # 현재 구간
        self.cur_key = INITIAL_KEY         # 마지막으로 선(또는 출발한) 정류장
        self.goal_key = None               # 지금 차에 내려 둔 목표
        self.leg_stops = []                # 이번 구간에서 지나는 중간 정류장 (순서대로)
        self.passed = set()
        self.in_leg = False
        self.changing = False              # 경로 변경 작업 중
        self.change_failed = set()         # 이번 구간에서 경로 변경에 실패한 정류장 (반복 시도 방지)

        # 정차
        self.dwell_key = None
        self.dwell_until = 0.0
        self.dwell_info = ""

        self.button_held = False
        self.retry_job = None
        self.log = []

        self.jobs = queue.Queue()
        threading.Thread(target=self.worker_loop, daemon=True).start()
        self.get_logger().info('셔틀 대기 중... RViz 에서 /shuttle/markers (MarkerArray) 를 추가하세요.')

    # ---------- 콜백 / 유틸 ----------
    def route_state_callback(self, msg):
        self.current_route_state = msg.state

    def op_mode_callback(self, msg):
        self.op_mode = msg.mode

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose.position
        self.speed = abs(msg.twist.twist.linear.x)

    def pose_xy(self):
        if self.current_pose is None:
            return None
        return (self.current_pose.x, self.current_pose.y)

    @staticmethod
    def dist_via(xy, key):
        x, y, _ = LOCATIONS[key]["via"]
        return math.hypot(x - xy[0], y - xy[1])

    def name(self, key):
        return LOCATIONS[key]["name"]

    def log_event(self, event, key, **extra):
        self.log.append({'time': datetime.datetime.now().isoformat(), 'event': event,
                         'stop': LOCATIONS[key]['name'], **extra})

    def worker_loop(self):
        while True:
            job = self.jobs.get()
            try:
                job()
            except Exception as e:
                self.get_logger().error(f'작업 처리 중 오류: {e}')
                with self.lock:
                    self.state = "HALTED"

    # ---------- 할 일 계산 ----------
    def has_work(self, k):
        return k in self.requests

    def next_target(self, frm):
        """frm 다음부터 순환 방향으로 처음 버튼이 눌린 정류장. 없으면 None"""
        for k in loop_after(frm):
            if self.has_work(k):
                return k
        return None

    def add_request(self, k, kind):
        r = self.requests.setdefault(k, {"station": False, "dest": False, "inside": False, "t": time.time()})
        r[kind] = True

    # ---------- 정류장 버튼 (기다리는 사람이 셔틀을 부름) ----------
    def station_button(self, k, held=False):
        if k not in LOCATIONS:
            self.get_logger().error('알 수 없는 정류장 번호입니다. (1~5)')
            return
        with self.lock:
            state = self.state
            if held:
                self.button_held = True
            if state == "DWELL" and self.dwell_key == k:
                self.dwell_until = time.time() + DWELL_SEC      # 문 열려 있음 -> 문 잡아 주기
                self.mark_boarding(k)
                here = "dwell"
            elif state == "IDLE" and self.cur_key == k:
                self.state = "STARTING"
                self.mark_boarding(k)
                here = "idle"
            else:
                self.add_request(k, "station")
                here = None
            eta_from = self.cur_key or LANE_ORDER[0]

        if here == "dwell":
            self.get_logger().info(f'{self.name(k)} 정류장 버튼: 문이 열려 있어 {DWELL_SEC:.0f}초 더 기다립니다.')
            return
        if here == "idle":
            self.get_logger().info(f'{self.name(k)} 정류장 버튼: 셔틀이 여기 있어 바로 문을 엽니다.')
            self.arrive_at(k)
            return

        if k != eta_from:
            eta_m = loop_dist(eta_from, k)
        else:
            eta_m = sum(LEG_M.values())      # 방금 떠난 정류장 -> 한 바퀴 돌아와야 함
        self.get_logger().info(
            f'{self.name(k)} 정류장 버튼 눌림 (셔틀 도착 예상 약 {eta_m / EST_SPEED / 60:.0f}분 이내)')
        self.log_event('station_button', k)

        if state == "IDLE":
            with self.lock:
                self.state = "STARTING"
            self.jobs.put(self.job_next_leg)
        # MOVING 이면 tick 에서 가는 길에 있는지 보고 목표를 앞당긴다

    def mark_boarding(self, k):
        """호출 정류장에서 문을 엶 -> 누가 탈 수 있음. 문 닫힐 때 도착지 버튼이 눌렸는지 본다 (lock 안)"""
        self.boarded_at = k
        self.dest_since_board = False

    # ---------- 차 안 도착지 버튼 ----------
    def dest_button(self, k):
        if k not in LOCATIONS:
            self.get_logger().error('알 수 없는 정류장 번호입니다. (d 1~5)')
            return
        with self.lock:
            state = self.state
            if (state == "DWELL" and self.dwell_key == k) or (state == "IDLE" and self.cur_key == k):
                here = True
            else:
                here = False
                self.add_request(k, "dest")
                self.dest_since_board = True
                released = self.run_until is not None
                self.run_until = None          # 도착지를 알았으니 한 바퀴 안전장치 해제
                if state == "IDLE":
                    self.state = "STARTING"
            frm = self.cur_key
        if here:
            self.get_logger().info(f'도착지 버튼: 지금 {self.name(k)}에 서 있습니다. 여기서 내리세요.')
            return
        just_left = state in ("MOVING", "STARTING") and k == frm
        self.get_logger().info(
            f'도착지 버튼: {self.name(k)}' + (' (방금 떠난 정류장이라 한 바퀴 돌아서 갑니다)' if just_left else '')
            + (' / 한 바퀴 안전장치 해제' if released else ''))
        self.log_event('dest_button', k)
        if state == "IDLE":
            self.jobs.put(self.job_next_leg)
        # MOVING 이면 tick 에서 가는 길에 있는지 보고 목표를 앞당긴다

    # ---------- 차 안 하차벨 ----------
    def button_press(self):
        with self.lock:
            self.button_held = True
            st = self.state
            target = None
            if st == "DWELL":
                self.dwell_until = time.time() + DWELL_SEC
            elif st in ("MOVING", "STARTING"):
                target = self.next_stop_ahead()
                if target is not None:
                    self.add_request(target, "inside")
        if st == "DWELL":
            self.get_logger().info(f'하차벨: 문을 {DWELL_SEC:.0f}초 더 열어 둡니다.')
        elif st in ("MOVING", "STARTING"):
            if target is None:
                self.get_logger().info('하차벨: 설 수 있는 다음 정류장이 없습니다.')
            else:
                self.get_logger().info(f'하차벨: 다음 정류장 {self.name(target)}에 정차합니다.')
                self.log_event('inside_button', target)
        elif st == "IDLE" and self.cur_key:
            nxt = loop_after(self.cur_key)[0]
            with self.lock:
                self.add_request(nxt, "inside")
                self.state = "STARTING"
            self.get_logger().info(f'하차벨: 대기 중이라 다음 정류장 {self.name(nxt)}까지 갑니다.')
            self.jobs.put(self.job_next_leg)
        else:
            self.get_logger().info('하차벨: 지금은 처리할 수 없습니다.')

    def button_release(self):
        with self.lock:
            self.button_held = False

    def button_tap(self):
        self.button_press()
        self.button_release()

    def next_stop_ahead(self):
        """가는 길에서 아직 안 지났고 설 수 있을 만큼 먼 첫 정류장 (없으면 목표)"""
        xy = self.pose_xy()
        for k in self.leg_stops:
            if k in self.passed:
                continue
            if xy is not None and self.dist_via(xy, k) < MIN_STOP_DIST_M:
                continue
            return k
        return self.goal_key

    # ---------- 0.5초마다 도는 메인 루프 ----------
    def tick(self):
        now = time.time()
        with self.lock:
            state = self.state
        if self.in_leg and state in ("MOVING", "STARTING"):
            self.track_passes()

        if state == "MOVING":
            if self.current_route_state == RouteState.ARRIVED and not self.changing:
                with self.lock:
                    self.state = "STARTING"
                self.arrive_at(self.goal_key)
            elif not self.changing:
                self.maybe_retarget()
        elif state == "DWELL":
            if self.button_held:
                self.dwell_until = now + DWELL_SEC
            if now >= self.dwell_until:
                self.finish_dwell()

        self.publish_markers()

    def track_passes(self):
        xy = self.pose_xy()
        if xy is None:
            return
        for i, k in enumerate(self.leg_stops):
            if k in self.passed:
                continue
            if self.dist_via(xy, k) <= PASS_RADIUS_M:
                self.passed.update(self.leg_stops[:i + 1])

    def maybe_retarget(self):
        """가는 길(지금 목표보다 앞)에 할 일이 생긴 정류장이 있으면 그쪽으로 목표를 앞당긴다"""
        xy = self.pose_xy()
        with self.lock:
            for k in self.leg_stops:
                if k in self.passed or k in self.change_failed:
                    continue
                if not self.has_work(k):
                    continue
                if xy is not None and self.dist_via(xy, k) < MIN_STOP_DIST_M:
                    continue
                self.changing = True
                self.jobs.put(lambda k=k: self.job_change(k))
                return

    # ---------- 정류장 도착 / 정차 ----------
    def serve_stop(self, k):
        """정류장 k 의 버튼 요청을 지우고 무엇 때문에 섰는지 글로 돌려준다 (lock 안에서 호출)"""
        r = self.requests.pop(k, None)
        if r is None:
            return '대기'
        why = []
        if r["dest"] or r["inside"]:
            why.append('내릴 사람')
        if r["station"]:
            self.mark_boarding(k)
            why.append(f'탑승 요청 ({time.time() - r["t"]:.0f}초 기다림)')
            self.log_event('served_station', k, waited_s=round(time.time() - r["t"], 1))
        return ', '.join(why)

    def arrive_at(self, k):
        with self.lock:
            loop_end = (k == self.run_until and k not in self.requests)
            if loop_end:
                # 탄 사람이 있었던 정류장으로 한 바퀴 돌아옴, 눌린 버튼 없음 -> 차가 비었다고 보고 대기
                self.run_until = None
                self.state = "STARTING"
                self.cur_key = k
                self.in_leg = False
                self.leg_stops = []
                self.goal_key = None
        if loop_end:
            self.get_logger().info(f'{self.name(k)}까지 한 바퀴 돌았습니다. 눌린 버튼이 없어 운행을 쉽니다.')
            self.jobs.put(self.job_next_leg)
            return
        with self.lock:
            info = self.serve_stop(k)
            self.state = "DWELL"
            self.dwell_key = k
            self.dwell_info = info
            self.dwell_until = time.time() + DWELL_SEC
            self.cur_key = k
            self.in_leg = False
            self.leg_stops = []
            self.goal_key = None
        self.get_logger().info(
            f'{self.name(k)} 정차, 문 열림: {info}. '
            f'{DWELL_SEC:.0f}초 뒤 출발 (버튼을 누르면 연장, hold 하는 동안은 출발하지 않음)')

    def finish_dwell(self):
        with self.lock:
            if self.state != "DWELL":
                return
            self.state = "STARTING"
            k = self.dwell_key
            self.dwell_key = None
            warn = self.boarded_at == k and not self.dest_since_board
            if warn:
                self.run_until = k          # 안전장치: 탄 사람이 도착지를 안 눌렀을 수 있음
            self.boarded_at = None
        if warn:
            self.get_logger().warn(
                f'{self.name(k)}에서 탄 사람이 도착지 버튼을 누르지 않았습니다. '
                f'"d 번호"로 내릴 정류장을 눌러 주세요. 그때까지는 {self.name(k)}까지 한 바퀴 돕니다.')
        self.jobs.put(self.job_next_leg)

    # ---------- 이동 작업 (작업 스레드) ----------
    def job_next_leg(self):
        """지금 정류장에서 다음 할 일이 있는 정류장까지 출발. 할 일이 없으면 대기"""
        with self.lock:
            frm = self.cur_key
            target = self.next_target(frm)
            home = False
            if target is None and self.run_until:
                target = self.run_until            # 탄 사람이 있을 수 있음 -> 한 바퀴 마저 돈다
            if target is None and IDLE_HOME and frm != IDLE_HOME:
                target, home = IDLE_HOME, True
            if target is None:
                self.state = "IDLE"
                self.goal_key = None
                self.leg_stops = []
                idle = True
            else:
                idle = False
                self.goal_key = target
                # 같은 정류장으로 돌아오는 경우(떠난 직후 그 정류장 버튼)는 한 바퀴 전체를 경유
                self.leg_stops = stops_between(frm, target) if target != frm else loop_after(frm)[:-1]
                self.passed = set()
                self.change_failed = set()
        if idle:
            self.get_logger().info(f'눌린 버튼이 없어 {self.name(frm)}에서 대기합니다.')
            return
        if target == self.run_until and not self.has_work(target):
            self.get_logger().info(
                f'눌린 버튼은 없지만 도착지를 안 누른 사람이 있을 수 있어 {self.name(target)}까지 한 바퀴 돕니다.')
        if home:
            self.get_logger().info(f'눌린 버튼이 없어 대기 정류장 {self.name(target)}(으)로 돌아갑니다.')
        if self.depart(target, self.leg_stops):
            with self.lock:
                self.in_leg = True
                self.state = "MOVING"
        else:
            self.halt(self.job_next_leg)

    def job_change(self, k):
        """주행 중 목표를 가는 길의 정류장 k 로 앞당김. 실패하면 원래 경로대로 계속 (k 는 다음 바퀴)"""
        try:
            with self.lock:
                idx = self.leg_stops.index(k) if k in self.leg_stops else None
                wps = [s for s in self.leg_stops[:idx] if s not in self.passed] if idx is not None else []
            if idx is None:
                return
            if self.change_route(k, wps):
                with self.lock:
                    self.goal_key = k
                    self.leg_stops = self.leg_stops[:idx]
                self.get_logger().info(f'가는 길의 {self.name(k)} 버튼이 눌려 먼저 섭니다.')
            else:
                with self.lock:
                    self.change_failed.add(k)
                self.get_logger().warn(
                    f'{self.name(k)}(으)로 목표를 바꾸지 못했습니다. 이번에는 지나가고 다음 바퀴에 들릅니다.')
        finally:
            self.changing = False

    def halt(self, retry_job):
        with self.lock:
            self.state = "HALTED"
            self.retry_job = retry_job
        self.get_logger().error('출발하지 못해 멈췄습니다. Autoware 상태를 확인한 뒤 "retry"를 입력하세요.')

    def retry(self):
        with self.lock:
            ok = self.state == "HALTED" and self.retry_job is not None
            if ok:
                job = self.retry_job
                self.retry_job = None
                self.state = "STARTING"
        if not ok:
            self.get_logger().info('다시 시도할 작업이 없습니다.')
            return
        self.jobs.put(job)

    # ---------- 경로 설정 / 자율주행 전환 ----------
    def call(self, client, req, label, timeout=10.0):
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f'{label} 서비스를 찾을 수 없습니다.')
            return None
        future = client.call_async(req)
        t0 = time.time()
        while not future.done() and time.time() - t0 < timeout:
            time.sleep(0.05)
        if not future.done():
            self.get_logger().error(f'{label} 응답 없음')
            return None
        res = future.result()
        if not res.status.success:
            self.get_logger().warn(f'{label} 실패: {res.status.message}')
        return res

    def make_route_req(self, goal_key, wps):
        req = SetRoutePoints.Request()
        req.header.frame_id = 'map'
        req.header.stamp = self.get_clock().now().to_msg()
        req.goal = goal_pose(goal_key)
        req.waypoints = [via_pose(k) for k in wps]
        return req

    def wait_route(self, states, timeout=ROUTE_SET_TIMEOUT_SEC):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.current_route_state in states:
                return True
            time.sleep(0.1)
        return False

    def depart(self, key, wps):
        """정류장에 서 있는 상태에서: 정지 모드 -> 경로 지우기 -> 경유점 포함 경로 설정 -> 자율주행"""
        via_txt = ' -> '.join(self.name(k) for k in wps) or '없음'
        self.get_logger().info(f'경로 설정: 목표 {self.name(key)}, 지나감 {via_txt}')

        if self.current_route_state != RouteState.UNSET:   # 모르면(None) 일단 지우고 시작
            if self.op_mode == OperationModeState.AUTONOMOUS:
                self.call(self.cli_stop, ChangeOperationMode.Request(), '정지 모드 전환')
                time.sleep(0.5)
            known = self.current_route_state is not None
            res = self.call(self.cli_clear, ClearRoute.Request(), '경로 지우기')
            if (res is None or not res.status.success) and known:
                return False
            self.wait_route((RouteState.UNSET,), 3.0)

        res = self.call(self.cli_set, self.make_route_req(key, wps), '경로 설정')
        if res is None or not res.status.success:
            self.get_logger().error(f'{self.name(key)} 경로가 설정되지 않았습니다. 초기 위치가 차선 위인지 확인하세요.')
            return False
        if not self.wait_route((RouteState.SET,)):
            self.get_logger().error('경로 상태가 SET 이 되지 않았습니다.')
            return False
        return self.engage()

    def change_route(self, key, wps):
        via_txt = ' -> '.join(self.name(k) for k in wps) or '없음'
        self.get_logger().info(f'경로 변경: 목표 {self.name(key)}, 지나감 {via_txt}')
        res = self.call(self.cli_change, self.make_route_req(key, wps), '경로 변경')
        if res is None or not res.status.success:
            return False
        self.wait_route((RouteState.SET,))
        return True

    def engage(self):
        if self.op_mode == OperationModeState.AUTONOMOUS:
            return True
        for attempt in range(1, ENGAGE_RETRIES + 1):
            t0 = time.time()      # Autoware 는 차가 완전히 서 있어야 전환을 받아 준다
            while self.speed > STATIONARY_SPEED and time.time() - t0 < STATIONARY_WAIT_SEC:
                time.sleep(0.1)
            res = self.call(self.cli_auto, ChangeOperationMode.Request(),
                            f'자율주행 전환 ({attempt}/{ENGAGE_RETRIES})', timeout=5.0)
            if res is not None and res.status.success:
                self.get_logger().info('자율주행 모드 전환 성공, 이동 시작')
                return True
            if attempt < ENGAGE_RETRIES:
                time.sleep(ENGAGE_RETRY_SEC)
        return False

    # ---------- RViz 표시 ----------
    def publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        with self.lock:
            state = self.state
            arr = MarkerArray()

            def mk(ns, mid, mtype, frame='map'):
                m = Marker()
                m.header.frame_id = frame
                m.header.stamp = stamp
                m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
                m.pose.orientation.w = 1.0
                return m

            for i, k in enumerate(LANE_ORDER):
                x, y, qz, qw = LOCATIONS[k]["goal"]
                req = self.requests.get(k)
                lines = [f"{k} {LOCATIONS[k]['en']}"]
                if state == "DWELL" and k == self.dwell_key:
                    rgba = (0.1, 0.85, 0.2, 0.9)
                    left = max(0.0, self.dwell_until - time.time())
                    lines.append(f"DOOR OPEN {left:.0f}s" + (" HOLD" if self.button_held else ""))
                elif k == self.goal_key and state in ("MOVING", "STARTING", "HALTED"):
                    rgba = (0.15, 0.45, 1.0, 0.9)
                    lines.append("NEXT STOP")
                elif k in self.passed and k in self.leg_stops:
                    rgba = (0.3, 0.3, 0.3, 0.6)
                elif req:
                    rgba = (1.0, 0.85, 0.1, 0.85)
                else:
                    rgba = (0.6, 0.6, 0.6, 0.5)
                if req:
                    lines.append(" + ".join(t for t, on in (("WAITING", req["station"]),
                                                             ("GET OFF", req["dest"] or req["inside"])) if on))

                disc = mk('stop_disc', i, Marker.CYLINDER)
                disc.pose.position.x, disc.pose.position.y, disc.pose.position.z = x, y, 0.1
                disc.scale.x = disc.scale.y = 8.0
                disc.scale.z = 0.2
                disc.color.r, disc.color.g, disc.color.b, disc.color.a = rgba
                arr.markers.append(disc)

                arrow = mk('stop_dir', i, Marker.ARROW)
                arrow.pose.position.x, arrow.pose.position.y, arrow.pose.position.z = x, y, 0.4
                arrow.pose.orientation.z, arrow.pose.orientation.w = qz, qw
                arrow.scale.x, arrow.scale.y, arrow.scale.z = 6.0, 1.0, 1.0
                arrow.color.r = arrow.color.g = arrow.color.b = 1.0
                arrow.color.a = 0.9
                arr.markers.append(arrow)

                label = mk('stop_text', i, Marker.TEXT_VIEW_FACING)
                label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y, 8.0
                label.scale.z = 5.0
                label.color.r = label.color.g = label.color.b = label.color.a = 1.0
                label.text = "\n".join(lines)
                arr.markers.append(label)

            goal = LOCATIONS[self.goal_key]['en'] if self.goal_key in LOCATIONS else '-'
            car = mk('shuttle_status', 0, Marker.TEXT_VIEW_FACING, frame='base_link')
            car.pose.position.z = 6.0
            car.scale.z = 2.5
            car.color.r, car.color.g, car.color.b, car.color.a = 1.0, 1.0, 0.3, 1.0
            car.text = (f"SHUTTLE {state}" + (f" -> {goal}" if state in ("MOVING", "STARTING") else "")
                        + f"\nrequests {len(self.requests)}")
            arr.markers.append(car)
        self.marker_pub.publish(arr)

    # ---------- 현황 ----------
    def print_status(self):
        with self.lock:
            state = self.state
            cur = self.name(self.cur_key) if self.cur_key else '-'
            goal = self.name(self.goal_key) if self.goal_key else '-'
            route = ' -> '.join(self.name(k) + ('(지남)' if k in self.passed else '')
                                for k in self.leg_stops) or '없음'
            now = time.time()
            rows = []
            for k in LANE_ORDER:
                r = self.requests.get(k)
                if r:
                    kinds = '+'.join(t for t, on in (('호출', r["station"]), ('도착지', r["dest"]),
                                                     ('하차벨', r["inside"])) if on)
                    rows.append(f'    {self.name(k)}: {kinds} 버튼, {now - r["t"]:.0f}초 전')
            extra = f', 문 닫힐 때까지 {max(0.0, self.dwell_until - now):.0f}초' if state == "DWELL" else ''
        self.get_logger().info(
            f'[현황] 상태={state} 마지막 정류장={cur} 다음 정차={goal}{extra}\n'
            f'  이번 구간 지나감: {route}\n'
            f'  한 바퀴 안전장치: {self.name(self.run_until) + "까지" if self.run_until else "없음"}\n'
            f'  눌린 버튼:\n' + ('\n'.join(rows) if rows else '    없음'))


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
                node.button_press()          # 차 안에서 문 잡기 (정차 중) / 정차 요청 (주행 중)
            elif raw == "release":
                node.button_release()
            elif raw == "retry":
                node.retry()
            elif raw == "status":
                node.print_status()
            elif raw in LOCATIONS:
                node.station_button(raw)
            elif raw.startswith('d'):
                node.dest_button(raw[1:].strip())
            else:
                print('형식이 맞지 않습니다. 정류장 호출 1~5, 차 안 도착지 d 1~5, 하차벨 b')

    threading.Thread(target=input_loop, daemon=True).start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
