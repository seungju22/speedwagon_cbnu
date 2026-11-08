#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from autoware_adapi_v1_msgs.srv import ChangeOperationMode
from autoware_adapi_v1_msgs.msg import RouteState
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

        self.state = "IDLE"               # IDLE / MOVING_TO_PICKUP / MOVING_TO_DEST
        self.current_route_state = None
        self.target_key = None
        self.pending_destination = None
        self.request_queue = deque()       # (origin, destination) 튜플 큐
        self.log = []

        self.get_logger().info('셔틀 대기 중...')

    def route_state_callback(self, msg):
        self.current_route_state = msg.state

    def call_shuttle(self, origin_key, dest_key):
        if origin_key not in LOCATIONS or dest_key not in LOCATIONS:
            self.get_logger().error('알 수 없는 지점 번호입니다.')
            return

        if origin_key == dest_key:
            self.get_logger().error('출발지와 목적지가 같습니다.')
            return

        if self.state != "IDLE":
            self.request_queue.append((origin_key, dest_key))
            self.get_logger().info(
                f'{LOCATIONS[origin_key]["name"]} -> {LOCATIONS[dest_key]["name"]} 요청을 대기열에 추가'
            )
            return

        self.start_trip(origin_key, dest_key)

    def start_trip(self, origin_key, dest_key):
        # 항상 픽업(출발지)부터 간다. 이미 근처에 있으면 Autoware가 알아서 짧게 처리함
        self.pending_destination = dest_key
        self.state = "MOVING_TO_PICKUP"
        self.target_key = origin_key

        self.log.append({
            'time': datetime.datetime.now().isoformat(),
            'event': 'depart_to_pickup',
            'destination': LOCATIONS[origin_key]['name']
        })
        self.send_goal(LOCATIONS[origin_key])

    def send_goal(self, loc):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = loc["x"]
        msg.pose.position.y = loc["y"]
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = loc["qz"]
        msg.pose.orientation.w = loc["qw"]

        self.get_logger().info(f'목적지 전송: {loc["name"]}')
        self.goal_pub.publish(msg)

        time.sleep(3.0)
        self.switch_to_autonomous()

    def switch_to_autonomous(self):
        if not self.autonomous_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('operation_mode 서비스를 찾을 수 없습니다.')
            return

        req = ChangeOperationMode.Request()
        future = self.autonomous_client.call_async(req)
        future.add_done_callback(self.autonomous_mode_callback)

    def autonomous_mode_callback(self, future):
        result = future.result()
        if result is not None and result.status.success:
            self.get_logger().info('자율주행 모드 전환 성공, 이동 시작')
        else:
            msg = result.status.message if result else '응답 없음'
            self.get_logger().error(f'자율주행 모드 전환 실패: {msg}')

    def check_arrival(self):
        if self.current_route_state != RouteState.ARRIVED:
            return

        if self.state == "MOVING_TO_PICKUP":
            arrived_name = LOCATIONS[self.target_key]["name"]
            self.get_logger().info(f'{arrived_name} 픽업 완료, 목적지로 출발')

            self.state = "MOVING_TO_DEST"
            self.target_key = self.pending_destination
            self.log.append({
                'time': datetime.datetime.now().isoformat(),
                'event': 'pickup_done_depart',
                'destination': LOCATIONS[self.target_key]['name']
            })
            self.send_goal(LOCATIONS[self.target_key])

        elif self.state == "MOVING_TO_DEST":
            arrived_name = LOCATIONS[self.target_key]["name"]
            self.get_logger().info(f'{arrived_name} 도착, 정차')
            self.target_key = None
            self.pending_destination = None

            self.log.append({
                'time': datetime.datetime.now().isoformat(),
                'event': 'arrive',
                'destination': arrived_name
            })

            self.state = "IDLE"

            if self.request_queue:
                origin, dest = self.request_queue.popleft()
                self.start_trip(origin, dest)
            else:
                print("\n호출 (출발지 도착지, 예: 1 5): ", end="", flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = ShuttleNode()
    node.create_timer(1.0, node.check_arrival)

    def input_loop():
        while rclpy.ok():
            print("\n호출 (출발지 도착지, 예: 1 5): ", end="")
            raw = input().strip()
            parts = raw.split()
            if len(parts) != 2:
                print("형식이 맞지 않습니다. 예: 1 5")
                continue
            origin, dest = parts
            node.call_shuttle(origin, dest)

    input_thread = threading.Thread(target=input_loop, daemon=True)
    input_thread.start()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
