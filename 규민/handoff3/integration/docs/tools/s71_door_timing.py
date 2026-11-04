"""10/26: 플래닝 시뮬레이터 가상 문(launch_dummy_doors)의 열림·닫힘 시간을 잼.
/api/vehicle/doors/status 를 구독해 상태가 바뀐 시각을 ms 로 남기고, 명령(열기 -> 닫기)을 N 번 보냄.
s62_ps_door.sh 는 1초마다 한 번 읽어 '약 2초'까지만 보였음. 이 스크립트는 메시지마다 시각을 봄.
컨테이너 안에서 돌림: docker exec -i -u 1000:1000 <컨테이너> bash -c 'source /opt/autoware/setup.bash; python3 - [N] [index]' < s71_door_timing.py
상태 값: 2 OPENED, 3 CLOSED, 4 OPENING, 5 CLOSING. 시간 = 명령 응답 받은 시각 -> 끝 상태(2·3) 처음 본 시각"""
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from autoware_adapi_v1_msgs.msg import DoorStatusArray
from autoware_adapi_v1_msgs.srv import SetDoorCommand
from autoware_adapi_v1_msgs.msg import DoorCommand

N = int(sys.argv[1]) if len(sys.argv) > 1 else 3
IDX = int(sys.argv[2]) if len(sys.argv) > 2 else 0


class T(Node):
    def __init__(self):
        super().__init__("s71_door_timing")
        self.last, self.events, self.msgs = None, [], 0
        self.create_subscription(DoorStatusArray, "/api/vehicle/doors/status", self.cb, QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE))
        self.cli = self.create_client(SetDoorCommand, "/api/vehicle/doors/command")

    def cb(self, m):
        self.msgs += 1
        st = m.doors[IDX].status if len(m.doors) > IDX else None
        if st != self.last:
            self.events.append((time.monotonic(), st)); self.last = st

    def wait(self, cond, to):
        t0 = time.monotonic()
        while time.monotonic() - t0 < to:
            rclpy.spin_once(self, timeout_sec=0.01)
            if cond(): return True
        return False

    def command(self, c):
        req = SetDoorCommand.Request(); d = DoorCommand(); d.index = IDX; d.command = c; req.doors = [d]
        f = self.cli.call_async(req)
        self.wait(f.done, 10)
        return time.monotonic(), f.result()


rclpy.init(); n = T()
n.cli.wait_for_service(timeout_sec=20)
n.wait(lambda: n.last is not None, 10)
t_a = time.monotonic(); n.wait(lambda: False, 2.0); rate = n.msgs / (time.monotonic() - t_a)
print(f"status 메시지 {rate:.1f}Hz, 처음 상태 {n.last}")
res = {1: [], 2: []}
for k in range(N):
    for c, want in ((1, 2), (2, 3)):
        mark = len(n.events)
        t0, r = n.command(c)
        ok = n.wait(lambda: n.last == want, 15)
        ev = n.events[mark:]
        seq = " ".join(f"{s}@{t - t0:+.3f}" for t, s in ev)
        dt = next((t - t0 for t, s in ev if s == want), None)
        res[c].append(dt)
        print(f"{k + 1} {'열기' if c == 1 else '닫기'} success={r.status.success if r else None} 상태 {seq} -> {want} 까지 {dt if dt is None else round(dt, 3)}s")
        n.wait(lambda: False, 1.0)
for c in (1, 2):
    v = [x for x in res[c] if x is not None]
    print(f"{'열기' if c == 1 else '닫기'} {len(v)}/{N}회, 최소 {min(v):.3f} 최대 {max(v):.3f} 평균 {sum(v) / len(v):.3f}s" if v else f"{c} 없음")
n.destroy_node(); rclpy.shutdown()
