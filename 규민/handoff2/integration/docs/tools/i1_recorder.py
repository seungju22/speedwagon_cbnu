"""I1: CARLA 레코더 시작/끝, 자차 id·위치 출력 (호스트 .venv-carla)
사용: python i1_recorder.py start <절대경로.log> | stop | ego
"""
import sys, carla
c = carla.Client("localhost", 2000); c.set_timeout(20)
cmd = sys.argv[1]
if cmd == "start":
    print(c.start_recorder(sys.argv[2], True))
elif cmd == "stop":
    c.stop_recorder(); print("stopped")
w = c.get_world(); w.wait_for_tick(10.0)  # 동기 모드: 첫 틱을 받아야 액터·스냅샷이 채워짐
for v in w.get_actors().filter("vehicle.*"):
    l = v.get_location(); sp = v.get_velocity()
    print("ego?", v.id, v.attributes.get("role_name"), "%.2f %.2f %.2f" % (l.x, l.y, l.z), "v %.2f" % (sp.length()), "frame", w.get_snapshot().frame, "sim_t %.2f" % w.get_snapshot().timestamp.elapsed_seconds)
