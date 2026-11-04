"""I5: 회차 정리(i2_cleanup.py 의 순서를 바꾼 판, 2026-10-11 Signal 11 대응)
순서: 녹화 중지 -> 센서 stop·destroy(동기면 직접 tick) -> 동기 모드 끄기 -> 남은 차량·보행자 destroy
i2_cleanup.py 는 동기 끄기를 먼저 했고, I5 a3 정리 중 그 직후 CARLA 가 Signal 11 로 죽었다(i5_report.md 2절)
사용: python i5_cleanup.py
no_rendering_mode 는 건드리지 않는다.
"""
import carla
c = carla.Client("localhost", 2000); c.set_timeout(30)
c.stop_recorder()
w = c.get_world()
sync = w.get_settings().synchronous_mode


def step():
    if sync:
        w.tick()
    else:
        w.wait_for_tick(10)


step()      # 새 클라이언트는 tick 을 한 번 받아야 액터 목록이 찬다(2026-10-11 p1: 안 하면 센서 3개를 못 보고 남김)
sensors = [a for a in w.get_actors() if a.type_id.startswith("sensor.")]
for a in sensors:
    try:
        if a.is_listening:
            a.stop()
    except RuntimeError as e:
        print("stop", a.id, e)
for r, a in zip(c.apply_batch_sync([carla.command.DestroyActor(a) for a in sensors], sync), sensors):
    print("destroy", a.id, a.type_id, not r.error)
step()
if sync:
    s = w.get_settings(); s.synchronous_mode = False; s.fixed_delta_seconds = None; w.apply_settings(s)
w.wait_for_tick(10)
rest = [a for a in w.get_actors() if a.type_id.startswith(("vehicle.", "walker."))]
for a in rest:
    print("destroy", a.id, a.type_id, a.attributes.get("role_name", ""), a.destroy())
w.wait_for_tick(10)
left = [a.id for a in w.get_actors() if a.type_id.startswith(("vehicle.", "sensor.", "walker."))]
s = w.get_settings()
print("map", w.get_map().name, "was_sync", sync, "sync", s.synchronous_mode, "no_render", s.no_rendering_mode, "left", left)
