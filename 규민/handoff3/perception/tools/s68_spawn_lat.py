"""10/23 3단계: 정답 노드 '소환 -> 처음 낸 시각' 시험 장면. 자차가 서 있을 때 앞 6~12m 차로 안에 보행자를 하나씩 소환했다가 지운다
- 소환 frame = 소환 뒤 스냅샷에 그 id 가 처음 보인 frame(서버 쪽 시각). 정답 노드 기록 csv(why=pub 첫 frame)와 맞춰 지연을 셈(s68_spawn_lat_eval.py)
- 간격은 2.35s(1초 주기와 위상이 바뀌게, 가짜 carla 시험과 같음). 각 보행자는 1.5s 둔 뒤 지움
사용: s68_spawn_lat.py <out.csv> [--n 12]
"""
import argparse, csv, math, time
import carla

ap = argparse.ArgumentParser(); ap.add_argument("out"); ap.add_argument("--n", type=int, default=12)
ap.add_argument("--gap", type=float, default=2.35); ap.add_argument("--hold", type=float, default=1.5)
a = ap.parse_args()
cl = carla.Client("localhost", 2000); cl.set_timeout(20); w = cl.get_world(); bl = w.get_blueprint_library()
ego = None
while ego is None:
    ego = next((v for v in w.get_actors().filter("vehicle.*") if v.attributes.get("role_name") == "ego_vehicle"), None)
    if ego is None: time.sleep(1)
tf = ego.get_transform(); fw = tf.get_forward_vector(); rt = tf.get_right_vector()
f = open(a.out, "w", newline=""); wr = csv.writer(f); wr.writerow(["k", "actor", "spawn_frame", "spawn_sim", "destroy_frame", "ahead_m", "lat_m", "x_map", "y_map"])
pb = bl.find("walker.pedestrian.0001"); pb.set_attribute("role_name", "s68_lat")
if pb.has_attribute("is_invincible"): pb.set_attribute("is_invincible", "false")
for k in range(a.n):
    ahead = 6 + (k % 4) * 2; lat = (-1.0, 0.0, 1.0)[k % 3]
    loc = tf.location + carla.Location(fw.x * ahead + rt.x * lat, fw.y * ahead + rt.y * lat, 1.0)
    p = w.try_spawn_actor(pb, carla.Transform(loc, tf.rotation))
    if p is None: print("spawn 실패", k, flush=True); continue
    while True:
        s = w.wait_for_tick(10)
        if s.find(p.id) is not None: sf, ss = s.frame, s.timestamp.elapsed_seconds; break
    t_end = ss + a.hold
    while True:
        s = w.wait_for_tick(10)
        if s.timestamp.elapsed_seconds >= t_end: break
    p.destroy(); df = w.wait_for_tick(10).frame
    wr.writerow([k, p.id, sf, round(ss, 3), df, ahead, lat, round(loc.x, 3), round(-loc.y, 3)]); f.flush()
    print("k %d actor %d spawn_frame %d" % (k, p.id, sf), flush=True)
    t_next = ss + a.gap
    while w.wait_for_tick(10).timestamp.elapsed_seconds < t_next: pass
print("done", flush=True)
