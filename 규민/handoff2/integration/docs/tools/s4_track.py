"""S4: i5_track.py 에 네 바퀴 이탈 판정을 더한 판(호스트). 매 틱 위치·속도·sim 시각, 차선 중심 옆 벗어남, 충돌 센서,
그리고 바퀴 4개 위치가 CARLA 지도상 도로(차도 Driving + 인도 Sidewalk) 위인지.
지면 판(Plane)을 켜면 차도를 벗어난 차가 떨어지지 않으므로(세션51) 낙하 대신 이 판정으로 이탈을 본다.
바퀴 위치 = 처음 한 번 get_physics_control().wheels 위치(월드, cm)를 차 좌표로 바꿔 두고, 매 틱 차 Transform 으로 다시 월드로.
이탈 = get_waypoint(바퀴, project_to_road=False, lane_type=Driving|Sidewalk) 가 None. offroad(차 중심, 차도만)는 i5_track 과 같은 열
사용: python s4_track.py <out.csv> <timeout_wall_s>   (SIGINT 로 끝)
"""
import csv, json, math, signal, sys, time, carla
c = carla.Client("localhost", 2000); c.set_timeout(30)
w = c.get_world(); m = w.get_map(); out, TO = sys.argv[1], float(sys.argv[2])
ROAD = carla.LaneType.Driving | carla.LaneType.Sidewalk
ego = None
while ego is None:
    for v in w.get_actors().filter("vehicle.*"):
        if v.attributes.get("role_name") == "ego_vehicle": ego = v
    if ego is None: time.sleep(1); w = c.get_world()
w.wait_for_tick(5.0)
tf0 = ego.get_transform(); yaw0 = math.radians(tf0.rotation.yaw)
wheels = []
for wh in ego.get_physics_control().wheels:
    dx, dy, dz = wh.position.x / 100 - tf0.location.x, wh.position.y / 100 - tf0.location.y, wh.position.z / 100 - tf0.location.z
    wheels.append((dx * math.cos(yaw0) + dy * math.sin(yaw0), -dx * math.sin(yaw0) + dy * math.cos(yaw0), dz))
print("ego", ego.id, ego.type_id, "wheels_local", json.dumps([[round(a, 3) for a in p] for p in wheels]), flush=True)
col = w.spawn_actor(w.get_blueprint_library().find("sensor.other.collision"), carla.Transform(), attach_to=ego)
hits = []
col.listen(lambda e: hits.append((e.timestamp, e.other_actor.type_id, e.normal_impulse.length())))
f = open(out, "w", newline=""); wr = csv.writer(f)
wr.writerow(["wall", "sim", "x", "y", "yaw", "v", "road", "lane", "s", "lat", "offroad", "hits", "wheels_off", "wheels_off_ids"])
stop = [False]; signal.signal(signal.SIGINT, lambda *a: stop.__setitem__(0, True)); signal.signal(signal.SIGTERM, lambda *a: stop.__setitem__(0, True))
t0 = time.time(); last = -1; nwo = 0
while not stop[0] and time.time() - t0 < TO:
    try: snap = w.wait_for_tick(2.0)
    except RuntimeError: continue
    if snap.frame == last: continue
    last = snap.frame
    tf = ego.get_transform(); l = tf.location; v = ego.get_velocity().length()
    wp = m.get_waypoint(l, project_to_road=True, lane_type=carla.LaneType.Driving)
    on = m.get_waypoint(l, project_to_road=False, lane_type=carla.LaneType.Driving)
    yaw = math.radians(wp.transform.rotation.yaw); dx, dy = l.x - wp.transform.location.x, l.y - wp.transform.location.y
    lat = -dx * math.sin(yaw) + dy * math.cos(yaw)
    cy, sy = math.cos(math.radians(tf.rotation.yaw)), math.sin(math.radians(tf.rotation.yaw))
    off = [i for i, (a, b, _) in enumerate(wheels)
           if m.get_waypoint(carla.Location(l.x + a * cy - b * sy, l.y + a * sy + b * cy, l.z), project_to_road=False, lane_type=ROAD) is None]
    nwo += bool(off)
    wr.writerow([f"{time.time():.3f}", f"{snap.timestamp.elapsed_seconds:.3f}", f"{l.x:.3f}", f"{l.y:.3f}", f"{tf.rotation.yaw:.2f}", f"{v:.3f}",
                 wp.road_id, wp.lane_id, f"{wp.s:.2f}", f"{lat:.3f}", int(on is None), len(hits), len(off), " ".join(map(str, off))])
f.flush(); col.stop(); col.destroy()
print("hits", hits[:10], "wheel_off_ticks", nwo); f.close()
