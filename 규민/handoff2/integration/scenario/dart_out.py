"""3-가 돌발 보행자(세션58): 차로 밖(인도·지면 판 위)에 세운 밴 앞(자차 쪽에서 보면 밴 뒤)에 보행자가 숨어 있다가 차로로 나와 건넌다.
i5_ped_cross.py 와 같은 방식(직접 WalkerControl)에 가리개 차와 보행자 충돌 센서를 더했다.
- 기준점: road lane -1 의 s. 밴(vehicle.mercedes.sprinter, 높이 약 2.5m) 중심은 차선 중심에서 오른쪽 --van-lat, s - 1.5m
- 보행자: 오른쪽 --ped-lat, s + 2.2m(밴 앞끝 바로 앞)에서 왼쪽 반대편 인도(차선 중심 왼쪽 4.5m)로 --speed m/s
- 출발: 자차 중심과 보행자 건너는 줄(s + 2.2) 거리 < --trigger
- 기록 0.05s: 자차 위치·속도, 보행자 위치, 앞 범퍼에서 보행자까지(자차 진행 방향 gap), 두 상자 사이 최소 거리(근사), 충돌
사용: python dart_out.py <out.csv> --road 1415 --s 45 [--trigger 20] [--speed 1.4] [--timeout 1200]
"""
import argparse, math, time, carla
ap = argparse.ArgumentParser(); ap.add_argument("out")
ap.add_argument("--road", type=int, default=1415); ap.add_argument("--s", type=float, default=45.0)
ap.add_argument("--trigger", type=float, default=20.0); ap.add_argument("--speed", type=float, default=1.4)
ap.add_argument("--van-lat", type=float, default=3.1); ap.add_argument("--ped-lat", type=float, default=2.6)
ap.add_argument("--timeout", type=float, default=1200)
a = ap.parse_args()
c = carla.Client("localhost", 2000); c.set_timeout(20); w = c.get_world(); m = w.get_map(); bl = w.get_blueprint_library()


def at(s, lat, z):
    wp = m.get_waypoint_xodr(a.road, -1, s); tf = wp.transform; yaw = math.radians(tf.rotation.yaw)
    rx, ry = -math.sin(yaw), math.cos(yaw)          # CARLA(왼손) 진행 방향 오른쪽
    return carla.Location(tf.location.x + rx * lat, tf.location.y + ry * lat, z), tf.rotation.yaw, (rx, ry)


vl, vyaw, (rx, ry) = at(a.s - 1.5, a.van_lat, 0.4)
pl, _, _ = at(a.s + 2.2, a.ped_lat, 1.0)
el_, _, _ = at(a.s + 2.2, -4.5, 1.0)
cx, cy = (pl.x + el_.x) / 2, (pl.y + el_.y) / 2
vb = bl.find("vehicle.mercedes.sprinter"); vb.set_attribute("role_name", "scn_parked")
van = w.spawn_actor(vb, carla.Transform(vl, carla.Rotation(yaw=vyaw)))
ego = None
while ego is None:
    ego = next((v for v in w.get_actors().filter("vehicle.*") if v.attributes.get("role_name") == "ego_vehicle"), None)
    if ego is None: time.sleep(1)
pb = bl.find("walker.pedestrian.0001"); pb.set_attribute("role_name", "scn_ped")
if pb.has_attribute("is_invincible"): pb.set_attribute("is_invincible", "false")
ped = w.spawn_actor(pb, carla.Transform(pl, carla.Rotation(yaw=math.degrees(math.atan2(el_.y - pl.y, el_.x - pl.x)))))
col = w.spawn_actor(bl.find("sensor.other.collision"), carla.Transform(), attach_to=ped); hits = []
col.listen(lambda e: hits.append((e.timestamp, e.other_actor.type_id, e.other_actor.id)))
L = math.hypot(el_.x - pl.x, el_.y - pl.y); dvec = carla.Vector3D((el_.x - pl.x) / L, (el_.y - pl.y) / L, 0)
ebb = ego.bounding_box
f = open(a.out, "w"); f.write("frame,wall,sim,state,ego_x,ego_y,ego_v,ped_x,ped_y,gap,box_dist,hits\n")
print("van %d at %.2f %.2f ped %d start %.2f %.2f end %.2f %.2f cross %.2fm ego %d" % (van.id, vl.x, vl.y, ped.id, pl.x, pl.y, el_.x, el_.y, L, ego.id), flush=True)
for _ in range(10): w.wait_for_tick(30)
van.set_simulate_physics(False)
state, t0 = "wait", time.time()
try:
    while time.time() - t0 < a.timeout:
        snap = w.wait_for_tick(30); sim = snap.timestamp.elapsed_seconds
        e = snap.find(ego.id); p = snap.find(ped.id)
        if e is None or p is None: break
        et = e.get_transform(); el, pp = et.location, p.get_transform().location; v = e.get_velocity().length()
        fwd = et.get_forward_vector(); rgt = et.get_right_vector()
        dxp, dyp = pp.x - el.x, pp.y - el.y
        lon = dxp * fwd.x + dyp * fwd.y - ebb.location.x; lat = dxp * rgt.x + dyp * rgt.y - ebb.location.y
        gap = lon - ebb.extent.x
        box = math.hypot(max(0.0, abs(lon) - ebb.extent.x - 0.19), max(0.0, abs(lat) - ebb.extent.y - 0.19))   # 보행자 반폭 0.19
        if state == "wait" and math.hypot(el.x - cx, el.y - cy) < a.trigger:
            state = "walk"; ped.apply_control(carla.WalkerControl(direction=dvec, speed=a.speed)); print("start sim %.2f ego v %.2f" % (sim, v), flush=True)
        elif state == "walk" and (pp.x - pl.x) * dvec.x + (pp.y - pl.y) * dvec.y >= L:
            state = "done"; ped.apply_control(carla.WalkerControl(speed=0.0)); print("arrived sim %.2f" % sim, flush=True)
        f.write("%d,%.3f,%.3f,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d\n" % (snap.frame, time.time(), sim, state, el.x, -el.y, v, pp.x, -pp.y, gap, box, len(hits)))
        if state == "done" and gap < -10: break     # 자차가 충분히 지나감
finally:
    f.close(); print("hits", hits[:5], flush=True)
    for x in (col, ped, van):
        try:
            if x.type_id.startswith("sensor"): x.stop()
            x.destroy()
        except Exception as ex: print("destroy", ex)
