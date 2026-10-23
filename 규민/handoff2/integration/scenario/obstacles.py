"""3-다 정차 차량·콘(세션58): 차로에 일부 걸친 세운 차 1대와 장애물 표시(role_name obstacle)를 단 콘 1개를 둔다.
- 차(vehicle.audi.a2): road lane -1 의 --car-s, 차선 중심 오른쪽 --car-lat(기본 1.875 = 차로 안으로 0.7m 걸침, 차선 폭 3.35, 차 폭 1.79)
- 콘(static.prop.trafficcone01, role obstacle): --cone-s, 오른쪽 --cone-lat(기본 0.9 = 차로 오른쪽 절반)
- 자차가 지나가거나 멈춘 동안 0.05s 마다 자차 위치·속도·차선 중심 옆 벗어남(lat)을 남긴다. 끝은 SIGINT 또는 --timeout
- 세션59: --no-car(콘만, 다1), --no-cone(걸친 차만, 다2)
사용: python obstacles.py <out.csv> [--road 1415] [--car-s 55] [--cone-s 30] [--no-car | --no-cone]
"""
import argparse, math, signal, time, carla
ap = argparse.ArgumentParser(); ap.add_argument("out")
ap.add_argument("--road", type=int, default=1415)
ap.add_argument("--car-s", type=float, default=55.0); ap.add_argument("--car-lat", type=float, default=1.875)
ap.add_argument("--cone-s", type=float, default=30.0); ap.add_argument("--cone-lat", type=float, default=0.9)
ap.add_argument("--timeout", type=float, default=1500)
ap.add_argument("--no-car", action="store_true"); ap.add_argument("--no-cone", action="store_true")
a = ap.parse_args()
c = carla.Client("localhost", 2000); c.set_timeout(20); w = c.get_world(); m = w.get_map(); bl = w.get_blueprint_library()


def at(s, lat, z):
    wp = m.get_waypoint_xodr(a.road, -1, s); tf = wp.transform; yaw = math.radians(tf.rotation.yaw)
    return carla.Location(tf.location.x - math.sin(yaw) * lat, tf.location.y + math.cos(yaw) * lat, z), tf.rotation.yaw


cl, cyaw = at(a.car_s, a.car_lat, 0.4); kl, kyaw = at(a.cone_s, a.cone_lat, 0.05)
cb = bl.find("vehicle.audi.a2"); cb.set_attribute("role_name", "scn_parked")
car = None if a.no_car else w.spawn_actor(cb, carla.Transform(cl, carla.Rotation(yaw=cyaw)))
kb = bl.find("static.prop.trafficcone01"); kb.set_attribute("role_name", "obstacle")
cone = None if a.no_cone else w.spawn_actor(kb, carla.Transform(kl, carla.Rotation(yaw=kyaw)))
print("car %s at %.2f %.2f (map y %.2f) s %.1f lat %.3f cone %s at %.2f %.2f s %.1f lat %.3f" % (car.id if car else None, cl.x, cl.y, -cl.y, a.car_s, a.car_lat,
      cone.id if cone else None, kl.x, kl.y, a.cone_s, a.cone_lat), flush=True)
stop = [False]; signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True)); signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
ego = None
while ego is None and not stop[0]:
    ego = next((v for v in w.get_actors().filter("vehicle.*") if v.attributes.get("role_name") == "ego_vehicle"), None)
    if ego is None: time.sleep(1)
for _ in range(10): w.wait_for_tick(30)
if car: car.set_simulate_physics(False)
f = open(a.out, "w"); f.write("frame,sim,ego_x,ego_y,ego_v,road,s,lat\n"); t0 = time.time()
try:
    while not stop[0] and time.time() - t0 < a.timeout:
        try: snap = w.wait_for_tick(5)
        except RuntimeError: continue
        e = snap.find(ego.id)
        if e is None: break
        l = e.get_transform().location; v = e.get_velocity().length()
        wp = m.get_waypoint(l, project_to_road=True, lane_type=carla.LaneType.Driving); yaw = math.radians(wp.transform.rotation.yaw)
        lat = -(l.x - wp.transform.location.x) * math.sin(yaw) + (l.y - wp.transform.location.y) * math.cos(yaw)
        f.write("%d,%.3f,%.2f,%.2f,%.2f,%d,%.2f,%.3f\n" % (snap.frame, snap.timestamp.elapsed_seconds, l.x, -l.y, v, wp.road_id, wp.s, lat))
finally:
    f.close()
    for x in (cone, car):
        if x is None: continue
        try: x.destroy()
        except Exception as ex: print("destroy", ex)
    print("end", flush=True)
