"""10/23 3단계: 거리별 평가 기록 장면. 자차가 서 있는 곧은 길(기본 r1194 lane -1 s=20, 앞 45m 가 곧음)에서
보행자·차·콘을 자차 앞 5·10·15·20·25·30·40m, 옆 0(차로 가운데)과 차로 가장자리(연석 쪽)에 하나씩 두고 --hold 초씩 둔다
- 앞 거리는 자차 뒷바퀴 축이 아니라 자차 원점(차 중심)에서 차로를 따라 잰 거리. 가장자리는 차로 폭/2(물체 중심이 경계선 위)
- 연석 쪽 = 경계선에서 1m 더 나간 자리가 Driving 차로가 아닌 쪽(반쪽 도로라 한쪽은 마주 오는 반쪽 도로)
- 기록 csv: k, 종류, 앞 거리, 옆 이름, 옆 m, actor, 지도 x·y, 놓은 frame(스냅샷에 처음 보인 frame), 지운 frame
사용: s68_dist_scene.py <out.csv> [--road 1194 --lane -1 --s 20] [--hold 3.5] [--types ped,car,cone] [--dists 5,10,15,20,25,30,40]
"""
import argparse, csv, math, time
import carla

ap = argparse.ArgumentParser(); ap.add_argument("out")
ap.add_argument("--road", type=int, default=1194); ap.add_argument("--lane", type=int, default=-1); ap.add_argument("--s", type=float, default=20.0)
ap.add_argument("--hold", type=float, default=3.5); ap.add_argument("--settle", type=float, default=0.5)
ap.add_argument("--types", default="ped,car,cone"); ap.add_argument("--dists", default="5,10,15,20,25,30,40")
a = ap.parse_args()
cl = carla.Client("localhost", 2000); cl.set_timeout(20); w = cl.get_world(); m = w.get_map(); bl = w.get_blueprint_library()
ego = None
while ego is None:
    ego = next((v for v in w.get_actors().filter("vehicle.*") if v.attributes.get("role_name") == "ego_vehicle"), None)
    if ego is None: time.sleep(1)
el = ego.get_transform().location
wp0 = m.get_waypoint(el, project_to_road=True, lane_type=carla.LaneType.Driving)
print("ego %.2f %.2f -> wp road %d lane %d s %.2f (지정 road %d s %.1f)" % (el.x, el.y, wp0.road_id, wp0.lane_id, wp0.s, a.road, a.s), flush=True)
BP = {"ped": "walker.pedestrian.0001", "car": "vehicle.audi.a2", "cone": "static.prop.trafficcone01"}


def curb_sign(wp):
    rt = wp.transform.get_right_vector(); hw = wp.lane_width / 2 + 1.0
    for sg in (1, -1):
        p = wp.transform.location + carla.Location(rt.x * hw * sg, rt.y * hw * sg, 0)
        q = m.get_waypoint(p, project_to_road=False, lane_type=carla.LaneType.Driving)
        if q is None: return sg
    return 1


def tick_until(t):
    while True:
        s = w.wait_for_tick(10)
        if s.timestamp.elapsed_seconds >= t: return s


f = open(a.out, "w", newline=""); wr = csv.writer(f)
wr.writerow(["k", "type", "ahead_m", "lat_name", "lat_m", "actor", "x_map", "y_map", "frame_on", "sim_on", "frame_off", "sim_off"])
k = 0
for typ in a.types.split(","):
    b = bl.find(BP[typ]); b.set_attribute("role_name", "obstacle" if typ == "cone" else "s68_dist")
    if b.has_attribute("is_invincible"): b.set_attribute("is_invincible", "false")
    for d in [float(x) for x in a.dists.split(",")]:
        nx = wp0.next(d)
        if not nx: print("앞 %.0fm waypoint 없음" % d, flush=True); continue
        wp = nx[0]; sg = curb_sign(wp); rt = wp.transform.get_right_vector()
        for lname, lat in (("center", 0.0), ("edge", sg * wp.lane_width / 2)):
            z = {"ped": 1.0, "car": 0.3, "cone": 0.05}[typ]
            loc = wp.transform.location + carla.Location(rt.x * lat, rt.y * lat, z)
            act = w.try_spawn_actor(b, carla.Transform(loc, wp.transform.rotation))
            if act is None: print("spawn 실패", typ, d, lname, flush=True); continue
            while True:
                s = w.wait_for_tick(10)
                if s.find(act.id) is not None: break
            if typ == "car": act.set_simulate_physics(False)
            fo, so = s.frame, s.timestamp.elapsed_seconds
            s2 = tick_until(so + a.settle + a.hold)
            act.destroy(); s3 = tick_until(s2.timestamp.elapsed_seconds + 0.3)
            wr.writerow([k, typ, d, lname, round(lat, 3), act.id, round(loc.x, 3), round(-loc.y, 3), fo, round(so, 3), s3.frame, round(s3.timestamp.elapsed_seconds, 3)]); f.flush()
            print("k %d %s %.0fm %s actor %d frame %d" % (k, typ, d, lname, act.id, fo), flush=True); k += 1
print("done", k, flush=True)
