"""정답 객체 2판(P1b, 세션58). P1(gt_objects.py)에 '차 시점' 모드를 더했다. P1 파일은 그대로 둔다.
토픽·형식은 P1 과 같다: /perception/object_recognition/objects (PredictedObjects, frame map). Autoware 는 고치지 않는다.

모드(--mode)
  car  (기본) 차 시점: 지붕 LiDAR 자리에서 보이는 것만 넘긴다
       - 거리: LiDAR 자리에서 물체 bbox 중심까지(기본값 근거 perception/docs/gt_objects2.md). 보행자 --ped-range, 차량 --veh-range, 장애물 소품 --prop-range, 모두 x --range-scale
       - 시야각: 표본점의 올려본 각이 LiDAR 상하 시야(--lower-fov ~ --upper-fov) 안이어야 한다
       - 가려짐: LiDAR 자리에서 표본점(가운데, 위 가운데, 가운데 높이 모서리 4, 위 모서리 4)으로 world.cast_ray.
         시야 안 표본점 하나라도 안 막히면 보인다. 막힘으로 안 치는 것: 그 물체 자기 상자 안, 자차 상자 안, 지면(라벨 Ground·Roads·
         Sidewalks·RoadLines·Terrain 또는 z < 0.15m)
       - 비 조건 자리(기본 꺼짐): --range-scale 1.0, --miss-prob 0(물체·틱마다 버릴 확률), --seed 0
  all  다 보기(P1 과 같음): 자차 중심에서 --range(80m) 안 모두
대상: 자차를 뺀 vehicle.*, walker.pedestrian.*, 그리고 role_name 이 obstacle 인 static.prop.*(UNKNOWN). 건물·정류장 시설·지면 판 등
      레이어 소환물(role_name 없음)은 넘기지 않는다
LiDAR 자리: 브리지 0.52.0 carla_sensor_kit_description sensor_kit_calibration.yaml velodyne_top_base_link (x 1.035, z 1.840, base_link 기준)
  -> sensor_kit_loader.py 가 축거 2.850/2 를 빼 CARLA 차 원점 기준 (x -0.390, y 0, z 1.840)
시야: upper_fov 10, lower_fov -30(sensor_mapping_nocam.yaml 과 브리지 기본 sensor_mapping.yaml 이 같다)
기본 거리(세션59): 기준 센서(브리지 기본 LiDAR 64ch·초당 30만 점·20Hz·100m, 센서 킷 자리, 버림·노이즈 0)로 시맨틱 LiDAR 10점 이상 가장 먼 거리.
  세션58 값(7.5/20/6.0)은 가벼운 LiDAR(32ch·10만 점) 기준이라 쓰지 않는다. 근거 perception/docs/gt_objects2.md, integration/docs/s59_report.md 2절

cast_ray 는 RPC 라 CARLA 틱 콜백에서 하지 않는다. 일꾼 스레드가 가장 최근 스냅샷으로 보이는지를 계산해 두고, 틱 콜백은 그 결과를 쓴다
(결과는 한두 틱 늦을 수 있다. 늦음은 로그 열 vis_lag 로 남긴다).

사용: python3 gt_objects2.py [--mode car|all] [--ped-range 17.5] [--veh-range 35] [--prop-range 12.5] [--range-scale 1.0] [--miss-prob 0] [--seed 0]
                             [--range 80] [--log 틱별.csv] [--check N]
  --log: 틱마다 물체마다 한 줄(정답 전체와 낸 것). --check N: ROS 없이 N 틱 동안 판정만 찍는다(호스트 .venv-carla 에서도 됨)
"""
import argparse, collections, math, queue, random, sys, threading, time
import carla
import carview   # 같은 폴더. 판정은 대조 도구와 같이 쓴다

ap = argparse.ArgumentParser()
ap.add_argument("--host", default="localhost")
ap.add_argument("--port", type=int, default=2000)
ap.add_argument("--role", default="ego_vehicle")
ap.add_argument("--mode", choices=["car", "all"], default="car")
ap.add_argument("--range", type=float, default=80.0, help="all 모드 거리")
ap.add_argument("--ped-range", type=float, default=17.5)  # 세션59 기준 센서 대조: 시맨틱 10점 이상 17.7m(앞, 두 자리). 가벼운 LiDAR 7.5
ap.add_argument("--veh-range", type=float, default=35.0)  # 같은 대조: 승용차(audi.a2) 35.2m(r1415)·36.7m(r1411). 가벼운 LiDAR 20
ap.add_argument("--prop-range", type=float, default=12.5)  # 같은 대조: 콘 12.7m(앞, 두 자리). 가벼운 LiDAR 6.0
ap.add_argument("--range-scale", type=float, default=1.0)
ap.add_argument("--miss-prob", type=float, default=0.0)
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--upper-fov", type=float, default=10.0)
ap.add_argument("--lower-fov", type=float, default=-30.0)
ap.add_argument("--lidar", default="-0.390,0.0,1.840", help="CARLA 차 원점 기준 x,y,z")
ap.add_argument("--log", default="")
ap.add_argument("--check", type=int, default=0)
a = ap.parse_args()

UNKNOWN, CAR, TRUCK, BUS, MOTORCYCLE, BICYCLE, PEDESTRIAN = 0, 1, 2, 3, 5, 6, 7
LABEL_NAME = {0: "UNKNOWN", 1: "CAR", 2: "TRUCK", 3: "BUS", 5: "MOTORCYCLE", 6: "BICYCLE", 7: "PEDESTRIAN"}
LX, LY, LZ = (float(v) for v in a.lidar.split(","))
RNG = random.Random(a.seed)


def label_of(actor):
    if actor.type_id.startswith("walker."):
        return PEDESTRIAN
    if actor.type_id.startswith("static.prop"):
        return UNKNOWN
    bt = actor.attributes.get("base_type", "").lower()
    if bt == "motorcycle": return MOTORCYCLE
    if bt == "bicycle": return BICYCLE
    if bt == "bus": return BUS
    if bt in ("truck", "van"): return TRUCK
    return CAR


def range_of(lab):
    r = a.ped_range if lab == PEDESTRIAN else (a.prop_range if lab == UNKNOWN else a.veh_range)
    return r * a.range_scale


def quat_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


c = carla.Client(a.host, a.port); c.set_timeout(20)
world = c.get_world()


class Cache:
    """actor 종류·bbox 는 1초마다 다시 읽는다(스냅샷에는 id·자세·속도만 있다)"""
    def __init__(s):
        s.t = 0; s.ego = None; s.ego_bb = None; s.info = {}

    def refresh(s, force=False):
        if not force and time.time() - s.t < 1.0: return
        s.t = time.time(); info = {}; ego = None; ego_bb = None
        for v in world.get_actors():
            tid = v.type_id
            if tid.startswith("vehicle."):
                if v.attributes.get("role_name") == a.role:
                    ego = v.id; ego_bb = v.bounding_box; continue
            elif tid.startswith("walker.pedestrian"):
                pass
            elif tid.startswith("static.prop") and v.attributes.get("role_name") == "obstacle":
                pass
            else:
                continue
            bb = v.bounding_box
            info[v.id] = (label_of(v), bb.location, bb.extent, tid)
        s.ego = ego; s.ego_bb = ego_bb; s.info = info


cache = Cache()


def visibility(snap):
    """{aid: (dist, in_range, n_fov, n_clear)}. RPC(cast_ray) 를 하므로 일꾼 스레드나 --check 에서만 부른다"""
    out = {}
    es = snap.find(cache.ego) if cache.ego else None
    if es is None: return out
    etf = es.get_transform()
    lid = carview.lidar_location(etf, (LX, LY, LZ))
    walkers = []
    for aid, (lab, bloc, ext, _) in list(cache.info.items()):
        sa = snap.find(aid) if lab == PEDESTRIAN else None
        if sa is not None:
            walkers.append(sa.get_transform())
    for aid, (lab, bloc, ext, _) in list(cache.info.items()):
        sa = snap.find(aid)
        if sa is None: continue
        out[aid] = carview.judge(world, lid, sa.get_transform(), bloc, ext, range_of(lab), a.lower_fov, a.upper_fov, etf, cache.ego_bb,
                                    walkers=walkers)
    return out


def objects(snap, vis, vis_frame):
    """스냅샷 -> 정답 전체 목록과 낼 목록. RPC 없음(틱 콜백에서 부름)
    반환 (all_rows, pub_rows, es). row = (aid, lab, x, y, z, yaw, vx, vy, wz, dx, dy, dz, vxw, vyw, dist_ego, why)"""
    rows, pub = [], []
    es = snap.find(cache.ego) if cache.ego else None
    if es is None: return rows, pub, None
    el = es.get_transform().location
    for aid, (lab, bloc, ext, _) in list(cache.info.items()):
        sa = snap.find(aid)
        if sa is None: continue
        t = sa.get_transform()
        p = t.transform(carla.Location(bloc.x, bloc.y, bloc.z))
        de = math.hypot(p.x - el.x, p.y - el.y)
        yaw = -math.radians(t.rotation.yaw)
        v = sa.get_velocity(); vxw, vyw = v.x, -v.y
        cy, sy = math.cos(yaw), math.sin(yaw)
        if a.mode == "all":
            why = "pub" if de <= a.range else "range"
        else:
            vv = vis.get(aid)
            if vv is None: why = "novis"
            elif not vv[1]: why = "range"
            elif vv[2] == 0: why = "fov"
            elif vv[3] == 0: why = "occluded"
            else: why = "pub"
            if why == "pub" and a.miss_prob > 0 and RNG.random() < a.miss_prob: why = "miss"
        r = (aid, lab, p.x, -p.y, p.z, yaw, cy * vxw + sy * vyw, -sy * vxw + cy * vyw,
             -math.radians(sa.get_angular_velocity().z), 2 * ext.x, 2 * ext.y, 2 * ext.z, vxw, vyw, de, why)
        rows.append(r)
        if why == "pub": pub.append(r)
    return rows, pub, es


class Logger:
    def __init__(s, path):
        s.f = open(path, "w") if path else None
        if s.f:
            s.f.write("frame,sim,mode,aid,label,x,y,dist_ego,dist_lidar,why,vis_lag,n_fov,n_clear\n")

    def write(s, frame, sim, rows, vis, vis_frame):
        if not s.f: return
        for r in rows:
            vv = vis.get(r[0], (float("nan"), False, 0, 0))
            s.f.write("%d,%.3f,%s,%d,%s,%.3f,%.3f,%.3f,%.3f,%s,%d,%d,%d\n" % (
                frame, sim, a.mode, r[0], LABEL_NAME[r[1]], r[2], r[3], r[14], vv[0], r[15],
                frame - vis_frame if vis_frame >= 0 else -1, vv[2], vv[3]))
        s.f.flush()


log = Logger(a.log)

if a.check:
    for _ in range(a.check):
        snap = world.wait_for_tick(10)
        cache.refresh()
        t0 = time.perf_counter()
        vis = visibility(snap) if a.mode == "car" else {}
        dt = (time.perf_counter() - t0) * 1000
        rows, pub, es = objects(snap, vis, snap.frame)
        log.write(snap.frame, snap.timestamp.elapsed_seconds, rows, vis, snap.frame)
        print("frame %d ego %s all %d pub %d vis_ms %.1f" % (snap.frame, cache.ego, len(rows), len(pub), dt), flush=True)
        for r in rows:
            vv = vis.get(r[0], (float("nan"), False, 0, 0))
            print("  id %d %s x %.2f y %.2f d_ego %.1f d_lidar %.1f fov %d clear %d -> %s" % (
                r[0], LABEL_NAME[r[1]], r[2], r[3], r[14], vv[0], vv[2], vv[3], r[15]), flush=True)
    sys.exit(0)

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time, Duration
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Pose
from autoware_perception_msgs.msg import (PredictedObjects, PredictedObject, ObjectClassification,
                                          PredictedPath, Shape)

COV = [0.0] * 36
for i in range(6): COV[i * 7] = 1e-4
DT, HORIZON = 0.5, 8.0


class Worker(threading.Thread):
    """가장 최근 스냅샷으로 보이는지 계산(car 모드만). 처리 시간도 잰다"""
    def __init__(s):
        super().__init__(daemon=True)
        s.q = queue.Queue(maxsize=1); s.vis = {}; s.frame = -1; s.lock = threading.Lock(); s.ms = collections.deque(maxlen=100000)

    def push(s, snap):
        try: s.q.get_nowait()
        except queue.Empty: pass
        s.q.put_nowait(snap)

    def run(s):
        while True:
            snap = s.q.get()
            t0 = time.perf_counter()
            try:
                v = visibility(snap)
            except Exception as e:
                print("vis error", e, flush=True); continue
            s.ms.append((time.perf_counter() - t0) * 1000)
            with s.lock: s.vis = v; s.frame = snap.frame

    def get(s):
        with s.lock: return s.vis, s.frame


class GTObjects(Node):
    def __init__(s):
        super().__init__("gt_objects", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        s.pub = s.create_publisher(PredictedObjects, "/perception/object_recognition/objects", 10)
        s.create_subscription(Clock, "/clock", s.on_clock, 10)
        s.diffs = collections.deque(maxlen=40); s.offset = None; s.last_frame = -1; s.n = 0
        s.lock = threading.Lock(); s.seen = {}; s.tick_ms = collections.deque(maxlen=100000)
        s.worker = Worker() if a.mode == "car" else None
        if s.worker: s.worker.start()
        s.create_timer(1.0, cache.refresh, clock=rclpy.clock.Clock())  # 실제 시간 1초마다 actor 목록
        s.create_timer(30.0, s.stats, clock=rclpy.clock.Clock())

    def stats(s):
        def q(d):
            if not d: return "-"
            x = sorted(d); n = len(x)
            return "med %.2f p95 %.2f max %.2f n %d" % (x[n // 2], x[min(n - 1, int(n * 0.95))], x[-1], n)
        print("stat tick_ms %s | vis_ms %s" % (q(s.tick_ms), q(s.worker.ms) if s.worker else "-"), flush=True)

    def on_clock(s, m):
        sim = m.clock.sec + m.clock.nanosec * 1e-9
        snap = world.get_snapshot()
        with s.lock:
            raw = snap.timestamp.elapsed_seconds - sim
            s.diffs.append((round(raw / 0.05), raw))
            if len(s.diffs) >= 10:
                k = collections.Counter(d[0] for d in s.diffs).most_common(1)[0][0]
                r = sorted(d[1] for d in s.diffs if d[0] == k)
                s.offset = r[len(r) // 2]

    def on_tick(s, snap):
        t0 = time.perf_counter()
        with s.lock: off = s.offset
        if off is None or snap.frame <= s.last_frame: return
        s.last_frame = snap.frame
        if s.worker:
            s.worker.push(snap); vis, vf = s.worker.get()
        else:
            vis, vf = {}, snap.frame
        rows, obs, es = objects(snap, vis, vf)
        if es is None: return
        t = max(0.0, snap.timestamp.elapsed_seconds - off)
        msg = PredictedObjects(); msg.header.stamp = Time(sec=int(t), nanosec=int((t - int(t)) * 1e9)); msg.header.frame_id = "map"
        for (aid, lab, x, y, z, yaw, vx, vy, wz, dx, dy, dz, vxw, vyw, de, why) in obs:
            o = PredictedObject()
            o.object_id.uuid = list(aid.to_bytes(8, "little")) + [0x67, 0x74] + [0] * 6
            o.existence_probability = 1.0
            o.classification = [ObjectClassification(label=lab, probability=1.0)]
            k = o.kinematics
            k.initial_pose_with_covariance.pose.position.x, k.initial_pose_with_covariance.pose.position.y = x, y
            k.initial_pose_with_covariance.pose.position.z = z
            q = quat_yaw(yaw)
            (k.initial_pose_with_covariance.pose.orientation.x, k.initial_pose_with_covariance.pose.orientation.y,
             k.initial_pose_with_covariance.pose.orientation.z, k.initial_pose_with_covariance.pose.orientation.w) = q
            k.initial_pose_with_covariance.covariance = COV
            k.initial_twist_with_covariance.twist.linear.x, k.initial_twist_with_covariance.twist.linear.y = vx, vy
            k.initial_twist_with_covariance.twist.angular.z = wz
            k.initial_twist_with_covariance.covariance = COV
            k.initial_acceleration_with_covariance.covariance = COV
            pp = PredictedPath(); pp.time_step = Duration(sec=0, nanosec=int(DT * 1e9)); pp.confidence = 1.0
            sp = math.hypot(vxw, vyw)
            hy = math.atan2(vyw, vxw) if sp > 0.2 else yaw
            hq = quat_yaw(hy)
            for i in range(int(HORIZON / DT) + 1):
                ps = Pose(); ps.position.x = x + vxw * i * DT; ps.position.y = y + vyw * i * DT; ps.position.z = z
                ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = hq
                pp.path.append(ps)
            k.predicted_paths = [pp]
            o.shape.type = Shape.BOUNDING_BOX
            o.shape.dimensions.x, o.shape.dimensions.y, o.shape.dimensions.z = dx, dy, dz
            msg.objects.append(o)
            if aid not in s.seen:
                s.seen[aid] = 1
                print("new id %d %s at %.2f %.2f size %.2fx%.2f t %.2f d_ego %.1f" % (aid, LABEL_NAME[lab], x, y, dx, dy, t, de), flush=True)
        s.pub.publish(msg); s.n += 1
        log.write(snap.frame, t, rows, vis, vf)
        s.tick_ms.append((time.perf_counter() - t0) * 1000)
        if s.n % 200 == 1:
            print("pub", s.n, "frame", snap.frame, "t %.2f" % t, "all", len(rows), "n_obj", len(msg.objects), flush=True)


print("mode %s ped %.1f veh %.1f prop %.1f scale %.2f miss %.2f seed %d fov %.0f~%.0f lidar %s all_range %.0f" % (
    a.mode, a.ped_range, a.veh_range, a.prop_range, a.range_scale, a.miss_prob, a.seed, a.lower_fov, a.upper_fov, a.lidar, a.range), flush=True)
rclpy.init()
node = GTObjects()
cache.refresh(force=True)
cb = world.on_tick(node.on_tick)
try:
    rclpy.spin(node)
finally:
    world.remove_on_tick(cb)
    node.stats()
