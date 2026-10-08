#!/usr/bin/env python3
# 맵세션38 2부(B1): 건물·나무·정류장·덩어리 소품 소환 시험 + 셔틀 후보 차량 제원. 결정은 사용자, 여기서는 측정만
# 전제: 서버가 v2 캠퍼스(5240ca8b883e42c0)를 이미 불러온 상태. 월드 재로딩 안 함(띄운 액터가 사라진다)
# 순서: 월드 확인 -> 남은 차량 정리 -> bbox 탐색(먼 곳 소환 후 지움) -> 소환 전 스크린샷·FPS·메모리(렌더 켬/끔) -> 장면 소환(겹침 검사 먼저)
#       -> 소환 후 스크린샷·FPS·메모리 -> 장면 지움 -> LiDAR(소품 하나씩 같은 자리, 10m·20m) -> 차량 제원 -> 전부 destroy·설정 복원
# 위치: r1392(4 노선 + 후문 공통, 길이 56.1m, 방향 변화 0.2도) 오른쪽(인도 쪽), 차도 가장자리에서 5m 이상 바깥
# 겹침 검사(코드): 발자국 위 1m 격자 점마다 (1) project_to_road=False 로 어떤 차선(Any)도 안 잡힐 것
#   (2) 가장 가까운 주행 차선 중심까지 거리 - 반차선폭 >= 5.0m. 노선 차도뿐 아니라 맵 전체 주행 차선 기준(더 엄격)
# 사용: python map/scripts/s38_spawn_trial.py
import hashlib
import json
import math
import queue
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import carla

HOST, PORT, TIMEOUT = "localhost", 2000, 30.0
V2_SHA16 = "5240ca8b883e42c0"
ROOT = Path(__file__).resolve().parents[1]
LOGS = ROOT / "docs" / "logs"
FIGS = ROOT / "docs" / "figures"
ROAD, LANE = 1392, -1
HALF_LANE = 3.35 / 2
EDGE_GAP = 5.0                      # 차도 가장자리에서 띄울 거리(지시문)
PLACE_GAP = 5.4                     # 실제 배치 여유(검사 5.0 + 0.4)
ROW_GAP = 6.0                       # 소품 사이 길이 방향 간격
DT = 0.05
FPS_TICKS = 100
MIN_AVAIL_MI = 2048
BLD = "/Game/Carla/Static/Building/{0}.{1}"
TREE = "/Game/Carla/Static/Vegetation/Trees/{0}.{0}"
# (이름, 종류, 블루프린트, mesh_path, scale) 세션4·15 목록. 덩어리는 Content 에서 찾은 GroundCube(정육면체 이름)
PROPS = [
    ("House01_x1", "building", "static.prop.mesh", BLD.format("SM_House01", "SM_House01"), 1.0),
    ("House01_x2", "building", "static.prop.mesh", BLD.format("SM_House01", "SM_House01"), 2.0),
    ("House02_x1", "building", "static.prop.mesh", BLD.format("SM_House02", "SM_House02"), 1.0),
    ("Apartment03_x1", "building", "static.prop.mesh",
     BLD.format("Apartment03/SM_Apartment03_v01_merged", "SM_Apartment03_v01_merged"), 1.0),
    ("Acer_02", "tree", "static.prop.mesh", TREE.format("SM_Acer_02"), 1.0),
    ("Ash_01", "tree", "static.prop.mesh", TREE.format("SM_Ash_01"), 1.0),
    ("busstop", "busstop", "static.prop.busstop", None, None),
    ("GroundCube", "block", "static.prop.mesh", "/Game/Carla/Static/Ground/GroundCube.GroundCube", 1.0),
]
VEHICLES = ["vehicle.mitsubishi.fusorosa", "vehicle.mercedes.sprinter", "vehicle.toyota.prius", "vehicle.audi.a2"]
# 브리지 sensor_mapping.yaml 을 이 PC 에서 못 찾음 -> 지시문 기본값
LIDAR = dict(channels="64", points_per_second="300000", rotation_frequency="20", range="100")
LIDAR_Z = 2.0                       # 차량 원점 위 2.0m(가정, 브리지 값 미확인)
OUT = {}


def log(*a):
    s = " ".join(str(x) for x in a)
    print(s, flush=True)
    with open(LOGS / "s38_spawn_trial.log", "a") as f:
        f.write(s + "\n")


def mem():
    m = {k: int(v.split()[0]) // 1024 for k, v in
         (l.split(":", 1) for l in open("/proc/meminfo")) if k in ("MemAvailable", "SwapTotal", "SwapFree")}
    gtt = None
    for p in ("/sys/class/drm/card0/device/mem_info_gtt_used", "/sys/class/drm/card1/device/mem_info_gtt_used"):
        try:
            gtt = round(int(open(p).read()) / 2 ** 20)
            break
        except OSError:
            pass
    return {"avail_mi": m["MemAvailable"], "swap_mi": m["SwapTotal"] - m["SwapFree"], "gtt_mi": gtt}


def guard(tag):
    m = mem()
    log(f"[메모리] {tag} {time.strftime('%H:%M:%S')} {m}")
    if m["avail_mi"] < MIN_AVAIL_MI or m["swap_mi"] > 4096:
        raise RuntimeError(f"중단 기준 도달 {m}")
    return m


def lane_point(cmap, s):
    """r1392 lane -1 중심. s 가 road 밖이면 끝점에서 직선 연장(직선 road)."""
    L = OUT["road_len"]
    sc = min(max(s, 0.05), L - 0.05)
    wp = cmap.get_waypoint_xodr(ROAD, LANE, sc)
    f = wp.transform.get_forward_vector()
    loc = wp.transform.location
    d = s - sc
    return carla.Location(loc.x + f.x * d, loc.y + f.y * d, loc.z), wp.transform.rotation.yaw, f, \
        wp.transform.get_right_vector()


# bbox 는 축·크기를 믿지 않는다(세션15 House02 0.33m, 세션38 House01 x 축 0.71m 인데 화면상 집 한 채).
# 발자국 = 정사각형, 반폭 h = max(bbox x, bbox y, 종류별 최소). 축이 틀려도 안전한 쪽
MIN_HALF = {"building": 8.0, "tree": 3.0, "busstop": 1.5, "block": 1.0}
NEAR_OFF = HALF_LANE + PLACE_GAP    # 차선 중심에서 앞면까지(= 차도 가장자리 + 5.4m)
SOLO_S = 28.0                       # LiDAR 단독 시험 자리(r1392 가운데)


def square(c, yaw, h):
    r = math.radians(yaw)
    f, g = (math.cos(r), math.sin(r)), (-math.sin(r), math.cos(r))
    return [carla.Location(c.x + f[0] * sx * h + g[0] * sy * h, c.y + f[1] * sx * h + g[1] * sy * h, c.z)
            for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1))]


def grid(poly, step=1.0):
    pts = []
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        k = max(1, int(math.dist((a.x, a.y), (b.x, b.y)) / step))
        pts += [(a.x + (b.x - a.x) * j / k, a.y + (b.y - a.y) * j / k) for j in range(k)]
    # 안쪽 격자(두 변 벡터로)
    o, u, v = poly[0], poly[1], poly[-1]
    lu, lv = math.dist((o.x, o.y), (u.x, u.y)), math.dist((o.x, o.y), (v.x, v.y))
    for i in range(1, int(lu / step)):
        for j in range(1, int(lv / step)):
            pts.append((o.x + (u.x - o.x) * i * step / lu + (v.x - o.x) * j * step / lv,
                        o.y + (u.y - o.y) * i * step / lu + (v.y - o.y) * j * step / lv))
    return pts


def overlap_check(cmap, pts):
    """(겹친 점 수, 주행 차선 가장자리까지 최소 거리 m, 그 road)."""
    hit = 0
    best = (1e9, None)
    for x, y in pts:
        loc = carla.Location(x, y, 0.5)
        if cmap.get_waypoint(loc, project_to_road=False, lane_type=carla.LaneType.Any) is not None:
            hit += 1
        wp = cmap.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
        d = math.dist((x, y), (wp.transform.location.x, wp.transform.location.y)) - wp.lane_width / 2
        if d < best[0]:
            best = (d, wp.road_id)
    return hit, round(best[0], 3), best[1]


def tick_fps(world, tag):
    world.tick()
    t0 = time.perf_counter()
    for _ in range(FPS_TICKS):
        world.tick()
    dt = time.perf_counter() - t0
    r = {"ticks": FPS_TICKS, "sec": round(dt, 3), "ms_per_tick": round(dt / FPS_TICKS * 1000, 2),
         "fps": round(FPS_TICKS / dt, 1)}
    log(f"[FPS] {tag} {r}")
    return r


def set_render(world, on):
    s = world.get_settings()
    s.no_rendering_mode = not on
    world.apply_settings(s)
    for _ in range(5):
        world.tick()


def measure(world, tag):
    res = {}
    for on in (True, False):
        set_render(world, on)
        time.sleep(3)
        k = f"{tag}_render_{'on' if on else 'off'}"
        res[k] = {"fps": tick_fps(world, k), "mem": guard(k)}
    set_render(world, True)
    return res


def screenshot(world, tf, name):
    bp = world.get_blueprint_library().find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", "1280")
    bp.set_attribute("image_size_y", "720")
    bp.set_attribute("fov", "90")
    cam = world.spawn_actor(bp, tf)
    q = queue.Queue()
    cam.listen(q.put)
    img = None
    try:
        for _ in range(12):             # 노출·스트리밍 안정
            world.tick()
            img = q.get(timeout=10)
        path = FIGS / f"s38_props_{name}.png"
        img.save_to_disk(str(path))
        log(f"[스크린샷] {path.name} frame={img.frame} cam={tf}")
    finally:
        cam.stop()
        cam.destroy()
        world.tick()


def probe(world, bp, kind):
    """먼 곳(r1392 오른쪽 80m)에 띄워 bbox 를 읽고 지운다. static.prop.mesh 는 set_transform 으로 안 움직여서(세션38 2차 시도)
    최종 자리에는 다시 소환한다."""
    c, yaw, f, rv = lane_point(world.get_map(), SOLO_S)
    a = world.spawn_actor(bp, carla.Transform(carla.Location(c.x + rv.x * 80, c.y + rv.y * 80, c.z), carla.Rotation(yaw=yaw)))
    world.tick()
    bb = a.bounding_box
    info = dict(actor_id_probe=a.id, attr_scale=a.attributes.get("scale"),
                bbox_extent_m=[round(bb.extent.x, 3), round(bb.extent.y, 3), round(bb.extent.z, 3)],
                bbox_size_m=[round(2 * bb.extent.x, 3), round(2 * bb.extent.y, 3), round(2 * bb.extent.z, 3)],
                bbox_center_offset_m=[round(bb.location.x, 3), round(bb.location.y, 3), round(bb.location.z, 3)])
    h = max(bb.extent.x, bb.extent.y, MIN_HALF[kind])
    info["footprint_half_m"] = round(h, 3)
    info["footprint_from"] = "bbox" if h > MIN_HALF[kind] else f"최소값 {MIN_HALF[kind]}m"
    a.destroy()
    world.tick()
    return info, h


def place(world, cmap, bp, s_center, near_off, h):
    """정사각형 발자국 앞면이 차선 중심에서 near_off 에 오게. 겹침 검사 통과하면 소환. (actor 또는 None, 검사 결과)"""
    c, yaw, f, rv = lane_point(cmap, s_center)
    off = near_off + h
    loc = carla.Location(c.x + rv.x * off, c.y + rv.y * off, c.z)
    pts = grid(square(loc, yaw, h))
    hit, dmin, droad = overlap_check(cmap, pts)
    chk = dict(s_center=round(s_center, 2), offset_from_lane_center_m=round(off, 2), check_points=len(pts),
               points_on_any_lane=hit, min_dist_to_driving_edge_m=dmin, nearest_driving_road=droad,
               overlap_ok=hit == 0 and dmin >= EDGE_GAP)
    if not chk["overlap_ok"]:
        return None, chk, loc, yaw
    try:
        a = world.spawn_actor(bp, carla.Transform(loc, carla.Rotation(yaw=yaw)))
    except RuntimeError as e:
        chk["spawn_error"] = str(e)
        return None, chk, loc, yaw
    world.tick()
    tl = a.get_transform().location
    chk["placed_err_m"] = round(math.dist((tl.x, tl.y), (loc.x, loc.y)), 3)
    return a, chk, loc, yaw


def lidar_solo(world, cmap, bp, h, dist):
    """소품 하나만 SOLO_S 에 띄우고, 발자국 최근접점까지 dist 인 lane -1 위(뒤쪽)에 a2 + semantic LiDAR, 한 바퀴."""
    a, chk, loc, yaw = place(world, cmap, bp, SOLO_S, NEAR_OFF, h)
    if a is None:
        return {"dist": dist, "error": "소환·검사 실패", **chk}
    gp = grid(square(loc, yaw, h), 0.5)

    def near(s):
        p = lane_point(cmap, s)[0]
        return min(math.dist((p.x, p.y), q) for q in gp)
    veh = lid = None
    try:
        lo, hi = SOLO_S, SOLO_S - 80.0
        if near(lo) > dist:
            return {"dist": dist, "feasible": False, "nearest_m": round(near(lo), 2)}
        for _ in range(40):
            m = (lo + hi) / 2
            lo, hi = (m, hi) if near(m) < dist else (lo, m)
        s = (lo + hi) / 2
        p, vyaw, _, _ = lane_point(cmap, s)
        veh = world.spawn_actor(world.get_blueprint_library().find("vehicle.audi.a2"),
                                carla.Transform(carla.Location(p.x, p.y, p.z + 0.3), carla.Rotation(yaw=vyaw)))
        veh.set_simulate_physics(False)
        lbp = world.get_blueprint_library().find("sensor.lidar.ray_cast_semantic")
        for k, v in LIDAR.items():
            lbp.set_attribute(k, v)
        lid = world.spawn_actor(lbp, carla.Transform(carla.Location(z=LIDAR_Z)), attach_to=veh)
        q = queue.Queue()
        lid.listen(q.put)
        for _ in range(3):
            world.tick()
            meas = q.get(timeout=10)
        counts = {}
        for d in meas:
            counts[d.object_idx] = counts.get(d.object_idx, 0) + 1
        lp = lid.get_transform().location
        top = sorted(counts.items(), key=lambda kv: -kv[1])[:4]
        return {"dist": dist, "feasible": True, "veh_s": round(s, 2),
                "lidar_to_footprint_m": round(min(math.dist((lp.x, lp.y), q) for q in gp), 2),
                "points_total": len(meas), "target_actor_id": a.id, "target_hits": counts.get(a.id, 0),
                "top_object_idx": top, "frame": meas.frame}
    finally:
        if lid is not None:
            lid.stop()
            lid.destroy()
        if veh is not None:
            veh.destroy()
        a.destroy()
        world.tick()


def vehicle_specs(world, cmap):
    lib = world.get_blueprint_library()
    out = {}
    loc, yaw, _, _ = lane_point(cmap, 28.0)
    for vid in VEHICLES:
        try:
            bp = lib.find(vid)
        except (IndexError, RuntimeError) as e:
            out[vid] = {"error": f"블루프린트 없음: {e}"}
            log(f"[차량] {vid} 없음 {e}")
            continue
        v = world.try_spawn_actor(bp, carla.Transform(carla.Location(loc.x, loc.y, loc.z + 0.5), carla.Rotation(yaw=yaw)))
        if v is None:
            out[vid] = {"error": "try_spawn_actor None"}
            log(f"[차량] {vid} 소환 실패")
            continue
        try:
            for _ in range(10):
                world.tick()
            e = v.bounding_box.extent
            pc = v.get_physics_control()
            vt = v.get_transform()
            ws = pc.wheels
            # WheelPhysicsControl.position 은 세계 좌표 cm(0.9.15 문서). 차량 위치(m)와 비교해 단위를 확인한다
            wl = [(w.position.x / 100, w.position.y / 100, w.position.z / 100) for w in ws]
            unit_ok = max(math.dist((p[0], p[1]), (vt.location.x, vt.location.y)) for p in wl) < 10
            fwd = vt.get_forward_vector()

            def along(p):
                return (p[0] - vt.location.x) * fwd.x + (p[1] - vt.location.y) * fwd.y
            a = sorted(range(len(wl)), key=lambda i: along(wl[i]))
            rear, front = a[:2], a[-2:]
            fm = [sum(wl[i][k] for i in front) / 2 for k in range(2)]
            rm = [sum(wl[i][k] for i in rear) / 2 for k in range(2)]
            wb = math.dist(fm, rm)
            st = max(ws[i].max_steer_angle for i in front)
            r = {"bbox_len_m": round(2 * e.x, 3), "bbox_wid_m": round(2 * e.y, 3), "bbox_hgt_m": round(2 * e.z, 3),
                 "n_wheels": len(ws), "wheel_pos_unit_cm_to_m_ok": unit_ok,
                 "wheelbase_m": round(wb, 3), "front_max_steer_deg": [round(ws[i].max_steer_angle, 2) for i in front],
                 "rear_max_steer_deg": [round(ws[i].max_steer_angle, 2) for i in rear],
                 "min_turn_radius_m_bicycle": round(wb / math.tan(math.radians(st)), 3) if st > 0 else None}
            out[vid] = r
            log(f"[차량] {vid} {r}")
        finally:
            v.destroy()
            world.tick()
    return out


def main():
    (LOGS / "s38_spawn_trial.log").write_text("")
    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)
    world = client.get_world()
    cmap = world.get_map()
    sha = hashlib.sha256(cmap.to_opendrive().encode()).hexdigest()[:16]
    log(f"[월드] map={cmap.name} sha16={sha}")
    if sha != V2_SHA16:
        raise SystemExit("v2 캠퍼스 아님 - 중단(재로딩은 하지 않는다)")
    orig = world.get_settings()
    OUT["settings_orig"] = str(orig)
    log(f"[설정] 원래 {orig}")
    stray = [(a.id, a.type_id) for a in world.get_actors()
             if a.type_id.startswith(("vehicle.", "static.prop.", "sensor."))]
    log(f"[정리 전] 남은 차량·소품·센서 {stray}")
    for a in world.get_actors():
        if a.type_id.startswith(("vehicle.", "static.prop.")):
            a.destroy()
    # 1차 시도 교훈: get_waypoint_xodr 는 road 밖 s 에서 예외 대신 None -> 길이는 xodr 에서 읽는다
    OUT["road_len"] = next(float(r.get("length")) for r in ET.fromstring(cmap.to_opendrive()).iter("road")
                           if r.get("id") == str(ROAD))
    log(f"[도로] r{ROAD} 길이 {OUT['road_len']:.3f}m")
    scene = {}
    try:
        s = world.get_settings()
        s.synchronous_mode = True
        s.fixed_delta_seconds = DT
        world.apply_settings(s)
        world.tick()
        guard("동기 전환 후")
        lib = world.get_blueprint_library()
        # 블루프린트 준비 + bbox 탐색(먼 곳에 띄웠다 지움)
        bps, OUT["props"] = {}, {}
        for name, kind, bpid, path, scale in PROPS:
            rec = {"kind": kind, "blueprint": bpid, "mesh_path": path, "scale": scale}
            OUT["props"][name] = rec
            try:
                bp = lib.find(bpid)
                if path:
                    bp.set_attribute("mesh_path", path)
                if scale is not None:
                    bp.set_attribute("scale", str(scale))
                info, h = probe(world, bp, kind)
            except (IndexError, RuntimeError) as e:
                rec["spawn"] = f"실패: {e}"
                log(f"[소환] {name} 실패 {e}")
                continue
            rec.update(info, spawn="성공")
            bps[name] = (bp, h, kind)
            log(f"[bbox] {name} {info}")

        # 시점: r1392 s=-10, 길 왼쪽 10m, 높이 18m 에서 오른쪽 앞(소품 줄)을 본다
        c, yaw, f, rv = lane_point(cmap, -10.0)
        cam_tf = carla.Transform(carla.Location(c.x - rv.x * 10, c.y - rv.y * 10, c.z + 18.0),
                                 carla.Rotation(pitch=-25.0, yaw=yaw + 35.0))
        OUT["cam_tf"] = str(cam_tf)
        screenshot(world, cam_tf, "before")
        OUT["before"] = measure(world, "before")

        # 장면: 앞줄(나무·정류장·덩어리·작은 집), 뒷줄(큰 건물). 앞줄 깊이 + 6m 뒤에 뒷줄
        front = [n for n in ("Acer_02", "Ash_01", "busstop", "GroundCube", "House02_x1") if n in bps]
        back = [n for n in ("House01_x1", "House01_x2", "Apartment03_x1") if n in bps]
        depth = max((2 * bps[n][1] for n in front), default=0.0)
        for row, names, near_off in (("front", front, NEAR_OFF), ("back", back, NEAR_OFF + depth + ROW_GAP)):
            cursor = 0.0
            for n in names:
                bp, h, kind = bps[n]
                a, chk, _, _ = place(world, cmap, bp, cursor + h, near_off, h)
                OUT["props"][n]["scene"] = dict(chk, row=row, actor_id=a.id if a else None)
                log(f"[장면] {n} {OUT['props'][n]['scene']}")
                if a is not None:
                    scene[n] = a
                cursor += 2 * h + ROW_GAP
        time.sleep(5)
        guard("소환 5s 후")
        screenshot(world, cam_tf, "after")
        OUT["after"] = measure(world, "after")
        for n, a in scene.items():
            a.destroy()
        world.tick()
        scene = {}
        time.sleep(3)
        OUT["mem_scene_cleared"] = guard("장면 지운 뒤")

        OUT["lidar_attr"] = dict(LIDAR, z_m=LIDAR_Z, upper_fov="10.0(기본)", lower_fov="-30.0(기본)",
                                 source="sensor_mapping.yaml 못 찾음 -> 지시문 기본값", mount="vehicle.audi.a2 원점 위")
        OUT["lidar"] = {}
        for n, (bp, h, kind) in bps.items():
            OUT["lidar"][n] = [lidar_solo(world, cmap, bp, h, d) for d in (10.0, 20.0)]
            log(f"[LiDAR] {n} {OUT['lidar'][n]}")
        OUT["vehicles"] = vehicle_specs(world, cmap)
    finally:
        for a in scene.values():
            try:
                a.destroy()
            except RuntimeError:
                pass
        done = {a.id for a in scene.values()}
        for a in world.get_actors():
            if a.id not in done and a.type_id.startswith(("vehicle.", "static.prop.", "sensor.")):
                a.destroy()
        if world.get_settings().synchronous_mode:
            world.tick()
        # 원래 값 그대로 apply 하지 않는다: fixed_delta_seconds 가 미설정(쓰레기값 표시)이라 None 으로 명시
        s = world.get_settings()
        s.synchronous_mode = False
        s.no_rendering_mode = False
        s.fixed_delta_seconds = None
        world.apply_settings(s)
        time.sleep(2)
        left = [(a.id, a.type_id) for a in world.get_actors()
                if a.type_id.startswith(("vehicle.", "static.prop.", "sensor."))]
        OUT["left_after_cleanup"] = left
        OUT["settings_after"] = str(world.get_settings())
        log(f"[정리] 남은 차량·소품·센서 {left} 설정 {world.get_settings()}")
        OUT["mem_end"] = mem()
        log(f"[메모리] 정리 후 {OUT['mem_end']}")
        (LOGS / "s38_spawn_trial.json").write_text(json.dumps(OUT, ensure_ascii=False, indent=1, default=str))


if __name__ == "__main__":
    main()
