"""11/04 1단계: CARLA 만(Autoware 없이) 차 한 대에 스로틀·브레이크·조향을 계단으로 넣고 실제 움직임을 잰다(호스트 .venv-carla)
목적: 브리지(autoware_carla_interface)의 raw_vehicle_cmd_converter 가 쓰는 accel_map.csv·brake_map.csv(프리우스용)와 같은 꼴의
      차량용 지도를 만들 원자료. 지도 만들기는 s76_make_maps.py(서버 없이)
꼴(브리지 1.9.0 확인): 행 = 페달(accel_cmd 가 그대로 CARLA throttle, brake_cmd 가 그대로 CARLA brake), 열 = 속도 m/s, 값 = 가속도 m/s2
자리: 동결 xodr road 1256 lane -1(곧은 길 123m, 높이 0). s 22 에서 출발, s 136 을 넘으면 출발점으로 옮기고 같은 속도를 다시 넣음(옮긴 뒤 0.5s 는 안 씀)
      앞 6m 를 보는 pure pursuit 로 차로 가운데를 따라감(조향이 작아 가속도에 거의 안 들어감)
단계:
  thr  : 정지 -> throttle T 를 v 14.5m/s 넘거나 속도가 5초 동안 0.05m/s 안 바뀔 때(최소 20초)까지, 최대 90초
  brk  : 정지 -> throttle UP 으로 v 14.2m/s(그리고 4.5m/s, --brk-from) 까지 -> throttle 0·brake B 로 멈출 때까지(최대 80초). B=0 은 타력(지도 0 행)
  steer: 1.5m/s 로 맞춘 뒤 steer S 를 1.2초(속도 유지), 앞 왼·오른 바퀴각, 요레이트 -> 회전 반경 R = v/w, 등가각 atan(L/R)
출력: <out>_ticks.csv(매 틱), <out>_meta.json. 끝나면 차를 지우고 원래 설정으로 되돌림
사용: python s76_pedal_bench.py --bp vehicle.mercedes.sprinter --out <접두어> [--thr 0.1,...] [--brk 0,...] [--steer 0.3,...] [--up 1.0]
"""
import argparse, csv, json, math, time
import carla

ap = argparse.ArgumentParser()
ap.add_argument("--bp", default="vehicle.mercedes.sprinter"); ap.add_argument("--out", required=True)
ap.add_argument("--thr", default="0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0")
ap.add_argument("--brk", default="0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,1.0")
ap.add_argument("--steer", default="0.3,0.5,0.6,0.7,0.8,-0.3,-0.5,-0.6,-0.7,-0.8")
ap.add_argument("--up", type=float, default=1.0, help="brk 단계에서 속도 올릴 때 throttle")
ap.add_argument("--brk-from", default="14.2,7.0,3.0", help="brk 단계 시작 속도들(낮은 속도 칸 표본을 늘리려고 두 번)")
a = ap.parse_args()
fl = lambda s: [float(x) for x in s.split(",")] if s else []

c = carla.Client("localhost", 2000); c.set_timeout(60); w = c.get_world(); m = w.get_map(); lib = w.get_blueprint_library()
orig = w.get_settings(); s = w.get_settings(); s.synchronous_mode = True; s.fixed_delta_seconds = 0.05; w.apply_settings(s)
DT = 0.05; ROAD, S0, S1 = 1256, 22.0, 136.0
START = m.get_waypoint_xodr(ROAD, -1, S0).transform; START.location.z += 0.3
TPT = m.get_waypoint_xodr(ROAD, -1, S0).transform; TPT.location.z += 0.05  # 옮길 때는 낮게(첫 판 0.3m 는 떨어져 튐)
veh = w.spawn_actor(lib.find(a.bp), START)
rows = []; t_sim = 0.0; tp_tick = -999; k = 0; phase = "init"; meta = dict(bp=a.bp, road=ROAD, s0=S0, s1=S1, dt=DT)


def tick():
    global t_sim, k
    w.tick(); t_sim += DT; k += 1


for _ in range(40): veh.apply_control(carla.VehicleControl(brake=1.0)); tick()
pc = veh.get_physics_control(); wh = pc.wheels
# 바퀴 위치는 월드 좌표(cm)라 차 좌표로 바꿔 축거를 구함
inv = veh.get_transform().get_inverse_matrix()
def loc_x(p): return inv[0][0] * p.x / 100 + inv[0][1] * p.y / 100 + inv[0][2] * p.z / 100 + inv[0][3]
L = loc_x(wh[0].position) - loc_x(wh[2].position)
meta.update(mass=pc.mass, wheelbase=round(L, 3), max_steer_deg=[round(x.max_steer_angle, 1) for x in wh],
            steering_curve=[[p.x, p.y] for p in pc.steering_curve], torque_curve=[[p.x, p.y] for p in pc.torque_curve],
            max_rpm=pc.max_rpm, gear_switch_time=pc.gear_switch_time, final_ratio=pc.final_ratio,
            forward_gears=[[g.ratio, g.up_ratio, g.down_ratio] for g in pc.forward_gears], drag=pc.drag_coefficient,
            brake_torque=[x.max_brake_torque for x in wh], tire_friction=[x.tire_friction for x in wh])
print("meta", json.dumps(meta)[:400], flush=True)


def fspeed():
    v = veh.get_velocity(); f = veh.get_transform().get_forward_vector(); return v.x * f.x + v.y * f.y + v.z * f.z


def pp_steer():
    t = veh.get_transform(); loc = t.location
    wp = m.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
    ld = max(5.0, 0.8 * abs(fspeed()) + 4.0); nx = wp.next(ld)
    if not nx: return 0.0, wp
    tg = nx[0].transform.location; yaw = math.radians(t.rotation.yaw)
    al = math.atan2(tg.y - loc.y, tg.x - loc.x) - yaw; al = math.atan2(math.sin(al), math.cos(al))
    return max(-0.6, min(0.6, math.atan2(2 * L * math.sin(al), ld))), wp


def teleport(spd):
    global tp_tick
    veh.set_transform(TPT); veh.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
    f = TPT.get_forward_vector(); veh.set_target_velocity(carla.Vector3D(f.x * spd, f.y * spd, 0.0)); tp_tick = k


def step(thr, brk, steer=None):
    """한 틱: 조향(없으면 pure pursuit) 넣고 틱, 기록. 끝 넘으면 옮김"""
    st, wp = (pp_steer() if steer is None else (steer, m.get_waypoint(veh.get_location(), project_to_road=True)))
    veh.apply_control(carla.VehicleControl(throttle=thr, brake=brk, steer=st)); tick()
    t = veh.get_transform(); ctl = veh.get_control(); av = veh.get_angular_velocity()
    wl = m.get_waypoint(t.location, project_to_road=True, lane_type=carla.LaneType.Driving)
    fl_a = veh.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel); fr_a = veh.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)
    v = fspeed()
    rows.append(dict(k=k, t=round(t_sim, 3), ph=phase, thr=thr, brk=brk, steer=round(st, 4), v=round(v, 4), gear=ctl.gear,
                     pitch=round(t.rotation.pitch, 3), x=round(t.location.x, 3), y=round(t.location.y, 3), z=round(t.location.z, 3),
                     yaw=round(t.rotation.yaw, 3), wz=round(av.z, 3), fl=round(fl_a, 2), fr=round(fr_a, 2), tp=k - tp_tick,
                     road=wl.road_id, s=round(wl.s, 2)))
    if wl.road_id != ROAD or wl.s > S1:
        teleport(max(0.0, v))
    return v


def stop_at_start():
    teleport(0.0)
    for _ in range(30): step(0.0, 1.0)


T0 = time.time()
for T in fl(a.thr):
    phase = f"thr{T:.2f}"; stop_at_start(); hist = []
    for i in range(int(90 / DT)):
        v = step(T, 0.0); hist.append(v)
        if v > 14.5: break
        if i * DT > 20 and abs(hist[-1] - hist[-int(5 / DT)]) < 0.05: break
    print(f"{phase} end v {v:.2f} t {i*DT:.1f}s wall {time.time()-T0:.0f}", flush=True)
for B, VF in [(b, vf) for b in fl(a.brk) for vf in fl(a.brk_from)]:
    phase = f"up{B:.2f}@{VF}"; stop_at_start()
    for i in range(int(120 / DT)):
        v = step(a.up, 0.0)
        if v >= VF: break
    phase = f"brk{B:.2f}@{VF}"; n0 = 0
    for i in range(int(80 / DT)):
        v = step(0.0, B); n0 = n0 + 1 if abs(v) < 0.05 else 0
        if n0 > 10: break
    print(f"{phase} end v {v:.2f} t {i*DT:.1f}s wall {time.time()-T0:.0f}", flush=True)
for S in fl(a.steer):
    phase = f"pre{S:+.2f}"; stop_at_start(); ie = 0.0
    for i in range(int(8 / DT)):  # 1.5m/s 로 맞춤(PI)
        e = 1.5 - fspeed(); ie += e * DT
        thr = max(0.0, min(1.0, 0.25 + 0.4 * e + 0.1 * ie)); brk = 0.3 if e < -0.3 else 0.0
        step(thr, brk)
    phase = f"steer{S:+.2f}"
    for i in range(int(1.2 / DT)):
        e = 1.5 - fspeed(); ie += e * DT
        thr = max(0.0, min(1.0, 0.25 + 0.4 * e + 0.1 * ie)); step(thr, 0.0, steer=S)
    phase = f"post{S:+.2f}"
    for i in range(int(3 / DT)): step(0.15, 0.0)
    print(f"steer {S:+.2f} done wall {time.time()-T0:.0f}", flush=True)

meta["wall_s"] = round(time.time() - T0, 1); meta["ticks"] = len(rows)
with open(a.out + "_ticks.csv", "w", newline="") as f:
    wr = csv.DictWriter(f, fieldnames=list(rows[0].keys())); wr.writeheader(); wr.writerows(rows)
open(a.out + "_meta.json", "w").write(json.dumps(meta, ensure_ascii=False, indent=1))
veh.destroy(); tick(); w.apply_settings(orig)
print("done", len(rows), meta["wall_s"], flush=True)
