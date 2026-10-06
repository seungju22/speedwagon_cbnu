#!/usr/bin/env python3
# 맵세션20 Phase 5. 제동 정지 시험(감속도·제동 작동 지연·정지거리 실측).
# 목적: TTC 임계값 TTC* = t_d + v/(2a) 를 인용값이 아니라 우리 차량(vehicle.audi.a2, test_drive 와 같음)
# 실측값으로 채운다(stops_status_2026-09-29.md "TTC 임계값 유도").
#
# 시험 1회: 직선 road 위 lane -1 에 스폰 -> BasicAgent(test_drive 와 같은 동기 20Hz, 목표 20km/h)로 가속
# -> 속도가 처음 20km/h(5.556m/s) 이상이 된 tick 에 전제동 명령(throttle 0, brake 1.0, steer 0)
# -> 속도 < STOP_MPS 까지 tick 마다 기록.
# 세션20 2차: --hold H 이면 20km/h 첫 도달 뒤 H 초 지난 다음, 속도가 다시 20km/h 이상인 첫 tick 에 제동
# (3차 수정: hold 만 두면 정속 중 속도 진동 4.1~6.3m/s 의 아무 위상에서 걸려 v0 가 4.07m/s 로 나옴)(1차는 가속 직후 제동 = 출발 가속 상태,
# 명령 뒤 0.1~0.15s 속도가 더 오름. 순항 상태 제동이 셔틀 정차·긴급 제동과 같은 조건)
# 모드: brake = brake 1.0 만 / hand = brake 1.0 + hand_brake(test_drive 완주 때 쓰는 제동과 같음)
# 동기 20Hz 는 결정적이라 같은 자리 반복은 같은 값이 나온다(세션17). 편차는 road·출발 위치를 바꿔 낸다.
# 같은 자리 1회 반복은 결정성 확인용.
#
# 정의
#  t_cmd    = 제동 명령을 넣은 tick 의 시각(그 tick 에 apply_control, 다음 world.tick 부터 반영)
#  작동 지연 t_act = t_cmd 부터 명령 뒤 최고 속도 tick 까지(이 동안 속도가 오히려 오름, 감속 시작 전)
#  복귀 시간 t_back = t_cmd 부터 속도가 처음으로 v0 보다 낮아진 tick 까지
#  평균 감속도 a_avg = v0 / (정지 시각 - t_cmd)         (지연 포함, 보수적)
#  제동 감속도 a_brk = v_peak / (정지 시각 - t_peak)    (명령 뒤 최고 속도 지점부터, 지연 제외)
#  정상 감속도 a_fit = 최고 속도 tick ~ 속도 >= FIT_MIN_MPS 마지막 tick 의 v-t 1차 회귀 기울기
#                     (세션20 첫 실행에서 마지막 1 tick 에 약 1.1m/s -> 0 으로 떨어지는 저속 정지 처리가 보여
#                      순간 최대 22m/s^2 가 나옴 -> 순간 최대는 버리고 이 값을 씀)
#  정지거리 d_stop = 명령 위치부터 정지 위치까지 tick 위치 누적
# 실행: python map/tests/test_brake.py            (서버 필요, 맵 로드 상태)
# 근거: https://carla.readthedocs.io/en/0.9.15/python_api/#carlavehiclecontrol
#       https://carla.readthedocs.io/en/0.9.15/adv_synchrony_timestep/
import csv
import math
import sys
import time

import carla

sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent))
import test_drive as td  # noqa: E402  (상수·동기 모드·follow 함수 재사용, main 은 실행 안 됨)

TRIGGER_MPS = 20.0 / 3.6
FIT_MIN_MPS = 1.0
STOP_MPS = 0.05
MAX_BRAKE_S = 10.0
MAX_ACCEL_S = 20.0
# (road, 출발 s, 모드). 직선성: r1172 yaw 편차 0.12°, r1125 0.0°, r1355 0.2° (세션20 오프라인 확인)
TRIALS = [(1172, 5.0, "brake"), (1172, 5.0, "brake"), (1172, 40.0, "brake"),
          (1125, 3.0, "brake"), (1355, 3.0, "brake"),
          (1172, 5.0, "hand"), (1125, 3.0, "hand"), (1355, 3.0, "hand")]
FIELDS = ["trial", "road", "start_s", "mode", "phase", "t", "frame", "x", "y",
          "speed_mps", "throttle", "brake", "hand_brake"]


def run_trial(world, cmap, bp, k, road, s0, mode, writer, spectator, hold=0.0):
    from agents.navigation.basic_agent import BasicAgent
    from agents.navigation.local_planner import RoadOption
    L = td.read_road_lengths()[road]
    wp0 = cmap.get_waypoint_xodr(road, td.LANE_ID, s0)
    tf = wp0.transform
    tf.location.z += 0.5
    v = world.spawn_actor(bp, tf)
    try:
        plan, s = [], s0 + 2.0
        while s < L - 0.5:
            plan.append((cmap.get_waypoint_xodr(road, td.LANE_ID, s), RoadOption.LANEFOLLOW))
            s += 2.0
        agent = BasicAgent(v, target_speed=td.DESIRED_SPEED_KMH, map_inst=cmap,
                           opt_dict={"ignore_traffic_lights": True, "ignore_stop_signs": True,
                                     "ignore_vehicles": True, "dt": td.FIXED_DELTA_S})
        agent.set_global_plan(plan, stop_waypoint_creation=True, clean_queue=True)
        for _ in range(20):            # 1s 낙하·안착(제동)
            v.apply_control(carla.VehicleControl(brake=1.0))
            world.tick()
        t0 = world.get_snapshot().timestamp.elapsed_seconds
        phase, rows, cmd, t_reach = "accel", [], None, None
        prev = v.get_transform().location
        dist = 0.0
        while True:
            snap = world.get_snapshot()
            t = snap.timestamp.elapsed_seconds - t0
            loc = v.get_transform().location
            vel = v.get_velocity()
            spd = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
            if spectator is not None:
                spectator.set_transform(td.follow_transform(v.get_transform()))
            if phase == "brake":
                dist += loc.distance(prev)
            prev = loc
            if t_reach is None and spd >= TRIGGER_MPS:
                t_reach = t
            if phase == "accel" and t_reach is not None and t - t_reach >= hold \
                    and (not hold or spd >= TRIGGER_MPS):
                phase = "brake"
                cmd = {"t": t, "v0": spd, "x": loc.x, "y": loc.y,
                       "s": cmap.get_waypoint(loc).s, "road": cmap.get_waypoint(loc).road_id}
            if phase == "accel":
                if t > MAX_ACCEL_S or agent.done():
                    return {"ok": False, "why": f"20km/h 미도달(t={t:.1f}s, {spd:.2f}m/s)"}
                c = agent.run_step()
            else:
                c = carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0,
                                         hand_brake=(mode == "hand"))
            row = {"trial": k, "road": road, "start_s": s0, "mode": mode, "phase": phase,
                   "t": round(t, 3), "frame": snap.frame, "x": round(loc.x, 3),
                   "y": round(loc.y, 3), "speed_mps": round(spd, 4),
                   "throttle": round(c.throttle, 3), "brake": round(c.brake, 3),
                   "hand_brake": c.hand_brake}
            writer.writerow(row)
            rows.append(row)
            if phase == "brake" and (spd < STOP_MPS or t - cmd["t"] > MAX_BRAKE_S):
                break
            v.apply_control(c)
            world.tick()
        br = [r for r in rows if r["phase"] == "brake"]
        t_stop = br[-1]["t"]
        v0 = cmd["v0"]
        drop = next((r for r in br[1:] if r["speed_mps"] < br[0]["speed_mps"]), None)
        pi = max(range(len(br)), key=lambda i: br[i]["speed_mps"])
        peak = br[pi]
        d_peak = t_stop - peak["t"]
        seg = [r for r in br[pi:] if r["speed_mps"] >= FIT_MIN_MPS]
        if len(seg) >= 2:
            mt = sum(r["t"] for r in seg) / len(seg)
            mv = sum(r["speed_mps"] for r in seg) / len(seg)
            a_fit = -sum((r["t"] - mt) * (r["speed_mps"] - mv) for r in seg) / \
                sum((r["t"] - mt) ** 2 for r in seg)
        else:
            a_fit = float("nan")
        return {"ok": br[-1]["speed_mps"] < STOP_MPS, "v0": v0, "cmd": cmd,
                "t_act": peak["t"] - cmd["t"],
                "t_back": (drop["t"] - cmd["t"]) if drop else float("nan"),
                "a_fit": a_fit,
                "v_peak": peak["speed_mps"], "t_brake": t_stop - cmd["t"],
                "a_avg": v0 / (t_stop - cmd["t"]),
                "a_brk": peak["speed_mps"] / d_peak if d_peak > 0 else float("nan"),
                "d_stop": dist,
                "final": br[-1]["speed_mps"]}
    finally:
        v.destroy()
        world.tick()


def main():
    import argparse
    ap = argparse.ArgumentParser(description="제동 정지 시험(세션20)")
    ap.add_argument("--hold", type=float, default=0.0,
                    help="20km/h 첫 도달 뒤 정속 유지 시간 s(기본 0 = 도달 즉시 제동)")
    hold = ap.parse_args().hold
    print(f"map_sha256={td.file_sha16()} ({td.XODR_PATH.name})")
    print(f"hold={hold:g}s (20km/h 첫 도달 뒤 이 시간 뒤 제동)")
    if str(td.AGENTS_PATH) not in sys.path:
        sys.path.insert(0, str(td.AGENTS_PATH))
    client = carla.Client(td.HOST, td.PORT)
    client.set_timeout(td.TIMEOUT)
    world = client.get_world()
    cmap = world.get_map()
    import hashlib
    srv16 = hashlib.sha256(cmap.to_opendrive().encode("utf-8")).hexdigest()[:16]
    print(f"server_map_sha256={srv16} ({'파일과 일치' if srv16 == td.file_sha16() else '불일치'})")
    removed = td.cleanup_stray_vehicles(world)
    if removed:
        print(f"기존 차량 액터 제거: {removed}")
    bp = world.get_blueprint_library().find("vehicle.audi.a2")
    ts = time.strftime("%Y%m%d_%H%M%S")
    path = td.LOG_DIR / f"brake_test_{ts}{f'_hold{hold:g}' if hold else ''}.csv"
    original = world.get_settings()
    results = []
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        try:
            td.set_sync(world, original)
            world.tick()
            spectator = world.get_spectator()
            for k, (road, s0, mode) in enumerate(TRIALS):
                r = run_trial(world, cmap, bp, k, road, s0, mode, w, spectator, hold)
                f.flush()
                results.append((k, road, s0, mode, r))
                if not r["ok"] and "v0" not in r:
                    print(f"#{k} r{road} s={s0} {mode}: 실패 {r['why']}")
                    continue
                print(f"#{k} r{road} s={s0:g} {mode}: v0 {r['v0']:.3f}m/s 작동지연 {r['t_act']:.2f}s "
                      f"복귀 {r['t_back']:.2f}s 최고 {r['v_peak']:.3f} 정지 {r['t_brake']:.2f}s "
                      f"{r['d_stop']:.2f}m a_avg {r['a_avg']:.2f} a_brk {r['a_brk']:.2f} "
                      f"a_fit {r['a_fit']:.2f} "
                      f"{'정지' if r['ok'] else '미정지'}")
        finally:
            try:
                world.tick()
            except Exception as e:  # noqa: BLE001
                print(f"[동기모드] 복귀 전 tick 실패: {e}")
            td.restore_settings(world, original)
    print(f"로그: {path}")
    for mode in ("brake", "hand"):
        ok = [r for _, _, _, m, r in results if m == mode and r.get("ok")]
        if not ok:
            continue
        def stat(key):
            xs = [r[key] for r in ok]
            mu = sum(xs) / len(xs)
            sd = math.sqrt(sum((x - mu) ** 2 for x in xs) / (len(xs) - 1)) if len(xs) > 1 else 0.0
            return f"{mu:.3f}±{sd:.3f}(최소 {min(xs):.3f}, 최대 {max(xs):.3f})"
        print(f"[{mode}] n={len(ok)} 작동지연 {stat('t_act')} 복귀 {stat('t_back')} "
              f"정지시간 {stat('t_brake')} 정지거리 {stat('d_stop')} a_avg {stat('a_avg')} "
              f"a_brk {stat('a_brk')} a_fit {stat('a_fit')}")


if __name__ == "__main__":
    main()
