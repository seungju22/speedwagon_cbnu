"""시나리오 보행자 도구(세션58). 노선 파일의 승강장(문 자리·문 구역·전방 통로) 기준으로 보행자를 만들고, 걷게 하고, 지운다.
브리지가 동기 모드로 틱을 돌리는 중에 따로 띄운다(wait_for_tick 으로 따라감). 서버만 켜져 있으면 혼자도 돈다(그때는 비동기).
- 탑승: 문 앞까지 걸어와 지운다(board)  - 하차: 문 앞에 만들고 인도로 걷는다(alight)
- 위치 이름(승강장마다 계산, 모두 지도 좌표. CARLA 는 (x, -y)):
  door_out     문 가운데(door_pose)에서 차 바깥쪽 0.45m (보행자 중심)
  dz_center    문 구역 다각형 무게중심
  walk_back    dz_center 에서 차 뒤쪽으로 6m (인도 띠를 따라)
  walk_fwd     dz_center 에서 차 앞쪽으로 6m
  corr_center  전방 통로 무게중심 (차로 안)
  corr_side    corr_center 에서 바깥쪽 3.0m (인도 쪽)
  [x, y]       지도 좌표를 바로 줘도 된다
- 단계(YAML 의 steps 또는 --steps "a;b;c"):
  wait_ego_stopped [still_s]     자차(role ego_vehicle)가 stop_zone 안에서 still_s 동안 0.05m/s 밑일 때까지
  wait_ego_near <m>              자차 앞 범퍼가 문 자리에서 m 안
  spawn <id> <위치>              보행자 만들기(청사진은 시드로 고정해 고름)
  walk <id> <위치> [speed]       그 위치까지 걷기(0.3m 안이면 끝)
  wait <s>                       시뮬레이터 시간 s 초
  destroy <id>
- 기록: --log CSV, 매 틱(0.05s) 보행자 위치·문 구역/전방 통로 안 여부·단계
사용: python ped_tool.py --platforms map/data/shuttle_platforms_v31_r1.yaml --platform lib_north (scenario.yaml | --steps "...") [--seed 0] [--log out.csv]
"""
import argparse, math, random, sys, time
from pathlib import Path
import json, subprocess
import carla
try:
    import yaml
    def load_yaml(path): return yaml.safe_load(open(path))
except ImportError:      # .venv-carla 에는 PyYAML 이 없다. 시스템 python3 로 읽는다(pip install 안 함)
    def load_yaml(path):
        return json.loads(subprocess.check_output(["/usr/bin/python3", "-c",
            "import sys,json,yaml;print(json.dumps(yaml.safe_load(open(sys.argv[1]))))", path]))

ap = argparse.ArgumentParser()
ap.add_argument("scenario", nargs="?")
ap.add_argument("--platforms", default=str(Path.home() / "campus_mobility_sim/map/data/shuttle_platforms_v31_r1.yaml"))
ap.add_argument("--platform")
ap.add_argument("--steps", default="")
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--log", default="")
ap.add_argument("--timeout", type=float, default=600.0, help="실제 시간 초")
ap.add_argument("--dry", action="store_true", help="서버 없이 위치 이름만 계산해 찍는다")
a = ap.parse_args()

sc = load_yaml(a.scenario) if a.scenario else {}
plat_id = a.platform or sc.get("platform")
seed = sc.get("seed", a.seed)
steps = sc.get("steps") or [s.strip() for s in a.steps.split(";") if s.strip()]
P = next(p for p in load_yaml(a.platforms)["platforms"] if p["id"] == plat_id)
rng = random.Random(seed)
PED_BPS = ["walker.pedestrian.0001", "walker.pedestrian.0002", "walker.pedestrian.0003", "walker.pedestrian.0005",
           "walker.pedestrian.0006", "walker.pedestrian.0007", "walker.pedestrian.0008", "walker.pedestrian.0015"]


def centroid(poly):
    xs, ys = zip(*poly); return sum(xs) / len(xs), sum(ys) / len(ys)


def in_poly(x, y, poly):
    n, ins = len(poly), False
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1: ins = not ins
    return ins


yaw = math.radians(P["door_pose"]["yaw_deg"])
fx, fy = math.cos(yaw), math.sin(yaw)          # 지도 좌표(반시계) 차 앞
rx, ry = fy, -fx                               # 차 오른쪽(문 쪽)
dx, dy = P["door_pose"]["x"], P["door_pose"]["y"]
dzc = centroid(P["door_zone"]) if P["door_zone"] else (dx + rx * 0.8, dy + ry * 0.8)
cc = centroid(P["front_corridor"])
POS = {"door_out": (dx + rx * 0.45, dy + ry * 0.45), "dz_center": dzc,
       "walk_back": (dzc[0] - fx * 6, dzc[1] - fy * 6), "walk_fwd": (dzc[0] + fx * 6, dzc[1] + fy * 6),
       "corr_center": cc, "corr_side": (cc[0] + rx * 3.0, cc[1] + ry * 3.0)}


def pos(tok):
    if isinstance(tok, (list, tuple)): return float(tok[0]), float(tok[1])
    if tok in POS: return POS[tok]
    x, y = tok.split(","); return float(x), float(y)


def parse(step):
    if isinstance(step, dict):
        k = next(iter(step)); v = step[k]
        return [k] + (v if isinstance(v, list) else [v])
    return step.split()


print("platform", plat_id, P["name"], "seed", seed, flush=True)
for k, v in POS.items():
    print("  %-11s %.2f %.2f door_zone %s corridor %s" % (k, v[0], v[1], in_poly(*v, P["door_zone"]) if P["door_zone"] else "-",
                                                         in_poly(*v, P["front_corridor"])), flush=True)
if a.dry:
    for s in steps: print("  step", parse(s))
    sys.exit(0)

c = carla.Client("localhost", 2000); c.set_timeout(20); w = c.get_world(); bl = w.get_blueprint_library()
peds = {}; f = open(a.log, "w") if a.log else None
if f: f.write("frame,sim,step,id,actor,x,y,in_door_zone,in_corridor,ego_x,ego_y,ego_v\n")
cur = "-"; last_log = -1.0; t_wall0 = time.time()


def ego():
    return next((v for v in w.get_actors().filter("vehicle.*") if v.attributes.get("role_name") == "ego_vehicle"), None)


def tick():
    global last_log
    snap = w.wait_for_tick(30); sim = snap.timestamp.elapsed_seconds
    if time.time() - t_wall0 > a.timeout: raise SystemExit("timeout")
    if f and sim - last_log >= 0.049:
        last_log = sim; e = ego(); es = snap.find(e.id) if e else None
        ex, ey, ev = (es.get_transform().location.x, -es.get_transform().location.y, es.get_velocity().length()) if es else (float("nan"),) * 3
        for pid, act in list(peds.items()):
            sa = snap.find(act.id)
            if sa is None: continue
            l = sa.get_transform().location; x, y = l.x, -l.y
            f.write("%d,%.2f,%s,%s,%d,%.2f,%.2f,%d,%d,%.2f,%.2f,%.2f\n" % (snap.frame, sim, cur, pid, act.id, x, y,
                    int(bool(P["door_zone"]) and in_poly(x, y, P["door_zone"])), int(in_poly(x, y, P["front_corridor"])), ex, ey, ev))
        f.flush()
    return snap, sim


def spawn(pid, where):
    x, y = pos(where); b = bl.find(rng.choice(PED_BPS)); b.set_attribute("role_name", "scn_ped")
    if b.has_attribute("is_invincible"): b.set_attribute("is_invincible", "false")
    for push in (0.0, 0.2, 0.4, 0.6):      # 차 옆면과 겹쳐 실패하면 바깥으로 조금씩
        act = w.try_spawn_actor(b, carla.Transform(carla.Location(x + rx * push, -(y + ry * push), 1.0), carla.Rotation(yaw=-math.degrees(yaw))))
        if act: break
    if act is None: raise SystemExit(f"spawn 실패 {pid} {where}")
    peds[pid] = act; print("spawn %s actor %d %s at %.2f %.2f (+%.1f) bp %s" % (pid, act.id, where, x, y, push, b.id), flush=True)


def walk(pid, where, speed=1.2):
    x, y = pos(where); act = peds[pid]
    while True:
        snap, sim = tick(); sa = snap.find(act.id)
        if sa is None: return
        l = sa.get_transform().location; ddx, ddy = x - l.x, -y - l.y; d = math.hypot(ddx, ddy)
        if d < 0.3:
            act.apply_control(carla.WalkerControl(speed=0.0)); print("walk %s -> %s done sim %.2f" % (pid, where, sim), flush=True); return
        act.apply_control(carla.WalkerControl(direction=carla.Vector3D(ddx / d, ddy / d, 0), speed=float(speed)))


try:
    for s in steps:
        p = parse(s); cur = p[0]; print("step", p, flush=True)
        if p[0] == "wait_ego_stopped":
            still = float(p[1]) if len(p) > 1 else 1.0; t_ok = None
            while True:
                snap, sim = tick(); e = ego(); es = snap.find(e.id) if e else None
                if es is None: continue
                l = es.get_transform().location
                if es.get_velocity().length() < 0.05 and in_poly(l.x, -l.y, P["stop_zone"]):
                    t_ok = t_ok if t_ok is not None else sim
                    if sim - t_ok >= still: print("ego stopped sim %.2f at %.2f %.2f" % (sim, l.x, -l.y), flush=True); break
                else: t_ok = None
        elif p[0] == "wait_ego_near":
            dist = float(p[1])
            while True:
                snap, sim = tick(); e = ego(); es = snap.find(e.id) if e else None
                if es is None: continue
                l = es.get_transform().location
                if math.hypot(l.x - dx, -l.y - dy) < dist: print("ego near sim %.2f" % sim, flush=True); break
        elif p[0] == "spawn": spawn(p[1], p[2])
        elif p[0] == "walk": walk(p[1], p[2], float(p[3]) if len(p) > 3 else 1.2)
        elif p[0] == "wait":
            snap, s0 = tick()
            while True:
                snap, sim = tick()
                if sim - s0 >= float(p[1]): break
        elif p[0] == "destroy":
            act = peds.pop(p[1]); act.destroy(); print("destroy", p[1], flush=True)
        else: raise SystemExit(f"모르는 단계 {p}")
    cur = "end"
finally:
    for pid, act in peds.items():
        try: act.destroy()
        except Exception: pass
    if f: f.close()
    print("end", flush=True)
