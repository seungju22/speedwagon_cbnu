#!/usr/bin/env python3
# 11/03: 주행 기록(track csv)의 바퀴 이탈 틱을 오프라인 carla.Map(동결 xodr)으로 다시 봄. 서버 없음
#   주행 중 판정(s4_track·s75_track)은 get_waypoint(project_to_road=False, Driving|Sidewalk) 가 None 이면 이탈. 차로 경계에 0.25m 쯤 가는 틈 줄이 있어
#   틈 위를 지나는 바퀴도 이탈로 셈 -> 같은 바퀴 자리에서 0.2m 맞은편 두 점이 둘 다 면이면 '틈'으로 따로 셈(s75_m20.py 와 같은 규칙)
#   바퀴 자리 = track log 첫 줄 wheels_local(차 좌표) 을 매 틱 x·y·yaw 로 옮김
# 사용: .venv-carla/bin/python s75_wheel_recheck.py <track.csv> <track.log> [out.json] [--box x0,x1,y0,y1(지도 좌표) 이름 ...]
import csv, json, math, re, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "map/scripts"))
import s72_common as K, carla
M = K.cmap(); RD = carla.LaneType.Driving | carla.LaneType.Sidewalk; cache = {}
def on0(x, y):   # CARLA 좌표
    k = (round(x * 20), round(y * 20))
    if k not in cache: cache[k] = M.get_waypoint(carla.Location(k[0] / 20, k[1] / 20, 0), project_to_road=False, lane_type=RD) is not None
    return cache[k]
def gap(x, y):
    d = 0.2
    return any(on0(x + dx, y + dy) and on0(x - dx, y - dy) for dx, dy in ((d, 0), (0, d), (d, d), (d, -d)))
csvp, logp = sys.argv[1], sys.argv[2]; outp = sys.argv[3] if len(sys.argv) > 3 and not sys.argv[3].startswith("--") else None
boxes = {}
if "--box" in sys.argv:
    i = sys.argv.index("--box"); args = sys.argv[i + 1:]
    for b, n in zip(args[::2], args[1::2]): boxes[n] = [float(v) for v in b.split(",")]
wl = json.loads(re.search(r"wheels_local (\[.*\])", open(logp).read()).group(1))
res = dict(ticks=0, off_ticks=0, real_off_ticks=0, gap_only_ticks=0, max_real_run_s=0.0, boxes={})
run0 = None; worst = []
for r in csv.DictReader(open(csvp)):
    res["ticks"] += 1
    if int(r["wheels_off"]) == 0: run0 = None; continue
    res["off_ticks"] += 1
    x, y, yaw = float(r["x"]), float(r["y"]), math.radians(float(r["yaw"])); c, s = math.cos(yaw), math.sin(yaw)
    real = [i for i, (a, b, _) in enumerate(wl) if not on0(x + a * c - b * s, y + a * s + b * c) and not gap(x + a * c - b * s, y + a * s + b * c)]
    sim = float(r["sim"])
    if real:
        res["real_off_ticks"] += 1
        if run0 is None: run0 = sim
        res["max_real_run_s"] = max(res["max_real_run_s"], round(sim - run0, 2)); worst.append((round(x, 2), round(-y, 2), r["road"], len(real)))
    else:
        res["gap_only_ticks"] += 1; run0 = None
    for n, (x0, x1, y0, y1) in boxes.items():
        if x0 < x < x1 and y0 < -y < y1:
            bx = res["boxes"].setdefault(n, dict(off_ticks=0, real_off_ticks=0)); bx["off_ticks"] += 1; bx["real_off_ticks"] += bool(real)
res["real_off_sample"] = worst[:20]
print(json.dumps(res, ensure_ascii=False))
if outp: Path(outp).write_text(json.dumps(res, ensure_ascii=False, indent=1))
