"""11/04 3단계: 순환 회차 평가(서버 없이). 입력 = s75_run_loop.sh 가 남긴 run log·track csv + 회차 목록 + 순환 노선 파일
구간마다: 도착(state 3)·걸린 시간·예상 대비, 정지 오차(문 기준), 차선 중심 옆 벗어남, 바퀴 이탈(주행 중 판정 그대로), 최대 가속·감속
- 정지 오차: run log 끝 pose(정답 측위 base_link, 지도 좌표) 를 목표(노선 파일 goal = 문 자리에서 뒤로 1.345m 뒷축 목표)와 견줌.
  앞뒤(목표 머리 방향 +) · 옆(왼 +). 목표가 문 자리에서 나왔으므로 앞뒤 오차 = 문 앞뒤 오차(문은 프리우스 1판 가정 x 1.345 그대로)
- 옆 벗어남: track 'lat'(차 원점이 CARLA 차로 가운데에서 옆으로, s4_track 정의). 구간 전체·유턴 레인렛 구간 빼고 둘 다
- 가감속: track v 를 0.25s(5틱) 앞뒤 차분. 최대 가속·최대 감속(정지 직전 0.5m/s 아래는 따로)
사용: python s76_loop_eval.py <tag> <segments.txt> <out.json> [--base-off 1.729]
"""
import argparse, csv, json, math, re
from datetime import datetime
from pathlib import Path

ap = argparse.ArgumentParser(); ap.add_argument("tag"); ap.add_argument("segs"); ap.add_argument("out")
ap.add_argument("--logdir", default=str(Path.home() / "campus_mobility_sim/integration/logs")); a = ap.parse_args()
L = Path(a.logdir); log = (L / f"i5_run_{a.tag}.log").read_text().splitlines()
segs = [l.split() for l in Path(a.segs).read_text().splitlines() if l.strip() and not l.startswith("#")]
T = list(csv.DictReader(open(L / f"i5_track_{a.tag}.csv")))
for r in T:
    for k in ("wall", "sim", "x", "y", "yaw", "v", "lat"): r[k] = float(r[k])
cut = (L / f"i5_run_{a.tag}.cut").read_text().strip() if (L / f"i5_run_{a.tag}.cut").exists() else ""
day = datetime.fromtimestamp(T[0]["wall"]).strftime("%Y-%m-%d")
def wall(hms): return datetime.strptime(f"{day} {hms}", "%Y-%m-%d %H:%M:%S").timestamp()
res = dict(tag=a.tag, cut=cut, segs=[])
for sg in segs:
    name, gx, gy, qz, qw = sg[0], float(sg[1]), float(sg[2]), float(sg[3]), float(sg[4]); exp = float(sg[10]) if len(sg) > 10 else None
    gyaw = 2 * math.atan2(qz, qw)
    st = next((l for l in log if f"seg {name} start" in l), None); en = next((l for l in log if f"seg {name} end" in l), None)
    d = dict(seg=name, expect_s=exp, started=st is not None)
    if st is None: res["segs"].append(d); continue
    t0 = wall(st.split()[0]); t1 = wall(en.split()[0]) if en else T[-1]["wall"]
    if en:
        m = re.search(r"elapsed_s (\d+) state (\d+) pose x: ([-\d.e]+) y: ([-\d.e]+)", en)
        el, state, px, py = int(m.group(1)), int(m.group(2)), float(m.group(3)), float(m.group(4))
        dx, dy = px - gx, py - gy
        d.update(elapsed_s=el, state=state, arrived=state == 3, end_pose=[round(px, 3), round(py, 3)],
                 stop_along_m=round(dx * math.cos(gyaw) + dy * math.sin(gyaw), 3), stop_lat_m=round(-dx * math.sin(gyaw) + dy * math.cos(gyaw), 3),
                 stop_dist_m=round(math.hypot(dx, dy), 3))
    else: d.update(arrived=False, note="end 줄 없음(끊김)")
    rs = [r for r in T if t0 <= r["wall"] <= t1]
    if not rs: res["segs"].append(d); continue
    mv = [r for r in rs if r["v"] > 0.3]
    lat = [abs(r["lat"]) for r in mv]
    # 유턴 레인렛 자리(CARLA 좌표, 지도 y 를 뒤집음): 모서리 유턴(716~722, 637~646)·후문 앞 유턴(965~980, 275~286) 둘레 15m
    def in_ut(r):
        x, y = r["x"], -r["y"]
        return math.hypot(x - 718, y - 642) < 15 or math.hypot(x - 972, y - 280) < 15
    lat2 = [abs(r["lat"]) for r in mv if not in_ut(r)]
    lat2.sort(); lat.sort()
    wo = [r for r in rs if int(r["wheels_off"]) > 0]; run = mx = 0
    for r in rs:
        run = run + 1 if int(r["wheels_off"]) > 0 else 0; mx = max(mx, run)
    acc = []
    for i in range(5, len(rs) - 5):
        dt = rs[i + 5]["sim"] - rs[i - 5]["sim"]
        if 0.4 < dt < 0.6: acc.append(((rs[i + 5]["v"] - rs[i - 5]["v"]) / dt, rs[i]["v"]))
    hi = [x for x, v in acc if v >= 0.5]
    d.update(ticks=len(rs), v_max=round(max(r["v"] for r in rs), 2), lat_max=round(lat[-1], 3) if lat else None,
             lat_p95=round(lat[int(0.95 * (len(lat) - 1))], 3) if lat else None,
             lat_max_no_uturn=round(lat2[-1], 3) if lat2 else None, lat_p95_no_uturn=round(lat2[int(0.95 * (len(lat2) - 1))], 3) if lat2 else None,
             wheels_off_ticks=len(wo), wheels_off_max_run_s=round(mx * 0.05, 2), wheels_off_max=max([int(r["wheels_off"]) for r in rs] + [0]),
             offroad_ticks=sum(int(r["offroad"]) for r in rs), hits=max(int(r["hits"]) for r in rs),
             acc_max=round(max(hi), 2) if hi else None, dec_max=round(min(hi), 2) if hi else None,
             dec_max_lowspeed=round(min([x for x, v in acc if v < 0.5] + [0]), 2),
             throttle_max=round(max(float(r["throttle"]) for r in rs), 3), brake_max=round(max(float(r["brake"]) for r in rs), 3),
             steer_abs_max=round(max(abs(float(r["steer"])) for r in rs), 3))
    res["segs"].append(d)
ok = [s for s in res["segs"] if s.get("arrived")]
res["summary"] = dict(arrived=f"{len(ok)}/{len(segs)}", total_s=sum(s.get("elapsed_s", 0) for s in res["segs"]),
                      stop_along_abs_max=max([abs(s["stop_along_m"]) for s in ok] + [0]), lat_max=max([s.get("lat_max") or 0 for s in res["segs"]]),
                      wheels_off_ticks=sum(s.get("wheels_off_ticks", 0) for s in res["segs"]),
                      acc_max=max([s.get("acc_max") or 0 for s in res["segs"]]), dec_max=min([s.get("dec_max") or 0 for s in res["segs"]]))
Path(a.out).write_text(json.dumps(res, ensure_ascii=False, indent=1))
print(json.dumps(res["summary"], ensure_ascii=False))
for s in res["segs"]:
    print(s["seg"][:30], s.get("arrived"), s.get("elapsed_s"), s.get("expect_s"), s.get("stop_along_m"), s.get("stop_lat_m"), s.get("lat_max_no_uturn"), s.get("wheels_off_ticks"), s.get("acc_max"), s.get("dec_max"))
