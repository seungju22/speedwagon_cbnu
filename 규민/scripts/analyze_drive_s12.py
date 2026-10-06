"""세션12 주행 로그 분석(읽기 전용). 정지 위치, 직전 3초, 충돌 방향, 잔여 목록 대조."""
import csv, math, sys
from collections import Counter

L = "/home/gyumin/campus_mobility_sim/map/docs/logs/"
TS = "20260924_113337"
log = list(csv.DictReader(open(L + f"drive_log_{TS}.csv")))
col = list(csv.DictReader(open(L + f"collision_events_{TS}.csv")))
res = list(csv.DictReader(open(L + "session11_residual_sites.csv")))
f = lambda r, k: float(r[k])

frame2row = {int(r["frame"]): r for r in log}
t_end = f(log[-1], "t")
print("rows", len(log), "t_end", t_end)

# 옛 정지 지점 통과
p = next((r for r in log if r["road_id"] == "1247" and f(r, "s") >= 89.35), None)
print("old stop passed:", p and (p["t"], p["s"], p["speed_mps"]))
first_cf = int(col[0]["frame"])
print("collisions before road1240:",
      sum(1 for c in col if frame2row.get(int(c["frame"]), {}).get("road_id") != "1240"))

# 첫 충돌 시점의 주행 행
fr = frame2row.get(first_cf) or min(log, key=lambda r: abs(int(r["frame"]) - first_cf))
t_c = f(fr, "t")
print(f"first collision frame {first_cf} t={t_c:.3f} road{fr['road_id']} s={fr['s']} "
      f"x={fr['x']} y={fr['y']} speed={fr['speed_mps']}")
print(f"last collision frame {col[-1]['frame']} count {len(col)}")
roads = Counter(frame2row[int(c['frame'])]['road_id'] for c in col if int(c['frame']) in frame2row)
print("collision roads:", dict(roads))

# 감속 시작: 첫 충돌 이전 마지막으로 speed>=5.0 인 시각 이후 처음 4.5 미만
dec = next((r for r in log if f(r, "t") > t_c - 5 and f(r, "speed_mps") < 4.5), None)
print("first speed<4.5 after t_c-5:", dec and (dec["t"], dec["s"], dec["speed_mps"]))

# 직전 3초 + 이후 2초 0.25s 간격
print("t s speed thr brk steer pitch roll yaw x y")
last = -99
for r in log:
    t = f(r, "t")
    if t_c - 3 <= t <= t_c + 2 and t - last >= 0.25:
        last = t
        print(f"{t:.2f} {f(r,'s'):.2f} {f(r,'speed_mps'):.2f} {f(r,'throttle'):.2f} "
              f"{f(r,'brake'):.2f} {f(r,'steer'):+.3f} {f(r,'pitch'):+.2f} {f(r,'roll'):+.2f} "
              f"{f(r,'vehicle_yaw'):.1f} {f(r,'x'):.2f} {f(r,'y'):.2f}")
e = log[-1]
print(f"END t={e['t']} road{e['road_id']} s={e['s']} x={e['x']} y={e['y']} z={e['z']} "
      f"yaw={e['vehicle_yaw']} thr={e['throttle']} brk={e['brake']} steer={e['steer']}")

# 임펄스 방향(차량 좌표): 전방=(cos,sin), 오른쪽=(-sin,cos) (CARLA 좌수계)
side = Counter(); fwd = Counter(); zc = 0
for c in col:
    r = frame2row.get(int(c["frame"]))
    if r is None: continue
    y = math.radians(f(r, "vehicle_yaw"))
    ix, iy = f(c, "impulse_x"), f(c, "impulse_y")
    m = math.hypot(ix, iy) or 1
    rt = (-math.sin(y) * ix + math.cos(y) * iy) / m
    fw = (math.cos(y) * ix + math.sin(y) * iy) / m
    side["push_right(contact_left)" if rt > 0.3 else "push_left(contact_right)" if rt < -0.3 else "side~0"] += 1
    fwd["push_fwd" if fw > 0.3 else "push_back(contact_front)" if fw < -0.3 else "fwd~0"] += 1
    zc += f(c, "impulse_z") == 0.0
print("side:", dict(side)); print("fwd:", dict(fwd)); print("impulse_z==0:", zc)
c0 = col[0]; r0 = frame2row[first_cf]; y = math.radians(f(r0, "vehicle_yaw"))
ix, iy = f(c0, "impulse_x"), f(c0, "impulse_y"); m = math.hypot(ix, iy)
print(f"first impulse unit fwd={(math.cos(y)*ix+math.sin(y)*iy)/m:+.3f} "
      f"right={(-math.sin(y)*ix+math.cos(y)*iy)/m:+.3f}")

# 잔여 61곳 대조
hit = [x for x in res if x["road_new"] == "1240"]
print("residual on road1240:", [(x["s"], x["f"], x["R2_m"], x["category"]) for x in hit])
print("residual roads:", sorted({x["road_new"] for x in res}))
route = {"1247","1914","1356","1917","1355","1446","1172","1428","1240","1695","1239","1426","1238","1564","1278"}
print("residual on route:", [x["road_new"] for x in res if x["road_new"] in route])

# junction1 r1915 구간
for r in log:
    if r["road_id"] in ("1915",) :
        print("r1915 row", r["t"], r["s"], r["x"], r["y"], r["is_junction"], r["junction_id"])
        break
print("r1915 rows:", sum(r["road_id"] == "1915" for r in log),
      "r1914 rows:", sum(r["road_id"] == "1914" for r in log))
