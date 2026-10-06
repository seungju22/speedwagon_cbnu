#!/usr/bin/env python3
# 세션19 Phase 4 검증(서버 없음): test_drive.py 의 record 판정 함수(plan_track, road_radius)와
# 같은 조건식으로 과거 주행 CSV 를 다시 판정한다. 충돌은 collision CSV 첫 frame 으로 대조.
# 메시 밖은 CSV mesh_out 열 사용(세션14 이후 로그만).
import csv, importlib.util, sys
from pathlib import Path
import carla
B = Path("/home/gyumin/campus_mobility_sim/map")
spec = importlib.util.spec_from_file_location("td", B / "tests/test_drive.py")
td = importlib.util.module_from_spec(spec); spec.loader.exec_module(td)
cmap = carla.Map("v1", td.XODR_PATH.read_text())
HALF_W = 0.894   # vehicle.audi.a2 bbox 반폭(vehicle_bbox_*.txt 와 같음)
RUNS = [("s13 TM(비동기)", "20260924_182607", "legacy"), ("s14", "20260924_231850", "legacy"),
        ("s15a", "20260925_112842", "legacy"), ("s15b", "20260925_113316", "legacy"),
        ("s16", "20260926_144047", "legacy"), ("s17", "20260926_180655", "legacy"),
        ("s17v", "20260926_181143", "legacy"), ("s18 north", "20260928_224708", "north"),
        ("s18 south", "20260928_224907", "south"), ("s18 middle", "20260928_225108", "middle")]
for lab, ts, route in RUNS:
    td.select_route(route)
    polys = td.sharp_polylines(cmap, td.ROUTE_ROADS)
    flat = [(r, p) for r, pts in polys.items() for p in pts]
    rows = list(csv.DictReader(open(B / f"docs/logs/drive_log_{ts}.csv")))
    col = [int(r["frame"]) for r in csv.DictReader(open(B / f"docs/logs/collision_events_{ts}.csv"))] \
        if (B / f"docs/logs/collision_events_{ts}.csv").exists() else []
    run, fail, segs, cur = {}, None, [], None
    for r in rows:
        loc = carla.Location(x=float(r["x"]), y=float(r["y"]))
        t = float(r["t"])
        rid, lat, dist, w = td.plan_track(flat, loc)
        over = abs(lat) + HALF_W - w / 2
        if over > 0:
            side = 1 if lat > 0 else -1
            if cur is None or cur["side"] != side:
                cur = {"t0": t, "side": side, "road": rid, "body": over, "center": abs(lat) - w / 2}
                segs.append(cur)
            if over > cur["body"]:
                cur.update(body=over, road=rid, center=abs(lat) - w / 2)
        else:
            cur = None
        conds = {"충돌": (bool(col) and int(r["frame"]) >= col[0], 1, 0.0),
                 "깊은 반대 차선 침범": (-lat - w / 2 >= td.DEEP_OPP_M, td.DEEP_OPP_TICKS, td.DEEP_OPP_SECONDS),
                 "경로 완전 이탈": (dist > td.LOST_M, td.LOST_TICKS, td.LOST_SECONDS),
                 "메시 밖": (r.get("mesh_out") == "True", td.OFFROAD_TICKS, td.OFFROAD_MIN_SECONDS)}
        for name, (hit, nn, ss) in conds.items():
            if not hit:
                run.pop(name, None); continue
            n, t0 = run.get(name, (0, t)); run[name] = (n + 1, t0)
            if n + 1 >= nn and t - t0 >= ss and fail is None:
                fail = f"{name} t={t:.2f}s road{rid}"
        if fail:
            break
    left = [g for g in segs if g["side"] < 0]
    print(f"{lab}: 로그 끝 t={rows[-1]['t']} / record 판정 {'종료 ' + fail if fail else '종료 없음(로그 끝까지 계속)'}")
    print(f"   차선 이탈(차체) {len(segs)}구간, 중앙선 쪽 {len(left)}: "
          + ", ".join(f"r{g['road']} {g['side']:+d} 차체{g['body']:.2f}/중심{max(g['center'],0):.2f}" for g in segs))
print("road_radius 검산:", {r: tuple(round(v, 2) for v in td.road_radius(cmap, r)) for r in (1446, 1428, 1564, 1592, 1838, 1286)})
