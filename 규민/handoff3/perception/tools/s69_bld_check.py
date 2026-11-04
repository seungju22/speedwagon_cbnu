"""10/25 2단계: 건물 점 지움 사각형(여유 --bpad)이 차로·인도·문 구역 보행자 자리에 걸치는지 기하로 확인. 호스트 .venv-carla(서버 없음)
- 사각형 둘레를 0.1m 간격으로 찍어(여유 더한 변) v3.1 동결 xodr 의 Driving·Sidewalk 차로 안에 드는 점을 셈
- 나 장면 csv(scn_*.csv)의 보행자 자리가 사각형 안에 드는지 셈
사용: s69_bld_check.py <out.json> [--bpad 0.3] [scn csv ...]
"""
import argparse, json, math
import csv as _csv
import carla
from pathlib import Path
C = Path.home() / "campus_mobility_sim"
ap = argparse.ArgumentParser(); ap.add_argument("out"); ap.add_argument("scn", nargs="*"); ap.add_argument("--bpad", type=float, default=0.3)
a = ap.parse_args()
R = json.load(open(C / "perception/config/s69_building_rects.json"))["rects"]
m = carla.Map("OpenDriveMap", open(C / "map/maps/cbnu_campus_frozen_v31.xodr").read())
hits = {"Driving": [], "Sidewalk": []}; n = 0
for r in R:
    h = r["side"] / 2 + a.bpad; c, s = math.cos(math.radians(r["yaw_deg"])), math.sin(math.radians(r["yaw_deg"]))
    k = int(2 * h / 0.1)
    for e in range(4):
        for i in range(k + 1):
            u = -h + i * 2 * h / k
            uv = [(u, -h), (h, u), (-u, h), (-h, -u)][e]
            x, y = r["cx"] + uv[0] * c - uv[1] * s, r["cy"] + uv[0] * s + uv[1] * c; n += 1
            for lt in ("Driving", "Sidewalk"):
                if m.get_waypoint(carla.Location(x, -y, 0), project_to_road=False, lane_type=getattr(carla.LaneType, lt)) is not None:
                    hits[lt].append((r["owner"], r["layer"], round(x, 2), round(y, 2)))
ped = {}
for f in a.scn:
    cnt = tot = 0
    for g in _csv.DictReader(open(f)):
        if g["in_door_zone"] != "1" and g["in_corridor"] != "1": continue
        tot += 1; px, py = float(g["x"]), float(g["y"])
        for r in R:
            h = r["side"] / 2 + a.bpad; c, s = math.cos(math.radians(r["yaw_deg"])), math.sin(math.radians(r["yaw_deg"]))
            dx, dy = px - r["cx"], py - r["cy"]
            if abs(dx * c + dy * s) <= h and abs(-dx * s + dy * c) <= h: cnt += 1; break
    ped[Path(f).name] = {"door_corridor_rows": tot, "inside_rect": cnt}
res = {"bpad": a.bpad, "rects": len(R), "edge_samples": n, "edge_in_driving": len(hits["Driving"]), "edge_in_sidewalk": len(hits["Sidewalk"]),
       "owners_driving": sorted({h[0] for h in hits["Driving"]}), "owners_sidewalk": sorted({h[0] for h in hits["Sidewalk"]}), "examples": {k: v[:10] for k, v in hits.items()}, "ped": ped}
json.dump(res, open(a.out, "w"), ensure_ascii=False, indent=1)
print(json.dumps({k: res[k] for k in ("rects", "edge_samples", "edge_in_driving", "edge_in_sidewalk", "owners_driving", "owners_sidewalk", "ped")}, ensure_ascii=False))
