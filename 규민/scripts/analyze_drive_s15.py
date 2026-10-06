#!/usr/bin/env python3
# 맵세션15 Phase D. 완주 궤적(drive_log_*.csv) 분석. 읽기전용(xodr/로그 미수정),
# 출력: map/docs/logs/session15_drive_analysis.log, session15_section_deviation.csv
#
# 계획 차선 기준 이탈량(전 구간, 사후 계산):
#   plan_road = 차 중심에서 가장 가까운 계획 중심선 표본의 road (계획 15 road, lane -1, 0.25m)
#   lat_m     = 그 표본 기준 부호 있는 횡거리(+=오른쪽). test_drive.sharp_offset 과 같은 방식
#   excess_m  = bbox 바닥 4꼭짓점(x,y,yaw,extent 로 복원) |횡거리| - 차선폭/2 최댓값
#               (각 꼭짓점도 가장 가까운 계획 표본 기준). + = 차선 밖으로 나간 거리
# 투영 road(get_waypoint)는 junction 커넥터 겹침(세션15 r1428/r1434) 때문에 쓰지 않는다.
import csv
import math
import sys
from pathlib import Path

import carla
import numpy as np

BASE_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(BASE_DIR / "tests"))
import test_drive as T  # noqa: E402

LOG_DIR = BASE_DIR / "docs" / "logs"
UTURN = {1915, 1449, 1429, 1697}


def main(csv_name):
    rows = list(csv.DictReader(open(LOG_DIR / csv_name)))
    m = carla.Map("s15", T.XODR_PATH.read_text())
    polys = T.sharp_polylines(m, T.ROUTE_ROADS)
    flat = [(rid,) + p for rid, pts in polys.items() for p in pts]
    arr = np.array([[p[2], p[3]] for p in flat])
    ext = [float(v.split("=")[1]) for v in
           (LOG_DIR / csv_name.replace("drive_log_", "vehicle_bbox_").replace(".csv", ".txt"))
           .read_text().split()]
    hx, hy = ext[0], ext[1]

    def nearest(x, y):
        i = int(np.argmin((arr[:, 0] - x) ** 2 + (arr[:, 1] - y) ** 2))
        rid, s, px, py, rx, ry, w = flat[i]
        return rid, s, (x - px) * rx + (y - py) * ry, w

    out = []
    for r in rows:
        x, y, yaw = float(r["x"]), float(r["y"]), math.radians(float(r["vehicle_yaw"]))
        rid, s, lat, w = nearest(x, y)
        fx, fy = math.cos(yaw), math.sin(yaw)
        rxv, ryv = -fy, fx
        exc = -1e9
        for dx, dy in ((hx, -hy), (hx, hy), (-hx, -hy), (-hx, hy)):
            cx, cy = x + dx * fx + dy * rxv, y + dx * fy + dy * ryv
            _, _, cl, cw = nearest(cx, cy)
            exc = max(exc, abs(cl) - cw / 2.0)
        out.append({"t": float(r["t"]), "x": x, "y": y, "speed": float(r["speed_mps"]),
                    "plan_road": rid, "plan_s": s, "lat": lat, "excess": exc,
                    "proj_road": int(r["road_id"]), "sidewalk": r["sidewalk"] == "True",
                    "target": r["agent_target_road"], "junction": r["junction_id"]})

    lines = []
    p = lines.append
    dist = sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"]) for a, b in zip(out, out[1:]))
    t_end = out[-1]["t"]
    p(f"입력: {csv_name}, tick {len(out)}")
    p(f"궤적 길이: {dist:.1f}m, 시뮬 시간 {t_end:.2f}s")
    p(f"평균 속도: {dist / t_end:.2f}m/s ({dist / t_end * 3.6:.1f}km/h)")
    vmax = max(out, key=lambda o: o["speed"])
    p(f"최고 속도: {vmax['speed']:.2f}m/s ({vmax['speed'] * 3.6:.1f}km/h) t={vmax['t']:.2f}")
    tg = [int(o["target"]) for o in out if o["target"]]
    p(f"agent 목표 U턴 커넥터 tick: {sum(t in UTURN for t in tg)}")
    p(f"투영 U턴 커넥터 tick: {sum(o['proj_road'] in UTURN for o in out)} (겹침 투영, 참고)")

    # 계획 road 시퀀스(최근접 계획 중심선 기준) + 구간 통과 시각
    p("\n=== 최근접 계획 road 시퀀스(구간 통과 시각) ===")
    segs = []
    for o in out:
        if not segs or segs[-1]["road"] != o["plan_road"]:
            segs.append({"road": o["plan_road"], "t0": o["t"], "ticks": []})
        segs[-1]["ticks"].append(o)
    for g in segs:
        tk = g["ticks"]
        kind = "conn" if g["road"] in T.CONNECTOR_JUNCTION else "road"
        jn = f" junction{T.CONNECTOR_JUNCTION[g['road']]}" if kind == "conn" else ""
        proj = sorted({o["proj_road"] for o in tk})
        p(f"r{g['road']} {kind}{jn}: t={tk[0]['t']:.2f}~{tk[-1]['t']:.2f}s, "
          f"max|lat| {max(abs(o['lat']) for o in tk):.2f}m, "
          f"max excess {max(o['excess'] for o in tk):+.2f}m, 투영 road {proj}")
    seq = [g["road"] for g in segs]
    p(f"시퀀스 = 계획 15 road: {seq == T.ROUTE_ROADS} ({len(seq)}개)")

    # 인도 침범 구간(연속 tick 묶음, 0.5s 이하 끊김은 이어붙임)
    p("\n=== 인도 침범 구간 ===")
    runs = []
    for o in out:
        if not o["sidewalk"]:
            continue
        if runs and o["t"] - runs[-1][-1]["t"] <= 0.5:
            runs[-1].append(o)
        else:
            runs.append([o])
    sec = []
    for k, rn in enumerate(runs, 1):
        roads = sorted({o["plan_road"] for o in rn})
        worst = max(rn, key=lambda o: o["excess"])
        sec.append({"section": f"sidewalk{k}", "plan_roads": " ".join(map(str, roads)),
                    "t_start": rn[0]["t"], "t_end": rn[-1]["t"], "ticks": len(rn),
                    "max_abs_lat_m": round(max(abs(o["lat"]) for o in rn), 2),
                    "max_excess_m": round(worst["excess"], 2),
                    "side": "오른쪽" if worst["lat"] > 0 else "왼쪽",
                    "x": round(worst["x"], 1), "y": round(worst["y"], 1)})
        p(f"sidewalk{k}: r{roads} t={rn[0]['t']:.2f}~{rn[-1]['t']:.2f}s {len(rn)}tick "
          f"max|lat| {sec[-1]['max_abs_lat_m']}m excess {worst['excess']:+.2f}m "
          f"({sec[-1]['side']}) at ({worst['x']:.1f},{worst['y']:.1f})")
    for rid in T.SHARP_CONNECTORS:
        tk = [o for o in out if o["plan_road"] == rid]
        worst = max(tk, key=lambda o: o["excess"])
        sec.append({"section": f"sharp r{rid}", "plan_roads": str(rid),
                    "t_start": tk[0]["t"], "t_end": tk[-1]["t"], "ticks": len(tk),
                    "max_abs_lat_m": round(max(abs(o["lat"]) for o in tk), 2),
                    "max_excess_m": round(worst["excess"], 2),
                    "side": "오른쪽" if worst["lat"] > 0 else "왼쪽",
                    "x": round(worst["x"], 1), "y": round(worst["y"], 1)})
    allx = max(out, key=lambda o: o["excess"])
    p(f"\n전 구간 max excess {allx['excess']:+.2f}m r{allx['plan_road']} t={allx['t']:.2f}")
    p(f"excess>0 tick: {sum(o['excess'] > 0 for o in out)} / {len(out)}")

    with open(LOG_DIR / "session15_section_deviation.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(sec[0].keys()))
        w.writeheader()
        w.writerows(sec)
    with open(LOG_DIR / "session15_drive_analysis.log", "w") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "drive_log_20260925_113316.csv")
