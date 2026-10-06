#!/usr/bin/env python3
# 맵세션21 Phase 1. 지난 주행 로그 전부를 완주 판정 새 조건으로 다시 판정(오프라인, 서버 불필요).
# 옛 조건: 마지막 road 위 s >= arrive_min_s 이고 정차점 5m 이내
# 새 조건(record 판정): 옛 조건 OR (속도 < 0.5m/s 가 5s 지속된 순간 정차점 2.0m 이내). 정체 선언보다 먼저 본다
# 정차점은 계획 CSV 마지막 road 로 루트를 정하고 test_drive.ROUTES 의 end_s 를 frozen_v1 맵에서 조회.
# 세션11 이전(평활화 전 맵) 로그도 같은 좌표로 대조한다(road1278 위치 차이는 판정에 영향 없는 수준인지 출력으로 확인)
import csv
import math
import sys
from pathlib import Path

import carla

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tests"))
import test_drive as td  # noqa: E402

LOGS = td.LOG_DIR
cmap = carla.Map("s21", td.XODR_PATH.read_text())
BY_LAST = {r["roads"][-1]: (name, r) for name, r in td.ROUTES.items()}


def judge(path):
    ts = path.stem.replace("drive_log_", "")
    plan = LOGS / f"drive_plan_{ts}.csv"
    last = int(list(csv.DictReader(plan.open()))[-1]["road_id"]) if plan.exists() else 1278
    name, r = BY_LAST[last]
    end = cmap.get_waypoint_xodr(last, -1, r["end_s"]).transform.location
    old_t = new_t = stuck_fail_t = None
    stuck_t0, min_d = None, 1e9
    for row in csv.DictReader(path.open()):
        if not row.get("speed_mps"):
            continue
        t, v = float(row["t"]), float(row["speed_mps"])
        d = math.hypot(float(row["x"]) - end.x, float(row["y"]) - end.y)
        min_d = min(min_d, d)
        road = int(row["road_id"]) if row.get("road_id") else None
        s = float(row["s"]) if row.get("s") else None
        if road is None and "road_id" in row:     # measure-stop 판정 뒤 행
            break
        pass_end = road == last and s is not None and s >= r["arrive_min_s"] and d <= td.ARRIVE_DIST_M
        stop_near = v < td.STUCK_SPEED_MPS and stuck_t0 is not None \
            and t - stuck_t0 >= td.STUCK_SECONDS and d <= td.ARRIVE_STOP_DIST_M
        if pass_end:
            old_t = t
            new_t = t if new_t is None else new_t
            break
        if stop_near and new_t is None:
            new_t = t
            break
        if v < td.STUCK_SPEED_MPS:
            stuck_t0 = t if stuck_t0 is None else stuck_t0
            if t - stuck_t0 >= td.STUCK_SECONDS and stuck_fail_t is None:
                stuck_fail_t = t
        else:
            stuck_t0 = None
    old = f"완주 t={old_t:.2f}" if old_t is not None else "미완주"
    new = (f"완주 t={new_t:.2f}" + ("" if new_t == old_t else " (정지 근접)")) if new_t is not None else "미완주"
    flag = "같음" if old == new else "변경"
    print(f"{ts} route={name:<9} 옛 {old:<14} 새 {new:<24} {flag} | 정차점 최소거리 {min_d:7.2f}m"
          f"{f' 정체조건 충족 t={stuck_fail_t:.2f}' if stuck_fail_t is not None else ''}")
    return flag


if __name__ == "__main__":
    print(f"map_sha256={td.file_sha16()} 새 조건: 정지(<{td.STUCK_SPEED_MPS}m/s {td.STUCK_SECONDS:g}s) "
          f"+ 정차점 {td.ARRIVE_STOP_DIST_M:g}m 이내")
    flags = [judge(p) for p in sorted(LOGS.glob("drive_log_*.csv"))]
    print(f"합계 {len(flags)}개, 변경 {flags.count('변경')}개")
