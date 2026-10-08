#!/usr/bin/env python3
# 맵세션31: reconversion_plan.md 3-1 "오프라인 사전 회귀" (읽기전용, 서버 없음, test_drive.py 수정 없음)
# 1 경로 합: 기존 dry-plan CSV 의 road 순서를 id 대응표로 번역 -> 새 맵 길이로 (앞 road 길이 합 + 종점 s). 기준 s26_route_sums.log, 허용 0.1m
# 2 dry-plan 대조: 기존 drive_plan_dry_<노선>.csv 각 표본 (road, s) 를 번역해 새 맵에서 같은 s 위치를 test_drive.xodr_point_at_s 로 계산.
#   기존 (x, y) 와 거리. 허용 0.01m. test_drive.py 는 import 만(함수 재사용), 파일은 쓰지 않는다
# 사용: .venv-carla/bin/python s31_offline_regress.py <새 xodr> <id map csv>
import csv
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(BASE / "tests"))
import test_drive as td  # noqa: E402  (모듈 import 시 서버 연결 없음: carla.Client 는 함수 안에만 있음)

LOGS = BASE / "docs/logs"
REF = {"north": (78.0, 622.9), "south": (47.0, 730.0), "middle": (70.0, 693.7), "yangseong": (22.0, 637.4)}  # s26_route_sums.log


def main():
    xodr, idmap = Path(sys.argv[1]), sys.argv[2]
    mp = {}
    for r in csv.DictReader(open(idmap)):
        mp[int(r["old_id"])] = (int(r["new_id"]), float(r["match_cost"]))
    root = ET.parse(str(xodr)).getroot()
    els = {int(r.get("id")): r for r in root.iter("road")}
    bad = 0
    for k, (end_s, ref) in REF.items():
        rows = list(csv.DictReader(open(LOGS / f"drive_plan_dry_{k}.csv")))
        seq = list(dict.fromkeys(int(r["road_id"]) for r in rows))
        new = [mp[r][0] for r in seq]
        s = sum(float(els[n].get("length")) for n in new[:-1]) + end_s
        worst = 0.0
        wrow = None
        for r in rows:
            n = mp[int(r["road_id"])][0]
            x, y = td.xodr_point_at_s(els[n], float(r["s"]))
            d = math.hypot(x - float(r["x"]), -y - float(r["y"]))
            if d > worst:
                worst, wrow = d, (r["road_id"], n, r["s"])
        ok1 = abs(s - ref) <= 0.1
        ok2 = worst <= 0.01
        bad += (not ok1) + (not ok2)
        print(f"{k}: road {len(seq)} 번역 {new}")
        print(f"  경로 합 {s:.2f}m (기준 {ref}, 차 {s - ref:+.2f}) {'통과' if ok1 else '초과'}"
              f" / dry-plan 표본 {len(rows)}점 최대 차 {worst:.4f}m (옛 r{wrow[0]} -> 새 r{wrow[1]} s={wrow[2]}) {'통과' if ok2 else '초과'}")
    print(f"오프라인 사전 회귀: 초과 {bad}건")


if __name__ == "__main__":
    main()
