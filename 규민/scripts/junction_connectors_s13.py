#!/usr/bin/env python3
# 맵세션13 최소 조사(읽기 전용, 서버 불필요). 확정 경로 15 road 가 지나는 junction 7개에서
# 진입 road 끝(lane -1)에서 갈 수 있는 커넥터를 전부 나열하고, 회전각과 U턴 여부를 판정한다.
# 서버 없이 carla.Map(name, xodr 문자열) 로 클라이언트 측 지도를 만든다.
# U턴 판정: 커넥터 뒤 road(나가는 road)의 시작점이 진입 road 끝점에서 가깝고(쌍둥이 간격)
#           시작 방향이 진입 방향과 150° 이상 반대. 회전각도 함께 기록.
# 출력: map/docs/logs/session13_junction_connectors.csv + 화면 요약
# 근거: https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Map
import csv
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import carla

BASE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(BASE / "tests"))
import test_drive as T  # noqa: E402  ROUTE_ROADS, CONNECTOR_JUNCTION, XODR_PATH 재사용

OUT = BASE / "docs" / "logs" / "session13_junction_connectors.csv"
TWIN_DIST_M = 8.0     # 진입 끝 ~ 나가는 road 시작 거리 상한(쌍둥이 차선중심 간격 여유)
UTURN_DEG = 150.0


def ang(a):
    return (a + 180.0) % 360.0 - 180.0


def main():
    xodr = T.XODR_PATH.read_text()
    m = carla.Map("cbnu_s13", xodr)
    L = {int(r.get("id")): float(r.get("length"))
         for r in ET.fromstring(xodr).findall("road")}
    rows = []
    for i, rid in enumerate(T.ROUTE_ROADS):
        if rid not in T.CONNECTOR_JUNCTION:
            continue
        inc = T.ROUTE_ROADS[i - 1]
        planned = rid
        end = m.get_waypoint_xodr(inc, T.LANE_ID, L[inc] - 0.05)
        e_loc, e_yaw = end.transform.location, end.transform.rotation.yaw
        for n in end.next(0.5):
            c = n.road_id
            c_end = m.get_waypoint_xodr(c, n.lane_id, max(L[c] - 0.05, 0.0))
            turn = ang(c_end.transform.rotation.yaw - e_yaw)
            outs = sorted({w.road_id for w in c_end.next(1.0)})
            o_wp = c_end.next(1.0)[0] if outs else None
            if o_wp is not None:
                o_start = m.get_waypoint_xodr(o_wp.road_id, o_wp.lane_id, 0.05)
                d = o_start.transform.location.distance(e_loc)
                rev = abs(ang(o_start.transform.rotation.yaw - e_yaw))
            else:
                d, rev = float("nan"), float("nan")
            uturn = d <= TWIN_DIST_M and rev >= UTURN_DEG
            rows.append({"junction": T.CONNECTOR_JUNCTION[planned], "incoming": inc,
                         "connector": c, "planned": int(c == planned),
                         "length_m": round(L[c], 2), "turn_deg": round(turn, 1),
                         "out_road": "/".join(map(str, outs)),
                         "out_start_dist_m": round(d, 2), "out_rev_deg": round(rev, 1),
                         "uturn": int(uturn)})
    with OUT.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    for j in dict.fromkeys(r["junction"] for r in rows):
        rs = [r for r in rows if r["junction"] == j]
        p = [r["connector"] for r in rs if r["planned"]]
        u = [r["connector"] for r in rs if r["uturn"]]
        print(f"j{j} in r{rs[0]['incoming']} 커넥터{len(rs)} 계획{p} U턴{len(u)}{u}")
    print(f"CSV: {OUT}")


if __name__ == "__main__":
    main()
