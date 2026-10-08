#!/usr/bin/env python3
# 맵세션6 Phase C. 878road판(cbnu_internal_only_localtm_tags73.xodr, 서버에 이미
# 로드됨 - 재로드 안 함) 위에서 TM set_path 로 확정경로(15 road, 781.22m)를
# 유도하며 20km/h 로 주행한다.
#
# 세션5 대비 변경(세션6, 사용자 승인 2026-09-19):
#  - 경로 유도: TM set_path (권고 수준 - 갈림길 분기 선택만 유도). 계획 경로는
#    get_waypoint_xodr 로 만든 웨이포인트(약 4m 간격, 커넥터 안쪽 포함).
#  - 속도: tm.set_desired_speed(vehicle, 20) (km/h)
#  - 기록: tick(서버 프레임)마다 CSV append+flush. 통과 road 전이 목록을 계획과
#    대조. 실패 시 마지막 10초 road 전이 출력.
#  - CSV 저장 버그 수정: 예외로 끊겨도 그때까지의 데이터가 남는다.
#
# 세션9 추가(사용자 승인 2026-09-22, 물리접촉 vs TM제동 판별용):
#  - 궤적 CSV에 제어값(throttle/brake/steer/hand_brake/reverse)과 차체 자세각
#    (pitch/roll/vehicle_yaw) 컬럼 추가. 기존 14개 컬럼은 이름·순서 불변.
#  - 차량 bounding_box.extent 를 스폰 직후 1회 별도 파일에 기록(횡편차 계산의
#    반폭 가정값 0.85m을 실측으로 교체하는 용도).
#  - sensor.other.collision 부착, 별도 CSV에 이벤트 기록. 근거: CARLA 공식문서
#    (https://carla.readthedocs.io/en/latest/ref_sensors/#collision-detector)
#    "서버가 건물·덤불 등 정적 요소에도 semantic tag 조회용 'fake' actor를
#    만들어 충돌이 감지되게 한다" - 정적 지형(인도 연석) 충돌도 잡힌다.
#    콜백은 별도 스레드에서 돈다(공식문서) - 메인루프의 상대시간 t와 공유하면
#    스레드간 오차가 생길 수 있어, 이벤트 자체의 event.frame/event.timestamp를
#    기록해 메인 CSV와 frame으로 맞춘다. semantic_tags는 원시 숫자로 저장하고
#    이름은 분석 단계에서 공식 태그표로 붙인다.
#
# 세션13 추가(사용자 승인 2026-09-24, TM -> BasicAgent 전환):
#  - --controller agent|tm (기본 agent). tm 은 세션10~13 재현용으로 동작 불변
#    (비동기 wait_for_tick, TM set_path). agent 는 BasicAgent.set_global_plan 에
#    get_waypoint_xodr 로 확정한 계획 웨이포인트를 넣어 커넥터를 직접 지정한다.
#    TM set_path 는 갈림길에서 경로점과 가까운 쪽을 고르는 휴리스틱이라 U턴
#    커넥터(r1915/r1449/r1429/r1697)로 빠질 수 있었다(세션13 j60 이탈).
#  - agent 모드는 동기 모드 20Hz(fixed_delta_seconds=0.05). BasicAgent PID 는
#    dt 0.05초를 가정한다. 비동기(약 251Hz)면 적분·미분 항이 약 12배 틀어지고
#    프레임 간격이 실행마다 달라 재현성도 없다. 종료 시 finally 에서 반드시
#    원래(비동기) 설정으로 되돌린다. SIGABRT/SIGKILL 은 finally 가 안 돌므로
#    --restore-async 로 수동 복구.
#  - 신호·정지표지·차량 무시. 교내에 신호등이 없고(ODD) 이번 주행에 다른 차량이
#    없다. 배경 차량을 넣을 때 다시 볼 것.
#  - 급회전 커넥터 r1446/r1428/r1564 에서 차선 이탈량 기록(CSV 열 추가, 기존 22열 불변).
#    근거: https://carla.readthedocs.io/en/0.9.15/adv_agents/
#          https://carla.readthedocs.io/en/0.9.15/adv_synchrony_timestep/
#
# 세션14 추가(사용자 승인 2026-09-24):
#  - 판정 기준 변경. 세션5~14 의 "도로 이탈 3tick(차 중심이 Driving 차선 밖)" 중단
#    조건을 폐지. 세션14 r1446 에서 중심은 인도였으나 왼쪽 모서리는 주행 차선 위였고
#    반대 차선·메시 밖도 아니었다(인도를 문 상태). 차선 이탈은 측정 지표이지 중단
#    사유가 아니다. 기록만: 인도 침범, 차선 이탈, 꼭짓점의 반대 차선 침범 깊이.
#    중단(3tick 이상+0.3s 이상 지속): 차체 전체 주행 차선 밖(바닥 4꼭짓점 모두),
#    반대 차선 침범(차 **중심**이 계획 밖 road 의 진행방향 135° 넘게 다른 Driving 차선 위,
#    사용자 결정: 코너 뒤 모서리 0.19m 같은 흔한 침범은 지표, U턴 진입은 중심 기준으로도
#    잡힘), 메시 밖(중심에 lane 없음 또는 z 가 노면보다 0.5m 넘게 낮음).
#    on_road 열은 기존 정의(중심) 그대로 기록만.
#  - --conn-spacing M: 커넥터 계획 점 간격(기본 없음=전 구간 4m). BasicAgent
#    LocalPlanner 는 직선거리 < 3.0+0.5v(m/s) 인 점을 버리고 남은 첫 점을 목표로
#    삼는다. 점이 성기면 목표가 커넥터 끝으로 건너뛰어 코너를 자른다(세션14 r1446).
#  - --sharp-speed K: 60° 이상 커넥터가 15m 안이거나 그 위일 때 목표 속도 K km/h.
#  - CSV 7열 추가(기존 28열 불변). 상세 map/docs/map_session14_design_density_speed.md
#
# 실행: python map/tests/test_drive.py                  (서버 필요, agent)
#       python map/tests/test_drive.py --controller tm  (서버 필요, 세션10~13 재현)
#       python map/tests/test_drive.py --restore-async  (서버 동기 모드 수동 해제)
#       python map/tests/test_drive.py --dry-plan       (서버 불필요, 계획 표본만 계산)
#       python map/tests/test_drive.py --conn-spacing 1.0 [--sharp-speed 10]  (agent 전용)
#
# 좌표: CARLA world 좌표는 xodr y 부호 반전. get_waypoint_xodr 는 CARLA 좌표를
# 돌려주므로 변환 불필요(dry-plan 만 xodr 에서 직접 계산해 y 를 반전한다).
import argparse
import csv
import hashlib
import math
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import carla

HOST = "localhost"
PORT = 2000
TM_PORT = 8000
TIMEOUT = 20.0

# ---- 경로 (세션6 정정: junction36 커넥터 r1564 포함 15 road) ----
ROUTE_ROADS = [1247, 1914, 1356, 1917, 1355, 1446, 1172, 1428, 1240,
               1695, 1239, 1426, 1238, 1564, 1278]
LAST_ROAD_END_S = 100.38            # road1278 위 확정 종점 s
LANE_ID = -1                        # 전 road 오른쪽 driving 차선 1개뿐
PLAN_SPACING_M = 4.0                # 계획 웨이포인트 목표 간격
PLAN_GAP_WARN_M = 5.0               # TM 이 경로점을 소비하는 거리 ~5.5m
CONNECTOR_JUNCTION = {1914: 1, 1917: 107, 1446: 42, 1428: 10,
                      1695: 60, 1426: 84, 1564: 36}

# ---- 속도 / 실패 조건 ----
DESIRED_SPEED_KMH = 20.0
DRIVE_SECONDS = 400.0               # 실패조건 5: 시뮬 시간 초과(세션19 240->400, 폭주 방지용. 후문 목표 300s 의 약 1.3배)
RUNAWAY_SPEED_MPS = 15.0            # 실패조건 3: 초과 시 직접 destroy
STUCK_SPEED_MPS = 0.5               # 실패조건 4: 이 미만이
STUCK_SECONDS = 5.0                 #            이 시간 이상 지속
OFFROAD_TICKS = 3                   # 실패조건 2: 연속 tick 수 and
OFFROAD_MIN_SECONDS = 0.3           #            최소 지속 시간(고fps 오탐 방지)
ARRIVE_DIST_M = 5.0
# 세션21: 정지 근접 완주(record 판정만). 정체 정의(STUCK_SPEED_MPS 미만 STUCK_SECONDS 지속)를 그대로 써서
# "섰다" 를 정하고, 그 순간 정차점 이 거리 안이면 정체 실패 대신 완주. 정체 선언보다 먼저 본다.
# 근거: 세션20 양성재 재실행이 정차점 1.06m 앞(s=20.98, 완주 조건 s>=21 에 0.02m 모자람)에 서서 정체 실패로 기록됨.
# 2.0m = 기존 완주 임계 1.0m + 여유 1.0m. --strict 는 적용 안 함(세션14~18 규칙 그대로)
ARRIVE_STOP_DIST_M = 2.0
# 세션19 판정 재설계(기본). --strict 는 세션14~18 규칙 그대로. 종료와 기록을 나눈다.
# 종료: 충돌 / 정체 / 시간 초과 / 폭주 / 액터 소멸 / 메시 밖(낙하) / 경로 완전 이탈 / 깊은 반대 차선 침범
# 기록: 차선 이탈 구간(수, 최대 이탈량·방향, 그 road 평균R·R2min, 진입 속도)
# 거리 기준은 계획 경로 중심선(위치 조회 road_id 아님, 커넥터 겹침 오배정 방지 s18 B-3)
DEEP_OPP_M = 0.5                    # 깊은 반대 차선: 차 중심이 계획 차선 왼쪽 경계를 이만큼 넘고
DEEP_OPP_TICKS = 20                 #   연속 tick 수 and
DEEP_OPP_SECONDS = 1.0              #   지속 시간(근거: r1592 실패 0.23m/7tick 은 기록, s13 U턴 4.5m 는 종료)
LOST_M = 3.35                       # 경로 완전 이탈: 계획 중심선에서 차선폭 1개 넘게
LOST_TICKS = 40                     #   연속 tick 수 and
LOST_SECONDS = 2.0                  #   지속 시간
STRICT = False                      # --strict 이면 True (세션14~18 판정)
COLLISIONS = [0]                    # 충돌 콜백이 올리는 누적 건수(메인 루프가 읽음)
ARRIVE_MIN_S = 95.0
FAILURE_WINDOW_S = 10.0
MEM_RECORD_AT_S = 60.0
SPEED_PRINT_EVERY_S = 1.0

# ---- 세션13: BasicAgent / 동기 모드 / 급회전 이탈량 ----
AGENTS_PATH = Path.home() / "carla" / "CARLA_0.9.15" / "PythonAPI" / "carla"
FIXED_DELTA_S = 0.05                # 20Hz. BasicAgent PID dt 기본값과 같게
TICK_CHECK_AT_S = 10.0              # 이 시각에 tick 간격 1회 대조
SHARP_CONNECTORS = {1446: 78.6, 1428: -68.5, 1564: -82.2}  # 회전각(CARLA yaw, +=오른쪽)
# 맵세션18: --route. 정문(road1247) 출발 정차 루트. 위 전역값이 legacy(세션5~17 경로)이고
# select_route() 가 전역을 통째로 바꾼다(함수 본문은 전역 참조라 불변).
# 도로 체인은 경유 road 사이를 lane-1 next() 로 이은 것(scratchpad chain_s18.py, legacy 로 방법 검증).
# 새 루트 완주는 정차점 1m 앞(end_s - 1.0)부터. legacy 는 세션15~17 기준값 비교를 위해 95.0 유지.
ROUTES = {
    "legacy": dict(roads=list(ROUTE_ROADS), end_s=LAST_ROAD_END_S, arrive_min_s=ARRIVE_MIN_S,
                   connectors=dict(CONNECTOR_JUNCTION), sharp=dict(SHARP_CONNECTORS),
                   stop="road1278 s=100.38 도서관 서쪽 모서리(출입구 없음)"),
    "north": dict(roads=[1247, 1914, 1356, 1917, 1355, 1447, 1373, 1673, 1374, 1591, 1375,
                         1689, 1376, 1625, 1377, 1640, 1378],
                  end_s=78.0, arrive_min_s=77.0,
                  connectors={1914: 1, 1917: 107, 1447: 42, 1673: 58, 1591: 44, 1689: 59,
                              1625: 47, 1640: 51},
                  sharp={}, stop="도서관 북문(구관 북동 면 중점 46.1m)"),
    "south": dict(roads=[1247, 1914, 1356, 1917, 1355, 1446, 1172, 1428, 1240, 1695, 1239,
                         1426, 1238, 1563, 1237],
                  end_s=47.0, arrive_min_s=46.0,
                  connectors={1914: 1, 1917: 107, 1446: 42, 1428: 10, 1695: 60, 1426: 84,
                              1563: 36},
                  sharp={1446: 78.4, 1428: -68.1},
                  stop="도서관 남문(자연대-도서관 사이, 남서 면 중점 약 57.5m)"),
    "middle": dict(roads=[1247, 1914, 1356, 1917, 1355, 1447, 1373, 1673, 1374, 1592, 1283,
                          1617, 1284, 1606, 1285, 1597, 1286, 1838, 1327],
                   end_s=70.0, arrive_min_s=69.0,
                   connectors={1914: 1, 1917: 107, 1447: 42, 1673: 58, 1592: 44, 1617: 43,
                               1606: 45, 1597: 46, 1838: 48},
                   sharp={1592: -97.2, 1838: -71.4},
                   stop="중문 앞(road1327 막다른 끝 5.7m 전, 문까지 약 40~80m)"),
    # 세션20: 양성재(서문). 정차점 세션19 확정(s19_scope.log). 앞 7 road 는 south 와 같고 j10 에서
    # south 는 r1428 좌회전, 양성재는 r1427 직진. 통과형(r1123 끝 j5 에서 그대로 진행, 회차 없음)
    "yangseong": dict(roads=[1247, 1914, 1356, 1917, 1355, 1446, 1172, 1427, 1127, 1699, 1126,
                             1726, 1125, 1733, 1124, 1826, 1123],
                      end_s=22.0, arrive_min_s=21.0,
                      connectors={1914: 1, 1917: 107, 1446: 42, 1427: 10, 1699: 9, 1726: 8,
                                  1733: 7, 1826: 6},
                      sharp={1446: 78.4},
                      stop="양성재(서문) 앞(건물 외곽선 45.7m)"),
    # 세션23: 막다른 끝 회차 커넥터 통과 시험. legacy 15 road(세션5~17 반복 주행) + r1278 나머지 38m(직선)
    # + j35 회차 r1565(9.89m, -126.8°, 기존 24개와 같은 틀) + 출구 r1151(직선). 정차점이 아니라 시험용
    "uturn": dict(roads=list(ROUTE_ROADS) + [1565, 1151],
                  end_s=30.0, arrive_min_s=29.0,
                  connectors={**CONNECTOR_JUNCTION, 1565: 35},
                  sharp={**SHARP_CONNECTORS, 1565: -126.8},
                  stop="시험용(회차 r1565 통과 후 r1151 s=30)"),
}
ROUTE_NAME = "legacy"
# 맵세션32: v2 후보(tags81_B_smooth_turnfix, sha 5240ca8b883e42c0) 노선. 위 ROUTES 를 대응표
# logs/s28_road_id_map_B.csv 로 번역(형상 비용 0, next() 끊김 0, 출발·종점 좌표 차 0m: logs/s32_route_endpoints.log).
# junction 번호는 junction name 으로 번역. end_s·arrive_min_s·sharp 각도·stop 은 v1 그대로.
# back(후문, 첫 주행): 정문 r1278 -> 추정 게이트(36.624844, 127.462860) 최근접 r1151 s=23.1 의 최단 경로 33 road.
#   앞 15 road 가 south 와 같다. 경로 합 1368.2m(logs/s32_backgate_reach.log). sharp 는 south 와 같은 두 커넥터
#   (나머지 커넥터 끝점 회전각 최대 51.3° < 60°)
ROUTES_V2 = {
    "north": dict(roads=[1278, 1969, 1393, 1972, 1392, 1485, 1410, 1712, 1411, 1629, 1412, 1728,
                         1413, 1663, 1414, 1678, 1415],
                  end_s=78.0, arrive_min_s=77.0,
                  connectors={1969: 1, 1972: 109, 1485: 43, 1712: 59, 1629: 45, 1728: 60,
                              1663: 48, 1678: 52},
                  sharp={}, stop=ROUTES["north"]["stop"]),
    "south": dict(roads=[1278, 1969, 1393, 1972, 1392, 1484, 1202, 1466, 1271, 1734, 1270,
                         1464, 1269, 1601, 1268],
                  end_s=47.0, arrive_min_s=46.0,
                  connectors={1969: 1, 1972: 109, 1484: 43, 1466: 11, 1734: 61, 1464: 85,
                              1601: 37},
                  sharp={1484: 78.4, 1466: -68.1}, stop=ROUTES["south"]["stop"]),
    "middle": dict(roads=[1278, 1969, 1393, 1972, 1392, 1485, 1410, 1712, 1411, 1630, 1314,
                          1655, 1315, 1644, 1316, 1635, 1317, 1877, 1360],
                   end_s=70.0, arrive_min_s=69.0,
                   connectors={1969: 1, 1972: 109, 1485: 43, 1712: 59, 1630: 45, 1655: 44,
                               1644: 46, 1635: 47, 1877: 49},
                   sharp={1630: -97.2, 1877: -71.4}, stop=ROUTES["middle"]["stop"]),
    "yangseong": dict(roads=[1278, 1969, 1393, 1972, 1392, 1484, 1202, 1465, 1157, 1738, 1156,
                             1765, 1155, 1772, 1154, 1865, 1153],
                      end_s=22.0, arrive_min_s=21.0,
                      connectors={1969: 1, 1972: 109, 1484: 43, 1465: 11, 1738: 10, 1765: 9,
                                  1772: 8, 1865: 7},
                      sharp={1484: 78.4}, stop=ROUTES["yangseong"]["stop"]),
    "back": dict(roads=[1278, 1969, 1393, 1972, 1392, 1484, 1202, 1466, 1271, 1734, 1270,
                        1464, 1269, 1601, 1268, 1457, 1250, 1828, 1249, 1619, 1254, 1609,
                        1253, 1822, 1252, 1860, 1251, 1948, 1391, 1952, 1248, 1896, 1151],
                 end_s=23.1, arrive_min_s=22.1,
                 connectors={1969: 1, 1972: 109, 1484: 43, 1466: 11, 1734: 61, 1464: 85,
                             1601: 37, 1457: 107, 1828: 78, 1619: 41, 1609: 39, 1822: 86,
                             1860: 83, 1948: 108, 1952: 106, 1896: 3},
                 sharp={1484: 78.4, 1466: -68.1},
                 # 세션35: 노선 전용 감속(인자 없이 항상 적용). r1391 차선 중심 최소 반경 2.483m(s=9.00, s34_curvature.py)에서
                 # 횡가속도 2.5m/s^2(세션23 회차 r1565 통과값: 8km/h, R2min 1.94m -> 2.55m/s^2) -> sqrt(2.5*2.483)=2.49m/s
                 # =8.97km/h, 0.1 단위 내림 8.9km/h. 4 노선 road 와 교집합 없음(시작 시 검사)
                 slow={1391: 8.9},
                 # 세션36: 목표점 계측 road(이 road 위 tick 마다 target_log_<ts>.csv). aim = road 위에서만 쓸 겨냥 거리 기본값(m)
                 trace=[1948, 1391, 1952], aim={1391: 1.0},
                 # 세션38: 노선 전용 계획점 간격(m). s36_spacing_model.py 와 같은 배치(기본 첫 점부터 간격씩, 끝 0.3m 전까지)
                 # 예측 0.601m(간격 1m + b 1.0) 가 이 점 배치 기준. 등분 중앙점(18점)이면 같은 모형 0.398m
                 spacing={1391: 1.0},
                 stop="후문B 추정 게이트(392632034 위, 36.624844, 127.462860 최근접 1.65m)"),
}
FROZEN_V1_SHA16 = "bf835cdfad0cea65"
MAP_ROUTES = {FROZEN_V1_SHA16: ROUTES, "5240ca8b883e42c0": ROUTES_V2}
ACTIVE_ROUTES = ROUTES              # __main__ 에서 --xodr 해시로 고른다
ROUTE_SLOW = {}                     # 세션35: 노선 전용 감속 {road: km/h}. select_route 가 채운다
ROUTE_TRACE = set()                 # 세션36: 목표점 계측 road
ROUTE_AIM = {}                      # 세션36: 노선 전용 겨냥 거리 기본값 {road: base_min_distance m}
ROUTE_SPACING = {}                  # 세션38: 노선 전용 계획점 간격 {road: m}


def select_route(name):
    global ROUTE_NAME, ROUTE_ROADS, LAST_ROAD_END_S, ARRIVE_MIN_S, CONNECTOR_JUNCTION, \
        SHARP_CONNECTORS, ROUTE_SLOW, ROUTE_TRACE, ROUTE_AIM, ROUTE_SPACING
    r = ACTIVE_ROUTES[name]
    ROUTE_SLOW = dict(r.get("slow", {}))
    ROUTE_TRACE = set(r.get("trace", []))
    ROUTE_AIM = dict(r.get("aim", {}))
    ROUTE_SPACING = dict(r.get("spacing", {}))
    if ROUTE_SPACING:
        # 세션38: 계획점 간격 변경 대상도 4 노선 경로와 겹치면 중단
        others = set().union(*(set(v["roads"]) for k, v in ACTIVE_ROUTES.items()
                               if k in ("north", "south", "middle", "yangseong")))
        inter = sorted(set(ROUTE_SPACING) & others)
        print(f"  노선 전용 계획점 간격 {ROUTE_SPACING}, 4 노선 경로와 교집합 {inter}")
        if inter:
            raise SystemExit("노선 전용 계획점 간격 대상이 4 노선 경로에 있음 - 중단")
    if ROUTE_AIM:
        # 세션36: 겨냥 거리 하향 대상도 4 노선 경로와 겹치면 중단
        others = set().union(*(set(v["roads"]) for k, v in ACTIVE_ROUTES.items()
                               if k in ("north", "south", "middle", "yangseong")))
        inter = sorted(set(ROUTE_AIM) & others)
        print(f"  노선 전용 겨냥 거리 {ROUTE_AIM}, 4 노선 경로와 교집합 {inter}")
        if inter:
            raise SystemExit("노선 전용 겨냥 거리 대상이 4 노선 경로에 있음 - 중단")
    if ROUTE_TRACE:
        print(f"  목표점 계측 road {sorted(ROUTE_TRACE)}")
    if ROUTE_SLOW:
        # 세션35: 감속 대상이 다른 정차 노선 경로에 있으면 중단(다른 노선 동작이 바뀌면 안 된다)
        others = set().union(*(set(v["roads"]) for k, v in ACTIVE_ROUTES.items()
                               if k in ("north", "south", "middle", "yangseong")))
        inter = sorted(set(ROUTE_SLOW) & others)
        print(f"  노선 전용 감속 {ROUTE_SLOW}, 4 노선 경로와 교집합 {inter}")
        if inter:
            raise SystemExit("노선 전용 감속 대상이 4 노선 경로에 있음 - 중단")
    ROUTE_NAME = name
    ROUTE_ROADS = list(r["roads"])
    LAST_ROAD_END_S = r["end_s"]
    ARRIVE_MIN_S = r["arrive_min_s"]
    CONNECTOR_JUNCTION = dict(r["connectors"])
    SHARP_CONNECTORS = dict(r["sharp"])
    print(f"route={name}: {len(ROUTE_ROADS)} road {ROUTE_ROADS}")
    print(f"  종점 road{ROUTE_ROADS[-1]} lane{LANE_ID} s={LAST_ROAD_END_S} "
          f"(완주 s>={ARRIVE_MIN_S} 이고 종점 {ARRIVE_DIST_M:g}m 이내) - {r['stop']}")
    print(f"  커넥터 {CONNECTOR_JUNCTION}, 급회전 {SHARP_CONNECTORS or '없음'}")
SHARP_SAMPLE_M = 0.25               # 커넥터 차선 중심선 표본 간격
SHARP_NEAR_M = 3.0                  # 차 중심이 중심선에서 이 거리 안일 때만 측정
# 세션14: 중단 판정(연속 OFFROAD_TICKS tick 이상 + OFFROAD_MIN_SECONDS 이상 지속)
OPPOSITE_DEG = 135.0                # 차 진행방향과 이보다 크게 다른 차선 = 반대 방향
MESH_Z_DROP_M = 0.5                 # 투영 노면 z 보다 이만큼 낮으면 메시 밖(낙하)
# 메시 판정용 lane 종류. LaneType.Any(-2) 는 project_to_road=False 에서 Driving 위인데도
# None 을 돌려줘(세션14 r1446 s=3.7 실측) 쓰지 않고 종류별로 조회한다
MESH_LANE_TYPES = ("Driving", "Sidewalk", "Shoulder", "Border", "Parking", "Biking")
SLOW_TURN_DEG = 60.0                # --sharp-speed 적용 커넥터 기준 회전각
SLOW_AHEAD_M = 15.0                 # 대상 커넥터 계획점이 이 직선거리 안이면 감속
# 세션15: --follow. spectator 를 매 tick 차량 뒤·위로 옮긴다(관찰용 카메라).
# 제어·기록에 관여하지 않고, 동기 모드는 스텝이 FIXED_DELTA_S 로 고정이라 주행 결과가
# follow 유무와 무관하다(벽시계 시간만 늘어남). 차량 pitch/roll 은 빼 인도 턱에서 안 흔들리게.
FOLLOW_BACK_M = 8.0
FOLLOW_UP_M = 4.0
FOLLOW_PITCH_DEG = -math.degrees(math.atan2(FOLLOW_UP_M, FOLLOW_BACK_M))  # 약 -26.6°
# 세션20: --measure-stop. 완주 판정 뒤 바로 끝내지 않고 같은 제동(brake=1.0, hand_brake)을 유지한 채
# 속도가 STOP_SPEED_MPS 미만이 될 때까지 tick 을 더 돌려 실제 정지 위치를 잰다(기본 꺼짐 = 세션19 와 동일).
# 판정 뒤 tick 도 같은 CSV 에 이어 쓴다(37열 불변, 판정 열은 빈칸).
STOP_SPEED_MPS = 0.1
STOP_SETTLE_MAX_S = 15.0

BASE_DIR = Path(__file__).resolve().parent.parent
# 맵세션11: 곡선 평활화(smooth_osm_curves.py) 재변환판. road id 878개 전부 동일하게 유지됨
# (verify_smoothing.py 대응표), 경로 15 road id 와 종점 s=100.38(road1278 길이 불변)도 그대로.
# 대조군(평활화 전) xodr: cbnu_internal_only_localtm_tags73.xodr
XODR_PATH = BASE_DIR / "maps" / "cbnu_internal_only_localtm_tags73_smooth.xodr"
LOG_DIR = BASE_DIR / "docs" / "logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)

LOG_FIELDS = ["t", "frame", "x", "y", "z", "speed_mps", "speed_limit_kmh",
              "road_id", "lane_id", "s", "is_junction", "junction_id",
              "on_road", "in_plan",
              "throttle", "brake", "steer", "hand_brake", "reverse",
              "pitch", "roll", "vehicle_yaw",
              # 세션13 추가(tm 모드는 agent_* 빈칸)
              "agent_target_road", "agent_target_s",
              "sharp_conn", "sharp_s", "lat_off_m", "body_excess_m",
              # 세션14 추가(판정 분리·밀도/속도 분석)
              "n_corner_offlane", "sidewalk", "opposite_lane", "opposite_depth_m",
              "mesh_out",
              "target_dist_m", "min_dist_m", "set_speed_kmh",
              # 세션15 추가(반대 차선 판정 보완, opposite_lane=True 일 때만 값)
              "in_plan_lane"]
PLAN_FIELDS = ["idx", "road_id", "kind", "s", "x", "y", "z", "gap_to_prev_m"]
COLLISION_FIELDS = ["frame", "sim_timestamp", "t_rel",
                     "other_actor_id", "other_actor_type_id",
                     "other_actor_semantic_tags",
                     "impulse_x", "impulse_y", "impulse_z", "impulse_mag",
                     "loc_x", "loc_y", "loc_z"]


# ---------------- 계획 표본 (서버 불필요) ----------------

def file_sha16(path=None):
    """맵 파일 sha256 앞 16자(세션19, 로그 첫 줄용)."""
    return hashlib.sha256((path or XODR_PATH).read_bytes()).hexdigest()[:16]


def read_road_lengths():
    root = ET.parse(XODR_PATH).getroot()
    return {int(r.get("id")): float(r.get("length")) for r in root.iter("road")}


def plan_samples(lengths, min_s_on_first=None, conn_spacing=None):
    """(road_id, s) 목록. 각 road 를 n등분한 구간의 중앙점 -> 구간 경계를
    피해 인접 road 와 중복점이 생기지 않고, 짧은 커넥터도 최소 1점 포함.
    conn_spacing 이 있으면 커넥터 road 만 그 간격(세션14)."""
    out = []
    for rid in ROUTE_ROADS:
        length = LAST_ROAD_END_S if rid == ROUTE_ROADS[-1] else lengths[rid]
        spacing = conn_spacing if (conn_spacing and rid in CONNECTOR_JUNCTION) \
            else PLAN_SPACING_M
        n = max(1, math.ceil(length / spacing))
        if rid in ROUTE_SPACING:
            # 세션38: 기본 배치의 첫 점에서 시작해 간격씩, 끝 0.3m 전까지(s36_spacing_model.py 와 같다)
            s = 0.5 * length / n
            while s < length - 0.3:
                if not (rid == ROUTE_ROADS[0] and min_s_on_first is not None
                        and s <= min_s_on_first):
                    out.append((rid, s))
                s += ROUTE_SPACING[rid]
            continue
        for i in range(n):
            s = (i + 0.5) * length / n
            if rid == ROUTE_ROADS[0] and min_s_on_first is not None \
                    and s <= min_s_on_first:
                continue        # 스폰 지점보다 뒤에 있는 점은 불필요
            out.append((rid, s))
    return out


def xodr_point_at_s(road_el, s):
    """dry-plan 전용 근사: s 에서의 (x, y). paramPoly3 는 p=(s-gs)/len 근사."""
    for g in road_el.find("planView").findall("geometry"):
        gs, gl = float(g.get("s")), float(g.get("length"))
        if gs <= s <= gs + gl + 1e-9:
            p = (s - gs) / gl
            x0, y0, h = float(g.get("x")), float(g.get("y")), float(g.get("hdg"))
            pp = g.find("paramPoly3")
            if pp is None:
                return x0 + (s - gs) * math.cos(h), y0 + (s - gs) * math.sin(h)
            a = {k: float(pp.get(k)) for k in
                 ("aU", "bU", "cU", "dU", "aV", "bV", "cV", "dV")}
            u = a["aU"] + a["bU"] * p + a["cU"] * p ** 2 + a["dU"] * p ** 3
            v = a["aV"] + a["bV"] * p + a["cV"] * p ** 2 + a["dV"] * p ** 3
            return (x0 + u * math.cos(h) - v * math.sin(h),
                    y0 + u * math.sin(h) + v * math.cos(h))
    raise ValueError(f"s={s} 가 road 범위 밖")


def write_plan_csv(rows, path):
    prev = None
    for i, r in enumerate(rows):
        r["idx"] = i
        r["gap_to_prev_m"] = "" if prev is None else round(
            math.dist((prev["x"], prev["y"], prev["z"]),
                      (r["x"], r["y"], r["z"])), 2)
        prev = r
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=PLAN_FIELDS)
        w.writeheader()
        w.writerows(rows)


def summarize_plan(rows):
    print(f"계획 웨이포인트 총 {len(rows)}개")
    print("road   kind  n  s범위          최대간격(m)")
    gaps_all = []
    for rid in ROUTE_ROADS:
        rs = [r for r in rows if r["road_id"] == rid]
        gs = [r["gap_to_prev_m"] for r in rs if r["gap_to_prev_m"] != ""]
        gaps_all += gs
        kind = "conn" if rid in CONNECTOR_JUNCTION else "road"
        print(f"{rid:<6} {kind:<5} {len(rs):<2} "
              f"{rs[0]['s']:.2f}~{rs[-1]['s']:.2f}  "
              f"{max(gs) if gs else '-'}")
    conn_pts = sum(1 for r in rows if r["road_id"] in CONNECTOR_JUNCTION)
    missing = [c for c in CONNECTOR_JUNCTION
               if not any(r["road_id"] == c for r in rows)]
    print(f"커넥터 안쪽 점: {conn_pts}개 / 점 없는 커넥터: {missing or '없음'}")
    print(f"전체 최대 간격 {max(gaps_all):.2f}m, 평균 "
          f"{sum(gaps_all) / len(gaps_all):.2f}m")
    if max(gaps_all) > PLAN_GAP_WARN_M:
        print(f"경고: 간격 {PLAN_GAP_WARN_M}m 초과 구간 있음")


def dry_plan(conn_spacing=None):
    """서버 없이 계획 표본 s 와 xodr 근사 위치만 계산해 CSV 로 저장."""
    lengths = read_road_lengths()
    root = ET.parse(XODR_PATH).getroot()
    els = {int(r.get("id")): r for r in root.iter("road")}
    rows = []
    for rid, s in plan_samples(lengths, min_s_on_first=2.0, conn_spacing=conn_spacing):
        x, y = xodr_point_at_s(els[rid], s)
        rows.append({"road_id": rid,
                     "kind": "conn" if rid in CONNECTOR_JUNCTION else "road",
                     "s": round(s, 3), "x": round(x, 3), "y": round(-y, 3),
                     "z": 0.0})
    # 기본 계획 파일(세션11~14 기준)은 덮어쓰지 않도록 이름 분리
    # 세션18: legacy 외 루트는 drive_plan_dry_<route>[_conn<M>].csv
    tag = "" if ROUTE_NAME == "legacy" else f"_{ROUTE_NAME}"
    # 세션32: frozen_v1 이 아닌 맵은 맵 해시 앞 8자를 붙여 v1 CSV 를 덮어쓰지 않는다
    if file_sha16() != FROZEN_V1_SHA16:
        tag += f"_map{file_sha16()[:8]}"
    path = LOG_DIR / (f"drive_plan_dry{tag}.csv" if not conn_spacing
                      else f"drive_plan_dry{tag}_conn{conn_spacing:g}.csv")
    write_plan_csv(rows, path)
    print(f"[dry-plan] 위치는 xodr 근사(스폰 s 는 2.0m 로 가정). 저장: {path}")
    summarize_plan(rows)


# ---------------- 서버 사용 ----------------

def build_plan(carla_map, spawn_s, conn_spacing=None):
    lengths = read_road_lengths()
    rows, locs = [], []
    for rid, s in plan_samples(lengths, min_s_on_first=spawn_s + 1.0,
                               conn_spacing=conn_spacing):
        wp = carla_map.get_waypoint_xodr(rid, LANE_ID, s)
        if wp is None:      # 하나라도 없으면 스폰 전에 중단
            raise RuntimeError(
                f"get_waypoint_xodr(road={rid}, lane={LANE_ID}, s={s:.2f}) = None")
        loc = wp.transform.location
        locs.append(loc)
        rows.append({"road_id": rid,
                     "kind": "conn" if rid in CONNECTOR_JUNCTION else "road",
                     "s": round(s, 3), "x": round(loc.x, 3),
                     "y": round(loc.y, 3), "z": round(loc.z, 3)})
    return rows, locs


def check_plan_adjacency(carla_map):
    """인접 road 쌍마다 앞 road 끝에서 next() 했을 때 다음 road 가 나오는지."""
    lengths = read_road_lengths()
    bad = []
    for a, b in zip(ROUTE_ROADS, ROUTE_ROADS[1:]):
        end_s = LAST_ROAD_END_S if a == ROUTE_ROADS[-1] else lengths[a]
        wp = carla_map.get_waypoint_xodr(a, LANE_ID, max(end_s - 0.05, 0.0))
        nxt = {w.road_id for w in wp.next(1.0)} if wp else set()
        if b not in nxt:
            bad.append((a, b, sorted(nxt)))
    return bad


def find_plain_road_spawn_on_path(carla_map, target_xy, path_road_ids):
    # junction=False 이면서 경로 위 spawn point 중 경로 순서가 가장 앞선 것
    def d(sp):
        return math.hypot(sp.location.x - target_xy[0], sp.location.y - target_xy[1])
    cands = []
    for sp in carla_map.get_spawn_points():
        wp = carla_map.get_waypoint(sp.location, project_to_road=True)
        if wp.is_junction:
            continue
        if wp.road_id in path_road_ids:
            cands.append((path_road_ids.index(wp.road_id), d(sp), sp))
    if not cands:
        return None
    cands.sort(key=lambda c: (c[0], c[1]))
    return cands[0][2]


def cleanup_stray_vehicles(world):
    removed = []
    for v in world.get_actors().filter("vehicle.*"):
        removed.append(v.id)
        v.destroy()
    return removed


def is_on_road(carla_map, location):
    return carla_map.get_waypoint(location, project_to_road=False,
                                  lane_type=carla.LaneType.Driving) is not None


def classify_body(carla_map, vehicle, tf, plan_set):
    """세션14 판정. 차 중심과 bbox 바닥 4꼭짓점을 지도에 질의한다.
    n_corner_offlane = Driving 차선 위가 아닌 바닥 꼭짓점 수(0~4, 4 = 차체 전체 차선 밖)
    sidewalk      = 중심 또는 꼭짓점 하나라도 Sidewalk lane 위(기록만)
    반대 차선 = 계획 밖 road 의 Driving 차선이고 진행방향이 차 진행방향과 OPPOSITE_DEG 넘게
                다름(쌍둥이 half-road·반대 방향 커넥터)
    opposite_lane    = 차 **중심**이 반대 차선 위(기록). 중단은 in_plan_lane 이 False 일 때만(세션15)
    opposite_depth_m = 반대 차선 위 꼭짓점의 침범 깊이 최댓값(기록만, 0 = 없음). 깊이는 그
                       차선의 왼쪽 경계(쌍둥이와 공유하는 기준선 쪽)에서 잰 거리
                       = 차선폭/2 + 그 차선 중심에서의 횡거리(+오른쪽), 0~차선폭으로 자름.
                       junction 안에서 비스듬히 겹친 커넥터는 근사값
    mesh_out      = 중심에 MESH_LANE_TYPES 어느 lane 도 없음, 또는 z 가 그 노면보다
                    MESH_Z_DROP_M 넘게 낮음"""
    bb = vehicle.bounding_box
    ex, ey, ez = bb.extent.x, bb.extent.y, bb.extent.z
    pts = [tf.transform(carla.Location(bb.location.x + dx, bb.location.y + dy,
                                       bb.location.z - ez))
           for dx, dy in ((ex, -ey), (ex, ey), (-ex, -ey), (-ex, ey))]
    center = tf.location
    yaw = tf.rotation.yaw
    offlane, sidewalk, depth = 0, False, 0.0

    def opposite_wp(q):
        d = carla_map.get_waypoint(q, project_to_road=False,
                                   lane_type=carla.LaneType.Driving)
        if d is None or d.road_id in plan_set:
            return d, False
        diff = abs((d.transform.rotation.yaw - yaw + 180.0) % 360.0 - 180.0)
        return d, diff > OPPOSITE_DEG

    for q in pts:
        d, opp = opposite_wp(q)
        if d is None:
            offlane += 1
            sidewalk |= carla_map.get_waypoint(
                q, project_to_road=False, lane_type=carla.LaneType.Sidewalk) is not None
        elif opp:
            c, rv = d.transform.location, d.transform.get_right_vector()
            lat = (q.x - c.x) * rv.x + (q.y - c.y) * rv.y
            depth = max(depth, min(d.lane_width, max(0.0, d.lane_width / 2.0 + lat)))
    _, opposite = opposite_wp(center)
    c_wp = None
    for name in MESH_LANE_TYPES:
        c_wp = carla_map.get_waypoint(center, project_to_road=False,
                                      lane_type=getattr(carla.LaneType, name))
        if c_wp is not None:
            break
    sidewalk |= c_wp is not None and c_wp.lane_type == carla.LaneType.Sidewalk
    if c_wp is None:
        mesh_out = True
    else:
        mesh_out = c_wp.transform.location.z - center.z > MESH_Z_DROP_M
    return {"n_corner_offlane": offlane, "sidewalk": sidewalk,
            "opposite_lane": opposite, "opposite_depth_m": round(depth, 3),
            "mesh_out": mesh_out}


def record_bbox_once(vehicle, path):
    ext = vehicle.bounding_box.extent
    text = (f"extent_x(half_length)={ext.x:.4f}\n"
            f"extent_y(half_width)={ext.y:.4f}\n"
            f"extent_z(half_height)={ext.z:.4f}\n")
    path.write_text(text, encoding="utf-8")
    print(f"bounding box: half_length={ext.x:.3f}m half_width={ext.y:.3f}m "
          f"half_height={ext.z:.3f}m -> {path}")


def setup_collision_sensor(world, vehicle, path):
    """정적 지형 포함 충돌 감지(근거: CARLA 공식문서 ref_sensors 콜리전 항목).
    콜백은 별도 스레드에서 돈다 - t0(메인루프 상대시간)와 공유하지 않고
    event.frame/event.timestamp를 그대로 기록해 메인 CSV와 frame으로 맞춘다."""
    log_file = open(path, "w", newline="")
    writer = csv.DictWriter(log_file, fieldnames=COLLISION_FIELDS)
    writer.writeheader()
    log_file.flush()

    printed = []  # 첫 충돌 1건만 화면 출력(세션10: print 폭주 후 SIGABRT 재발 방지)

    def on_collision(event):
        imp = event.normal_impulse
        loc = event.transform.location
        row = {
            "frame": event.frame,
            "sim_timestamp": round(event.timestamp, 4),
            "t_rel": "",  # 스레드간 오차 우려로 비움. frame으로 메인CSV와 대조
            "other_actor_id": event.other_actor.id,
            "other_actor_type_id": event.other_actor.type_id,
            "other_actor_semantic_tags": ";".join(
                str(t) for t in event.other_actor.semantic_tags),
            "impulse_x": round(imp.x, 4), "impulse_y": round(imp.y, 4),
            "impulse_z": round(imp.z, 4),
            "impulse_mag": round(math.sqrt(imp.x**2 + imp.y**2 + imp.z**2), 4),
            "loc_x": round(loc.x, 3), "loc_y": round(loc.y, 3),
            "loc_z": round(loc.z, 3),
        }
        writer.writerow(row)
        log_file.flush()
        COLLISIONS[0] += 1
        if not printed:
            printed.append(1)
            print(f"[충돌] frame={event.frame} other={event.other_actor.type_id} "
                  f"tags={row['other_actor_semantic_tags']} impulse={row['impulse_mag']:.2f}"
                  " (이후 충돌은 CSV만 기록)")

    bp = world.get_blueprint_library().find("sensor.other.collision")
    sensor = world.spawn_actor(bp, carla.Transform(), attach_to=vehicle)
    sensor.listen(on_collision)
    print(f"충돌 센서 부착: actor_id={sensor.id} -> {path}")
    return sensor, log_file


def record_system_memory(label):
    free_out = subprocess.run(["free", "-h"], capture_output=True, text=True).stdout
    gtt = None
    for p in Path("/sys/class/drm").glob("card*/device/mem_info_gtt_used"):
        try:
            gtt = int(p.read_text().strip())
            break
        except OSError:
            continue
    print(f"--- 메모리 측정({label}) ---")
    print(free_out.strip())
    print(f"gtt_used: {gtt} bytes ({gtt / 1024**3:.2f}GiB)" if gtt else "gtt_used: 읽기 실패")


def print_last_seconds(transitions, t_end):
    print(f"\n=== 실패 직전 {FAILURE_WINDOW_S:.0f}초 road 전이 (t_end={t_end:.2f}s) ===")
    recent = [tr for tr in transitions if tr["t"] >= t_end - FAILURE_WINDOW_S]
    if not recent and transitions:
        recent = [transitions[-1]]
    for tr in recent:
        kind = "커넥터" if tr["junction"] else "일반"
        print(f"t={tr['t']:7.2f}s road{tr['road']:<5} {kind:<3} "
              f"speed={tr['speed']:5.2f}m/s ({tr['x']:.1f},{tr['y']:.1f})")
    if not recent:
        print("(전이 기록 없음)")


def compare_with_plan(transitions):
    actual = []
    for tr in transitions:
        if not actual or actual[-1] != tr["road"]:
            actual.append(tr["road"])
    print("\n=== 계획 vs 실제 road 시퀀스 ===")
    first_diff = None
    for i in range(max(len(ROUTE_ROADS), len(actual))):
        p = ROUTE_ROADS[i] if i < len(ROUTE_ROADS) else "-"
        a = actual[i] if i < len(actual) else "-"
        mark = "" if p == a else "  <-- 불일치"
        if p != a and first_diff is None:
            first_diff = i
        print(f"{i:2d} 계획 {p!s:<5} 실제 {a!s:<5}{mark}")
    print("첫 불일치 위치:", "없음" if first_diff is None else
          f"{first_diff}번째 (계획 {ROUTE_ROADS[first_diff] if first_diff < len(ROUTE_ROADS) else '-'}"
          f" / 실제 {actual[first_diff] if first_diff < len(actual) else '-'})")
    print("junction 통과(커넥터 road 를 실제로 밟았는지):")
    for c, j in CONNECTOR_JUNCTION.items():
        print(f"junction{j:<3} (r{c}): {'통과' if c in actual else '미통과'}")
    print("junction2(출발 전), junction35(종점 밖)는 통과 대상 아님")
    return actual


# ---------------- 세션13: 동기 모드 / BasicAgent / 급회전 이탈량 ----------------

def wants_sync(controller):
    """agent 만 동기 모드. tm 은 세션10~13 과 같은 비동기 그대로."""
    return controller == "agent"


def set_sync(world, original):
    """동기 모드 20Hz 로 전환. original 은 호출 전에 받아 둔 원래 설정(finally 복구용).
    적용 도중 예외가 나도 호출자가 이미 original 을 갖고 있어 복구할 수 있다."""
    print(f"[동기모드] 전환 전: synchronous_mode={original.synchronous_mode} "
          f"fixed_delta_seconds={original.fixed_delta_seconds}")
    s = world.get_settings()
    s.synchronous_mode = True
    s.fixed_delta_seconds = FIXED_DELTA_S
    world.apply_settings(s)
    now = world.get_settings()
    print(f"[동기모드] 전환 후: synchronous_mode={now.synchronous_mode} "
          f"fixed_delta_seconds={now.fixed_delta_seconds}")


def restore_settings(world, original):
    """원래 설정으로 복귀. 실패해도 예외를 밖으로 내지 않고 수동 복구를 안내한다."""
    try:
        world.apply_settings(original)
        now = world.get_settings()
        print(f"[동기모드] 복귀: synchronous_mode={now.synchronous_mode} "
              f"fixed_delta_seconds={now.fixed_delta_seconds}")
    except Exception as e:  # noqa: BLE001 - 어떤 오류든 복구 안내가 우선
        print(f"[동기모드] 복귀 실패: {e} -> "
              "python map/tests/test_drive.py --restore-async 실행할 것")


def restore_async_only():
    """--restore-async: 차량 없이 서버 동기 모드만 해제(SIGABRT 등 수동 복구용)."""
    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)
    world = client.get_world()
    s = world.get_settings()
    print(f"[동기모드] 현재: synchronous_mode={s.synchronous_mode} "
          f"fixed_delta_seconds={s.fixed_delta_seconds}")
    s.synchronous_mode = False
    s.fixed_delta_seconds = None
    world.apply_settings(s)
    now = world.get_settings()
    print(f"[동기모드] 해제 후: synchronous_mode={now.synchronous_mode} "
          f"fixed_delta_seconds={now.fixed_delta_seconds}")


def plan_waypoints(carla_map, rows):
    """build_plan rows 의 (road_id, s) 로 웨이포인트를 다시 확정해 BasicAgent 계획으로."""
    from agents.navigation.local_planner import RoadOption
    plan = []
    for r in rows:
        wp = carla_map.get_waypoint_xodr(r["road_id"], LANE_ID, r["s"])
        if wp is None:
            raise RuntimeError(f"get_waypoint_xodr(road={r['road_id']}, s={r['s']}) = None")
        plan.append((wp, RoadOption.LANEFOLLOW))
    return plan


def make_agent(vehicle, carla_map, plan):
    from agents.navigation.basic_agent import BasicAgent
    # 신호·정지표지·차량 무시: 교내 신호등 없음(ODD), 이번 주행에 다른 차량 없음
    opt = {"ignore_traffic_lights": True, "ignore_stop_signs": True,
           "ignore_vehicles": True, "dt": FIXED_DELTA_S}
    agent = BasicAgent(vehicle, target_speed=DESIRED_SPEED_KMH, opt_dict=opt,
                       map_inst=carla_map)
    agent.set_target_speed(DESIRED_SPEED_KMH)
    agent.set_global_plan(plan, stop_waypoint_creation=True, clean_queue=True)
    print(f"BasicAgent set_global_plan {len(plan)}점, target_speed={DESIRED_SPEED_KMH}km/h, "
          f"dt={FIXED_DELTA_S}")
    return agent


def sharp_polylines(carla_map, roads=None):
    """road 별 차선(lane -1) 중심선 표본: (s, x, y, 오른쪽단위벡터, 차선폭).
    기본은 급회전 커넥터, 세션15 부터 계획 15 road 전체에도 쓴다.
    세션18: 기본값을 정의 시점이 아니라 호출 시점 SHARP_CONNECTORS 로(--route 반영)."""
    if roads is None:
        roads = SHARP_CONNECTORS
    lengths = read_road_lengths()
    polys = {}
    for rid in roads:
        pts, s = [], 0.0
        while s <= lengths[rid]:
            wp = carla_map.get_waypoint_xodr(rid, LANE_ID, s)
            if wp is not None:
                loc = wp.transform.location
                rv = wp.transform.get_right_vector()
                n = math.hypot(rv.x, rv.y) or 1.0
                pts.append((s, loc.x, loc.y, rv.x / n, rv.y / n, wp.lane_width))
            s += SHARP_SAMPLE_M
        polys[rid] = pts
    return polys


def _lateral(pts, x, y):
    """(x,y) 에 가장 가까운 표본 index 와, 그 표본 기준 부호 있는 횡거리(+=오른쪽)."""
    i = min(range(len(pts)), key=lambda k: (pts[k][1] - x) ** 2 + (pts[k][2] - y) ** 2)
    _, px, py, rx, ry, _ = pts[i]
    return i, (x - px) * rx + (y - py) * ry


def in_plan_lane(polys, loc):
    """세션15 반대 차선 판정 보완. loc 가 계획 road 어느 하나의 차선(lane -1) 안이면 True.
    = 가장 가까운 중심선 표본까지 거리 <= 그 표본 차선폭/2 (표본 간격 SHARP_SAMPLE_M).
    junction 안에서 계획 커넥터와 반대 방향 커넥터의 차선 영역이 겹치면(세션15 r1428/r1434)
    get_waypoint 는 한쪽만 돌려준다. 차 중심이 자기 계획 차선 안이면 반대 차선으로 치지 않는다."""
    for pts in polys.values():
        for _, px, py, _, _, w in pts:
            if (px - loc.x) ** 2 + (py - loc.y) ** 2 <= (w / 2.0) ** 2:
                return True
    return False


def sharp_offset(polys, vehicle, tf):
    """이탈량 정의(지표로 고정):
    lat_off_m    = 차량 bbox 중심의, 계획 커넥터 차선 중심선에서의 부호 있는 수직거리
                   (+ = 진행 방향 오른쪽)
    body_excess_m = 차량 bbox 8개 꼭짓점 각각의 |중심선 수직거리| - 차선폭/2 의 최댓값
                   (차선 경계 기준, + = 그 꼭짓점이 차선 밖으로 나간 거리 m)
    차 중심이 커넥터 중심선 SHARP_NEAR_M 안이고 가장 가까운 표본이 양 끝점이 아닐 때만
    측정(커넥터 구간 안). 해당 없으면 None."""
    bb = vehicle.bounding_box
    c = tf.transform(carla.Location(bb.location.x, bb.location.y, bb.location.z))
    for rid, pts in polys.items():
        if len(pts) < 3:
            continue
        i, lat = _lateral(pts, c.x, c.y)
        if i in (0, len(pts) - 1) or abs(lat) > SHARP_NEAR_M:
            continue
        excess, side_lat = -1e9, 0.0
        for v in bb.get_world_vertices(tf):
            _, vl = _lateral(pts, v.x, v.y)
            e = abs(vl) - pts[i][5] / 2.0
            if e > excess:
                excess, side_lat = e, vl
        return {"conn": rid, "s": pts[i][0], "lat": lat, "excess": excess,
                "side_lat": side_lat}
    return None


def summarize_sharp(stats, collision_path):
    """급회전 커넥터별 요약. 충돌은 커넥터 구간 frame 범위로 대조."""
    frames = []
    if collision_path is not None and collision_path.exists():
        with collision_path.open() as f:
            frames = [int(r["frame"]) for r in csv.DictReader(f)]
    print("\n=== 급회전 커넥터 이탈량 (정의: sharp_offset docstring) ===")
    for rid, turn in SHARP_CONNECTORS.items():
        st = stats.get(rid)
        if not st:
            print(f"r{rid} ({turn:+.1f}°): 측정 없음(미도달 또는 중심선 {SHARP_NEAR_M}m 밖)")
            continue
        side = "안쪽" if (st["side_lat"] > 0) == (turn > 0) else "바깥쪽"
        n_col = sum(1 for fr in frames if st["f0"] <= fr <= st["f1"])
        print(f"r{rid} ({turn:+.1f}°): tick {st['n']}, 최대|lat| {st['max_abs_lat']:.2f}m, "
              f"최대 차체초과 {st['max_excess']:+.2f}m({side}, s={st['s_at_max']:.1f}), "
              f"on_road False {st['offroad']}tick, 충돌 {n_col}건")


def plan_track(flat, loc):
    """세션19. 계획 경로 중심선(0.25m 표본) 기준 위치. 반환 (road, lat, 거리, 차선폭).
    lat + = 진행방향 오른쪽(인도 쪽), - = 왼쪽(중앙선·반대 방향 쪽)."""
    best, bi = 1e18, 0
    for k, (_, p) in enumerate(flat):
        d = (p[1] - loc.x) ** 2 + (p[2] - loc.y) ** 2
        if d < best:
            best, bi = d, k
    rid, (_, px, py, rx, ry, w) = flat[bi]
    return rid, (loc.x - px) * rx + (loc.y - py) * ry, math.sqrt(best), w


_LENGTHS = {}


def road_radius(carla_map, rid):
    """평균R = 길이/|끝점 회전|, R2min = 2m 창 최소 반경(세션11·s18 A2 정의, 0.5m 간격, 양끝 1m 제외)."""
    wps = carla_map.get_waypoint_xodr
    a = wps(rid, LANE_ID, 0.001)
    # 길이는 xodr 에서 읽는다(서버 맵과 같은 파일인지는 첫 줄·server_map 해시로 확인)
    if not _LENGTHS:
        _LENGTHS.update(read_road_lengths())
    L = _LENGTHS.get(rid, 0.0)
    b = wps(rid, LANE_ID, max(L - 0.001, 0.0))
    if a is None or b is None or L <= 0:
        return float("nan"), float("nan")
    turn = abs(math.radians((b.transform.rotation.yaw - a.transform.rotation.yaw + 180) % 360 - 180))
    avg = L / turn if turn > 1e-6 else float("inf")
    r2, s = float("inf"), 1.0
    while s <= L - 1.0:
        p, q = wps(rid, LANE_ID, s - 1.0), wps(rid, LANE_ID, s + 1.0)
        k = abs((q.transform.rotation.yaw - p.transform.rotation.yaw + 180) % 360 - 180) / 2.0
        if k > 1e-6:
            r2 = min(r2, 1.0 / math.radians(k))
        s += 0.5
    return avg, r2


def summarize_departures(segs, radius):
    """세션19 기록: 차선 이탈 구간(차체 기준). 방향 + = 인도 쪽, - = 중앙선 쪽."""
    print("\n=== 차선 이탈 구간 (차체 기준, 계획 중심선 기준, 세션19) ===")
    if not segs:
        print("없음")
        return
    for g in segs:
        avg, r2 = radius.get(g["road"], (float("nan"), float("nan")))
        print(f"t={g['t0']:.2f}~{g['t1']:.2f}s road{g['road']} {'+인도' if g['side'] > 0 else '-중앙선'} 쪽 "
              f"최대 lat {g['lat']:+.2f}m 차체초과 {g['body']:.2f}m 중심초과 {max(g['center'], 0):.2f}m "
              f"{g['n']}tick, 진입 {g['v0'] * 3.6:.1f}km/h, road 평균R {avg:.2f} R2min {r2:.2f}")
    left = [g for g in segs if g["side"] < 0]
    right = [g for g in segs if g["side"] > 0]
    print(f"합계 {len(segs)}구간 (중앙선 쪽 {len(left)}, 인도 쪽 {len(right)}), "
          f"최대 차체초과 {max(g['body'] for g in segs):.2f}m")


def summarize_body(body_stats):
    """세션14 판정 열의 road 별 요약(기록만 조건 포함)."""
    print("\n=== 차체 판정 road 별 (tick 수, 최대 차선밖 꼭짓점) ===")
    if not body_stats:
        print("기록 없음")
    for rid, b in body_stats.items():
        if b["sidewalk"] or b["offlane_max"] or b["opposite"] or b["mesh_out"] \
                or b["depth_max"]:
            print(f"road{rid}: 인도 {b['sidewalk']}tick, 차선밖꼭짓점 최대 {b['offlane_max']}/4 "
                  f"(1개 이상 {b['offlane_any']}tick), 반대차선(중심) {b['opposite']}tick, "
                  f"(계획 차선 안 {b['opp_inplan']}tick), "
                  f"꼭짓점 침범깊이 최대 {b['depth_max']:.2f}m, 메시밖 {b['mesh_out']}tick")
    print("(목록에 없는 road 는 전부 0)")


def follow_transform(tf):
    """차량 transform -> 뒤 FOLLOW_BACK_M, 위 FOLLOW_UP_M 에서 차량을 보는 spectator transform.
    방향은 차량 yaw 만 쓴다."""
    yaw = math.radians(tf.rotation.yaw)
    loc = carla.Location(x=tf.location.x - FOLLOW_BACK_M * math.cos(yaw),
                         y=tf.location.y - FOLLOW_BACK_M * math.sin(yaw),
                         z=tf.location.z + FOLLOW_UP_M)
    return carla.Transform(loc, carla.Rotation(pitch=FOLLOW_PITCH_DEG, yaw=tf.rotation.yaw))


def settle_and_measure(world, vehicle, end_tf, t_judge, judge_loc, log_writer, log_file, t0,
                       spectator):
    """세션20 --measure-stop. 완주 판정 직후 호출(제동은 이미 적용, 1 tick 진행된 상태).
    반환 dict: 정지까지 시간·거리, 최종 위치의 정차점 대비 오차.
    오차 부호: 종방향 + = 정차점을 지나침(진행방향 앞), 횡방향 + = 오른쪽(인도 쪽)."""
    fwd, right = end_tf.get_forward_vector(), end_tf.get_right_vector()
    e = end_tf.location
    # 판정 tick 에 제동 명령 후 이미 1 tick 진행됨 -> 시각은 t_judge + 1 tick, 거리는 판정 위치부터
    prev = judge_loc
    path_m, t_now, speed, ticks = 0.0, t_judge + FIXED_DELTA_S, None, 1
    while True:
        tf = vehicle.get_transform()
        vel = vehicle.get_velocity()
        speed = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
        path_m += tf.location.distance(prev)
        prev = tf.location
        if speed < STOP_SPEED_MPS or t_now - t_judge >= STOP_SETTLE_MAX_S:
            break
        world.tick()
        ticks += 1
        snap = world.get_snapshot()
        t_now = snap.timestamp.elapsed_seconds - t0
        tf = vehicle.get_transform()
        if spectator is not None:
            spectator.set_transform(follow_transform(tf))
        ctrl = vehicle.get_control()
        v = vehicle.get_velocity()
        log_writer.writerow({"t": round(t_now, 3), "frame": snap.frame,
                             "x": round(tf.location.x, 3), "y": round(tf.location.y, 3),
                             "z": round(tf.location.z, 3),
                             "speed_mps": round(math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2), 3),
                             "throttle": round(ctrl.throttle, 3), "brake": round(ctrl.brake, 3),
                             "steer": round(ctrl.steer, 3), "hand_brake": ctrl.hand_brake,
                             "reverse": ctrl.reverse, "vehicle_yaw": round(tf.rotation.yaw, 3)})
        log_file.flush()
    loc = vehicle.get_transform().location
    dx, dy = loc.x - e.x, loc.y - e.y
    return {"stopped": speed < STOP_SPEED_MPS, "t_stop": t_now, "dt": t_now - t_judge,
            "ticks": ticks, "path_m": path_m, "final_speed": speed,
            "err_m": math.hypot(dx, dy), "lon_m": dx * fwd.x + dy * fwd.y,
            "lat_m": dx * right.x + dy * right.y}


def main(controller="agent", conn_spacing=None, sharp_speed=None, follow=True,
         measure_stop=False):
    use_sync = wants_sync(controller)
    print(f"controller={controller}, 동기모드={'켬 ' + str(FIXED_DELTA_S) + 's' if use_sync else '끔(비동기)'}")
    print(f"conn_spacing={conn_spacing or PLAN_SPACING_M}m"
          f"{'(커넥터만)' if conn_spacing else '(전 구간)'}, "
          f"sharp_speed={str(sharp_speed) + 'km/h' if sharp_speed else '없음'}")
    print(f"follow={'on' if follow else 'off'}"
          f"{f'(뒤 {FOLLOW_BACK_M:g}m, 위 {FOLLOW_UP_M:g}m, 매 tick)' if follow else ''}")
    if measure_stop:
        print(f"measure_stop=on (판정 뒤 제동 유지, 속도<{STOP_SPEED_MPS}m/s 또는 "
              f"{STOP_SETTLE_MAX_S:g}s 까지)")
    if controller == "agent" and str(AGENTS_PATH) not in sys.path:
        sys.path.insert(0, str(AGENTS_PATH))
    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)
    world = client.get_world()
    carla_map = world.get_map()
    print(f"맵: {carla_map.name}, spawn point {len(carla_map.get_spawn_points())}개")
    # 세션19: 서버에 실제로 올라간 맵(to_opendrive)의 해시. 첫 줄의 파일 해시와 대조
    srv16 = hashlib.sha256(carla_map.to_opendrive().encode("utf-8")).hexdigest()[:16]
    print(f"server_map_sha256={srv16} "
          f"({'파일과 일치' if srv16 == file_sha16() else '파일과 불일치 - 다른 맵이 로드됨'})")

    removed = cleanup_stray_vehicles(world)
    if removed:
        print(f"기존 차량 액터 제거: {removed}")

    spawn = find_plain_road_spawn_on_path(carla_map, (221.40, -1086.84), ROUTE_ROADS)
    if spawn is None:
        raise RuntimeError("경로 위 plain road spawn point 없음")
    spawn_wp = carla_map.get_waypoint(spawn.location, project_to_road=True)
    print(f"spawn: {spawn.location} road{spawn_wp.road_id} lane{spawn_wp.lane_id} "
          f"s={spawn_wp.s:.2f}")

    # ---- 계획 만들기 (스폰 전, 실패 시 여기서 중단) ----
    plan_rows, plan_locs = build_plan(carla_map, spawn_wp.s, conn_spacing)
    ts = time.strftime("%Y%m%d_%H%M%S")
    plan_path = LOG_DIR / f"drive_plan_{ts}.csv"
    write_plan_csv(plan_rows, plan_path)
    print(f"계획 CSV: {plan_path}")
    summarize_plan(plan_rows)
    bad = check_plan_adjacency(carla_map)
    if bad:
        print(f"경고: 인접 road 연결 확인 실패 {bad}")
    else:
        print(f"인접 road 연결 확인: {len(ROUTE_ROADS) - 1}쌍 모두 next() 로 이어짐")
    end_tf = carla_map.get_waypoint_xodr(ROUTE_ROADS[-1], LANE_ID, LAST_ROAD_END_S).transform
    end_loc = end_tf.location
    plan_set = set(ROUTE_ROADS)
    agent_plan = plan_waypoints(carla_map, plan_rows) if controller == "agent" else None
    sharp_polys = sharp_polylines(carla_map)
    plan_polys = sharp_polylines(carla_map, ROUTE_ROADS)
    plan_flat = [(rid, p) for rid, pts in plan_polys.items() for p in pts]
    route_radius = {} if STRICT else {r: road_radius(carla_map, r) for r in ROUTE_ROADS}
    dep_segs = []                   # 세션19 차선 이탈 구간 기록
    print(f"판정: {'strict(세션14~18 규칙)' if STRICT else 'record(세션19, 종료/기록 분리)'}")
    sharp_stats = {}
    body_stats = {}
    slow_roads = {r for r, a in SHARP_CONNECTORS.items() if abs(a) >= SLOW_TURN_DEG}
    set_speed = DESIRED_SPEED_KMH if controller == "agent" else None
    sharp_entry = {}                # 급회전 커넥터 첫 tick 속도(m/s)

    # ---- CSV 는 tick 마다 append+flush ----
    log_path = LOG_DIR / f"drive_log_{ts}.csv"
    log_file = open(log_path, "w", newline="")
    log_writer = csv.DictWriter(log_file, fieldnames=LOG_FIELDS)
    log_writer.writeheader()
    log_file.flush()
    print(f"궤적 로그(tick마다 기록): {log_path}")
    # 세션36: 목표점 계측(ROUTE_TRACE road 위에서만). 기존 drive_log 열은 그대로 두려고 별도 파일
    trace_file = trace_writer = None
    if ROUTE_TRACE and controller == "agent":
        trace_path = LOG_DIR / f"target_log_{ts}.csv"
        trace_file = open(trace_path, "w", newline="")
        trace_writer = csv.writer(trace_file)
        trace_writer.writerow(["t", "x", "y", "road_id", "s", "speed_mps", "min_distance_m", "base_min_distance_m",
                               "target_road", "target_s", "target_x", "target_y", "target_dist_m", "steer"])
        print(f"목표점 로그: {trace_path}")

    vehicle = None
    collision_sensor = None
    collision_log_file = None
    transitions = []
    fail_reason = None
    arrived = False
    arrive_kind = None              # 세션21: "정차점 통과" | "정지 근접"
    diverged_note = None
    t_now = 0.0
    collision_path = None
    original_settings = None        # 동기 모드 전환 시에만 채워짐
    agent = None
    tm = None
    stop_meas = None                # 세션20 --measure-stop 결과
    try:
        bp = world.get_blueprint_library().find("vehicle.audi.a2")
        vehicle = world.spawn_actor(bp, spawn)
        print(f"차량 스폰: actor_id={vehicle.id}")

        bbox_path = LOG_DIR / f"vehicle_bbox_{ts}.txt"
        record_bbox_once(vehicle, bbox_path)

        collision_path = LOG_DIR / f"collision_events_{ts}.csv"
        collision_sensor, collision_log_file = setup_collision_sensor(
            world, vehicle, collision_path)

        if controller == "tm":
            tm = client.get_trafficmanager(TM_PORT)
            vehicle.set_autopilot(True, tm.get_port())
            tm.set_path(vehicle, plan_locs)
            tm.set_desired_speed(vehicle, DESIRED_SPEED_KMH)
            print(f"TM set_path {len(plan_locs)}점, desired_speed={DESIRED_SPEED_KMH}km/h")
        else:
            agent = make_agent(vehicle, carla_map, agent_plan)

        if use_sync:
            original_settings = world.get_settings()
            set_sync(world, original_settings)
            world.tick()
            first = world.get_snapshot()
        else:
            first = world.wait_for_tick()
        t0 = first.timestamp.elapsed_seconds
        frame0 = first.frame
        wall0 = time.monotonic()
        tick_checked = False
        done_warned = False
        aim_default = None          # 세션36: LocalPlanner 원래 base_min_distance(첫 사용 때 읽음)
        last_road = None
        plan_idx = -1               # 계획에서 마지막으로 매칭된 road 위치
        stop_run = {}               # 중단 조건별 (연속 tick, 시작 t)
        COLLISIONS[0] = 0
        half_w = vehicle.bounding_box.extent.y
        cur_seg = None
        stuck_t0 = None
        mem_done = False
        next_print = 0.0
        spectator = world.get_spectator() if follow else None

        while True:
            try:
                if use_sync:
                    world.tick()
                    snap = world.get_snapshot()
                else:
                    snap = world.wait_for_tick()
                t_now = snap.timestamp.elapsed_seconds - t0
                if not vehicle.is_alive:
                    raise RuntimeError("vehicle.is_alive=False")
                tf = vehicle.get_transform()
                vel = vehicle.get_velocity()
                speed_limit = vehicle.get_speed_limit()
                ctrl = vehicle.get_control()
                if spectator is not None:
                    spectator.set_transform(follow_transform(tf))
            except RuntimeError as e:
                # 실패조건 1: 액터 소멸(서버 자동 제거 포함)/서버 오류
                fail_reason = f"액터 소멸 또는 서버 오류: {e}"
                break
            loc = tf.location
            speed = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
            on_road = is_on_road(carla_map, loc)
            wp = carla_map.get_waypoint(loc, project_to_road=True,
                                        lane_type=carla.LaneType.Driving)
            road_id, lane_id, s_val = wp.road_id, wp.lane_id, wp.s
            in_plan = road_id in plan_set
            junction_id = wp.get_junction().id if wp.is_junction else -1

            rot = tf.rotation
            row = {"t": round(t_now, 3), "frame": snap.frame,
                   "x": round(loc.x, 3), "y": round(loc.y, 3), "z": round(loc.z, 3),
                   "speed_mps": round(speed, 3),
                   "speed_limit_kmh": round(speed_limit, 1),
                   "road_id": road_id, "lane_id": lane_id, "s": round(s_val, 2),
                   "is_junction": wp.is_junction, "junction_id": junction_id,
                   "on_road": on_road, "in_plan": in_plan,
                   "throttle": round(ctrl.throttle, 3),
                   "brake": round(ctrl.brake, 3),
                   "steer": round(ctrl.steer, 3),
                   "hand_brake": ctrl.hand_brake, "reverse": ctrl.reverse,
                   "pitch": round(rot.pitch, 3), "roll": round(rot.roll, 3),
                   "vehicle_yaw": round(rot.yaw, 3)}
            tgt = agent.get_local_planner().target_waypoint if agent else None
            row["agent_target_road"] = tgt.road_id if tgt else ""
            row["agent_target_s"] = round(tgt.s, 2) if tgt else ""
            so = sharp_offset(sharp_polys, vehicle, tf)
            if so:
                row.update({"sharp_conn": so["conn"], "sharp_s": round(so["s"], 2),
                            "lat_off_m": round(so["lat"], 3),
                            "body_excess_m": round(so["excess"], 3)})
                st = sharp_stats.setdefault(so["conn"], {
                    "n": 0, "max_abs_lat": 0.0, "max_excess": -1e9, "s_at_max": 0.0,
                    "side_lat": 0.0, "offroad": 0, "f0": snap.frame, "f1": snap.frame})
                st["n"] += 1
                st["f1"] = snap.frame
                st["max_abs_lat"] = max(st["max_abs_lat"], abs(so["lat"]))
                if so["excess"] > st["max_excess"]:
                    st.update(max_excess=so["excess"], s_at_max=so["s"],
                              side_lat=so["side_lat"])
                st["offroad"] += 0 if on_road else 1
            else:
                row.update({"sharp_conn": "", "sharp_s": "", "lat_off_m": "",
                            "body_excess_m": ""})
            body = classify_body(carla_map, vehicle, tf, plan_set)
            body["in_plan_lane"] = in_plan_lane(plan_polys, loc) \
                if body["opposite_lane"] else ""
            row.update(body)
            lp = agent.get_local_planner() if agent else None
            row["target_dist_m"] = round(loc.distance(tgt.transform.location), 2) if tgt else ""
            md = getattr(lp, "_min_distance", None) if lp else None   # 직전 run_step 값
            row["min_dist_m"] = round(md, 2) if md is not None else ""
            row["set_speed_kmh"] = set_speed if set_speed is not None else ""
            b = body_stats.setdefault(road_id, {"sidewalk": 0, "offlane_max": 0,
                                                "offlane_any": 0, "opposite": 0,
                                                "opp_inplan": 0,
                                                "depth_max": 0.0, "mesh_out": 0})
            b["sidewalk"] += body["sidewalk"]
            b["offlane_max"] = max(b["offlane_max"], body["n_corner_offlane"])
            b["offlane_any"] += body["n_corner_offlane"] > 0
            b["opposite"] += body["opposite_lane"]
            b["opp_inplan"] += body["in_plan_lane"] is True
            b["depth_max"] = max(b["depth_max"], body["opposite_depth_m"])
            b["mesh_out"] += body["mesh_out"]
            if road_id in SHARP_CONNECTORS and road_id not in sharp_entry:
                sharp_entry[road_id] = (t_now, speed)
            log_writer.writerow(row)
            log_file.flush()

            if road_id != last_road:
                transitions.append({"t": t_now, "road": road_id,
                                    "junction": wp.is_junction, "speed": speed,
                                    "x": loc.x, "y": loc.y})
                print(f"[전이] t={t_now:7.2f}s road{road_id} "
                      f"{'커넥터' if wp.is_junction else '일반'} "
                      f"speed={speed:5.2f}m/s limit={speed_limit:.0f}km/h")
                last_road = road_id
                if diverged_note is None:
                    # 계획의 현재/다음 road 가 아니면 비치명 경고(1회)
                    ok = (plan_idx >= 0 and road_id == ROUTE_ROADS[plan_idx]) or \
                         (plan_idx + 1 < len(ROUTE_ROADS)
                          and road_id == ROUTE_ROADS[plan_idx + 1])
                    if ok and plan_idx + 1 < len(ROUTE_ROADS) \
                            and road_id == ROUTE_ROADS[plan_idx + 1]:
                        plan_idx += 1
                    elif not ok:
                        diverged_note = (f"t={t_now:.2f}s road{road_id} 는 계획의 "
                                         f"현재/다음 road 가 아님(비치명, 계속 주행)")
                        print("[계획 이탈] " + diverged_note)

            if t_now >= next_print:
                print(f"t={t_now:6.1f}s ({loc.x:8.2f},{loc.y:8.2f}) "
                      f"speed={speed:5.2f}m/s road{road_id} s={s_val:6.1f} "
                      f"on_road={on_road}")
                next_print = t_now + SPEED_PRINT_EVERY_S

            if not mem_done and t_now >= MEM_RECORD_AT_S:
                record_system_memory("주행중(차량1대+TM set_path)" if controller == "tm"
                                     else "주행중(차량1대+BasicAgent, 동기 20Hz)")
                mem_done = True

            if not tick_checked and t_now >= TICK_CHECK_AT_S:
                n_fr = snap.frame - frame0
                wall = time.monotonic() - wall0
                print(f"[tick 대조] fixed_delta={world.get_settings().fixed_delta_seconds} "
                      f"delta_seconds={snap.timestamp.delta_seconds:.4f} "
                      f"시뮬경과/프레임={t_now / max(n_fr, 1):.4f}s "
                      f"벽시계 {n_fr / max(wall, 1e-6):.1f} tick/s (프레임 {n_fr})")
                tick_checked = True

            # 완주: 마지막 road 위 s>=ARRIVE_MIN_S 이고 종점 5m 이내(legacy: road1278 s>=95)
            # 세션21: OR 정지 근접(정체 조건 충족 순간 정차점 ARRIVE_STOP_DIST_M 이내). 아래 정체 판정보다 먼저 본다
            pass_end = road_id == ROUTE_ROADS[-1] and s_val >= ARRIVE_MIN_S \
                and loc.distance(end_loc) <= ARRIVE_DIST_M
            stop_near = (not STRICT and speed < STUCK_SPEED_MPS and stuck_t0 is not None
                         and t_now - stuck_t0 >= STUCK_SECONDS
                         and loc.distance(end_loc) <= ARRIVE_STOP_DIST_M)
            if pass_end or stop_near:
                arrived = True
                arrive_kind = "정차점 통과" if pass_end else "정지 근접"
                if stop_near and not pass_end:
                    print(f"[정지 근접 완주] 속도<{STUCK_SPEED_MPS}m/s 가 t={stuck_t0:.2f}s 부터 "
                          f"{t_now - stuck_t0:.1f}s, road{road_id} s={s_val:.2f}, "
                          f"정차점까지 {loc.distance(end_loc):.2f}m <= {ARRIVE_STOP_DIST_M:g}m")
                if tm is not None:
                    vehicle.set_autopilot(False, tm.get_port())
                vehicle.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
                if use_sync:
                    world.tick()    # 제동을 반영하고 멈춤
                print(f"완주: t={t_now:.1f}s 종점까지 {loc.distance(end_loc):.2f}m")
                if measure_stop and use_sync:
                    d_j = loc - end_loc
                    fv = end_tf.get_forward_vector()
                    stop_meas = {"t_judge": t_now, "x": loc.x, "y": loc.y, "v_judge": speed,
                                 "d_judge": loc.distance(end_loc),
                                 "lon_judge": d_j.x * fv.x + d_j.y * fv.y}
                    stop_meas.update(settle_and_measure(world, vehicle, end_tf, t_now, loc,
                                                        log_writer, log_file, t0, spectator))
                elif measure_stop:
                    print("measure_stop: 비동기(tm) 모드는 지원 안 함 - 건너뜀")
                break

            # 실패조건 3: 폭주 -> 직접 destroy
            if speed > RUNAWAY_SPEED_MPS:
                fail_reason = (f"폭주 속도 {speed:.1f}m/s > {RUNAWAY_SPEED_MPS}m/s "
                               f"(road{road_id}) - 직접 destroy")
                try:
                    vehicle.destroy()
                except RuntimeError:
                    pass
                vehicle = None
                break

            if STRICT:
                # 실패조건 2(세션14 변경): 차체 전체 차선 밖 / 반대 차선 / 메시 밖 이
                # 연속 OFFROAD_TICKS tick 이상 + OFFROAD_MIN_SECONDS 이상. 인도·차선 이탈은 기록만
                conds = {"차체 전체 주행 차선 밖": (body["n_corner_offlane"] == 4,
                                                    OFFROAD_TICKS, OFFROAD_MIN_SECONDS),
                         "반대 차선 침범(중심)": (body["opposite_lane"]
                                                  and not body["in_plan_lane"],
                                                  OFFROAD_TICKS, OFFROAD_MIN_SECONDS),
                         "메시 밖": (body["mesh_out"], OFFROAD_TICKS, OFFROAD_MIN_SECONDS)}
            else:
                # 세션19: 계획 중심선 기준. 기록(차선 이탈 구간)은 종료와 별개로 쌓는다
                p_road, p_lat, p_dist, p_w = plan_track(plan_flat, loc)
                body_over = abs(p_lat) + half_w - p_w / 2.0
                if body_over > 0:
                    side = 1 if p_lat > 0 else -1
                    if cur_seg is None or cur_seg["side"] != side:
                        cur_seg = {"t0": t_now, "t1": t_now, "road": p_road, "side": side,
                                   "lat": p_lat, "body": body_over, "center": abs(p_lat) - p_w / 2.0,
                                   "n": 0, "v0": speed}
                        dep_segs.append(cur_seg)
                    cur_seg["t1"], cur_seg["n"] = t_now, cur_seg["n"] + 1
                    if body_over > cur_seg["body"]:
                        cur_seg.update(lat=p_lat, body=body_over, road=p_road,
                                       center=abs(p_lat) - p_w / 2.0)
                else:
                    cur_seg = None
                conds = {"충돌": (COLLISIONS[0] > 0, 1, 0.0),
                         "깊은 반대 차선 침범": (-p_lat - p_w / 2.0 >= DEEP_OPP_M,
                                                 DEEP_OPP_TICKS, DEEP_OPP_SECONDS),
                         "경로 완전 이탈": (p_dist > LOST_M, LOST_TICKS, LOST_SECONDS),
                         "메시 밖": (body["mesh_out"], OFFROAD_TICKS, OFFROAD_MIN_SECONDS)}
            for name, (hit, need_n, need_s) in conds.items():
                if not hit:
                    stop_run.pop(name, None)
                    continue
                n, t_s = stop_run.get(name, (0, t_now))
                stop_run[name] = (n + 1, t_s)
                if n + 1 >= need_n and t_now - t_s >= need_s:
                    fail_reason = (f"{name} {n + 1}tick/{t_now - t_s:.2f}s 연속 "
                                   f"(road{road_id} s={s_val:.1f})")
            if fail_reason:
                break

            # 실패조건 4: 정체
            if speed < STUCK_SPEED_MPS:
                stuck_t0 = t_now if stuck_t0 is None else stuck_t0
                if t_now - stuck_t0 >= STUCK_SECONDS:
                    fail_reason = (f"정체 속도<{STUCK_SPEED_MPS}m/s "
                                   f"{t_now - stuck_t0:.1f}s 지속 (road{road_id})")
                    break
            else:
                stuck_t0 = None

            # 실패조건 5: 시간 초과
            if t_now >= DRIVE_SECONDS:
                fail_reason = f"시간 초과 {DRIVE_SECONDS:.0f}s (완주 못 함, road{road_id})"
                break

            # agent: 다음 tick 에 쓸 제어 계산·적용 (tm 은 서버측 autopilot)
            if agent is not None:
                if sharp_speed or ROUTE_SLOW:
                    # 세션35: --sharp-speed 대상과 노선 전용 감속(ROUTE_SLOW)을 한 표로. sharp_speed 만 있을 때 동작은 전과 같다
                    targets = {r: sharp_speed for r in slow_roads} if sharp_speed else {}
                    targets.update(ROUTE_SLOW)
                    hit = targets.get(road_id)
                    if hit is None:
                        for w, _ in lp.get_plan():
                            dist = loc.distance(w.transform.location)
                            if w.road_id in targets and dist <= SLOW_AHEAD_M:
                                hit = targets[w.road_id]
                                break
                            if dist > 3 * SLOW_AHEAD_M:
                                break
                    near = hit is not None
                    want = hit if near else DESIRED_SPEED_KMH
                    if want != set_speed:
                        print(f"[{'감속' if near else '복귀'}] t={t_now:.2f}s road{road_id} "
                              f"{set_speed:g}->{want:g}km/h speed={speed * 3.6:.1f}km/h")
                        agent.set_target_speed(want)
                        set_speed = want
                if agent.done() and not done_warned:
                    print(f"[계획 소진] t={t_now:.2f}s 완주 전에 계획 웨이포인트가 모두 "
                          "소진됨(이후 제동, 정체 조건으로 판정)")
                    done_warned = True
                if ROUTE_AIM:
                    # 세션36: 겨냥 거리 기본값을 대상 road 위에서만 바꾸고 벗어나면 원래 값(LocalPlanner 기본 3.0)으로.
                    # LocalPlanner 에 setter 가 없어 opt_dict 와 같은 이름의 속성 _base_min_distance 를 직접 쓴다
                    if aim_default is None:
                        aim_default = lp._base_min_distance
                    want_aim = ROUTE_AIM.get(road_id, aim_default)
                    if want_aim != lp._base_min_distance:
                        print(f"[겨냥] t={t_now:.2f}s road{road_id} base_min_distance "
                              f"{lp._base_min_distance:g}->{want_aim:g}m")
                        lp._base_min_distance = want_aim
                ctl = agent.run_step()
                vehicle.apply_control(ctl)
                if trace_writer is not None and road_id in ROUTE_TRACE:
                    tw = lp.target_waypoint
                    tl = tw.transform.location if tw is not None else None
                    trace_writer.writerow([round(t_now, 3), round(loc.x, 3), round(loc.y, 3), road_id,
                                           round(s_val, 3) if s_val is not None else "", round(speed, 3),
                                           round(lp._min_distance, 3), lp._base_min_distance,
                                           tw.road_id if tw else "", round(tw.s, 3) if tw else "",
                                           round(tl.x, 3) if tl else "", round(tl.y, 3) if tl else "",
                                           round(loc.distance(tl), 3) if tl else "", round(ctl.steer, 4)])

        print("\n=== 요약 ===")
        print(f"route: {ROUTE_NAME} (종점 road{ROUTE_ROADS[-1]} s={LAST_ROAD_END_S})")
        print(f"결과: {'완주' if arrived else '실패 - ' + str(fail_reason)}"
              f"{' (정지 근접 판정, 세션21)' if arrive_kind == '정지 근접' else ''}")
        print(f"주행 시간(시뮬): {t_now:.1f}s")
        if diverged_note:
            print(f"계획 이탈 기록: {diverged_note}")
        compare_with_plan(transitions)
        summarize_sharp(sharp_stats, collision_path)
        for rid, (te, ve) in sharp_entry.items():
            print(f"r{rid} 진입: t={te:.2f}s speed={ve:.2f}m/s({ve * 3.6:.1f}km/h)")
        summarize_body(body_stats)
        if not STRICT:
            summarize_departures(dep_segs, route_radius)
            print(f"충돌 누적 {COLLISIONS[0]}건")
        print(f"판정: {'strict' if STRICT else 'record'}")
        print(f"follow={'on' if follow else 'off'}")
        if stop_meas:
            m = stop_meas
            print("\n=== 실제 정지 위치 (세션20 --measure-stop) ===")
            print(f"판정 시점: t={m['t_judge']:.2f}s 위치({m['x']:.2f},{m['y']:.2f}) "
                  f"속도 {m['v_judge']:.2f}m/s({m['v_judge'] * 3.6:.1f}km/h) "
                  f"정차점까지 {m['d_judge']:.2f}m (종방향 {m['lon_judge']:+.2f}m)")
            print(f"정지: {'완료' if m['stopped'] else '미완(상한 도달)'} t={m['t_stop']:.2f}s, "
                  f"판정부터 {m['dt']:.2f}s({m['ticks']}tick) {m['path_m']:.2f}m, "
                  f"최종 속도 {m['final_speed']:.3f}m/s")
            print(f"최종 정지 위치 오차: {m['err_m']:.2f}m (종방향 {m['lon_m']:+.2f}m "
                  f"+=지나침, 횡방향 {m['lat_m']:+.2f}m +=인도 쪽)")
            if m["dt"] > 0:
                print(f"판정~정지 평균 감속도 {m['v_judge'] / m['dt']:.2f}m/s^2 "
                      f"(판정 tick 에 제동 명령)")
        if not arrived:
            print_last_seconds(transitions, t_now)

    except Exception as e:
        print(f"예외 발생: {e}")
        if transitions:
            print_last_seconds(transitions, t_now)
        raise
    finally:
        try:
            log_file.close()
            if trace_file is not None:
                trace_file.close()
            print(f"궤적 로그: {log_path}")

            # 정리 순서: 센서(차량에 부착) 먼저 stop->destroy, 그 다음 vehicle.
            # 세션5처럼 서버가 액터를 먼저 제거했을 경우에 대비해 각각 try/except.
            if collision_sensor is not None:
                try:
                    collision_sensor.stop()
                except RuntimeError:
                    print("충돌 센서 stop 실패(이미 정리된 상태로 추정)")
                try:
                    collision_sensor.destroy()
                    print(f"충돌 센서(actor_id={collision_sensor.id}) 제거함")
                except RuntimeError:
                    print("충돌 센서가 이미 서버측에서 제거된 상태")
            if collision_log_file is not None:
                collision_log_file.close()

            if vehicle is not None and not arrived:
                try:
                    vehicle.destroy()
                    print(f"실패/예외로 차량(actor_id={vehicle.id}) 제거함")
                except RuntimeError:
                    print("차량이 이미 서버측에서 제거된 상태")
            elif arrived:
                print(f"정상 종료 - 차량(actor_id={vehicle.id}) 브레이크 유지로 남겨둠")
        finally:
            # 동기 모드였으면 반드시 원래 설정으로 복귀(클라이언트가 동기 모드로 남긴 채
            # 끝나면 서버가 tick 을 기다리며 멈춘다). 제거 반영용 tick 1회 후 복귀.
            if original_settings is not None:
                try:
                    world.tick()
                except Exception as e:  # noqa: BLE001
                    print(f"[동기모드] 복귀 전 tick 실패: {e}")
                restore_settings(world, original_settings)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="확정 경로 주행 시험(세션13: agent 기본)")
    ap.add_argument("--controller", choices=["agent", "tm"], default="agent",
                    help="agent=BasicAgent+동기 20Hz(기본), tm=TM set_path+비동기(세션10~13 재현)")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--dry-plan", action="store_true", help="서버 없이 계획 표본만 계산")
    g.add_argument("--restore-async", action="store_true",
                   help="서버 동기 모드만 해제하고 종료(비정상 종료 뒤 수동 복구)")
    ap.add_argument("--conn-spacing", type=float, default=None, metavar="M",
                    help=f"커넥터 계획 점 간격 m(agent 전용, 기본 전 구간 {PLAN_SPACING_M}m)")
    ap.add_argument("--sharp-speed", type=float, default=None, metavar="K",
                    help=f"{SLOW_TURN_DEG:g}° 이상 커넥터 목표 속도 km/h(agent 전용, 기본 감속 없음)")
    ap.add_argument("--follow", choices=["on", "off"], default="on",
                    help="spectator 가 매 tick 차량 뒤 8m·위 4m 에서 추종(기본 on, 결과 불변)")
    ap.add_argument("--route", choices=["north", "south", "middle", "yangseong", "back", "legacy", "uturn"],
                    default="legacy",
                    help="정차 루트(세션18). legacy=세션5~17 경로(기본), back=v2 맵에서만(세션32)")
    ap.add_argument("--xodr", type=Path, default=None,
                    help="맵 xodr(세션32). 기본 frozen_v1. 해시로 노선 표를 고른다(MAP_ROUTES)")
    ap.add_argument("--strict", action="store_true",
                    help="세션14~18 판정 규칙 그대로(기본: 세션19 종료/기록 분리 판정)")
    ap.add_argument("--measure-stop", action="store_true",
                    help="완주 판정 뒤 제동 유지하며 완전 정지까지 돌려 실제 정지 위치 오차 기록"
                         "(세션20, 기본 꺼짐 = 판정 순간 종료)")
    args = ap.parse_args()
    STRICT = args.strict
    if args.xodr is not None:
        XODR_PATH = args.xodr.resolve()
    # 세션19: 두 맵 병행 대비. 로그 첫 줄에 맵 파일 sha256 앞 16자
    print(f"map_sha256={file_sha16()} ({XODR_PATH.name})")
    if file_sha16() not in MAP_ROUTES:
        print(f"노선 표 없는 맵(sha {file_sha16()}). 알려진 맵: {sorted(MAP_ROUTES)}. 실행하지 않고 종료")
        sys.exit(2)
    ACTIVE_ROUTES = MAP_ROUTES[file_sha16()]
    if args.route not in ACTIVE_ROUTES and not args.restore_async:
        print(f"route={args.route}: 이 맵 노선 표에 없음({sorted(ACTIVE_ROUTES)}). 실행하지 않고 종료")
        sys.exit(2)
    if args.route == "back" and ACTIVE_ROUTES is ROUTES:
        print("route=back: 재변환필요 - 후문 way 가 xodr 에 없음(road1333 고립, "
              "map/docs/map_session18_report.md). 실행하지 않고 종료")
        sys.exit(2)
    if not args.restore_async:
        select_route(args.route)
    if args.sharp_speed and not SHARP_CONNECTORS:
        print(f"경고: route={args.route} 에는 급회전 커넥터가 없어 --sharp-speed 대상 없음")
    if args.controller == "tm" and (args.conn_spacing or args.sharp_speed):
        ap.error("--conn-spacing/--sharp-speed 는 agent 전용(tm 은 세션10~13 재현용)")
    if args.conn_spacing is not None and args.conn_spacing <= 0:
        ap.error("--conn-spacing 은 0보다 커야 함")
    if args.dry_plan:
        dry_plan(args.conn_spacing)
    elif args.restore_async:
        restore_async_only()
    else:
        main(args.controller, args.conn_spacing, args.sharp_speed, args.follow == "on",
             args.measure_stop)
