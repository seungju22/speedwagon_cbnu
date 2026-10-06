# 맵 세션14 설계: 커넥터 계획 밀도(1차) + 급회전 감속(2차)

사용자 승인(2026-09-24): (a)안 채택, 한 세션에서 조건을 바꿔 2회 주행.
1차 = 커넥터 계획 점 1m 간격만. 2차 = 1차가 이탈하면 거기에 급회전 커넥터 10km/h 추가.

## 1. 설계 전 확인(읽기전용)

### 1-1. 이탈 판정 시점의 차 위치
- 판정: `is_on_road`(test_drive.py:277) = `get_waypoint(차 중심, project_to_road=False,
  lane_type=Driving)` 가 None. 연속 3tick 이상 + 0.3s 이상이면 실패(OFFROAD_TICKS/MIN_SECONDS)
- 로그의 road_id/lane_id(1446/-1)는 project_to_road=True 투영값이라 위치 판별에 못 씀
- 오프라인 지도 판정(docs/logs/session14_offroad_where.log, 스크립트 scratchpad s14_offroad_where.py):
  - t=52.00~52.30 차 중심: Driving 없음, Any = r1446 lane -2 **Sidewalk**(r1446 자신의 인도, 폭 2.80m)
  - 오른쪽 모서리(FR/RR): r1446/-2 Sidewalk. 왼쪽 모서리(FL/RL): r1446/-1 Driving
  - 즉 차 오른쪽 절반이 우회전 안쪽 모서리 인도 위. 인도 안쪽 가장자리에서 약 0.4~0.8m 들어감
- 반대 차선 아님: 우회전 안쪽은 자기 인도 방향이고 반대 방향 차선과 무관
- 메시 밖 아님: 인도는 메시 일부(연석 0.15m). roll -7.4°, z 최대 0.13 = 오른쪽 바퀴가 턱 위
- junction42 주행 가능 면: 커넥터 19쌍 중 Driving 16, Sidewalk 3(r1435/r1439/r1442/r1446 의 -2).
  우회전 안쪽에서 주행 가능 면은 r1446 lane -1 의 오른쪽 경계(중심 +1.675m)까지. 그 밖은 인도
- 심각도: 시뮬레이션상 물리 차단·낙하는 없음. 실제 셔틀이라면 보행 공간 침범이라 안전상 실패

### 1-2. sharp_offset 이탈량 정의 (test_drive.py:486 docstring, 코드와 일치 확인)
- lat_off_m: **차선 중심 기준**. 차량 bbox 중심의 계획 커넥터 lane -1 중심선까지 부호 있는
  수직거리. + = 진행 방향 오른쪽. 중심선은 0.25m 간격 표본, 최근접 표본의 right vector 로 부호
- body_excess_m: **차선 경계 기준**. bbox 바닥 포함 8꼭짓점 각각 (|중심선 수직거리| - 차선폭/2)의
  최댓값. + = 그 꼭짓점이 차선 밖으로 나간 거리
- 수치 검산(t=52.30): 차선폭 3.35 -> 반폭 1.675. RR 모서리 = 인도 중심(1.675+1.40=3.075)에서
  0.65 -> 중심선에서 약 2.43m -> 초과 약 0.75. 기록값 0.756 과 일치
- 측정 조건: 차 중심이 중심선 3.0m 안, 최근접 표본이 커넥터 양 끝점이 아닐 때만

### 1-3. BasicAgent 목표점 선택 (0.9.15 agents/navigation 소스)
- 별도 lookahead 파라미터 없음. LocalPlanner.run_step(local_planner.py:239~256):
  큐 앞쪽부터 차와의 **직선거리 < min_distance** 인 점을 버리고, 남은 첫 점이 목표점
- min_distance = base_min_distance 3.0 + distance_ratio 0.5 x 속도(m/s). 마지막 점만 1m
- 횡 제어(controller.py:218~): 차 전방 벡터와 "차->목표점" 벡터의 각도를 PID(K_P 1.95, K_I 0.05,
  K_D 0.2, dt 0.05), 조향 상한 0.8
- 따라서 실효 lookahead 는 min_distance(20km/h 에서 약 5.8m)이고, 점 간격은
  "min_distance 를 넘는 첫 점이 얼마나 더 멀리 있나"를 정한다. 촘촘하게 하면 목표점이
  약 min_distance 거리로 고정되고, 성기면 커넥터 끝으로 건너뛴다(세션14 실측: s=1.6 에서 목표 s=12.17)
- 속도를 낮추면 min_distance 도 줄어든다(10km/h 에서 약 4.4m). 즉 2차는 속도와 lookahead 를
  함께 바꾼다. 둘을 가르려고 min_distance 를 CSV 에 기록한다(4절)
- 기하 참고(추정, 예측값 아님): r1446 길이 13.91m, 회전 78.6° -> R 약 10.1m. 현(chord)의 처짐
  R - sqrt(R^2-(c/2)^2): 세션14 목표(s1.6->12.17, 현 약 11.6m) 1.81m <-> 실측 1.63m.
  1차(현 약 5.8m) 약 0.42m, 2차(약 4.4m) 약 0.24m. 차선 반폭 1.675 - 차 반폭 0.894 = 0.78m 여유

### 1-4. 구간별 속도 변경 가능 여부
- BasicAgent.set_target_speed(km/h) -> LocalPlanner.set_speed 가 `_target_speed` 만 바꾼다.
  follow_speed_limits 는 기본 False(make_agent 에서 안 켬)라 덮어쓰지 않는다
- run_step 마다 `_target_speed` 를 읽으므로 tick 단위로 바꿀 수 있다. 가능

## 2. 변경 범위: test_drive.py 만

### 2-1. 인자 (기본값 = 세션14 동작 그대로, 재현성 유지)
- `--conn-spacing M` : 커넥터 road 계획 점 간격(m). 미지정 = 전 구간 4m(세션14 와 동일)
- `--sharp-speed K`  : 급회전 커넥터 목표 속도(km/h). 미지정 = 감속 없음
- 둘 다 agent 전용. `--controller tm` 과 같이 주면 오류로 종료(TM 계획은 세션10~13 재현용)
- 1차: `python map/tests/test_drive.py --conn-spacing 1.0`
- 2차: `python map/tests/test_drive.py --conn-spacing 1.0 --sharp-speed 10`

### 2-2. plan_samples(lengths, min_s_on_first, conn_spacing=None)
- road 가 CONNECTOR_JUNCTION 에 있고 conn_spacing 이 있으면 n = ceil(L / conn_spacing), 아니면 기존 4m
- 중앙점 방식(구간 경계 회피) 유지. r1446 은 4점 -> 14점
- --dry-plan 도 같은 인자를 받아 계획 CSV 만 만든다(서버 없이 확인용)

### 2-3. 급회전 감속 (--sharp-speed 있을 때만)
- 대상: SHARP_CONNECTORS 중 |회전각| >= 60° -> r1446, r1428, r1564 (셋 다 해당, r1917 38° 제외)
- 매 tick 판정: (a) 차 중심 투영 road 가 대상이거나 (b) LocalPlanner 큐(get_plan)에서
  차와의 직선거리 SLOW_AHEAD_M=15m 안의 점 중 대상 road 가 있으면 감속, 아니면 20km/h
- 15m 근거: 20km/h 에서 약 2.7초. 종방향 PID max_brake 0.3 이라 감속이 완만함. 실제 진입 속도를
  CSV 로 확인한다(충분하지 않으면 기록만 하고 이번 세션에선 안 바꿈)
- 속도 전환 시 한 줄 출력: `[감속] t=.. r1446 20->10km/h`, `[복귀] ...`

### 2-4. CSV 열 추가 (기존 28열 불변, 뒤에 3열)
- target_dist_m : 차 중심 -> 현재 목표점 직선거리
- min_dist_m    : LocalPlanner._min_distance (비공개 속성, 없으면 공백)
- set_speed_kmh : 그 tick 의 목표 속도
- 이 3열로 "점 간격 효과"(목표점 건너뜀 유무)와 "속도 효과"(min_distance 감소 포함)를 가른다

### 2-5. 요약 출력 추가
- 급회전 커넥터별 진입 속도(커넥터 첫 tick), 최대 target_dist_m
- 기존 이탈량 요약·실패 판정·finally 순서 불변

### 2-6. 바꾸지 않는 것
- (변경됨, 사용자 결정) 실패 판정은 세션14 판정 기준 변경을 따른다. map_session14_report.md "판정 기준 변경" 참조. 1~3차 모두 같은 새 기준
- base_min_distance / distance_ratio / PID 계수: 기본값 유지
- xodr, test_load_map_878.py, 벽 0

## 3. 실행 순서 (다음 세션, 재부팅 후)
- (갱신, 사용자 결정) 1차 = 판정 기준만 변경(인자 없음), 2차 = --conn-spacing 1.0, 3차 = --conn-spacing 1.0 --sharp-speed 10. 아래 원안은 참고용
1. 재부팅 확인 -> 서버 기동(사용자) -> 기본 맵 2분 -> `--wall-height 0` 로드 -> 2분
2. 사전점검(1m 계획 점 수·최대 간격·U턴 미포함)
3. 1차 주행. 완주하면 2차 안 함
4. 1차가 이탈하면 메모리 확인(중단 기준) 후 2차 주행. 맵 재로드 없음
5. 결과 비교: r1446/r1428/r1564 lat·excess·진입 속도·target_dist, 진행 거리

## 4. 검증 (서버 없이, 작성 후)
- py_compile, --help
- 인자 없이 --dry-plan: 기존 drive_plan_dry.csv(sha256 82643554) 와 바이트 동일(기본 동작 불변 확인)
- --dry-plan --conn-spacing 1.0: 커넥터 점 수·간격 출력
- --controller tm --conn-spacing 1.0: 오류로 종료
