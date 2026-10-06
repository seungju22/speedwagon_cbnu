# 맵 세션14 보고: BasicAgent 주행 검증 (2026-09-24)

## 요약
- 조건: 평활화 xodr(sha256 bf835cdfad0cea65), wall_height=0, agent 모드, 동기 20Hz, 20km/h
- 결과: 실패. r1446(급회전 +78.6°) 안쪽 파고듦 -> 스크립트 도로이탈 판정(7tick/0.30s)으로 종료
- 진행: 궤적 258.7m / 781m, 시뮬 52.3s, 실제 약 7s(23:18:49~23:18:56)
- 세션13(TM) 554m 보다 짧다. 단, TM 은 r1446 을 통과했고 BasicAgent 는 여기서 멈춤
- 물리적 정지 아님: 벽 0 이라 차는 인도 턱 위로 계속 가는 중이었고(roll -7°), 판정 기준이 종료시킴

## Phase 0
- 부팅 23:00:37, Firefox·content 0, CarlaUE4 없음, avail 10Gi/swap 0B
- test_drive.py: 세션13 작성·검증판(재작성 없음). tm/agent dry-plan 동일(sha256 82643554),
  wants_sync agent=True/tm=False, 이탈량 정의 test_drive.py:487 docstring
- 이탈량 정의(고정): lat_off_m = bbox 중심의 계획 커넥터 차선 중심선 부호 있는 수직거리(+오른쪽).
  body_excess_m = bbox 8꼭짓점 (|중심선 수직거리| - 차선폭/2) 최댓값, 차선 경계 기준(+=밖)

## Phase A
- 서버 23:12:39 PID 4171. 명령 원문:
  cd ~/carla/CARLA_0.9.15 && ./CarlaUE4.sh -quality-level=Low -windowed -ResX=800 -ResY=600
- 기본 맵 136s: avail 2.5Gi / swap 1.9Gi / gtt 2.91GiB
- python map/tests/test_load_map_878.py --wall-height 0: exit 0, "* wall_height=0.0", spawn 1243
- 로드 직후 avail 2.3Gi / gtt 4.00GiB, 2분 후 avail 4.7Gi / gtt 1.63GiB

## Phase B
- B-1 사전점검(docs/logs/session14_precheck.log): 204점, 최대간격 4.38m, 14쌍 next() OK,
  agent 계획 road 순서 = 계획 15 road, U턴 커넥터 미포함, 서버 sync=False 액터 0
- B-2 동기 모드: 전환 전 sync=False/None -> 전환 후 sync=True/0.05
- tick 대조(t=10s, 프레임 200): delta_seconds 0.0500, 시뮬경과/프레임 0.0500s, 벽시계 226.4 tick/s
  -> 시뮬 기준 20Hz 정확. 동기 모드라 벽시계는 서버가 낼 수 있는 최대 속도(약 11배속)
- 복귀: sync=False / fixed_delta=None (성공). 정리 후 액터 vehicle 0 / sensor 0
- 로그: session14_drive_stdout.log, drive_log/collision_events/drive_plan_20260924_231850.*

## 관찰 항목
1. U턴 커넥터: 진입 없음. 순간 투영 2건만
   - r1915 t=31.55s 1tick(0.05s), s=0.16, agent 목표 r1356(계획대로) -> 바로 r1247->r1914
   - r1449 t=51.15~51.20s 2tick, s=0.00~0.05, 목표 r1446 -> 바로 r1355->r1446
   - 둘 다 junction 입구에서 커넥터 시작점이 겹쳐 get_waypoint 가 잠깐 쌍둥이 쪽으로 투영된 것
     (세션12·13 r1915 0.12s 와 같은 유형). 조향 이상 없음
2. 세션13 정지 지점(road1240 끝/junction60): 도달 못 함
3. 급회전 이탈량
   - r1446: 최대 lat +1.63m(오른쪽=우회전 안쪽), 최대 차체초과 +0.76m(s=7.2), 그때 속도 4.4m/s
     진입 6.1m/s(22km/h). on_road False 가 s=5.0 부터
   - r1428, r1564: 미도달
4. r1917: 통과(t=39.65s, 이어서 r1916 0.25s 경유 후 r1355)
5. 충돌: 0건(collision CSV 헤더만)
6. 완주: 아니오
7. 낙하: 없음. z -0.01~0.13, 마지막 roll -7.4°(오른쪽 바퀴가 인도 턱 위로)

## 막힌 지점
- 종료 위치: road1446 s=7.3, (357.7, -893.3), 시뮬 52.30s
- 직전 3초(5tick 간격):
  - t=49.30 r1355 v6.29 thr0.00 br0.30 st+0.00 roll-0.0
  - t=49.80 r1355 v5.06 thr0.75 br0.00 st-0.00
  - t=50.80 r1355 v5.17 thr0.75 st+0.09 roll-0.5
  - t=51.30 r1446 s0.1 v6.12 thr0.00 br0.30 st+0.23 roll-1.2
  - t=51.55 r1446 s1.6 v5.08 thr0.68 st+0.50 pitch+1.8 roll-5.9 lat1.05
  - t=51.80 r1446 s3.5 v4.66 thr0.75 st+0.19 roll-6.8 lat1.45
  - t=52.05 r1446 s5.4 v4.16 thr0.75 st+0.10 roll-6.7 lat1.62 on_road False
  - t=52.30 r1446 s7.3 v4.44 thr0.75 st+0.48 roll-7.4 lat1.63 ex0.76
- 벗어난 방식: road 시퀀스는 계획대로(1355->1446). 커넥터 안에서 횡으로 벗어남(안쪽 파고듦)

## 원인 해석
- 핵심 증거: 차가 r1446 s=1.6 에 있을 때 agent 목표점이 이미 커넥터 끝(r1446 s=12.17)이었다.
  steer 가 +0.50 에서 +0.10 까지 풀리며 거의 직진으로 현(chord)을 가로질렀다
- LocalPlanner(0.9.15 local_planner.py:241): 목표점 소비 거리
  = base_min_distance 3.0 + distance_ratio 0.5 x 속도(m/s) = 약 6m(6m/s 기준)
- 계획은 TM 용 4m 간격이라 r1446(12.2m)에 점 4개(s 1.74/5.21/8.69/12.17, 3.5m 간격)뿐
  -> 커넥터에 들어서자 6m 안의 점이 한꺼번에 소비되고 끝점을 겨냥
- 세션13 에서 추정한 "0.8m 파고듦" 은 계획 간격 문제를 반영하지 않았다. 실측 중심 1.63m / 차체 0.76m 초과
- TM 이 통과한 이유(추정): TM 은 set_path 점 사이를 자체 경로로 채운다. 미확인
- 속도 자체보다 계획 밀도 문제일 가능성이 크다. 진입 22km/h(목표 20 약간 초과)

## 다음 단계 후보(결정 필요, 같은 조건 재시도 안 함)
- (a) agent 모드 계획 밀도를 커넥터에서 촘촘하게(예: 1m. 기본 GRP 해상도는 2m)
- (b) base_min_distance 축소(예: 1.5~2m) - 직선 구간 진동 가능성
- (c) 60° 이상 커넥터 10km/h(세션13 대안) - 소비 거리 3.0+0.5x2.8 = 4.4m, 여전히 점 간격보다 큼
- 권장: (a) 먼저. 원인에 직접 대응하고 다른 파라미터를 건드리지 않는다

## 메모리
- 주행 중 측정 없음(MEM_RECORD_AT_S=60s 전에 종료). 주행 직후 23:20:06 avail 4.7Gi / swap 809Mi / gtt 1.63GiB

## 스크린샷 대기
- spectator (357.0, -895.5, 25) pitch -90 yaw -90 로 이동함(차량은 이미 제거됨)

## 스크린샷
- 23-21-23 -> figures/20_s14_r1446_topdown.png. junction42 십자 교차로 위. 모서리 둥글림이 작아 안쪽 인도(회색)가 커넥터 가까이 있다. 벽·띠 없음. 차량은 제거 후라 궤적 없음

## 정리
- 서버 종료 23:22:34, CarlaUE4 없음. avail 6.9Gi / swap 407Mi / gtt 0.05GiB. 시작 전 10Gi 대비 약 3.1Gi 미회복(세션12 3.4, 세션13 3.2)
- Phase D 미실행(완주 아님)
- 다음 세션: (a) 커넥터 계획 1m 간격 설계->승인->재부팅 후 주행. (b)(c) 는 대안으로 보류

## 세션 후 확인·결정
- 결과 문구(사용자 지시): U턴 커넥터 진입 0건, 충돌 0건. BasicAgent 전환으로 경로 추종 문제는 해결됐다. 새 문제는 급회전 커넥터에서 코너를 잘라 먹는 것(r1446 에서 1.63m). 물리적 차단이 아니라 스크립트의 도로 이탈 판정이 실행을 끊은 것이다. TM 은 차선 유지가 강하고 경로 추종이 약했으며, BasicAgent 는 그 반대다.
- 이탈 위치: 차 중심·오른쪽 모서리가 r1446 자기 인도(lane -2) 위. 반대 차선·메시 밖 아님
- (a)안 승인, 2회 주행 설계: map_session14_design_density_speed.md

## 판정 기준 변경(서버 없음, 사용자 승인)
- 폐지: 세션5~14 의 "도로 이탈 3tick"(차 중심이 Driving 차선 밖 3tick+0.3s)
- 기록만: 인도 침범(sidewalk), 차선 이탈(body_excess_m, n_corner_offlane), 꼭짓점 반대 차선 침범 깊이(opposite_depth_m)
- 중단(3tick+0.3s 지속): 차체 전체 차선 밖(바닥 4꼭짓점), 반대 차선(차 중심), 메시 밖(중심에 lane 없음 또는 z 0.5m 이상 하강)
- 반대 차선 = 계획 밖 road 의 Driving 차선, 진행방향 차이 135° 초과
- 기존 유지: 액터 소멸, 15m/s 초과, 정체 5초, 240초
- 버그 수정: LaneType.Any(-2) 가 차선 위에서도 None -> 메시 밖 오판. 종류별 조회로 변경. 궤적 재생으로 발견
- 재생 검증: 세션14 궤적 중단 없음(꼭짓점 침범 최대 0.19m, r1247 s=98 좌커브). 세션13 궤적 r1429 U턴 진입 t=75.47 중단

## test_drive.py 인자(세션14 추가)
- --conn-spacing M: 커넥터 계획 점 간격(기본 전 구간 4m). 1.0 이면 263점(커넥터 84)
- --sharp-speed K: 60° 이상 커넥터(r1446/r1428/r1564) 15m 안/위에서 목표 K km/h
- CSV 36열 = 기존 28 + n_corner_offlane, sidewalk, opposite_lane, opposite_depth_m, mesh_out, target_dist_m, min_dist_m, set_speed_kmh

## 마감과 다음 세션
- 서버 미기동으로 마감(한도 부족, 3.1Gi 미회복 -> 재부팅 필요)
- 다음: 재부팅 -> 브라우저 확인 -> 서버 -> 기본맵 2분 -> --wall-height 0 로드 -> 1차 test_drive.py(기본값)
- 1차 관찰: r1446 통과, 인도 침범량, 반대 차선 깊이, U턴 진입, 급회전 3곳 이탈량, 781m 완주
- 완주 못 하면 2차 --conn-spacing 1.0, 3차 --conn-spacing 1.0 --sharp-speed 10
