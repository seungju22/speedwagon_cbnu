# 맵세션13 보고: 벽 제거(wall_height=0) 주행 검증

날짜: 2026-09-24
조건: 세션12 와 같은 xodr(평활화판, sha256 bf835cdfad0cea65), 같은 test_drive.py.
바꾼 변수는 로드 파라미터 wall_height 1.0 -> 0.0 하나.

## Phase 0
- 부팅 18:11:58. Firefox(PID 2790 + contentproc 10개)가 실행 중이었음
  (세션12 와 같은 패턴). 사용자가 pkill -f firefox 로 종료, 재확인 0개
- avail 10Gi / swap 0B, CarlaUE4 없음
- xodr sha256 일치

## Phase A
- 서버 명령 원문: cd ~/carla/CARLA_0.9.15 && ./CarlaUE4.sh -quality-level=Low
  -windowed -ResX=800 -ResY=600 (18:18:00, PID 3933)
- 기본 맵 2분 후(18:20:10): avail 2.3Gi / swap 1.8Gi / gtt 3.01GiB
- 로드: python map/tests/test_load_map_878.py --wall-height 0, exit 0
  - 출력 "* wall_height=0.0", 나머지 6개 기본값(vertex_distance 2.0,
    max_road_length 50.0, additional_width 0.6, smooth_junctions/
    enable_mesh_visibility/enable_pedestrian_navigation True)
  - spawn point 1243 (세션12 동일)
  - 직후 avail 2.3Gi / swap 816Mi / gtt 4.07GiB
  - 2분 후 avail 4.7Gi / swap 775Mi / gtt 1.53GiB
- 벽 확인
  - 광선(map/scripts/wall_raycast_s13.py, 읽기 전용): road1240 s=133~149
    중앙선·바깥 수평 광선 20개 충돌 0. 그러나 양성 대조에서 연석 수직면도
    수평 광선에 안 잡힘(수직 광선은 노면 z0.00, 인도 z0.15 검출).
    -> cast_ray 는 절차 메시 수직면을 못 잡는다. 벽 판정 수단으로 부적합
  - 스크린샷(fig16/17 과 같은 위치·각도): figures/18_s13_wall0_topdown.png,
    19_s13_wall0_side.png. 중앙선·바깥 검은 띠와 1m 판이 사라짐, 인도 턱만 남음
  - 판정: 서버가 wall_height=0 을 실제로 적용함(미확인 1번 해소)

## Phase B
- 사전 점검(docs/logs/session13_precheck.log): spawn road1247 s=0.00, 계획
  204점, 최대 간격 4.38m, 인접 14쌍 next() OK, 종점 road1278 s=100.38.
  세션12 와 동일. 첫 시도는 세션12 와 같은 호출 순서 오류(KeyError
  gap_to_prev_m, write_plan_csv 가 gap 을 채움)로 재실행
- 주행: python map/tests/test_drive.py, 18:26:06~18:27:54, exit 0
  - stdout: docs/logs/session13_drive_stdout.log
  - 궤적: map/docs/logs/drive_log_20260924_182607.csv
  - 충돌: map/docs/logs/collision_events_20260924_182607.csv (헤더만, 0건)

### 결과
- 판정: 실패 - 도로 이탈 71tick/0.30s 연속 (test_drive 실패조건 2)
- 시뮬 시간 106.7s, 궤적 길이 554.4m (세션12 548m)
- 충돌 0건 (세션12 2139건, 세션10 1604건)
- 세션12 정지 지점(road1240 s=148.1~148.7) 통과 (t=105.2s, s=148.17)
- 속도 최고 5.97 m/s, 평균 5.24 m/s
- junction1: r1915 0.12s(순간 투영) 후 r1914 0.96s -> r1914 주행(세션12 동일)
- r1917·r1446·r1428 통과
- 메시 밖 낙하: 없음. z 범위 -0.089~0.126m. 이탈 지점은 인도 위(z 0.15)

### 이탈 경위 (junction60)
- road1240 끝에서 커넥터 3개(서버 조회, docs/logs/session13_junction60.log)
  - r1695 len 15.12 turn +4.8° -> r1239 (계획, 직진)
  - r1696 len 13.65 turn -99.9° -> r1302
  - r1697 len 9.89 turn -153.2° -> r1371 (쌍둥이로 되돌아가는 U턴)
- 차는 road1240 s=145.16(끝 4m 앞, plain road)에서 좌조향 시작.
  s=145.9 steer -0.39, r1697 진입 후 steer -0.8 포화, yaw 46° -> -73°
- 궤적 r1240 -> r1697 -> r1694 -> r1690 -> r1697. 이탈 위치 (425.1,-665.2)는
  road1690 lane-2 Sidewalk(junction60 안). z 0.08, pitch 3.4° = 인도 턱 위
- throttle 0.34~0.85, brake 0 (TM 이 의도적으로 멈추지 않음)
- 계획 CSV 의 다음 점은 r1695 s=1.89 (422.05,-660.0) 로 직진 방향.
  차는 계획이 아닌 U턴 커넥터 쪽으로 조향했다

### 세션12 와의 연결
- 세션12 첫 좌조향 s=145.09 steer -0.149 yaw 46.155
- 세션13 첫 좌조향 s=145.16 steer -0.149 yaw 46.157
- 두 주행이 road1240 끝까지 사실상 같다. 세션12 에서 "좌회전을 road 끝
  4m 앞에서 시작" 한 것은 r1697(U턴) 방향 조향이었고, 벽이 그것을 막아
  정지했다. 벽을 없애니 같은 조향이 인도 위로 이어졌다
- 따라서 벽은 정지의 직접 원인이었지만, 근본 원인은 TM 이 계획(r1695
  직진) 대신 U턴 커넥터로 조향한 것이다. 조향 선택 이유는 미확인

### 잔여 61곳 대조
- 이탈 지점 최근접 105.9m (r1298 s61.47). 경로 road(1240/1695/1697/1371)
  해당 없음. 무관

### 메모리
- 주행 직전 avail 4.6Gi / swap 755Mi
- 주행 중: test_drive 기록(60s) 대비 avail 4.6Gi
- 주행 후 18:29:56 avail 4.6Gi / swap 745Mi / gtt 1.47GiB

## 비교표 (키: 세션12 벽1.0 / 세션13 벽0.0)
- 궤적 길이: 548m / 554.4m
- 충돌: 2139건 / 0건
- 정지·실패 위치: road1240 s=148 / junction60 인도(r1690)
- 실패 유형: 정체(벽 접촉) / 도로 이탈
- 첫 좌조향: s=145.09 / s=145.16
- 완주: 아니오 / 아니오

## 정리
- 액터 확인: vehicle 0 / sensor 0 / static.prop 0 (test_drive 가 센서·차량 제거)
- 서버 사용자 종료. 18:46:02 CarlaUE4 프로세스 없음
- 종료 후 avail 6.8Gi / swap 263Mi / gtt 0.06GiB
- 시작 전 10Gi 대비 약 3.2Gi 미회복(세션12 3.4Gi, 패턴 재확인)
- Phase D(소품) 는 완주 조건 미충족으로 실행 안 함

## 다음 단계 후보 (결정 필요)
- TM 이 road1240 끝에서 r1697(U턴)로 조향한 이유 조사
  - 세션6 U턴 커넥터 문제, junction1 r1915 순간 투영과 같은 계열로 추정
- 지시서의 "또 막히면" 분기: agents(BasicAgent, set_global_plan) 전환 검토.
  networkx, shapely 설치 승인 필요
