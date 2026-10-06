# 맵세션13 설계: TM -> BasicAgent 전환

작성 2026-09-24. 코드는 아직 쓰지 않음(승인 대기). 주행은 재부팅 후 다음 세션.

## 1. 최소 조사: 경로 junction 7개의 커넥터 (읽기 전용)

스크립트: map/scripts/junction_connectors_s13.py (서버 없이 carla.Map(name, xodr) 사용)
결과: map/docs/logs/session13_junction_connectors.csv

회전각은 CARLA yaw 기준(+ = 오른쪽, - = 왼쪽). 지시서의 r1446 -78.8° / r1428 68.6° 와
부호가 반대인 것은 부호 규약 차이이고 크기는 같다(78.6~78.8, 68.5~68.6).

- j1 (in r1247): 커넥터 2, 계획 r1914(+2.5°), U턴 1 = r1915
- j107 (in r1356): 커넥터 1, 계획 r1917(+37.9°), U턴 0
- j42 (in r1355): 커넥터 4, 계획 r1446(+78.6°), U턴 1 = r1449
- j10 (in r1172): 커넥터 3, 계획 r1428(-68.5°), U턴 1 = r1429
- j60 (in r1240): 커넥터 3, 계획 r1695(+4.8°), U턴 1 = r1697
- j84 (in r1239): 커넥터 2, 계획 r1426(-5.5°), U턴 0
- j36 (in r1238): 커넥터 2, 계획 r1564(-82.2°), U턴 0

관찰
- U턴 커넥터 4개(r1915/r1449/r1429/r1697)는 전부 길이 9.89m, 회전 -153.2°,
  나가는 road 가 진입 road 의 쌍둥이(시작점 3.35m, 방향 180° 반대).
  netconvert 템플릿 커넥터 길이(세션2, 9.892m)와 같다
- 세션13 주행 궤적에서 TM 은 U턴 커넥터가 있는 4곳 모두에서 그 커넥터를
  밟았다: r1915 0.12s, r1449 0.16s, r1429 0.58s, r1697 끝까지(이탈).
  앞의 셋은 순간 투영 후 복귀, j60 에서만 완전히 들어갔다
- r1564(-82.2°)도 급회전이다. 관찰 목록에 추가 제안
- 조치: 지금은 목록만. Lanelet2 작도 때 다시 본다

## 2. 목적

TM set_path 는 갈림길에서 경로점에 가까운 쪽을 고르는 휴리스틱이라 커넥터를
지정할 수 없다(세션6). BasicAgent 의 set_global_plan 은 우리가 준 웨이포인트
열을 그대로 따라가고 분기 선택이 없다. 커넥터를 get_waypoint_xodr 로 확정해
넣으면 U턴 커넥터로 갈 여지가 구조적으로 없다.

"교수님이 왜 바꿨냐고 물으면": 셔틀은 고정 노선이다. 노선을 제안(TM)이 아니라
명령(웨이포인트 열 + PID 추종)으로 줘야 재현성이 생긴다.

## 3. 설치·경로

- 설치 완료: networkx 3.4.2, shapely 2.1.2 (.venv-carla, 버전 미고정).
  numpy 2.2.6 유지. import OK
- agents 경로: ~/carla/CARLA_0.9.15/PythonAPI/carla (agents 패키지의 부모)
- 방식(권장): 스크립트 안에서 sys.path.insert(0, 그 경로).
  이유: ~/.bashrc 수정 불필요, 실행 명령 불변, 재현 시 추가 설정 없음.
  대안: 실행 때 PYTHONPATH=... 앞에 붙이기(명령이 길어짐)

## 4. 구조 (test_drive.py 의 TM 부분만 교체)

그대로 유지
- 스폰, build_plan(15 road, 현재 204점, 약 4m 간격), 인접 14쌍 점검
- tick 마다 CSV 기록, 충돌 센서, bbox 기록, 메모리 기록(60s)
- 실패조건 5개(도로 이탈 / 폭주 15m/s / 정체 5s / 시간 240s / 예외), 완주 판정

바꾸는 것
- vehicle.set_autopilot + tm.set_path + tm.set_desired_speed 삭제
- 대신 BasicAgent 생성, set_target_speed(20), set_global_plan(계획)
- 루프 안에서 control = agent.run_step(); vehicle.apply_control(control)
- 완주·실패 시 set_autopilot(False) 대신 제동 제어만 적용

## 5. 핵심 함수

- plan_waypoints(carla_map, rows): build_plan 결과 rows 의 (road_id, s) 로
  get_waypoint_xodr(road_id, -1, s) 를 다시 불러 carla.Waypoint 목록 생성.
  build_plan 은 손대지 않는다(사전 점검 코드와 공유)
- make_agent(vehicle, carla_map, wps):
  BasicAgent(vehicle, target_speed=20,
  opt_dict={ignore_traffic_lights, ignore_stop_signs, ignore_vehicles: True},
  map_inst=carla_map)
  agent.set_global_plan([(wp, RoadOption.LANEFOLLOW) ...],
  stop_waypoint_creation=True, clean_queue=True)
  stop_waypoint_creation=True -> 계획이 끝나도 임의 웨이포인트를 만들지 않음
- 기록 추가(열 2개): agent_target_road, agent_target_s
  (local planner 가 지금 쫓는 점). 급회전 구간 해석용

## 6. 데이터 흐름

xodr -> 서버 맵 -> build_plan(rows) -> plan_waypoints(wps)
-> set_global_plan(queue) -> 매 tick: run_step 이 queue 앞의 가까운 점을 버리고
다음 점을 PID 로 추종 -> VehicleControl -> apply_control -> 차량
-> get_transform 등 -> CSV / 실패조건 / 완주 판정

## 7. 결정 필요 사항

(1) 동기 모드. 권장: 켠다(fixed_delta_seconds=0.05, 20Hz)
- BasicAgent 의 PID 는 dt=0.05(1/20s) 가정이 기본값이다(local_planner.py 81~82).
- 지금 test_drive 는 비동기(wait_for_tick)이고 세션13 실측 tick 은 약 251Hz.
  이 상태로 쓰면 적분항은 약 12배 과대, 미분항은 약 12배 과소가 된다
- 동기 모드면 루프가 world.tick() 을 부르고, 끝날 때(finally) 반드시 비동기로
  되돌린다(안 되돌리면 서버가 다음 클라이언트를 기다리며 멈춤)
- 부수 효과: CSV 행이 약 12분의 1로 준다. 도로 이탈 판정(3tick 그리고 0.3s)은
  20Hz 에서 6tick 이 되므로 사실상 0.3s 기준으로만 작동
- 대안: 비동기 유지 + opt_dict dt 를 실측값으로. tick 간격이 흔들려 비권장

(2) 급회전 커넥터 속도. 권장: 첫 주행은 20km/h 그대로, 관찰
- local planner 는 차에서 min_distance = 3.0 + 0.5 x 속도(m/s) 안의 점을
  버린다. 20km/h 에서 약 5.8m. 점 간격 4m 이므로 쫓는 점은 5.8~9.8m 앞
- PID 는 그 점을 향해 조향하므로 곡선 안쪽을 가로지른다. r1446 기준
  (R = 13.91m / 1.372rad = 10.1m, 앞 거리 평균 7.8m) 현(弦) 처짐 약 0.8m
- 차선 여유: 차선 반폭 1.68 - 차 반폭 0.894 = 0.79m. 경계와 거의 같다
- 즉 r1446·r1564·r1428 에서 안쪽 연석을 밟을 가능성이 있다
- 대안 B: 회전 60° 이상 커넥터(r1446/r1428/r1564)와 그 앞 10m 에서
  set_target_speed(10). 5.8m -> 4.4m, 처짐 약 0.5m
- 권장 이유: 변수를 하나씩 바꾼다. 첫 주행은 컨트롤러 교체만

(3) 파일. 권장: test_drive.py 에 --controller {agent,tm} 인자, 기본 agent
- tm 경로는 세션10~13 과 같은 동작으로 남겨 재현 가능하게
- 대안: test_drive.py 를 직접 교체(옛 동작은 이전 로그로만 남음)
- 대안: 새 파일 test_drive_agent.py

## 8. 관찰 항목 (다음 세션)

1. 계획 15 road 와 실제 통과 시퀀스가 일치하는가(U턴 커넥터 0회)
2. j60 r1695 통과(세션12·13 실패 지점)
3. 급회전 커넥터 r1446(+78.6°) / r1428(-68.5°) / r1564(-82.2°) 에서
   횡편차, 속도, steer 포화(0.8), on_road, 충돌
4. r1917(2.97m, 37.9°) 통과
5. 충돌 건수(벽 0 조건)
6. 781m 완주, 소요 시간, 평균·최고 속도
7. 메모리

## 9. 위험

- BasicAgent 생성자가 GlobalRoutePlanner 를 만든다(map.get_topology 기반,
  sampling 2m). 878 road 지도에서 시간·오류는 미확인. set_global_plan 만 쓰므로
  경로 탐색은 쓰지 않는다
- 계획이 끝나면 queue 가 비어 brake=1.0 이 된다. 완주 판정(s>=95, 5m 이내)이
  그보다 먼저 걸려야 한다(마지막 점 s=98.45). 안 걸리면 정체로 실패 처리
- wall_height=0 유지. 도로 밖으로 나가면 막는 것이 없다(세션13: 인도 턱만)

근거
- https://carla.readthedocs.io/en/0.9.15/adv_agents/
- https://github.com/carla-simulator/carla/blob/0.9.15/PythonAPI/carla/agents/navigation/basic_agent.py
- https://github.com/carla-simulator/carla/blob/0.9.15/PythonAPI/carla/agents/navigation/local_planner.py
- https://carla.readthedocs.io/en/0.9.15/adv_synchrony_timestep/

## 10. 코드 수준 설계 (승인 3건 반영, 코드 작성 전)

대상 파일: map/tests/test_drive.py 하나. 새 파일 없음.

### 인자
- --controller agent|tm (기본 agent). tm 은 세션10~13 과 같은 동작(비동기, TM)
- --restore-async: 서버에 접속해 동기 모드를 끄고 바로 종료. 차량 없음.
  finally 로 못 잡는 종료(SIGABRT/SIGKILL, 세션10 사례)에 쓰는 수동 복구용
- --dry-plan: 기존 그대로

### 상수 추가
- AGENTS_PATH = ~/carla/CARLA_0.9.15/PythonAPI/carla (sys.path 앞에 추가,
  agent 모드일 때만 import)
- FIXED_DELTA_S = 0.05 (20Hz, PID dt 와 같은 값으로 opt_dict['dt'] 에도 넘김)
- SHARP_CONNECTORS = {1446: +78.6, 1428: -68.5, 1564: -82.2} (CARLA yaw 부호)
- TICK_CHECK_AT_S = 10.0 (tick 간격 대조 시점)

### 새 함수
- set_sync(world, on): get_settings 사본을 고쳐 apply_settings. 전환 전후
  synchronous_mode / fixed_delta_seconds 를 출력(로그에 남음). 원래 설정 반환
- restore_settings(world, original): finally 에서 호출. 원래 설정으로 되돌리고
  다시 읽어 확인 출력. 실패해도 예외를 삼키고 "복구 실패, --restore-async 실행"
  안내를 출력
- plan_waypoints(carla_map, rows): rows 의 (road_id, s) -> get_waypoint_xodr
  -> [(wp, RoadOption.LANEFOLLOW)]. None 이면 스폰 전에 중단
- make_agent(vehicle, carla_map, plan): BasicAgent(target_speed=20,
  opt_dict={ignore_traffic_lights/stop_signs/vehicles: True, dt: 0.05},
  map_inst=carla_map), set_target_speed(20), set_global_plan(plan,
  stop_waypoint_creation=True, clean_queue=True)
- sharp_polylines(carla_map): 세 커넥터 lane -1 을 0.25m 간격으로
  get_waypoint_xodr 해 (s, 위치, 오른쪽 벡터, 차선폭) 목록을 미리 만든다
- sharp_offset(polys, loc): 가장 가까운 표본이 2.0m 안이고 양 끝점이 아니면
  (커넥터 id, s, 부호 있는 횡편차 lat[+ = 오른쪽], 차체 초과량
  excess = |lat| + 차 반폭 - 차선폭/2 [+ = 차체가 차선 밖]) 반환.
  계획 커넥터 기준이라 junction 안에서 투영 road 가 바뀌어도 흔들리지 않음

### CSV 열 추가(맨 뒤, 기존 22열 이름·순서 불변, tm 모드는 빈칸)
- agent_target_road, agent_target_s: local planner 가 지금 쫓는 점
- sharp_conn, sharp_s, lat_off_m, body_excess_m

### 루프 (agent 모드)
- 스폰, bbox, 충돌 센서는 그대로(비동기 상태에서)
- make_agent -> set_sync(True) -> 첫 world.tick()
- 매 반복: frame = world.tick(); snap = world.get_snapshot()
  -> 기존 기록·전이·실패조건·완주 판정 그대로
  -> control = agent.run_step(); vehicle.apply_control(control)
- t >= 10s 에서 1회: snap.timestamp.delta_seconds, (시뮬 경과 / 프레임 수),
  벽시계 기준 초당 tick 수를 출력해 20Hz 확인
- agent.done() 이 완주 전에 True 가 되면 1회 경고(계획 소진). 실제 실패
  판정은 기존 정체 조건이 한다
- 완주 시 set_autopilot(False) 대신 제동+핸드브레이크 적용 후 tick 1회

### finally 순서
1. CSV 닫기 2. 센서 stop/destroy 3. 차량 destroy(실패 시)
4. agent 모드면 world.tick() 1회(제거 반영) 후 restore_settings
각 단계 try/except 로 감싸 앞 단계 실패가 뒤 단계를 막지 않게 한다

### 요약 출력 추가
- 급회전 커넥터별: 표본 수, 최대 |lat|, 최대 excess(+ 면 차선 밖), 안쪽/바깥쪽,
  on_road False tick 수, 그 구간 충돌 건수(frame 범위로 대조)
- tick 대조 결과, 동기 모드 전환/복귀 여부

### 한계
- finally 는 파이썬 예외와 Ctrl+C 까지만 잡는다. SIGABRT/SIGKILL 로
  프로세스가 죽으면 실행되지 않는다 -> --restore-async 로 수동 복구
- 차체 초과량은 차 중심 기준 근사(차체 회전 무시). 급회전 중 모서리는
  더 나갈 수 있다

## 11. 구현 결과 (2026-09-24, 서버 미사용 검증까지)

### 설계 대비 변경 1건
- 차체 초과량을 "차 중심 + 반폭 근사"에서 "bbox 8개 꼭짓점 실측"으로 바꿈.
  지표 정의를 고정하라는 지시(확인 3)에 따라 근사를 없앴다

### 이탈량 정의 (지표, 고정)
- lat_off_m: 차량 bbox 중심의, 계획 커넥터 차선 중심선에서의 부호 있는
  수직거리(차선 중심 기준, + = 진행 방향 오른쪽)
- body_excess_m: 차량 bbox 8개 꼭짓점 각각의 |차선 중심선 수직거리| - 차선폭/2
  중 최댓값(차선 경계 기준, + = 가장 많이 나간 꼭짓점이 차선 밖으로 나간 거리 m)
- 중심선: 계획 커넥터 lane -1 을 get_waypoint_xodr 로 0.25m 간격 표본화.
  꼭짓점마다 가장 가까운 표본의 오른쪽 벡터로 투영
- 측정 구간: 차 중심이 중심선 3.0m 안이고 가장 가까운 표본이 양 끝점이 아닐 때
- 요약의 안쪽/바깥쪽: 최대 초과 꼭짓점이 회전 방향 쪽이면 안쪽

### 검증
- py_compile OK, --help OK
- --controller tm --dry-plan 과 --dry-plan(agent) 출력 CSV 바이트 동일, 세션11
  dry 계획(수정 전 백업 drive_plan_dry_before_s13.csv)과도 바이트 동일, 203점
- 세션13 실제 주행 계획(204점)과 (road_id, s) 203점 일치. 차이 1점
  (road1247 s=1.96)은 스폰 위치 의존 첫 점(dry 는 스폰을 모름)
- 모드 분기: wants_sync('agent')=True, wants_sync('tm')=False. world.tick() 호출
  4곳은 전부 use_sync 또는 original_settings(동기 전환 시에만 채워짐) 조건 안.
  tm 은 wait_for_tick 경로 그대로. 서버로는 미확인
- sharp_offset 오프라인 검산(carla.Map 클라이언트 지도, 가짜 bbox
  1.853x0.894): 커넥터 s=6 에서 좌우 이동량 0/+0.5/-1.0m 에 대해
  lat 이 +0.000/+0.500/-1.000 으로 정확히 나옴. excess 는 차선폭 3.35m,
  곡선 위라 앞뒤 꼭짓점이 바깥으로 나가 중앙 정렬에서도 -0.53~-0.65m
- 미확인: 서버에서의 동기 전환·복귀, BasicAgent 생성 시간(GRP), 실제 20Hz

### 기록
- BasicAgent 는 신호·차량 무시로 설정한다. 교내에 신호등이 없고(ODD 확인),
  이번 주행에 다른 차량이 없다. 시나리오에 배경 차량을 넣을 때는 이 설정을
  다시 볼 것
