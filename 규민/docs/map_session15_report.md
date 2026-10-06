# 맵 세션15 보고 (2026-09-25)

## Phase 0
- 부팅: 2026-09-25 11:16:18
- Firefox: 실행 중이었음(PID 2660 + content 9개) -> 사용자가 종료, 0개 확인
- xodr sha256: bf835cdfad0cea65 (일치)
- 기본 dry-plan sha256: 82643554 (불변, --follow 추가 뒤에도 같음)
- --follow on|off 추가(기본 on): 매 tick spectator 를 차량 뒤 8m·위 4m 에 둠,
  pitch -26.6° 고정, 차량 yaw 만 따라감. 실행 로그 시작·요약에 follow=on|off 출력.
  CSV 36열 불변. 동기 모드는 스텝이 0.05s 로 고정이라 주행 결과는 follow 와
  무관하고 벽시계 시간만 늘어난다(세션14 와 조건 동일)

## Phase A
- 서버: 11:23:26 기동, 명령 원문
  cd ~/carla/CARLA_0.9.15 && ./CarlaUE4.sh -quality-level=Low -windowed -ResX=800 -ResY=600
- 기본 맵 2분 후(11:25:30): avail 2.3Gi / swap 1.7Gi / gtt 2.94GiB
- 로드: test_load_map_878.py --wall-height 0, exit 0, "* wall_height=0.0", spawn 1243
- 로드 직후: avail 2.4Gi / swap 840Mi / gtt 3.51GiB
- 로드 2분 후(11:27:49): avail 4.8Gi / swap 838Mi / gtt 1.55GiB

## Phase B 1차 주행 (기본값: conn_spacing 4m, sharp_speed 없음, follow on)
- 명령: python map/tests/test_drive.py (11:28:41~11:28:51, exit 0)
- 로그: map/docs/logs/session15_run1.log, drive_log_20260925_112842.csv
- 결과: 실패 - 반대 차선 침범(중심) 7tick/0.30s, road1434 s=11.7 (309.3,-774.6)
- 시뮬 77.6s, 궤적 387.7m (세션14 258.7m), 최고 6.43m/s
- tick 대조 t=10s: delta 0.0500, 벽시계 193.4 tick/s (세션14 226.4, follow 영향 추정)
- 주행 중 메모리(t=60s): avail 4.7Gi / swap 838Mi / gtt 1.56GiB
- 동기 복귀: synchronous_mode=False, fixed_delta_seconds=None 성공

### 관찰 항목
1. r1446 통과: 예 (진입 t=51.20s, 14.9km/h)
2. 급회전 이탈량
   - r1446 (+78.6°): 최대 |lat| 1.65m, 최대 차체초과 +0.96m(안쪽, s=10.0),
     인도 39tick, 차선밖 꼭짓점 최대 2/4, 메시밖 1tick(중단 아님)
   - r1428 (-68.5°): s=4.5 까지 최대 |lat| 1.42m, 차체초과 +0.58m(안쪽) - 여기서 중단
   - r1564 (-82.2°): 미도달
3. 반대 차선: 있음(중단 사유). r1434 위, 꼭짓점 깊이 최대 3.00m. 아래 분석 참고
4. U턴 커넥터 진입: agent 목표 road 기준 0tick. 투영 road 기준 r1429 17tick,
   r1449 3tick, r1915 2tick (junction 안 겹침 투영, 목표는 계속 계획 road)
5. r1917 통과: 예
6. 충돌: 0건
7. 메시 밖 낙하: 없음(r1446 mesh_out 1tick, 지속 조건 미충족)
8. 완주: 아니오

### 중단 지점 분석 (junction10, 좌회전 커넥터 r1428)
- agent 목표는 끝까지 r1428 (t=75.2s 부터 r1428 s=1.78 -> 12.46). 경로 추종은 정상
- 중단 시점 차 중심: r1428 중심선에서 왼쪽(안쪽) 1.41m, 차선 반폭 1.675m
  -> 차 중심은 여전히 자기 차선(r1428) 안(여유 0.27m)
- 같은 점이 r1434 중심선에서 0.42m. r1434 = junction10 의 반대 방향 커넥터
  (1254 -> 1301, yaw -64.9°, 폭 3.35m). get_waypoint(project_to_road=False, Driving)
  은 r1434 를 돌려줌
- 즉 r1428 과 r1434 의 차선 영역이 서로 겹친다(중심선 간격 약 1.8m < 차선폭 3.35m).
  차 중심이 두 차선 모두의 안에 있고, 조회가 반대 방향 쪽을 골라 중단됐다
- 코너 안쪽 1.4m 처짐 자체는 세션14 원인(4m 간격, 목표 건너뜀)과 같은 계열:
  target_dist_m 6~9m, min_dist_m 5.0~6.1m
- 쌍둥이 half-road 구조에서 비롯된 것으로 추정(미결 항목과 같은 뿌리)

### 결정 필요
- (a) 판정 유지하고 2차(--conn-spacing 1.0) 진행: 처짐이 약 0.4m 로 줄면
  r1434 와의 거리가 커져 중단을 피할 가능성. 다만 겹침은 그대로라 보장 없음
- (b) 반대 차선 판정 보완: 차 중심이 계획 커넥터 차선 안(|lat| <= 반폭)이면
  반대 차선으로 치지 않음. 코드 수정이라 승인 필요. 변수 두 개가 섞이지 않도록
  1차 조건(4m)으로 재주행하는 것이 비교에 맞음(같은 조건 재시도 아님: 판정이 다름)

## 판정 보완 (b) 적용 (사용자 승인 "비로" = (b)로)
- test_drive.py: in_plan_lane(plan_polys, loc) 신규. 계획 15 road 의 lane -1 중심선을
  0.25m 간격으로 표본화(3281점, sharp_polylines 에 roads 인자 추가해 재사용).
  차 중심에서 가장 가까운 계획 중심선 표본까지 거리 <= 차선폭/2 이면 True
- 중단 조건: 반대 차선(중심) AND NOT in_plan_lane. opposite_lane 열은 기존대로 기록
- CSV 열 in_plan_lane 추가(끝에, 37열). opposite_lane=True 일 때만 값, 아니면 빈칸
- 요약에 "(계획 차선 안 Ntick)" 추가
- 백업: scratchpad test_drive_before_s15b.py
- 검증: py_compile OK, --help OK, 기본 dry-plan sha256 82643554 불변

## 오프라인 재생 (map/docs/logs/session15_replay_inplan.log)
- s15 1차: 중심 반대차선 7tick, 전부 계획 차선 안 -> 새 판정 중단 없음
- s14: 반대차선 0tick -> 중단 없음(변화 없음)
- s13 TM: 중심 반대차선 132tick, 전부 계획 차선 안 -> 중단 없음

### 세션14 결정 전제 정정
- 세션14 재생에서 "세션13 r1429 U턴 진입, 중심 반대차선 t=75.47 중단"이라 기록했으나
  궤적을 다시 보면 세션13 TM 차량은 junction10 에서 U턴하지 않았다
  - t=74.5~77.0: 계획 r1428 중심선에서 0.54~0.95m, 진행방향 차이 4~8°
  - t=77.5 부터 계획 다음 road r1240 위(중심선 0.01~0.36m)로 정상 진행
  - 투영 road 가 r1429/r1434/r1431/r1432 로 바뀐 것은 junction10 커넥터 겹침 때문
- 즉 그 "U턴 진입"은 이번 1차와 같은 겹침 오탐이었다
- 세션13 의 실제 U턴은 junction60 r1697(종료 t=106.7). 중심 반대차선 판정은
  보완 전에도 이것을 잡지 못한다(차가 U턴 커넥터 방향으로 달리므로 "반대"가 아님)
- 결론: 반대 차선 판정은 U턴 진입 검출 수단이 아니다. agent 는 목표 road 가
  U턴 커넥터인지(agent_target_road)로 볼 수 있고, 1차는 0tick

## Phase B 재주행 (판정 보완 후, 조건은 1차와 동일: 기본값, follow on)
- 명령: python map/tests/test_drive.py (11:33:16~11:33:32, exit 0)
- 로그: map/docs/logs/session15_run2.log, drive_log_20260925_113316.csv (37열)
- 결과: 완주 - t=153.2s, 종점까지 4.97m (road1278 s>=95, 5m 이내)
- 동기 복귀: synchronous_mode=False, fixed_delta_seconds=None 성공
- 차량(actor 27): 브레이크 유지로 서버에 남아 있음 -> 정리 때 제거 필요
- tick 대조 t=10s: delta 0.0500, 벽시계 215.2 tick/s
- 주행 중 메모리(t=60s): avail 4.6Gi / swap 824Mi / gtt 1.52GiB

## Phase D 완주 분석
- 스크립트(신규, 읽기전용): map/scripts/analyze_drive_s15.py,
  map/scripts/plot_drive_trajectory_s15.py
- 출력: map/docs/logs/session15_drive_analysis.log,
  map/docs/logs/session15_section_deviation.csv, map/docs/figures/11_drive_trajectory.png
- 그림: 10_overlay_confirmed.png 와 같은 범위(old_axis_bounds)·figsize·dpi, 위쪽=진북.
  변환은 평활화 xodr 자신의 junction 으로 재적합. 건물·보행로 배경은 넣지 않음

### 시간·속도
- 시뮬: 153.15s / 실제: 스크립트 전체 16s(11:33:16~32), 주행 루프 약 14.2s
  (3063 tick / 215.2 tick/s). 약 10.7배속
- 궤적 길이 774.4m (계획 경로 약 781m, 스폰 s=0 ~ 종점 4.97m 앞)
- 평균 5.06m/s (18.2km/h), 최고 6.42m/s (23.1km/h, t=73.05, 목표 20km/h 초과 3.1km/h)

### 계획 vs 실제 시퀀스
- 판정 기준: 차 중심에서 가장 가까운 계획 중심선 표본의 road (투영 road 는
  junction 겹침 때문에 쓰지 않음. test_drive 요약의 "실제" 33개는 투영 road 라 불일치 표시)
- 결과: 15 road 시퀀스가 계획과 완전히 일치 (15/15)

### junction 별 통과 (최근접 계획 road 기준 구간 시각, 투영 road 는 참고)
- junction1   r1914: t=31.60~32.45s  투영 [1914]
- junction107 r1917: t=39.60~40.15s  투영 [1916, 1917]
- junction42  r1446: t=51.20~53.10s  투영 [1355, 1446]
- junction10  r1428: t=76.55~79.30s  투영 [1428, 1429, 1431, 1432, 1434]
- junction60  r1695: t=108.70~111.60s 투영 [1695, 1696]
- junction84  r1426: t=121.70~125.05s 투영 [1239, 1422, 1424, 1426]
- junction36  r1564: t=131.90~134.50s 투영 [1558, 1559, 1561, 1564]
- agent 목표 road 가 U턴 커넥터인 tick: 0 (투영만 20tick: 겹침)

### 구간별 이탈량 (실험 지표. 정의: analyze_drive_s15.py 머리말)
- lat = 최근접 계획 중심선 부호 있는 횡거리(+오른쪽), excess = 바닥 4꼭짓점 기준
  차선 경계 초과량(+ = 밖). test_drive 의 sharp_offset(8꼭짓점, 커넥터 창) 값은 괄호
- r1446 (+78.6°, 우): max|lat| 1.71m, excess +0.95m (test_drive 1.61m / +0.94m)
- r1428 (-68.5°, 좌): max|lat| 1.41m, excess +0.66m (test_drive 1.40m / +0.66m)
- r1564 (-82.2°, 좌): max|lat| 1.59m, excess +0.76m (test_drive 1.62m / +0.77m)
- 인도 침범 1: r1247 스폰 직후 t=0.05~1.10s 22tick, excess -0.78m
  (차체는 차선 안. 스폰 지점 road 시작부 지도 판정 특이, 미조사)
- 인도 침범 2: r1356 t=39.20s 1tick, excess +0.05m (오른쪽)
- 인도 침범 3: r1917 t=39.90~40.00s 3tick, excess +0.08m (오른쪽)
- 인도 침범 4: r1355 끝~r1446~r1172 시작 t=51.10~54.00s 59tick, excess +0.95m (오른쪽)
- 급회전 좌회전 2곳(r1428, r1564)은 인도 침범 0tick. 안쪽(왼쪽)으로 처지나 그쪽은
  반대 방향 커넥터/차선이라 인도가 아니다(1차 중단 원인과 같은 위치)
- 일반 road 최대: r1172 +0.63m(r1446 출구), r1278 +0.48m
- excess>0 tick: 219 / 3063 (7.2%)

### 세션별 비교
- 세션10 (벽 1.0 + TM): 92m, 중앙선 벽 물리 충돌로 정체, 충돌 1604건, U턴 미도달, 완주 X
- 세션13 (벽 0 + TM): 554m, U턴 커넥터 r1697 진입 후 도로 이탈 판정, 충돌 0, U턴 1건
  (r1429 는 세션15 에서 겹침 오탐으로 정정), 완주 X
- 세션14 (벽 0 + Agent): 258.7m, 인도 침범을 도로 이탈로 판정해 중단, 충돌 0, U턴 0, 완주 X
- 세션15-1 (판정 변경): 387.7m, junction10 커넥터 겹침으로 반대 차선 오판 중단, 충돌 0, U턴 0, 완주 X
- 세션15-2 (반대 차선 판정 보완): 774.4m, 충돌 0, U턴 0, 완주 O (153.2s)
- 2차(--conn-spacing 1.0)·3차(--sharp-speed 10) 미실행: 기본값으로 완주

## Phase E 소품 시험
- 스크립트: map/tests/test_props_s15.py (신규), 로그 map/docs/logs/session15_props.log
- road1247 오른쪽 도로변, 5/5 소환 성공(actor 29~33)
  - SM_Acer_02 scale 1.0: bbox extent (4.32, 4.06, 9.64)
  - SM_Ash_01 scale 1.0: (6.58, 7.30, 15.07)
  - SM_Acer_02 scale 2.0: (8.21, 8.55, 19.28) -> 높이 2.00배
  - SM_House01: (3.68, 6.80, 6.32)
  - SM_House02: (0.33, 5.87, 4.24) - bbox 이상, 육안으로는 온전한 집
- 메모리: 소환 전 avail 4.5Gi/gtt 1.53 -> 나무 후 4.4Gi/1.53 -> 건물 후 4.4Gi/1.52
- 육안(사용자): 나무 땅 위, scale 2 약 두 배, 집 2채 지붕·창 보임
- 그림: map/docs/figures/21_s15_props_topdown.png
- 결론: static.prop.mesh 로 나무·건물 배치 가능, /Game/ = Content/ 1:1 실증,
  scale 속성 동작. bbox 는 건물 크기 판단에 믿을 수 없음(House02)
- 미확인: 도로 밖 지면이 void(회색 평면), 대량 배치 시 메모리

## 정리
- 11:41 vehicle 27 제거, static.prop.mesh 5개 제거, 액터 spectator 1 만,
  synchronous_mode=False/fixed_delta None (비동기 복귀는 주행 finally 에서 이미 성공)
- 서버 사용자 종료, 11:41:59 CarlaUE4 없음
- 종료 후 avail 6.8Gi / swap 420Mi / gtt 0.07GiB. 시작 10Gi 대비 약 3.2Gi 미회복
  (세션12 3.4, 13 3.2, 14 3.1 과 같은 패턴). 재부팅으로 해소

## 새 미결 항목
- 반대 차선 판정은 U턴 검출 수단이 아님. agent 는 agent_target_road 로 확인
- 스폰 직후(r1247 s=0) 인도 22tick 판정, 차체는 차선 안 - 미조사
- junction 커넥터 차선 영역 겹침(r1428/r1434 등): Lanelet2 작도 때 재확인
- 소품: 도로 밖 지면 void, 대량 배치 메모리 미측정, bbox 로 건물 크기 판단 불가
