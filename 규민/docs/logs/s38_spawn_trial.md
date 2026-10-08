# 세션38 2부 소품 소환 시험 로그 (2026-10-08 23:30~23:34)

원자료: logs/s38_spawn_trial.log, s38_spawn_trial.json (3차 시도 = 채택)
스크립트: map/scripts/s38_spawn_trial.py (신규). xodr·v2·test_drive.py 수정 없음. 월드 재로딩 없음

## 시도 이력
- 1차(try1): r1392 끝 너머 s 에서 get_waypoint_xodr 가 예외 대신 None -> 스크립트 AttributeError. 소환 3개(House01 x1·x2, House02) 정리됨, 남은 액터 0
- 2차(try2): 끝까지 돌았으나 static.prop.mesh 가 set_transform 으로 안 움직임(busstop 블루프린트는 움직임) -> 메시 소품이 임시 자리(오른쪽 80m)에 남음, LiDAR 10·20m 불가. 화면 logs/s38_props_after_try2.png. 남은 액터 0
- 3차(채택): bbox 는 먼 곳에 띄워 읽고 지운 뒤, 최종 자리 겹침 검사를 먼저 하고 그 자리에 새로 소환

## 월드
- 서버 맵 sha16 5240ca8b883e42c0(v2) 확인, r1392 길이 56.074m
- 시작 시 남은 액터: 1차 때 vehicle.audi.a2 29(1부 3회차 하네스 차, 브레이크 유지로 남겨둔 것) -> 지움
- 원래 설정 synchronous_mode=False, no_rendering_mode=False, fixed_delta_seconds 미설정(쓰레기값 표시)

## 소환 (bbox = API 값, m, 길이·폭·높이 = 2*extent)
- House01 scale 1.0: 성공, bbox 0.707 x 15.436 x 12.646
- House01 scale 2.0: 성공, bbox 1.415 x 30.872 x 25.293 (정확히 2배, attr scale '2.0')
- House02 scale 1.0: 성공, bbox 1.179 x 11.690 x 8.476
- Apartment03_v01_merged: 성공, bbox 1.881 x 29.870 x 32.256, 중심 오프셋 (-7.93, 6.00, 16.13)
- SM_Acer_02: 성공, bbox 0.911 x 11.820 x 19.277
- SM_Ash_01: 성공, bbox 3.146 x 19.396 x 30.134
- static.prop.busstop: 성공, bbox 2.024 x 3.809 x 2.739
- 덩어리 /Game/Carla/Static/Ground/GroundCube: 성공, bbox 0.222 x 1.397 x 1.000 (화면상 작은 정육면체)
  - 후보 탐색: Content 에서 이름에 cube/plane 이 든 정적 메시 = GroundCube, SideWalkCube, SM_RailingCube(_Opt), Other/SM_Plane. GroundCube 만 시험
- bbox 의 x 축 값이 건물·나무 모두 0.7~3.1m 로 화면 크기와 안 맞음 -> 축·크기 신뢰 불가(세션15 House02 0.33m 와 같은 성격). 0 은 없음
- 발자국: 정사각형, 반폭 = max(bbox x, bbox y, 최소값 건물 8 / 나무 3 / 정류장 1.5 / 덩어리 1.0m)

## 장면 배치 (r1392 lane -1 오른쪽, 앞줄 앞면 = 차도 가장자리 + 5.4m, 뒷줄 = 앞줄 깊이 + 6m 뒤)
- 겹침 검사 = 발자국 1m 격자 점: 어떤 차선도 안 잡힘 + 맵 전체 주행 차선 가장자리까지 >= 5.0m
- Acer_02 앞줄 s=5.91, 최소 5.236m(r1971) 통과
- Ash_01 앞줄 s=27.52, 5.400m(r1392) 통과
- busstop 앞줄 s=45.12, 5.400m(r1392) 통과
- GroundCube 앞줄 s=54.02, 5.230m(r1487) 통과
- House02 앞줄 s=69.02(r1392 끝 너머 직선 연장), 차선 위 점 174, -1.663m(r1334) -> 불합격, 소환 안 함
- House01 x1 뒷줄 s=8.0, 30.625m 통과
- House01 x2 뒷줄 s=37.44, 13.704m(r1202) 통과
- Apartment03 뒷줄 s=73.81, 차선 위 점 370, -1.671m(r1202) -> 불합격, 소환 안 함
- 장면 6/8. 불합격 2개는 r1392(56m) 길이를 넘어 다음 도로와 겹친 배치 문제. 메시 자체 문제 아님(LiDAR 단독 시험에서는 소환됨)

## 메모리·FPS (동기 0.05s, 틱 100번, 센서 없음)
- 소환 전 렌더 켬: 4.29ms/틱(233.2), avail 4452Mi swap 787Mi gtt 1477Mi
- 소환 전 렌더 끔: 0.63ms/틱(1577.7), 4479Mi
- 소환 후(6개) 렌더 켬: 4.04ms/틱(247.7), 4480Mi
- 소환 후 렌더 끔: 0.63ms/틱(1582.2), 4492Mi
- 장면 지운 뒤 4491Mi, 정리 후 4464Mi. gtt 1477Mi 내내 같음
- 참고 반복값(2차 시도, 같은 조건): 소환 전 렌더 켬 3.92ms(255.0) / 1차 4.30ms(232.3) -> 렌더 켬 틱 시간 흔들림 약 0.4ms
- 해석: 6개로는 메모리·FPS 차이가 측정 흔들림 안. 소품 수를 늘린 시험은 안 함

## LiDAR (semantic, 소품 하나씩 r1392 s=28 앞줄 자리에 단독 소환, a2 를 lane -1 위 뒤쪽에 세워 한 바퀴)
- 속성: channels 64, points_per_second 300000, rotation_frequency 20, range 100, upper/lower_fov 10/-30(기본)
  - 브리지 sensor_mapping.yaml 이 이 PC 에 없음(find 0건) -> 지시문 기본값 사용
  - 장착: a2 원점 위 2.0m(가정). a2 physics 끔. 3틱째 측정 사용
- 거리 = LiDAR 에서 발자국(정사각형) 최근접점까지 수평 거리(실제 메시 면까지는 더 멀 수 있음)
- 적중 점 수(10m / 20m)
  - House01 x1: 628 / 337
  - House01 x2: 742 / 479
  - House02: 433 / 244
  - Apartment03: 639 / 515
  - Acer_02: 108 / 86
  - Ash_01: 512 / 311
  - busstop: 125 / 32
  - GroundCube: 43 / 8
- 한 바퀴 전체 점 4548~6807. object_idx 0(지면·도로) 3664~5838, 자기 차(a2) 405 점이 매번 잡힘
- object_idx = 소품 actor id 로 구분됨(확인)
- 20m 중 일부는 차 위치 s<0(r1392 시작 전 직선 연장, r1393 쪽). 차 physics 끔이라 무관

## 셔틀 후보 차량 제원 (소환 직후 기록 후 지움, 주행 없음)
- 단위: bbox m, WheelPhysicsControl.position cm -> /100 해서 m(차 위치와 10m 안 확인), max_steer_angle 도
- vehicle.mitsubishi.fusorosa: 있음(0.9.15 블루프린트 라이브러리에 존재). 10.273 x 3.944 x 4.253, 축거 5.630, 앞바퀴 최대 조향 70.0도, 최소 회전 반경 2.049m
- vehicle.mercedes.sprinter: 5.915 x 1.988 x 2.561, 축거 3.662, 70.0도, 1.333m
- vehicle.toyota.prius: 4.514 x 2.007 x 1.525, 축거 2.819, 70.0도, 1.026m
- vehicle.audi.a2: 3.705 x 1.789 x 1.549, 축거 2.506, 70.0도, 0.912m
- 회전 반경 = 축거 / tan(최대 조향각), 자전거 모형 추정. 네 차 모두 API 값이 70도라 반경이 비현실적으로 작다 [추정: 실차 조향각보다 큼]
- 뒷바퀴 최대 조향 0도(네 차 모두)

## 뒷정리
- 스크린샷 figures/s38_props_before.png, s38_props_after.png (같은 시점, RGB 1280x720 fov 90)
- 띄운 액터 전부 destroy. 남은 차량·소품·센서 0
- 설정 복원: synchronous_mode=False, no_rendering_mode=False, fixed_delta_seconds=None
