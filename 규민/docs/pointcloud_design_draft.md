# 점군 지도 사전 설계 초안 (맵세션18 야간 배치, 코드 없음)

상태: 설계 초안. 구현(build_pointcloud.py) 안 함. 수치 중 "가정"으로 표시한 것은 실측·공식 문서 대조 전 값.
근거 원칙(CLAUDE.md): 공식 문서만. Autoware 문서는 main 기준이므로 실제 값은 1.9.0 태그 파일로 다시 확인할 것.

## 0. Autoware 가 요구하는 맵 3개와 현재 상태
- pointcloud_map.pcd: 없음. 이 문서의 대상
- lanelet2_map.osm: 없음. xodr 에서 변환 필요(별도 설계). 원본 OSM 과 다른 포맷
- map_projector_info.yaml: 없음
- 참고 실물(세션 2026-09-13 다운로드, ~/autoware_data/maps/Town01): Autoware 가 CARLA Town01 용으로 배포한 지도
  - map_projector_info.yaml 내용 "projector_type: Local" 한 줄(22B)
  - pointcloud_map.pcd: PCD v0.7, FIELDS x y z, SIZE 4 4 4, TYPE F, POINTS 16,726,884, DATA binary_compressed, 190MB
  - 즉 CARLA 맵용 공식 예시는 로컬 좌표 + xyz 만 있는 압축 바이너리 PCD 다. 이 형식을 기본안으로 삼는다

## E-1. NDT 특징 부족 문제 (최우선)
### 현재 맵에 있는 수직 구조
- xodr elevation 878개 전부 a=0 (완전 평면). 경사면 없음
- 로드 옵션 --wall-height 0: 세션12 에서 중앙선 위 1m 벽(충돌 1604건 원인)을 없앤 설정. 벽은 수직 특징이었지만 주행을 막아 제거됨
- 남은 수직 요소: 인도(lane -2, 폭 2.80m) 연석 턱 하나. 높이는 미측정(세션14 "인도 턱 위로 올라감" 기록만 있음. 가정: 0.1~0.2m)
- 도로 밖은 void(세션15 "도로 밖 지면이 void" 기록). 건물·수목 없음
- 결론: LiDAR 로 찍히는 것은 평평한 노면 + 좌우 낮은 연석 두 줄뿐

### 특징이 부족하면 측위가 어떻게 깨지는가 (원리, 가정 포함)
- NDT 는 스캔 점군을 지도 점군의 복셀 정규분포에 맞춰 6자유도 포즈를 푼다
- 평면만 있으면 z, roll, pitch 는 잡히지만 x, y, yaw 는 구속이 없다(평면 위 어디로 미끄러져도 점수가 같음)
- 연석 두 줄이 있는 직선 구간: 도로 폭 방향(횡)과 yaw 는 연석이 잡아 주지만, 도로 진행 방향(종)은 여전히 구속 없음
  -> 직선 구간에서 종방향 위치가 흘러간다(퇴화, degenerate). 곡선·교차로에서만 종방향이 잡힌다
- 연석 높이가 LiDAR 수직 분해능보다 작으면 연석조차 몇 줄의 링에만 걸려 횡방향 구속도 약해진다
- 증상 예상: 직선 road1172(119m), road1240(150m) 같은 구간에서 NDT 수렴 점수는 좋은데 위치가 앞뒤로 틀어짐 -> 정차점 오차, 커브 진입 시점 오류
- 이 판단은 실측 전 이론이다. 짧은 구간 시험(아래 E-2 마지막)으로 먼저 확인할 것

### prop 배치는 장식이 아니라 측위 요구사항인가
- 잠정 결론: 예, NDT 를 쓸 거라면 요구사항이다. 최소한 직선 구간마다 종방향을 잡아 줄 수직 구조물(건물 벽, 기둥, 나무 줄기)이
  LiDAR 범위 안에 계속 보여야 한다
- 세션15 사실: static.prop.mesh 로 나무·건물 소환 가능(5/5), 5개에 메모리 약 0.1Gi 증가, scale 동작
- 작업량 영향: 캠퍼스 건물 OSM footprint(building=* way)를 prop 배치 좌표로 쓰면 수작업을 줄일 수 있다(가정, 미조사)
- 대안(작업량 비교용, 채택 아님):
  (a) NDT 대신 GNSS 기반 포즈(CARLA GNSS 센서 -> Autoware gnss 입력). 특징 부족과 무관하지만 Autoware 1.9.0 에서 설정 경로 확인 필요
  (b) 시뮬레이션 단계에서는 CARLA 참값 포즈를 localization 출력으로 쓰고, NDT 는 prop 을 넣은 뒤 별도 실험으로 분리
  (c) 벽을 낮은 높이(예 0.3m)로만 되살려 수직 특징 확보 — 세션12 충돌 원인이 중앙선 위 벽이었으므로 인도 바깥쪽에만 둘 수 있는지가 관건(CARLA 옵션으로는 위치 지정 불가로 보임, 가정)
- 사용자 결정 필요: NDT 측위를 실험 항목으로 넣을지, (b)처럼 분리할지. 이 결정이 prop 작업량을 정한다

## E-2. 설계 항목
### LiDAR 설정 (CARLA sensor.lidar.ray_cast 속성, 근거 https://carla.readthedocs.io/en/0.9.15/ref_sensors/#lidar-sensor)
- channels 32, range 100m, upper_fov +10°, lower_fov -30°(가정: 근거리 노면·연석 밀도 확보용)
- rotation_frequency 20Hz: 동기 모드 fixed_delta 0.05s(20Hz) 이므로 한 tick 에 한 바퀴가 되도록 1/delta 와 맞춘다
  (CARLA 문서: 한 tick 에 스캔되는 각도 = rotation_frequency x delta x 360°)
- points_per_second 600,000 -> tick 당 약 30,000점(가정). 지도 제작용이라 주행용보다 밀도 우선
- 장착 높이 약 1.8m(차 지붕 위, 가정). noise·dropoff 는 지도 제작 시 0(깨끗한 지도), 측위 시험 시 켬
- 좌표: CARLA 센서 점군은 센서 기준 왼손 좌표(y 반전). 지도에 합칠 때 차량 포즈(시뮬 참값)로 월드 변환 후 y 부호 반전

### 스캔 경로
- 기본안: 4개 루트(north/south/middle, 재변환 뒤 back) 를 따라 주행하며 스캔. 측위가 필요한 곳은 셔틀 노선 위뿐
- 격자 주행은 878 road 전체(27.7km, 일반 road 18.4km)를 덮어야 해 과하다. 노선 밖은 필요 시 추가
- 모든 road 가 일방통행(반쪽 도로)이라 루트를 한 방향으로만 달리면 반대편 쌍둥이 road 는 비스듬히만 찍힌다. 노선만 쓰면 문제 없음
- 속도 10km/h(현재 20km/h 의 절반, 가정): 점 밀도 2배, 급커브 이탈 감소
- 정답 포즈: 시뮬 참값(vehicle.get_transform)으로 점을 누적. SLAM 은 쓰지 않는다(지도 오차를 없애기 위해)

### 복셀 다운샘플링
- 최종 지도 0.2m (가정. Autoware 지도 로더·NDT 가 쓰는 해상도 범위와 맞출 것 — 1.9.0 설정 파일로 확인 필요)
- 누적 중간 단계 0.1m 로 먼저 줄이고 마지막에 0.2m
- 노면이 대부분이라 노면 점을 더 성기게(예 0.5m) 두고 수직 구조는 0.2m 로 두는 이중 해상도도 후보

### 좌표계와 map_projector_info.yaml
- xodr 좌표 = 로컬 TM(+proj=tmerc +lat_0=36.627298 +lon_0=127.456394 +ellps=WGS84) + header offset(521.51, 493.61)
- CARLA 월드 = (x_xodr, -y_xodr). 세션16 에서 좌표 변환 검증(출발점-OSM 정문노드 1.7m)
- 기본안: projector_type Local (Town01 예시와 같음). PCD·Lanelet2 모두 xodr 좌표(= CARLA x, -y) 그대로
  - 장점: 변환이 y 반전 하나뿐. 단점: 위경도와 연결 안 됨(GNSS 쓰면 문제)
- 대안: TransverseMercator(lat_0/lon_0 = 위 값) 로 두고 PCD·Lanelet2 에서 offset(521.51, 493.61)을 빼서 TM 원점 기준으로 저장
  - GNSS 를 쓸 경우 필요. Autoware 1.9.0 의 map_projector_info 지원 형식은 태그 안 파일로 확인할 것(미확인)
- 어느 쪽이든 PCD, Lanelet2, CARLA 브리지(autoware_carla_interface)의 좌표가 같은 원점이어야 한다. 세 개를 같은 세션에서 정할 것

### PCD 포맷
- binary_compressed, FIELDS x y z, float32 (Town01 예시와 같은 형식). 필요하면 intensity 추가(CARLA ray_cast 는 intensity 필드 제공)
- 파일 하나로 둘지 격자 분할(Autoware 분할 지도 로딩)로 둘지는 크기 보고 결정. 아래 추정으로는 하나로 충분

### 메모리 전략 (12GiB 머신, CARLA 실행 중 avail 약 4.3~4.5GiB)
- 원시 누적은 불가능한 규모: 30,000점 x 20Hz x 150s(한 루트) = 약 9천만 점 x 12B = 약 1.1GB. 4루트면 4GB 이상
- 따라서 누적과 동시에 복셀 축소: 매 N tick(예 100 tick = 5s) 마다 버퍼를 0.1m 복셀로 줄여 전역 복셀 해시에 합치고 버퍼 비움
- 최종 크기 추정(가정): 노선 폭 약 6m(차선 3.35 + 인도 2.8) x 노선 길이 약 3km = 약 18,000m², 0.2m 복셀이면 면당 25점/m²
  -> 약 45만 점 + prop 수직면. 수 MB~수십 MB. Town01(1,670만 점, 190MB)보다 훨씬 작다
- 878 road 전체를 찍어도 약 13만 m² -> 약 330만 점, 약 40MB 수준(가정). 전체를 한 번에 들고 있는 것 자체는 문제 아님.
  문제는 원시 누적이며, 위 증분 축소로 피한다
- 스캔은 루트별 별도 실행 -> 루트별 PCD 저장 -> 오프라인(서버 끈 상태)에서 병합. CARLA 와 병합 작업을 동시에 메모리에 올리지 않는다
- 메모리 중단 기준(avail<2GiB, swap>4GiB)은 스캔 주행에도 그대로 적용

### 짧은 구간 시험 (구현 첫 단계로 권장)
- 직선 road1172(119m) 한 구간만 스캔 -> PCD -> 같은 구간을 다시 달리며 NDT(또는 오프라인 ICP)로 정합 -> 종방향 오차 확인
- 이 결과로 E-1 의 "평면·연석만으로는 종방향 퇴화" 를 확인하거나 기각한 뒤 prop 작업량을 정한다

## E-3. VRAM 제약
- 실측: iGPU(Radeon 680M) mem_info_vram_total 2.00GiB, gtt_total 6.36GiB (2026-09-28 읽기전용 확인)
- 세션17 서버 기동 8s 에 vram 1.79GiB(총량의 약 90%), 안정 후 1.28~1.40GiB. 로드 2분 후 gtt 1.52~1.54GiB
- VRAM 이 차면 드라이버가 GTT(시스템 RAM)로 넘긴다. 즉 VRAM 부족은 곧 RAM 여유(avail) 감소로 나타난다
- prop(메시·텍스처)은 VRAM 을 먹는다. 세션15 에서 5개로 약 0.1Gi 늘었지만(메모리), 수백 개 배치 시 선형 증가 여부는 미측정
- LiDAR ray_cast 는 CPU 쪽 물리 레이캐스트라 VRAM 부담은 작을 것으로 본다(가정). 대신 CPU·RAM 부담
- 설계 제약:
  1. prop 은 노선에서 LiDAR 범위(100m) 안, 종방향 특징이 필요한 직선 구간 위주로 최소 개수만
  2. prop 을 10개, 50개, 100개로 늘려가며 vram_used, gtt_used, avail 을 매번 측정(로드 2분 후 기준)
  3. 멈추는 기준: 기존 avail 2GiB / swap 4GiB 그대로. VRAM 자체 기준은 없으므로 gtt 증가량을 같이 기록
  4. 스캔 중에는 spectator 추종(--follow)과 창 해상도를 낮게 유지(-quality-level=Low). 오프스크린 렌더링은 공식 옵션 확인 후 검토

## 결정 필요 (사용자)
1. NDT 측위를 이 프로젝트의 실험 항목으로 둘지, 시뮬 참값/GNSS 로 대체하고 NDT 를 분리할지 -> prop 작업량이 여기서 정해진다
2. projector_type Local 과 TransverseMercator 중 무엇으로 할지(GNSS 사용 여부와 연결)
3. 재변환(후문·중문) 채택 여부. 채택한다면 점군 지도보다 먼저 (s18_D_reconvert.md 참조)
