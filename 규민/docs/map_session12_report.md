# 맵 세션12 보고: 평활화 지도 주행 검증 (2026-09-24)

## Phase 0
- 부팅 11:18:23, CarlaUE4 없음
- Firefox 실행 중이었음(PID 2551, 하위 9개). 사용자가 종료 -> 재확인 0개
- 종료 후 avail 10Gi, swap 0B
- test_drive.py 세션11 갱신판 확인(XODR_PATH=_smooth, 첫 충돌만 print,
  event.timestamp 직접 기록)

## Phase A
- 서버 명령 원문(사용자 별도 터미널, 11:22:27경):
  cd ~/carla/CARLA_0.9.15
  ./CarlaUE4.sh -quality-level=Low -windowed -ResX=800 -ResY=600
- 기본 맵 129s 후: avail 2.3Gi / swap 1.7Gi / gtt 2.95GiB
- 로드: python map/tests/test_load_map_878.py (인자 없음=평활화판), exit 0
- 로드 직후: avail 2.5Gi / swap 818Mi / gtt 4.01GiB
- 2분 후: avail 4.9Gi / swap 815Mi / gtt 1.44GiB
- spawn point 1243개(옛 xodr 1245개, -2. 원인 미조사)

## Phase B-1 사전 점검(서버 기준, 액터 생성 없음)
- spawn road1247 lane-1 s=0.00
- 계획 204점(dry-plan 203점은 spawn s=2.0 가정), 최대 간격 4.38m
- 커넥터 안쪽 24점, 인접 14쌍 next() 연결 OK
- 종점 road1278 s=100.38 (584.78, -679.17)
- 옛 정지 지점 road1247 s=89.35 (227.18, -998.45)
- 로그: docs/logs/session12_precheck.log
- 주의: 첫 시도는 내 점검 코드의 호출 순서 오류(KeyError)로 중단,
  write_plan_csv 먼저 호출하도록 고쳐 재실행. 서버·스크립트 문제 아님

## Phase B-2 주행(1회)
- 명령: python map/tests/test_drive.py, 11:33:37~11:35:31, exit 0
- 결과: 실패 - 정체(speed<0.5m/s 5s) road1240
- 시뮬 113.2s, 궤적 로그 29,544행
- 파일: map/docs/logs/drive_log_20260924_113337.csv,
  collision_events_20260924_113337.csv, docs/logs/session12_drive_stdout.log
- 분석: map/scripts/analyze_drive_s12.py -> map/docs/logs/session12_drive_analysis.txt

### 관찰 1. 옛 정지 지점
- 통과. t=18.229s road1247 s=89.35, speed 5.30m/s, 충돌 0건

### 관찰 2. 충돌
- 2,139건(세션10 1,604건). 전부 road1240 끝(정지 지점), 그 전 구간 0건
  (1건은 궤적 로그에 해당 frame 없음, 크기 31.8)
- other=static.unknown, tags 빈값(세션10 과 같음)
- 첫 충돌 frame 165683, t=104.591s, 크기 3376
- 방향: 2,138건 전부 차를 왼쪽으로 밀었다 -> 접촉면은 차의 **오른쪽**
  (세션10 은 왼쪽). 전후 성분: fwd~0 1,299 / 차를 앞으로 민 것 839.
  첫 충돌 단위벡터 fwd +0.255, right -0.967
- impulse z: 1,294건 비0 이지만 최대 0.65, z/크기 최대 0.008
  -> 사실상 수평 임펄스, 수직면 접촉. 세션10 "z 전부 0.0" 은 CARLA 가 z 를
  0 으로 고정하는 구조 때문은 아님(이번엔 비0 값이 나옴)

### 관찰 3. junction1
- r1914 로 통과(197행). r1915 는 t=30.28s s=0.45 에서 31행(약 0.12s)
  잡힘. 두 커넥터 시작점이 겹친 곳에서 get_waypoint 가 r1915 를 고른 것으로
  보이며, 이후 r1914 -> road1356 으로 진행. U턴 안 함 -> 세션5 폭주 가설의
  "r1915 U턴" 경로는 이번 주행에서 발생하지 않음
- 같은 성격의 순간 전이: r1380(t=0.98), r1916, r1449, r1445,
  r1429/1434/1431/1432. 비교표의 "불일치" 는 이 순간 전이 때문이며
  실제 경로 이탈은 아님(추정: 겹치는 커넥터 투영)

### 관찰 4~5. 커넥터
- r1446 통과(t=49.26~51.18s), r1428 통과(t=74.18~76.69s)
- r1917 통과(t=38.06~38.74s)

### 관찰 6. 완주
- 미완주. 궤적 누적 547.7m / 781m(70%), 정지까지 104.6s, 평균 18.9km/h, 최고 19.7km/h

## 정지 분석
- 위치: road1240 s=148.11(첫 충돌) -> 최종 s=148.68,
  (420.24, -662.02, z=-0.01). road1240 길이 149.50m, 끝 1.4m 앞.
  다음은 junction60(OSM 노드 4402727513) 커넥터 r1695(15.12m) -> road1239
- 직전 3초(0.25s 간격 요약):
  t=101.6~103.9 speed 5.29, thr 0.56~0.57, brake 0, steer ~0, yaw 45~46
  t=104.11 steer -0.394, roll +0.52 (좌회전 시작)
  t=104.36 steer -0.239, roll +1.05, yaw 35.8
  t=104.61 speed 1.76, thr 0.68, pitch -0.21 (충돌)
  t=104.86~ speed 0.04~0.11, thr 0.85, brake 0, pitch +0.5~0.7
  최종 steer -0.80, yaw 9.8
- 충돌 시각 = 감속 시각(t=104.591s 첫 충돌, 같은 행에서 speed<4.5 첫 기록)
- 판정: 세션10 과 같은 유형(throttle 0.85, brake 0, speed 0, 충돌·감속 동시).
  단 접촉면은 반대쪽(오른쪽), 위치는 junction 진입부
- 잔여 61곳 대조: road1240 없음, 경로 15 road 전체 해당 없음.
  잔여 목록은 plain road 만 담고 있어 junction 내부 커넥터는 애초에 평가 대상이
  아니었다 -> 새 지점

## 스크린샷용 spectator 좌표(차량은 스크립트가 이미 제거함)
- 위: Location(420.2, -662.0, 25.0), Rotation(pitch=-90, yaw=-90, roll=0)
  (yaw=-90 이면 화면 위쪽 = CARLA -y = 진북)
- 옆(차 오른쪽 8m): Location(418.9, -654.1, 2.5),
  Rotation(pitch=-10, yaw=-80, roll=0)

## 메모리
- 주행중(스크립트 측정): avail 4.7Gi / swap 683Mi / gtt 1.44GiB
- 주행 후 11:37:04(서버 유지): avail 4.8Gi / swap 683Mi
- 중단 기준 도달: 아니오

## 정리(대기 중)
- 충돌 센서·차량: 스크립트가 제거함(출력 확인)
- 서버 종료: 사용자가 서버 터미널에서 종료, 11:47:37 pgrep -fa CarlaUE4 결과 없음
- 종료 후: avail 6.6Gi / swap 318Mi / gtt 0.09GiB. 시작 전 10Gi 대비 약 3.4Gi 미회복
  (세션 기록의 약 4Gi 미회복 패턴 재확인, 재부팅으로 해소 예정)

## 스크린샷(서버 유지 중 촬영, 차량 없음)
- 원본 ~/Pictures/Screenshots/Screenshot from 2026-09-24 11-46-24.png
  -> map/docs/figures/16_s12_stop_point_topdown.png (cp -n)
- 원본 ... 11-46-36.png -> map/docs/figures/17_s12_stop_point_side.png
- 관찰(판정 아님): 위 시점에서 도로 가장자리를 따라 어두운 띠가 있고,
  옆 시점에서는 그 띠가 지면 위로 선 얇은 수직 판(높이 약 1m 로 보임)이다.
  junction 모서리에서 판 끝이 도로 쪽으로 튀어나와 보이는 곳이 있다.
  wall_height=1.0 기본값과 일치하는 모습이지만, 차가 이 판에 닿았는지는
  스크린샷만으로 확정할 수 없다(차량이 제거된 상태, 축척 미보정)

## 추가 조사: wall_height (읽기 전용, 서버 없음)

### 1. OpendriveGenerationParameters 7개 (공식 문서 + 로컬 0.9.15 기본값 일치)
근거: https://carla.readthedocs.io/en/0.9.15/python_api/ (carla.OpendriveGenerationParameters)
- vertex_distance 2.0: 메시 정점 간격
- max_road_length 50.0: 메시 조각 최대 길이
- wall_height 1.0: "Height of walls created on the boundaries of the road.
  These prevent vehicles from falling off the road."
- additional_width 0.6: junction 차선 추가 폭
- smooth_junctions True: junction 메시 평활화
- enable_mesh_visibility True: 도로 메시 렌더 여부
- enable_pedestrian_navigation True: Recast 보행자 내비
- 조정 범위: 문서에 없음. 소스(MeshFactory.cpp)에 wall_height 검사·제한 없음
  (resolution>0 만 RELEASE_ASSERT). 0 이면 높이 벡터가 (0,0,0) 이 되어 넓이 0 인
  삼각형 띠가 생긴다. 그 삼각형이 충돌로 남는지는 확인 안 함(UE 쿠킹 동작, 미확인)
- 주의: MeshFactory.h 의 RoadParameters 기본값은 wall_height 0.6 이지만
  MeshFactory(params) 생성자가 rpc 값(1.0)으로 덮어쓴다(MeshFactory.cpp:25)

### 2. 벽의 목적
근거: https://carla.readthedocs.io/en/0.9.15/adv_opendrive/
- "Visible walls are created at the boundaries of the road, to act as a last
  safety measure." 절차 생성 메시 밖은 빈 공간이라 떨어지지 않게 하는 장치

### 벽이 서는 위치 (공식 GitHub 소스 0.9.15)
근거: https://github.com/carla-simulator/carla/blob/0.9.15/LibCarla/source/carla/road/MeshFactory.cpp
- GenerateAllWithMaxLen: junction 이 아닌 road 에만 벽 생성(junction 안에는 없음)
- GenerateWalls: min_lane(가장 오른쪽) 오른쪽 가장자리, max_lane 왼쪽 가장자리.
  lanes 맵의 마지막 키가 0 이면 max_lane=-1
- 이 지도의 half-road 는 전부 오른쪽 차선만 있다(center 0, -1 driving 3.35,
  -2 sidewalk 2.80. road1240·1247·1695 확인). 따라서
  **왼쪽 벽 = lane -1 왼쪽 가장자리 = 기준선(t=0)**. 쌍둥이 road 도 같은 선에
  자기 벽을 세운다. 즉 양방향 도로 중앙선에 높이 1m 벽이 있다
- 오른쪽 벽: t=-6.15m(인도 바깥)
- 벽은 GenerateRightWall/LeftWall 에서 높이 벡터 (0,0,wall_height) 만 더한다 -> 수직면

### 3. 벽을 없애거나 낮출 때의 부작용
- 공식 문서상 벽은 추락 방지 장치다. 없애면 차가 도로 메시 밖으로 나갈 때
  막는 것이 없어 빈 공간으로 떨어질 수 있다(문서 취지에서 추론, 실험 안 함)
- 중앙선 벽도 같이 사라진다. 이번 두 정지 원인 후보가 바로 이 벽이라 제거하면
  풀릴 가능성이 있다(추정, 미검증)
- 이 지도는 인도가 차선 바깥 2.8m 로 붙어 있어 바깥 벽은 인도 끝에 선다.
  TM 이 차선을 약간 벗어나도 인도 위라 바로 추락하지는 않는다(추정)
- 낮추는 경우(예 0.1~0.3m) 바퀴·bbox 에 걸리는지는 모름
- wall_height 는 전 지도 공통값이다. 특정 선(중앙선)만 없애는 매개변수는 없다.
  중앙선 벽만 없애려면 xodr 구조(쌍둥이 half-road -> 양쪽 차선 가진 1개 road)를
  바꿔야 한다. 소스상 lanes 에 양수 차선이 있으면 max_lane 이 그 차선이 되어
  왼쪽 벽이 반대편 인도 바깥으로 간다(소스 해석, 미검증)

### 4. 스크린샷(17_s12_stop_point_side.png) 수직 판 높이
- 방법: 카메라 z=2.5, pitch -10, FOV 90(spectator 기본, 가정), 렌더 800x600,
  렌더 영역 시작 32행(제목줄). 예측 수평선 261.5행, 실측 경계 263~268행 -> 가정 정합
- 판 윗변·아랫변 픽셀에서 거리 D 와 높이 H 동시 해 (핀홀 모델)
- 가까운 판: D 9.3~10.4m, H 0.94~0.97m (4열)
- 먼 판: D 14.7~18.7m, H 1.05~1.08m (4열)
- 결론: 약 0.94~1.08m, wall_height=1.0 과 맞음(픽셀 ±1 오차 범위)

### 5. 충돌 임펄스 z
- 세션10(collision_events_20260922_213844.csv): 1604건 전부 0.0
- 세션12(collision_events_20260924_113337.csv): 2139건 중 1294건 비0,
  최대 0.652, z/크기 최대 0.008 -> 사실상 수평. 둘 다 수직면 접촉과 일치
- CSV 는 소수 4자리 반올림이라 세션10 의 0.0 은 |z|<5e-5 일 수 있음

### 6. 잔여 61곳 대조
- 분류: 원본 내부노드 29 / 고정노드(공유·끝점) 근처 28 / 호 근처 3 / 호+고정 1
- road1240 은 목록에 없음. 정지 지점에서 가장 가까운 잔여 지점은
  road1298 s=61.47 로 100.2m 떨어짐(원본 내부노드). "junction 근처 28곳" 해당 없음

### 7. 판정 보조: 벽 재구성 거리 (map/scripts/wall_proximity_s12.py, 읽기 전용)
- 위 소스 규칙대로 반경 40m 안 road 의 벽 선을 xodr 에서 재구성, 첫 충돌 frame
  차량 bbox(1.853x0.894) 와 거리 계산
- 첫 충돌 시 차 (420.08,-662.67) yaw 30.0
- 가장 가까운 벽: road1240/쌍둥이 road1371 **중앙선 벽**, FL 모서리에서 0.25m,
  벽 끝점 (421.9,-662.5) = road1240 끝(junction60 경계).
  벽 끝이 차 기준 lat -0.77 lon +1.64 로 bbox(반폭 0.894, 반길이 1.853) 안에 있음
  -> 차 앞부분이 이미 중앙선 벽 끝을 넘어 걸친 상태
- 다음 후보: road1240 바깥 벽 3.65m, road1371 바깥 벽 5.98m
- 해석(추정): 좌회전을 road 끝 4m 앞(s≈145.7)에서 시작해 앞머리가 중앙선을
  넘었고, 끝난 벽 끝이 차 앞부분 오른쪽에 끼었다. 이 경우 임펄스가 차를 왼쪽·앞으로
  미는 것(관찰: 왼쪽 2138건, 앞 839건)과 맞는다. "접촉면=오른쪽" 판정과 벽 위치
  (차 왼쪽 선)가 어긋나 보이던 것은 벽 끝을 넘어선 뒤의 접촉이라는 것으로 설명된다
- 세션10 과의 연결: 세션11 재구성에서 정지 구간 왼쪽 모서리 최대 t=+0.19
  (기준선 넘음)였다. 기준선에 1m 벽이 있다는 이번 소스 확인과 맞는다.
  세션10 후보 (b) "half-road 기준선 경계면" 은 실체가 중앙선 벽일 가능성이 높다
- 한계: 벽 위치는 소스 규칙과 xodr 로 재구성한 값이다. 실제 메시(smooth,
  junction additional_width)는 CARLA 안에서 확인하지 않았다

## A안: wall_height=0 (재변환 없음, 로드 파라미터로 처리)

### 전제 정정
- 지시 원안은 osm_to_xodr.py 의 OpendriveGenerationParameters 수정이었으나,
  osm_to_xodr.py 는 carla.Osm2Odr.convert + Osm2OdrSettings(11개, wall 항목 없음)만
  쓰고 xodr 안 "wall" 문자열은 0건. 벽은 서버 로드 시 generate_opendrive_world 에
  넘기는 OpendriveGenerationParameters 로 생성된다 -> 재변환 불필요(사용자 승인)

### C-1 test_load_map_878.py 변경(사용자 승인)
- OpendriveGenerationParameters 7개 전부 인자화:
  --vertex-distance --max-road-length --wall-height --additional-width
  --smooth-junctions --enable-mesh-visibility --enable-pedestrian-navigation
- 미지정 항목은 CARLA 생성자 기본값 유지(인자 없이 실행하면 이전과 동일)
- bool 은 true/false 로 지정(잘못된 값은 argparse 오류)
- 로드 전에 실제 적용된 7개 값을 전부 출력, 인자로 바꾼 항목은 * 표시
- 변경 이유 주석 추가(사용자 지정 문구 + 근거 URL)
- 확인(서버 없이): py_compile OK, --help 7개 표시·CARLA 기본값 표기,
  --smooth-junctions maybe -> 오류, 로컬 객체 setattr(wall_height 0.0,
  smooth_junctions False) 반영 확인. 서버 적용 여부는 다음 세션 로드 때 확인
- 원본 백업: 세션 스크래치패드(프로젝트 밖)

### C-2 검증
- xodr 는 바뀌지 않음: 평활화판 sha256 bf835cdfad0cea65... (세션11 기록과 동일)
- 따라서 road/junction 878/95, road id, 경로 780.97m, 종점 출입구 16.85m 는
  세션11 검증값 그대로 유효(파일 동일)
- 벽 기하는 xodr 에 원래 없음(wall 0건). 빠졌는지는 xodr 로 확인할 수 없고
  서버 로드 후 화면·주행으로만 확인된다

### C-3 미확인 항목
- 높이 0 인 띠(넓이 0 삼각형)가 충돌 판정에 남는지 미확인. 주행으로만 확인된다
- 차가 도로 밖으로 나갔을 때 떨어지는 위험. 경로 추종 실험이라 도로 이탈은
  드물지만, 발생하면 궤적 로그(z, on_road)에 남는다
- 벽 위치는 소스 규칙으로 재구성한 값이고 실제 메시는 미확인
- 벽을 없애면 바깥 벽(인도 끝)도 같이 사라진다. 중앙선 벽만 없애는 파라미터는 없음

### C-4
- test_drive.py 수정 없음. XODR_PATH 는 평활화판 그대로(76행), xodr 동일하므로
  dry-plan 재실행 생략(세션11 결과 203점, 최대 간격 4.48m 유효)

## 다음 세션(13) 실행 계획
1. 재부팅 -> 브라우저 부재 확인(pgrep -fa firefox, content 프로세스) ->
   서버 기동(사용자, 시작 맵 인자 없음, 명령 원문 기록) -> 기본맵 2분 후 측정
   (avail 2Gi 미만이면 중단)
2. python map/tests/test_load_map_878.py --wall-height 0
   (출력 전체를 로그로 남긴다. 적용 파라미터 7개 중 wall_height 만 * 인지 확인)
   로드 직후·2분 후 메모리, spawn point 개수
3. python map/tests/test_drive.py (1회, 같은 조건 재시도 금지)
4. 관찰:
   - road1240 끝(junction60 입구, s≈148) 통과 여부
   - 충돌 이벤트 건수(세션10 1604, 세션12 2139)
   - junction1 커넥터 r1914 vs r1915
   - r1446 / r1428 / r1917 통과
   - 781m 완주(road1278 s>=95, 종점 5m 이내)
   - 도로 이탈·낙하 여부(z 급감, on_road False)
5. 완주하면 궤적 그림 11_drive_trajectory.png 생성(세션11 오버레이와 같은
   축척·범위, 진북 위쪽)
6. 또 막히면 접촉 위치와 임펄스 방향으로 벽 외 원인 재조사
   (wall_proximity_s12.py 방식으로 주변 기하와 거리 계산)

## 마감 (11:59)
- CARLA 프로세스 없음(ps 확인), avail 6.1Gi / swap 313Mi (약 4Gi 미회복 상태 유지)
- 결론: 평활화로 옛 정지 지점(road1247 s≈89.35)은 해소. 새 정지 지점
  road1240 끝(junction60 입구)은 중앙선 1m 벽(wall_height, half-road 구조)이
  유력 원인. 다음 세션에서 같은 xodr 를 --wall-height 0 으로 로드해 검증
- 산출물: map/docs/map_session12_report.md, map/scripts/analyze_drive_s12.py,
  map/scripts/wall_proximity_s12.py, map/tests/test_load_map_878.py(인자 7개),
  map/docs/figures/16·17, map/docs/logs/drive_log_/collision_events_20260924_113337.csv,
  docs/logs/session12_*.log
- 미결 추가: 높이 0 띠 충돌 잔존, 벽 제거 시 낙하 위험, 실제 메시 미확인,
  spawn point 1245->1243 원인, 중앙선 벽만 없애려면 xodr 구조 변경 필요(보류)
