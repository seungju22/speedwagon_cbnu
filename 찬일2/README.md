# 찬일 / 셔틀 호출·운행 로직

담당: 경로계획, 판단

충북대 캠퍼스 자율주행 셔틀 프로젝트(speedwagon)의 셔틀 호출·운행 로직. Autoware Planning Simulator 위에서 동작하며 Autoware AD API만 사용한다 (ROS 2, rclpy).

## 파일

| 폴더 | 파일 | 맵 | 운행 방식 |
|---|---|---|---|
| `v2/` | `shuttle_v2_stopbutton.py` | Autoware 샘플 맵 (sample-map-planning) | 출발지 → 도착지 호출, 정류장 정차 + 정차 버튼 |
| `v3/` | `shuttle_v2_loop6.py` | 팀 맵 `OpenDriveMap_v33d_loop` (순환 노선 v3.3d) | 순환 노선 + 정류장 호출 버튼 + 차 안 도착지 버튼 (중간발표 시연 버전) |
| `v3/` | `shuttle_v2_loop7.py` | 위와 같음 | loop6 + 도착지 미입력 시 문 닫고 제자리 대기 (**최신**) |

---

## v3: 순환 노선 셔틀 (`v3/shuttle_v2_loop7.py`)

### 운행 방식

- 셔틀은 **1 정문 → 2 양성재 → 3 후문 → 4 도서관남문 → 5 도서관북문 → 1 …** 한 방향으로만 돈다.
- 처음에는 호출마다 출발지에서 도착지로 따로 갔는데, 동선 낭비가 커서 순환 노선을 따라 도는 방식으로 바꿨다.
- **눌린 정류장(호출 + 도착지)에만 서고**, 나머지는 경유점으로 지나간다.
- 도착지 버튼이 남아 있으면 아직 탄 사람이 있다는 뜻이다. 그래서 센서 없이도 내릴 사람이 다 내릴 때까지 계속 간다.
- 주행 중에 가는 길 앞쪽 정류장이 새로 눌리면 멈추지 않고 목표를 그 정류장으로 앞당긴다 (`change_route_points`). 방금 지나친 정류장이면 다음 바퀴에 들른다.
- 눌린 버튼이 하나도 없으면 그 자리에서 대기한다. `IDLE_HOME`을 정하면 그 정류장으로 가서 대기한다.

### 버튼

| 입력 | 버튼 | 동작 |
|---|---|---|
| `1`~`5` | 정류장 호출 버튼 | 정류장에서 기다리는 사람이 누름 → 그 정류장에 선다 |
| `d 1`~`d 5` | 차 안 도착지 버튼 | 탄 사람이 내릴 정류장을 누름 → 그 정류장에 선다 |
| `b` | 차 안 하차벨 | 가는 길의 바로 다음 정류장에 선다 (도착지 버튼을 깜빡했을 때) |
| `hold` / `release` | 문 잡기 | 누르고 있는 동안 문이 열려 있고, 떼면 `DWELL_SEC` 뒤 출발 |
| `status` | | 현재 상태, 눌린 버튼, 문 상태 출력 |
| `retry` | | 경로 설정이나 자율주행 전환에 실패해 멈췄을 때(HALTED) 다시 시도 |

### 문 (엘리베이터처럼)

- 정류장에 서면 문이 `DWELL_SEC` 동안 열린다. 그 정류장의 호출 버튼이나 `b`를 누르면 다시 연장된다.
- 문이 닫힌 뒤 그 자리에서 대기할 때는 문을 닫아 둔다. 그 정류장 호출 버튼을 누르면 다시 열린다.

### 도착지를 안 누른 승객 (`NO_DEST_POLICY`)

호출 정류장에서 문이 열렸다가 닫힐 때까지 도착지 버튼이 하나도 안 눌린 경우의 처리 방식이다.

| 값 | 동작 |
|---|---|
| `"wait"` (loop7 기본) | 엘리베이터처럼 문을 닫고 그 자리에서 기다린다. `REMIND_SEC`(20초)마다 "내릴 정류장을 눌러 주세요"를 안내한다. 그 사이 다른 정류장 호출이 오면 그대로 출발하고, 탄 사람은 가는 중에 도착지를 누르면 된다. 아무도 안 탔으면 그대로 대기하므로 빈 차로 도는 일이 없다. |
| `"loop"` (loop6 방식) | 탄 사람이 도착지를 안 눌렀을 수 있다고 보고, 그 정류장으로 돌아올 때까지 한 바퀴 돈다. 도중에 도착지 버튼이 눌리면 이 규칙은 풀린다. |

### 상태

| 상태 | 의미 |
|---|---|
| `IDLE` | 할 일 없음, 정류장에서 대기 |
| `STARTING` | 경로 설정과 자율주행 전환 중 |
| `MOVING` | 이동 중 |
| `DWELL` | 정류장에 정차 중 (문 열림) |
| `HALTED` | 경로 설정이나 전환 실패 → `retry` |

### 정류장

| 번호 | 이름 | 다음 정류장까지 (m) |
|---|---|---|
| 1 | 정문 | 426 |
| 2 | 양성재 | 943 |
| 3 | 후문 | 605 |
| 4 | 도서관남문 | 492 |
| 5 | 도서관북문 | 539 |

좌표는 코드의 `LOCATIONS`에 있다 (`info/circulation_v33d_r1.yaml` 기준). `goal`은 정차 자리, `via`는 지나갈 때 쓰는 경유점(차선 가운데)이다. 거리(`LEG_M`)는 도착 예상 시간 안내용이다.

### 주요 설정값

| 이름 | 값 | 설명 |
|---|---|---|
| `INITIAL_KEY` | `"1"` | 시작할 때 차가 서 있는 정류장 |
| `IDLE_HOME` | `None` | 할 일이 없을 때 돌아갈 정류장 (None이면 그 자리) |
| `NO_DEST_POLICY` | `"wait"` | 도착지 미입력 승객 처리 (`"wait"` / `"loop"`) |
| `DWELL_SEC` | `3.0` | 정차 시간 (테스트용 3초) |
| `EST_SPEED` | `4.2` m/s | 도착 예상 시간 계산용 평균 속도 (약 15 km/h) |
| `PASS_RADIUS_M` | `6.0` | 경유점에서 이 거리 안을 지나가면 "지나쳤다"고 본다 |
| `MIN_STOP_DIST_M` | `10.0` | 이보다 가까운 정류장은 다음 바퀴에 처리 |
| `ENGAGE_RETRIES` | `5` | 자율주행 전환 재시도 횟수 |

### 사용하는 Autoware 인터페이스

| 종류 | 이름 |
|---|---|
| 서비스 | `/api/routing/set_route_points`, `/api/routing/change_route_points`, `/api/routing/clear_route` |
| 서비스 | `/api/operation_mode/change_to_autonomous`, `/api/operation_mode/change_to_stop` |
| 구독 | `/api/routing/state`, `/api/operation_mode/state`, `/localization/kinematic_state` |
| 발행 | `/shuttle/markers` (MarkerArray, RViz 표시) |

### RViz 표시

RViz에서 Add → By topic → `/shuttle/markers` → MarkerArray를 추가한다.

- 초록 = 지금 정차 중(문 열림)
- 파랑 = 다음 정차 / 문 닫고 대기 중
- 노랑 = 눌린 정류장 (WAITING = 호출, GET OFF = 내릴 사람)
- 어두운 회색 = 이번 구간에서 이미 지남
- 회색 = 요청 없음

### 실행 방법

**1. 시뮬레이터 실행 (터미널 A)**

```bash
source ~/autoware/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=$HOME/autoware_map/OpenDriveMap_v33d_loop \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit
```

`map_path`는 각자 맵을 둔 위치로 바꾼다. `~` 대신 `$HOME`을 써야 한다.

**2. 초기 위치 설정**

RViz의 2D Pose Estimate(P 키)로 차량을 **정문(1번) 정류장 안쪽 차선 위**에 올리고, 드래그해서 차선 방향과 맞춘다 (`INITIAL_KEY = "1"`).

**3. 셔틀 노드 실행 (터미널 B)**

```bash
source ~/autoware/install/setup.bash
python3 v3/shuttle_v2_loop7.py
```

conda 같은 가상환경이 켜져 있으면 ROS 파이썬 패키지를 못 찾을 수 있으니 시스템 파이썬으로 실행한다.

### 시연 예시

```
1        # 정문에서 호출 → 정문에서 문 열림
d 3      # 탄 사람이 후문 누름 → 양성재는 지나가고 후문에 정차
4        # 주행 중 도서관남문 호출 → 후문 다음에 도서관남문 정차
```

---

## v2: 호출 + 정차 버튼 (`v2/shuttle_v2_stopbutton.py`)

샘플 맵 위에서 `출발지 도착지`로 호출하는 버전이다.

### 운행 방식

```
호출(출발지 도착지) → 출발지로 이동 → 정차 10초(탑승) → 도착지로 이동 → 정차 10초(하차) → 다음 호출
```

- 운행 중에 들어온 호출은 대기열(FIFO)에 쌓였다가 순서대로 처리한다.
- 정류장은 `LANE_ORDER`(1→2→3→4→5) 순서로 길 위에 놓여 있다고 보고, 반대 방향도 같은 정류장을 역순으로 지나갈 수 있다고 가정한다.

### 정차 버튼 (버스 하차벨과 비슷하게)

- **주행 중에 누르면** 가는 길에 있는 다음 정류장에 10초 정차했다가 원래 목적지로 계속 간다.
  - 예) 1 → 3 이동 중 2 이전에 누르면 2에서 정차, 2를 지난 뒤에 누르면 3(원래 정차 예정)
  - 정류장이 너무 가까워 설 수 없으면(`MIN_STOP_DIST_M` = 10 m) 그다음 정류장에 정차한다.
- **정차 중에 누르면** 남은 정차 시간이 다시 10초로 늘어난다.
- **누르고 있는 동안**(`hold` ~ `release`)은 출발하지 않고, 떼면 10초 뒤에 출발한다.
- 아무도 안 누르면 정차 10초 뒤 알아서 다음 호출로 출발한다.

### 입력

| 입력 | 동작 |
|---|---|
| `1 5` | 호출 (출발지 도착지) |
| `b` | 정차 버튼 |
| `hold` / `release` | 정차 버튼 누르기 / 떼기 |
| `status` | 현재 상태 출력 |
| `retry` | 목표 전송이나 자율주행 전환 실패 시 다시 시도 |

### 상태

`IDLE`(대기) / `STARTING`(목표 전송·자율주행 전환 중) / `MOVING`(이동 중) / `DWELL`(정차 중: 탑승·하차·정차 요청) / `HALTED`(실패해 멈춤 → `retry`)

### 지점

| 번호 | 이름 |
|---|---|
| 1 | 정문 |
| 2 | 기타 |
| 3 | 남문 |
| 4 | 중앙 |
| 5 | 동문 |

좌표는 Autoware 샘플 맵(sample-map-planning) 기준이고 코드의 `LOCATIONS`에 있다. 샘플 맵은 차선이 방향별로 하나라서 반대 방향 호출은 큰 루프를 돌아 간다.

### 실행 방법

```bash
# 터미널 A
source ~/autoware/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=$HOME/autoware_map/sample-map-planning \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit

# RViz에서 2D Pose Estimate로 차량을 차선 위에 올린 뒤

# 터미널 B
source ~/autoware/install/setup.bash
python3 v2/shuttle_v2_stopbutton.py
```

v2는 목표를 `/planning/mission_planning/goal` 토픽으로 보내고, 위치는 `/localization/pose_twist_fusion_filter/pose_with_covariance`에서 받는다.

---

## 문제 해결

| 증상 | 확인할 것 |
|---|---|
| 자율주행 모드 전환 실패: `The target mode is not available` | 궤적이 나오는지 `ros2 topic hz /planning/scenario_planning/trajectory`로, 전환 가능 여부는 `ros2 topic echo /api/operation_mode/state --once`(`is_autonomous_mode_available`)로 확인한다. 노트북 부하가 크면 RViz의 PointCloud Map 표시를 끈다. 멈췄으면 `retry`. |
| 서비스를 찾을 수 없음 | 셔틀 노드 터미널에서 `source ~/autoware/install/setup.bash`를 했는지, 시뮬레이터가 완전히 뜬 뒤인지 확인한다. |
| 경로가 설정되지 않음 | 초기 위치가 차선 위인지, 방향이 차선 방향과 맞는지 확인한다. 경로 상태는 `ros2 topic echo /api/routing/state`로 본다. |
| 맵이 안 뜸 | `map_path`에 `~`를 썼는지 확인한다. |

## 향후 계획

- 최종 결과에서 팀 기준인 정류장 6개로 맞추기
- CARLA 시뮬레이터에 적용
- 실제 주행 시간 반영, 호출 웹/GUI 화면

## 변경 이력

- v1: 출발지 → 도착지 호출, FIFO 대기열 (샘플 맵, 정차 없음)
- v2: 정류장 정차 10초, 정차 버튼(b / hold / release), 자율주행 전환 재시도와 `retry`
- v3 loop6: 팀 맵(v3.3d 순환 노선)으로 전환, 순환 운행 + 정류장 호출 버튼 + 차 안 도착지 버튼, 주행 중 목표 앞당기기, RViz 마커 (중간발표 시연)
- v3 loop7: 도착지 미입력 시 한 바퀴 대신 문 닫고 제자리 대기 (`NO_DEST_POLICY`)
