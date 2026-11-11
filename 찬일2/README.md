# 찬일 / 셔틀 호출·운행 로직

담당: 경로계획, 판단

충북대 캠퍼스 자율주행 셔틀 프로젝트(speedwagon)의 셔틀 호출과 운행 로직. Autoware Planning Simulator 위에서 동작하며 Autoware AD API만 사용한다.

## 버전 현황

| 버전 | 맵 | 운행 방식 | 상태 |
|---|---|---|---|
| v2 (`shuttle_v2.py`) | Autoware 샘플 맵 (sample-map-planning), 지점 5개 | 호출마다 출발지 → 도착지 왕복, FIFO 대기열 | 아래에 문서화 |
| v3 (최신) | 팀 맵 `OpenDriveMap_v33d_loop` (순환 노선 v3.3d), 정류장 5개 | 순환 노선 + 호출/도착지 버튼 정차 | 코드 업로드 예정 |

### v3 운행 방식 (최신)

- 정류장 5개: 정문 / 양성재 / 후문 / 도서관 남문 / 도서관 북문
- 호출할 때마다 출발지에서 도착지로 따로 가면 동선 낭비가 커서, 순환 노선을 한 바퀴씩 크게 도는 방식으로 바꿨다.
- 정류장에서는 **호출 버튼**만 누르고, 탑승한 뒤 차 안에서 **내릴 정류장(도착지) 버튼**을 누른다.
- 셔틀은 순환 노선을 따라가다가 호출된 정류장과 눌린 도착지에서만 정차한다. 그 외 정류장은 지나간다.
- 승객이 타 있으면 다음 정차 지점까지 계속 간다. 센서 없이 버튼 입력만으로 판단한다.

### 개선 예정

- 탑승 후 도착지를 입력하지 않으면 지금은 한 바퀴를 돈다. 앞으로는 엘리베이터처럼 문을 닫고 제자리에서 대기하도록 바꿀 예정이다.
- 최종 결과에서는 팀 기준인 정류장 6개에 맞출 예정이다.
- CARLA 시뮬레이터에 적용할 예정이다.

---

## v2: 호출 노드 (`shuttle_v2.py`)

터미널에서 `출발지 도착지` 번호를 입력하면 차량이 출발지로 가서 승객을 태운(픽업) 뒤 도착지까지 데려다준다. 이동 중에 들어온 호출은 대기열에 쌓였다가 순서대로 처리한다.

### 주요 기능

- 호출 입력(`1 5` 형식): 출발지 → 도착지 운행
- 선입선출(FIFO) 호출 대기열: 운행 중 들어온 호출은 대기열에 저장했다가, 현재 운행이 끝나면 다음 호출을 처리
- Autoware AD API 사용: 목표 전송, 자율주행 모드 전환, 경로 상태(도착) 감지
- 미리 정한 5개 지점(정문, 기타, 남문, 중앙, 동문)의 좌표로 운행

### 동작 방식

```
IDLE ──호출──▶ MOVING_TO_PICKUP ──출발지 도착──▶ MOVING_TO_DEST ──도착지 도착──▶ IDLE
                                                                    │
                                                  대기열에 호출이 있으면 바로 다음 호출 시작
```

1. `출발지 도착지`를 입력한다. 차량이 쉬고 있으면(IDLE) 바로 시작하고, 운행 중이면 대기열에 추가한다.
2. 출발지 좌표를 `/planning/mission_planning/goal`로 보내고, 3초 뒤 `/api/operation_mode/change_to_autonomous`로 자율주행 모드로 전환한다.
3. `/api/routing/state`가 `ARRIVED`가 되면 출발지에 도착한 것으로 보고 도착지 좌표를 보낸다.
4. 도착지에 도착하면 IDLE로 돌아가 대기열의 다음 호출을 시작한다.

도착 여부는 1초마다 `check_arrival()`에서 확인한다.

### 지점 번호

| 번호 | 이름 |
|---|---|
| 1 | 정문 |
| 2 | 기타 |
| 3 | 남문 |
| 4 | 중앙 |
| 5 | 동문 |

좌표(x, y)와 방향(qz, qw)은 코드 상단의 `LOCATIONS`에 있다. 좌표 기준은 Autoware 샘플 맵(sample-map-planning)이다.

### 요구 사항

- Ubuntu + ROS 2 (Autoware가 지원하는 배포판)
- Autoware 빌드 완료
- Autoware 샘플 맵 (`~/autoware_map/sample-map-planning`)
- 메시지: `geometry_msgs/PoseStamped`, `autoware_adapi_v1_msgs` (`ChangeOperationMode`, `RouteState`)

### 실행 방법

**1. 시뮬레이터 실행 (터미널 A)**

```bash
source ~/autoware/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=$HOME/autoware_map/sample-map-planning \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit
```

`map_path`에는 `~` 대신 `$HOME`을 써야 한다.

**2. 초기 위치 설정**

RViz 툴바의 2D Pose Estimate(P 키)로 차량을 차선 위에 올린다. 클릭한 채 드래그해서 방향도 차선 방향과 맞춘다.

**3. 셔틀 노드 실행 (터미널 B)**

```bash
source ~/autoware/install/setup.bash     # autoware_adapi_v1_msgs 때문에 필요
python3 shuttle_v2.py
```

conda 같은 가상환경이 켜져 있으면 ROS 파이썬 패키지를 찾지 못할 수 있다. 시스템 파이썬으로 실행한다.

### 사용법

```
호출 (출발지 도착지, 예: 1 5): 1 3
[INFO] [shuttle_node]: 목적지 전송: 정문
[INFO] [shuttle_node]: 자율주행 모드 전환 성공, 이동 시작
...
[INFO] [shuttle_node]: 정문 픽업 완료, 목적지로 출발
[INFO] [shuttle_node]: 목적지 전송: 남문
[INFO] [shuttle_node]: 남문 도착, 정차
```

운행 중에 호출하면 대기열에 쌓인다.

```
호출 (출발지 도착지, 예: 1 5): 2 1
[INFO] [shuttle_node]: 기타 -> 정문 요청을 대기열에 추가
```

| 잘못된 입력 | 메시지 |
|---|---|
| 번호가 두 개가 아님 | 형식이 맞지 않습니다. 예: 1 5 |
| 없는 지점 번호 | 알 수 없는 지점 번호입니다. |
| 출발지와 도착지가 같음 | 출발지와 목적지가 같습니다. |

### 코드 구조

| 부분 | 역할 |
|---|---|
| `LOCATIONS` | 지점 번호별 이름과 목표 좌표·방향 |
| `ShuttleNode.call_shuttle()` | 호출 검증, 대기열 추가 또는 즉시 시작 |
| `ShuttleNode.start_trip()` | 출발지로 이동 시작 (MOVING_TO_PICKUP) |
| `ShuttleNode.send_goal()` | 목표 전송 후 자율주행 전환 |
| `ShuttleNode.check_arrival()` | 1초마다 도착 감지, 상태 전환, 대기열 처리 |
| `main()` | 입력 스레드와 ROS 스핀 구성 |

### 알려진 한계 (v2)

- 한 번에 한 호출만 처리한다. 같은 방향 호출이 여러 건이어도 함께 태우지 않는다.
- 대기열이 선입선출이라 가까운 호출을 먼저 처리하지 않는다.
- 이동 중에 들어온 호출은 현재 운행이 끝난 뒤에야 처리한다.
- 목표를 보낸 뒤 고정으로 3초 기다리고 자율주행 전환을 한 번만 시도한다. 실패해도 재시도하지 않아서, 상태가 MOVING_TO_PICKUP에서 풀리지 않고 이후 호출은 대기열에만 쌓인다.
- 출발지에 도착해도 정차하지 않고 곧바로 도착지로 출발한다.
- 비상정지 명령이 없고, 로그(`self.log`)는 메모리에만 저장된다.
- 샘플 맵의 차선이 방향별로 하나라서 반대 방향 호출은 큰 루프를 돌아서 간다.

### 문제 해결

| 증상 | 확인할 것 |
|---|---|
| 자율주행 모드 전환 실패: `The target mode is not available` | Autoware가 자율주행 모드를 거부한 상태다. 궤적이 나오는지 `ros2 topic hz /planning/scenario_planning/trajectory`로, 자율주행 가능 여부는 `ros2 topic echo /api/operation_mode/state --once`(`is_autonomous_mode_available`)로 확인한다. 노트북 부하가 크면 RViz의 PointCloud Map 표시를 끈다. |
| `operation_mode 서비스를 찾을 수 없습니다.` | 셔틀 노드를 띄운 터미널에서 `source ~/autoware/install/setup.bash`를 했는지, 시뮬레이터가 완전히 뜬 뒤인지 확인한다. |
| 경로가 안 생기고 멈춤 | 지점이 차선 위에 있는지, 방향이 차선 방향과 맞는지 확인한다. 경로 상태는 `ros2 topic echo /api/routing/state`로 본다. |
| 맵이 안 뜸 | `map_path`에 `~`를 썼는지 확인한다. |

지점 좌표를 새로 따려면 RViz의 2D Goal Pose로 원하는 위치를 찍고, 목표 토픽을 echo해서 나온 x, y, qz, qw를 `LOCATIONS`에 옮긴다.

```bash
ros2 topic echo /planning/mission_planning/goal
```

## 변경 이력

- v2: 출발지 → 도착지 호출, FIFO 대기열 (Autoware 샘플 맵)
- v3: 팀 맵(v3.3d 순환 노선)으로 전환, 순환 운행 + 호출/도착지 버튼 정차
