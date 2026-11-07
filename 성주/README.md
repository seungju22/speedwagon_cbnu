# 성주 / 셔틀 제어

캠퍼스 셔틀(도서관, 정문, 후문)의 제어 노드. 호출을 받으면 픽업지로 이동해 탑승을 기다린 뒤 목적지로 이동한다. Autoware AD API만 사용한다.

## 파일
- `shuttle_node.py`: 셔틀 제어 노드 (ROS 2 Humble, rclpy)

## 동작 흐름
`IDLE → 픽업지로 이동 → 탑승 대기 → 목적지로 이동 → IDLE`

- 호출은 `/shuttle/command` 토픽(`std_msgs/String`)으로 받고, 대기열에 쌓여 순서대로 처리된다.
- 이미 픽업지에 있으면 이동 없이 바로 탑승 대기로 넘어간다.
- 새 경로를 설정하기 전에 기존 경로를 지운다.

## 명령어
| 명령 | 동작 |
|---|---|
| `call <픽업지> <목적지>` | 호출 접수 |
| `boarded` | 탑승 완료, 목적지로 출발 |
| `pause` / `resume` | 일시정지 / 재개 |
| `estop` / `estop_clear` | 비상정지 / 해제 |
| `speed <km/h>` | 최대 속도 제한 변경 (0~30) |
| `cancel` | 임무와 대기열 취소 |
| `status` | 상태 출력 |

정차 장소: `library`, `front_gate`, `back_gate`

## 실행 방법
터미널마다 환경 설정이 필요하다.

```bash
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | tr ':' '\n' | grep -v acados | paste -sd:):/opt/acados/lib
source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash
```

1. Autoware planning simulator 실행

```bash
ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=$HOME/autoware_map/nishishinjuku_autoware_map \
  vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit \
  planning_module_preset:=test
```

2. RViz에서 2D Pose Estimate로 차량 위치 지정
3. 노드 실행: `python3 shuttle_node.py`
4. 다른 터미널에서 명령 전송

```bash
ros2 topic pub --once /shuttle/command std_msgs/msg/String "{data: 'call library front_gate'}"
ros2 topic pub --once /shuttle/command std_msgs/msg/String "{data: 'boarded'}"
```

## 확인한 것
- planning simulator에서 3지점 순환(도서관 → 정문 → 후문 → 도서관) 3구간의 호출, 픽업, 탑승 대기, 이동, 하차 흐름을 확인했다.

## 한계, 남은 일
- 비상정지, 속도 변경, 정지 이벤트 감시는 시험 진행 중이다.
- 정차 좌표(`STOPS`)는 니시신주쿠 샘플 맵 위의 임의 지점이다. 캠퍼스 맵이 준비되면 좌표만 교체한다.
- AWSIM 연동은 하지 않았다 (개발 노트북에 NVIDIA GPU 없음).
- 센서 기반 장애물 인식은 검증하지 못했다.
- 한 구간에서 원인을 알 수 없는 정지가 한 번 관찰되었다.
