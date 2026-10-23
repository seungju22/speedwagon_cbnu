# 플래닝 시뮬레이터로 개발하기 (CARLA 없이)

셔틀 미션 노드(정류장 도착 -> 문 -> 출발, 경유점, 요청 정차)를 CARLA 없이 만들 때 쓰는 순서임.
Autoware 1.9.0 의 planning_simulator.launch.xml 을 캠퍼스 v3.1 지도(28a57711)로 띄움. 차는 시뮬레이터 안 가상 차이고, 측위·인지는 정답 값임.
세션62(2026-10-15)에 README 의 '한 번만 하는 준비'를 마친 임시 HOME 에서 아래 순서를 그대로 돌렸음. CARLA 는 띄우지 않음.

## 띄우기

준비는 handoff_2_README.md '한 번만 하는 준비'와 같음(지도 폴더 `~/autoware_data/maps/OpenDriveMap`). 점군 지도는 없어도 됨.

    cd ~/autoware/docker/examples/demos/planning-simulator
    docker compose -f docker-compose.yaml -f ~/campus_mobility_sim/docker/planning-simulator.pin.yaml \
      run --rm -T --name s62_ps planning-simulator bash -c \
      'ros2 launch autoware_launch planning_simulator.launch.xml map_path:=/home/aw/autoware_data/maps/OpenDriveMap vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit rviz:=false' \
      > ps_launch.log 2>&1 &

- 1분 30초쯤 뒤에 쓸 수 있음. 화면을 보려면 rviz:=false 를 빼고 `xhost +local:` 을 먼저 함(세션62 는 메모리를 아끼려고 껐음)
- 컨테이너 메모리 2.1~2.8GB, 이 노트북(16GB) 최소 여유 8.6GB. CARLA 를 같이 띄울 때보다 훨씬 가벼움
- 로그에 나오는 오류 가운데 무시해도 되는 것
  - `pointcloud_map_loader ... PCD load failed`, `map_hash_generator ... pointcloud_map.pcd is neither file nor directory`: 점군 지도가 없어서임. 레인렛 지도는 읽히고 주행에 영향 없었음. placeholder 점군은 필요 없었음
  - `net.core.rmem_max` 경고, 시작 직후 진단 그래프 ERROR 줄들: 차를 놓기 전 상태라 그럼
- 끄기: `docker stop -t 20 s62_ps`. `docker ps` 가 비었는지 봄

## 한 구간 달리기 (AD API)

`C/integration/docs/tools/s62_ps_goal.sh <tag> <목표 x y qz qw> [초기 x y qz qw]` 가 아래를 차례로 부름. 값은 노선 파일 2판(`map/data/routes/pilot_route_r2.yaml`) goal 임.

1. `/api/localization/initialize` 로 차를 놓음(처음 한 번)
2. `/api/routing/set_route_points` 로 목표를 줌. 경유점은 waypoints 에 넣음
3. `/api/operation_mode/change_to_autonomous`
4. `/api/routing/state` 가 3(ARRIVED)이 되면 도착

    # p_middle_north: 중문(건너편) -> 도서관 북문
    bash ~/campus_mobility_sim/integration/docs/tools/s62_ps_goal.sh seg1 656.162 684.456 -0.304989 0.952356 619.358 907.515 -0.684987 0.728555
    # 이어서 p_north_yang: 북문 -> 양성재(이쪽). 초기 위치는 안 준다
    TO=500 bash ~/campus_mobility_sim/integration/docs/tools/s62_ps_goal.sh seg2 213.999 550.838 -0.54348 0.839422

세션62 결과(로그 `C/integration/logs/ps_goal_<tag>.log`)
- p_middle_north: 경로 설정 -> 자율 -> 도착 78초, 멈춘 자리와 목표 차이 0.005m
- p_north_yang 이어 달리기: 도착 약 360초, 목표 차이 0.018m. I6-2 CARLA 주행(81.1초, 356.7초)과 비슷함

이어 달리기에서 알게 된 것(미션 노드가 지켜야 할 순서)
- 도착하면 routing state 3(ARRIVED), 운전 모드는 1(STOP)로 돌아감
- ARRIVED 에서 바로 set_route_points 를 부르면 `The route is already set.` 으로 거절됨. 먼저 `/api/routing/clear_route` 를 부름
- 그 뒤 set_route_points -> change_to_autonomous. 스크립트는 state 3 이면 clear_route 를 먼저 부름
- 경로가 없는 STOP 상태에서도 change_to_autonomous 는 성공함. 그러면 다음 경로를 넣는 순간 바로 출발함. 문이 닫히기 전에 경로를 넣지 않음

## 더미 보행자

`s62_ps_ped.sh [x y]`. 기본값은 도서관 북문 문 구역 가운데(656.05, 682.0, `shuttle_platforms_v31_r1.yaml` lib_north door_zone 네 점 평균).
`/simulation/dummy_perception_publisher/object_info` 에 서 있는 보행자(label 7, 원기둥 0.6x0.6x1.8m)를 하나 보내고 `/perception/object_recognition/objects` 를 한 번 읽음.

- 1.9.0 은 메시지 형식이 `autoware_simulation_msgs/msg/SimulatedObject` 임. 예전 문서의 `tier4_simulation_msgs/msg/DummyObject` 로 보내면 연결이 안 돼 publish 가 끝나지 않음
- 더미 인지는 차에서 300m(visible_range) 안의 것만 냄. 차가 양성재에 있을 때 북문 문 구역(461m)에 놓은 보행자는 안 나왔음. 차를 북문 정차 자리에 다시 놓은 뒤에는 label 7, (656.05, 682.00) 으로 나왔음
- 보행자 때문에 출발이 막히는지(문 구역 판단)는 이번에 안 봤음. 미션 노드가 이 토픽으로 직접 판단하면 됨

## 가상 문

`launch_dummy_doors` 는 기본으로 켜져 있음. 문 4개(dummy door 1~4).
`s62_ps_door.sh open|close [index]` 가 `/api/vehicle/doors/command`(SetDoorCommand, OPEN 1 / CLOSE 2)를 부르고 `/api/vehicle/doors/status` 를 1초마다 7번 읽음.

- 열기: success, 상태 3(CLOSED) -> 4(OPENING) -> 약 2초 뒤 2(OPENED)
- 닫기: success, 5(CLOSING) -> 약 2초 뒤 3(CLOSED)
- 문 배치는 `/api/vehicle/doors/layout` 으로 봄

## CARLA 쪽과 다른 것

- 차가 CARLA 차가 아니라 시뮬레이터 안 가상 차(sample_vehicle)임. 승강장 정지 오차, 회전 반경은 CARLA 회차(handoff_2_README)로 다시 봄
- 장애물은 손으로 놓은 더미뿐임. 장면 도구(ped_tool, obstacles)는 CARLA 용이라 여기서 안 씀
- 세션62 로그: `C/integration/logs/s62/`(ps_launch.log, monitor.log), 보고서 `integration/docs/s62_report.md`

근거: https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim , AD API https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/
