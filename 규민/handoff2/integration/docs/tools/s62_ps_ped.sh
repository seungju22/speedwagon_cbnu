#!/bin/bash
# 세션62: 플래닝 시뮬레이터(컨테이너 s62_ps)에 서 있는 더미 보행자 하나를 놓고, 객체 토픽에 나오는지 본다
# 1.9.0 의 dummy_perception_publisher 는 autoware_simulation_msgs/msg/SimulatedObject 를 받는다(tier4_simulation_msgs/DummyObject 로 보내면 연결 안 됨, 세션62)
# 사용: s62_ps_ped.sh <x> <y>   (기본: 도서관 북문 문 구역 가운데, shuttle_platforms_v31_r1.yaml lib_north door_zone 네 점 평균)
PX=${1:-656.05}; PY=${2:-682.0}; CT=${CT:-s62_ps}
docker exec -u 1000:1000 $CT bash -c "source /opt/autoware/setup.bash
timeout 15 ros2 topic pub --once -w 1 /simulation/dummy_perception_publisher/object_info autoware_simulation_msgs/msg/SimulatedObject \"{header: {frame_id: map}, id: {uuid: [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]}, initial_state: {pose_covariance: {pose: {position: {x: $PX, y: $PY, z: 0.0}}}}, classification: {label: 7, probability: 1.0}, shape: {type: 1, dimensions: {x: 0.6, y: 0.6, z: 1.8}}, max_velocity: 0.0, min_velocity: 0.0, action: 0}\" > /dev/null
timeout 10 ros2 topic echo --once /perception/object_recognition/objects | grep -E 'label:|  x:|  y:' | head -6" 2>&1
