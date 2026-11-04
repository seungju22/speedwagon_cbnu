#!/bin/bash
# 11/02 선택(기본 꺼짐, 안 주면 전과 같음): VEHICLE_TYPE=<CARLA 블루프린트> 이면 vehicle_type:=(브리지 차 종류, 기본 vehicle.toyota.prius),
#   VEH_COMPOSE=<오버레이 yaml>(예 integration/config/s73_starex.override.yaml: vehicle_info·지붕 LiDAR 를 덮어 마운트). 밴 센서 파일은 SMAP=s73_sensor_mapping_pps545k_van.yaml
# 10/23 선택(기본 꺼짐): PERCEPTION=clustering(lidar_detection_model:=clustering 은 원래 들어 있음, use_pointcloud_map:=false 그대로)
# 10/23 선택(기본 꺼짐, 안 주면 10/14 와 같음): SPEED_BUMP=1 이면 launch_speed_bump_module:=true(1.9.0 은 이것만으로 안 켜져 s59_run.sh 가 load_speed_bump_plugin.sh 를 부름),
#   MAP_COMPOSE=<오버레이 yaml> MAP_PATH=<컨테이너 안 지도 폴더> 로 다른 지도 폴더를 씀(폴더 이름은 OpenDriveMap 이어야 함)
# 10/14 사본: 센서 파일을 SMAP 환경 변수로 고른다(기본 sensor_mapping_nocam.yaml, 기준 센서 sensor_mapping_ref.yaml)
# I5: 캠퍼스 v3 지도(~/autoware_data/maps/OpenDriveMap)로 Autoware(브리지 포함) 기동.
# 설정은 I4·P1 과 같다: 카메라 뺀 센서 파일, localization:=false + 정답 측위 오버레이, use_pointcloud_map:=false + 진단 그래프 점군 검사 제외,
# 정답 객체(use_empty_dynamic_object_publisher:=true, 클러스터링 끔). 사용: i5_aw_up.sh <tag> <max_real_delta> <spawn x,y,z,roll,pitch,yaw>
TAG=$1; DELTA=$2; SPAWN=$3; SMAP=${SMAP:-sensor_mapping_nocam.yaml}; C=~/campus_mobility_sim; I=$C/integration; P=$C/perception
F=(-f docker-compose.yaml -f $C/docker/planning-simulator.pin.yaml -f $C/docker/carla-integration.override.yaml -f $I/config/i4_logs.override.yaml
   -f $I/config/i3_gt_localization.override.yaml -f $I/config/i4_nopcd_map.override.yaml -f $P/config/gt_objects.override.yaml)
[ -n "$MAP_COMPOSE" ] && F+=(-f "$MAP_COMPOSE")
[ -n "$VEH_COMPOSE" ] && F+=(-f "$VEH_COMPOSE")
ARGS="map_path:=${MAP_PATH:-/home/aw/autoware_data/maps/OpenDriveMap} vehicle_model:=sample_vehicle sensor_model:=carla_sensor_kit simulator_type:=carla rviz:=false lidar_detection_model:=clustering use_traffic_light_recognition:=false cuda_ground_segmentation_node_param_path:=unused occupancy_grid_map_method:=laserscan_based use_light_weight_sensor_mapping:=True sensor_mapping_file:=/home/aw/autoware_data/maps/i1_config/$SMAP max_real_delta_seconds:=$DELTA spawn_point:=$SPAWN localization:=false use_pointcloud_map:=false"
# 10/23: PERCEPTION=clustering 이면 빈 목록 노드 대신 Autoware 검출(클러스터링)·추적·예측을 띄움. 안 주면 전과 같음(정답 객체)
[ "$PERCEPTION" = clustering ] || ARGS="$ARGS use_empty_dynamic_object_publisher:=true"
[ "$SPEED_BUMP" = 1 ] && ARGS="$ARGS launch_speed_bump_module:=true"
[ -n "$VEHICLE_TYPE" ] && ARGS="$ARGS vehicle_type:=$VEHICLE_TYPE"
mkdir -p $I/maps/nopcd/Town01  # i4_nopcd_map 오버레이가 마운트하는 빈 폴더. 없으면 docker 가 root 소유로 만든다
cd ~/autoware/docker/examples/demos/planning-simulator
echo "$(date +%T) docker compose ${F[*]} run --rm -T --name i1_aw planning-simulator bash -c \"ros2 launch autoware_launch e2e_simulator.launch.xml $ARGS\"" > $I/logs/i5_launch_$TAG.log
exec docker compose "${F[@]}" run --rm -T --name i1_aw planning-simulator bash -c "ros2 launch autoware_launch e2e_simulator.launch.xml $ARGS" >> $I/logs/i5_launch_$TAG.log 2>&1
