#!/bin/bash
# 10/23 3단계: 주행 중 검출 단계(lidar_detection_model:=clustering, perception_mode lidar, use_pointcloud_map:=false)와 같은 노드를 오프라인으로 돌림
#   입력: 주행 bag 의 /perception/obstacle_segmentation/pointcloud(Autoware 가 지면을 뺀 점군, base_link). 주행 중에도 이 토픽이 클러스터링 입력임
#   흐름(tier4_perception_launch 0.52.0 detection.launch.xml lidar_rule_detector 와 같은 순서):
#     voxel_grid_downsample_filter 는 euclidean_cluster 안 voxel 과 겹쳐 뺌(10/14 와 같음) -> voxel_grid_based_euclidean_cluster
#     -> shape_estimation -> detected_object_feature_remover. 설정은 autoware_launch 0.52.0 것 그대로
#   처리 시간: 1.9.0 판 이 노드들은 processing_time debug 토픽을 안 냄(10/23 확인). 입력 점군도 같이 기록해 같은 stamp 의 받은 시각 차로 셈(s67_cluster_eval.py)
#   남기는 것: 입력 점군, 클러스터(/s60/clusters, s60_cluster_compare.py 가 읽는 이름), 객체(/s67/objects), 처리 시간 debug 토픽, 토픽 목록,
#     docker stats(2초마다 CPU·메모리, 호스트에서)
# 컨테이너(네트워크 없음, NET_ADMIN, lo 만 쓰는 cyclonedds, s60 과 같음). CARLA·Autoware 는 안 띄움
# 사용: bash perception/tools/s67_cluster_offline.sh <주행 bag 경로(integration/logs 기준)> <출력 태그> [배속 0.5]
IN=$1; OUT=$2; RATE=${3:-0.5}; C=~/campus_mobility_sim; L=$C/integration/logs/s67; mkdir -p $L
rm -rf $L/bag_${OUT}_objects   # 이 스크립트가 만드는 결과만 지운다
DDS='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General></Domain></CycloneDDS>'
NAME=s67_off_$OUT
( while true; do s=$(docker stats --no-stream --format '{{.CPUPerc}} {{.MemUsage}}' $NAME 2>/dev/null) && [ -n "$s" ] && echo "$(date +%T.%N | cut -c1-12) $s"; sleep 2; done ) > $L/stats_$OUT.txt &
SP=$!
docker run --rm --name $NAME --network none --cap-add NET_ADMIN -u 0 -v $C/integration/logs:/l -e HOST_UID=$(id -u) -e CYCLONEDDS_URI="$DDS" \
  ghcr.io/autowarefoundation/autoware:universe-humble-1.9.0 bash -c "
set -e; source /opt/ros/humble/setup.bash; source /opt/autoware/setup.bash
P=/opt/autoware/autoware_launch/share/autoware_launch/config/perception/object_recognition/detection/clustering/voxel_grid_based_euclidean_cluster.param.yaml
ros2 launch autoware_euclidean_cluster voxel_grid_based_euclidean_cluster.launch.xml input_pointcloud:=/perception/obstacle_segmentation/pointcloud \
  output_clusters:=/s60/clusters voxel_grid_based_euclidean_param_path:=\$P > /l/s67/cl_launch_$OUT.log 2>&1 &
P1=\$!
ros2 launch autoware_shape_estimation shape_estimation.launch.xml input/objects:=/s60/clusters output/objects:=/s67/objects_with_feature > /l/s67/se_launch_$OUT.log 2>&1 &
P2=\$!
ros2 launch autoware_detected_object_feature_remover detected_object_feature_remover.launch.xml input:=/s67/objects_with_feature output:=/s67/objects > /l/s67/fr_launch_$OUT.log 2>&1 &
P3=\$!; sleep 10
ros2 topic list > /l/s67/topics_$OUT.txt
DBG=\$(grep -E 'processing_time|cyclic_time' /l/s67/topics_$OUT.txt | tr '\n' ' ')
ros2 bag record -o /l/s67/bag_${OUT}_objects /perception/obstacle_segmentation/pointcloud /s60/clusters /s67/objects \$DBG > /l/s67/record_$OUT.log 2>&1 &
RP=\$!; sleep 3
ros2 bag play /l/$IN --topics /perception/obstacle_segmentation/pointcloud /tf_static --rate $RATE > /l/s67/play_$OUT.log 2>&1
sleep 5; kill -INT \$RP; sleep 3; kill -INT \$P1 \$P2 \$P3; sleep 2
chown -R \$HOST_UID:\$HOST_UID /l/s67/bag_${OUT}_objects /l/s67/*_$OUT.*
echo done"
kill $SP
