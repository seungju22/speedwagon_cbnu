#!/bin/bash
# 10/25 1·2단계: 기록된 bag 의 지면 뺀 점군으로 클러스터 파라미터 쓸기(s69_sweep.launch.py). CARLA·Autoware 는 안 띄움
#   컨테이너(네트워크 없음, lo 만 쓰는 cyclonedds, s67_cluster_offline.sh 와 같음)에서 런치를 띄우고 bag 을 --rate 로 틀어 결과 bag 을 남김
#   틀어 주는 것: /perception/obstacle_segmentation/pointcloud, /localization/kinematic_state(점 지움 노드가 자차 자세로 씀)
#   남기는 것: /s69/<조합>/objects, 조합마다 클러스터 debug/processing_time_ms, 점 지움 로그 csv, 런치 로그
# 사용: bash s69_sweep_offline.sh <bag(integration/logs 기준)> <출력 태그> [배속 0.5] [mins 3,5,7,10] [leafs 0.3,0.2] [crops f,fb]
IN=$1; OUT=$2; RATE=${3:-0.5}; MINS=${4:-3,5,7,10}; LEAFS=${5:-0.3,0.2}; CROPS=${6:-f,fb}
C=~/campus_mobility_sim; L=$C/integration/logs/s69/sweep; mkdir -p $L
rm -rf $L/bag_$OUT   # 이 스크립트가 만드는 결과만 지운다
DDS='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General></Domain></CycloneDDS>'
NAME=s69_sw_$OUT
( while true; do s=$(docker stats --no-stream --format '{{.CPUPerc}} {{.MemUsage}}' $NAME 2>/dev/null) && [ -n "$s" ] && echo "$(date +%T) $s"; sleep 2; done ) > $L/stats_$OUT.txt &
SP=$!
docker run --rm --name $NAME --network none --cap-add NET_ADMIN -u 0 -v $C/integration/logs:/l -v $C/perception:/p:ro -e HOST_UID=$(id -u) -e CYCLONEDDS_URI="$DDS" \
  ghcr.io/autowarefoundation/autoware:universe-humble-1.9.0 bash -c "
source /opt/ros/humble/setup.bash; source /opt/autoware/setup.bash
ros2 launch /p/tools/s69_sweep.launch.py mins:=$MINS leafs:=$LEAFS crops:=$CROPS log:=/l/s69/sweep/$OUT > /l/s69/sweep/launch_$OUT.log 2>&1 &
LP=\$!; sleep 15
ros2 topic list > /l/s69/sweep/topics_$OUT.txt
ros2 bag record -o /l/s69/sweep/bag_$OUT -e '/s69/.*/(objects|voxel_grid_based_euclidean_cluster/debug/processing_time_ms)' > /l/s69/sweep/record_$OUT.log 2>&1 &
RP=\$!; sleep 5
ros2 bag play /l/$IN --topics /perception/obstacle_segmentation/pointcloud /localization/kinematic_state --rate $RATE > /l/s69/sweep/play_$OUT.log 2>&1
sleep 5; kill -INT \$RP; sleep 4; kill -INT \$LP; sleep 4
chown -R \$HOST_UID:\$HOST_UID /l/s69/sweep/bag_$OUT /l/s69/sweep/*_$OUT*
echo done"
kill $SP
