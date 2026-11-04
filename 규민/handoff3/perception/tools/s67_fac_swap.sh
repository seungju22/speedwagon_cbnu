#!/bin/bash
# 10/23 5단계: 주행 중 클러스터링 입력을 시설 점 지운 점군으로 바꿈(Autoware 소스·런치 파일은 안 고침, 실행 중 컴포넌트만 바꿈)
#   1) s67_facility_crop.py 를 컨테이너에서 띄움(pointcloud_map_filtered -> /s67/fac_crop/pointcloud)
#   2) pointcloud_container 안 clustering/euclidean_cluster 컴포넌트(플러그인 VoxelGridBasedEuclideanClusterNode)를 내리고(unload),
#      같은 플러그인을 같은 이름·이름공간·출력으로, 입력만 /s67/fac_crop/pointcloud 로 다시 올림(load). 설정 값은 autoware_launch 0.52.0 파일 그대로 -p 로 넘김
#   원래 입력은 pointcloud_map_filter 출력(use_pointcloud_map:=false 면 voxel 줄이기만, base_link). 지움 노드가 그것을 받아 시설 점만 빼고 넘기므로 다른 조건은 같음
# 10/25: BLDCROP(기본 1)이면 건물 바닥 사각형(perception/config/s69_building_rects.json, 레이어 v1·v2 건물, 여유 0.3m)도 지움. BLDCROP=0 이면 10/23 과 같음(시설만)
#   CLMIN=<n> 이면 다시 올리는 클러스터 컴포넌트의 min_points_per_cluster 만 n 으로 바꿈
#   10/25 4단계 판정: CLMIN 기본 5(안 주면 5). CLMIN=10 이면 autoware_launch 값과 같음(10/25 3단계까지)
# 10/25: RAIN_RATE=<mm/h>(기본 0 = 끔)이면 지움 노드에 --rain-rate·--rain-seed(RAIN_SEED, 기본 0)를 넘겨 비 열화를 함(s67_facility_crop.py 설명).
#   보통은 s59_run.sh 의 RAIN=1|2 가 정함(비1 9, 비2 25 mm/h). 안 주면 10/25 와 같음
# 사용: s67_fac_swap.sh <tag>   (컨테이너 i1_aw 가 clustering 모드로 떠 있을 때)
BARGS=""; [ "${BLDCROP:-1}" = 1 ] && BARGS="--buildings /home/aw/gt_perception/config/s69_building_rects.json --bpad 0.3"
[ "${RAIN_RATE:-0}" != 0 ] && BARGS="$BARGS --rain-rate $RAIN_RATE --rain-seed ${RAIN_SEED:-0}"
TAG=$1; S='source /opt/autoware/setup.bash;'; L=~/campus_mobility_sim/integration/logs/s67; mkdir -p $L
NS=/perception/object_recognition/detection/clustering
docker exec -u 1000:1000 -d i1_aw bash -c "$S python3 -u /home/aw/gt_perception/tools/s67_facility_crop.py --input /perception/object_recognition/detection/pointcloud_map_filtered/pointcloud --log /home/aw/i4_logs/s67/faccrop_$TAG.csv $BARGS > /home/aw/i4_logs/s67/faccrop_$TAG.log 2>&1 & echo \$! > /home/aw/i4_logs/s67/faccrop_$TAG.pid; wait"
docker exec -u 1000:1000 i1_aw bash -c "$S timeout 20 ros2 component list" > $L/components_before_$TAG.txt 2>&1
CT=$(grep -B200 'clustering/euclidean_cluster$' $L/components_before_$TAG.txt | grep '^/' | tail -1)
ID=$(grep 'clustering/euclidean_cluster$' $L/components_before_$TAG.txt | awk '{print $1}' | head -1)
echo "container $CT id $ID bldcrop ${BLDCROP:-1} clmin ${CLMIN:-5}$([ "${RAIN_RATE:-0}" != 0 ] && echo " rain ${RAIN_RATE} seed ${RAIN_SEED:-0}")" > $L/fac_swap_$TAG.log
[ -z "$CT" ] || [ -z "$ID" ] && { echo "컴포넌트 못 찾음" >> $L/fac_swap_$TAG.log; exit 1; }
P=/opt/autoware/autoware_launch/share/autoware_launch/config/perception/object_recognition/detection/clustering/voxel_grid_based_euclidean_cluster.param.yaml
PARGS=$(docker exec i1_aw python3 -c "
import yaml; d=yaml.safe_load(open('$P'))['/**']['ros__parameters']
d['min_points_per_cluster'] = int('${CLMIN:-5}')
print(' '.join('-p %s:=%s' % (k, ('\"%s\"' % v) if isinstance(v, str) else str(v).lower() if isinstance(v, bool) else v) for k, v in d.items()))")
echo "params $PARGS" >> $L/fac_swap_$TAG.log
docker exec -u 1000:1000 i1_aw bash -c "$S timeout 20 ros2 component unload $CT $ID" >> $L/fac_swap_$TAG.log 2>&1
docker exec -u 1000:1000 i1_aw bash -c "$S timeout 30 ros2 component load $CT autoware_euclidean_cluster autoware::euclidean_cluster::VoxelGridBasedEuclideanClusterNode -n euclidean_cluster -r __ns:=$NS -r input:=/s67/fac_crop/pointcloud -r output:=$NS/clusters -p use_sim_time:=true $PARGS" >> $L/fac_swap_$TAG.log 2>&1
docker exec -u 1000:1000 i1_aw bash -c "$S timeout 20 ros2 component list" > $L/components_after_$TAG.txt 2>&1
docker exec -u 1000:1000 i1_aw bash -c "$S timeout 10 ros2 topic info -v $NS/clusters | grep -E 'Node name|Topic type|count'; timeout 10 ros2 topic info /s67/fac_crop/pointcloud" >> $L/fac_swap_$TAG.log 2>&1
echo "$(date +%T) swap done" >> $L/fac_swap_$TAG.log
