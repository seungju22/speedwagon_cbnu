#!/bin/bash
# 10/23 3단계: 자차를 세운 채(경로 없음, 자율 전환 없음) Autoware(clustering + 시설 점 지움)·정답 측위·정답 객체를 띄우고 장면 하나를 돌린다
#   s59_run.sh 의 기동·정리 순서를 따름(i5_cleanup -> s59_aw_up.sh -> 정답 측위 -> 정답 객체 -> 시설 지움 -> 장면 -> 브리지 SIGINT -> 컨테이너 내림 -> i5_cleanup)
#   장면 lat : s68_spawn_lat.py(소환 지연). 정답 노드 둘: 차 시점 -> /perception/gt/objects, 다 보기 -> /perception/gt/objects_all. bag 없음
#   장면 dist: s68_dist_scene.py(거리별 배치). 정답 노드 다 보기 하나 -> /perception/gt/objects. bag(브리지 LiDAR·지면 뺀 점군·클러스터·객체·정답·자세·처리 시간)
#   자차 자리: 기본 r1194 lane -1 s=20(CARLA 727.02, -868.10, yaw -169.23, 앞 45m 곧음). SPAWN 으로 바꿀 수 있음
# 10/25 4단계 판정: 센서 파일 기본 sensor_mapping_nocam_pps545k.yaml(54.5만). 10/23 과 같게 하려면 SMAP=sensor_mapping_nocam.yaml
# 사용: LSTATE=<레이어 상태 파일> bash s68_static_run.sh <tag> <lat|dist> [장면 추가 인자...]
TAG=$1; SCENE=$2; shift 2
C=~/campus_mobility_sim; I=$C/integration; T=$I/docs/tools; P=$C/perception; L=$I/logs/s68; mkdir -p $L
PY=$C/.venv-carla/bin/python; LOG=$L/run_$TAG.log; : > $LOG; S='source /opt/autoware/setup.bash;'
SPAWN=${SPAWN:-727.02,-868.10,0.30,0.0,0.0,-169.23}
echo "$(date +%T) tag $TAG scene $SCENE spawn $SPAWN smap ${SMAP:-sensor_mapping_nocam_pps545k.yaml} faccrop ${FACCROP:-1} args [$*]" >> $LOG
[ -n "$LSTATE" ] && echo "$(date +%T) layer_set $($PY -c "import json,sys;d=json.load(open(sys.argv[1]));print(d.get('layer_set','판 없음'),len(d['actors']))" $LSTATE) ($LSTATE)" >> $LOG
$PY $T/i5_cleanup.py >> $LOG 2>&1
[ -n "$LSTATE" ] && $PY $T/s4_layers.py mark $LSTATE $L/layers_mark_$TAG.json >> $LOG 2>&1
PERCEPTION=clustering SMAP=${SMAP:-sensor_mapping_nocam_pps545k.yaml} bash $T/s59_aw_up.sh $TAG 0.05 $SPAWN &
for i in $(seq 1 60); do docker ps --format '{{.Names}}' | grep -q i1_aw && break; sleep 2; done
echo "$(date +%T) container up" >> $LOG
sleep 8
docker exec -u 1000:1000 -d i1_aw bash -c "$S python3 -u /home/aw/i3_tools/i3_gt_localizer.py > /home/aw/i4_logs/i5_gt_$TAG.log 2>&1"
GT=/home/aw/gt_perception/gt_objects/gt_objects2.py
if [ "$SCENE" = lat ]; then
  docker exec -u 1000:1000 -d i1_aw bash -c "$S python3 -u $GT --mode car --topic /perception/gt/objects --log /home/aw/gt_logs/s68_obj_${TAG}_car.csv > /home/aw/i4_logs/s68/gtobj_${TAG}_car.log 2>&1"
  docker exec -u 1000:1000 -d i1_aw bash -c "$S python3 -u $GT --mode all --topic /perception/gt/objects_all --log /home/aw/gt_logs/s68_obj_${TAG}_all.csv > /home/aw/i4_logs/s68/gtobj_${TAG}_all.log 2>&1"
else
  docker exec -u 1000:1000 -d i1_aw bash -c "$S python3 -u $GT --mode all --topic /perception/gt/objects --log /home/aw/gt_logs/s68_obj_${TAG}_all.csv > /home/aw/i4_logs/s68/gtobj_${TAG}_all.log 2>&1"
fi
for i in $(seq 1 90); do grep -q "^pub 1 " $I/logs/i5_gt_$TAG.log 2>/dev/null && break; sleep 3; done
echo "$(date +%T) gt: $(grep -E '^ego |^pub 1 ' $I/logs/i5_gt_$TAG.log | tr '\n' ' ')" >> $LOG
[ -n "$LSTATE" ] && $PY $T/s4_layers.py check $LSTATE $L/layers_mark_$TAG.json >> $LOG 2>&1
echo "$(date +%T) clustering: empty_objects_publisher $(docker exec i1_aw pgrep -fc '[e]mpty_objects_publisher') (0 이어야 함)" >> $LOG
if [ "${FACCROP:-1}" = 1 ]; then
  bash $P/tools/s67_fac_swap.sh $TAG > /dev/null 2>&1; echo "$(date +%T) faccrop on: $(grep -E '^Loaded|^Unloaded|못 찾음' $I/logs/s67/fac_swap_$TAG.log | tr '\n' ' ')" >> $LOG
fi
for i in $(seq 1 30); do grep -q '^pub' $L/gtobj_${TAG}_all.log 2>/dev/null && break; sleep 2; done
if [ "$SCENE" = dist ]; then
  OB=/perception/object_recognition/detection; DB=$OB/clustering/voxel_grid_based_euclidean_cluster/debug
  docker exec -u 1000:1000 -d i1_aw bash -c "$S ros2 bag record -o /home/aw/i4_logs/s68/bag_$TAG /sensing/lidar/top/pointcloud_before_sync /perception/obstacle_segmentation/pointcloud $OB/clustering/clusters $OB/clustering/objects /perception/object_recognition/tracking/objects /perception/object_recognition/objects /perception/gt/objects /localization/kinematic_state /tf_static $DB/processing_time_ms > /home/aw/i4_logs/s68/bag_$TAG.log 2>&1 & echo \$! > /home/aw/i4_logs/s68/bag_$TAG.pid; wait"
  sleep 5
  echo "$(date +%T) scene dist start" >> $LOG
  timeout 900 $PY $P/tools/s68_dist_scene.py $L/scn_$TAG.csv "$@" > $L/scn_$TAG.log 2>&1
  BP=$(cat $L/bag_$TAG.pid); docker exec i1_aw kill -INT $BP; sleep 5; echo "$(date +%T) bag pid $BP stopped" >> $LOG
else
  sleep 5
  echo "$(date +%T) scene lat start" >> $LOG
  timeout 600 $PY $P/tools/s68_spawn_lat.py $L/scn_$TAG.csv "$@" > $L/scn_$TAG.log 2>&1
fi
echo "$(date +%T) scene end: $(tail -1 $L/scn_$TAG.log)" >> $LOG
docker exec i1_aw bash -c "grep '^stat' /home/aw/i4_logs/s68/gtobj_${TAG}_*.log | tail -2" >> $LOG 2>&1
BR=$(docker exec i1_aw pgrep -f "[a]utoware_carla_interface/lib/autoware_carla_interface/autoware_carla_interface" | head -1)
docker exec i1_aw kill -INT $BR
for i in $(seq 1 15); do docker exec i1_aw kill -0 $BR 2>/dev/null || break; sleep 1; done; echo "$(date +%T) bridge pid $BR sigint wait ${i}s" >> $LOG
docker stop -t 20 i1_aw >> $LOG 2>&1; sleep 2
$PY $T/i5_cleanup.py >> $LOG 2>&1
echo "$(date +%T) end" >> $LOG
