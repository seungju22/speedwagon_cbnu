#!/bin/bash
# 세션59 사본: 로그 logs/s59, 센서 파일 SMAP(s59_aw_up.sh), gt_objects2 추가 인자 GTARGS, LiDAR bag 은 기준 센서 기록용으로 그대로
# 세션58 한 회차(i63_run.sh 사본에서 바꿈): 정답 객체를 gt_objects2.py(P1b, --mode)로, 장면 스크립트·추적 카메라·LiDAR bag 을 더함,
# 목표를 노선 파일 2판 북문 뒷축 목표로(pilot_route_r2 -> shuttle_platforms_v31_r1 lib_north base_link_goal). 프로세스는 PID 로 끈다
# 사용: s58_run.sh <tag> <쌍> <mode car|all> <scene ped|door|obst|cone|pcar|none> [cam 0|1] [bag 0|1]
TAG=$1; PAIR=$2; MODE=$3; SCENE=$4; CAM=${5:-0}; BAG=${6:-0}
C=~/campus_mobility_sim; I=$C/integration; T=$I/docs/tools; SC=$I/scenario; P=$C/perception; LS=$I/logs/s59
PY=$C/.venv-carla/bin/python; LOG=$I/logs/i5_run_$TAG.log; : > $LOG; S='source /opt/autoware/setup.bash;'
mkdir -p $LS
eval "$($PY $T/i62_route.py $PAIR)"
# 북문 뒷축 목표(노선 파일 2판). i62_route.py 의 목표(표지 = 뒷축)보다 1.345m 뒤
GX=656.162; GY=684.456; GYAW=-35.52; QZ=-0.304989; QW=0.952356
echo "$(date +%T) smap ${SMAP:-sensor_mapping_nocam.yaml} gtargs [$GTARGS] pair $PAIR mode $MODE scene $SCENE cam $CAM bag $BAG spawn $SPAWN goal $GX $GY $GYAW (pilot_route_r2 lib_north base_link_goal)" >> $LOG
$PY $T/i5_cleanup.py >> $LOG 2>&1
$PY $T/i5_marker.py place $I/logs/i5_marker_$TAG.json >> $LOG 2>&1
[ -n "$LSTATE" ] && $PY $T/s4_layers.py mark $LSTATE $I/logs/s4_layers_mark_$TAG.json >> $LOG 2>&1
SMAP=${SMAP:-sensor_mapping_nocam.yaml} bash $T/s59_aw_up.sh $TAG 0.05 $SPAWN &
for i in $(seq 1 60); do docker ps --format '{{.Names}}' | grep -q i1_aw && break; sleep 2; done
echo "$(date +%T) container up" >> $LOG
sleep 8
docker exec -u 1000:1000 -d i1_aw bash -c "$S python3 -u /home/aw/i3_tools/i3_gt_localizer.py > /home/aw/i4_logs/i5_gt_$TAG.log 2>&1"
docker exec -u 1000:1000 -d i1_aw bash -c "$S python3 -u /home/aw/gt_perception/gt_objects/gt_objects2.py --mode $MODE $GTARGS --log /home/aw/gt_logs/s59_obj_$TAG.csv > /home/aw/i4_logs/s59/gtobj_$TAG.log 2>&1"
for i in $(seq 1 90); do grep -q "^pub 1 " $I/logs/i5_gt_$TAG.log 2>/dev/null && break; sleep 3; done
echo "$(date +%T) gt: $(grep -E '^ego |^pub 1 ' $I/logs/i5_gt_$TAG.log | tr '\n' ' ')" >> $LOG
$PY $T/i5_marker.py check $I/logs/i5_marker_$TAG.json >> $LOG 2>&1
[ -n "$LSTATE" ] && $PY $T/s4_layers.py check $LSTATE $I/logs/s4_layers_mark_$TAG.json >> $LOG 2>&1
# empty_objects_publisher 를 PID 로 끈다(컨테이너 안 pgrep 은 대괄호 패턴)
for i in $(seq 1 30); do EP=$(docker exec i1_aw pgrep -f '[e]mpty_objects_publisher' | head -1); [ -n "$EP" ] && break; sleep 2; done
docker exec i1_aw kill $EP; sleep 2
echo "$(date +%T) empty_objects_publisher pid $EP killed, left: $(docker exec i1_aw pgrep -fc '[e]mpty_objects_publisher')" >> $LOG
$PY $T/s4_track.py $I/logs/i5_track_$TAG.csv 1800 > $I/logs/i5_track_$TAG.log 2>&1 & TR=$!
SCN=""
case $SCENE in
  ped)  $PY $SC/dart_out.py $LS/scn_$TAG.csv --road 1415 --s 45 --trigger 20 > $LS/scn_$TAG.log 2>&1 & SCN=$! ;;
  door) $PY $SC/ped_tool.py $SC/door_lib_north.yaml --log $LS/scn_$TAG.csv --timeout 1500 > $LS/scn_$TAG.log 2>&1 & SCN=$! ;;
  obst) $PY $SC/obstacles.py $LS/scn_$TAG.csv --road 1415 --car-s 55 --cone-s 30 > $LS/scn_$TAG.log 2>&1 & SCN=$! ;;
  cone) $PY $SC/obstacles.py $LS/scn_$TAG.csv --road 1415 --car-s 55 --cone-s 30 --no-car > $LS/scn_$TAG.log 2>&1 & SCN=$! ;;
  pcar) $PY $SC/obstacles.py $LS/scn_$TAG.csv --road 1415 --car-s 55 --cone-s 30 --no-cone > $LS/scn_$TAG.log 2>&1 & SCN=$! ;;
esac
CP=""; [ "$CAM" = 1 ] && { $PY $SC/chase_cam.py $LS/cam_$TAG > $LS/cam_$TAG.log 2>&1 & CP=$!; }
if [ "$BAG" = 1 ]; then
  docker exec -u 1000:1000 -d i1_aw bash -c "$S ros2 bag record -o /home/aw/i4_logs/s59/bag_$TAG /sensing/lidar/top/pointcloud_before_sync /perception/obstacle_segmentation/pointcloud /tf_static /localization/kinematic_state > /home/aw/i4_logs/s59/bag_$TAG.log 2>&1 & echo \$! > /home/aw/i4_logs/s59/bag_$TAG.pid; wait"
fi
echo "$(date +%T) track $TR scene $SCN cam $CP" >> $LOG
bash $T/i6_stall.sh $TAG $GX $GY 90 & ST=$!
GX=$GX GY=$GY GYAW=$GYAW QZ=$QZ QW=$QW PAIR=$PAIR bash $T/i63_drive.sh $TAG 1500; kill $ST 2>/dev/null
echo "$(date +%T) drive done" >> $LOG
if [ "$SCENE" = door ] && [ -n "$SCN" ]; then   # 정차 뒤 승하차 단계가 끝날 때까지(최대 180s)
  for i in $(seq 1 90); do kill -0 $SCN 2>/dev/null || break; sleep 2; done; echo "$(date +%T) door scene wait ${i}x2s" >> $LOG
fi
[ -n "$SCN" ] && kill -INT $SCN 2>/dev/null; sleep 2
[ -n "$CP" ] && kill -INT $CP 2>/dev/null
if [ "$BAG" = 1 ]; then BP=$(cat $I/logs/s59/bag_$TAG.pid); docker exec i1_aw kill -INT $BP; sleep 5; echo "$(date +%T) bag pid $BP stopped" >> $LOG; fi
kill -INT $TR; sleep 3
BR=$(docker exec i1_aw pgrep -f "[a]utoware_carla_interface/lib/autoware_carla_interface/autoware_carla_interface" | head -1)
docker exec i1_aw kill -INT $BR   # 브리지 정상 종료(자차·센서 스스로 지움)
for i in $(seq 1 15); do docker exec i1_aw kill -0 $BR 2>/dev/null || break; sleep 1; done; echo "$(date +%T) bridge pid $BR sigint wait ${i}s" >> $LOG
docker stop -t 20 i1_aw >> $LOG 2>&1; sleep 2
[ -n "$CP" ] && wait $CP 2>/dev/null
$PY $T/i5_cleanup.py >> $LOG 2>&1
echo "$(date +%T) end" >> $LOG
