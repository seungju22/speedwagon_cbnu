#!/bin/bash
# 11/04: s75_run_loop.sh 사본. GOAL_OVR=<x,y,yaw,qz,qw>(지도 좌표) 로 한 목표 회차의 목표를 덮음(스프린터 문 기준 목표),
#   PLATFORMS=<승강장 yaml> 이면 문 구역 장면(ped_tool)이 그 파일의 문 자리를 씀
#   VEHICLE=sprinter|prius(기본 sprinter, 11/04). sprinter 면 VEHICLE_TYPE·VEH_COMPOSE·SMAP·GTARGS·PLATFORMS 와 한 목표 기본(북문, 스프린터 문 기준)을 채움(따로 준 값이 먼저)
#   VEHICLE=prius 면 이 블록을 건너뛰어 s75_run_loop.sh 와 같음(프리우스, sample_vehicle)
# 11/03: s74_run_loop.sh 사본. 바꾼 것(LOOP_SEGS 쓸 때만):
#   (1) 회차 목록 11번째 열 = 그 구간 예상 시간(초, 플래닝 시뮬레이터 값). 구간 제한 시간 = 2배(없으면 SEG_TO, 기본 700)
#   (2) 헤맴 막기: s75_offroad_watch.py 가 바퀴 이탈이 시뮬 10초(OFFSEC) 넘게 이어지면 그 구간 스크립트를 끄고 회차를 끊음(i5_run_<tag>.cut 에 까닭)
#   (4) 매 틱 기록을 s75_track.py(s4_track.py + 조향 명령·앞바퀴 실제 각·z·pitch·roll)로, 제한 2400초
#   (3) STEERBAG=1 이면 조향 bag(logs/s75/steerbag_<tag>: 명령 control_cmd, 실제 steering_status, 자세, 속도) 을 남김(고화질 다시 찍기·조향 비교용)
# 11/03: s59_run.sh 사본. SPAWN_OVR=<x,y,z,roll,pitch,yaw>(CARLA 좌표) 로 출발 자리를 덮고, LOOP_SEGS=<회차 목록>(s74_ps_run 과 같은 꼴: tag goal WP init)
#   을 주면 한 목표(i63_drive.sh) 대신 목록 구간을 차례로 달림(s73_ps_goal.sh, 컨테이너 i1_aw, 도착 = routing state 3). 안 주면 s59_run.sh 와 같음
# 10/23 인지 모드 clustering(기본 car·all 은 그대로): mode 에 clustering 을 주면 Autoware LiDAR 클러스터링(검출·추적·예측)이 객체 토픽을 냄
#   (s59_aw_up.sh 에 PERCEPTION=clustering). gt_objects2 는 차 시점으로 돌되 비교용 /perception/gt/objects 에만 냄. 빈 목록 노드 끄기는 건너뜀
#   clustering 이면 객체 bag(logs/s67/objbag_<tag>: 지면 뺀 점군·클러스터·검출·추적·최종 객체·정답 비교 토픽·자세)을 자동으로 남김. OBJBAG=1 이면 다른 모드도 남김
#   10/23: clustering 이면 정류장 시설 점 지움(perception/tools/s67_fac_swap.sh)을 기본으로 켬. FACCROP=0 이면 10/23 과 같음
#   10/25 4단계 판정: clustering 이면 센서 파일 기본을 sensor_mapping_nocam_pps545k.yaml(54.5만 점/s)로. car·all 은 그대로 30만. SMAP 을 주면 그것
#   10/25 2단계 판정: 모든 모드(car·all·clustering) 센서 파일 기본을 sensor_mapping_nocam_pps545k.yaml 로 통일. 30만은 SMAP=sensor_mapping_nocam.yaml
#   10/25 4단계: RAIN=0|1|2(기본 0 맑음). 1 = 비1 9mm/h, 2 = 비2 25mm/h(근거 integration/docs/s70_report.md 4단계, Goodin et al. 2019)
#     car 모드: 정답 노드 범위를 모델 최대 거리로 줄임(비1 --veh-range 21.8, 비2 --ped-range 16.2 --veh-range 16.2, 콘 12.5 그대로), --miss-prob 0(오프라인에서 범위 안 추가 놓침 0)
#     clustering 모드: 지움 노드에 RAIN_RATE(9|25)·RAIN_SEED(기본 0)를 넘겨 점군 열화(s67_fac_swap.sh). all(다 보기)은 비와 무관해 그대로
#     CARLA 날씨는 바꾸지 않음(0.9.15 LiDAR 속성에 날씨 연결이 없음)
#   TOPLOG=1 이면 컨테이너 안 top(5초)을 logs/s67/top_<tag>.txt 로. STALL=<초> 는 정지 감시 시간(기본 90)
# 10/23: LSTATE 를 주면 상태 파일의 레이어 판(layer_set, place_campus_layers_s66.py)과 액터 수를 로그에 한 줄 남김. 안 주면 전과 같음
# 10/23 선택(기본 꺼짐, 안 주면 10/14 와 같음): SPEED_BUMP=1 이면 경로 넣기 전에 load_speed_bump_plugin.sh(방지턱 모듈, v3.2 지도용),
#   PAIR_GOAL=1 이면 목표를 북문 대신 i62_route.py 의 쌍 목표로(30쌍과 같음). MAP_COMPOSE·MAP_PATH 는 s59_aw_up.sh 로 넘어감
# 10/14 사본: 로그 logs/s59, 센서 파일 SMAP(s59_aw_up.sh), gt_objects2 추가 인자 GTARGS, LiDAR bag 은 기준 센서 기록용으로 그대로
# 10/14 한 회차(i63_run.sh 사본에서 바꿈): 정답 객체를 gt_objects2.py(P1b, --mode)로, 장면 스크립트·추적 카메라·LiDAR bag 을 더함,
# 목표를 노선 파일 2판 북문 뒷축 목표로(pilot_route_r2 -> shuttle_platforms_v31_r1 lib_north base_link_goal). 프로세스는 PID 로 끈다
# 사용: s58_run.sh <tag> <쌍> <mode car|all> <scene ped|door|obst|cone|pcar|none> [cam 0|1] [bag 0|1]
TAG=$1; PAIR=$2; MODE=$3; SCENE=$4; CAM=${5:-0}; BAG=${6:-0}
C=~/campus_mobility_sim; I=$C/integration; T=$I/docs/tools; SC=$I/scenario; P=$C/perception; LS=$I/logs/s59
PY=$C/.venv-carla/bin/python; LOG=$I/logs/i5_run_$TAG.log; : > $LOG; S='source /opt/autoware/setup.bash;'
VEHICLE=${VEHICLE:-sprinter}
if [ "$VEHICLE" = sprinter ]; then
  export VEHICLE_TYPE=${VEHICLE_TYPE:-vehicle.mercedes.sprinter}
  export VEH_COMPOSE=${VEH_COMPOSE:-$C/integration/config/s76_sprinter.override.yaml}
  SMAP=${SMAP:-s74_sensor_mapping_van_pick.yaml}
  GTARGS=${GTARGS:---lidar=-0.229,0.0,2.130}
  PLATFORMS=${PLATFORMS:-$C/map/data/shuttle_platforms_v33d_r1_sprinter.yaml}
  [ -z "$GOAL_OVR" ] && [ "$PAIR_GOAL" != 1 ] && GOAL_OVR=656.016,684.56,-35.52,-0.304989,0.952356
fi
mkdir -p $LS
GTMODE=$MODE; GTTOPIC=""
SMAP0=sensor_mapping_nocam_pps545k.yaml  # 10/25: 30만(sensor_mapping_nocam.yaml)에서 바꿈
case "${RAIN:-0}" in
  1) RAINGT="--veh-range 21.8 --miss-prob 0 --seed ${RAIN_SEED:-0}"; export RAIN_RATE=9 RAIN_SEED=${RAIN_SEED:-0};;
  2) RAINGT="--ped-range 16.2 --veh-range 16.2 --miss-prob 0 --seed ${RAIN_SEED:-0}"; export RAIN_RATE=25 RAIN_SEED=${RAIN_SEED:-0};;
  *) RAINGT=""; unset RAIN_RATE;;
esac
[ "$MODE" = car ] && [ -n "$RAINGT" ] && GTARGS="$GTARGS $RAINGT"
if [ "$MODE" = clustering ]; then SMAP0=sensor_mapping_nocam_pps545k.yaml; GTMODE=car; GTTOPIC="--topic /perception/gt/objects"; export PERCEPTION=clustering; OBJBAG=1; fi
[ "$OBJBAG$TOPLOG" ] && mkdir -p $I/logs/s67
eval "$($PY $T/i62_route.py $PAIR)"
[ -n "$SPAWN_OVR" ] && SPAWN=$SPAWN_OVR
# 북문 뒷축 목표(노선 파일 2판). i62_route.py 의 목표(표지 = 뒷축)보다 1.345m 뒤
GX=656.162; GY=684.456; GYAW=-35.52; QZ=-0.304989; QW=0.952356
[ "$PAIR_GOAL" = 1 ] && eval "$($PY $T/i62_route.py $PAIR)"
[ -n "$GOAL_OVR" ] && IFS=, read GX GY GYAW QZ QW <<< "$GOAL_OVR"
echo "$(date +%T) speed_bump [${SPEED_BUMP:-0}] pair_goal [${PAIR_GOAL:-0}] map [${MAP_PATH:-OpenDriveMap 기본}]" >> $LOG
echo "$(date +%T) vehicle $VEHICLE type [${VEHICLE_TYPE:-기본 프리우스}] goal_ovr [${GOAL_OVR:-}] platforms [${PLATFORMS:-}]" >> $LOG
echo "$(date +%T) smap ${SMAP:-$SMAP0} gtargs [$GTARGS] pair $PAIR mode $MODE scene $SCENE cam $CAM bag $BAG spawn $SPAWN goal $GX $GY $GYAW rain ${RAIN:-0} (pilot_route_r2 lib_north base_link_goal)" >> $LOG
[ -n "$LSTATE" ] && echo "$(date +%T) layer_set $($PY -c "import json,sys;d=json.load(open(sys.argv[1]));print(d.get('layer_set','판 없음(세션67 전 상태 파일)'),len(d['actors']))" $LSTATE) ($LSTATE)" >> $LOG
$PY $T/i5_cleanup.py >> $LOG 2>&1
$PY $T/i5_marker.py place $I/logs/i5_marker_$TAG.json >> $LOG 2>&1
[ -n "$LSTATE" ] && $PY $T/s4_layers.py mark $LSTATE $I/logs/s4_layers_mark_$TAG.json >> $LOG 2>&1
SMAP=${SMAP:-$SMAP0} bash $T/s59_aw_up.sh $TAG 0.05 $SPAWN &
for i in $(seq 1 60); do docker ps --format '{{.Names}}' | grep -q i1_aw && break; sleep 2; done
echo "$(date +%T) container up" >> $LOG
sleep 8
docker exec -u 1000:1000 -d i1_aw bash -c "$S python3 -u /home/aw/i3_tools/i3_gt_localizer.py > /home/aw/i4_logs/i5_gt_$TAG.log 2>&1"
docker exec -u 1000:1000 -d i1_aw bash -c "$S python3 -u /home/aw/gt_perception/gt_objects/gt_objects2.py --mode $GTMODE $GTTOPIC $GTARGS --log /home/aw/gt_logs/s59_obj_$TAG.csv > /home/aw/i4_logs/s59/gtobj_$TAG.log 2>&1"
for i in $(seq 1 90); do grep -q "^pub 1 " $I/logs/i5_gt_$TAG.log 2>/dev/null && break; sleep 3; done
echo "$(date +%T) gt: $(grep -E '^ego |^pub 1 ' $I/logs/i5_gt_$TAG.log | tr '\n' ' ')" >> $LOG
$PY $T/i5_marker.py check $I/logs/i5_marker_$TAG.json >> $LOG 2>&1
[ -n "$LSTATE" ] && $PY $T/s4_layers.py check $LSTATE $I/logs/s4_layers_mark_$TAG.json >> $LOG 2>&1
# empty_objects_publisher 를 PID 로 끈다(컨테이너 안 pgrep 은 대괄호 패턴)
if [ "$MODE" != clustering ]; then
for i in $(seq 1 30); do EP=$(docker exec i1_aw pgrep -f '[e]mpty_objects_publisher' | head -1); [ -n "$EP" ] && break; sleep 2; done
docker exec i1_aw kill $EP; sleep 2
echo "$(date +%T) empty_objects_publisher pid $EP killed, left: $(docker exec i1_aw pgrep -fc '[e]mpty_objects_publisher')" >> $LOG
else
echo "$(date +%T) clustering: empty_objects_publisher $(docker exec i1_aw pgrep -fc '[e]mpty_objects_publisher') (0 이어야 함), euclidean $(docker exec i1_aw bash -c "$S ros2 node list 2>/dev/null" | grep -c euclidean)" >> $LOG
# 10/23: clustering 은 정류장 시설 점 지움을 기본으로 켬(FACCROP=0 이면 끔, 비교용). 실행 중 클러스터 컴포넌트 입력을 바꿔 끼움
if [ "${FACCROP:-1}" = 1 ]; then
  bash $P/tools/s67_fac_swap.sh $TAG > /dev/null 2>&1; echo "$(date +%T) faccrop on: $(grep -E '^Loaded|^Unloaded|못 찾음' $I/logs/s67/fac_swap_$TAG.log | tr '\n' ' ')" >> $LOG
else
  echo "$(date +%T) faccrop off (FACCROP=0)" >> $LOG
fi
fi
if [ "$SPEED_BUMP" = 1 ]; then
  bash $T/load_speed_bump_plugin.sh i1_aw > $LS/speed_bump_$TAG.log 2>&1; echo "$(date +%T) $(tail -1 $LS/speed_bump_$TAG.log)" >> $LOG
fi
$PY $T/s75_track.py $I/logs/i5_track_$TAG.csv 2400 > $I/logs/i5_track_$TAG.log 2>&1 & TR=$!
SCN=""
case $SCENE in
  ped)  $PY $SC/dart_out.py $LS/scn_$TAG.csv --road 1415 --s 45 --trigger 20 > $LS/scn_$TAG.log 2>&1 & SCN=$! ;;
  door) $PY $SC/ped_tool.py ${PLATFORMS:+--platforms $PLATFORMS} $SC/door_lib_north.yaml --log $LS/scn_$TAG.csv --timeout 1500 > $LS/scn_$TAG.log 2>&1 & SCN=$! ;;
  obst) $PY $SC/obstacles.py $LS/scn_$TAG.csv --road 1415 --car-s 55 --cone-s 30 > $LS/scn_$TAG.log 2>&1 & SCN=$! ;;
  cone) $PY $SC/obstacles.py $LS/scn_$TAG.csv --road 1415 --car-s 55 --cone-s 30 --no-car > $LS/scn_$TAG.log 2>&1 & SCN=$! ;;
  pcar) $PY $SC/obstacles.py $LS/scn_$TAG.csv --road 1415 --car-s 55 --cone-s 30 --no-cone > $LS/scn_$TAG.log 2>&1 & SCN=$! ;;
esac
CP=""; [ "$CAM" = 1 ] && { $PY $SC/chase_cam.py $LS/cam_$TAG > $LS/cam_$TAG.log 2>&1 & CP=$!; }
if [ "$BAG" = 1 ]; then
  docker exec -u 1000:1000 -d i1_aw bash -c "$S ros2 bag record -o /home/aw/i4_logs/s59/bag_$TAG /sensing/lidar/top/pointcloud_before_sync /perception/obstacle_segmentation/pointcloud /tf_static /localization/kinematic_state > /home/aw/i4_logs/s59/bag_$TAG.log 2>&1 & echo \$! > /home/aw/i4_logs/s59/bag_$TAG.pid; wait"
fi
if [ "$OBJBAG" = 1 ]; then
  OB=/perception/object_recognition/detection
  docker exec -u 1000:1000 -d i1_aw bash -c "$S ros2 bag record -o /home/aw/i4_logs/s67/objbag_$TAG /perception/obstacle_segmentation/pointcloud $OB/clustering/clusters $OB/clustering/objects $OB/objects /perception/object_recognition/tracking/objects /perception/object_recognition/objects /perception/gt/objects /localization/kinematic_state > /home/aw/i4_logs/s67/objbag_$TAG.log 2>&1 & echo \$! > /home/aw/i4_logs/s67/objbag_$TAG.pid; wait"
fi
if [ "$TOPLOG" = 1 ]; then
  docker exec -u 1000:1000 -d i1_aw bash -c "top -b -c -d 5 -w 250 > /home/aw/i4_logs/s67/top_$TAG.txt 2>&1 & echo \$! > /home/aw/i4_logs/s67/top_$TAG.pid; wait"
fi
if [ "$STEERBAG" = 1 ]; then
  mkdir -p $I/logs/s75
  docker exec -u 1000:1000 -d i1_aw bash -c "$S ros2 bag record -o /home/aw/i4_logs/s75/steerbag_$TAG /control/command/control_cmd /vehicle/status/steering_status /vehicle/status/velocity_status /localization/kinematic_state > /home/aw/i4_logs/s75/steerbag_$TAG.log 2>&1 & echo \$! > /home/aw/i4_logs/s75/steerbag_$TAG.pid; wait"
fi
echo "$(date +%T) track $TR scene $SCN cam $CP" >> $LOG
[ -n "$OBJBAG$TOPLOG$STALL" ] && echo "$(date +%T) objbag ${OBJBAG:-0} toplog ${TOPLOG:-0} stall ${STALL:-90}" >> $LOG
if [ -n "$LOOP_SEGS" ]; then
  CUT=$I/logs/i5_run_$TAG.cut; rm -f $CUT
  grep -v '^#' $LOOP_SEGS | while read tag gx gy qz qw wp ix iy iqz iqw exp; do
    W=""; [ "$wp" != "-" ] && W=$wp
    TOS=${SEG_TO:-700}; [ -n "$exp" ] && TOS=$(( exp * 2 ))
    echo "$(date +%T) seg $tag start to ${TOS}s (expect ${exp:--})" >> $LOG
    CT=i1_aw TO=$TOS WP=$W bash $T/s73_ps_goal.sh ${TAG}_$tag $gx $gy $qz $qw > /dev/null & GP=$!
    $PY $T/s75_offroad_watch.py $I/logs/i5_track_$TAG.csv $CUT --pid $GP --sec ${OFFSEC:-10} >> $LOG 2>&1 & OW=$!
    wait $GP; kill $OW 2>/dev/null
    if [ -s $CUT ]; then echo "$(date +%T) seg $tag 끊음: $(cat $CUT)" >> $LOG; break; fi
    echo "$(date +%T) seg $tag $(tail -1 $I/logs/ps_goal_${TAG}_$tag.log)" >> $LOG
    grep -q "state 3" <(tail -1 $I/logs/ps_goal_${TAG}_$tag.log) || { echo "$(date +%T) seg $tag 도착 못 함(2배 넘김 등), 멈춤" >> $LOG; echo "timeout $tag" > $CUT; break; }
  done
else
bash $T/i6_stall.sh $TAG $GX $GY ${STALL:-90} & ST=$!
GX=$GX GY=$GY GYAW=$GYAW QZ=$QZ QW=$QW PAIR=$PAIR bash $T/i63_drive.sh $TAG 1500; kill $ST 2>/dev/null
fi
echo "$(date +%T) drive done" >> $LOG
if [ "$SCENE" = door ] && [ -n "$SCN" ]; then   # 정차 뒤 승하차 단계가 끝날 때까지(최대 180s)
  for i in $(seq 1 90); do kill -0 $SCN 2>/dev/null || break; sleep 2; done; echo "$(date +%T) door scene wait ${i}x2s" >> $LOG
fi
[ -n "$SCN" ] && kill -INT $SCN 2>/dev/null; sleep 2
[ -n "$CP" ] && kill -INT $CP 2>/dev/null
if [ "$BAG" = 1 ]; then BP=$(cat $I/logs/s59/bag_$TAG.pid); docker exec i1_aw kill -INT $BP; sleep 5; echo "$(date +%T) bag pid $BP stopped" >> $LOG; fi
if [ "$OBJBAG" = 1 ]; then OP=$(cat $I/logs/s67/objbag_$TAG.pid); docker exec i1_aw kill -INT $OP; sleep 5; echo "$(date +%T) objbag pid $OP stopped" >> $LOG; fi
[ "$TOPLOG" = 1 ] && docker exec i1_aw kill $(cat $I/logs/s67/top_$TAG.pid)
if [ "$STEERBAG" = 1 ]; then SB=$(cat $I/logs/s75/steerbag_$TAG.pid); docker exec i1_aw kill -INT $SB; sleep 5; echo "$(date +%T) steerbag pid $SB stopped" >> $LOG; fi
kill -INT $TR; sleep 3
BR=$(docker exec i1_aw pgrep -f "[a]utoware_carla_interface/lib/autoware_carla_interface/autoware_carla_interface" | head -1)
docker exec i1_aw kill -INT $BR   # 브리지 정상 종료(자차·센서 스스로 지움)
for i in $(seq 1 15); do docker exec i1_aw kill -0 $BR 2>/dev/null || break; sleep 1; done; echo "$(date +%T) bridge pid $BR sigint wait ${i}s" >> $LOG
docker stop -t 20 i1_aw >> $LOG 2>&1; sleep 2
[ -n "$CP" ] && wait $CP 2>/dev/null
$PY $T/i5_cleanup.py >> $LOG 2>&1
echo "$(date +%T) end" >> $LOG
