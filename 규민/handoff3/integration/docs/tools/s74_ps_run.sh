#!/bin/bash
# 11/03(s73_ps_run.sh 사본: 지도·회차 목록·차량 오버레이·기록 폴더를 인자로) 11/02 6단계: 플래닝 시뮬레이터(CARLA 없음)로 v3.3a + 스타렉스 vehicle_info(s73_starex.override.yaml) 순환 한 바퀴와 바뀐 쌍
#   임시 HOME(H) 아래 autoware 링크, campus_mobility_sim 링크(오버레이 경로), autoware_data/maps/s73_v33a 사본. 지도는 map_path 로 고름
#   회차 목록 integration/logs/s73/ps_segments.txt(tag, goal, WP, init). s73_ps_goal.sh 로 한 구간씩(도착 = routing state 3)
#   감시 s58_monitor.sh(가용 1GiB 30초·swap 7GiB 면 컨테이너 내림). 끝나면 docker stop
# 사용: H=<임시 HOME> bash s73_ps_run.sh
C=~/campus_mobility_sim; L=${L:?}; T=$C/integration/docs/tools; CT=${CT:-s74_ps}; MAPD=${MAPD:-$C/map/maps/v33b}; SEGS=${SEGS:?}; mkdir -p $L
mkdir -p $H/autoware_data/maps/s74_map; cp $MAPD/lanelet2_map.osm $MAPD/map_projector_info.yaml $H/autoware_data/maps/s74_map/
[ -e $H/autoware ] || ln -s ~/autoware $H/autoware; [ -e $H/campus_mobility_sim ] || ln -s $C $H/campus_mobility_sim
CMD="ros2 launch autoware_launch planning_simulator.launch.xml map_path:=/home/aw/autoware_data/maps/s74_map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit rviz:=false"
cd $H/autoware/docker/examples/demos/planning-simulator
echo "$(date +%T) avail $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)MiB HOME=$H docker compose -f docker-compose.yaml -f $C/docker/planning-simulator.pin.yaml ${VEHOVR:+-f $VEHOVR} run --rm -T --name $CT planning-simulator bash -c '$CMD'" > $L/ps_launch.log
HOME=$H docker compose -f docker-compose.yaml -f $C/docker/planning-simulator.pin.yaml ${VEHOVR:+-f $VEHOVR} run --rm -T --name $CT planning-simulator bash -c "$CMD" >> $L/ps_launch.log 2>&1 &
echo "COMPOSE=$!" > $L/pids.env; echo "RUNNER=$$" >> $L/pids.env
bash $T/s58_monitor.sh $L/monitor.log $CT "" > /dev/null 2>&1 & echo "MON=$!" >> $L/pids.env
X() { docker exec -u 1000:1000 $CT bash -c "source /opt/autoware/setup.bash; $1" 2>&1; }
for i in $(seq 1 60); do sleep 5; X "ros2 service list 2>/dev/null" | grep -q /api/localization/initialize && break; done
sleep 30; echo "ready $(date +%T)" >> $L/events.txt
X "ros2 param get /planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner wheel_base 2>/dev/null; ros2 param get /control/trajectory_follower/controller_node_exe wheel_base 2>/dev/null" >> $L/events.txt
grep -v '^#' $SEGS | while read tag gx gy qz qw wp ix iy iqz iqw; do
  grep -q "STOP" $L/monitor.log 2>/dev/null && { echo "monitor STOP, 멈춤" >> $L/events.txt; break; }
  W=""; [ "$wp" != "-" ] && W=$wp
  if [ "$ix" != "-" ] && [ -n "$iy" ]; then CT=$CT TO=${TO:-600} WP=$W bash $T/s73_ps_goal.sh $tag $gx $gy $qz $qw $ix $iy $iqz $iqw > /dev/null
  else CT=$CT TO=${TO:-600} WP=$W bash $T/s73_ps_goal.sh $tag $gx $gy $qz $qw > /dev/null; fi
  mv $C/integration/logs/ps_goal_$tag.log $L/ 2>/dev/null
  echo "$tag $(grep '^route\|^engage' $L/ps_goal_$tag.log | tr '\n' ' ' | cut -c1-160) | $(tail -1 $L/ps_goal_$tag.log)" >> $L/events.txt
done
docker stop -t 20 $CT > /dev/null 2>&1; . $L/pids.env; kill $MON 2>/dev/null
echo "done $(date +%T)" >> $L/events.txt
