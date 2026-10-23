#!/bin/bash
# 세션62: 플래닝 시뮬레이터(컨테이너 s62_ps)에서 AD API 로 한 구간 달리기. 경로 설정 -> 자율 -> 도착(routing state 3) 까지
# 초기 위치를 주면 먼저 /api/localization/initialize 로 차를 그 자리에 둔다(처음 한 번만 준다)
# 사용: s62_ps_goal.sh <tag> <gx> <gy> <qz> <qw> [ix iy iqz iqw]   값은 노선 파일 2판(pilot_route_r2.yaml) goal
TAG=$1; GX=$2; GY=$3; QZ=$4; QW=$5; CT=${CT:-s62_ps}; TO=${TO:-300}
OUT=~/campus_mobility_sim/integration/logs/ps_goal_$TAG.log; : > $OUT
X() { docker exec -u 1000:1000 $CT bash -c "source /opt/autoware/setup.bash; $1" 2>&1; }
F() { X "timeout 8 ros2 topic echo --once $1 --field $2" | head -1; }
if [ -n "$6" ]; then
  X "timeout 20 ros2 service call /api/localization/initialize autoware_adapi_v1_msgs/srv/InitializeLocalization \"{pose: [{header: {frame_id: map}, pose: {pose: {position: {x: $6, y: $7, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $8, w: $9}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0.01,0,0,0, 0,0,0,0.01,0,0, 0,0,0,0,0.01,0, 0,0,0,0,0,0.07]}}]}\"" \
    | grep -o "success=[A-Za-z]*.*message='[^']*'" | sed "s/^/init $(date +%T) /" >> $OUT
  sleep 3
fi
rs=$(F /api/routing/state state); echo "route_state_before $rs $(date +%T)" >> $OUT
if [ "$rs" = "3" ]; then   # 도착(ARRIVED) 상태에서는 set_route 가 'The route is already set.' 으로 거절된다 -> 먼저 지운다
  X "timeout 20 ros2 service call /api/routing/clear_route autoware_adapi_v1_msgs/srv/ClearRoute {}" \
    | grep -o "success=[A-Za-z]*.*message='[^']*'" | sed "s/^/clear $(date +%T) /" >> $OUT
  sleep 2
fi
X "timeout 20 ros2 service call /api/routing/set_route_points autoware_adapi_v1_msgs/srv/SetRoutePoints \"{header: {frame_id: map}, option: {allow_goal_modification: false}, goal: {position: {x: $GX, y: $GY, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}, waypoints: []}\"" \
  | grep -o "success=[A-Za-z]*.*message='[^']*'" | sed "s/^/route $(date +%T) /" >> $OUT
for i in $(seq 1 40); do   # 이미 자율(mode 2)이면 기다리지 않는다
  a=$(F /api/operation_mode/state is_autonomous_mode_available); m=$(F /api/operation_mode/state mode); [ "$a" = "True" ] || [ "$m" = "2" ] && break; sleep 3
done
echo "auto_available $a mode $(F /api/operation_mode/state mode) $(date +%T)" >> $OUT
X "timeout 20 ros2 service call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMode {}" \
  | grep -o "success=[A-Za-z]*.*message='[^']*'" | sed "s/^/engage $(date +%T) /" >> $OUT
t0=$(date +%s)
while :; do   # RouteState: 1 UNSET, 2 SET, 3 ARRIVED, 4 CHANGING
  s=$(F /api/routing/state state); [ "$s" = "3" ] && break
  [ $(( $(date +%s) - t0 )) -gt $TO ] && { echo "timeout state=$s" >> $OUT; break; }
  sleep 5
done
p=$(X "timeout 8 ros2 topic echo --once /localization/kinematic_state --field pose.pose.position" | tr '\n' ' ')
echo "end $(date +%T) elapsed_s $(( $(date +%s) - t0 )) state $s pose $p" >> $OUT
cat $OUT
