#!/bin/bash
# I6-3(i6_drive.sh 사본, 2026-10-13): 출발 전 Autoware 경로를 xodr 노선과 비교(i63_route_get·i63_routechk), 다르면 출발 안 함
# I6(i5_drive.sh 사본, 2026-10-11): CARLA 녹화(i1_recorder start/stop) 뺌. 위치 기록은 s4_track.py 가 매 틱 남긴다
TAG=$1; TO=${2:-900}; D=~/campus_mobility_sim/integration; PY=~/campus_mobility_sim/.venv-carla/bin/python
OUT=$D/logs/i5_drive_$TAG.log; : > $OUT
SRC='source /opt/autoware/setup.bash;'
docker exec -u 1000:1000 -d i1_aw bash -c "$SRC python3 /home/aw/i3_tools/i2_watch.py /home/aw/i4_logs/i5_watch_$TAG.csv $GX $GY $GYAW $TO > /home/aw/i4_logs/i5_watch_$TAG.err 2>&1"
docker exec -u 1000:1000 -d i1_aw bash -c "$SRC python3 /home/aw/i3_tools/i4_compare.py /home/aw/i4_logs/i5_compare_$TAG.csv $TO > /home/aw/i4_logs/i5_compare_$TAG.err 2>&1"
docker exec -u 1000:1000 -d i1_aw bash -c "$SRC timeout $TO ros2 topic hz /control/command/control_cmd > /home/aw/i4_logs/i5_hz_$TAG.log 2>&1"
sleep 3
docker exec -u 1000:1000 i1_aw bash -c "$SRC timeout 20 ros2 service call /api/routing/set_route_points autoware_adapi_v1_msgs/srv/SetRoutePoints \"{header: {frame_id: map}, option: {allow_goal_modification: false}, goal: {position: {x: $GX, y: $GY, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}, waypoints: []}\"" 2>&1 | grep -o "success=[A-Za-z]*.*message='[^']*'" | sed 's/^/route /' >> $OUT
for i in $(seq 1 60); do
  a=$(docker exec -u 1000:1000 i1_aw bash -c "$SRC timeout 5 ros2 topic echo --once /api/operation_mode/state --field is_autonomous_mode_available" 2>/dev/null | head -1)
  [ "$a" = "True" ] || [ "$a" = "true" ] && break; sleep 3
done
echo "auto_available $a $(date +%T)" >> $OUT
docker exec -u 1000:1000 i1_aw bash -c "$SRC python3 /home/aw/i3_tools/i63_route_get.py /home/aw/i4_logs/i63_route_$TAG.txt 30" >> $OUT 2>&1
$PY $D/docs/tools/i63_routechk.py $D/logs/i63_route_$TAG.txt $PAIR > $D/logs/i63_routechk_$TAG.log 2>&1; cat $D/logs/i63_routechk_$TAG.log >> $OUT
if ! grep -q '^ROUTE_SAME' $D/logs/i63_routechk_$TAG.log; then   # 다르면 출발 안 함
  echo "route_diff -> no engage $(date +%T)" >> $OUT; docker exec i1_aw pkill -INT -f "i2_watch.py|i4_compare.py|ros2 topic hz" >/dev/null 2>&1; echo done >> $OUT; exit 3
fi
echo "engage_wall $(date +%s.%N) $(date +%T)" >> $OUT
docker exec -u 1000:1000 i1_aw bash -c "$SRC timeout 20 ros2 service call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMode {}" 2>&1 | grep -o "success=[A-Za-z]*.*message='[^']*'" | sed 's/^/engage /' >> $OUT
t0=$(date +%s)
while docker exec i1_aw pgrep -f i2_watch.py >/dev/null 2>&1; do
  [ $(( $(date +%s) - t0 )) -gt $((TO+60)) ] && break
  sleep 5
done
echo "watch_end $(date +%T)" >> $OUT
docker exec i1_aw pkill -INT -f "i4_compare.py|ros2 topic hz" >/dev/null 2>&1
$PY $D/docs/tools/i1_recorder.py ego >> $OUT 2>&1
echo done >> $OUT
