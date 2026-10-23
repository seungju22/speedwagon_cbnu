#!/bin/bash
# 세션62: 플래닝 시뮬레이터의 가상 문(launch_dummy_doors, 기본 켜짐, 문 4개)에 AD API 로 열기/닫기를 보내고 상태를 본다
# 사용: s62_ps_door.sh <open|close> [index]   상태 값: 2 OPENED, 3 CLOSED, 4 OPENING, 5 CLOSING
C=$([ "$1" = open ] && echo 1 || echo 2); IDX=${2:-0}; CT=${CT:-s62_ps}
docker exec -u 1000:1000 $CT bash -c "source /opt/autoware/setup.bash
timeout 10 ros2 service call /api/vehicle/doors/command autoware_adapi_v1_msgs/srv/SetDoorCommand '{doors: [{index: $IDX, command: $C}]}' | grep -o 'success=[A-Za-z]*.*message=.[^)]*'
for t in 0 1 2 3 4 5 6; do echo \"\$t s: \$(timeout 5 ros2 topic echo --once /api/vehicle/doors/status --field doors | grep -o 'status=[0-9]' | tr '\n' ' ')\"; sleep 1; done" 2>&1
