#!/bin/bash
# v3.2 후보용: 떠 있는 Autoware 1.9.0 컨테이너에서 방지턱 모듈(speed_bump)을 켬. 팀 지도(v3.1) 회차에는 아직 안 씀(10/22)
#
# 왜 필요한가(1.9.0 이미지 안 파일)
#   tier4_planning_launch/share/tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml
#     182행: launch_speed_bump_module 이 true 면 'autoware::behavior_velocity_planner::experimental::SpeedBumpModulePlugin' 을 모듈 목록에 넣음
#   autoware_behavior_velocity_speed_bump_module/share/autoware_behavior_velocity_speed_bump_module/plugins.xml
#     2행: 등록 이름은 'autoware::behavior_velocity_planner::SpeedBumpModulePlugin' 하나뿐(experimental:: 없음, base_class 만 experimental::PluginInterface)
#   그래서 띄움 로그에 "The scene plugin '...experimental::SpeedBumpModulePlugin' is not available." 이 나오고 모듈이 안 켜짐
#   비교: crosswalk 모듈 plugins.xml 은 2·4행에 두 이름을 다 등록함(그래서 런치 122행 experimental:: 이름으로 켜짐)
#   autoware_launch/config/planning/preset/default_preset.yaml 87~88행: launch_speed_bump_module 기본값 "false"
# 하는 일: behavior_velocity_planner 의 ~/service/load_plugin(autoware_internal_planning_msgs/srv/LoadPlugin)에 등록 이름을 넣음
#   Autoware 코드·런치는 안 고침. 컨테이너를 다시 띄우면 다시 불러야 함
#   응답 success 는 1.9.0 에서 False 로 와도 실제로는 켜짐(10/16·10/22). 그래서 판정은 파라미터 speed_bump.* 수와 로그 'is loaded' 로 함
# 쓰는 때: 지도를 읽고 AD API(/api/localization/initialize)가 보인 뒤. 경로를 넣기 전에 부르는 것이 좋음(이미 지난 방지턱은 안 봄)
# 사용: load_speed_bump_plugin.sh <컨테이너 이름>   예: load_speed_bump_plugin.sh s64_w_v32c
#   끝 줄: 'speed_bump on params=N' (N>0 이면 켜짐) 또는 'speed_bump off'. 이미 켜져 있으면 부르지 않고 'already' 를 붙임
set -u
CT=${1:?컨테이너 이름}
NODE=/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner
NAME='autoware::behavior_velocity_planner::SpeedBumpModulePlugin'
X() { docker exec -u 1000:1000 "$CT" bash -c "source /opt/autoware/setup.bash; $1" 2>&1; }
np() { X "timeout 20 ros2 param list $NODE" | grep -c 'speed_bump\.'; }
for i in $(seq 1 60); do   # AD API 가 보일 때까지(최대 5분)
  X "ros2 service list 2>/dev/null" | grep -q /api/localization/initialize && break; sleep 5
done
n=$(np)
if [ "$n" -gt 0 ]; then echo "$(date +%T) speed_bump on params=$n already"; exit 0; fi
r=$(X "timeout 20 ros2 service call $NODE/service/load_plugin autoware_internal_planning_msgs/srv/LoadPlugin \"{plugin_name: '$NAME'}\"" | grep -o 'success=[A-Za-z]*')
sleep 2; n=$(np)
echo "$(date +%T) load_plugin response ${r:-none}"
if [ "$n" -gt 0 ]; then echo "$(date +%T) speed_bump on params=$n"; else echo "$(date +%T) speed_bump off"; exit 1; fi
