# v3.2 후보 Lanelet2 (횡단보도·과속방지턱)

2026-10-16 작업, 2026-10-22 추가 점검. 팀 지도(v3.1, Lanelet2 28a57711)는 그대로 둠.
중간발표 전에는 팀 지도로 안 바꿈(10/16 뒤 결정). 발표에서는 맵 심화 결과로 보여 주고 시연은 v3.1 로 함.
파일: map/maps/v32_candidate/lanelet2_map.osm (sha256 4719e5e5a4f4588bc0e8a48c53d9bc71beaade7c446389e0f5a4ee3c47cfc789), map_projector_info.yaml(v3.1 과 같음).
만드는 스크립트 map/scripts/s63_v32_candidate.py, 지점 목록 map/maps/v32_candidate/v32_elements.csv(10/22 에 s64_check 열 추가, X30·X44 보류).
방지턱 모듈 켜기 map/maps/v32_candidate/load_speed_bump_plugin.sh(10/22).
10/23: 두 판 map/maps/v32a(42곳, 4719e5e5 그대로)·v32b(X30·X44 뺀 40곳, 48542920). 바꾸는 법 map/docs/v32_switch_howto.md. 실행 스크립트 s59_run.sh 에 SPEED_BUMP=1 선택(기본 꺼짐).

## 넣은 것

- 횡단보도 18곳, 과속방지턱 24곳. X15 는 같은 자리에 둘 다 넣음
- 정한 방법: crossings_v3_candidates.csv 의 keep 이 56행 모두 빈칸이라 OSM 태그로만 정함
  - footway=crossing 길(18개 모두 crossing=marked, zebra) -> 횡단보도
  - traffic_calming(hump, bump) -> 과속방지턱
  - 태그 없는 추정 후보 15행(승강장 가운데, 보행로 만남, v3 에서 지운 보행로)은 안 넣음
  - 줄마다 정한 방법은 v32_elements.csv how 열
- 자리: CSV 의 carla_x, -carla_y(= map 좌표). 가장 가까운 도로 레인렛 진행 방향에 수직으로, 그 단면의 나란한 레인렛을 이은 차도 폭만큼 놓음
  - 42곳 모두 도로 레인렛 위(거리 0.00m). 차도 폭 대부분 6.7m, 교차로 안 4곳은 10.7~17.0m 로 넓게 잡힘(X21, X28, X30, X44)

## 형식

근거: autoware_lanelet2_extension docs/extra_regulatory_elements.md(Crosswalk, Speed Bump), 1.9.0 이미지 안 crosswalk.hpp·speed_bump.hpp.

- 횡단보도: lanelet subtype=crosswalk(경계 type=pedestrian_marking, participant:pedestrian=yes) + way type=crosswalk_polygon area=yes
  + 규제 요소 subtype=crosswalk(refers 레인렛, crosswalk_polygon 면). 정지선은 안 넣음. 모듈이 횡단보도 3.5m 앞에 만듦(crosswalk.param stop_distance_from_crosswalk)
- 과속방지턱: way type=speed_bump area=yes height=0.10 + 규제 요소 subtype=speed_bump(refers 면)
- 면과 겹치는 도로 레인렛 106개가 regulatory_element 멤버로 두 규제 요소를 참조함
- 새 id 는 900001 부터(원본 최대 53168)

## 가정값

OSM 원본에 height·width 태그 없음. 아래는 실측 아님.

- 횡단보도 폭 4.0m, 차도 밖으로 양쪽 1.0m 더 늘림(보도 대기 자리)
- 과속방지턱 길이 3.6m, 높이 0.10m. 원호형 과속방지턱 표준 규격 값을 그대로 씀(10/22 결정)
  - 근거: 길이 3.6m·높이 10cm 원호형은 영국 TRRL 시험(Watts, 1973)에서 표준 규격으로 정한 값임. 국내 도로안전시설 설치 및 관리지침(과속방지턱 편)의 원호형 표준도 이 값으로 알려져 있으나 지침 원문으로는 아직 확인 못 함
  - 확인한 곳: 임창식·최양원(2016), 대한토목학회논문집 36권 6호 1075~1082쪽, https://doi.org/10.12652/Ksce.2016.36.6.1075 2.2절(TRRL 시험, 3.6m·10cm 를 표준 규격으로 결정), 지침 이름은 같은 논문 2.3절(국토해양부, 2011)
  - 지침 원문은 못 열었음(국토교통부 사이트 연결 실패). 근거는 위 논문으로 하고 지침은 원문 미확인으로 둠(10/23 결정, 다시 찾지 않음). 지침이 2026-07 행정예고로 고쳐지는 중이라는 점만 적어 둠
  - speed_bump.param 직선식(0.05m 10km/h, 0.30m 5km/h)으로 0.10m 면 약 9.0km/h

## v3.1 과 달라진 것

- diff: 지운 줄 0, 바뀐 줄 0, 더한 줄 1,978. 그중 기존 도로 레인렛에 member 130줄, 나머지는 새 node·way·relation
- 원본 312,226줄이 순서 그대로 모두 남음. 기존 레인렛·경계·ID 그대로
- 30쌍 Lanelet2 최단 경로(레인렛 id 열) v3.1 == v3.2 후보 30/30(map/scripts/s63_route_check.py, lanelet2 1.2.3)
- Autoware 1.9.0 확장(autoware_lanelet2_extension_python)으로 읽으면 오류 0, speed_bump 24, crosswalk 18

## 플래닝 시뮬레이터 전/후 (CARLA 없음)

실행: integration/docs/tools/s63_ps_run2.sh, 기록 s63_ps_log.py, 비교 s63_compare.py. 로그 integration/logs/s63/.
두 지도 모두 같은 설정. 차 최고 15.5km/h(sample_vehicle).

1.9.0 에서 launch_speed_bump_module:=true 만으로는 방지턱 모듈이 안 켜짐.
뜬 뒤 behavior_velocity_planner/service/load_plugin 에 등록 이름을 넣어 켬(코드 안 고침). 10/22 부터 load_speed_bump_plugin.sh 로 함.

- 런치가 부르는 이름: tier4_planning_launch/.../behavior_planning/behavior_planning.launch.xml 182행 'autoware::behavior_velocity_planner::experimental::SpeedBumpModulePlugin'
- 등록 이름: autoware_behavior_velocity_speed_bump_module/share/.../plugins.xml 2행 'autoware::behavior_velocity_planner::SpeedBumpModulePlugin' 하나뿐(3행 base_class 만 experimental::PluginInterface)
- crosswalk 모듈 plugins.xml 은 2·4행에 두 이름을 다 등록해서 런치 이름(122행)으로 켜짐
- launch_speed_bump_module 기본값은 autoware_launch/config/planning/preset/default_preset.yaml 87~88행 "false"
- load_plugin 응답 success 는 False 로 오지만 파라미터 speed_bump.* 7개가 생기고 로그에 'is loaded' 가 나옴. 스크립트는 파라미터 수로 판정함

구간 시간(출발 ~ 도착), 최대 감속(/localization/acceleration)

| 구간 | v3.1 | v3.2 후보 | 지나간 요소 |
|---|---|---|---|
| p_main_middle | 131.8s, -1.14 | 145.3s, -1.13 | 방지턱 5, 횡단보도 3(X15 는 둘 다) |
| p_middle_north | 73.9s, -1.12 | 77.2s, -1.16 | 방지턱 1, 횡단보도 2 |
| p_north_yang | 349.4s, -1.11 | 365.6s, -1.23 | 방지턱 6, 횡단보도 4 |

- 방지턱마다 앞 15m 15.0km/h -> 위 최저 8.8~9.0km/h -> 뒤 15m 다시 15km/h 근처(v3.1 은 같은 자리 15.0km/h). 표 전체는 integration/logs/s63/compare.txt
- 세 구간 합 555.1s -> 588.1s(+33.0s, +5.9%). 최대 감속은 1.1~1.2m/s2 로 거의 같음
- 사람 없는 횡단보도에서는 안 서고 속도도 안 줄임

횡단보도 X32(p_middle_north 가운데, r1322)에 건너는 더미 보행자

- 보행자는 차 오른쪽 보도 끝(차도 밖 0.5m)에서 횡단보도를 따라 건넘
- 1.0m/s, 차가 35m 앞일 때 출발: 차가 오기 전에 차 차로를 벗어남
  - v3.1: 안 섬, 최저 3.8km/h(장애물·run-out 감속)
  - v3.2 후보: 안 섬, 감속 없음(최저 12.3km/h 는 굽은 길). crosswalk 요인이 24m 앞에서 나왔다가 보행자가 차로를 벗어나며 사라짐
- 0.4m/s, 차가 40m 앞일 때 출발: 차가 닿을 때 보행자가 차 차로 안
  - v3.1: 보행자 출발 10.4s 뒤 섬(0.0km/h), X32 가운데 8.9m 앞(경로 따라, 뒷축 기준), 0.5m/s 밑 5.1s, X32 지남 18.1s
  - v3.2 후보: 10.2s 뒤 최저 0.4km/h, 9.3m 앞, 0.5m/s 밑 2.1s, X32 지남 15.2s
  - 차가 서는 자리는 거의 같음(0.4m 차). v3.2 후보가 2.9s 먼저 지나감. 속도 요인은 v3.1 route-obstacle·run-out·avoidance, v3.2 후보 crosswalk·route-obstacle

![전/후 속도](figures/s63_v32_compare.png)

위 그림은 p_north_yang 속도(점선이 요소 자리), 아래 그림은 0.4m/s 보행자 회차.

X32 앞 인도에 서 있다가 건너는 보행자(10/22, integration/logs/s64/wait_cmp.txt)

- 보행자는 차 오른쪽 차도 밖 0.5m 에 처음부터 서 있다가(v=0), 차가 X32 가운데 20m 안에 들면 1.0m/s 로 건넘(같은 id MODIFY, 직선)
- 시드: dummy_perception_publisher random_seed 0, use_fixed_random_seed false(1.9.0 기본). 검출률 1.0, ray tracing 끔, 직선 이동이라 무작위가 쓰이는 곳 없음
- 서 있는 동안: 둘 다 안 줄임. v3.2 후보는 crosswalk 요인이 35m 앞부터 나오지만 속도는 그대로
- v3.1: 완전히 섬(걷기 시작 5.2s 뒤), X32 가운데 9.1m 앞, 0.5m/s 밑 4.4s, 보행자가 차도를 다 건넌 뒤 0.9s 에 출발, X32 지남 12.1s
- v3.2 후보: 최저 1.7km/h(4.7s), 9.5m 앞, 0.5m/s 밑 0.6s, 보행자가 차 차로를 벗어난 뒤 1.1s 에 가속(다 건너기 2.2s 전), X32 지남 9.6s
- 서는 자리는 같고, v3.2 후보가 2.5s 먼저 지나감. 지나갈 때 보행자는 이미 차도 밖. 0.4m/s 회차(10/16)와 같은 경향

![기다리는 보행자](figures/s64_wait_ped.png)

## 교차로 안 4곳 (10/22)

그림 figures/s64_cross4.png(위 Lanelet2 면, 아래 OSM 원본), 스크립트 map/scripts/s64_cross4.py·s64_pairs4.py.

| 지점 | 종류 | OSM 차도 교차점까지 | 겹친 레인렛 / 방향 갈래 | 판정 |
|---|---|---|---|---|
| X21 | 방지턱 | 55.9m | 4 / 1 | 교차로 아님. 직선 구간에 나란한 road 4개가 겹쳐 폭 10.7m |
| X28 | 횡단보도 | 5.6m | 10 / 2 | T자 교차로 가장자리. 면이 회전 레인렛까지 걸침 |
| X30 | 횡단보도 | 5.2m | 8 / 3 | 교차로 안. 보류 |
| X44 | 방지턱 | 0.7m | 8 / 1 | 교차로 안(합류부). 보류 |

- 결정(2026-10-23 ): X30·X44 는 OSM 그대로 두고 v3.2 는 v32a(42곳)로 함. v32b(40곳)는 대안으로 남김. 팀 지도 전환은 그대로 중간발표 뒤

- 사람 없이 v3.2 후보로 p_yang_back(X44), p_back_south(X21·X28), p_south_middle(X21·X30)을 달림. 모두 도착, 중간 정지 0
  - 방지턱 X21·X44 8.5km/h 로 넘음, X28 은 15km/h 그대로, X30 은 회전 감속 8.7km/h(10/16 p_north_yang 에서 v3.1 도 같은 자리 7.4km/h)
- 거동은 이상 없어 보행자 회차는 안 함. X30·X44 는 면 자리·방향이 실제와 다를 가능성이 커서 v32_elements.csv s64_check 열에 '보류'와 이유를 적음(지우지 않음)

## MRM (10/22)

- v3.1·v3.2 후보를 각 3번 새로 띄워 처음 5분을 봄(방지턱 모듈 켜는 때: 준비되자마자, 달리는 중, 안 켬). 6번 모두 기동 0.6~1.2s 의 1번뿐
- 이번 세션 다른 띄움 4번 중 2단계 v3.2 후보 회차에서 경로 넣기 전후로 2번 더 뜸(34.2s 는 load_plugin 과 같은 때, 49.2s 는 경로 넣은 때). 같은 때 켠 다른 5번은 안 뜸
- 10/16 v3.2 첫 회차 로그를 다시 세면 전체 536번, 처음 300s 안 140번(그 기록의 112번과 다름). 139번이 routing/state STALE, 띄운 뒤 약 100s 부터, load_plugin(212s)보다 먼저 시작
- 정리: 방지턱 모듈 켜기와 묶인다는 근거 없음. routing/state STALE 깜빡임은 이번 10번에서 다시 안 나옴. 원인 못 찾음
- 결론(10/23, 결정 반영): 10/16 깜빡임은 기동 뒤 첫 구간 무렵 /autoware/planning/routing/state STALE 로 생긴 것이고 load_plugin 전에 시작했음. v3.2 후보 지도와 방지턱 모듈과는 무관한 것으로 봄. 10/16 기록의 112번은 536번(300s 안 140번)으로 정정함

## 남은 것

- 가정값(폭·높이)을 현장에서 재야 함. 높이가 바뀌면 방지턱 속도가 바뀜
- X30·X44(보류)는 현장 위치를 보고 면 자리·방향을 다시 정하거나 뺄지 정해야 함. X21·X28 은 그대로 써도 됨
  - 10/23 결정: X30·X44 는 OSM 그대로(v32a). 면 자리·방향이 실제와 다를 수 있다는 점은 한계로 남김
- 1.9.0 런치의 speed_bump 플러그인 이름 문제는 Autoware 쪽 일. CARLA 회차에서도 load_speed_bump_plugin.sh 가 필요함
- 10/16 첫 회차의 MRM 깜빡임(routing/state STALE)은 10/22 에서 10번 띄워 다시 안 나옴. 원인 못 찾음. v3.2 후보·방지턱 모듈과는 무관
- CARLA 쪽은 안 봄. xodr 는 v3.1 그대로라 CARLA 화면에는 횡단보도·방지턱이 없음
