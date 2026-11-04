# 인지 -> 판단 인터페이스 (P2)

2026-10-10 기준. Autoware 1.9.0 이미지, CARLA 0.9.15, 브리지 autoware_carla_interface 0.52.0.
인지 쪽이 판단(planning) 쪽에 넘기는 것은 토픽 하나임. 지금은 정답 객체 노드가 채움.
노드 설명과 시험 결과는 gt_objects.md 에 있음. 경로는 `~/campus_mobility_sim` 을 `C` 로 줄여 씀.

## 토픽

- 이름: `/perception/object_recognition/objects`
- 형식: `autoware_perception_msgs/msg/PredictedObjects`
- 내는 쪽: `C/perception/gt_objects/gt_objects.py` (컨테이너 안)
- 받는 쪽: behavior_path_planner, behavior_velocity_planner, motion_velocity_planner 등 계획 모듈
- 원래 내는 쪽: detection -> tracking -> map_based_prediction. `use_empty_dynamic_object_publisher:=true` 로 셋 다 끔

## 주기와 시각

- CARLA 틱마다 한 번 냄. 틱은 0.05s(브리지 고정)라서 시뮬레이터 시간으로 20Hz 임.
- 실제 시간 주기는 max_real_delta_seconds 에 따름. 0.05 면 거의 실시간 20Hz, 0.2 면 실제 5Hz.
- header.stamp 는 그 틱의 CARLA elapsed_seconds 에서 브리지 /clock 과의 차이를 뺀 값임.
  차이는 틱 단위 최빈값 묶음의 중앙값으로 정함(정답 측위 i3_gt_localizer.py 와 같은 방법).
- Town01 시험에서 stamp 와 그때 /clock 의 차는 0 또는 0.05s(한 틱)였음.

## frame_id 와 좌표

- header.frame_id 는 `map` 임.
- map 좌표 = CARLA 좌표에서 y 를 뒤집은 것 (x, -y). yaw 도 부호를 바꿈. 원점 이동은 없음.
  브리지 0.52.0 과 같은 규칙임(carla_utils.py 95행, 126행).
- 캠퍼스 지도는 xodr 평면 좌표를 그대로 map 좌표로 씀(map_projector_info.yaml 이 Local).
  xodr 머리말의 offset 은 CARLA 가 읽지 않으므로 적용하지 않음.
- z 는 bounding box 중심 높이임. 캠퍼스는 고도가 모두 0 임.

## 객체 하나에 채우는 필드

- object_id: CARLA actor id 를 UUID 16바이트 앞쪽에 넣음. 같은 actor 는 늘 같은 id 임.
- existence_probability: 1.0
- classification: 라벨 하나, probability 1.0
- kinematics.initial_pose_with_covariance: bbox 중심 위치, 방향(yaw 만). 공분산 대각 1e-4, 나머지 0
- kinematics.initial_twist_with_covariance: 객체 좌표계 속도(앞 x, 왼쪽 y), 각속도 z. 공분산 대각 1e-4
- kinematics.initial_acceleration_with_covariance: 값 0, 공분산 대각 1e-4
- kinematics.predicted_paths: 등속 직선 한 개, time_step 0.5s, 8s(17점), confidence 1.0
- shape: BOUNDING_BOX, dimensions = bbox extent 의 두 배(x 길이, y 폭, z 높이)
- 대상: 자차를 뺀 vehicle.* 와 walker.pedestrian.*, 자차 중심에서 80m 안(--range)

## 출력 모드 두 개 (gt_objects2.py, 10/14)

같은 토픽·형식으로 두 모드를 냄. `--mode all`(다 보기)은 위 P1 과 같이 자차 중심 80m 안 차량·보행자를 모두 냄.
`--mode car`(차 시점, 기본)는 지붕 LiDAR 자리(차 원점 기준 -0.39, 0, 1.84m)에서 거리·상하 시야(+10~-30도)·가려짐(cast_ray)을
넘은 것만 냄. 거리는 보행자 17.5m, 차량 35m, 표시 단 소품 12.5m 이고, 기준 센서(64채널·초당 30만 점·버림 0)로 시맨틱 LiDAR 가
10점 이상 맞히는 거리에서 정했음(perception/docs/gt_objects2.md). 두 모드 모두 role_name obstacle 인 static.prop 을 UNKNOWN 으로 더 냄.
판단 쪽은 모드를 몰라도 됨. 차 시점에서는 물체가 늦게 나타나고 가려지면 빠진다는 점만 다름.

## 라벨 대응 (CARLA -> Autoware ObjectClassification)

- walker.pedestrian.* -> PEDESTRIAN(7)
- vehicle.*, base_type motorcycle -> MOTORCYCLE(5)
- vehicle.*, base_type bicycle -> BICYCLE(6)
- vehicle.*, base_type bus -> BUS(3)
- vehicle.*, base_type truck 또는 van -> TRUCK(2)
- 나머지 vehicle.* -> CAR(1)
- UNKNOWN(0), TRAILER(4) 는 내지 않음

## 정답 객체와 클러스터링 바꾸는 법

정답 객체(기본)
- 런치 인자 `use_empty_dynamic_object_publisher:=true`
- compose 에 `-f C/perception/config/gt_objects.override.yaml`
- 기동 뒤 `docker exec i1_aw pkill -f empty_objects_publisher` 로 빈 목록 노드를 끔
- 노드 실행: gt_objects.md "켜는 법"
- 한 번에 하는 스크립트: `C/integration/docs/tools/i5_run.sh`(캠퍼스), `C/perception/tools/run_ped.sh`(Town01)

LiDAR 클러스터링
- 위 인자와 오버레이를 빼고 `lidar_detection_model:=clustering` 만 둠(I1~I4 설정)
- 점군 지도가 없으면(`use_pointcloud_map:=false`) compare map 필터가 꺼져 건물·벽도 장애물로 잡힐 수 있음

주행 중 클러스터링으로 바꿔 끼우기 (2026-10-23)
- 캠퍼스 회차: `LSTATE=<상태 파일> bash C/integration/docs/tools/s59_run.sh <tag> <쌍> clustering <장면> [cam] [bag]`.
  s59_aw_up.sh 에 PERCEPTION=clustering 이 넘어가 `use_empty_dynamic_object_publisher:=true` 를 안 붙임. 다른 런치 인자는 정답 객체 회차와 같음.
- 이때 gt_objects2.py 는 차 시점으로 돌되 `--topic /perception/gt/objects` 로만 냄. 계획은 Autoware 인지 출력만 받음. 정답 토픽은 비교용임.
- 1.9.0 흐름: obstacle_segmentation/pointcloud > pointcloud_map_filter(지도 없으면 voxel 만) > clustering/euclidean_cluster > shape_estimation > detected_object_feature_remover
  > clustering/objects > multi_object_tracker > map_based_prediction > /perception/object_recognition/objects.
- tier4_perception_component 기본 use_multi_channel_tracker_merger=true 라 추적기가 clustering/objects 를 바로 받음. lanelet 필터와 detection/objects 는 이 경로에 없음.
  그래서 차로 밖 건물·정류장 시설 덩어리도 UNKNOWN 객체로 나감. 10/23 회차에서는 궤적 밖이라 계획이 서지 않았음.
- 클러스터 노드 처리 시간은 `.../clustering/voxel_grid_based_euclidean_cluster/debug/processing_time_ms` 로 나옴. 10/23 중앙 0.5~0.7ms.
- 정류장 시설 점 지움: `bash C/perception/tools/s67_fac_swap.sh <tag>` 를 Autoware 가 뜬 뒤 부름. s67_facility_crop.py 가 시설 다각형 안 점을 지우고,
  실행 중 euclidean_cluster 컴포넌트를 내리고 입력만 바꿔 다시 올림. Autoware 소스와 런치 파일은 안 고침.
- 10/23: clustering 회차는 시설 점 지움을 기본으로 켬(s59_run.sh 가 기동 뒤 s67_fac_swap.sh 를 부름). 비교 실험으로 끄려면 `FACCROP=0`.
  - 한계: 정류장 시설 바닥 다각형(+0.3m) 안에 선 사람의 점도 같이 지워짐. 문 구역은 처음부터 시설을 뺀 구역으로 그렸으니(노선 파일 1판) 출발 판단 구역에는 영향 없음.
- 10/25: clustering 기본을 바꿈. 지움에 건물 바닥 사각형(perception/config/s69_building_rects.json, 레이어 v1·v2 건물, BLDCROP=0 이면 끔)을 더하고,
  다시 올리는 클러스터의 min_points_per_cluster 를 5 로(CLMIN, autoware_launch 값 10 은 CLMIN=10). LiDAR 는 54.5만 점/s(sensor_mapping_nocam_pps545k.yaml, s59_run.sh·s68_static_run.sh 의 clustering 기본).
  - 근거: 같은 CARLA 거리별 평가에서 보행자 90% 거리 5m -> 15m(13m 100%), 차 앞 차로 안 거짓 0.0019개/틱, 문 구역 84/84, 실시간 0.998. 콘은 13m 86%(한계). integration/docs/s69_report.md 3·4단계
- 추적기 입력은 지금 경로(lanelet 필터 안 거침)로 둠. 이유:
  - 클러스터 객체는 모양 추정만 해서 라벨이 모두 UNKNOWN 임.
  - object_lanelet_filter 는 기본으로 UNKNOWN 만 차로와 안 겹치면 지움(autoware_launch 0.52.0 object_lanelet_filter.param.yaml filter_target_label, 1.9.0 이미지에서 UNKNOWN true·나머지 false 확인, 10/23).
  - 거치게 하면 인도 위 문 구역 보행자(차로 밖)도 UNKNOWN 이라 같이 지워짐.
  - 지금 거짓 객체는 건물·정류장 시설 자리뿐이고 차로 위는 0 임(10/23 다섯 회차). 계획도 이 때문에 선 일 없음.
- 평가 도구: perception/tools/s67_objects_dump.py(bag > CSV·처리 시간), s67_false_objects.py(거짓 객체 자리), s67_door_compare.py(문 구역·전방 통로).
- 결과와 한계: integration/docs/s67_report.md 4·5단계, docs/midterm_material/tab10_clustering_online.md.

## 한계

- 예측 경로가 등속 직선 하나라 방향을 바꾸는 객체는 늦게 반영됨.
- 가려짐이 없음(P1, gt_objects2 all). 건물 뒤 객체도 80m 안이면 보임. 차 시점(gt_objects2 car)은 가려짐을 봄.
- 신호등, 정지 표지판, 정적 소품(static.prop)은 내지 않음.
  (gt_objects2.py 는 role_name obstacle 인 소품만 UNKNOWN 으로 냄)
- gt_objects2 차 시점: 보행자 상자 안에 있는 건물 벽은 가림으로 못 봄(보행자 둘레 보이지 않는 cast_ray 상자를 빼느라).
- gt_objects2 두 모드: actor 목록을 1초마다 다시 읽어 새 물체는 최대 1초 늦게 들어옴.
- 장애물 점군 경로(obstacle_segmentation)는 그대로 돎. 점군을 쓰는 모듈은 건물을 볼 수 있음.
- empty_objects_publisher 를 손으로 끄는 단계가 있음. 끄지 않으면 빈 목록과 섞임.
- 공분산은 형식만 채운 작은 값임. 실제 센서 오차를 흉내 내지 않음.

## 판단 담당에게 확인할 것

1. 예측 경로가 등속 직선 하나, 8s 면 충분한가. 지도 따라가는 예측(map_based_prediction)이 필요한가.
2. 정적 소품(콘, 표지)을 장애물로 받아야 하는가. 지금은 빠짐.
3. 탐지 거리 80m 와 가려짐 없음을 그대로 써도 되는가, 아니면 센서 범위·가려짐을 넣어야 하는가.
