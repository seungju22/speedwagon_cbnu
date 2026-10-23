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

## 출력 모드 두 개 (gt_objects2.py, 세션58~59)

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
