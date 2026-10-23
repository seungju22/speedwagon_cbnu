# 정답 객체 2판 gt_objects2.py (차 시점 노드) 사용법

2026-10-14 세션59 기준. 노드 `C/perception/gt_objects/gt_objects2.py`, 판정 `C/perception/gt_objects/carview.py`. `C` = `~/campus_mobility_sim`.
P1(gt_objects.py)과 토픽·형식이 같음: `/perception/object_recognition/objects`(PredictedObjects, frame map). 필드는 interface.md.

## 모드
- `--mode car`(기본, 차 시점): 지붕 LiDAR 자리에서 보이는 물체만 냄. 세 조건을 모두 넘어야 함
  - 거리: LiDAR 자리에서 물체 상자 중심까지가 종류별 거리 안
  - 시야: 상자 표본점(10개) 가운데 하나 이상이 LiDAR 상하 시야(+10 ~ -30도) 안
  - 가려짐: 시야 안 표본점 하나 이상이 world.cast_ray 로 안 막힘(자기 상자·자차 상자·지면은 막힘으로 안 침)
- `--mode all`(다 보기): P1 과 같음. 자차 중심 `--range`(80m) 안 모두

## 대상
- 자차 뺀 `vehicle.*`, `walker.pedestrian.*`, role_name 이 `obstacle` 인 `static.prop.*`(UNKNOWN 으로). 두 모드 같음
- 레이어 소환물(건물·정류장 시설·지면 판)은 role_name 이 없어서 안 냄

## 인자와 기본값
- `--ped-range 17.5`, `--veh-range 35`, `--prop-range 12.5` (m, LiDAR 자리 기준)
- `--range-scale 1.0`: 세 거리에 곱함(비 조건 자리)
- `--miss-prob 0`, `--seed 0`: 물체·틱마다 버릴 확률(비 조건 자리, 기본 꺼짐)
- `--upper-fov 10`, `--lower-fov -30`, `--lidar -0.390,0.0,1.840`(CARLA 차 원점 기준 LiDAR 자리)
- `--log 파일.csv`: 틱마다 물체마다 한 줄(frame, sim, mode, aid, label, x, y, dist_ego, dist_lidar, why, vis_lag, n_fov, n_clear).
  why = pub(냄) / range(거리 밖) / fov(시야 밖) / occluded(가려짐) / novis(아직 판정 전) / miss(버림)
- `--check N`: ROS 없이 N 틱 판정만 찍음(호스트 `.venv-carla` 에서도 됨)

## 기본 거리의 근거 (세션59)
- 기준 센서: 브리지 기본 LiDAR(1.9.0 이미지 sensor_mapping.yaml) 64채널, 초당 30만 점, 회전 20Hz, 범위 100m, 상하 +10/-30도,
  센서 킷 자리. 무작위 버림 0, 노이즈 0(백로그 S1 기준 조건)
- 잰 방법: `C/perception/tools/s59_lidar_bench.py --lidar ref`. 곧은 길(r1411 s40, r1415 s80)에 차를 세우고 물체를 하나씩 앞·옆에 둠.
  10틱 기다린 뒤 5틱 중앙값으로 시맨틱 LiDAR 가 그 물체(ObjIdx)에 맞힌 점 수를 셈
- 기준: 10점 이상(Autoware 클러스터링 min_points_per_cluster 10, voxel_grid_based_euclidean_cluster.param.yaml)
- 10점 이상 가장 먼 거리(두 자리 모두): 보행자 17.7m, 승용차 audi.a2 35.2m, 콘 12.7m -> 17.5 / 35 / 12.5
- 가까운 쪽: 차 옆 0.3~3m(문 구역 자리) 보행자 298~78점, 콘 28~150점, 앞 범퍼 0.5m 보행자 104점. 10점 못 미치는 거리가 없어 가까운 쪽 한계는 두지 않음
- 보수적임: 한 바퀴 1만 5천 점이라 실제 센서(VLP-16 10Hz 약 3만, OS1-64 1024x10 약 6.5만)보다 성김. 클러스터링은 0.3m 복셀 뒤
  10점이라 실제로는 더 가까워야 잡힐 수 있음
- 세션58 값(보행자 7.5, 차량 20, 소품 6.0m)은 '가벼운 LiDAR'(32채널·초당 10만 점, I1 부터 쓰던 파일) 기준임. 발표 근거로 쓰지 않음.
  같은 방법 대조표는 `C/integration/docs/s58_report.md` 3-0
- 시뮬레이터 LiDAR(브리지)도 세션59 부터 64채널·30만 점임(howto_campus.md). 다만 브리지는 버림 0.45 를 못 바꿈

## 백로그 S1 닫음 (2026-10-15 세션60, 사용자 결정)
- 센서 설정은 위 기준 센서(64채널·초당 30만 점·20Hz·100m·센서 킷 자리)로 정함. 시뮬레이터 LiDAR(브리지)도 같은 값임
- 원래 끝 기준 '20m 보행자 10점 이상'은 못 맞췄음. 기준 센서로 보행자 10점 이상은 17.7m 까지임(20.7m 8점)
- 끝 기준을 바꿨음: 운행 속도 15km/h 에서 반응 1초 + 일반 감속 1.0m/s2 로 서는 거리 안의 보행자에 10점 이상
  - 정지 거리 = 4.17m/s x 1s + 4.17^2 / (2 x 1.0) = 4.17 + 8.68 = 12.9m
  - 보행자는 12.7m 13점, 15.7m 11점, 17.7m 10점이라 12.9m 안을 넘게 덮음(기본값 17.5m)
- 바꾼 이유: 20m 는 근거 없이 둔 어림값이었음. 셔틀이 실제로 서야 하는 거리(운행 속도·반응·감속)로 정하는 쪽이 설명이 됨.
  15km/h 는 캠퍼스 셔틀 운행 속도 가정, 1.0m/s2 는 승객이 서 있어도 되는 일반 감속으로 둔 가정임
- 콘(소품)은 이 거리에 조금 못 미침. 10점 이상이 12.7m(r1415, r1411 은 13.7m)까지라 기본값 12.5m 가 12.9m 보다 0.4m 짧음.
  콘처럼 낮고 작은 물체는 정지 거리 끝(12.5~12.9m)에서 늦게 잡힐 수 있음
- 한계: 브리지는 LiDAR 의 range·rotation_frequency·channels·upper_fov·lower_fov·points_per_second 만 넣음.
  무작위 버림(CARLA 기본 dropoff_general_rate 0.45)·노이즈는 못 바꿈. 그래서 주행 중 점군은 거리 기준(버림 0)보다 점이 약 절반임
  - 근거: 1.9.0 이미지 /opt/autoware/autoware_carla_interface/local/lib/python3.10/dist-packages/autoware_carla_interface/modules/carla_wrapper.py
    _configure_lidar_attributes(249~256행). 바꾸려면 브리지 소스를 고쳐야 해서 하지 않음(서버 자율 운용 규칙 2번)

## 켜는 법
- 한 번에: `C/integration/docs/tools/s59_run.sh <tag> <쌍> <car|all> <장면> [cam] [bag]` (GTARGS 환경 변수로 인자 더함)
- 손으로(컨테이너 안, empty_objects_publisher 를 끈 뒤):
  `python3 -u /home/aw/gt_perception/gt_objects/gt_objects2.py --mode car --log /home/aw/gt_logs/<tag>.csv`
- 노드는 30초마다 `stat tick_ms / vis_ms` 중앙값·95%·최대를 찍음. 세션59 차 시점 틱 중앙값 0.37ms

## 한계
- 판정은 표본점 10개 중 하나라도 보이면 보임으로 침. 부분 가림에 너그러움(점 수는 세지 않음)
- 보행자 둘레에 cast_ray 에만 걸리는 보이지 않는 상자(라벨 NONE)가 있어 '라벨 NONE + 보행자 상자 안' 은 가림으로 안 침.
  그래서 보행자 상자 안에 있는 건물 벽은 못 봄
- actor 목록을 1초마다 다시 읽음. 새로 생긴 물체는 두 모드 모두 최대 1초 늦게 들어옴
- 새 물체의 첫 1틱은 판정 전(novis)이라 차 시점에서 빠짐
- 거리는 종류별 상수임. 물체 크기·방향·자세에 따른 점 수 차이를 넣지 않음
