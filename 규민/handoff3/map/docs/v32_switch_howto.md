# v3.2 로 바꾸는 법 (팀원용, 2026-10-23)

지금 팀 지도는 v3.1(Lanelet2 28a57711)임. 언제 v3.2 로 바꿀지는 중간발표 뒤에 정함. 이 문서는 바꿀 때 순서만 적음.
v3.2 는 v3.1 에 횡단보도·과속방지턱을 더한 Lanelet2 판임. 도로(xodr 214507af)와 CARLA 월드는 그대로라 CARLA 화면에는 횡단보도·방지턱이 안 보임.
내용과 시험 결과는 map/docs/v32_candidate.md.

## 판 두 개

v3.2 는 v32a(OSM 그대로 42곳)로 정함(10/23 결정, X30·X44 포함). v32b 는 지우지 않고 대안으로 둠. X30·X44 를 빼야 할 일이 생기면 v32b 를 씀.

| 판 | 폴더 | 요소 | lanelet2_map.osm sha256 |
|---|---|---|---|
| v32a | map/maps/v32a | 횡단보도 18, 방지턱 24(X30·X44 포함) | 4719e5e5a4f4588bc0e8a48c53d9bc71beaade7c446389e0f5a4ee3c47cfc789 |
| v32b | map/maps/v32b | 횡단보도 17, 방지턱 23(X30·X44 뺌) | 485429206e9861547e8ef5a2fa5858637abea94f41fcfae38e9044d28ded4305 |

- map_projector_info.yaml 은 셋 다 같음(a6fb6af3)
- 둘 다 30쌍 Lanelet2 경로가 v3.1 과 같음(30/30), Autoware 1.9.0 확장으로 읽기 오류 0
- v32b 는 v32a 에서 X30·X44 가 만든 줄만 지운 판(map/scripts/s66_make_v32ab.py, 지운 목록 map/maps/v32b/v32b_dropped.txt)

## 바꾸기

1. CARLA·Autoware 를 모두 끔
2. 지금 지도 폴더를 옆에 남김

       cp -a ~/autoware_data/maps/OpenDriveMap ~/autoware_data/maps/OpenDriveMap_v31_bak

3. v32a 의 lanelet2_map.osm 을 지도 폴더에 덮어씀(폴더 이름은 OpenDriveMap 그대로. 브리지가 이 이름으로 CARLA 맵을 고름)

       cp ~/campus_mobility_sim/map/maps/v32a/lanelet2_map.osm ~/autoware_data/maps/OpenDriveMap/lanelet2_map.osm
       sha256sum ~/autoware_data/maps/OpenDriveMap/lanelet2_map.osm

4. 해시가 위 표와 같은지 봄

## 방지턱 모듈 켜기

Autoware 1.9.0 런치는 방지턱 모듈을 이름 불일치로 못 켬(런치 experimental:: 이름, 패키지 등록 이름 다름). 그래서 뜬 뒤 load_plugin 서비스로 켬.
횡단보도 모듈은 원래 켜져 있어 따로 할 것 없음.

- CARLA 회차: 실행 스크립트에 SPEED_BUMP=1 을 붙임

      SPEED_BUMP=1 bash ~/campus_mobility_sim/integration/docs/tools/s59_run.sh try1 p_middle_north car none

  로그 `integration/logs/i5_run_try1.log` 에 'speed_bump on params=7' 이 나오면 켜진 것. 안 붙이면 예전과 똑같이 돎(모듈 꺼짐)
- 플래닝 시뮬레이터: 뜬 뒤 경로를 넣기 전에 `bash integration/docs/tools/load_speed_bump_plugin.sh <컨테이너 이름>`
- 다른 쌍을 30쌍 목표로 달릴 때는 PAIR_GOAL=1 도 붙임(안 붙이면 북문 목표, 예전과 같음)

## 되돌리기

    cp ~/autoware_data/maps/OpenDriveMap_v31_bak/lanelet2_map.osm ~/autoware_data/maps/OpenDriveMap/lanelet2_map.osm
    sha256sum ~/autoware_data/maps/OpenDriveMap/lanelet2_map.osm   # 28a57711094effaf3e0523cfd103e94a010c2bac01d37feebd0de90438dd02f9

SPEED_BUMP=1 을 빼면 방지턱 모듈도 안 켜짐(컨테이너를 새로 띄우면 꺼진 상태로 시작함).

## 알아 둘 것

- 크기는 가정값임(횡단보도 폭 4.0m, 방지턱 길이 3.6m·높이 0.10m). 방지턱 위 속도 약 9km/h, 구간 시간 +4.6~9.5%(CARLA 두 구간)
- CARLA 도로에는 방지턱 모양이 없어 차는 평평한 길에서 감속만 함
- 사람이 서 있는 횡단보도에서는 v3.1 보다 빨리 다시 출발함(보행자가 차 차로를 벗어나면 출발). 결과는 v32_candidate.md
