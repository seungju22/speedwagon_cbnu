# 인계 2 대비 바뀐 것 (2026-10-26)

인계 2(GitHub 규민/handoff2, 커밋 bb227ed, 69개) 뒤 작업 폴더에서 바뀌거나 새로 생긴 지도·인지 관련 파일임.
동결 v3.1 묶음(frozen_v31.sha256 10개)과 Autoware 지도 28a57711 은 그대로임.

## 인계 2 안 파일 가운데 바뀐 것 (7개)

| 파일 | 인계 2 sha256 앞 16자(11/04 문구 정리 뒤) | 지금 | 바뀐 것 |
|---|---|---|---|
| integration/docs/tools/s59_run.sh | f1f3b7d1981256ea | afe2f5304d5f986e | 인지 모드 clustering, 센서 기본 54.5만, RAIN·SPEED_BUMP·PAIR_GOAL·LSTATE 선택. 인자를 안 주면 차 시점·맑음·북문 목표 |
| integration/docs/tools/s59_aw_up.sh | 008f8f453e5ade88 | 95bea817a1ade96a | PERCEPTION=clustering, SPEED_BUMP, MAP_COMPOSE·MAP_PATH 선택 |
| perception/gt_objects/gt_objects2.py | ff857bb7c97dd5e3 | af9acbac09a026bf | 새 액터를 틱마다 바로 읽음(처음 낸 시각 1.03 -> 0.10s), 종류별 범위·비 인자 |
| perception/docs/gt_objects2.md | ece7ead601c9b795 | 2b5a497d72fa1574 | 위 고침, 비 값, ρ 설명 |
| perception/docs/interface.md | 1c3aa93bcd3abc62 | 8410b19fe2fed36c | 클러스터링 바꿔 끼우기, 시설·건물 점 지움 |
| integration/docs/howto_campus.md | 7e96b9b650b31849 | 4052574b2bac06a6 | 레이어 v2, clustering, 센서 통일, 비 |
| map/docs/shuttle_route_file.md | ff3d54edabcc2ec6 | 02d206745872a9ad | 대체 정차점 절 |

## 지도

- v3.2(횡단보도 18·방지턱 24): map/maps/v32a(4719e5e5, 팀 판으로 정함), v32b(485429206e986154, X30·X44 뺌). 바꾸는 법 map/docs/v32_switch_howto.md. 도로·CARLA 월드는 v3.1 과 같음
- 방지턱 모듈 켜기: integration/docs/tools/load_speed_bump_plugin.sh(SPEED_BUMP=1)
- 장식 레이어 v2: 건물 2순위 11동(정육면체 164) + OSM 나무 82. map/scripts/place_campus_layers_s66.py, data/buildings_layout_tier2_s66.csv·trees_layout_s66.csv
- 정류장 대체 정차점 10곳: 승강장 2판 shuttle_platforms_v31_r2.yaml(8f9995c5), 노선 3판 pilot_route_r3.yaml(b3067b49). 1판·노선 2판은 그대로
- 지도 재현: OSM 원본부터 v3.1·v3.2 와 레이어 자료까지 다시 만들어 해시를 견주는 s71_reproduce.sh 와 그 입력·스크립트, 문서 map_reproduce.md
- 라이선스: 맨 위 LICENSE(우리 스크립트·문서 MIT), 파일별 구분과 남의 소프트웨어 고지 NOTICE.md
- map/scripts/s27_turnfix.py 한 줄(개인 절대 경로를 스크립트 자리 기준으로, 뜻 같음)

## 인지

- 인지 모드 clustering: Autoware LiDAR 클러스터링 출력을 주행 객체 토픽으로 씀. 정답은 /perception/gt/objects 로만
- 클러스터 앞 점 지움 노드(perception/tools/s67_facility_crop.py, 실행 중 바꿔 끼우기 s67_fac_swap.sh): 정류장 시설 10곳, 레이어 건물 바닥 사각형(여유 0.3m), 클러스터 최소 점 5
- 센서 기본을 세 모드 모두 54.5만 점/s(sensor_mapping_nocam_pps545k.yaml)로 통일. 브리지가 버리는 0.45 만큼 늘린 값
- 비 조건 RAIN=1|2: Goodin 2019 모델. car 는 정답 범위 줄임, clustering 은 점군 열화. ρ 0.07 은 반사율 약 22%. perception/docs/rain_model.md
- 평가 도구(거리별 검출률·거짓 객체·문 구역·처리 시간)와 오프라인 쓸기 스크립트(perception/tools/s67~s70)

## 지도 v3.3·순환 노선·스프린터 (11/02~11/04)

- v3.3a~d Lanelet2: 개성재 승강장, 현장 확인으로 뺀 길, 정문·후문 안쪽 승강장, 유턴 두 곳(도서관 길 모서리, 후문 앞 섬 둘레), N14 통로. 도로·CARLA 월드는 v3.1 그대로. 마지막 판 map/maps/v33d(fe9ee6ff), 설명 map/docs/v33d_readme.md
- 순환 노선 6곳 약 3,406m: circulation_v33d_r1.yaml(프리우스), circulation_v33d_r1_sprinter.yaml(스프린터 문 기준)
- 스프린터(CARLA vehicle.mercedes.sprinter) 설정: vehicle_info, 지붕 LiDAR, CARLA 에서 잰 가속·제동 지도, max_throttle 0.6, 미러 자르기. integration/config/s76_sprinter.override.yaml
- 순환 이어 달리기 integration/docs/tools/s77_run_loop.sh(VEHICLE 로 차 고름, 구간 예상 시간 2배·바퀴 이탈 10초면 끊음)

## 주요 숫자 (근거 작업 기록, 작업 폴더에만 있음)

- 클러스터링(54.5만, 건물 점 지움, 최소 점 5) 90% 검출 거리: 보행자 15m, 차 25m, 콘 10m(13m 86%). 거짓 객체 0.60개/틱, 차 앞 차로 안 0.0019개/틱. 문 구역 84/84
- 비2(25mm/h, 반사율 22%): 보행자 15m, 차 15m, 콘 10m. 거짓 1.33개/틱(건물 벽 잡음)
- 54.5만 대 30만(차 시점): 도착·정지 같음(차이 0.005m), 실시간 0.998배, 최소 가용 -17MiB, 브리지 CPU +12.9%p
- 레이어 v2 대 v1: 도착 80.7s·실시간 0.998배 같음, 최소 가용 -38MiB
