# 지도 v3.3b 설명 (11/02·11/03, 2026-11-02~03)

인계 2(v3.1) 이후 맵 점검(11/2) 판정으로 고친 판임. 도로(xodr 214507af)·CARLA 월드·장식 레이어는 v3.1 그대로이고, Autoware Lanelet2 지도만 바뀜.
v3.1 동결 묶음과 ~/autoware_data/maps/OpenDriveMap 은 그대로 둠. 새 판은 다른 경로의 OpenDriveMap 폴더로 고름.

## 파일
- map/maps/v33b/lanelet2_map.osm(바뀐 것 v33b_changes.json), 바탕 v3.3a map/maps/v33a(바뀐 것 v33a_changes.json, 뺀 road 표와 근거)
- 승강장 map/data/shuttle_platforms_v33b_r1.yaml, 순환 노선 map/data/routes/circulation_v33b_r1.yaml
- 만들기 map/scripts/s73_make_v33.py(v3.3a) -> s74_make_v33b.py(v3.3b) -> s73_platforms.py -> s73_loop.py -> s74_write.py, 확인 s74_check_v33b.py, 차체 검사 s74_m20.py

## 인계 2(v3.1) 대비 바뀐 것
- 중문 -> 개성재: 옛 중문 승강장 자리(미술관~진리관 사이 남북 길)는 바로 위 볼라드로 차가 못 지나가 뺌. 개성재 게이트(관리동 앞 교차로) 안 길과 정의관 남쪽 동서 길도 뺌(현장 확인). OSM 은 이 길들을 차도(service)로 적고 있어 'OSM 과 다르게 뺀 것'으로 표시함
- 개성재 승강장: 미술관과 N14(인문사회관) 사이 대각선 길, 게이트 앞 약 33m. 화면 이름 '개성재', 내부 ID(middle_gate_*)·30쌍 이름은 그대로
- 개성재에서 나올 때: N14 를 ㄷ자로 두른 주차장 통로(10/12 에서 뺐던 r1318·r1188)를 되살려 한 바퀴 돌아 나옴. 차가 없을 때 개성재 앞에서 크게 도는 길도 있다고 함(레인렛은 안 만듦)
- 남문: 문 구역을 막던 r1235·r1369 는 OSM 인도(footway)였음. 빼서 남문(이쪽) 하차 가능
- 북문: 개신문화관 앞에서 좌회전해 올라간 북문 길 끝(사범대 앞)에서 유턴(유턴 레인렛). 10/09 에 이은 북쪽 고리 연결은 뺌. 북문 출발 쌍이 각 약 325m 짧아짐. 사범대 앞 승강장(교대 학생 하차, 선택)
- 후문: 후문 앞 유턴 레인렛(: 공간 있음, 맞은편 차가 안 올 때 돌림. 기다리는 판단은 판단 쪽 몫)
- 정문·후문 안쪽 순환용 승강장: 원래 정문·후문 승강장은 지도 끝 road 라 고리에 못 넣어 30쌍 시험용으로 남김

## 유턴 레인렛
- 모양: 열쇠 구멍(오른쪽으로 벌렸다가 왼쪽으로 크게 돌고 돌아옴). 중심선 반경 5.0m(스타렉스 뒷축 최소 반경 3.77m 보다 넉넉), 가운데 폭 4.0m
- 필요한 자리: 차로 끝에서 앞으로 약 12.5m, 차로 밖으로 양쪽 약 3.3m. 넓이 근거는 현장 확인(OSM 에 그 면은 없음)
- 북문 relation 52799(r1415 끝 -> r1409), 후문 relation 52800(r1251 끝 -> r1384)

## 순환 노선
- 한 바퀴 약 3,482m: 정문 안쪽 > 양성재(이쪽) > 후문 안쪽 > (후문 앞 유턴) > 도서관 남문(건너편) > 도서관 북문 > 사범대 앞 > (북문 길 끝 유턴) > 개성재 > (N14 통로) > 정문 안쪽
- 노선 파일의 정류장마다 via(요청 없으면 지나감)와 goal(요청 있으면 섬). 순환 운행 논리는 판단 쪽 몫이라 여기 없음
- 큰 도로 36개 가운데 19개가 양방향·고리 안. 나머지는 지도 끝, OSM 일방, 막다른 주차장 길, 뺀 길(표: integration/docs/s74_report.md 1절). 그림 map/docs/figs/v33b_loop.png
- 덮기 노선(고리 안 큰 도로를 모두 지나게 경유점): 약 5,389m

## 쓰는 법
- CARLA: MAP_COMPOSE=integration/config/s74_v33b_map.override.yaml MAP_PATH=/home/aw/s74_maps/OpenDriveMap 를 붙여 s59_run.sh(한 목표) 또는 s74_run_loop.sh(LOOP_SEGS 로 순환 구간)를 돌림. 사본 폴더 integration/maps/s74_v33b/OpenDriveMap(이름은 OpenDriveMap 이어야 함)
- 플래닝 시뮬레이터: map_path 로 v33b 사본 폴더(integration/docs/tools/s74_ps_run.sh)

## 차량 바꾸는 법(기본값은 아직 sample_vehicle·프리우스)
- VEHICLE_TYPE=<CARLA 블루프린트>(브리지 vehicle_type), VEH_COMPOSE=integration/config/s73_starex.override.yaml(스타렉스 vehicle_info, 지붕 LiDAR z 2.13), SMAP=<밴 센서 파일(vehicle_config.wheelbase = CARLA 원점 -> 뒷축 x 2)>
- 문 위치·승강장 목표: map/scripts/s73_platforms.py --veh starex(문 위치 DOOR_X 1.502)
- 정답 노드 LiDAR 자리: GTARGS='--lidar <x>,0,2.13'

## 한계
- N14 통로 ㄷ자 모서리는 바깥 경계를 최대 3.0m 넓혔지만 차체 검사에서 지금 차·스타렉스가 아직 조금 나감. 개성재 승강장 앞 r1316 꺾어 드는 곳도 0.27~0.55m. 실주행 결과는 integration/docs/s74_report.md
- 스타렉스 앞뒤 오버행은 공식 표에 없어 가정. 최소 회전 반경 5.6m 의 기준점도 표에 없어 바깥 앞바퀴로 봄
- v3.2 횡단보도·방지턱은 아직 v3.3b 에 안 얹음
- 실제 볼라드·게이트·유턴 넓이는 현장 확인에 기댐(OSM 에 없음)
