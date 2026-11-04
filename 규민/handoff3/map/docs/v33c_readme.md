# 지도 v3.3c 설명 (2026-11-03)

v3.3b 에서 유턴 자리와 N14 모서리를 고친 판임. 도로(xodr 214507af)·CARLA 월드·장식 레이어는 v3.1 그대로이고, Autoware Lanelet2 지도만 바뀜.
v3.1 동결 묶음과 ~/autoware_data/maps/OpenDriveMap 은 그대로 둠. 새 판은 다른 경로의 OpenDriveMap 폴더로 고름.
v3.3a·b 에서 바뀐 것(개성재 승강장, 뺀 길, 후문 앞 유턴, 정문·후문 안쪽 승강장 등)은 map/docs/v33b_readme.md 그대로임.

## 파일
- map/maps/v33c/lanelet2_map.osm(바뀐 것 v33c_changes.json), 바탕 v3.3b map/maps/v33b
- 승강장 map/data/shuttle_platforms_v33c_r1.yaml(값은 v33b 와 같음, 사범대 앞 레인렛 id 만 바뀜), 순환 노선 map/data/routes/circulation_v33c_r1.yaml
- 만들기 map/scripts/s75_uturn_search.py(유턴 모양 찾기) -> s75_make_v33c.py -> s75_write.py. 확인 s75_check_v33c.py, 차체·바퀴 검사 s75_m20.py, 그림 s75_uturn_fig.py·s75_loop_fig.py
- 다시 만들기: `.venv-carla/bin/python map/scripts/s75_uturn_search.py --emit 0` -> `python3 map/scripts/s75_make_v33c.py` -> `~/lanelet2_trial_venv/bin/python map/scripts/s75_check_v33c.py map/maps/v33c/lanelet2_map.osm integration/logs/s75/v33c_route_check.json` -> `python3 map/scripts/s75_write.py --seg-len 484.0 --loop-len 3355.0`

## v3.3b 대비 바뀐 것
- 유턴 자리: 도서관 길이 남동에서 북동으로 꺾이는 모서리(개신문화관 정면)로 옮김. 현장 설명(11/3): 도서관 지나 개신문화관이 정면에 보이는 곳, 좌회전만 되는 길에서 좌회전하는 척하다 차가 안 오면 유턴함
  - v3.3a·b 의 북문 길 끝 유턴(52799)은 자리를 잘못 잡은 것이라 지움. 그 자리는 CARLA 도로면이 없어 11/3 오전 주행에서 8분 헤맸음
  - 실제 포장 범위는 모름. 그래서 지금 CARLA 도로면(차로 둘 + 양쪽 인도 면, 합쳐 약 11m) 안에서만 잡음
  - 모양: 오른쪽으로 살짝(0.66rad) 벌렸다 왼쪽으로 크게(4.71rad) 돌고 오른쪽으로(0.91rad) 맞춤. 반경 4.1m, 유턴 25.7m + 끝 곧은 8m. 경로 점을 레인렛 centerline 으로 넣음
  - 필요한 조향: 지금 차 0.60rad, 스프린터 0.73rad(최대 0.77 안). 바퀴는 CARLA 도로면 안(여유 0~0.25m), 몸체 모서리는 인도 면 밖으로 조금 나감
  - r1415·r1409 레인렛을 이 자리에서 둘로 나눔(도서관 쪽 조각이 원래 id). 북문 출발 쌍이 각 약 125m 짧아짐
- 사범대 앞 승강장(교대 하차)은 유턴 뒤쪽이 되어 순환에서 빠짐. 승강장 파일에는 남김
- N14 모서리: v3.3b 의 바깥 경계만 넓히기는 가운데선을 CARLA 도로면 밖으로 밀어서 되돌림. 대신 가운데선을 고르고 꺾이는 자리 둘레 20m 에서 왼쪽으로 최대 2.5m 비켜 감. 바퀴가 CARLA 도로면 안에 듦(지금 차·스타렉스·스프린터)

## 순환 노선
- 한 바퀴 약 3,355m, 6곳: 정문 안쪽 > 양성재(이쪽) > 후문 안쪽 > (후문 앞 유턴) > 도서관 남문(건너편) > 도서관 북문 > (모서리 유턴) > 개성재 > (N14 통로) > 정문 안쪽
- 노선 파일의 정류장마다 via(요청 없으면 지나감)와 goal(요청 있으면 섬). 순환 운행 논리는 판단 쪽 몫이라 여기 없음. 그림 map/docs/figs/v33c_loop.png

## 쓰는 법
- CARLA: MAP_COMPOSE=integration/config/s75_v33c_map.override.yaml MAP_PATH=/home/aw/s75_maps/OpenDriveMap 를 붙여 s59_run.sh(한 목표) 또는 s75_run_loop.sh(LOOP_SEGS 로 순환 구간). 사본 폴더 integration/maps/s75_v33c/OpenDriveMap(이름은 OpenDriveMap 이어야 함)
- s75_run_loop.sh 는 회차 목록 11번째 열(예상 시간)의 2배를 넘기거나 바퀴 이탈이 10초 이어지면 그 회차를 끊음
- 플래닝 시뮬레이터: integration/docs/tools/s74_ps_run.sh 에 MAPD=map/maps/v33c

## 차량 바꾸는 법(기본값은 아직 sample_vehicle·프리우스)
- 스프린터: VEHICLE_TYPE=vehicle.mercedes.sprinter VEH_COMPOSE=integration/config/s75_sprinter.override.yaml SMAP=<밴 센서 파일, vehicle_config.wheelbase 3.458> GTARGS='--lidar=-0.229,0.0,2.130'(값이 '-' 로 시작하므로 = 꼴)
- 빼면(인자 없이) 프리우스·sample_vehicle 로 되돌아감
- vehicle_info 근거는 integration/config/s75_vehicle_info_sprinter.param.yaml 머리 주석(CARLA 실측)

## 한계
- 실제 유턴 자리 포장 범위는 모름. 알려 주면 그 범위로 다시 잡을 수 있음
- 후문 앞 유턴은 CARLA 도로면 밖을 조금 지나감(바퀴 검사 19점). 양성재 -> 후문 구간 한 곳(229, 505)도 바퀴가 조금 나감
- 개성재 승강장 앞 r1316 꺾어 드는 곳 차체 0.27~0.55m(v3.3b 그대로)
- 스프린터는 30쌍에서 레인렛 밖으로 몸체가 나가는 곳이 많음(차체 검사 389점 24건, 좁은 꺾임)
- 스프린터는 플래닝 시뮬레이터에서는 순환 6구간을 다 돌았지만 CARLA 에서는 출발을 못 함(0.036m/s). 브리지 가속 지도(autoware_carla_interface accel_map.csv)가 프리우스용이라 3톤 차에 스로틀이 모자람. 스프린터 가속·제동 지도를 CARLA 에서 재서 오버레이로 덮어야 함(다음 할 일)

## 실주행(11/3 오후, CARLA Low, integration/docs/s75_report.md)
- 프리우스: 도서관 북문 -> 모서리 유턴 -> 개성재 149s, 개성재 -> N14 -> 정문 안쪽 136s 도착. 헤맴 없음. 바퀴 한 개가 0.2s 이하로 인도 면 끝을 스치는 정도(19틱)
- 플래닝 시뮬레이터: 스프린터 6구간 도착(113·278·148·134·140·133s), 지금 차 고친 구간 141·127s
- 인지(정지 장면 거리별): 프리우스는 10/25 과 같은 수준. 스프린터 지붕 LiDAR 는 차를 더 멀리(30m 76%) 보지만 자기 사이드미러를 매 틱 객체 두 개로 냄(미러 자르기 값이 프리우스용). 스프린터용 mirror.param.yaml 을 덮어야 함
