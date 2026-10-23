# 맵 frozen_v3.1 동결 기록 (맵세션57, 2026-10-13)

사용자 결정(세션57 지시): v3.1 동결. 도로망은 s54 후보(214507af), Autoware 지도는 28a57711 사본을 정식판으로 씀.
I6-3 에서 p_north_middle 이 도착해 30쌍 30/30 이 된 것이 동결 조건이었음(integration/docs/i6_report.md I6-3).
frozen_v1·v2·v3 와 그 문서는 그대로 둠. v3 결과는 계속 v3 기준임. GitHub 반영 안 함. 초안은 map_frozen_v31.md.bak_s57.
도로망은 v3.1 로 고정하고, 노선 파일(승강장·시범 노선)은 앞으로 판을 따로 올림(아래 '노선 파일' 절).

## 절차 (v3 와 같음)
- 후보를 그대로 두고 바이트 복사본을 새 이름으로 만들었음(cp -p --no-clobber, cmp 같음 확인. mtime 은 원본 것)
  - map/maps/cbnu_campus_v31_candidate_s54.xodr -> map/maps/cbnu_campus_frozen_v31.xodr
  - map/build_s55/nB/{lanelet2_map.osm, map_projector_info.yaml} -> autoware_map/cbnu_campus_v31/
- 해시 목록: map/maps/frozen_v31.sha256 (확인: sha256sum -c map/maps/frozen_v31.sha256)

## 동결 파일 (sha256, 크기 B)
- xodr: map/maps/cbnu_campus_frozen_v31.xodr
  214507afc87c2523f191e1c19154fb44b781eb16930b15bbc965fa2ab37ceb84, 2,045,474 (후보 s54 와 같음)
- Autoware 지도: autoware_map/cbnu_campus_v31/
  - lanelet2_map.osm 28a57711094effaf3e0523cfd103e94a010c2bac01d37feebd0de90438dd02f9, 11,917,366
  - map_projector_info.yaml a6fb6af33e164e61a075f9a79eeffa5eca3ead4435dc3de49e811525175b8307, 22 (projector_type: Local)
  - ~/autoware_data/maps/OpenDriveMap 두 파일과 같음(세션57 cmp). 폴더 이름은 CARLA 맵 이름이라 OpenDriveMap 그대로
- 정류장: map/data/stops_v3.yaml befb9fddffe112add253f3a1ff65e00ba7973b91bb5b97fba0c11272dd14576e, 9,704
  - v3 와 같은 파일을 그대로 씀(바뀐 것 없음). 정류장·승강장 road 11개 중 v3.1 에서 바뀐 것은 r1415 하나이고,
    북문 승강장 s78 은 바뀐 구간(s125~) 밖. 승강장 11곳 위치·방위 차 0(map_session53a·54)
  - 파일 머리의 map_sha256_16 은 v3 값(a17ab012) 그대로임
- 레이어 데이터
  - map/data/buildings_layout_v3.csv ac9bd41d4ef7561f9d3f601a04dd5919729f06cb7f3f3203e9fb1829d5cf3158, 19,487
  - map/scripts/place_campus_layers.py 8b067484ad98c62c801e03ce55dcf985dbda0981c02497f2d0d04f05296c1645, 41,254
  - 배치 결과(dry, 표현 나): map/docs/logs/campus_layers_dry_나_s54_v31_actors.csv
    dcf93ea8e4103baacadda0fd59a544d0ed68ed9e70e752fd32f80d4299852fdb, 100,445
    / campus_layers_dry_나_s54_v31.json 18287ed60591578d7e1ee5178aa9816432099724b01ed9f388cdaaf3ec9e9975, 233,009
    (건물 721·정류장 10, 침범 0. I6 주행은 Plane 포함 732 소환)
- 노선: map/tests/test_drive.py ROUTES_V3S46(5노선 + 30쌍). v3.1 에서 road id 그대로라 같은 표를 씀

## Autoware 지도 만든 법
- 동결 xodr -> autoware_map/scripts/l1_build_lanelet2.py(2251fe0f) --chord-fix map/build_v31_s54/s54_chord_list_no1317_1825.json(d20cafc7)
  -> 빌더 출력 83edb7755588184f(lanelet 704) -> map/scripts/s55_drop_lanelets.py(5dc2e15b) 1318 1188 -> 28a57711(lanelet 702)
- 한 번에 다시 만들고 해시 맞추기: bash map/scripts/s57_make_v31_lanelet2.sh <새 폴더> (세션57 확인: 83edb775·28a57711·a6fb6af3 모두 같음)
- 빌더에 제외 규칙을 넣지 않은 까닭: 빌더 안에서 레인렛을 빼면 끝점 공유 평균을 다시 해 이웃 레인렛 끝점이 바뀜.
  그러면 30쌍을 다시 봐야 함. 그래서 빌더 출력에서 두 road 의 레인렛 블록 줄만 지움(남은 줄 바이트 그대로, 추가 0줄)
- 보정 목록: 세션54 의 16개에서 r1317·r1825 를 뺀 14개(r1173·1208·1351·1454·1456·1461·1589·1653·1704·1827·1852·1871·1921·2038)

## 규칙: Autoware 지도에서 뺀 길 (세션58 문구 고침)
- Autoware 지도에서 N14 주차장 안 통로 r1318·r1188 두 개만 뺌. 까닭: r1318 반경 0 지그재그·r1188 접힘, 노선에 필요 없음(같은 두 점을 잇는 본선이 있음)
- 태그로 일괄 제외하지 않음. 원본 OSM 은 캠퍼스 본선 대부분에도 service=parking_aisle 을 붙였음(247 road 중 30쌍이 85개 사용, map/logs/s57_parking_aisle.log)
- xodr·CARLA 에는 두 road 가 남아 있고 Autoware 지도에만 없음
- (세션57 문구 '셔틀 운행 범위에서 주차장 통로는 뺀다'는 태그 뜻으로 읽힐 수 있어 세션58 사용자 지시로 고쳤음. 옛 문구는 map_frozen_v31.md.bak_s58)

## 로드
- python map/scripts/load_campus_world.py (세션57 부터 기본 v3.1. v3 는 --map v3, v2 는 --map v2, 다른 파일은 --xodr)
- test_drive.py MAP_ROUTES 에 214507afc87c2523 -> ROUTES_V3S46 (세션57)
- 생성 인자는 v3 와 같음(vertex_distance 2.0, max_road_length 50.0, wall_height 0.0, additional_width 0.6, smooth_junctions·mesh_visibility·pedestrian_navigation True)

## 규모
- road 847 / junction 105, topology 간선 1192(v3 와 같음). 새·없어진 road 0. v3 대비 바뀐 road 34, 모두 link·lanes 그대로
- Lanelet2 lanelet 702, 뒤집힘 1(r1589)·끊김 2, 30쌍 경로 30/30

## v3 -> v3.1 변경 이력
- 세션53a(map_session53a_report.md): Autoware 가 못 지나는 곳 전수 조사 뒤 꺾임·반경 고침 26 road(s53 후보 b9cc2369)
  - 목표 차선 중심 반경 4.0m(Autoware 기본 차 최소 회전 반경 3.312m 에 약 20% 여유). 끝점·link·lanes·road ID 그대로
  - 예: r1393 5.6 -> 100.9m, r1972 0 -> 직선, r1391 2.84 -> 4.15m, r1415 1.94 -> 7.43m, r1471 2.47 -> 4.08m, r1317 0 -> 4.08m
  - 승강장 road 는 승강장 s + 2m 뒤에서만 고침(승강장 위치 차 0)
- 세션54(map_session54_report.md): p_yang_back 막힌 곳(r1220·r1825) 고침(s54 후보 214507af, s53 대비 바뀐 road 10, 그중 53a 에 없던 것 8)
  - r1220 + 짝 r1352: 같은 3차 Hermite 기준선, 차선 반경 0 -> 7.73m, 옛 차선 중심에서 최대 3.04m, 띠 밖 0
  - P2·P3 4.0m 이동: r1219·r1351 토막과 j77·j78 연결로 6개(r1913·r1914·r1824·r1825·r1826·r1827). r1825 접힘 -> 4.85m.
    옮긴 자리는 옛 도로 띠 안이지만 OSM 점에서 벗어남. 사용자 결정(세션57): 받음
- 세션55(map_session55_report.md): xodr 그대로. Autoware 지도에서 r1318·r1188 레인렛 뺌(28a57711)
  - 까닭: Lanelet2 거리 비용으로 p_north_middle 이 r1318 쪽(8.1m 짧음)으로 가서 r1318 s75.7 에서 섰음(I6-2).
    xodr 노선은 125도 연결로 r2044 를 회차로 봐서 피했음(110도/150도 두 기준의 틈)

## 30쌍 Autoware 결과 (integration/docs/i6_report.md)
- 설정: 정답 측위·정답 객체, max_real_delta 0.05(실시간 0.998배, 제어 19.96Hz), 레이어 나+정류장+Plane 732
- I6-1(2026-10-11, s53 후보, 7쌍): 도착 6/7. p_yang_back 이 r1220 s11.9 에서 'outside drivable area' 로 섬 -> 세션54 에서 고침
- I6-2(2026-10-12, s54 후보, Autoware 지도 83edb775, 30쌍): 도착 29/30. p_north_middle 이 r1318 s75.7 에서 섬 -> 세션55
  - 시연용 p_main_back 317.0s, p_yang_back 도착(I6-1 실패 풀림)
- I6-3(2026-10-13, Autoware 지도 28a57711, p_north_middle): 도착 271.4s, 정지 -0.453/+0.002m, 옆 0.476m, 바퀴 0, 999.7m
  - 출발 전 Autoware 경로 = xodr 노선 확인. 나머지 29쌍은 28a57711 이 83edb775 에서 두 road 블록만 지운 판이고
    오프라인 경로가 29쌍 모두 같아서 다시 안 돌렸음
- 30쌍 합계: 도착 30/30, 노선 30쌍 모두 xodr 노선과 같음. 후진 0·충돌 0·차 중심 차도 밖 0
  - 0.8m 넘는 옆 벗어남 0쌍, 1초 넘게 이어진 바퀴 이탈 0쌍
  - 0.5m 넘는 옆 벗어남: r1471(p_yang_north 0.566, p_yang_south 0.555). 0.4~0.5m: r1317 s8.6(5쌍), r1415 s155(4쌍)
  - 짧은 바퀴 이탈(0.5s 이하): r1457·r1625·r1881·r1629 (한계 9)
- 오프라인(세션53a·54·55): 차체 검사 Autoware 30쌍 경로 0/30. xodr 35경로(30쌍+5노선)는 아래 한계 1·2

## 알려진 한계 (고치지 않고 동결)
1. 옛 middle 노선(r1360 끝, 정류장을 옮겨 안 쓰는 길): 차체 검사 r1317 2점·r1877 1점(j49, 차선 반경 1.891m).
   30쌍은 안 지남. 사용자 결정(세션57): 고치지 않음
2. 옛 5노선 차체 검사: r1278 s=0(맵 가장자리) 출발이라 첫 3점이 걸렸음. 지도 문제 아님.
   사용자 결정(세션57): 출발을 1.2m 앞에서 하도록 검사 기준을 바꿈. 결과(28a57711, s53_footprint 7098d825): 35경로 5/35 -> 1/35(middle 만), 30쌍 0/30 그대로. map_session57_report.md 6절
3. r1827(j78, 30쌍 밖) 차선 반경 1.28m. 4m 로 하면 옛 도로 띠 밖 0.69m. 사용자 결정(세션57): 그대로 두고 한계로 적음
4. P2·P3 4.0m 이동은 옛 도로 띠 안이지만 OSM 점에서 벗어난 것임(받음, 세션57)
5. N14 주차장 통로 r1318·r1188: xodr·CARLA 에는 남아 있고 Autoware 지도에만 없음. r1318 s77.2·s81.2 반경 0 지그재그(OSM 점 그대로), r1188 s167~169 접힘.
   CARLA 만 쓰는 주행(test_drive.py)은 이 길을 탈 수 있음(30쌍 노선은 안 탐)
6. 요소 경계 반경 0 꺾임 14곳 / 8 road(r1166·r1188·r1189·r1208·r1318·r1339·r1341·r1366). 모두 노선 밖(35경로·Autoware 30쌍 밖, logs/s55_kinks.log).
   반경 조사(s53_survey)는 이것을 못 봄. 사용자 결정(세션57): 고치지 않음
7. r2044(j, r1194 -> r1191, -124.9도): 차체 검사 7점. 30쌍 Autoware 경로 밖(28a57711 기준). 다른 목적지에서 쓰일 수 있음
8. Lanelet2 r1589(j33) 뒤집힘·끊김 2: 30쌍 밖
9. 짧은 바퀴 이탈(0.5s 이하, r1457·r1625·r1881·r1629)은 판정 방식의 한계임. 교차로 안에서 CARLA 위치 조회
   (get_waypoint, project_to_road=False)가 겹친 다른 연결로를 잡음. 바퀴는 차선 안임(map_session55 4절)
10. r1471(j11, -109.7도, 차선 반경 4.08m): 주행 옆 벗어남 0.56m
11. 회차 연결로: xodr 에는 있고 Lanelet2 는 150도 이상만 뺌. 30쌍 xodr 노선은 110도 초과를 회차로 보고 피함(두 기준이 다름)
12. 원본 OSM parking_aisle 태그가 본선에도 붙어 있음. 그래서 뺄 길은 태그가 아니라 하나씩 정함(지금 r1318·r1188 둘, 세션57 2-2·세션58)
13. v3 한계 5~10 그대로: 정문 출구 1차로, 중문 문 위치 추정, 캠퍼스 경계 밖 차선 약 678m, 고도 0·신호 0, 위경도 약 716m 어긋남, 커넥터 겹침 road 조회

## 노선 파일 (1판, 세션57)
- 도로망(v3.1)과 따로 판을 올림. 1판 = 세션56·56b·56c 결과. 바이트 복사본을 판 이름으로 고정했음(cp -p --no-clobber, cmp 같음)
  - map/data/shuttle_platforms_v31_r1.yaml = shuttle_platforms_v31.yaml(56b 고친 판)
    075b224c73008c38955d59467a2d4dee5d7e11ea69bd9711ac3c7c6aa3ed380e, 17,393
  - map/data/routes/pilot_route_r1.yaml = pilot_route.yaml(56c 고친 판)
    a9ca2108c49a45bc86dbdf3d4a0b2fb0c3885f1fb233a3fda92dd7abd6a249fa, 5,369
  - 설명 문서(묶음 밖, 참고): map/docs/shuttle_route_file.md a717ae9ab2e1bb43(56c 고친 판)
- 원래 이름 파일(shuttle_platforms_v31.yaml, pilot_route.yaml)은 맵 터미널이 계속 고칠 수 있음. 2판을 낼 때 _r2 로 복사함
- 2판(세션58): map/data/routes/pilot_route_r2.yaml 06a0cfb159c5c5d9. 1판에서 platforms 참조만 shuttle_platforms_v31_r1.yaml(075b224c) 로 고정하고 해시를 적음. 승강장 파일은 1판 그대로
- 내용
  - 승강장 11곳(lib_south_se 는 정차 불가: 문 구역 없음). 뒷축 목표, 문 위치·정차 여유·문 구역·전방 통로는 가정값
  - 시범 노선: 정문(이쪽) -> 중문(건너편, 통과) -> 도서관 북문(정차) -> 양성재(이쪽). xodr 2105.1m, status ready
    세 구간 I6-2 도착(139.0 + 81.1 + 356.7 = 576.8s), 차체 검사 0점(28a57711)
  - 최종 확장안(양성재 -> 후문 붙임, 3314.0m)은 shuttle_route_file.md '다른 후보'에만 있고 pilot_route 에는 없음
- 지도 해시 표시: pilot_route 는 xodr 214507af·lanelet2 28a57711. shuttle_platforms 는 lanelet2 83edb775 로 적혀 있음
  (28a57711 에서 빠진 r1318·r1188 은 승강장에서 29.0m 이상 떨어져 11곳 값이 같다고 파일 주석과 56b 3절에 적혀 있음). 1판은 그대로 둠
- 알려진 것: 북문 출발 뒤 동쪽 고리 582m 를 돌고 r1321~r1322 를 두 번 지남(Lanelet2 가 r1415 끝 회차 r1703 을 뺐기 때문, 56c 2절). 지도 안 고침

## 되돌리는 법
- CARLA 월드: python map/scripts/load_campus_world.py --map v3 (v3 동결본 a17ab012). 로더 자체는 load_campus_world.py.bak_s57(9cd427a0)
- test_drive.py: v3 xodr 를 주면 그대로 v3 로 돎. 줄을 빼려면 test_drive.py.bak_s57(0ab79fe0)
- Autoware 지도 폴더(~/autoware_data/maps/OpenDriveMap)
  - v3.1 빌더 출력(83edb775): integration/maps/OpenDriveMap_v31s54_bak_i63/ 두 파일을 복사
  - v3(cbnu_campus_v3): autoware_map/cbnu_campus_v3/{lanelet2_map.osm, map_projector_info.yaml} 를 복사
  - 바꾼 뒤 sha256 으로 확인하고 setup_log 에 적음
- CLAUDE.md 지도 줄: CLAUDE.md.bak_s57
- 동결 파일은 지우지 않음. 판을 고르는 것으로 되돌림

## 로그 해시
- 주행 로그 map_sha256=214507afc87c2523, Autoware 지도 lanelet2_map.osm 28a57711094effaf
