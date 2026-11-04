# 지도 재현: OSM -> xodr -> Lanelet2 -> 검사

캠퍼스 지도를 OSM 원본에서 다시 만드는 순서임. 단계마다 스크립트·입력·출력·해시를 적음.
2026-10-26에 빈 임시 HOME 에서 아래 순서를 그대로 돌려 28개 출력이 모두 동결본과 바이트까지 같았음.
경로는 작업 폴더(`~/campus_mobility_sim`) 기준. 해시는 sha256 앞 16자.

## 한 번에 돌리기

    bash map/scripts/s71_reproduce.sh <빈 임시 HOME>

- 작업 폴더는 안 건드림. `<임시 HOME>/campus_mobility_sim` 에 스크립트와 입력만 복사해 그 안에서 돌리고, 출력마다 작업 폴더 동결본과 cmp 로 견줌
- 필요한 것: `.venv-carla`(carla 0.9.15, numpy, shapely, matplotlib). carla 패키지 안 Osm2Odr 로 변환하므로 CARLA 서버는 필요 없음
- 걸린 시간 약 26분(이 노트북). 대부분 3단계 s53(꺾임 고침 탐색, 약 20분)
- 기록 integration/logs/s71/step2/reproduce.log. 줄마다 '같음/다름 <파일> <해시>'

Osm2Odr 출력 xodr 에는 생성 시각이 두 곳(3행 주석, header 의 date) 들어감. 그래서 변환 직후 xodr 은 이 두 곳을 뺀 내용이 같은지 먼저 보고, 두 곳을 동결본 값으로 바꿔 넣은 뒤 바이트로 견줌. 그 뒤 단계는 바꿔 넣은 파일을 이어 씀.

## 단계

| 단계 | 스크립트 | 입력 | 출력 | 해시 |
|---|---|---|---|---|
| 0 내려받기 | download_osm.sh | OSM API 0.6 bbox(127.446624, 36.619902, 127.466839, 36.636427) | data/raw/cbnu_campus.osm | c383870eb4d1e196(2026-09-15 받은 것) |
| 1-0 v1 기준선 | s27_reconvert.py run --base | 원본 OSM, 캠퍼스 경계 폴리곤 | 태그 보정 73 OSM -> 경계 안 OSM -> 평활화 OSM -> xodr | dfcc5ff7·ce457630·caa2c69b·bf835cdf(frozen_v1) |
| 1-1 v2 변환 | s27_reconvert.py run --variant B | 같음 | tags81_B 세 OSM -> xodr | a60f88de·a4ed7872·f682fd34·d01e5f86 |
| 1-2 A안 | s27_reconvert.py run --variant A | 같음 | tags82_A xodr(정문 출구 형상만 다음 단계가 읽음) | a0b2d75b |
| 1-3 회차 후처리 | s27_turnfix.py write | tags81_B xodr | v2(cbnu_campus_frozen_v2.xodr) | 5240ca8b |
| 2 v3 | s43_build_v3.py -> s45_build_v3.py -> s46_build_v3.py | v2, A안 xodr, 원본 OSM, s42_way_dir.csv | s43·s45 후보 -> v3(frozen_v3) | dae3f058·d9b9c504·a17ab012 |
| 3 v3.1 | s53_build_v31.py -> s54_build.py | v3 | s53 후보 -> v3.1(frozen_v31) | b9cc2369·214507af |
| 4 Lanelet2 | s57_make_v31_lanelet2.sh(l1_build_lanelet2.py --chord-fix, s55_drop_lanelets.py 1318 1188) | v3.1, 보정 목록 14 | 빌더 출력 -> v3.1 Lanelet2, map_projector_info.yaml | 83edb775·28a57711·a6fb6af3 |
| 5 v3.2 | s63_v32_candidate.py -> s66_make_v32ab.py | 28a57711, crossings_v3_candidates.csv | v3.2 후보 = v32a, v32b | 4719e5e5·485429206e986154 |
| 6 정류장 | s48_stops.py | stops_v3_draft_s46.yaml | stops_v3.yaml | befb9fdd |
| 6 건물 1순위 | s49_b2_v3.py | 원본 OSM, v2·v3, s25_buildings.csv, s40_b2_cut.csv | buildings_layout_v3.csv | ac9bd41d |
| 6 건물 2순위 | s66_b2_tier2.py | 원본 OSM, v3.1, s25_buildings.csv, 1순위 | buildings_layout_tier2_s66.csv | 4cb01976 |
| 6 나무 | s66_trees.py | 원본 OSM, v3.1, 건물 둘 | trees_layout_s66.csv | 0c60976b |

스크립트는 모두 map/scripts/(Lanelet2 빌더만 autoware_map/scripts/). 각 스크립트 머리 주석에 방법과 근거 세션이 있음.
노선표(map/tests/test_drive.py)를 s53_geom·place_campus_layers 가 불러 씀.

## 검사 (만든 뒤 무엇을 봤나)

- xodr: CARLA 월드로 올려 해시 확인(load_campus_world.py, 서버 to_opendrive 해시 = 파일 해시), 30쌍 Autoware 주행 30/30 도착(I6-2·I6-3), 차체 검사 30쌍 0점(s53_footprint.py)
- Lanelet2: Autoware 1.9.0 이미지 로더로 읽기 오류 0, 30쌍 Lanelet2 최단 경로 = xodr 노선(l1_eval.py, s63_route_check.py·s66_route_check.py)
- v3.2: 30쌍 경로가 v3.1 과 같음(30/30), Autoware 확장 로더 읽기 오류 0
- 정류장·대체 정차점: 차체 lanelet 밖 0, 들어오는 26쌍 경로가 원래 경로 앞부분과 같음(s70_alt_route_check.py)
- 레이어: 건물·나무가 노선 차선에 안 걸림(배치 스크립트가 셈, dry-run 로그)
- 근거 문서: map/docs/map_frozen_v31.md, map/docs/v32_candidate.md, map/docs/shuttle_route_file.md

## 스크립트로 다시 안 만든 것 (손으로 정했거나 그때 받은 것)

스크립트가 아니라 사람 판단이나 그때의 외부 자료로 정해진 입력임. 재현은 이 값들을 그대로 넣어야 같은 결과가 나옴.

1. OSM 원본(2026-09-15 받은 것). download_osm.sh 를 지금 돌리면 그 뒤 OSM 편집이 들어가 다른 파일이 나옴. 그래서 받은 파일을 넣음
2. 캠퍼스 경계 폴리곤(data/processed/cbnu_relation_polygon.json, c44ea70a). Nominatim relation 6705106 을 그때 받은 것
3. 태그 보정 way 목록(fix_tags.py 73개 + s27_reconvert.py 추가 9개, B안은 정문 출구 481950060 뺌). 길이·노선 기준으로 고르고 정함(09/18~10/07). 스크립트 안 목록
4. 곡선 평활화 인자(smooth_osm_curves.py 기본값, 09/24)
5. v2 -> v3 에서 지운 way 7개(s43_build_v3.py DEL_WAYS). 검토 시트(map/docs/s42_review_sheet.csv)에 답한 것. 그 판단에 쓴 way 방향표 s42_way_dir.csv(s42_diag.py 출력)는 그대로 넣음
6. 북문 고리 길 경로와 끝 간격 14·10m(s45), 꺾임 9곳 목록(s46), 고칠 road 26 목록과 목표 반경 4.0m(s53), r1220·r1825 둘레와 이동 4.0m(s54). 앞 단계 조사 결과를 보고 정해 스크립트에 넣은 값. 이동 4.0m 은 OSM 점에서 벗어나지만 옛 도로 띠 안이라 받음(10/13)
7. Lanelet2 보정 목록 14 road(map/build_v31_s54/s54_chord_list_no1317_1825.json, d20cafc7). 10/11 비교로 고름
8. Autoware 지도에서 뺀 r1318·r1188(s57_make_v31_lanelet2.sh 인자). 결정(10/14)
9. 횡단보도·방지턱 keep 칸(map/data/crossings_v3_candidates.csv, 2596f8a3). 로드뷰를 보고 적음. v32a 는 X30·X44 포함(10/23 결정), v32b 는 뺌. 크기(횡단보도 폭 4.0m, 방지턱 길이 3.6m·높이 0.10m)는 가정값
10. 정류장 자리(stops_v3_draft_s46.yaml, b3398f20). 중문 후보 1(답, s46_stops.py), 그 앞 s45 초안. 이번 재현은 s48 정리 단계부터만 다시 돌림
11. 건물 1순위 30동·2순위 11동 고르기(s25_buildings.csv 의 tier, 노선 거리 기준)와 깎기 기준(s40_b2_cut.csv), 층 높이 4.0m·층수 없으면 4층 가정
12. 승강장·노선 파일(shuttle_platforms_v31_r1·r2, pilot_route_r1~r3). s56_shuttle.py·s56_write.py·s70_write.py 로 만들었고 문 위치·정차 여유·문 구역은 가정값. 1판은 56b·56c 에서 고친 판이라 이번 재현에 넣지 않았음
13. 장식 레이어 배치 결과(campus_layers_dry_*). 배치 스크립트 dry-run 출력이라 CARLA 메시 크기(mesh_sizes.json, 서버에서 잼)에 따라 바뀜. 이번 재현에 넣지 않았음

## 재현 범위 밖

- CARLA 월드 메시: CARLA 가 xodr 로 만듦(load_campus_world.py 인자 고정). 메시 파일은 저장 안 함
- 측정·주행 결과(30쌍, 인지 평가): 서버가 필요하고 같은 숫자가 나오는지는 기록마다 따로 적었음

데이터 © OpenStreetMap contributors, ODbL 1.0. NOTICE.md 를 봄.
