# 맵세션40 보고서 — 인계 1 준비 (2026-10-09, 서버 없음)

결정(확정): v2 동결 예 / 후문 처리 완료(세션38 3/3) / 건물 표현 보류
수행: 1 건너뜀, 2·3·5·6 완료, 4 멈춤(생성 인자 출처 불일치). 서버·xodr 수정·git 없음

## 1. backlog 대응표 — 건너뜀
- docs/backlog_v2.md 없음. ~ 아래 전체(find -iname '*backlog*') 0건
- docs/backlog_status.md 만들지 않음

## 2. v2 동결 (M7) — map/docs/map_frozen_v2.md (신규)
- 절차 비교
  - frozen_v1(세션19): 파일을 옮기거나 이름을 바꾸지 않고 문서로만 고정(map_frozen_v1.md 3행)
  - 이번: 지시 "v2 후보는 두고 동결본을 새 이름으로" -> 후보를 cp -p --no-clobber 로 복사. 문서 형식은 frozen_v1 그대로
- 동결본: map/maps/cbnu_campus_frozen_v2.xodr, sha256 5240ca8b883e42c02840ae863fc2df47e0f5b7371f714444c8b0b122522ed922, 2,219,959 B
  - 후보 cbnu_internal_only_localtm_tags81_B_smooth_turnfix.xodr 와 cmp 동일. 후보 변경 없음
  - test_drive.py 는 파일 해시로 노선표를 고르므로(MAP_ROUTES) 동결본 경로를 줘도 ROUTES_V2
- 규모: road 917 / junction 112 / spawn 1302 (map_session32_report.md 10·109·110행)
- 고도: elevation 917개 모두 a·b·c·d 0. signal 0개 (세션40 계수)
- 노선별 결과
  - 4 노선(세션33 개정 기준): 북문 622.4m/123.2s, 남문 726.8m/143.9s, 중문 689.9m/대기 제외 137.1s, 양성재 633.8~634.0m/125.5~125.7s, 충돌 0
  - 후문(세션38): 3/3, 1363.9~1364.0m, 276.0/281.4/275.5s, 충돌 0
  - 단서: 4 노선은 test_drive.py 2935c016 로 아직 재주행 안 함(다음 서버 세션)
- v1 -> v2 변경 이력: map_frozen_v2.md "v1 -> v2 변경 이력" 절(입력 way 73->81, 904/99 -> 917/112, id 이관 866/878, 막다른 끝 13->0, 도달 913/917, 정문 왕복 4/878->4/917, 후문 도달 가능, spawn 1243->1302)
  - 정정(작성 중 발견): 처음에 B안 추가 8 을 "박물관 고리 + 후문 연결 6" 으로 적었다가 고침. 실제는 9 way 에서 정문 출구 481950060 을 뺀 8 이라 392632034 포함(map_session31_report.md 27행)
  - 참고: reconversion_plan.md 9~10행의 "추가 8" 목록(481950060 포함)은 세션27 값이다(같은 문서 181행이 밝힘)
- 숫자 카드: presentation_numbers_locked.md 끝에 "v2 동결 숫자 카드" 절 append(위 1~8절 v1 값 그대로). 항목마다 출처
- sync_to_repo.sh: MAPS=(cbnu_internal_only_localtm_tags73_smooth.xodr NOTICE.md) 그대로 둠. cbnu_campus_frozen_v2.xodr 추가 필요(고치지 않음)

## 3. 정류장·차고지 (M5) — map/data/stops_v2.yaml (신규), scripts/s40_stops.py, logs/s40_stops.log
- 기존 노선 파일 형식 없음(노선은 test_drive.py 안 파이썬 dict) -> 지시대로 yaml
- 정류장 6: 노선 정의(test_drive.py ROUTES_V2)에서 모음
  - 정문 r1278 s=0.0 lane -1 (36.6326490, 127.4529988) 캠퍼스 경계 밖 — s=0.00 은 logs/s38_v2_back_run1.log spawn 줄
  - 도서관 북문 r1415 s=78.0 (36.6290107, 127.4579117)
  - 도서관 남문 r1268 s=47.0 (36.6279306, 127.4569369)
  - 중문 r1360 s=70.0 (36.6317018, 127.4580144)
  - 양성재 r1153 s=22.0 (36.6278026, 127.4529621)
  - 후문 r1151 s=23.1 (36.6248296, 127.4628555) — 세션38 종점. 기록된 추정 게이트(36.624844, 127.462860)와 약 1.6m(기록 1.65m 와 맞음)
- 위경도: s22_buildings.Frame(xodr 머리말 투영 + offset) 역산. 왕복 차 0.000m
  - 발견: carla.Map.transform_to_geolocation 은 약 716m 어긋남(첫 실행 값, 로그는 덮어써짐). xodr offset 을 반영 안 하는 것으로 보인다 [추정]. README 에 쓰지 말라고 적음
- 경계 판정: data/processed/cbnu_relation_polygon.json(classify_internal.py 와 같은 함수). 정문만 밖
- 차고지 후보(고르지 않음)
  - 박물관 버스 차고지: OSM way 442595850 노드 8 평균 (36.6276833, 127.4550105). 진입 j4 답사점 36.627598, 127.454728 함께 적음
  - N14 주차장: 좌표 기록 없음 -> 빈 값. 근거 문서(field_survey_2026-10-04.md)는 "건물을 ㄷ자로 감싼 형태" 만 적었다
- 정차 구역 길이: 빈 값

## 4. 캠퍼스 월드 로더 — 멈춤
- 지시: "생성 인자는 test_drive.py 에서 읽어 옮긴다 ... 두 파일의 생성 인자가 다르면 멈추고 보고"
- 확인: test_drive.py 에는 생성 인자가 없다. OpendriveGenerationParameters·generate_opendrive_world·wall_height 0건.
  test_drive.py 는 이미 올라간 월드에 접속만 한다(980행 world.get_map())
- test_load_map_878.py: 7개 인자를 CARLA 기본값에서 시작해 인자로 바꾼다. 실제 적용값(logs/s38_v2_back_run1_load.log):
  vertex_distance 2.0 / max_road_length 50.0 / wall_height 0.0(인자) / additional_width 0.6 / smooth_junctions True / enable_mesh_visibility True / enable_pedestrian_navigation True
- 그래서 load_campus_world.py 는 만들지 않았다. 사용자 결정 필요: 생성 인자 출처를 test_load_map_878.py(+ 로드 로그 적용값)로 해도 되는가
- README 3절은 지금 쓰는 명령(test_load_map_878.py --wall-height 0 --xodr 동결본)으로 적고, 로더는 "아직 없음" 으로 표시

## 5. 인계 1 README — docs/handoff_1_README.md (신규)
- 준비물, 서버 띄우기, 월드 띄우기(현행 명령), 정류장 파일 읽는 법, 알려진 한계(근거 문서 붙임), 하지 말 것, 변경 규칙
- 한계 항목 중 확인 못 한 것은 그렇게 적음
  - 브리지 기본 차 prius: 브리지 설정 파일을 이 PC 에서 못 찾음(세션38) -> "알려져 있으나 확인 안 함"
  - Rosa 공개 길이 6,245~7,730mm: 세션40 지시문 제공 값, 원문 확인 안 함
- 평가어 쓰지 않음
- 위치 주의: docs/ (작업 폴더 최상위)는 sync_to_repo.sh 대상이 아니다(대상은 map/docs·scripts·tests·maps·README). map/data/stops_v2.yaml 도 대상 밖. 팀원에게 가려면 위치 이동이나 sync 대상 추가가 필요

## 6. 정정·확인 기록
### a. r1391 반경 정정 — map_session35_report.md 끝에 "정정 2" append
- 2.483m 는 s=9.00 이음 기하 값(0.06m 구간), 끝 구간 최소는 3.114m(s=17.67)
- D 근거 재계산 [검산 일치]: 20km/h 끝 9.91m/s^2(1.01g), 2.5m/s^2 기준 끝 10.0km/h. 현재 8.9km/h(= 2.483m 기준 8.97 내림)는 더 보수적, 유지
- 지시문 숫자 그대로 맞음. 고친 것 없음

### b. 건물 초안 확인 — scripts/s40_building_check.py, logs/s40_building_check.log (판정 없음)
- 18동인 이유: s39_building_draft.py 는 s22_buildings.py 의 KEY_NAMES(세션22 발표·배치 대상 이름 13개 패턴)로 골랐다. 세션25 1순위 30(logs/s25_buildings.csv tier=1)을 쓰지 않았다
  - 둘 다 9, s39 에만 9(개신문화관·구학생회관·등용관·법학관·신민관·신학생회관·양현재 관리동·중앙도서관 신관·청운관), s25 1순위에만 21(이름 없음 5 포함)
- 겹침 7동. 깊이 = 겹친 점에서 그 반쪽 도로 바깥 끝(인도 바깥 가장자리)까지 거리의 최댓값. 이 값만큼 물리면 그 점이 도로 밖
  - 경영대학 본관: 깊이 5.68m(주행 5.68, 인도 2.75). r1189·1315·1644. 노선 차선: r1315 lane -1·-2, r1644 lane -2 (중문). 점 폴리곤 142 / 사각형 571 / 2분할 468 / 3분할 364
  - 등용관: 4.82m(주행 4.82, 인도 2.79). r1244. 노선 아님. 49 / 81 / 80 / 58
  - 법학관: 2.71m(인도만). r1196·1198. 노선 아님. 42 / 187 / 172 / 171
  - 사회과학대학 본관: 5.14m(주행 5.14, 인도 2.78). r1242·1941·1942. 노선 아님. 118 / 270 / 210 / 275
  - 양현재 관리동: 1.60m(인도만). r1814. 노선 아님. 15 / 152 / 152 / 154
  - 인문사회관: 0.29m(인도만). r1188. 노선 아님. 31 / 116 / 110 / 110
  - 지선관: 2.53m(인도만). r1794. 노선 아님. 21 / 24 / 23 / 23
- 폴리곤이면 사라지나: 아니다. s39 초안이 이미 폴리곤 기준이고 7동 모두 폴리곤에서도 겹친다. 사각형은 7동 모두 겹친 점이 더 많다
- 사각형 2~3개로 나누면 사라지나: 7동 모두 사라지지 않는다(2분할·3분할 모두 겹친 점 > 0, 폴리곤보다 많다)
  - 나누기 방식: 외접 사각형 긴 축을 같은 폭 2·3 조각으로 자르고, 조각마다 그 안의 폴리곤 부분을 감싼 사각형
- 노선 차선과 겹치는 건물은 경영대학 본관 1동(중문 r1315 주행 차선 포함). 세션25 보고서 "중문 궤적이 N13 폴리곤 안을 지남" 과 같은 위치
- 빼기 / 물리기 / 나누기는 사용자가 정한다

## 7. 다음 서버 세션 첫 단계 (적어 두기만)
1. load_campus_world.py 첫 실행 (4절 결정 뒤 작성)
2. 4 노선 1회씩 재주행 (test_drive.py 2935c016)
3. s39_vehicle_raw.py 로 fusorosa 바퀴 원값

## 8. 마감
- 해시: frozen_v1 bf835cdfad0cea65 / v2 후보 5240ca8b883e42c0 / 동결본 5240ca8b883e42c0 / test_drive.py 2935c016f9315b70 / test_load_map_878.py b76cd1143f8c3666 (수정 없음)
- 인계 1 파일 목록
  - docs/handoff_1_README.md (sync 대상 밖)
  - map/maps/cbnu_campus_frozen_v2.xodr (sync MAPS 에 없음)
  - map/maps/NOTICE.md (sync 대상)
  - map/data/stops_v2.yaml (sync 대상 밖)
  - map/docs/map_frozen_v2.md, map/docs/presentation_numbers_locked.md (sync 대상)
  - map/tests/test_load_map_878.py (sync 대상, 현행 로더)
  - (보류) map/scripts/load_campus_world.py
- sync_to_repo.sh 미리보기(--dry-run, logs/s40_sync_preview.log): docs 261(신규 248), scripts 42(신규 41), tests 1, maps 0, README 0. --apply 안 함
  - 작업 폴더가 앞선 쪽: field_survey_2026-10-06.md 는 작업 폴더에 세션28 절(108행~)이 더 있다. 저장소 쪽 직접 편집 흔적은 이 확인 범위에서 못 봄
- 새 파일: map/docs/map_frozen_v2.md·map_session40_report.md, map/maps/cbnu_campus_frozen_v2.xodr, map/data/stops_v2.yaml,
  map/scripts/s40_stops.py·s40_building_check.py, map/docs/logs/s40_stops.log·s40_building_check.log·s40_sync_preview.log, docs/handoff_1_README.md
- append: map/docs/presentation_numbers_locked.md(v2 카드), map/docs/map_session35_report.md(정정 2), docs/setup_log.md
- 서버·xodr 수정·재변환·git·push·--apply 없음
- 다음 시작점: 4절 로더 인자 출처 결정 -> load_campus_world.py 작성 -> 서버 세션(로더 첫 실행, 4 노선 회귀, fusorosa 원값)

---

# 세션40 추가 (2026-10-09, 사용자 4번 결정 뒤. 서버·git·--apply 없음)

## 4'. 캠퍼스 월드 로더 — map/scripts/load_campus_world.py (신규, 0abbe948ee192947)
- 사용자 결정: 생성 인자 출처 = tests/test_load_map_878.py. 세션38 로드 로그 적용값과 대조해 같으면 진행
- 대조 [확인]: test_load_map_878.py 방식(CARLA OpendriveGenerationParameters() 기본값 + wall_height 0.0)을 이 venv(carla 0.9.15)에서 만든 7개 값
  = logs/s38_v2_back_run1_load.log 적용값 7개. diff 0 -> 진행
- 로더 내용
  - 생성 인자 7개를 전부 값으로 적음: vertex_distance 2.0, max_road_length 50.0, wall_height 0.0, additional_width 0.6, smooth_junctions True, enable_mesh_visibility True, enable_pedestrian_navigation True
  - 인자: --xodr(기본 maps/cbnu_campus_frozen_v2.xodr, 스크립트 위치 기준 상대), --host(localhost), --port(2000). 절대 경로 없음(주석의 서버 명령 ~/carla 만)
  - 시작 때 xodr sha256 앞 16자 출력, 5240ca8b883e42c0 로 시작하지 않으면 경고
  - 출력: 맵 이름, 서버 to_opendrive 해시 일치 여부, 도로·교차로 수, 스폰 수. timeout 30s·reset_settings=True 는 test_load_map_878.py 와 같음
- 검사: py_compile 통과, --help 동작, GEN_PARAMS 7개를 AST 로 읽어 로그 값과 다시 대조 7/7 일치
- test_load_map_878.py 수정 없음(b76cd1143f8c3666). 첫 실행은 다음 서버 세션. README 3절은 그대로(첫 실행 통과 뒤 바꿈)

## A. 건물 초안 다시 (B2) — scripts/s40_b2_cut.py, data/processed/s40_b2_cut.csv, logs/s40_b2_cut.log, figures/s40_b2_cut_grid.png
- 대상: 세션25 1순위 30동(logs/s25_buildings.csv tier=1, 전부 way). 세션22 이름 목록 아님
- 방법: OSM 폴리곤 그대로(CARLA 좌표, s22 Frame). 깎는 영역 = 동결본 917 road 의 차선+인도 띠(s34 analyze 표본) 합집합을 0.5m 넓힌 것
  - 띠: t = laneOffset ~ laneOffset - (lane -1 폭 + lane -2 폭). 인도 없는 road 385 는 차선만(차선 구성 계수 (-2,-1,0) 532 / (-1,0) 385)
  - 도로 띠 합집합 133,208m2, 깎는 영역 143,830m2
- 노선 lane -1 띠(5 노선 65 road)와 원래 폴리곤이 겹치는 동(맨 위)
  - (이름 없음) way765358663: 깎임 35.6%(212.8 -> 137.0m2), 깎은 뒤 2 조각, 노선 차선 겹침 17.03m2
  - 경영대학 본관 (N13) way446440341: 깎임 7.1%(1920.8 -> 1785.2m2), 1 조각, 노선 차선 겹침 8.99m2
- 그 밖 깎인 동(비율 큰 순)
  - (이름 없음) way471400965 42.5% / 농업생명과학대학 강의동 6.4% / 실험동 5.8% / 사회과학대학 본관 5.1% / 진리관 4.9%
  - 지선관 1.9% / 미술관 1.6% / 인문사회관 0.9% / (이름 없음) way442760317 0.7% / 개성재 관리동 0.1%
- 깎임 0: 18동 / 30
- 깎인 비율 50% 초과: 0동(최대 42.5%)
- 그림: figures/s40_b2_cut_grid.png 30칸. 빨간 선 = 원래, 파랑 = 깎은 뒤, 회색 = 도로(차선+인도, +0.5m 전). 노선 겹침 동 제목에 [노선]
- 참고: 세션39 초안의 겹침 7동 중 1순위 30에 든 것은 경영대학 본관·사회과학대학 본관·인문사회관·지선관 4동. 등용관·법학관·양현재 관리동은 1순위 밖
- 판정 없음. 빼기 / 물리기 / 나누기 / 깎기 채택은 사용자 결정

## B. sync 대상 안 (보고만, 스크립트·파일 그대로)
- 지금 sync_to_repo.sh: SRC=…/campus_mobility_sim/map, 대상 docs·scripts·tests·maps(MAPS 목록)·README
- 세 파일 상태와 안
  1. 동결본 xodr(2,219,959 B): MAPS=(cbnu_internal_only_localtm_tags73_smooth.xodr NOTICE.md) 에 cbnu_campus_frozen_v2.xodr 를 더한다.
     v1 은 문서들이 계속 가리키므로 빼지 않는 안
  2. map/data/stops_v2.yaml(3,790 B): data 는 지금 대상이 아니다. data/ 전체는 OSM 원본·중간 파일이 있어 넣지 않고,
     MAPS 처럼 DATA=(stops_v2.yaml) 목록을 두고 "$SRC/data/$d" -> "$DST/data/" 로 보내는 절을 하나 추가하는 안
  3. docs/handoff_1_README.md(6,118 B): SRC(map/) 밖(작업 폴더 최상위 docs/)이다. 옮기지 않는 조건이면
     ROOT=…/campus_mobility_sim 를 하나 더 두고 HANDOFF=(handoff_1_README.md) 를 "$ROOT/docs/$h" -> "$DST/docs/" 로 보내는 절 추가.
     저장소 docs/ 에 같은 이름 파일 없음(확인). 저장소 최상위에 두려면 목적지만 "$DST/" 로
- 같이 고칠 곳: 토큰 검사 grep 대상에 새 원본 경로(data/stops_v2.yaml, 최상위 docs/handoff_1_README.md)를 넣어야 검사 범위가 맞는다
- 동기화 뒤 README 안 경로 문구 "map/data/stops_v2.yaml(작업 폴더 경로, 동기화 대상 아님)" 은 저장소 경로 data/stops_v2.yaml 로 바꿔야 한다(README 수정은 로더 첫 실행 뒤 함께)
- 승인 전 수정 없음

## C. 위경도 716m 어긋남 (보고만) — logs/s40_geo_check.log
- 원인 [확인, 소스 + 수치]: 투영 기준점(lat_0/lon_0)은 같다. CARLA 가 xodr 머리말 <offset> 을 읽지 않는다
  - xodr 머리말: +proj=tmerc +lat_0=36.627298 +lon_0=127.456394 +ellps=WGS84, <offset x="521.51" y="493.61"> (크기 718.07m)
  - 프로젝트 좌표 사슬(s22 Frame): 위경도 -> tmerc(lat_0, lon_0) -> + offset -> xodr -> CARLA(x, -y)
  - CARLA 0.9.15 GeoReferenceParser.cpp: geoReference 문자열에서 +lat_0, +lon_0 만 읽는다. <offset> 은 읽지 않는다
    https://github.com/carla-simulator/carla/blob/0.9.15/LibCarla/source/carla/opendrive/parser/GeoReferenceParser.cpp
  - CARLA GeoLocation.cpp Transform: 기준점에서 (x, -y) 미터를 위도 스케일 메르카토르로 더한다(tmerc 아님, offset 없음)
    https://github.com/carla-simulator/carla/blob/0.9.15/LibCarla/source/carla/geom/GeoLocation.cpp
  - 수치: 위 공식을 재현하면 transform_to_geolocation 과 6지점 모두 0.000m. 사슬 값과 차 716.44~719.18m(≈ offset 크기 718.07m)
    offset 을 빼고 같은 공식이면 남는 차 0.228~1.918m(메르카토르 근사 vs tmerc 차이)
- GNSS 센서: 같은 기준을 쓴다 [확인, 소스]
  - GnssSensor.cpp: CurrentGeoReference = episode->GetGeoReference(), CurrentGeoReference.Transform(Location)
    https://github.com/carla-simulator/carla/blob/0.9.15/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/GnssSensor.cpp
  - CarlaGameModeBase.cpp ParseOpenDrive(): Episode->MapGeoReference = Map->GetGeoReference() (위 파서 결과)
  - 그래서 이 맵에서 CARLA GNSS 위경도는 실제(OSM) 위치와 약 718m 어긋난다 [추정: 서버에서 GNSS 값 직접 확인은 안 함]
- 인계 2 측위 영향(한 줄): GNSS 위경도를 실좌표·외부 지도와 맞춰 쓰면 약 718m(+최대 1.9m) 어긋나므로, 측위 원점을 offset 반영 기준으로 맞추는 처리가 필요하다(CARLA 맵 좌표만 쓰는 측위면 영향 없음)

## 추가 마감
- 해시: frozen_v1 bf835cdfad0cea65 / v2 후보 5240ca8b883e42c0 / 동결본 5240ca8b883e42c0 / test_load_map_878.py b76cd1143f8c3666 / load_campus_world.py 0abbe948ee192947
- 새 파일: map/scripts/load_campus_world.py·s40_b2_cut.py, map/data/processed/s40_b2_cut.csv, map/docs/figures/s40_b2_cut_grid.png, map/docs/logs/s40_b2_cut.log·s40_geo_check.log
- 서버·xodr 수정·git·--apply 없음. sync_to_repo.sh·README 수정 없음
- 다음 시작점: sync 대상 안 승인 여부 -> 서버 세션(load_campus_world.py 첫 실행, 4 노선 회귀, fusorosa 원값, 가능하면 GNSS 값 1회 확인)
