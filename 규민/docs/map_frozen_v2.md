# 맵 frozen_v2 동결 기록 (세션40, 2026-10-09)

사용자 결정(세션39, map_session39_report.md 0절): v2 동결 예. 이 문서가 v2 동결 기록이다.
map_frozen_v2_candidate.md(세션31)는 그대로 둔다(그 문서 첫머리: "통과하면 이 문서를 그대로 두고 map_frozen_v2.md 를 새로 쓴다").
frozen_v1(map_frozen_v1.md)도 그대로 둔다. v1 결과는 계속 v1 기준이다.

## 절차 (frozen_v1 과 다른 점)
- frozen_v1(세션19): 파일을 옮기거나 이름을 바꾸지 않고 문서로만 고정(map_frozen_v1.md 3행)
- frozen_v2(세션40): 세션40 지시 "v2 후보는 두고 동결본을 새 이름으로" 에 따라 후보를 그대로 두고 바이트 복사본을 새 이름으로 만들었다
  - cp -p --no-clobber (후보 파일 변경 없음, cmp 동일 확인). mtime 은 후보의 것(2026-10-07 14:16:44 +0900) 그대로
  - 나머지(해시 기록·규모·로드 인자·노선·결과·한계·로그 해시)는 frozen_v1 문서 형식 그대로

## 파일
- 동결본 경로: map/maps/cbnu_campus_frozen_v2.xodr
- 원본(후보, 그대로 둠): map/maps/cbnu_internal_only_localtm_tags81_B_smooth_turnfix.xodr
- sha256: 5240ca8b883e42c02840ae863fc2df47e0f5b7371f714444c8b0b122522ed922 (앞 16자 5240ca8b883e42c0) — 두 파일 같음
- 크기 2,219,959 B
- 만든 과정: map_frozen_v2_candidate.md "파일" 절(세션31). raw OSM -> 태그 보정 81 -> 폴리곤 분류 -> 평활화(세션11 인자)
  -> Osm2Odr(frozen_v1 과 같은 설정) -> s27_turnfix write(회차 커넥터 13 추가)
- test_drive.py 는 --xodr 파일의 sha256 으로 노선표를 고른다(MAP_ROUTES). 동결본·후보 어느 경로를 줘도 같은 ROUTES_V2 가 쓰인다

## 규모
- road 917 / junction 112 / spawn point 1302
  - 출처: road·junction map_session32_report.md 10행(파일 계수)·109행(서버 로드), spawn map_session32_report.md 110행

## 로드 파라미터
- frozen_v1 과 같다(map_frozen_v1.md "로드 파라미터"). 명령: python map/tests/test_load_map_878.py --wall-height 0 --xodr <동결본>
- 세션40 신규 로더 map/scripts/load_campus_world.py 는 같은 인자를 쓴다(인자 대조 map_session40_report.md 4절, 서버 실행 전)

## 노선 정의 (test_drive.py ROUTES_V2, --xodr <v2>, 전부 정문 road1278 lane -1 에서 출발)
- north(도서관 북문): 종점 r1415 s=78.0, 17 road
- south(도서관 남문): 종점 r1268 s=47.0, 15 road
- middle(중문): 종점 r1360 s=70.0, 19 road
- yangseong(양성재): 종점 r1153 s=22.0, 17 road
- back(후문B 추정 게이트): 종점 r1151 s=23.1, 33 road. 노선 전용 설정 r1391 감속 8.9km/h·계획점 간격 1m·겨냥 1.0m(세션35·38)
- road 목록은 test_drive.py ROUTES_V2(184행~)

## 검증된 결과
- 4 노선: 세션33 개정 기준(reconversion_plan.md 9절, 시간은 대기 제외 ±1.0s) 통과
  - 북문 완주 622.4m / 123.2s / 충돌 0 (map_session33_report.md 88행)
  - 남문 완주 726.8m / 143.9s / 충돌 0 (같은 보고서 90행)
  - 중문 완주(정지 근접) 689.9m / 원값 142.1s·대기 제외 137.1s / 충돌 0 (92행)
  - 양성재 3회 633.8~634.0m / 125.5~125.7s / 충돌 0 (66~68행)
- 후문: 3/3 완주 1363.9~1364.0m / 276.0·281.4(정지 근접 5s 포함)·275.5s / 충돌 0 (map_session38_report.md 61~66행)
- 단서: 4 노선은 test_drive.py 2935c016f9315b70(세션38 변경)으로 아직 재주행 안 함(다음 서버 세션). 세션38 변경은 back 노선 전용 필드만이고 dry-plan v1 6·v2 4 변경 0(map_session38_report.md 1-1절)

## v1 -> v2 변경 이력
- 입력 way: 태그 보정 73 -> 81. 추가 8 = 입력 way 9(481950060, 442595850, 481945510, 481943505, 481475019, 452870644, 481945509, 481943503, 392632034)
  에서 정문 출구 481950060 을 뺀 B안 (map_session31_report.md 27행, reconversion_plan.md 7절 A/B)
  - 박물관 차고지 고리 442595850, 후문 연결 6, 후문B 게이트 진입로 392632034(세션28 포함 승인)
- 변환 직후 904 / 99 -> 회차 후처리 13 추가 917 / 112 (map_frozen_v2_candidate.md "규모")
- road id 전부 바뀜. 옛 878 중 형상 일치 이관 866/878(98.6%) (map_session31_report.md 96행, 대응표 logs/s28_road_id_map_B.csv)
- 회차 없는 막다른 끝(정의 없는 junction) 13 -> 0 (map_session31_report.md 116행)
- 도달 913/917, 주 순환망 복귀 802 -> 903, 고립 덩어리 13 -> 1 (map_session31_report.md 121행, map_session32_report.md 19행)
- 정문 왕복 4/878 -> 4/917 (변화 없음, B안 규칙 3 예상값) (map_session32_report.md 16행)
- 후문: v1 에서 갈 수 없음(최근접 217m) -> v2 에서 추정 게이트 도달 가능, 정문에서 약 1,368m (map_session32_report.md 63행)
- spawn 1243 -> 1302 (map_session32_report.md 110행)
- 4 노선 경로 합 차 0.1m 안, dry-plan 차 0.0012m (map_frozen_v2_candidate.md "오프라인 확인")

## 알려진 한계 (고치지 않고 동결)
- 정문 j109 커넥터 r1972(= v1 r1917) 이음매 1.085m·37.9도 / 0.895m·30.9도 그대로 (map_frozen_v2_candidate.md)
- 후문 고리 조각 r1368·1892·1893·1894 정문에서 도달 불가 (map_session32_report.md 63행)
- 정문 왕복 4/917
- r1391: 하네스(BasicAgent)는 노선 전용 설정으로 통과. 맵 결함으로 보지 않음(map_session36_report.md 세션37 결정 2절). 최종 판정은 Autoware 플래너 도입 후
- 고도 0: 동결본 elevation 917개 모두 a·b·c·d 0 (세션40 계수, frozen_v1 도 같음 map_session28_report.md 67행). 교통 규제 미반영: signal 0개(세션40 계수), 회전 제한 relation 변환 입력 0(세션24 s24_offline_checks)
- 커넥터 겹침으로 위치 기반 road 조회 때 겹친 커넥터 번호가 찍힐 수 있다(세션38 r1947·1953·1951 기록)

## 로그 해시
- 로그 첫 줄 map_sha256=5240ca8b883e42c0 가 찍혀야 한다(동결본·후보 같은 값)
