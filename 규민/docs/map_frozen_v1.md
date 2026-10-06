# 맵 frozen_v1 동결 기록 (세션19, 2026-09-30)

이 문서는 "지금까지 검증한 맵"을 한 장으로 고정한다. 파일은 옮기거나 이름을 바꾸지 않았다.
재변환(후문·중문·양성재 way 추가)을 하면 road 번호가 878개 전부 바뀌므로(s18_D_reconvert.md),
그 전의 결과는 전부 이 frozen_v1 기준이다. 앞으로 두 맵을 병행하므로 모든 로그 첫 줄에
맵 해시를 찍는다(아래 "로그 해시").

## 파일
- 경로: map/maps/cbnu_internal_only_localtm_tags73_smooth.xodr (이름 그대로)
- sha256: bf835cdfad0cea65b813e59bda2fd268edfbbfde1069dd5d217ae119d0b1cc9a (앞 16자 bf835cdfad0cea65)
- 크기 2,125,011 B, mtime 2026-09-24 01:43:06 +0900
- 만든 과정: OSM(cbnu_campus.osm) -> 태그 보정 73건(service 등 -> unclassified) -> 캠퍼스 폴리곤 내부 분류
  -> 곡선 평활화(세션11) -> CARLA Osm2Odr 변환. 파이프라인은 결정적(같은 입력이면 같은 출력, 세션18 D 확인)

## 규모
- road 878 / junction 95 / spawn point 1243 (세션18 로드 로그 session18_load.log)
- 모든 road 는 "반쪽 도로": 오른쪽 주행 차선 1개(lane -1, 폭 3.35m) + 인도(lane -2, 2.80m).
  양방향 도로 하나가 서로 반대 방향인 단방향 road 두 개로 나뉘어 있다

## 로드 파라미터 (carla.OpendriveGenerationParameters)
- 명령: python map/tests/test_load_map_878.py --wall-height 0
- vertex_distance=2.0 / max_road_length=50.0 / wall_height=0.0(인자로 변경) / additional_width=0.6
  smooth_junctions=True / enable_mesh_visibility=True / enable_pedestrian_navigation=True
- wall_height 0 인 이유: CARLA 는 가장 바깥 차선 가장자리에 1m 벽을 세우는데, 반쪽 도로 구조에서는 그 가장자리가
  중앙선이라 벽이 중앙선 위에 선다(세션12 확인, 같은 파일 대조 충돌 2,139건(벽 1.0) -> 0건(세션13, 벽 0). 1,604건은 평활화 전 파일(세션10) 값 [세션26 정정]). 발표 자료 3-2 참조
- 서버: cd ~/carla/CARLA_0.9.15 && ./CarlaUE4.sh -quality-level=Low -win

## 루트 정의 (test_drive.py --route, 전부 정문 road1247 lane -1 에서 출발)
- north(도서관 북문): 종점 road1378 s=78.0, 17 road
  1247,1914,1356,1917,1355,1447,1373,1673,1374,1591,1375,1689,1376,1625,1377,1640,1378
- south(도서관 남문): 종점 road1237 s=47.0, 15 road
  1247,1914,1356,1917,1355,1446,1172,1428,1240,1695,1239,1426,1238,1563,1237
- middle(중문): 종점 road1327 s=70.0, 19 road
  1247,1914,1356,1917,1355,1447,1373,1673,1374,1592,1283,1617,1284,1606,1285,1597,1286,1838,1327
- legacy(세션5~17 경로, 도서관 서쪽 모서리): 종점 road1278 s=100.38, 완주 판정 s>=95, 15 road
  1247,1914,1356,1917,1355,1446,1172,1428,1240,1695,1239,1426,1238,1564,1278
- 루트 체인은 lane -1 의 next() 로만 이어 만들었다(역방향 불가능, s18_C 확인)

## 검증된 결과 (세션18, 2026-09-28, BasicAgent·동기 20Hz·기본 인자, follow on)
- north: 완주 622.5m / 123.1s / 충돌 0 / 정차 오차 0.93m / 최고 24.0km/h
- south: 완주 726.8m / 143.8s / 충돌 0 / 정차 오차 0.92m / 최고 23.1km/h
- middle: 실패(구 판정 기준) t=87.1s, 정문부터 437.9m. j44 좌회전 커넥터 r1592(-97°) 안에서 차 중심이
  차선 경계를 0.01~0.23m 넘어 반대 방향 영역으로 7 tick(0.35s. 세션18 보고서 7tick/0.30s, 7 x 0.05 = 0.35s. 보수적으로 0.35s 채택 [세션26-B]) -> "반대 차선 침범" 중단. 충돌 0
- legacy(세션17): 완주 궤적 774.4m / 153.4s (774.6m 는 세션16 값 [세션26 정정]) / 충돌 0

## 알려진 한계 (고치지 않고 동결)
- 커넥터 겹침: junction 상자 겹침 16쌍(s18_B_overlap.md). 위치로 road 를 조회하면 겹친 커넥터 번호가 찍힐 수 있다
  (주행 제어에는 영향 없음, 로그·판정 road_id 에만 영향)
- 평활화 잔여 61지점(세션11). 경로 위는 middle road1286 의 2곳뿐(s=5.67, s=9.36)
- j107 커넥터 r1917 결함: 앞 도로 끝과 1.06m 어긋나고 31~38° 꺾인 2.97m 지그재그. 정문 구간 인도 침범 기록 원인
- 정문 스폰 기록 아티팩트: 스폰이 road1247 s=0(도로 시작점)이라 출발 순간 차 뒤 꼭짓점이 도로 밖으로 조회(22 tick).
  실제 침범 아님
- middle road1286: 일반 road 인데 +83° 꺾임, R2min 1.29m(s=9.5). r1592 보다 나쁠 수 있는 미검증 지점
- 후문·중문 문 앞 way, 양성재 진입 way 는 service 태그라 변환에서 빠짐(세션19 s19_scope.md)

## 로그 해시 (세션19 추가)
- test_load_map_878.py: 첫 줄 map_sha256=<로드할 파일 앞 16자> (<파일명>)
- test_drive.py: 첫 줄 map_sha256=<XODR_PATH 앞 16자>, 서버 접속 뒤 server_map_sha256=<to_opendrive 앞 16자> 와 일치 여부
  (서버 문자열과 파일 바이트가 같다는 것은 가정, 다음 서버 기동 때 확인)
- frozen_v1 이면 bf835cdfad0cea65 가 찍혀야 한다
