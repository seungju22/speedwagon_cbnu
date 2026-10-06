# s18 D. 재변환 준비와 road ID 안정성 실험 (야간 배치, 서버 없음, 채택 안 함)

판정: ID이동 (번호는 사실상 전부 바뀜. 단 형상은 871/878 road 가 그대로이고, 옛→새 번호 대응표로 기계적 변환 가능)

## 안전 확인
- 원본 파일 sha256 앞 16자, 시험 전후 동일:
  xodr cbnu_internal_only_localtm_tags73_smooth.xodr bf835cdfad0cea65
  raw cbnu_campus.osm c383870eb4d1e196 / fixed_tags73 dfcc5ff7b1a41532 / internal_boundary_tags73 ce457630b2de089f / ..._smooth.osm caa2c69b026ccc75
- 기존 스크립트 코드는 수정하지 않음. fix_tags.py 와 classify_internal.py 는 출력 경로가 코드에 고정되어 그대로 실행하면
  기존 tags73 OSM 을 덮어쓴다. 또 fix_tags.write_log 는 map/logs/fixed_way_ids.txt 를 덮어쓴다.
  그래서 scratchpad/trial_convert_s18.py 가 두 스크립트의 함수만 import 해 새 경로로 쓰고 write_log 는 부르지 않았다.
  smooth_osm_curves.py 와 osm_to_xodr.py 는 --input/--output 인자로 실행(기본 파라미터 = 세션11 실행과 같음)
- fixed_way_ids.txt mtime 9월 18일 그대로
- 시험 산출물(전부 새 파일, 채택 안 함):
  map/data/processed/*_trial_s18_base.osm, *_trial_s18_gates.osm (각 3개)
  map/maps/cbnu_internal_only_localtm_tags73_smooth_trial_s18_base.xodr, ..._trial_s18_gates.xodr
  map/docs/logs/session18_trial_{base,gates}_{changes,skipped}.csv, _report.json, map/logs/osm_to_xodr_20260928_2317{07,54}.log
  대조 로그: s18_D_trial_base.log, s18_D_trial_gates.log, s18_D_id_compare.log, s18_D_id_map.log, 대응표 s18_D_road_id_map.csv,
  반경표 s18_road_R2_trial_gates.csv

## 재현성 기준선(먼저 확인)
- 73건 그대로 파이프라인을 다시 돌린 _base: 중간 OSM 3개 sha256 원본과 완전 일치, xodr 은 크기 2,125,011B 동일,
  diff 2줄(생성 시각 주석과 header date)만 다름. road 878 / junction 95, 평활화 필렛 38 / 변경 way 17 / 신규 노드 168 로 세션11 과 동일
- 즉 파이프라인은 결정적이다. 아래 차이는 way 추가의 효과다

## D-1. 추가 way (세션7 문서 목록만)
후문 7개(humun_b_confirmed.md), 중문 2개(setup_log 190행)
- 452870644 service/parking_aisle, 5노드 (번호48)
- 392632034 service/parking_aisle, 5노드 (번호1, 폴리곤 boundary: 밖 약 24m 포함)
- 481945510 service/parking_aisle/oneway=yes, 3노드
- 481943505 service/parking_aisle/oneway=yes, 3노드
- 481475019 service/parking_aisle, 2노드
- 481943503 service/parking_aisle/oneway=yes, 3노드 (진입 쌍)
- 481945509 service/parking_aisle/oneway=yes, 3노드 (진입 쌍)
- 446440352 service/parking_aisle, 2노드 (중문, 끝점1 노드4437127634 에서 시작)
- 446440353 service/parking_aisle, 3노드 (중문, 같은 노드에서 시작)
- 미변환 사유: 9개 모두 highway=service. 세션4·7 기록대로 CARLA 0.9.15 Osm2Odr 가 service 를 변환 대상에서 빼는 것과 같은 원인.
  폴리곤 분류(internal/boundary)는 이미 통과해 internal_boundary OSM 안에 있었다(classify 결과 447/38/664 불변)
- 필요한 태그 변경: highway=service -> highway=unclassified (기존 73건과 같은 방식). oneway·service 태그는 유지
- 미결(세션7 부터): 392632034 의 폴리곤 밖 24m(1순환로 접속부). way 단위 변환이라 이번 시험에는 통째로 포함됨

## D-2. 시험 변환
- 결과: road 912 / junction 98 (현재 878 / 95 대비 +34 road, +3 junction), xodr 2,200,263B
- 평활화: 문제구간 f<0.30 88 -> 63 -> 60 (현재판 89 -> 64 -> 61), 필렛 38 / 변경 way 17 / 신규 노드 168 (현재판과 같음)

## D-3. ID 안정성
번호 기준(같은 id + 같은 시작점·방향·길이):
- 878 중 0개 유지. 같은 id 로 남은 828개도 전부 다른 도로를 가리킴. 사라진 id 50, 새 id 84(1998~2081)
- 루트: legacy 15 / north 17 / south 15 / middle 19 전부 번호 변경. 급커브 r1446 r1428 r1564 r1592 r1838 전부 번호 변경
- junction 연결 목록까지 같은 junction 0/95
- 옛 번호와 새 번호의 차이는 일정하지 않음(+62 가 459개, +84 99개, +55 79개, +51 78개 ...). 앞쪽에 way 가 끼어 뒤 번호가 밀린 형태
형상 기준(시작점·방향·길이 최근접 대응, s18_D_road_id_map.csv):
- 871/878 road 가 형상 일치(비용 < 0.05)
- 형상이 바뀐 7개: 1220, 1898(후문 쪽 접속부, 0.63), 1349(1.25), 1327·1840·1199(중문 막다른 끝, 7.6~15.2), 1333(후문 고립 road, 대응 없음 243.8 = 새 way 들에 잘려 여러 road 로 나뉜 것으로 보임)
- 루트 형상: legacy/north/south 전부 유지. middle 은 종점 road1327(-> r1382)만 변경(막다른 끝이 새 junction 으로 이어짐)
- 급커브 대응: r1446->1508, r1428->1490, r1564->1626, r1592->1654, r1838->1900 (전부 형상 일치)
- 루트 새 번호:
  legacy 1301,1998,1417,2001,1416,1508,1223,1490,1294,1757,1293,1488,1292,1626,1333
  north 1301,1998,1417,2001,1416,1509,1434,1735,1435,1653,1436,1751,1437,1687,1438,1702,1439
  south 1301,1998,1417,2001,1416,1508,1223,1490,1294,1757,1293,1488,1292,1625,1291
  middle 1301,...,1654,1338,1679,1339,1668,1340,1659,1341,1900,1382
  주의: 새 맵의 road1333 은 옛 road1278(legacy 종점)이다. 번호만 보면 혼동된다
- 결론: ID이동. 재변환을 채택하면 test_drive ROUTES·SHARP, 세션11 잔여 61지점, 세션15~17 기준값 대조표를
  대응표로 전부 번역해야 한다. 형상은 유지되므로 주행 기준값(774.6m 등)은 번역 후 비교 가능할 것으로 보임(가정, 주행 미검증)

## D-4. 후문 도달 예측 (시험 맵, 위치 기준)
- 정문 출발 road = 시험 r1301(옛 1247)
- 정문에서 도달 가능한 최근접 차선 -> 후문B 경계횡단점(36.624701,127.463656): 시험 r1442 s=2.0, 거리 1.2m (현재 맵 217.2m)
- 최단 경로 약 1443m, 34 road. 남문 정차점 road1237(시험 r1291)을 지나고, 그 뒤 옛 1419,1219,...,1221,1821 의 직진 연장을 따라감
  -> "남문 길을 이어서 쭉 직진하면 후문" 설명과 맞는 구조(회전 방향은 아래 커브 목록 참고)
- 옛 road1333 은 통째 대응되는 road 가 없다(여러 조각으로 나뉨). 대응 후보 r1392 는 정문에서 도달 불가.
  즉 "고립 road 가 그대로 이어졌다" 가 아니라 "새 way 로 후문까지 새 경로가 생겼다" 로 봐야 한다
- 경로 위 커브(회전 30° 이상 또는 R2min<9): 옛 r1446(+78.8, 평균R 10.12), 옛 r1428(-68.6, 11.9) 는 south 와 같음.
  새 road: r1977(+35.8°, 평균R 10.95, R2min 5.25), r1415(-89.0° 좌, 평균R 11.51, R2min 3.22), r1981(+51.3°, 평균R 9.30, R2min 8.43),
  r1442(-126.8°, 평균R 4.47 = 막다른 끝 U턴 커넥터. 경계점 최근접이 U턴 커넥터라 실제 정차점은 그 앞 road 로 잡아야 함)
- A-2 잠정 경계(평균R 약 9m) 대조: 새 커넥터 중 평균R<9 는 r1442(U턴, 정차점 뒤)뿐. r1981 9.30 은 경계 부근. r1415 는 평균R 11.5 지만
  R2min 3.22(국소 꺾임)라 주의
- 중문: 옛 road1327(시험 r1382) 끝이 새 road 1906/1907/1908 로 이어짐 -> 중문 쪽으로 연장 경로 생김(세부 미확인)
- 새로 생긴 커넥터 중 회전 30° 이상 또는 R2min<9 인 것은 60개 이상(s18_D_id_compare.log). 대부분 새 junction(j5, j13, j21, j36, j51, j99)의
  U턴·회전 커넥터이며 루트 경로 밖

## 재변환 실제 수행 시 절차 (문서화 요구)
1. 원본 xodr·OSM 은 절대 덮어쓰지 않는다. 새 파일명(예: _v2)으로만 출력
2. fix_tags.py / classify_internal.py 는 출력 경로가 고정이므로 그대로 실행하지 말 것(덮어씀). 이번 래퍼 방식 또는
   인자 추가(코드 수정, 사용자 승인 후)로 새 경로에 출력
3. 먼저 기존 73건 그대로 돌려 원본과 같은지 확인(이번에 결정적임을 확인)
4. 변환 직후 형상 대응표를 만들고(이번 id_map_s18.py 방식) 루트·급커브·잔여 지점 번호를 번역한다
5. 번역한 루트를 next() 체인으로 다시 검증하고 dry-plan 을 만든다
6. 이번 결과상 번호는 반드시 바뀐다고 가정한다(세션7 road1327/1333 이 유효했던 것은 way 추가 없이 태그 보정·평활화만 했기 때문으로 보임)

## 순서 판단 근거 (점군 지도와의 관계)
점군 지도는 맵 형상을 LiDAR 로 스캔해 만든다. 형상이 바뀌면 점군도 다시 만들어야 한다.
이번 시험에서 way 추가는 기존 871 road 의 형상을 바꾸지 않았지만, 후문·중문 주변 7개 road 와 새 34 road 는 바뀌거나 새로 생긴다.
따라서 후문·중문을 넣을 거라면 재변환은 점군 지도보다 먼저 하는 것이 맞다(나중에 하면 적어도 그 주변 점군을 다시 만들어야 함).
사용자 결정 사항: "4곳 모두 포함(재변환 먼저)" 또는 "2곳(북문·남문)으로 동결". 이 배치에서는 채택하지 않았다.
