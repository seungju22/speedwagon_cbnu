# 재변환 당일 실행 계획 (작성 2026-10-06 세션27, 실행 예정 2026-10-09 이후)

이 문서는 실행 전에 성공 기준을 고정하기 위한 절차서다. 결과를 본 뒤 기준을 바꾸지 않는다.
기준을 바꿔야 하면 이 문서 끝에 "기준 변경" 절을 append 하고 이유를 적는다(원문 수정 금지).
숫자는 전부 출처 파일에서 가져왔다. 출처가 없는 값은 "근거 없음, 추정" 으로 적었다.

## 0. 대상과 원칙

- 입력 way: 기존 태그 보정 73 + 추가 8 = 81 (logs/s24_reconv_list.md "2026-10-06 갱신" 절)
  - 추가 8: 481950060(정문 출구), 442595850(박물관 차고지 고리), 후문 연결 6(481945510, 481943505, 481475019,
    452870644, 481945509, 481943503)
- 원본 frozen_v1 은 절대 건드리지 않는다
  - map/maps/cbnu_internal_only_localtm_tags73_smooth.xodr, sha256 앞 16자 bf835cdfad0cea65
  - 중간 OSM 3개(tags73, internal_boundary_tags73, _smooth)와 raw OSM 도 그대로
- 순서 고정(사용자 지시, s23_deadend_cause.md): 재변환 -> 회차 없는 끝 재집계 -> 후처리
- 실행은 한 단계씩. 각 단계 끝에 frozen_v1 해시를 다시 찍는다

## 1. 준비된 도구 (세션27 작성)

- map/scripts/s27_reconvert.py: 재변환 래퍼
  - check: 읽기전용 점검(입력 해시 5개, 추가 way 존재·태그, 기존 73 과 중복, 폴리곤 분류, 출력 파일 미존재). 세션27 에 실행, 문제 0건
  - run --tag T [--base]: 실제 실행. 세션27 에는 실행 안 함
  - 왜 래퍼인가: fix_tags.py·classify_internal.py 는 출력 경로가 고정이라 그대로 돌리면 tags73 OSM 을 덮어쓴다.
    fix_tags.write_log 는 map/logs/fixed_way_ids.txt 를 덮어쓴다. smooth_osm_curves.py 는 --log-prefix 없이 돌리면
    docs/logs/session11_smoothing_*.csv 를 덮어쓴다. 래퍼는 함수만 import 하고 write_log 를 부르지 않으며 새 접두어를 준다
  - 세션18 에 같은 일을 한 trial_convert_s18.py 는 scratchpad 에만 있어 사라졌다. 이번 래퍼는 프로젝트 안에 남긴다
- map/scripts/s27_id_map.py: 옛 -> 새 road id 형상 대응표
  - 검증: frozen_v1 -> 세션18 시험 맵으로 돌려 s18_D_road_id_map.csv 를 871/871 그대로 재현, 불일치 7개 비용까지 같음
    (logs/s27_id_map_check.log)
- map/scripts/s27_turnfix.py: 회차 없는 막다른 끝 탐지·채움
  - scan(읽기전용): frozen_v1 에서 정의 없는 junction 14개(j4 15 17 26 55 56 75 83 89 92 97 98 99 104), 쌍 14개,
    회차 틀 24개를 s23 기록과 똑같이 찾음. 틀의 상대 위치 일정(방위 0.4636rad = atan 0.5, 폭 0.0002m 이내)
    (logs/s27_turnfix_scan_v1.log)
  - write/verify: 세션27 에 실행 안 함(새 xodr 생성 금지) = 미검증. 10/9 에 먼저 frozen_v1 사본으로 검증한다(4절)
  - 개수·번호를 코드에 넣지 않았다. 입력 xodr 를 읽어 스스로 찾는다
- 기존: s23_connectivity.py(인자: xodr 경로, 정문 road id), tests/test_load_map_878.py(--xodr 있음),
  tests/test_drive.py(맵 경로 고정 XODR_PATH, ROUTES 번호 고정 -> 2-3 에서 수정 필요)

## 2. 단계별 절차

### 2-0. 시작 확인 (서버 없음, 약 5분)
1. sha256: frozen_v1 = bf835cdfad0cea65. 다르면 즉시 중단
2. `~/campus_mobility_sim/.venv-carla/bin/python map/scripts/s27_reconvert.py check` -> "문제 0건"
   - 입력 해시 기대값: raw c383870eb4d1e196 / fixed_tags73 dfcc5ff7b1a41532 / internal_boundary_tags73 ce457630b2de089f
     / _smooth caa2c69b026ccc75 (세션18 기록과 세션27 실측 같음)
3. free -h 기록. map/logs/fixed_way_ids.txt mtime(9월 18일) 기록

### 2-1. 입력 준비와 기준선 재현 (서버 없음, 약 5분. 근거: 세션18 두 변환 로그 시각 차 47초, 추정 포함)
1. 기준선: `s27_reconvert.py run --base --tag tags73_rebase`
   - 기존 73건 그대로. 결과 xodr 가 frozen_v1 과 diff 2줄(생성 시각 주석, header date)만 달라야 한다(세션18 D 확인)
   - 크기 2,125,011B, 중간 OSM 3개 sha256 이 원본과 같아야 한다
   - 다르면 중단: 변환 환경이 바뀐 것이다(CARLA 라이브러리, 파이썬 패키지 등). 본 변환 결과를 해석할 수 없다
2. 태그 필터 재현: Osm2Odr 는 highway=service 를 변환하지 않는다(세션4·7, s24 4절: 맵 차선 겹침 service 0/155).
   그래서 추가 way 의 highway 를 unclassified 로 바꾼다. service·oneway 등 다른 태그는 유지(세션4 73건과 같은 방식)

### 2-2. 변환 실행 (서버 없음, 약 3분)
1. `s27_reconvert.py run --tag tags81_v2`
2. 출력 파일명 규칙: map/data/processed/cbnu_campus_fixed_tags81_v2.osm, cbnu_internal_boundary_tags81_v2.osm,
   cbnu_internal_boundary_tags81_v2_smooth.osm, map/maps/cbnu_internal_only_localtm_tags81_v2_smooth.xodr
   - 후처리 뒤: map/maps/cbnu_internal_only_localtm_tags81_v2_smooth_turnfix.xodr (최종 후보, 이하 "v2")
3. Osm2Odr 설정값(logs/osm_to_xodr_20260924_014306.log = frozen_v1 생성 로그, osm_to_xodr.py 코드값과 같음)
   - default_lane_width 3.35 / generate_traffic_lights False / all_junctions_with_traffic_lights False
   - proj_string +proj=tmerc +lat_0=36.627298 +lon_0=127.456394 +ellps=WGS84
   - center_map True / use_offsets False / offset_x 0.0 / offset_y 0.0 / elevation_layer_height 0.0
4. 평활화 인자: smooth_osm_curves.py 기본값(세션11 승인값) R_TARGET 11.0 / R_ACC 10.0 / d_max 2.0 / th_cause 8.0 /
   th_absorb 10.0 / arc_step 2.0 / m_fixed 2.0 / near_m 8.0 / max_iter 3 / max_align_err 3.0
5. 기록: 출력 sha256, road·junction 수, 평활화 필렛 수(세션11·18: 38, 변경 way 17, 신규 노드 168 은 73건 기준 값이라
   이번에는 달라질 수 있음, 비교만), osm_to_xodr 로그의 경고 줄 수
6. frozen_v1 해시 재확인

### 2-3. ID 이관 (서버 없음, 약 20분)
1. `s27_id_map.py <frozen_v1> <v2 변환 직후 xodr> map/docs/logs/s28_road_id_map.csv`
   - 형식은 s18_D_road_id_map.csv 와 같음(열 이름만 trial_id -> new_id)
2. 이관 대상: test_drive.py ROUTES 의 roads·connectors·sharp, SHARP_CONNECTORS, legacy ROUTE_ROADS·CONNECTOR_JUNCTION
   - junction 번호도 바뀐다(세션18 시험: 같은 junction 0/95). junction 은 name(= OSM 노드 id)으로 대응시킨다. 이름은 안 바뀐다
3. 이관 실패를 알아차리는 방법 (하나라도 걸리면 그 노선은 주행하지 않는다)
   - 노선 road 가 대응표에서 비용 0.05 이상: 형상이 바뀐 것. 특히 정문 커넥터 r1914(j1)·r1917(j107) 확인.
     481950060 이 j1·j107 에 새로 붙어 두 교차로 커넥터를 변환기가 다시 만든다(세션18 시험에는 이 way 가 없어 미시험)
   - 새 id 중복(두 옛 road 가 같은 새 road 로): s27_id_map 요약 "일치 중 새 id 중복" 이 0 이 아님
   - next() 체인 끊김: 번역한 노선을 새 맵에서 lane -1 next() 로 이어 검사(세션18 방식). 끊기면 실패
   - dry-plan 대조: 번역 노선의 dry-plan 표본 (x, y) 가 기존 drive_plan_dry_<route>.csv 와 0.01m 이내.
     주의: test_drive.py dry_plan 은 파일명이 고정이라 v2 로 돌리면 기존 v1 CSV 를 덮어쓴다 -> 코드 수정 때 파일명에 맵 꼬리 추가
   - 경로 합: 번역 노선 경로 합이 북문 622.9 / 남문 730.0 / 중문 693.7 / 양성재 637.4m 와 0.1m 이내
     (logs/s26_route_sums.log, 확정 숫자 카드 2절)
4. test_drive.py 수정(사용자 확인 후): 백업 test_drive.py.bak_s28, --xodr 인자와 맵별 ROUTES 표 추가. 기본값은 frozen_v1 그대로.
   legacy dry-plan 82643554 불변 확인

### 2-4. 회차 후처리 (서버 없음, 약 20분)
1. 도구 검증 먼저(frozen_v1 사본, 결과는 scratchpad 에만):
   `s27_turnfix.py write <frozen_v1> <scratchpad>/frozen_turnfix_test.xodr` -> `verify` -> s23_connectivity.py 로 연결성
   - 기대: 커넥터 14 추가, verify 실패 0, 도달 불가 51(반대편 차선) -> 0 근처(r1333·r1344 관련은 남음)
   - 기대와 다르면 v2 에 적용하지 않는다
2. v2 탐지: `s27_turnfix.py scan <v2 변환 직후>` -> 정의 없는 junction 수와 목록을 기록. frozen_v1 목록(14)과 비교만 한다.
   개수를 미리 정하지 않는다(회차 생성은 망 전체에 따라 달라짐 s23 E1: 14 -> 34)
3. 채움: `write <v2 변환 직후> <v2 turnfix>` -> `verify`
   - "채움가능=False" 로 건너뛴 끝이 있으면 목록 기록. 그 끝은 경로 계획 진입 금지(s23 선택지 d)
4. 주의: 템플릿 그대로 복제라 기존 24개와 같은 이음매 결함(0.77m, 26.6도)을 가진다(s23). 셔틀 노선은 종점 회차 외에는 쓰지 않는다

### 2-5. 연결성 재분석 (서버 없음, 약 10분, 추정)
1. `s23_connectivity.py <v2 변환 직후> <새 정문 id>` 와 `s23_connectivity.py <v2 turnfix> <새 정문 id>` 두 번
   - 후처리 효과와 way 추가 효과를 나눠 보기 위해
2. 비교 대상(재변환 전, logs/s23_connectivity.md): 도달 826/878(94.1%), 주 순환망 복귀 776/878(88.4%),
   정문 왕복 4/878(0.5%), 고립 덩어리 14, 회차 없는 막다른 길 15(양방향 14 + 일방통행 r1344)

### 2-6. 주행 재검증 (서버, 사용자 기동)
- 서버 기동(사용자): cd ~/carla/CARLA_0.9.15 && ./CarlaUE4.sh -quality-level=Low -win
- 로드: python map/tests/test_load_map_878.py --wall-height 0 --xodr <v2 turnfix> -> road·junction·spawn 수, 서버 해시 = 파일 해시
- 2분 후 메모리 측정(중단 기준 아래 4절)
- 순서(위험 낮은 것부터, 앞이 깨지면 뒤는 안 한다)
  1. 양성재(교차로 회전 적고 반복 기록 많음) 2. 북문 3. 남문 4. 중문 5. 후문(새 노선)
  - legacy 는 선택(시간이 남으면, 기준 774.4m / 153.4s)
- 명령: python map/tests/test_drive.py --route <이름> --xodr <v2 turnfix> (기본 인자, follow on)
- 후문 노선 정의는 재변환 맵에서 만든다: 정차점 = 고리 위 경계 최근접 점(세션23 결정 1, 경계에서 38.2m). 게이트 위치 답에 따라 변경
- 예상 소요(근거: 세션20 양성재 주행 실제 시계 21초/시뮬 125.6s, 세션21 로드·대기 기록)
  - 서버 기동·로드·2분 대기 약 10분 / 노선당 실행·확인 약 3분 x 5 = 15분 / 기록 15분 -> 서버 구간 약 40분(추정)
- 전체(2-0 ~ 2-6) 약 2시간(추정). 막히면 그 단계에서 끊고 보고

## 3. 성공 기준 (실행 전에 고정)

### 3-1. 회귀 기준 (가장 중요. 하나라도 깨지면 재변환 재검토)
기준값 출처: presentation_numbers_locked.md 2절(map_session18·20 보고서)
- 북문 완주, 충돌 0, 시간 123.1s, 궤적 622.5m
- 남문 완주, 충돌 0, 시간 143.8s, 궤적 726.8m
- 중문 완주, 충돌 0, 시간 136.8s, 궤적 690.2m
- 양성재 완주, 충돌 0, 시간 125.6s, 궤적 633.9m
"깨짐" 정의
- 미완주(기본 record 판정) 또는 충돌 1건 이상
- 시간이 기준 대비 ±1.0s 를 넘음
  - 근거: 같은 맵·같은 인자 재실행 차이가 양성재 4회 125.4~125.7s(0.3s, map_session21_report.md), legacy 153.2s·153.4s
    (세션15·17). 1.0s 는 그 약 3배로 정한 값(공학적 선택)
- 궤적 거리 차 2.0m 초과
  - 근거: 같은 맵 재실행 거리 차 0.2m 이하(map_session20_report.md 80행). 2.0m 는 그 10배(공학적 선택)
- 오프라인 사전 회귀(서버 전): 2-3 의 경로 합 0.1m, dry-plan 0.01m 를 넘으면 그 노선 주행 전에 "형상 변경" 으로 보고
- 해석 주의: 북문·남문 기준값은 세션18 판정 코드 시절 값이다. 세션19~21 에 판정 규칙이 바뀌었지만 완주 시각 조건(s >= 종점 - 1.0)은
  같다. 1.0s 를 넘으면 같은 날 frozen_v1 로 같은 노선을 다시 달려 맵 탓인지 환경 탓인지 가른다(서버 시간 추가 약 5분)

### 3-2. 개선 기준
- 정문 왕복 비율이 오른다
  - 기준 4/878(0.5%). 성공 = 새 정문 road 가 주 순환망(최대 강연결 성분) 안에 든다 = 정문 왕복 수 = 주 순환망 복귀 수
  - 예측(s19·s23): 주 순환망 수준(재변환 전 776/878, 88.4%). 예측이지 기준이 아니다
- 도달 가능 비율이 오른다
  - 기준 826/878(94.1%, 차선 기준). 변환 직후(후처리 전) 이 비율 이상, 후처리 뒤 도달 불가가 "후처리로 못 채운 끝" 으로만 설명됨
- 후문이 본 도로망과 연결된다
  - 기준: road1333(후문 고리) 고립, 정문에서 최근접 217m(확정 숫자 카드 2절). 성공 = 고리 조각이 정문에서 도달 가능하고 주 순환망으로 복귀 가능
- 후문 노선이 완주한다: 완주, 충돌 0. 시간·거리 기준값은 없다(첫 주행). 결과를 기록만 한다

### 3-3. 중립 기준 (기록만, 성공·실패 아님)
- road 수·junction 수·spawn 수: 재변환 전 878 / 95 / 1243 (logs/session18_load.log)
- 회차 없는 막다른 끝(정의 없는 junction): 재변환 전 14(+ 일방통행 r1344), 회차 연결로 24 (s23, 세션27 scan 재현)
- 차선 이탈(차체, 중앙선 쪽) 4노선 합 8회(북1 남2 중3 양2), r1592 1.19m, r1838 0.84m, r1428 0.61m, 정문 r1247 0.10~0.17m
- 참고: 세션18 시험(9 way, 입력 다름) 912 / 98 은 비교 대상이 아니다

## 4. 중단 조건
- 회귀 기준(3-1)이 하나라도 깨지면 다음 노선으로 넘어가지 않고 즉시 중단·보고
- frozen_v1 해시가 bf835cdfad0cea65 가 아니면 즉시 중단(단계마다 확인)
- 기준선 재현(2-1)이 실패하면 본 변환으로 넘어가지 않는다
- 메모리(CLAUDE.md 승계): 안정 상태 avail 2GiB 미만 또는 swap 4GiB 초과 -> 즉시 중단·보고. 로드 직후 피크는 제외하되
  2분 후 재측정도 기준 미만이면 중단. 같은 세션 재시도 금지
- 같은 오류 2회 -> 멈추고 보고

## 5. 이날 하지 않는 것
- frozen_v1·원본 OSM·중간 OSM 덮어쓰기
- 결과를 본 뒤 3절 기준 수정(필요하면 "기준 변경" 절 append)
- 점군 지도, Lanelet2 변환, 건물 배치

## 6. 실행 전 사용자 확인
1. 후문 게이트 위치(field_survey_2026-10-06.md 5절). 공도 쪽이면 392632034 재검토 -> 입력 way 가 9개가 됨
2. test_drive.py 수정(2-3 의 4) 승인
