# 맵세션32 보고서 (2026-10-07) — v2 주행 검증 + 고도 메시 확인

표기 [실측] / [코드] / [추정] / [불명]. 숫자 출처는 reconversion_plan.md, map_session31_report.md, 로그 파일
v2 = map/maps/cbnu_internal_only_localtm_tags81_B_smooth_turnfix.xodr

## Phase 0 오프라인 선행 (서버 없음)

### 0-1 해시와 수량 [실측]
- frozen_v1 sha256 bf835cdfad0cea65 (시작 확인)
- v2 sha256 5240ca8b883e42c0, road 917, junction 112 (파일 직접 계수)
- 세션31 보고서 값(5240ca8b883e42c0, 917/112)과 일치

### 0-2 정문 왕복 비율 (B안 + 회차 후처리 뒤) [실측]
- 다시 계산: s23_connectivity.py v2 1278 (정문 r1247 -> v2 r1278, 대응표 비용 0)
  - 출력이 logs/s31_connectivity_B_turnfix.log 와 줄 단위로 같음(앞 300자 비교)
- 정문 왕복 4/917 (0.4%): [1150, 1278, 1417, 1970]
- 재변환 전(frozen_v1): 4/878 (0.5%) (s31_connectivity_frozen.log)
- 변화 없음. 계획 7-4 규칙 3 의 예상값과 같다
- 참고: 주 순환망 복귀 903/913, 도달 913/917
- 정정: 지시문은 "세션31 보고서에 이 항목이 없다" 고 했으나 보고서 Phase 5 에 "정문 왕복 4 -> 4" 가 있다. 분모(917)가 없었을 뿐

### 0-3 회귀 기준 선인용 (reconversion_plan.md 3-1 원문, 주행 전 옮김)
```
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
```
- 같은 계획 2-6 의 실행 조건: 명령 test_drive.py --route <이름> --xodr <v2>, 기본 인자, follow on
- 후문(3-2): 완주, 충돌 0. 시간·거리 기준값 없음(첫 주행), 기록만

### 0-4 노선 재해석 [실측] — logs/s32_route_endpoints.log (scripts/s32_route_endpoints.py)
- 출발·종점 출처: 지시문은 "세션27 기록" 이라 했으나 세션27 보고서에는 4 노선 출발·종점이 없다.
  실제 정의는 tests/test_drive.py ROUTES(세션18~20, sha c2478f02cf5de93e) 다. 그것을 썼다
- 방법: ROUTES road 를 logs/s28_road_id_map_B.csv 로 번역, v2(turnfix)에서 좌표·next()·junction name 대조
- 북문: v1 r1247 s=0 -> v2 r1278, 종점 v1 r1378 s=78.0 -> v2 r1415 (658.230, 685.038), 좌표 차 0.0000m
- 남문: 종점 v1 r1237 s=47.0 -> v2 r1268 (570.783, 565.327), 차 0.0000m
- 중문: 종점 v1 r1327 s=70.0 -> v2 r1360 (664.836, 982.805), 차 0.0000m
- 양성재: 종점 v1 r1123 s=22.0 -> v2 r1153 (216.108, 550.232), 차 0.0000m
- 출발은 4 노선 공통 v1 r1247 s=0 (219.519, 1087.496) -> v2 r1278 같은 좌표
- 4 노선 모두 최대 형상 비용 0.0000, next() 끊김 0, 커넥터 junction name 대응 불일치 0
- 대응 실패 노선 0
- 오프라인 사전 회귀를 turnfix 본으로 다시 돌림(s31 은 같은 값): 경로 합 차 -0.01~+0.03m, dry-plan 최대 0.0012m, 초과 0
- 스폰: test_drive.py 는 인덱스가 아니라 좌표 (221.40, -1086.84) 와 경로 road 집합으로 spawn 을 찾는다 [코드].
  인덱스 재사용 문제 없음. 단 경로 road 집합이 v1 id 라 v2 에서는 ROUTES 번역이 있어야 찾는다
- v2 ROUTES 번역값(test_drive.py 수정 시 그대로 쓸 값)은 로그의 "v2 roads / connectors / sharp" 줄

### 0-5 후문 종점 도달 가능성 [실측] — logs/s32_backgate_reach.log (scripts/s32_backgate_reach.py)
- 종점 출처: 세션28 은 하나의 확정 좌표를 정하지 않았다. 확정은 "후문=B, 게이트는 392632034 위(추정)" 이고
  계획 7-5 가 정차점 후보 둘(고리 위 / 게이트 추정 36.624844, 127.462860)을 둔다. 7-8 목적 "정차점 = 게이트" 에 따라 게이트 추정점을 원래 종점으로 본다
- 정문 r1278 에서 도달 913/917, 불가 [1368, 1892, 1893, 1894]
- 추정 게이트 36.624844, 127.462860: 최근접 r1151 lane -1 s=23.1 (1.65m), 도달 가능, 정문에서 33 road 약 1,368m
  -> 그대로 쓴다. 대체 종점 불필요
- 참고 후보
  - 고리 접점 36.624909, 127.462514: 최근접 r1893(j3) 도달 불가. 도달 가능 최근접 r1896 s=7.0, 1.62m, 약 1,337m
  - 경계 횡단점 36.624701, 127.463656: r1418(j4, 회차) 도달 가능, 약 1,449m
  - 고리 위 경계 최근접 노드(세션23 결정 1 의 정차점): node4496854270 은 r1368(도달 불가 조각) 위.
    도달 가능 최근접 r1366 s=87.5, 8.32m
  - 세션23 은 이 거리를 "경계에서 38.2m" 로 적었으나 이번 계산은 경계 횡단점에서 49.9m. 기준 점이 다른지 미확인 [불명]
- 거리는 세션28 시험 맵 참고값(1,337 / 1,368 / 1,449m)과 같다
- 주의: 게이트 방향 r1151 lane -1 은 안 -> 밖(나가는 방향). 들어오는 방향만 현장 근거가 있다(계획 7-8) [코드 + 기록]
- 후문 노선은 test_drive.py ROUTES 에 없다("back" 은 미구현 종료 코드 2) [코드]

### Phase 0 판정
- 중단 조건 해당 없음: frozen_v1 불변, 노선 대응 실패 0, 후문 종점 도달 가능
- 서버 전 남은 결정(사용자)
  1. test_drive.py 수정 승인(계획 2-3 의 4 / 7-6 의 2 "미결"). v2 를 달리려면 --xodr 와 v2 ROUTES 가 필요하다
  2. 주행 순서: 지시문 북문->남문->중문->양성재 / 계획 2-6 양성재->북문->남문->중문. 서로 다르다
- 메모리 측정 전제: 이 시점 pgrep -f firefox 13건(0건 아님)

## 사용자 결정 (2026-10-07, Phase 0 뒤)
- test_drive.py 수정 승인(4 노선 + 후문 back)
- 주행 순서: 계획서 2-6(양성재 -> 북문 -> 남문 -> 중문 -> 후문). 지시문 순서는 철회
- 이 세션 규칙: 절차·순서·판정 기준이 지시문과 계획서에서 어긋나면 계획서를 따르고 어긋남만 기록. 파일 수정·범위 변경은 계속 묻는다
- 서버는 Claude 가 직접 기동 가능(사용자 근거리 조건). 플래그 고정 -quality-level=Low -win, PID 로만 kill,
  avail < 1GiB 관측 시 즉시 kill 후 보고(추가 기준)

## test_drive.py 수정 [실측]
- 백업 tests/test_drive.py.bak_s32 (c2478f02cf5de93e)
- 추가: ROUTES_V2(4 노선 번역 + back), MAP_ROUTES(맵 sha16 -> 노선 표), --xodr 인자
  - 알 수 없는 맵·표에 없는 노선은 실행 없이 종료 코드 2. 기본값은 frozen_v1 그대로
  - dry-plan 파일명: frozen_v1 이 아니면 _map<sha8> 꼬리(v1 CSV 덮어쓰기 방지)
- back 노선: 33 road, 앞 15 road = south, 종점 r1151 s=23.1, 경로 합 1368.2m. sharp 는 south 두 커넥터(나머지 최대 51.3°)
- 검증
  - v1 dry-plan 6개(legacy 포함) 재생성 후 바이트 비교 변경 0. legacy 82643554 불변
  - v2 dry-plan 4 노선 표본 수 같음(164·189·183·167), v1 대비 (x, y) 최대 차 0.0000m
  - back dry-plan 357점, 마지막 r1151 s=21.175 (1097.979, -221.777)
  - 오류 경로: v1 + back -> 종료 2, v2 + legacy -> 종료 2

## Phase 1 서버 기동과 로드 [실측]
- 기동 전: pgrep -f firefox 0(자기 매칭 제외), CARLA 프로세스 0, avail 10.8GiB, swap 0, frozen_v1 bf835cdfad0cea65
- 기동 명령 원문(Claude 실행, 20:12:35)
  cd ~/carla/CARLA_0.9.15 && nohup ./CarlaUE4.sh -quality-level=Low -win > /tmp/carla_s32.log 2>&1 &
- PID: CarlaUE4.sh 5616, CarlaUE4-Linux-Shipping 5623(kill 대상). 포트 2000 약 10s 에 열림(폴링)
- 로드 명령: .venv-carla/bin/python map/tests/test_load_map_878.py --wall-height 0 --xodr map/maps/cbnu_internal_only_localtm_tags81_B_smooth_turnfix.xodr
  (logs/s32_load_v2.log, 20:12:52~20:13:02)
- 로드 성공. 서버 해시 5240ca8b883e42c0 = 파일. road 917 / junction 112 (오프라인 값과 일치)
- spawn 1302 (처음 측정. frozen_v1 1243)
- 메모리: 로드 직후 avail 1217Mi·swap 976Mi(피크) -> 2분 뒤 avail 4195Mi, swap 907Mi. 중단 기준 미해당
- 주의: 로드 직후 피크가 추가 기준(1GiB)에 0.2GiB 차까지 갔다

## Phase 2 회귀 주행 [실측] — 1번째 노선(양성재)에서 기준 위반, 중단
명령: .venv-carla/bin/python map/tests/test_drive.py --route yangseong --xodr map/maps/cbnu_internal_only_localtm_tags81_B_smooth_turnfix.xodr
(기본 인자, follow on. stdout logs/s32_yangseong_run1.log, 궤적 drive_log_20261007_201537.csv)

### 양성재 v2 (20:15:36~20:15:58)
- 서버 해시 5240ca8b883e42c0 일치. spawn r1278 s=0.00 (217.85, -1087.41). target_speed 20.0km/h(기존 DESIRED_SPEED_KMH 같음)
- 결과: 완주(정지 근접 판정, 세션21 규칙). 충돌 0
- 시간 130.9s (기준 125.6s, 차 +5.3s) -> 기준 ±1.0s 초과 = 위반
- 궤적 633.7m (기준 633.9m, 차 -0.2m, CSV xy 누적, scripts/s32_traj_len.py) -> 통과
- 경과: t=125.30 계획 소진 -> r1153 s=20.94(완주 조건 s>=21.0 에 0.06m 모자람)에서 정지 -> 정체 5.0s 후 정지 근접 완주(정차점 1.11m)
  즉 +5.3s 중 5.0s 는 정지 근접 판정의 대기 시간이다
- 주행 후 메모리 avail 4171Mi, swap 905Mi

### 계획 3-1 해석 절차: 같은 날 frozen_v1 재주행
- 같은 서버에서 frozen_v1 재로드(logs/s32_load_frozen_v1.log: 해시 bf835cdfad0cea65 일치, 878/95, spawn 1243). 2분 뒤 avail 4230Mi, swap 880Mi
- 명령: test_drive.py --route yangseong (기본, logs/s32_yangseong_frozen_v1_run1.log, drive_log_20261007_201853.csv)
- 결과: 완주(일반 판정), 125.5s, 종점 0.81m, 궤적 634.1m, 충돌 0 -> 기준 통과
- 주행 후 avail 4228Mi, swap 880Mi
- 판정: 같은 환경에서 frozen_v1 은 통과, v2 는 시간 위반. 계획 4절에 따라 다음 노선(북문)으로 넘어가지 않았다

### 위반 진단 (기준은 바꾸지 않음)
- 노선 road 가 대응표에서 바뀌었나: id 는 바뀜(17 road 전부 새 id), 형상 비용 전부 0.0000
- 바뀐 도로 기하: dry-plan 표본 167점 v1 대비 최대 0.0000m. 기하 변화 없음
- 회차 후처리 연결로를 지나나: 안 지남(추가 13개 r2054~r2066, 주행 road 와 교집합 0)
- 궤적 비교(같은 tick 위치 차): v2 vs 오늘 v1 최대 1.26m(t=119.9) / 오늘 v1 vs 세션21 v1 최대 1.66m(t=73.95).
  맵이 같은 두 v1 주행 차이가 v2-v1 차이보다 크다
- t=125.3 속도: v2 4.11m/s(s=19.47) / v1 6.13m/s(s=20.16). 정지 직전 감속 차이로 0.06m 모자람
- 같은 현상 선례: 세션20 양성재 재실행이 s=20.98 에서 서서 정체 실패(test_drive.py ARRIVE_STOP_DIST_M 주석) -> 세션21 정지 근접 규칙 도입 이유
- 출발 직후 r1970(정문 회차) 1tick 오배정은 v1 의 r1915 와 같은 시각(t=31.50)에 똑같이 나온다(세션21 로그). 커넥터 겹침 오배정, 기존 현상
- 해석 [추정]: 맵 차이보다 정차 직전 재현성(0.06m 경계) 문제일 가능성이 크다. 그러나 기준 위반은 위반으로 기록한다. 판단은 사용자
- 하지 않은 것: v2 재실행(계획에 없는 절차라 하지 않음), 북문·남문·중문, Phase 3 후문, Phase 4 고도

## 서버 종료 [실측]
- kill 5623(기록한 PID만) -> 프로세스 0. 감시 루프 자동 종료(kill 기록 없음 = avail 1GiB 미만 관측 0)
- 재기동 0회(맵 재로드 1회는 같은 서버에서 frozen_v1 로)
- /tmp/carla_s32.log 71줄: error·warn 문자열 0. 종료 시 RequestExit 직후 Signal 11(segfault, core dump) 1회.
  kill 뒤 종료 과정에서 난 것이고 주행 중이 아니다
- 종료 후 avail 6382Mi

## 마감
- frozen_v1 sha256 bf835cdfad0cea65 (시작·로드 전·마감 확인, 불변). v2 5240ca8b883e42c0 불변
- 동결 가능 조건: 충족 안 됨(양성재 시간 기준 위반, 나머지 미주행)
- 새 파일: 이 보고서, scripts/s32_route_endpoints.py·s32_backgate_reach.py·s32_traj_len.py,
  logs/s32_route_endpoints.log·s32_backgate_reach.log·s32_load_v2.log·s32_load_frozen_v1.log·s32_yangseong_run1.log·
  s32_yangseong_frozen_v1_run1.log, drive_log·collision_events 20261007_201537·201853, drive_plan_dry_<노선>_map5240ca8b.csv 5개
- 수정: tests/test_drive.py (5f682fdf2fa5ad45, 백업 .bak_s32 = c2478f02cf5de93e)
- 주행 기록 형식: 기존과 같이 노선별 stdout 로그 + drive_log CSV. 누적 주행 기록 파일은 없어 따로 append 하지 않음
- git·sync·Autoware 없음
- 다음 시작점: 양성재 위반 판단(사용자) 뒤 v2 회귀 주행 재개 또는 판정 규칙 검토
