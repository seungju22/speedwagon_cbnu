# 맵세션23 보고서 (2026-10-03) — 연결성 분석·회차 누락 원인·U턴 시험

## 시작
- frozen_v1 sha256 bf835cdfad0cea65 확인(map/maps/cbnu_internal_only_localtm_tags73_smooth.xodr)
- 재변환 실행 안 함(사용자 결정: 현장 답사 결과를 받고 한 번만). Autoware 저장소·OS·docs/logs 의 /home/gm 손대지 않음
- 진행: Phase 0 -> HARD STOP 1 -> 원인 분석·way 목록 -> HARD STOP -> U턴 시험 2회 -> 마감. 중간에 현장 답사 목록 작성

## Phase 0 연결성 — map/docs/logs/s23_connectivity.md
- 스크립트 map/scripts/s23_connectivity.py (방향 그래프 = lane -1 끝 next(), Phase 4 에서 새 맵에 그대로 재사용)
- 도달 가능 826 / 878 (94.1%), 정문 왕복 4 / 878 (0.5%), 주 순환망 복귀 776 / 878 (88.4%), 고립 덩어리 14, 가장 큰 덩어리 10 road
- 미도달 52 의 성격(판정 1): 물리적으로 못 가는 곳은 road1333(후문 주차장 일방통행 고리) 1개.
  나머지 51 은 이미 가는 길의 반대편 차선(회차 없는 막다른 길 14곳의 나오는 반쪽 + 커넥터). 94.1% 는 차선 기준 숫자
- 단 회차 없는 14곳은 들어가면 못 나오는 함정. 회차가 생기기 전까지 경로 계획에서 진입 금지
- 정문 왕복 불가는 정문 출구 way 미변환 때문(s19 와 같음). 772 차이는 출구 하나 때문일 가능성 높음(판정 2, 재변환 후 확인)
- road1333 연결에 필요한 way 6개 = 후문 7개 목록 안

## 회차 누락 원인 — map/docs/logs/s23_deadend_cause.md
- 새 발견: xodr 이 참조만 하고 정의하지 않은 junction 14개 = 회차 커넥터가 없는 양방향 막다른 끝
- 받은 쪽 24 vs 못 받은 쪽 14: way 길이·태그·노드 수·끝 노드 공유·폭·oneway·연결 차수, 첫/끝 노드, 끝 구간 길이, 끝점 기하 — 전부 겹침
- 기각 가설 10(발표 자료 추가): "회차 누락은 way 국소 속성 때문" -> 같은 way468616610 의 한쪽 끝(j96)만 회차.
  변환 범위 실험(scratchpad, 운영 재변환 아님) 14 -> 34 -> 전부. 주변 망 전체에 좌우됨
- 평활화 원인 아님(평활화 전 맵도 같은 14곳). 변환은 결정적(재실행 diff = 머리말 날짜 1줄)
- Osm2OdrSettings 11개 중 회차 관련 0 -> 설정으로 해결 불가
- 후처리 설계: 기존 24개 회차 커넥터는 계수·길이·laneOffset·상대 위치가 전부 같은 틀. 정의 없는 junction 은 기존 road 가 이미 참조
  -> junction 정의 + 커넥터 road 만 추가(기존 road 무변경)
- 설계 수정(사용자): junction 번호를 박지 않고 "xodr 에서 회차 없는 끝을 찾아 채운다". 파이프라인 마지막 단계.
  순서 고정: 재변환 -> 회차 없는 끝 재집계 -> 후처리. frozen_v1 의 14개는 고치지 않음
- 기존 회차 커넥터 공통 결함 발견: 양 끝 0.77m 어긋남, 26.6° 꺾임(= atan 0.5)

## 재변환 way 최종 9개 — map/docs/logs/s23_scope_final.md
- 후문 연결 6(481945510, 481943505, 481475019, 452870644, 481945509, 481943503) + 중문 2(446440352, 446440353) + 정문 출구 1(481950060)
- 392632034 제외: (1) 경계 38.2m 지점까지 도달 가능, 북문 46.1m 와 같은 수준 (2) cbnu_internal_only 범위 원칙.
  정문 출구는 순환 구조용(왕복 0.5% -> 약 88% 예상), 392632034 는 정차점을 당기는 것뿐 -> 목적이 달라 기준이 자의적이지 않음
- 세션4 오판 2개(392632034, 452870644)는 후문 7개 안. 오판 기준: 지명으로 "캠퍼스 밖" 판단. 폴리곤 기준으로는 안쪽
- 세션4 제외 나머지 8개: 사유 기록 없음, 폴리곤 기준 전부 75% 이상 안쪽 -> 현장 답사로 확인
- 주의: 중문 way 2개가 붙으면 중문 회차 r1840 위치가 바뀜(재변환 후 확인)

## U턴 시험 — map/docs/logs/s23_uturn_test.md
- test_drive.py 에 --route uturn 추가(legacy 15 road + r1278 나머지 + j35 회차 r1565 + r1151 s=30). 북문 r1665 는 진입로 R2min 1.05m 꺾임이 섞여 제외
  백업 test_drive.py.bak_s23, sha c2478f02cf5de93e(이전 bfdcefb0bb546289), 기존 5개 루트 dry-plan 전후 동일(legacy 82643554)
- 서버 사용자 기동, 명령 원문 `cd ~/carla/CARLA_0.9.15 && ./CarlaUE4.sh -quality-level=Low -win`(PID 5144). 878/95/1243, 서버 해시 일치
- 메모리: 기동 전 avail 9Gi / swap 0B, Firefox 0 -> 로드 직후 955Mi(일시) -> 2분 후 avail 3.9Gi / swap 838Mi 통과 -> 주행 후 3.9Gi / 829Mi
- 1차(기본 인자): 완주·충돌 0 이지만 r1565 미통과. 커넥터 점 3개를 건너뛰고 자체 U턴, 인도 쪽 3.03m(차체초과 2.25m)
- 2차(--sharp-speed 8, 한 변수): 완주 192.8s·충돌 0, r1565 통과. 커넥터 위 차체초과 0.62m(안쪽). 출구 뒤 인도 쪽 2.62m(1.84m)
  - 원인: 감속 구간이 U턴 도중 풀려 조향 포화(-0.80) 상태로 가속(7.5 -> 14.8km/h)
- 판정(사용자 확정): 템플릿 복제 정당. 한계: 조향 -0.80 은 max_steering 기본값 = 포화 -> "이음매 영향을 측정하지 못함".
  U턴을 제어기 한계에서 돔. 복제 후 14개 기하가 r1565 와 같은지 확인
- 메커니즘: LocalPlanner 가 차에서 직선거리 3.0 + 0.5 x 속도(m/s) 안의 점을 지움(20km/h 5.78m, 8km/h 4.11m).
  접힌 U턴은 점들이 직선으로 가까워 고속이면 통째로 지워짐. 회차 구간 감속은 필수
- 개선안(설계만): 감속 유지 조건을 "커넥터 계획점 근처" -> "차 방향이 진출 road 방향과 20° 안이 될 때까지" 로

## 현장 답사 목록 — map/docs/field_survey_2026-10-04.md
- 스크립트 map/scripts/s23_field_survey.py, 로그 logs/s23_field_survey.log
- 27지점(A 8 / B 2 / C 1 / D 1 / E 14 / F 1) + 경사 6구간(멈추지 않음). 4구역 도보 순서, 직선 합 4.4km, 약 4시간
- 1순위(A·B·C) 11지점만 도는 동선 직선 3.1km
- 작성 중 건물 기준 방향 21곳을 반대로 적었다가 고침(검산: j26 은 자연대 4호관의 북쪽)

## 하지 않은 것
- 재변환(답사 후), 후처리 구현, 감속 개선 구현, 세션13 TM U턴 대조, Phase 5~7

## 마감
- frozen_v1 sha256 bf835cdfad0cea65 (시작 bf835cdfad0cea65, 불변)
- 신규 맵 없음(재변환 안 함)
- 서버: 마감 확인 때 이미 종료돼 있었음(CarlaUE4 프로세스 0, 종료 주체 미확인). 확인 시점 avail 5.3Gi / swap 386Mi
- 새 파일: map_session23_report.md, field_survey_2026-10-04.md,
  logs/s23_connectivity.md·.log·_<맵>.json, s23_deadend_cause.md·.log, s23_scope_final.md, s23_uturn_test.md, s23_field_survey.log,
  session23_load.log, session23_uturn_stdout.log, session23_uturn_slow8_stdout.log, drive_log/plan/collision_20261003_232519·232728,
  drive_plan_dry_uturn.csv, scripts/s23_connectivity.py·s23_deadend_cause.py·s23_field_survey.py, tests/test_drive.py.bak_s23
- 수정: tests/test_drive.py(--route uturn), presentation_2026-10-08.md·ppt_outline_2026-10-08.md(기각 가설 10 append)
- 다음 시작점: 현장 답사 결과 반영(세션4 제외 8 way 등) 후 재변환 way 목록 확정 -> 재변환 1회 -> 회차 없는 끝 재집계 -> 후처리
