# 맵세션27 보고서 (2026-10-06) — 후문 답사 반영·재변환 사전 준비

## 시작
- frozen_v1 sha256 bf835cdfad0cea65 확인. CARLA 서버 실행 안 함. 재변환·새 xodr·새 OSM 생성 안 함
- Autoware 저장소 읽기만(1.9.0 태그, 수정·checkout·pull 없음). Docker 이미지 열지 않음
- Phase 0: field_survey_2026-10-04.md, logs/s24_reconv_list.md, logs/s25_coords.md 있음. map/scripts 기존 67개 항목
- 진행 순서: 1 -> 2(3의 근거로 먼저 일부) -> 3 -> 4 -> 5 -> 6. 전부 완료
- 새 발견(준비 공백): 세션18 재변환 래퍼·ID 대응표 스크립트는 scratchpad 에만 있어 사라졌다 -> 프로젝트 안에 다시 만들고 검증

## Phase 1 후문 답사 문서화 — field_survey_2026-10-06.md (신규)
- 출처 "2026-10-06 사용자 직접 현장 촬영 및 육안 확인". 1~4절 지시문 내용 그대로, 연결 판정은 "가능성이 높음"
- 5절 OSM 대조
  - 일치: 후문 바깥 고가(개신오거리고가차도 bridge=yes) 경계점에서 9m -> 답사한 후문 = 후문B. 문 구조물·barrier 없음
  - OSM 에 없음: 차량 관리 게이트·경비실·라바콘·볼라드·고원식
  - 충돌: 양성재. OSM·주행 기록 모두 캠퍼스 서쪽. "후문 동선상 표지석" 과 맞지 않음
  - OSM 버스정류장 "충북대학교후문" 은 후문B 가 아니라 616~741m 북쪽(후문A 쪽)
  - 주의: 게이트가 공도 쪽 진입로(392632034, 재변환 제외)에 있으면 캠퍼스 안에서 오는 셔틀은 게이트를 안 지난다

## Phase 2 오프라인 확인 — logs/s27_route_checks.md, .log, 스크립트 scripts/s27_route_checks.py
- 2-1 양성재 정차점(주행 CSV 5개 종점 평균 36.627810, 127.452956): 캠퍼스 중심에서 서쪽 270m. "서문" 방향 맞음(이름은 OSM 에 없음)
  - 양성재-후문 겹침: 앞 7 road, 386.3m. 양성재는 후문 노선의 앞 구간이 아니다(경유 시 +484m)
  - 지시문 가설과 다른 더 좋은 소식: 후문 최단 경로(시험 맵, 약 1,339m)가 남문 노선 15 road 전체를 포함. 남문 경로 합 730.0m 까지 검증 구간
- 2-2 후문 게이트: OSM 미등록. 추정 좌표 36.624844, 127.462860(392632034 위, 경계에서 72.9m. 이름 없는 81m2 건물을 경비실로 가정, 미확인)

## Phase 3 재변환 목록 확정 — logs/s24_reconv_list.md 에 "2026-10-06 갱신" 절 append (원본 9,929B 바이트 보존 확인)
- 후문 6: 보류 -> 포함(태그·기하·끝 노드 사슬 개별 확인, 현장은 간접)
- 중문 2: 제외 / A5: 제외 / A8: 보류 유지, 재변환 제외
- 최종: 포함 8 / 제외 8 / 보류 1 (17). 세션24 "확정 2, 최대 12" -> 확정 8
- 태그 보정 총 81(73 + 8). 읽기전용 점검으로 8개 모두 service, internal, 기존과 겹침 0 확인

## Phase 4 재변환 실행 계획 — reconversion_plan.md (신규)
- 절차 2-0 ~ 2-6, 성공 기준(회귀·개선·중립)을 숫자로 고정, 중단 조건
- 회귀 "깨짐": 미완주·충돌·시간 ±1.0s 초과·거리 2.0m 초과(재실행 차 0.3s·0.2m 근거, 배수는 공학적 선택)
- 새로 짚은 위험: 481950060 이 j1·j107 에 붙어 모든 노선의 정문 커넥터 r1914·r1917 이 다시 만들어질 수 있음(세션18 시험에 없던 way)
- 새로 짚은 함정: test_drive.py dry-plan 파일명 고정 -> v2 로 돌리면 기존 CSV 덮어씀. 코드 수정 때 고칠 것
- 스크립트 3개 작성, 읽기전용 부분만 실행·검증
  - s27_reconvert.py check: 입력 해시 5개 일치, 문제 0건 (run 은 미실행)
  - s27_id_map.py --check-s18: 세션18 대응표 871/871 재현, 불일치 7개 비용까지 같음
  - s27_turnfix.py scan(frozen_v1): 정의 없는 junction 14·쌍 14·회차 틀 24, s23 과 같음. write/verify 미실행 = 미검증

## Phase 5 Lanelet2 공수 — logs/s27_lanelet2_survey.md
- xodr -> Lanelet2 자동 도구: CommonRoad Scenario Designer(BETA, GPL-3.0) 존재. 우리 맵 품질 확인 불가
- 로컬 사실: CARLA Town01 Autoware 지도 = 주행 차선 124 -> lanelet 124, 태그 최소(turn_direction·규제 0)
- Autoware 필수(lanelet2_extension 1.2.0 원문): ele, 교차로 turn_direction, 신호등 height(우리 해당 없음)
- 단계별 분량: 합 약 7~10 작업일(추정). 가장 큰 불확실성 = 도구 시험(반나절)과 교차로 우선권 범위

## 마감
- frozen_v1 sha256 bf835cdfad0cea65 (시작과 같음, 불변)
- map/maps 9개 그대로, map/data/processed 13개 그대로, map/logs/fixed_way_ids.txt mtime 9월 18일 그대로
- 새 파일: field_survey_2026-10-06.md, reconversion_plan.md, map_session27_report.md, logs/s27_route_checks.md·.log,
  logs/s27_lanelet2_survey.md, logs/s27_id_map_check.log, logs/s27_turnfix_scan_v1.log, logs/s27_reconvert_check.log,
  scripts/s27_route_checks.py, s27_id_map.py, s27_turnfix.py, s27_reconvert.py
- append: logs/s24_reconv_list.md, docs/setup_log.md(1줄)
- 10/9 재변환 시작 전 확인할 것: 후문 게이트가 공도 쪽 진입로 위인가(그렇다면 392632034 재검토로 입력 way 9개)
