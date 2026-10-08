# 맵 v2 후보 기록 (세션31, 2026-10-07) — 아직 동결 아님

주행 회귀(reconversion_plan.md 3-1)를 통과하기 전이라 "동결" 이 아니라 "후보" 로 적는다.
통과하면 이 문서를 그대로 두고 map_frozen_v2.md 를 새로 쓴다. frozen_v1 은 계속 기준 맵이다.

## 파일
- 경로: map/maps/cbnu_internal_only_localtm_tags81_B_smooth_turnfix.xodr
- sha256: 5240ca8b883e42c02840ae863fc2df47e0f5b7371f714444c8b0b122522ed922 (앞 16자 5240ca8b883e42c0), 크기 2,219,959 B
- 변환 직후(후처리 전): map/maps/cbnu_internal_only_localtm_tags81_B_smooth.xodr, sha256 d01e5f86767bc9d7, 2,186,804 B
- 만든 과정: raw OSM -> 태그 보정 81(기존 73 + 추가 8, 정문 출구 481950060 제외) -> 폴리곤 분류 -> 평활화(세션11 인자)
  -> Osm2Odr(frozen_v1 과 같은 설정) -> s27_turnfix write(회차 커넥터 13 추가)
  - 명령: `.venv-carla/bin/python map/scripts/s27_reconvert.py run --variant B` 그리고
    `.venv-carla/bin/python map/scripts/s27_turnfix.py write <변환 직후> <turnfix>`
  - 같은 환경 근거: 기준선(--base) 결과가 frozen_v1 과 diff 2줄(날짜)만

## 규모
- road 917 / junction 112 / spawn 미측정(서버 로드에서 잰다)
- 변환 직후 904 / 99

## 왜 B안인가
- reconversion_plan.md 7-4 선택 규칙 3: A안(정문 출구 포함)은 정문 j1·j107 연결로가 바뀌고(회차 연결로 2개 없어짐) 4 노선 모두 형상 비용 0.05 이상
- B안은 정문 junction 구성이 frozen_v1 과 같고 4 노선 "연결 유지"
- 정문 왕복(481950060)은 다음 단계 과제로 넘김(A안은 803/905 로 정문 왕복이 되지만 규칙상 채택 안 함)

## 오프라인 확인 [실측]
- 4 노선 경로 합: 622.89 / 730.02 / 693.73 / 637.43m (기준 차 0.1m 안)
- dry-plan 표본 최대 차 0.0012m (0.01m 안)
- 도달 913/917(99.6%), 주 순환망 복귀 903, 고립 덩어리 1(후문 고리), 회차 없는 막다른 끝 0, 일방 막다른 끝 2(r1366, r1379)
- 노선 road 번역: logs/s31_ab_compare_B.log "번역" 줄, 대응표 logs/s28_road_id_map_B.csv(866/878 일치)

## 알려진 결함
- 정문 j109 커넥터 r1972(= frozen_v1 r1917) 이음매 1.085m·37.9도 / 0.895m·30.9도 그대로
- 후문 고리가 둘로 나뉘어(r1368 / r1366) 정문에서 도달 불가. 후문 개선 기준 미달
- Lanelet2 변환 시 원인 A 152 / B 24 (세션30 과 같은 성격)

## 남은 확인 (서버)
- 서버 로드 해시·spawn 수, 4 노선 회귀, 후문, 고도 메시. map_session31_report.md 마지막 절
