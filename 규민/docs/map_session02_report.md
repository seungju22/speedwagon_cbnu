# 맵세션2 보고 (2026-09-16)

## 목표 대비 결과
오늘 목표는 "캠퍼스 도로망이 CARLA에서 실제로 어떻게 보이는지 육안 확인"
이었으나, 대조군 측정 단계에서 중단기준에 도달해 CARLA 육안검증 자체는
다음 세션으로 미뤄졌다. 대신 CARLA 없이 할 수 있는 파일 수준 분석과
시각화를 진행했다.

## 0. 시작 점검
- CLAUDE.md 고정버전 복창, docs/setup_log.md 마지막기록(맵세션1 마감,
  다음시작점 4개) 확인
- 기준선: used2.0Gi/avail10Gi/swap0B, 컨테이너·CARLA 잔존 없음

## 1. 대조군 측정 — Town10HD_Opt
- 최초 "지난세션(09-15) avail1.1GiB와 동일조건 재현" 시도 중 사용자가
  실행명령 원문 확인을 요구 → setup_log 99·103행에 서술 요약만 있고
  명령 원문이 없음을 확인, 비교 불가 판정
- 사용자 결정(HARD STOP C): 대조군을 Town10HD_Opt로, 실험군을
  cbnu_internal_only.xodr로 재설계. 공통옵션
  `-quality-level=Low -windowed -ResX=800 -ResY=600`(세션2 FPS측정
  조건, 앞으로 실험표준으로 채택)
- 실행결과: 로드맵 Carla/Maps/Town10HD_Opt, 2분후 avail1.0Gi/swap3.1Gi
  → 중단기준(avail2GiB미만) 도달, 사용자 승인으로 중단
- 종료후 avail5.6Gi/swap1.2Gi 확인
- 사용자 최종결정: 중단기준 유지(완화 안 함). 근거 — avail1Gi이하
  구간은 스왑개입으로 측정값이 신뢰불가하며, 09-15와 09-16 두 세션
  연속 avail1Gi대 도달을 하드웨어 한계의 근거로 채택
- CLAUDE.md 기록규칙에 "CARLA 실행은 서술요약 금지, 명령원문 그대로
  기록" 추가(재발방지)

## 2. 캠퍼스맵 CARLA 로드
미실시. 1번에서 중단기준 도달로 보류, 같은 세션 재시도 안 함
(CLAUDE.md 금지목록).

## 3. 파일수준 기하 분석 (CARLA 없이, 읽기전용)
`map/scripts/analyze_geometry.py` → `map/docs/xodr_geometry_analysis.md`

- 좌표범위: bbox 1199.4m x 1702.3m (지난세션 캠퍼스 남북실측 1240m
  대비 137%, 폴리곤과 다운로드bbox 범위가 원래 달라 정상/이상 판정 보류)
- road 길이분포: 일반road 44개(문서상 "134"는 junction내부 connector
  포함 전체 road수), min/median/mean/max = 5.24/257.16/292.34/783.27m
- 확정 이상: **junction 26(4748296088) 1건만**. incoming road319→
  connecting road404(둘 다 직선), 위치는 정확히 일치(gap=0m)하는데
  방향만 37.86° 꺾임. 육안검증 최우선 대상
- 변환기 템플릿 아티팩트(결함 아님): 40개 connection에서 나온
  "3.35m 위치오프셋 + 26.565° 방향차"는 netconvert가 삽입한 고정
  템플릿 곡선(길이9.892401m, 계수 소수점까지 완전동일) 하나에서
  나온 결과로 확인. 실제 도로형상과 무관. 사용자의 최초 가설
  ("3.35 옆+6.7 앞 삼각형")은 기각, 실제로는 그 곡선 자체의
  chord길이(6.7m)였음 — 재조사 불필요 사항으로 기록

## 4. 시각화
`map/scripts/plot_roads.py` (matplotlib, `.venv-carla`에 설치)
- `map/docs/figures/01_full_network.png`: 전체 134 road, 등축비율,
  미터축, junction25개 표식
- `map/docs/figures/02_gate_library_segment.png`: road
  319/299/404/405/318 강조("확인된 구간만", 전체 정문-도서관
  경로 아님)
- `map/docs/figures/03_junction26_detail.png`: 위 5개 road 확대,
  진행방향 화살표
- 한글깨짐(DejaVu Sans) 발견 → Noto Sans CJK JP로 수정 후 재생성

## 5. 방위 검증
`map/scripts/geo_calibrate.py` (읽기전용)
- xodr geoReference가 "+proj=tmerc"뿐, lon_0/lat_0/ellps 없어
  직접 역변환 불가(offset값이 수백만 단위인 것이 근거)
- 대안: junction이름=원본OSM노드ID 성질 이용, 실측위경도-xodr좌표
  25쌍으로 유사변환(Umeyama) 적합
  - 회전각 -141.89° → **그림의 "위쪽"은 진북이 아님**(대략 남서향)
  - 스케일 0.7716 (xodr 1m ≠ 실제 1m)
  - 잔차 RMS 2.684m / 최대 6.010m (1.2~1.7km 지도 기준 0.2% 이내)
- 강조구간 자유단 재확인: road299/318이 junction11↔26 왕복쌍(길이
  거의 동일)이라 자유단은 정확히 junction1/junction11 둘뿐
- 질의점 4곳 위경도(소수점6자리):
  - junction1(4748296080, 원본OSM 직접조회, 오차0):
    lat=36.631513 lon=127.453602
  - junction11(4402717742, 원본OSM 직접조회, 오차0):
    lat=36.629468 lon=127.457141
  - 북쪽 고립덩어리 중심(연결요소분석, road1개 컴포넌트,
    변환추정): lat=36.624250 lon=127.462499
  - 도로망 전체 중심(변환추정): lat=36.627298 lon=127.456394

## 미해결·다음 세션 이월
- CARLA 단독 실행 avail 3GiB+ 미설명분(RSS+GTT로 4GiB만 설명됨) 원인
- 40건 템플릿 아티팩트의 근본원인(netconvert 소스/공식문서 미확인)
- 정문-도서관 전체 경로(392532207 등 포함 완전한 way목록) 미확정,
  오늘은 junction26 부근 구간만 확인
- 육안검증(3번,4번 항목) 전체 미실시

## 다음 세션 시작점
1. 대조군 재측정 생략, 처음부터 cbnu_internal_only.xodr로 CARLA 기동
2. 기동 1회로 메모리측정 + 육안검증(junction26 우선, 정문진입구간)
   동시 진행
3. 실행명령은 반드시 원문 그대로 기록(재발방지 규칙 적용)
