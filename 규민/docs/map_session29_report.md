# 맵세션29 보고서 (2026-10-07) — Lanelet2 도구 시험

상세: map/docs/logs/s29_lanelet2_trial.md. 표기 [실측] 직접 실행 / [코드] 패키지 소스 확인 / [추정] / [불명]

## 시작·환경
- frozen_v1 sha256 bf835cdfad0cea65 확인. 입력은 복사본만(scratchpad/s29). CARLA 서버·재변환·Autoware 실행 없음
- 디스크 여유 78G, pypi 접속 가능, 파이썬 3.10.12, available 9.8Gi

## venv (다음 세션에 이어서 쓴다. 지우지 않음)
- 경로: ~/lanelet2_trial_venv (/home/gyumin/lanelet2_trial_venv, 프로젝트 밖). 크기 1.1G
- 내용: commonroad-scenario-designer 0.8.5(commonroad-io 2024.3), lanelet2 1.2.3, pip 26.2.1. 의존성 61개
- 실행: `~/lanelet2_trial_venv/bin/python map/scripts/s29_lanelet2_trial.py <작업> <xodr 복사본> <꼬리> [--autoware --local]`
- 주의: crdesigner CLI 는 시작 시 TypeError 로 죽는다(typer 0.9.4 + click 8.5.0 추정). 파이썬 API 로만 쓴다
- 설치 용량(풀린 크기 약 1,003MB)이 1GB 기준을 넘어 사용자 승인 후 설치. sudo·apt 없음, --user 없음
- 변환 산출물(.osm 21~48MB, .cr.xml 18MB)은 scratchpad 에만 있다. 세션이 끝나면 사라진다. 다시 만드는 데 약 20초

## 결과 요약 [실측]
- 변환: 878 road 전체 한 번에 성공. 18.7s(기본) / 20.8s(autoware 설정), 메모리 최대 1.3GiB, logging 경고 0
- 구조: lanelet 1370 = 차량 872 + 보행 498. xodr 주행 차선 878·인도 504 전부 lanelet 에 대응(차선 중심 표본 3점이 0.5m 안)
- 회차 연결로 24개 모두 lanelet 으로 나옴. 회차 전용 junction 24 는 intersection 에 안 들어감(71/95 만 들어감)
- 연결: 공식 Lanelet2 로 읽으면 load 오류 0, 그러나 차량 lanelet 872 중 뒤 없음 201, 정문에서 도달 2/872
  - 201 중 154 는 같은 자리(0.001m 안)인데 왼쪽 경계 점 id 가 달라 연결로 인정 안 됨. 41 은 3m 이상, 6 은 0.05~3m
  - CommonRoad 중간 단계에서는 뒤 없음 16 뿐 -> 끊김은 CommonRoad -> Lanelet2 단계에서 생김
- 고도: 도구가 무시(아래 2-2)
- Autoware 요소: ele 형식 충족(autoware 설정, 값 0). turn_direction 0, regulatory 0, speed_limit 0
  - local_x/local_y 가 EPSG:3857 값 그대로(미터 지역 좌표 아님), 노드마다 mgrs_code
- 반쪽 도로: 쌍이 서로 이어지지 않은 단방향 lanelet 둘. 맞은편 이웃 0
- 검증기 autoware_lanelet2_map_validator: 호스트 미보유. Docker 이미지 안 여부 [불명]
- 도구 평가: 쓸 수 있으나 그대로는 Autoware 에 못 쓴다. 연결 끊김이 풀리기 전에는 경로 계획 불가

## 2-1. 공수 추정의 성격 변화
- 숫자: 세션27 7~10 작업일 -> 세션29 7~11 작업일. 거의 같다. 둘 다 추정
- 내용은 다르다
  - 세션27: "도구가 쓸 만한가" 가 미지수였고, 실패하면 878 lanelet 을 손으로 그리는 최악이 숨어 있었다(분량 근거 없음)
  - 세션29: 자동 변환이 실측 20초로 확인됐다. 878 차선 전부 lanelet 이 된다
  - 그래서 최악 시나리오("878개 손으로 그리기")가 제거됐다
  - 대신 새 작업이 생겼다: 연결 복구(끊김 201, 실측), ele 후처리(필요 확정, 실측), 좌표 태그 정리(실측)
- 이번 성과는 평균이 줄어든 것이 아니라 분산이 줄어든 것이다. 7~11일의 위쪽 꼬리(손으로 그리기)가 잘렸다
- 검증기 통과 시간은 이 추정에 포함되지 않았다. 검증기를 아직 못 돌려 반복 횟수를 알 수 없다. 실제는 더 늘 수 있다

## 2-2. 고도 전제 수정 (이전 전제가 틀렸다)
- 이전 전제: xodr 에 고도를 넣으면 Lanelet2 변환 때 ele 가 자동으로 채워질 것
- 실제[실측]: 도구가 xodr 고도를 무시한다
  - 878 road 전부 elevation 을 a=5.00, b=0.02 로 바꾼 시험 사본으로 변환 -> ele 전부 "0"
  - Lanelet2 출력이 고도 없는 것과 바이트까지 같다
  - [코드] 파서는 elevationProfile 을 읽지만 기하 생성에서 쓰지 않는다. 점이 2차원이라 ele=0 이 들어간다
- 결과: ele 는 Lanelet2 쪽 별도 후처리로 채워야 한다. xodr 고도를 읽어 lanelet 점에 매핑하는 작업이다
- 작업 순서는 여전히 고도 먼저가 유리하다. ele 후처리 때 xodr 에 실제 고도가 있으면 그 값을 쓰고, 없으면 0 을 쓰게 되기 때문이다
- 단 "자동으로 따라온다" 는 기대는 폐기한다

## 2-3. 위치 오차 1.69m 의 성격
- 잰 것: 정문 r1247 lane -1 차선 중심점(carla 오프라인 waypoint)에서 가장 가까운 Lanelet2 경계 노드까지 1.69m
- 이 값이 설계상 정상 오프셋인지 실제 오차인지 구분되지 않았다. 정상 오프셋 여부 미확인
  - 가능한 설명 1: 차선 중심에서 경계선까지는 반폭(3.35/2 = 1.675m)이라 그만큼 떨어지는 것이 정상일 수 있다. 1.69 와의 차 0.015m 는 검토 안 함
  - 가능한 설명 2(사용자 지적): xodr 기준선(도로 중심)과 lanelet 중심선(차선 중심)이 half-road 구조에서 반폭만큼 어긋나는 것이 정상일 수 있다
  - 둘 중 어느 쪽인지, 또는 다른 오차가 섞였는지 가르지 않았다
- 참고로 함께 잰 것: 차선 중심 표본 3점이 lanelet 중심선 0.5m 안(878/878). 0.5m 는 대응용 허용치이고 실제 거리 값은 기록하지 않았다
- 위치 정확도는 확인된 것으로 쓰지 않는다. 다음에 차선 중심 -> lanelet 중심선 거리를 숫자로 뽑아야 한다

## 2-4. 다음 최우선 작업: 점 병합 후처리 시험
- 끊김 201 중 154 가 "같은 자리 다른 점". 허용오차 안의 점을 합치는 후처리로 풀릴 가능성이 크나 미시험
- 이것이 현재 추정의 가장 큰 미지수다. 결과에 따라 7~11일 추정이 크게 바뀐다
  - 풀리면: 연결 복구가 스크립트 하나로 끝나고 남은 47 만 조사
  - 안 풀리면: 연결을 손으로 잇거나 다른 경로를 찾아야 한다(분량 근거 없음)
- 예상 분량: 반나절 이하(추정). 확인 기준: 정문에서 도달 가능 lanelet 수(지금 2/872)와 xodr 의 826/878 비교
- 나머지 47 은 원인 미확인으로 남긴다(41 은 3m 이상, 6 은 0.05~3m)

## 공수 표 요약 (상세는 s29_lanelet2_trial.md Phase 4)
- 자동: 설치(실측 수 분), 변환(실측 20초), 점 병합(추정 반나절~1일), turn_direction(추정 반나절, CommonRoad 교차로 정보 이용),
  ele 후처리(추정 반나절~1일, DEM [불명]), 투영 정리(추정 반나절), 속도·보행 정리(추정 2~3시간)
- 사람: 남은 끊김 47 조사, U턴·회차 처리 결정, 횡단보도·정지선·우선권, 라바콘·볼라드·게이트 규칙, Autoware 경로 판정
- 불명: 검증기 통과, 우선권 범위, DEM 원천, 이 최소 지도로 Autoware 가 경로를 내는지

## 마감
- frozen_v1 sha256 bf835cdfad0cea65 (시작·끝 같음, 불변)
- 새 파일: map_session29_report.md, logs/s29_lanelet2_trial.md, logs/s29_convert_full_plain·full_autoware·elev_autoware.log,
  logs/s29_eval.log, s29_lanelet2_load.log, s29_lanelet2_gaps.log, s29_halfroad.log,
  scripts/s29_lanelet2_trial.py, s29_xodr_lanes.py, s29_lanelet2_eval.py, s29_lanelet2_load.py, s29_lanelet2_gaps.py, s29_halfroad.py
- 수정 없음: map/maps, map/data, Autoware 저장소
- venv: ~/lanelet2_trial_venv 유지
- 하지 않음: 점 병합 시험, ele 후처리, 수작업 보정, 재변환, 서버

## 정정 (2026-10-07 세션29 마감 뒤, 위 본문은 그대로 둔다)
- 1.69m 는 위치 정확도의 증거가 아니다. 차선 중심에서 그 차선 경계 노드까지 잰 값이라 반폭(1.675m)과 비교한 "폭" 측정이다
  - 맞는 표현: 차선 폭이 보존됨(1.69 vs 1.675, 차 0.015m, 노드 한 개 기준)
  - 본문의 "노드 위경도가 실제 위치에 맞다", "위치도 맞다" 는 이 값으로는 근거가 없다. 철회
- 위치에 대해 실제로 잰 것은 대응 검사 하나뿐이다: xodr 차선 중심 표본 3점(s=시작·중간·끝)이 lanelet 중심선 0.5m 안(878/878, 인도 504/504)
  - 즉 "모든 차선이 0.5m 안" 까지는 확인. 실제 거리 값(평균·최대)은 기록하지 않았다
  - 그래서 위치 정확도는 0.5m 상한만 있고 실제 값은 미측정. 정밀도 판정은 미측정으로 둔다
- 다음: lanelet 중심선 <-> xodr 차선 중심선 거리를 값으로 뽑는다(s29_lanelet2_eval.py 의 대응 거리 출력만 추가하면 됨, 미구현)
- 스크립트 위치 확인: 변환·평가 스크립트 6개는 처음부터 map/scripts/ 에 작성했다(scratchpad 아님)
  - 변환 호출 s29_lanelet2_trial.py / 끊김 분석 s29_lanelet2_gaps.py / 위치 비교 s29_lanelet2_eval.py + s29_xodr_lanes.py /
    로드·경로 s29_lanelet2_load.py / 반쪽 도로 s29_halfroad.py
  - scratchpad 에는 산출물(.osm, .cr.xml, 복사본 xodr, xodr_lanes.csv)과 pip 설치 기록만 있다
  - git: campus_mobility_sim 은 git 저장소가 아니다. 커밋은 하지 않았다(팀 저장소 speedwagon_cbnu 반영은 사용자 결정)

## 정정 2 (위 정정의 "미측정" 표현 보완)
- 위치 정확도: 정밀도는 미측정. 다만 878/878 차선이 0.5m 안에 들어와 "크게 어긋나진 않았다" 고는 말할 수 있다
  - 0.5m 는 차선 폭 3.35m 의 약 15%
- 동기화: 커밋은 이번에 하지 않는다. 점 병합 시험 뒤 한 번에 올린다
  - 동기화 시점: 점 병합 시험 끝 / 재변환 + 노선 재검증 끝 / Lanelet2 단계 끝 / 제출 전. 매 세션 하지 않음
  - 도구: map/scripts/sync_to_repo.sh (기본 미리보기, --apply 때만 복사, git 명령 없음). 작성만 하고 실행 안 함(문법 검사만)
  - 저장소에서 따로 고친 field_survey 2개는 보호 목록(덮어쓰지 않음). 작업 폴더 10-06 의 6·7절(세션28 정정·후문 확정)은
    저장소 정리본에 없으므로 동기화 때 손으로 옮겨야 한다
