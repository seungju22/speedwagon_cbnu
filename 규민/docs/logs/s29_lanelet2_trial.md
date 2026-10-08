# s29 Lanelet2 도구 시험 — CommonRoad Scenario Designer 0.8.5 (2026-10-07, 서버·재변환 없음)

표기: [실측] 이번에 직접 실행·측정 / [코드] 설치된 패키지 소스를 읽어 확인 / [추정] 해보지 않은 짐작 / [불명] 확인 못 함
입력은 frozen_v1 복사본(scratchpad/s29/frozen_v1_copy.xodr, sha256 bf835cdfad0cea65 = 원본과 같음). 원본은 해시만 읽음
시험 venv: ~/lanelet2_trial_venv (프로젝트 밖, 지우지 않음). 호스트 파이썬·--user 설치 없음. sudo 없음
변환 산출물(.osm, .cr.xml)은 scratchpad/s29 에만 있다(용량 크고 세션 끝나면 사라짐). 다시 만들려면 아래 명령으로 약 20초

## Phase 0 사전 확인 [실측]
- frozen_v1 sha256 bf835cdfad0cea65
- 디스크 / 여유 78G (설치 전), 77G (설치 후)
- 네트워크: pypi.org 접속 HTTP 200
- 파이썬: 시스템 3.10.12. crdesigner 요구 3.10~3.13 범위 안
- 메모리: available 9.8Gi (시작 시)

## Phase 1 설치 [실측]
- venv 생성 -> pip 22.0.2 를 venv 안에서 26.2.1 로 올림
- 설치 전 의존성: `pip install --dry-run --report` -> 61개 패키지, 다운로드 약 335MB(PyPI 크기 합). 휠 실제 다운로드 320M
  - 휠 안 파일 크기 합(풀린 크기) 약 1,003MB -> 1GB 기준 초과로 사용자에게 먼저 보고, "전부 설치" 승인 받음
  - 큰 것: libsumo 265MB, PyQt6-Qt6 224MB, scipy 121MB, numpy 59MB, sumo-data 50MB. GUI·SUMO 계열 합 318MB 는 xodr->Lanelet2 에 직접 안 씀 [코드: map_conversion_interface 가 commonroad_sumo 를 import 하므로 빼면 import 실패 가능, 추정]
- 첫 시도 실패: `--no-index --find-links` 로 오프라인 설치하니 sdist(antlr4-python3-runtime 등 4개) 빌드에 setuptools 가 없어 실패
  - 도구 문제가 아니라 내 설치 방식 문제. index 허용으로 다시 설치 -> 성공(21초)
- 결과: commonroad-scenario-designer 0.8.5, commonroad-io 2024.3. venv 크기 1.1G
- sudo·시스템 패키지(apt) 필요 없음. 빌드 단계 없음(sdist 4개는 순수 파이썬)
- 문제 1개: CLI 가 시작하자마자 죽는다
  - `crdesigner --help` -> `TypeError: Secondary flag is not valid for non-boolean flag.`
  - 원인 [추정]: 고정된 typer 0.9.4(<0.10 제한) 와 새 click 8.5.0 의 비호환. click 을 낮추면 될 수 있으나 시도 안 함
  - 우회: 파이썬 API(map_conversion_interface 의 opendrive_to_commonroad + CR2LaneletConverter)를 직접 불렀다. 공개 함수 opendrive_to_lanelet() 과 같은 두 단계다
- 결과 판정 확인용으로 공식 Lanelet2 파이썬 라이브러리 lanelet2 1.2.3(PyPI 휠 1개, 의존성 없음)도 같은 venv 에 설치

## Phase 2 변환 [실측]
명령(시험 venv):
`~/lanelet2_trial_venv/bin/python map/scripts/s29_lanelet2_trial.py <작업> <xodr 복사본> <꼬리> [--autoware --local]`
- 전체 878 road 한 번에 성공. 작은 조각으로 자를 필요 없었다
- 기본 설정(full_plain): 1단계 xodr->CommonRoad 15.5s, 2단계 CommonRoad->Lanelet2 3.2s, 합 18.7s. 최대 RSS 864MiB
  - 출력 21,464,869B(21MB). CommonRoad 중간 17,947,406B
- Autoware 설정(full_autoware, autoware=True, use_local_coordinates=True): 15.7s + 5.1s = 20.8s. 최대 RSS 1,323MiB. 출력 47,519,300B
- 경고·오류: logging WARNING 이상 0건. python DeprecationWarning 878건 = road 당 1건, 전부 같은 내용
  ("Function precalculate is called but its results ... not used", roadPlanView.py:300). 결과와 무관한 내부 경고로 보임 [추정]
- 로그: s29_convert_full_plain.log, s29_convert_full_autoware.log

## Phase 3 결과 평가
도구: s29_lanelet2_eval.py(s29_eval.log), s29_lanelet2_load.py(s29_lanelet2_load.log), s29_lanelet2_gaps.py(s29_lanelet2_gaps.log),
s29_halfroad.py(s29_halfroad.log), xodr 차선 표본은 s29_xodr_lanes.py(.venv-carla, carla 오프라인 Map)

### 3-1 구조 [실측]
- lanelet 1370 = 차량(subtype road) 872 + 보행(walkway) 498
  - xodr: 주행 차선 878, 인도 차선 504
  - xodr 차선 -> lanelet 대응(차선 중심 표본 3점이 lanelet 중심선 0.5m 안): 주행 878/878, 인도 504/504 전부 대응
  - lanelet 하나에 xodr 차선 여러 개가 들어간 경우: 주행 5개 lanelet(878 -> 872), 인도 5개(504 -> 498). 이어진 road 를 합친 것으로 보임 [추정]
  - Town01 처럼 "차선 하나 = lanelet 하나" 가 거의 맞다(878 중 872, 합쳐진 것 외 1:1)
- 위치: 노드 위경도가 실제 위치에 맞다. 정문 r1247 차선 중심에서 최근접 경계 노드 1.69m(반폭 1.675m 안팎이 정상)
  - xodr header 의 offset(521.51, 493.61)을 도구가 반영했다. 오프셋을 무시했다면 718.1m 어긋났을 것
- 교차로: CommonRoad 단계에 intersection 69. xodr 커넥터 있는 junction 95 중 71 이 들어감(3개 intersection 은 junction 2개씩 합침)
  - 빠진 24 = 회차 전용 junction 24개(j2 27 35 37 39 41 62 64 65 66 71 72 73 80 81 87 94 96 100 102 103 106 108 109, s28 틀 목록과 같음)
  - Lanelet2 출력에는 교차로를 나타내는 요소가 없다: turn_direction 0, regulatory_element 0
  - 단 CommonRoad 중간 파일에는 intersection 마다 successorsLeft / Right / Straight 가 있다(590줄). turn_direction 을 여기서 뽑을 수 있다 [추정, 미구현]
- 차선 연결 — 여기가 가장 큰 문제
  - CommonRoad 단계: 차량 lanelet 중 앞 없음 15, 뒤 없음 16. xodr 의 막다른 끝 16(일방 r1333·r1344 + 정의 없는 junction 14)과 수가 같다(하나하나 대응은 미확인)
  - Lanelet2 단계(공식 라이브러리 routing graph, germany vehicle 규칙): 차량 lanelet 872 중 뒤 없음 201, 앞 없음 203
  - 정문 lanelet 에서 도달 가능 2/872. xodr 기준 도달 826/878(s23)과 크게 다르다. 이대로는 경로 계획이 안 된다
  - 원인 분류(뒤 없는 201): 154 는 끝점과 다음 lanelet 시작점이 0.001m 안으로 같은 자리인데 왼쪽 경계 점 id 가 달라 연결로 인정 안 됨
    (오른쪽 경계 점은 공유). 41 은 3m 이상 떨어짐, 6 은 0.05~3m
    -> CommonRoad -> Lanelet2 변환에서 같은 자리 점을 하나로 합치지 못한 결함. 점 합치기 후처리로 154 는 풀릴 가능성이 높다 [추정, 미시험]
  - 공식 라이브러리 load 오류 0, routing graph checkValidity 문제 0. "읽히지만 이어지지 않는" 지도다

### 3-2 고도 [실측 + 코드]
- 기본 설정 출력: ele 태그 0개(노드 153,033 중). [코드] 값이 0 이면 autoware=False 일 때 태그를 쓰지 않는다(lanelet2.py Node)
- Autoware 설정 출력: 모든 노드에 ele="0"(153,033/153,033). 오류 없음
- 고도를 넣은 복사본으로 다시 변환: 878 road 전부 elevation 을 a=5.00, b=0.02(2% 오르막)로 바꾼 시험 사본(frozen_v1_copy_elev_test.xodr)
  - 결과 ele 전부 "0". Lanelet2 출력 파일이 고도 없는 것과 바이트까지 같다(cmp 동일). CommonRoad 중간 파일도 줄 순서 차이뿐(정렬 비교 동일)
  - [코드] xodr 파서는 elevationProfile 을 읽지만(opendrive_parser), 기하 생성(opendrive_conversion)에는 elevation 을 쓰는 곳이 없다. 점이 2차원이라 cr2lanelet 이 ele=0 을 넣는다
- 결론: 이 도구로는 xodr 의 고도가 Lanelet2 로 따라오지 않는다. "고도를 먼저 넣고 한 번만 변환" 은 성립하지 않는다
  - 고도는 Lanelet2 를 만든 뒤 노드마다 ele 를 채우는 후처리가 따로 필요하다(노드 위경도로 DEM 을 바로 읽거나, xodr 고도를 위치로 대응) [추정, 미구현]
  - 이 후처리는 xodr 고도와 무관하게 할 수 있다. 그래서 CARLA 용 xodr 고도 작업과 Lanelet2 고도 작업은 같은 원천(DEM)을 두 번 쓰는 별개 작업이 된다 [추정]

### 3-3 Autoware 필수 요소 (수동 점검, 검증기 미보유)
- 검증기: autoware_lanelet2_map_validator 호스트에 없음(파일 검색 0). Autoware 1.9.0 repos 에도 없음.
  Docker 이미지 universe-humble-1.9.0 안에 있는지는 컨테이너를 열어야 해서 미확인 [불명]
- ele: autoware 설정이면 전 노드에 있음(값 0) -> 형식은 충족, 값은 평지
- turn_direction: 0 (교차로 lanelet 631 이 CommonRoad 에서는 intersection 유형인데 Lanelet2 에 태그로 안 옮겨짐) -> 미충족
- speed_limit 0, regulatory_element 0(정지선·횡단보도·우선권 없음) -> 미충족(우리 맵이 원래 없음)
- 차선 경계 태그: line_thin + solid / solid_solid / dashed, autoware 설정이면 lane_change yes 97 / no 2,145 -> 있음
- 좌표 태그 [실측, 영향은 추정]
  - local_x / local_y 값이 EPSG:3857(웹 메르카토르) 좌표 그대로다(예 14188075.86, 4387863.82). 미터 단위 지역 좌표가 아니다(위도 36.6도에서 약 1.25배 늘어난 값)
  - mgrs_code 를 노드마다 1m 격자 코드로 붙인다(예 52SCF6174655106). Autoware 는 map_projector_info.yaml 로 투영을 정한다 [확인-세션27]
  - 위경도 자체는 맞으므로 MGRS 투영으로 읽으면 위치는 맞을 것 [추정]. Local 투영으로 local_x 를 쓰면 틀린다 [추정]
- 보행 lanelet 498(walkway): 인도 차선이 전부 lanelet 이 됐다. Autoware 에 둘지 뺄지 결정 필요

### 3-4 우리 맵의 특수 사정 [실측]
- 반쪽 도로(단방향 2개): 각각 따로 된 단방향 lanelet 이다
  - 차량 lanelet 두 개가 왼쪽 경계를 같은 선으로 공유한 경우 0. 같은 자리(0.05m 안)에 다른 선으로 겹쳐 있는 lanelet 704(교차로 겹침 포함 상한)
  - routing graph 왼쪽 이웃 0. 맞은편 차선 관계가 지도에 없다. 우리 노선은 맞은편 차선을 쓰지 않으므로 경로 계획에는 직접 문제 없을 것 [추정]
  - 차량-보행 lanelet 은 경계 선 498개를 공유(인도는 차로 오른쪽에 붙음)
- 회차 연결로: 틀 24개 전부 lanelet 으로 나왔다(중심선 0.01m 안, CommonRoad 유형 urban+intersection). 정문 j1 r1915·j107 r1916 회차도 나옴
  - 단 회차 전용 junction 24 는 intersection 에 안 들어갔다. Autoware turn_direction 에 U턴 값이 없다(세션27) -> 처리 결정 필요
- 변환 시간: 878 road 에 약 20초, 메모리 최대 1.3GiB. 재변환(9 way 추가) 후 다시 돌려도 부담 없음

## Phase 4 공수 추정 갱신
형식: 단계 / 자동·수작업 / 분량 / 근거
1. 도구 설치·환경 / 자동 / 의존성 확인·다운로드·설치 수 분(설치 자체 21초) + CLI 고장 우회 조사 / 실측. venv 1.1G, sudo 없음
2. xodr -> Lanelet2 변환 / 자동 / 약 20초 / 실측(878 road 전체)
3. 차선 연결 보정 / 자동(스크립트) + 남은 것 수작업 / 154개(같은 자리 다른 점): 점 합치기 스크립트 반나절 + 확인 반나절. 나머지 47개: 원인 조사 반나절~1일 / 개수 실측, 시간 추정. 47 중 16 은 원래 막다른 끝일 가능성 [추정]
4. 교차로 보정(turn_direction) / 자동(CommonRoad 중간 파일의 Left/Right/Straight 이용) / 반나절 / 재료 있음은 실측, 스크립트는 추정. 교차로 lanelet 631
5. U턴·회차 처리(회차 lanelet 24 + 정문 2) / 사람 판단 + 자동 / 반나절 / 추정. Autoware 값 체계에 U턴 없음
6. 정지선·횡단보도·우선권 / 수작업 / 횡단보도 10곳 내외 3~5시간 + 우선권은 범위에 따라 1~2일 / 추정(세션27 과 같음). 도구가 만들어 주는 것 0 은 실측
7. 고도(ele) / 자동(후처리 스크립트) / 반나절~1일 + DEM 확보 [불명] / 도구가 고도를 무시함은 실측, 후처리 분량은 추정
8. 좌표·투영 정리(local_x 3857 문제, mgrs_code, map_projector_info) / 자동 + 판단 / 반나절 / 문제 존재는 실측, 해결 분량 추정. 어느 투영이 맞는지 [불명]
9. 속도 제한·보행 lanelet 정리 / 자동 / 2~3시간 / 추정
10. 회차 구역·금지 구역(라바콘, 볼라드, 게이트) / 수작업 / 1일 / 추정(세션27 과 같음)
11. Autoware 검증기 통과 / 실행 + 반복 수정 / [불명] / 검증기 미보유. 이미지 안 여부도 미확인
12. Autoware 로드·경로 계획 확인 / 실행 / 1~2일 / 추정(세션27 과 같음). 메모리 제약
합: 1~10, 12 를 더하면 약 7.4~10.5 -> 약 7~11 작업일 [추정]. 11 은 넣지 못했다

세션27 추정(7~10일)과 비교
- 줄어든 불확실성: "도구가 우리 맵에 먹는가" 는 답이 나왔다. 먹는다. 878 차선 전부 lanelet 이 되고 위치도 맞다
  -> 세션27 이 걱정한 최악(878 lanelet 손으로 그리기)은 사라졌다
- 새로 생긴 일: 연결 끊김 201(실측), 고도 후처리(실측으로 필요 확정), 좌표 태그 문제(실측)
- 그래서 범위 자체는 거의 그대로다(7~10일 -> 7~11일). 정확해진 것은 "무엇에 시간이 드는가" 이지 총량이 아니다
- 여전히 모르는 것: 검증기 반복 횟수, 우선권 범위, DEM 원천, Autoware 가 이 최소 지도로 경로를 내는가

## 결론
- 혼자 맡으면: 약 7~11 작업일(추정). 검증기 단계는 미포함이라 늘어날 수 있다
- 도구·스크립트로 줄일 수 있는 부분: 변환(실측 20초), 점 합치기로 연결 복구, turn_direction(CommonRoad 교차로 정보), ele 채우기, 투영·속도 태그
- 사람이 반드시 해야 하는 부분: 횡단보도·정지선 위치, 교차로 우선권, 회차·U턴 처리 결정, 라바콘·볼라드·게이트 같은 현장 규칙, Autoware 에서 경로가 맞는지 판정
- 도구 평가: "쓸 수 있으나 그대로는 Autoware 에 못 쓴다". 실패도 성공도 아닌 중간. 연결 끊김 201 이 해결되기 전에는 경로 계획이 안 된다(정문에서 2/872)

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
