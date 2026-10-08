# 맵세션38 보고서 — 후문 r1391 마지막 시도 + 소품 소환 시험 (2026-10-08)

## 0. 선행 확인 (서버 전, 23:04)
- map_session38_report.md: 없었음(처음 실행)
- frozen_v1 bf835cdfad0cea65 일치, v2 후보 5240ca8b883e42c0 일치, test_drive.py f7b9eff66db29b98 일치
- D 유지: back slow={1391: 8.9}. 노선 전용 겨냥 거리 aim={} 빈 값
- 세션37 결정 기록 위치: map_session36_report.md 68~78행("세션37 결정 기록")
- 1.192m 정정 문장 위치: map_session35_report.md 100행(1.192m 출처), 112~113행(1.19m 정정)
- 예측 0.601m: logs/s36_spacing_model.log 11행 / 여유 0.7807m: map_session36_report.md 29행
- firefox 0, CarlaUE4 0 (첫 확인 pgrep -f firefox 는 명령 자신이 잡혀 2건 -> 이후 pgrep -f '[f]irefox' 형식, 사용자 지시)
- 기준선 free -h: avail 11Gi, swap used 0B

## 1부 결정: 주행 횟수 (2026-10-08 사용자 결정)
- 어긋남: 세션38 지시문은 "후문 3회, 3/3·1~2/3·0/3 판정", 세션37 기록(map_session36_report.md 71·78행)은 "후문 1회"
- 사용자 판단: 지시문 쪽 오류. 두 안(1회 / 3회)을 순서로 합친다
- 결정
  1. 첫 1회가 판정이다(세션37 기록대로). 충돌·미완주면 바로 F, 추가 주행 없음. 관측 이탈 vs 예측 0.601m 는 기록
  2. 첫 1회 통과면 같은 설정으로 2회 더. 3/3 이어야 "후문 완료". 2·3회째 실패면 경계선: 이탈 분포만 보고하고 멈춤, F 여부는 사용자가 정한다
  3. 모형 대조는 회차별 최대 이탈 전부로. 평균·최소·최대를 0.601m 와 비교
- 이유(사용자)
  - 실패 판정은 1회로 충분하다. 완료 선언은 반복 확인이 필요하다
  - 세션35 같은 설정 3회 이탈이 1.11~1.23m 로 0.12m 흔들렸고, 새 설정에서 모형은 검증된 적 없다
  - backlog 노선 검증 기준도 노선당 3회다

## 1-1. 설정 (r1391 한정) [확인]
- test_drive.py back 노선: aim={1391: 1.0}, spacing={1391: 1.0} 추가(새 필드 spacing, ROUTE_SPACING)
- 점 배치: s36_spacing_model.py 와 같게 했다. 기본 배치 첫 점(s=1.787)부터 1m씩, 끝 0.3m 전까지 = 16점(1.787~16.787)
  - 이유: 예측 0.601m 는 이 점 배치 기준이다. 코드의 기존 등분 중앙점 방식(18점, 0.496~17.370)이면 같은 모형 예측이 0.398m 로 달라진다(b=3.0 은 0.810m)
  - 사전에 정한 예측(0.601m)을 그대로 검증하려고 모형 배치를 따랐다
- 전역 PLAN_SPACING_M 4.0·전역 겨냥 거리 변경 없음
- 4 노선 교집합 검사(select_route, 시작 시 코드로): 간격 [] / 겨냥 [] / 감속 [] -> 비어 있음
- 백업 tests/test_drive.py.bak_s38 = f7b9eff66db29b98, 새 test_drive.py = 2935c016f9315b70
- dry-plan: 수정 전(bak_s38) 재생성 = 기존 파일과 동일 확인 뒤 수정본 실행
  - v1 6개(north·south·middle·yangseong·legacy·uturn) 변경 0, v2 4개(north·south·middle·yangseong) 변경 0
  - v2 back: r1391 점 5 -> 16 개, r1391 밖 352점 동일. 수정 전 파일은 logs/drive_plan_dry_back_map5240ca8b_pre_s38.csv

## 1-2. 주행 — 후문 3회 [확인] (logs/s38_runs_stdout.log, s38_v2_back_run{1,2,3}.log, s38_metrics.log)
- 서버: 사용자가 기동(cd ~/carla/CARLA_0.9.15 && ./CarlaUE4.sh -quality-level=Low -win), PID 4892
  - 기동 직후 23:19 avail 2.2Gi swap 1.6Gi, 2분 후 23:21 avail 2251Mi swap 1595Mi -> 기준(2048Mi/4096Mi) 안
- 실행: map/scripts/s38_back_run.sh(신규, s33_one_run.sh 에서 서버 기동·종료 뺀 것). 1회차만 v2 로드(test_load_map_878.py --wall-height 0 --xodr v2)
  - 로드: 서버 to_opendrive sha256 5240ca8b883e42c0 일치, road 917 / junction 112, spawn point 1302
  - 로드 직후 avail 2353Mi swap 826Mi, 2분 후 4760Mi / 824Mi
- 회차별
  - 1회차: 완주, 충돌 0, 276.0s, 궤적 1364.0m, 종점까지 0.95m. 주행 후 avail 4760Mi swap 822Mi
  - 2회차: 완주(정지 근접 판정: 속도<0.5m/s 가 t=276.35s 부터 5.0s, road1151 s=22.04, 정차점 1.06m), 충돌 0, 281.4s, 1363.9m. 주행 후 4670Mi / 822Mi
  - 3회차: 완주, 충돌 0, 275.5s, 1364.0m, 종점까지 0.98m. 주행 후 4680Mi / 821Mi
  - r1391 최대 이탈(차선 중심 대비, 왼쪽 t=0 가장자리 쪽, 전부 s=17.05): +0.626 / +0.626 / +0.624m, 차 왼쪽 끝 t=-0.15~-0.16(가장자리 안)
  - r1391 구간(171tick 씩) 속도 8.25~8.72km/h(D 8.9 유지), 조향 -0.580~-0.007(포화 0.8 미도달)
  - 목표점: r1391 진입 때 겨냥 3->1m, 계획점 1m 간격으로 하나씩 넘어감(목표 거리 약 3.0~3.5m), 차 s=15.198/15.199/15.21 에서 목표 r1952 s=1.387 로 전환, r1953(junction 106) 진입 때 1->3m 복원
  - 참고: 주행 CSV 에 r1947·1953·1951 이 잠깐 찍힘(junction 108·106 안 겹친 커넥터를 get_waypoint 가 고른 것으로 보임 [추정]). 경로 이탈 아님(완주·궤적 길이 같음)
  - 2회차 정지 근접 5s: 종점 앞 정지 판정 차이. 세션33 의 하네스 비결정성(대기 발동)과 같은 종류로 보임 [추정]. r1391 과 무관(r1391 지표 3회 같음)

## 1-3. 핵심 산출물: 관측 vs 예측 [확인]
- 예측 0.601m(큐 규칙 모형, s36_spacing_model.log, 최대 지점 차 s=15.2 -> 목표 r1952 s=1.387, 이탈 위치 s=17.03)
- 관측 회차별 최대: 0.626 / 0.626 / 0.624m -> 평균 0.626, 최소 0.624, 최대 0.626m
- 차(평균-예측) +0.025m, 회차 간 범위 0.002m. 이탈 위치 s=17.05(모형 17.03), 목표 전환 차 s=15.20~15.21(모형 15.2)
- 판정: 맞음. 세션36 모형 오차 크기(최대 0.115m) 안. 큐 규칙 모형 확정
  - 단, 모형은 점 배치에 민감하다(같은 간격 1m 라도 등분 중앙점 배치면 0.398m). 예측은 점 배치까지 고정해야 의미가 있다
- 세션35(D 조건) 3회는 1.11~1.23m 로 흔들렸는데 이번 3회는 0.002m. 이유는 확인 안 함 [미확인]

## 1-4. 판정
- 결정(위 "1부 결정") 1: 첫 1회 완주 -> 2: 같은 설정 2회 더 -> 3/3 완주
- 후문 노선 완료 (v2 back, r1391 한정 간격 1m + 겨냥 1.0m + D 8.9km/h)
  - 거리: 궤적 1363.9~1364.0m
  - 시간: 276.0 / 281.4(정지 근접 5s 포함) / 275.5s
  - 종점 도달: road1151 lane-1 s=23.1 기준 0.95 / 1.06 / 0.98m (후문B 추정 게이트, 392632034 위)
- F 해당 없음

---

# 2부 — 소품 소환 시험 (B1) [확인] — 상세 logs/s38_spawn_trial.md

## 2-1. 전제
- 서버 맵 v2(5240ca8b883e42c0) 확인, 재로딩 없음. 스크립트 map/scripts/s38_spawn_trial.py(신규). xodr·v2·test_drive.py 수정 없음
- 직선 구간: r1392(4 노선 + 후문 공통, 길이 56.074m, 방향 변화 0.2도), lane -1 오른쪽
- 3차 시도에서 완료. 1차는 내 스크립트 오류(road 밖 s 처리), 2차는 아래 발견 1 때문에 배치 실패. 둘 다 액터 0 으로 정리됨

## 2-2. 발견
1. static.prop.mesh 는 소환 뒤 set_transform 으로 움직이지 않는다(busstop 블루프린트는 움직임) [확인, 2차 시도 화면]
   -> 배치 스크립트는 자리를 먼저 계산·검사하고 그 자리에 소환해야 한다(props_plan 3-3 설계에 반영 필요)
2. bbox 는 0 은 아니지만 축·크기가 화면과 안 맞는다(건물 x 축 0.7~1.9m). 겹침 검사에 그대로 못 쓴다 [확인]
3. scale 은 동작한다: House01 scale 2.0 bbox 가 1.0 의 정확히 2배(0.707x15.436x12.646 -> 1.415x30.872x25.293), 화면도 큼
4. vehicle.mitsubishi.fusorosa 는 0.9.15 블루프린트 라이브러리에 있다(문서에서 빠졌을 뿐)
5. 브리지 sensor_mapping.yaml 은 이 PC 에 없다 -> LiDAR 는 지시문 기본값(64ch, 30만 점/s, 20Hz, 100m)

## 2-3. 측정 요약
- 소환: 8종 모두 성공(건물 3종 + scale 2.0, 나무 2, busstop, GroundCube). 장면 배치 6/8(House02·Apartment03 은 r1392 끝 너머 자리라 겹침 불합격, 소환 안 함)
- 메모리·FPS(동기 틱 100번): 소환 6개 전/후 렌더 켬 4.29 -> 4.04ms/틱, 렌더 끔 0.63 -> 0.63ms, avail 4452 -> 4480Mi, gtt 1477 그대로
  - 차이가 반복 흔들림(렌더 켬 약 0.4ms)보다 작다 = 6개로는 영향 측정 안 됨. 수십 개 규모는 안 재었다
- LiDAR 적중(10m / 20m): House01 628/337, House01x2 742/479, House02 433/244, Apartment03 639/515, Acer 108/86, Ash 512/311, busstop 125/32, GroundCube 43/8
- 셔틀 후보(자전거 모형 최소 회전 반경, 최대 조향각 API 값 70도라 비현실적으로 작음)
  - fusorosa 10.273x3.944x4.253m 축거 5.630m 2.049m / sprinter 5.915x1.988x2.561 축거 3.662 1.333m
  - prius 4.514x2.007x1.525 축거 2.819 1.026m / a2 3.705x1.789x1.549 축거 2.506 0.912m
- 스크린샷: docs/figures/s38_props_before.png, s38_props_after.png (같은 시점)

## 2-4. 결정표 (고르지 않음)
- 가: 패키지 건물 메시 그대로
  - 되는가: 됨(House01·02, Apartment03 소환, scale 동작). 단 bbox 못 믿음, 소환 뒤 못 옮김
  - 메모리·FPS: 6개(건물 2 포함)로 변화 측정 안 됨
  - LiDAR: 건물 10m 433~742 점, 20m 244~515 점
  - 생김새: 주택·아파트 모양(창·지붕), 캠퍼스 건물 모양과는 다름(s38_props_after.png)
- 나: 덩어리 메시
  - 되는가: GroundCube(약 1m 정육면체) 소환 됨. scale 은 세 축 같은 값 -> 한 개로는 건물 비율(길고 낮은 상자)을 못 만든다. 여러 개를 쌓거나 정육면체 그대로 키워야 함 [추정, 안 해 봄]
  - 메모리·FPS: 1개로는 측정 안 됨
  - LiDAR: 1m 크기 10m 43 점, 20m 8 점. 건물 크기 덩어리는 안 재었다
  - 생김새: 회색 작은 상자(s38_props_after.png 정류장 옆)
- 다: 혼합(랜드마크만 메시)
  - 되는가: 가·나 둘 다 소환되므로 가능
  - 메모리·FPS·LiDAR: 가·나 값의 조합. 따로 재지 않음
  - 생김새: 따로 찍지 않음
- 공통 미측정: 수십 개 규모의 메모리·FPS, 건물 크기 덩어리의 LiDAR

## 2-5. 뒷정리
- 띄운 액터 전부 destroy, 남은 차량·소품·센서 0(1부 하네스 차 a2 29 포함 정리)
- 설정 복원 synchronous_mode=False, no_rendering_mode=False, fixed_delta_seconds=None. 정리 후 avail 4464Mi swap 786Mi

---

# 3. 마감
- 해시: frozen_v1 bf835cdfad0cea65 불변, v2 5240ca8b883e42c0 불변, test_drive.py 2935c016f9315b70(세션38 변경, 백업 .bak_s38 = f7b9eff66db29b98)
- v2 동결 가능 조건: 충족
  - 근거: 세션33 마감 "남은 것: 후문 첫 주행(3-2)" -> 이번 3/3 완주. 4 노선 회귀(3-1, 개정 9-3)는 세션33 통과
  - 단서: 후문 완주는 r1391 한정 하네스 설정(감속 8.9km/h, 계획점 간격 1m, 겨냥 1.0m) 포함. 4 노선은 이 변경 뒤 재주행 안 함(dry-plan 변경 0 으로 확인)
  - 동결 선언은 하지 않음(사용자 결정)
- 새 파일: map/scripts/s38_back_run.sh·s38_back_summary.py·s38_spawn_trial.py, tests/test_drive.py.bak_s38,
  logs/s38_runs_stdout.log·s38_metrics.log·s38_v2_back_run{1,2,3}(_load).log·drive/target/collision_log_20261008_23{2341,2453,2538}·
  s38_spawn_trial(.log·.json·.md·_try1·_try2)·s38_props_after_try2.png·drive_plan_dry_back_map5240ca8b_pre_s38.csv, figures/s38_props_before·after.png
- 변경 파일: tests/test_drive.py, logs/drive_plan_dry_back_map5240ca8b.csv(재생성)
- git·commit·push·sync --apply 없음. xodr 수정 없음. CARLA 종료는 사용자
- 다음 시작점: v2 동결 여부 사용자 결정 + 2부 결정표에서 건물 표현(가·나·다) 선택
