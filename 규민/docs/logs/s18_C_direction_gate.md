# s18 C. 경로 방향 검증과 정문 인도 침범 (야간 배치, 서버 없음, 코드 수정 없음)

## C-1. 진행 방향 검증 — 닫음
상세: map/docs/logs/s18_B_direction.md (앞선 지시 버전에서 같은 내용으로 작성), 로그 s18_B_direction.log
- 878 road 전부 오른쪽 driving lane 1개(lane -1), 왼쪽 driving 0. laneSection 1개
- legacy 15 / north 17 / south 15 / middle 19: road 마다 (a) lane -1 driving (b) 앞 road 끝에서 next() 로 이 road 의 lane -1, s≈0 도착
  (c) 실주행 s 증가 — 역방향 0
- 루트 체인은 lane -1 next() 로만 만들어 구조상 역방향이 들어갈 수 없다. 무방향 그래프(networkx.Graph)는 OSM 단계 분석에만 쓰였다
- 판정: 무방향 그래프 문제는 현재 4개 루트에 영향 없음. 세션11 부터의 미해결 항목을 닫는다
- 재변환 뒤 후문 루트도 같은 next() 체인 방법을 쓸 것

## C-2. 정문 부근 인도 침범
스크립트: scratchpad/gate_sidewalk_s18.py + 인라인, 로그 map/docs/logs/s18_C_gate_sidewalk.log (north CSV 기준, 세 루트 공통)

### 침범 구간(north, 28 tick)
1. road1247 s=0.00~0.41, t=0.05~1.10 (22 tick): 정지·출발 순간. 차 중심 lat 0.00, 꼭짓점 1개가 Driving 밖, 중심은 인도 아님
2. road1356 s=33.9~34.4, t=39.10~39.20 (3 tick): 차 중심 lat +0.47~+0.57m(차선 안, 반폭 1.675)인데 중심이 인도로 조회.
   조회된 인도 = road1916 lane -2 (j107 의 다른 커넥터 1916 의 인도). t=39.05~39.65 동안 계속 겹침(판정은 3 tick 만 표시)
3. road1917(j107 커넥터) s=2.0~2.45, t=39.90~40.00 (3 tick): lat +0.14~+0.37m, 꼭짓점 1개가 Driving 밖

### 해당 구간 기하
- road1247 / 1356 / 1917 / 1355 / 1914 모두: laneOffset 없음, lane -1 driving 3.35m(상수), lane -2 sidewalk 2.80m(상수), 왼쪽 lane 없음
- 차량 bbox 반폭 0.894m(전폭 1.79m) < 차선 3.35m. 차선이 차체보다 좁은 구간 아님
- 곡률: road1356 회전 -10.1°/37m, R2min 13.1m. road1247 -50.5°/153m, R2min 10.3m(s=95.5, 침범 구간 아님). 급커브 아님
- road1247 s=0 앞은 junction 커넥터 road1380(s=8.9). 스폰 위치(217.85,-1087.41)가 road1247 s=0.00 정확히 도로 시작점
- j107 결함(핵심 발견):
  road1356 끝 yaw 29.04° 위치 (310.05,-933.87)
  road1917 시작 yaw 66.90° 위치 (309.34,-934.66) — 앞 road 끝과 1.06m 어긋남
  road1917 끝 yaw 66.90° (310.50,-931.95), 길이 2.97m 직선
  road1355 시작 yaw 35.98° (311.07,-931.23)
  => 2.97m 짜리 커넥터가 앞뒤 도로보다 약 31~38° 돌아간 짧은 지그재그. 커넥터 자체 회전은 0° 라 R2 스캔(양끝 1m 제외)과
     앞뒤 접선(+6.9°)으로는 안 잡혔다. 같은 자리에 j107 의 다른 커넥터 1916 의 인도 lane 이 주행 차선 위로 겹친다
- additional_width 0.6: CARLA 공식 설명은 "junction lane 에 더하는 폭"(OpendriveGenerationParameters, 근거
  https://carla.readthedocs.io/en/0.9.15/python_api/#carla.OpendriveGenerationParameters). 메시만 넓히고
  waypoint lane_width(3.35)와 lane 조회에는 영향이 없다고 본다(가정, 코드 미확인). 인도 판정은 lane 조회 기반이라
  0.6 은 이번 침범 기록의 원인이 아니다. 메시 쪽으로는 junction 을 넓혀 오히려 유리

### 원인 후보(좁힌 결과)
1. 스폰 위치가 road1247 s=0(도로 시작점) — 차 뒤쪽 1.85m 가 도로 시작 전 junction 영역에 걸려 꼭짓점 1개가 Driving 밖으로 조회.
   실제 인도 침범 아님(기록 아티팩트). 고치는 쪽: 주행 파라미터(스폰을 s>=2.0m 로, 또는 첫 tick 들 제외). 맵 수정 불필요
2. j107 커넥터 r1917 의 기하 결함(1.06m 어긋남 + 31~38° 꺾인 2.97m 커넥터) + 이웃 커넥터 1916 인도의 겹침.
   고치는 쪽: 맵 수정(재변환 시 j107 부근 OSM 노드 정리 또는 평활화 대상 추가). 주행 파라미터로는 못 고침(기록은 계속 남음)
3. road1356 에서 차가 오른쪽으로 +0.9m 치우침(north +0.92, south +0.95, middle +0.93) — j107 지그재그를 계획점(4m 간격)이 따라가며
   미리 오른쪽으로 당겨진 것으로 보임(가정). 고치는 쪽: 1차는 맵(2번 해소 시 같이 사라질 가능성), 2차는 주행 파라미터(커넥터 계획점 간격 --conn-spacing).
   단 A-4 지시대로 이번에는 파라미터를 바꾸지 않음
- 급커브는 원인이 아니다(해당 구간 R2min 10m 이상, 회전 10° 이하)

## C-3. 잔여 61개 지점 교차 확인
파일 위치 정정: 잔여 목록은 map/docs/logs/session11_residual_sites.csv (지시문의 docs/logs/ 경로에는 없음)
로그: map/docs/logs/s18_C_residual.log
- 잔여 61개는 모두 일반 road(junction 커넥터 0개). 세션11 스캔이 일반 road 대상이었기 때문
- legacy 0개 / north 0개 / south 0개 / middle 2개
- middle: road1286 s=5.67 (f=0.173, R2 7.44m) / road1286 s=9.36 (f=-3.876, R2 1.26m, 접힘), 둘 다 way442595034, "고정노드(공유/끝점) 근처"
  -> A-1 에서 새로 찾은 r1286 +83° 꺾임(R2min 1.29@s9.5)과 같은 지점. middle 이 r1592 를 통과했어도 여기서 문제가 생길 가능성
- A-1 반경표 대조, 경로 road 중 R2min<9m (종점 road 제외):
  legacy 3개(r1446 7.40, r1428 6.48, r1564 6.42) / north 0개 / south 2개(r1446, r1428) / middle 3개(r1592 5.02, r1286 1.29, r1838 1.50)
- 주의: 커넥터는 세션11 잔여 목록에 없으므로 "잔여 0개"가 "커브 문제 0개"를 뜻하지 않는다. 커넥터는 A-1 표로 볼 것
