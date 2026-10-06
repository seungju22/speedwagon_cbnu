# s18 B. 루트 진행 방향 검증 (야간 배치, 서버 없음, 읽기전용)

스크립트: scratchpad/direction_s18.py / 전체 출력: map/docs/logs/s18_B_direction.log

## 방법 (road 마다 세 가지 확인)
- (a) xodr: 그 road 의 lane -1 이 driving 인가. CARLA 는 우측통행이라 lane -1(오른쪽) 차선은 +s 방향으로 달린다
- (b) 연결: 앞 road 끝(lane -1, s=L-0.05)에서 next() 로 가면 이 road 의 lane -1, s≈0 에 도착하는가.
  역방향이면 lane +1 이나 s≈L 로 도착하거나 아예 연결되지 않는다
- (c) 실주행: 그 road 로 조회된 tick 에서 s 가 증가하는가(주행한 구간만)

## 결과
- 878 road 전부 (왼쪽 driving 0, 오른쪽 driving 1). laneSection 2개 이상인 road 0개.
  즉 지시문 배경의 "878 road 전부 일방통행(lane -1 하나)" 을 xodr 로 확인
- legacy 15 / north 17 / south 15 / middle 19 road: 역방향 0, 확인필요 0
- 모든 연결에서 next() 가 다음 road 의 lane -1, s=0.45 에 도착(순방향)
- 실주행 s 증가비: 정문 road1247 만 0.98(출발 직후 정지 상태의 s 요동), 나머지 전부 1.00
- middle 은 road1592 s=4.2 까지만 주행 표본이 있다(그 뒤 10개 road 는 (a)(b)로만 확인)

## 해석
- 루트 체인은 chain_s18.py 에서 lane -1 next() 로만 이어 만들었다. 이 방법은 구조상 역방향 road 를 넣을 수 없다.
  세션16 이후 test_drive 경로도 check_plan_adjacency(next() 연결) 를 통과한 것만 쓴다
- 무방향 그래프가 쓰인 곳은 OSM 단계 분석(plot_route_realmap_s16.py build_graph = networkx.Graph)과
  세션7 gate 조사 일부다. 이것은 "OSM 에서 어디가 이어지는가" 를 볼 때만 썼고 CARLA 주행 경로에는 들어가지 않는다(가정: 과거 세션 스크립트를 전수 확인하지는 않음)
- 양방향 OSM way 는 xodr 에서 방향별 road 두 개(예: road1237/road1368, 길이 130.52m 로 같음)로 나뉜다.
  그래서 반대 방향 road 는 별개 id 이고 next() 경로에 섞이지 않는다

## 판정
무방향 그래프 문제는 현재 4개 루트(legacy/north/south/middle)에 영향 없음. 이 항목을 닫는다.
middle 실패의 원인 후보에서 "역방향 road" 는 제외.
남은 주의: 후문 재변환 뒤 새 루트를 만들 때도 같은 next() 체인 방법을 쓸 것(OSM 무방향 경로를 그대로 옮기지 말 것).
