# 맵 세션 6 보고 (작성 중 — Phase B 이후 갱신)

상세 수치: map/docs/logs/session6_phaseA.txt
재현 스크립트: map/scripts/analyze_connectors_s6.py
(실행: python3 map/scripts/analyze_connectors_s6.py, stdlib만 사용)

## Phase A 결과 (2026-09-19, 사용자 승인 후 실행)

- gap 이상 131건은 전부 netconvert 템플릿 곡선(len 9.8924).
  서명 3종(n=126/3/2)은 반올림 차이일 뿐 같은 형상.
- gap 이 정상인데 방향차만 큰 실형상 결함 3건:
  j20 r1480(76.5°, 경로 밖), j67 r1742(42.0°, 0.97m, 경로 밖),
  j107 r1917(37.9°, 2.97m, 경로 위).
- 5m 미만 road 27개 중 junction 내부 3개(r1742/r1917/r1914).
  0.2m plain road 14개는 stub 으로 보이나 미확인.
- 878 road 전부 오른쪽 driving 차선만 보유 = 전 road 단방향.
- 경로상 실형상 결함은 r1917 하나. 아직 주행으로 밟아 보지 못함.

## 정정 1: 세션5 2차 실패 원인 (세션5 보고서에도 정정문 추가함)

r1917 이 원인이라는 추정은 오류. 실패 좌표는 road1914 시작점 부근이고
r1917 은 41.8m 앞이라 도달하지 못했다. r1914 는 4.29m, 회전 -2.4°,
접합각 0° 로 형상상 결함 없음. lane 폭(3.35m)과 고도(0)도 일정.

가설(미검증): junction1 에는 커넥터가 둘 있다. r1914(정방향, 4.29m)와
r1915(r1120 으로 가는 템플릿 9.89m, 회전 126.9°, U턴). test_drive.py 는
set_autopilot(True) 만 써서 junction 커넥터를 TM 이 자동 선택한다.
실패 좌표 2점은 r1915 최근접거리 2.2m/2.0m. 2차 주행 CSV 가 없어
어느 커넥터를 탔는지 데이터로는 확인 불가.

## 정정 2: 확정 경로 14 road / 768.26m -> 15 road / 781.22m

- 누락: r1564 (junction36, r1238 -> r1278, 12.96m, 회전 82.4°).
  커넥터는 xodr 에 존재한다. 도로망 결함이 아니다.
- 결함 기록(find_library_entrances.py 경로 계산):
  "junction36에 닿는 도로를 목표로 잡고 종점 도로를 이어 붙이는 방식이라
  사이 커넥터가 누락됨. 다른 junction은 정상 포함되나 종점 도로 쪽에서만
  발생."
  route_length_to_point() 가 goal_roads = roads_touching_junction 으로
  dijkstra 후 path + [rid] 를 붙인다. 최종 단계에만 발생.
- 같은 방식으로 계산한 다른 경로 점검 결과:
  S5 출입구1/2: 같은 누락(r1564). 출입구2 도 같은 골격이라 721.84m 는
    12.96m 과소.
  S5 출입구3(790.52m): r1238, r1368 이 모두 junction36 진입 도로라
    정방향 연결이 없다. 경로 무효(채택되지 않았으므로 영향 없음).
  S4 A안(15 road, 529.86m): 누락 0, 역방향 0. 정상.
- 부가 발견: dijkstra 그래프(build_road_graph)는 무방향이다. 전 road 가
  단방향인데 방향을 검사하지 않는다. 채택 경로(정정 15 road)와 S4 A안은
  전부 정방향임을 확인했다.
- 종점 위치(road1278 s=100.38m)는 변하지 않는다.

## Phase B-4 조사: 주행 경로 강제 (근거: 0.9.15 태그 공식 소스·문서)

근거: PythonAPI/python_api.md(0.9.15 패키지 동봉),
LibCarla/source/carla/trafficmanager/{Parameters,LocalizationStage}.cpp
@0.9.15, PythonAPI/carla/agents/navigation/*.py(패키지 동봉).

### 방법 1. TM set_path(actor, [Location...])
- 존재함. 공식 문서 경고: "도로 토폴로지가 경로를 막지 않는지 확인".
- 동작(LocalizationStage::ImportPath): 갈림길(next waypoint 2개 이상)에서만
  개입한다. 각 분기의 junction 통과 후 지점 도로 id 가 경로 지점의 도로 id 와
  같으면 그 분기를, 아니면 경로 지점과 가장 가까운 분기를 고른다.
  갈림길이 아닌 곳은 TM 이 차선을 따라간다.
- 판정: 커넥터를 직접 지정하지 않고 "출구 도로 쪽" 으로 유도한다.
  경로 지점 간격·위치에 따라 선택이 달라질 수 있는 휴리스틱이다.
  강제가 아니라 유도(권고에 가까움). 실제 주행 로그로 검증 필요.
- set_route(actor, ['Left','Right','Straight']): 갈림길마다 RoadOption 이
  일치하는 분기 선택. 존재함. junction1 에서 r1914 는 직진, r1915 는
  Left/Right 로 분류될 것이나 CARLA 의 분류 결과는 미확인.

### 방법 2. agents (BasicAgent / BehaviorAgent / ConstantVelocityAgent)
- 패키지에 있음: ~/carla/CARLA_0.9.15/PythonAPI/carla/agents/navigation/
  (basic_agent, behavior_agent, constant_velocity_agent, global_route_planner,
  local_planner, controller). PyPI carla 휠(venv)에는 없다. PYTHONPATH 로
  PythonAPI/carla 를 추가해야 한다.
- 의존성: networkx, shapely 가 venv 에 없다(둘 다 ImportError 확인).
  requirements.txt 는 numpy==1.18.4, Shapely==1.6.4.post2 로 고정돼 있어
  Python 3.10·numpy 2.2.6 과 충돌 가능. 버전 미고정 설치가 필요(승인 필요).
- 경로 지정:
  set_global_plan([(Waypoint, RoadOption), ...]) — 우리가 만든 웨이포인트를
  그대로 따라간다. TM 의 분기 선택이 없어 커넥터를 확정적으로 지정 가능.
  웨이포인트는 map.get_waypoint_xodr(road_id, lane_id, s) 로 만들 수 있다
  (road id 목록 + lane -1). 
  set_destination(end) — GlobalRoutePlanner 최단경로. 템플릿 커넥터의
  3.35m 어긋남이 그래프 노드 연결에 영향을 주는지는 미확인.
- 제어: VehicleControl 을 직접 계산(PID)해 apply_control. autopilot/TM 미사용.
- 속도: BasicAgent(target_speed=km/h), set_target_speed(km/h),
  follow_speed_limits(True/False).

### 속도 제한 (TM)
- set_desired_speed(actor, km/h): 소스에서 값/3.6 으로 m/s 변환 확인.
  지정하면 percentage 설정은 무시된다.
- vehicle_percentage_speed_difference(actor, %): 제한속도 대비 감속 비율.
  기본 30%. 세션5 순항 7.95m/s(28.6km/h) 는 40km/h x 0.7 과 부합한다는
  추정을 했으나 아래 "제한속도 근거 점검"에서 근거 없음으로 판명.
- 20km/h 는 set_desired_speed(vehicle, 20) 로 직접 지정 가능.

### 방법 3. 경로에서 특정 road 배제
- 해당 없음: 확정 경로는 이미 r1914 를 포함한다. 문제는 분기 선택.

### 공식 GitHub 이슈 검색 (carla-simulator/carla, API 키워드 검색)
- 스탠드얼론 OpenDRIVE 의 물리 이탈·폭주를 다룬 이슈는 찾지 못함.
  (키워드 검색의 한계로 "없다"는 확정 아님)
- 관련: #6940 (open) Osm2Odr missing junctions (0.9.14).

## 제한속도 근거 점검 (2026-09-19, 사용자 요청)

xodr 확인 결과:
- road/type 878개 전부 type="town", <speed> 자식 없음(road 단위 제한 없음).
- <speed> 는 lane 단위 1382개에 있다. max 는 m/s(unit 속성 없음=기본 m/s).
  13.89(=50km/h) 935개, 템플릿 커넥터 3.73(=13.4km/h) 131개, 나머지는
  커넥터별 4~13.9 값(곡률 기반으로 보임). 경로상: plain 13.89, r1446 7.84,
  r1428 9.99, r1564 8.74, r1915 3.73. <signal> 요소 0개, OSM maxspeed 0개.
- TM 이 읽는 제한속도는 lane speed 가 아니다. 0.9.15 소스: TM 은
  simulation_state.GetSpeedLimit(=Vehicle::GetSpeedLimit)를 쓰고, 이 값은
  AWheeledVehicleAIController::SpeedLimit(기본 30.0)이며 USpeedLimitComponent
  (속도제한 표지판 박스 overlap)만 갱신한다. 이 맵엔 표지판이 없다.
- 그러면 기본 30km/h x (1-0.3) = 21km/h = 5.83m/s 여야 하나 세션5 실측은
  7.95m/s. 40x0.7=28km/h(7.78m/s)와는 근사하나 40km/h 의 출처를 찾지
  못했다. 50x0.7=9.72 도 불일치.
판정: "제한속도 40km/h" 는 추정이었고 근거를 찾지 못했다. 확정 아님(철회).
7.95m/s 의 원인은 미확인. 대응: set_desired_speed 는 제한속도를 우회한다
(소스: exact_desired_speed 가 있으면 그 값을 그대로 반환). 주행 로그에
vehicle.get_speed_limit() 를 매 tick 기록해 실측으로 확정한다.

## 미결 항목 (세션6 추가)

1. 경로 탐색 무방향 그래프: 878 road 전부 단방향인데 경로 탐색
   (verify_session4.build_road_graph / dijkstra_shortest)에 방향 검사가
   없음. 채택 경로 2개(S5 정정 15road, S4 A안)는 우연히 정방향. 다른 경로
   계산 시 역주행 경로가 나올 수 있음. (S5 출입구3 이 실제 사례:
   정방향 연결 없는 무효 경로가 790.52m 로 산출됨)
2. 제한속도 근거 미확정(위 절). 세션5 순항 7.95m/s 의 원인 미확인.
3. 2차 주행에서 어느 커넥터를 탔는지 미검증(r1915 가설).
4. r1917 실형상 결함(방향 37.9°/31.0° 지그재그, 2.97m)은 경로 위에 있으나
   아직 주행으로 밟아 보지 못함.

## CSV 저장 버그 (수정 완료, 2026-09-19)

원인(코드 확인): test_drive.py 의 저장 블록이 루프 뒤 try 블록 끝에만 있었다.
루프 안 vehicle.get_transform() 이 서버의 액터 제거로 RuntimeError 를 내면
except 로 건너뛰어 저장이 실행되지 않고 메모리의 궤적이 사라진다. 1차
주행 CSV 만 남은 것은 정상 종료였기 때문. 수정: 시작 시 파일을 열고 tick
마다 writerow+flush, finally 에서 close. (아직 실행 검증 전)

## Phase C 진행 상황 (2026-09-19)

- 서버 명령 원문(사용자 별도 터미널): cd ~/carla/CARLA_0.9.15 &&
  ./CarlaUE4.sh -quality-level=Low -windowed -ResX=800 -ResY=600
- 시점1(기본맵 안정 후, 15:44:42): avail 1.3Gi / swap 2.6Gi / RSS 1.79GiB /
  gtt 2.99GiB / vram 1.27GiB -> 중단 기준(avail 2GiB 미만) 도달.
- 878road 로드·주행 미실시. test_drive.py 는 작성·문법검사·dry-plan 까지만.
- 커스텀 맵 직접 기동 방법 없음(공식 문서·소스 확인), 시작 맵 변경 안 함.

### 원인 정정 (사용자 확인)
브라우저 잔존은 추정이 아니라 사용자 확인. Firefox를 닫지 않은 상태에서
서버를 기동했다. 하드웨어나 도로망 문제가 아니며, 다음 세션에서 브라우저
종료 후 재측정하면 정상 진행 가능할 것으로 예상.
(당시 firefox 계열 RSS 합 약 0.9GiB. 세션5와의 avail 차이 전체를 이것만으로
설명하는지는 재측정으로 확인 필요.)

## 다음 세션 체크리스트
- 사용자가 '브라우저 닫았다'고 말해도 pgrep으로 실제 확인할 것. 세션6에서
  구두 확인만 믿고 진행했다가 중단 기준에 도달했다.

## 서버 종료 후 회복치 (15:47:15)
used 6.2Gi / avail 6.1Gi / swap 1.2Gi / gtt 0.07GiB. 서버 종료로 avail 은
1.3Gi -> 6.1Gi 회복. 그러나 firefox 계열(RSS 합 1.00GiB)은 실행 중이고,
세션 시작 시점(avail 10Gi, used 2.5Gi, swap 0B)에는 못 미친다. 서버 없이도
약 3.7Gi 가 더 쓰이는 상태이며 Firefox RSS 만으로는 설명되지 않는다.
원인 미분해. 다음 세션 baseline 을 브라우저 종료 전후로 비교 측정할 것.

## 마감 (2026-09-19)
Phase A, B-4 완료. Phase C, D 는 중단 기준 도달로 미실시.
다음 시작점:
1. baseline 재측정(브라우저 종료를 pgrep 으로 확인 후, 종료 전후 비교)
2. 서버 기동 -> 878road 로드 -> 시점 2/3 측정 -> test_drive.py 실행
3. 주행 로그로 확정할 것: junction1 에서 r1914 선택 여부(r1915 가설),
   r1917 지그재그 통과, get_speed_limit 실측
4. 실패 시 agents(set_global_plan) 로 전환(networkx/shapely 설치 승인 필요)
5. 미결: 무방향 그래프 경로 탐색, 제한속도 근거, 번호49, 24-road 컴포넌트,
   junction20 76.5°, 50m 미만 연결로, 제1학생회관 명칭, 병원 고립 덩어리

### 원인 정정 재정정 (사용자 승인)
Firefox 단독 원인이 아니다. "브라우저 때문" 단정은 성급했다(사용자 인정).
확인된 사실: Firefox 를 닫지 않고 서버를 기동함. 미확인: 서버 종료 후에도
세션 시작 대비 약 3.7Gi 추가 사용이 남아 Firefox RSS 1.0GiB 로는 설명 안 됨.
원인 미확정. 위 "정상 진행 가능할 것으로 예상" 은 미검증 예상이다.

## 다음 세션 baseline 재측정 절차 (전부 읽기전용, Firefox 종료만 사용자)
기준값(참고): 세션6 시작 avail 10Gi/used 2.5Gi/swap 0B, 서버 종료 후
avail 6.1Gi/used 6.2Gi/swap 1.2Gi(Firefox 1.00GiB 실행 중).

0. 사전: pgrep -fa 'firefox|CarlaUE4' 로 실제 프로세스 확인(구두 확인 불가).
   시각 기록. 각 단계 결과는 파일로 저장(scratchpad 또는 docs/logs).
1. 현재 상태(종료 전)
   free -h ; swapon --show ; cat /proc/meminfo (MemAvailable, AnonPages,
   Shmem, Slab, SUnreclaim, KReclaimable, Cached) ;
   ps -eo pid,rss,pss,comm --sort=-rss | head -11 (pss 열이 없으면
   각 pid 의 /proc/PID/smaps_rollup 의 Pss) ;
   gtt/vram: /sys/class/drm/card*/device/mem_info_{gtt,vram}_used
2. Firefox 종료(사용자): 종료 후 pgrep -fa firefox 가 비었는지 확인.
   프로세스 이름이 Isolated Web Content/Web Content/Privileged Content
   등으로 나뉘므로 firefox 문자열만으로 놓칠 수 있음 - comm 목록도 확인.
3. 재측정: 종료 30초 뒤와 2분 뒤 두 번 1번과 같은 항목. 시각 기록.
4. 대조: 회복량(avail 증가, used 감소) vs 종료한 프로세스의 RSS 합/PSS 합.
   맞으면(오차 약 0.3GiB 이내) Firefox 가 원인. 남는 차이가 크면 5번.
   같은 조건에서 3회 재현되는지 보려면 2분 간격 3회 측정.
5. 남는 부분 조사(누가 점유?):
   - GPU/UMA: gtt_used, vram_used 변화, /sys/kernel/debug/dri 는 root
     필요(사용자 실행)
   - shmem: /proc/meminfo Shmem, df -h /dev/shm /run/user/*, ipcs -m,
     ls -la /dev/shm
   - 커널: Slab, SUnreclaim, (slabtop -o 는 sudo, 사용자 실행)
   - 페이지 캐시: buff/cache 는 회수 가능하므로 avail 에 이미 반영
   - swap: swapon --show, vmstat 1 5(si/so 활동), swap 에 남은 페이지는
     프로세스별 /proc/PID/status 의 VmSwap 합
   - 프로세스 외 항목 합이 used 와 맞는지: AnonPages+Shmem+Slab+PageTables
     +KernelStack 와 used 비교
   - 성과 없으면 원인 미분해로 기록하고, 서버 기동은 avail 6Gi 이상에서만
     진행(세션5는 로드 2분 후 avail 4.6Gi 였음)
6. 판정 후 서버 기동은 별도 승인. 기동 후 시점 1/2/3(기본맵 안정,
   878road 로드 직후, 2분 후) 측정. CLAUDE.md 의 수정된 중단 기준 적용.
