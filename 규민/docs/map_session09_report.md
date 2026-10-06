# 맵세션9 보고

## Phase 0
- 세션8 Town01 기동: 인자 무시(world.get_map().name=Town10HD_Opt, 31s/122s 동일)
- 세션8 기록 누락: 없음(setup_log.md에 Town01/878road메모리/진단2·3 전부 기록됨)
- pgrep CarlaUE4: 실행 중 아님. df avail 87G/147G

## CARLA 세션 운영 절차(2026-09-22, 사용자 확정, setup_log.md 기록)
1. 재부팅 2.pgrep -fa firefox로 브라우저 부재 확인 3.서버 기동
4.기본맵 2분 후 측정(2Gi 이상이면 진행) 5.878 road 로드
CLI 인자로 시작맵 지정하는 방식은 0.9.15 패키지판에서 동작 안 함(종결).

## Phase A. 급커브 가설 검증

### 발견한 버그(선행 수정)
`map/scripts/analyze_geometry.py`의 `poly_deriv`가 paramPoly3 3차항
미분에서 `3*d*p`로 계산(정답은 `3*d*p**2`, p제곱 누락). p=0/1(커넥터
끝점)에서는 두 식이 우연히 같아 세션2~8의 junction 끝점 heading_diff
분석(r1446 -78.8도, r1428 68.6도, r1917 38도 등)은 영향 없음. 그러나
곡선 내부 s 지점의 yaw는 완전히 틀린 값이 나왔다(첫 실행에서 정지
구간 반경이 23.92m로 나와 CARLA 서버 실측 -11.7deg/m과 불일치, 수식
검증 후 확인). 수정 후 CARLA 서버 실측(session8_diag_yaw.txt)과
xodr 직접계산이 s=90.37~96.37 구간 dyaw 부호만 반대(좌표계 차이)이고
크기가 근접 일치(예: s=94.37~95.37 CARLA -11.7도 vs xodr수정후 +10.37도).
방법 신뢰성 확인됨.

### A-1. 통과 커브(s=80~85) vs 정지 커브(s=93~96)
- 통과 최소 곡률반경: **4.36m** (s=80.0~80.5, rate=13.15deg/m)
  - 이 구간 세션7 실측 속도: 5.29~5.30m/s 로 전혀 감속 없음(정속 순항)
- 정지 최소 곡률반경: **4.81m** (s=95.0~95.5, rate=11.92deg/m)
  - 이 구간은 이미 정지 상태(0.06m/s, t=25.16s)라 원인이 아니라 결과

**핵심 반증**: 정지 구간(4.81m)이 통과 구간(4.36m)보다 오히려 곡률이
완만하다(반경이 더 크다). 두 커브는 기하학적으로 비슷하거나 통과
구간이 더 급한데, TM은 통과 구간에서는 전혀 반응하지 않았고 정지
구간에서만 s=89.4부터 감속을 시작해 완전정지했다.
**곡률 반경 단독으로는 통과/정지를 가르지 못한다.**

경로 웨이포인트(drive_plan CSV, ~4m 간격) 단위로도 재확인: idx20
(s=80.55) 지점 인접 세그먼트 꺾임 약 -15.8도, idx23(s=92.34) 지점
약 -17.1도로 역시 비슷한 크기. 웨이포인트 성김 때문에 생기는 차이도
아닌 것으로 보인다.

가능한 남은 설명(미검증, 다음 세션 후보):
1. TM의 감속이 s=89.4에 시작되는 것 자체는 곡률을 앞서 감지하는
   정상 동작일 수 있으나(선행 감속), s=80 커브는 왜 이 선행 감속이
   전혀 없었는지 설명 안 됨 — 두 커브 모두 진행방향 곡률은 비슷한데
   전방 lookahead 판단 로직이 무엇을 다르게 보는지 불명
2. s=80 커브는 road1247 시작(s=0)에서 가깝고, s=93~96 커브는
   junction1(다음 교차로) 입구에 더 가깝다 — junction 접근 시
   TM의 별도 감속 로직(교차로 감속)이 개입했을 가능성
3. TM 내부 상태(교통신호 추종/경로추종 모드 전환 등)는 이번 읽기전용
   분석으로 확인 불가 — 서버 필요

### A-2. 확정경로 15 road 급커브 전수(기준 rate>=6deg/m, 30deg/5m 환산)
- 총 6곳: plain road 3 / connector 3
  - road1247 s=79.5~81.0 반경4.36m (통과함)
  - road1247 s=93.5~95.5 반경4.81m (정지 지점)
  - road1356 s=11.5~12.0 반경6.76m (미검증 — 차량이 도달 못 함)
  - road1564[connector junction36] 반경7.22m
  - road1428[connector junction10] 반경8.16m
  - road1446[connector junction42] 반경8.25m
- 같은 유형이 road1247에 이미 2곳(같은 반경대), 경로 전체로는 6곳.
  단일 지점 문제가 아니라 경로 전반에 비슷한 급커브가 흩어져 있다.

### A-3. road1247 원본 OSM 대응
- road1247 predecessor=junction2(OSM노드 2261340221),
  successor=junction1(OSM노드 4748296080)
- OSM 노드 체인(cbnu_campus_fixed_tags73.osm, 8개 노드, 누적147.64m):
  - 2261340221(0m) - 3957495734(54.2m,꺾임+3.5) -
    3957495733(80.0m,꺾임+16.7) - 4748296079(84.0m,꺾임-0.1) -
    3957495732(95.8m,꺾임+28.5) - 4748296078(122.6m,꺾임-0.0) -
    3957495731(140.2m,꺾임+1.8) - 4748296080(147.6m)
- xodr s=93.0/96.0 을 유사변환(스케일1.000709,회전0.065도,잔차RMS3.7m)으로
  위경도 변환 -> 두 지점 모두 최근접 OSM노드는 3957495732(누적95.8m),
  거리 2.86m/2.01m. **정지 구간 급커브는 이 노드의 원본 28.5도 단일
  꺾임에서 비롯됨 — 변환(paramPoly3 적합) 산물이 아니라 OSM 원본.**
- 통과 구간(s~80)도 마찬가지로 노드 3957495733의 원본 16.7도 꺾임에서
  비롯됨. **둘 다 원본 노드 꺾임이며, 이 점에서도 두 커브는 같은 유형.**
- 노드 간격: 정지구간 앞뒤 11.78m/26.77m, 통과구간 앞뒤 25.79m/4.07m.
  둘 다 노드 간격이 수 m~수십 m 대로 도시도로 치고는 성긴 편(OSM이
  실제 곡선을 적은 노드로 각지게 그렸다는 원래 가설과 일치).

## Phase A 판정(A-1 결과 기준)
- 급커브 출처: **OSM 원본** (변환 과정 아님) — 이 부분은 원 가설 지지
- 그러나 **곡률 반경만으로는 정지 원인을 설명 못함** — 통과한 커브가
  더 급하거나 비슷함.

## 추가진단: 물리접촉 vs TM제동 스크리닝 (map_session09_curb_analysis.md)
CSV(drive_log_20260919_231354.csv)와 xodr 기하로 사용자 가설(총꺾임각
차이로 인도 연석에 물리접촉) 1차 스크리닝. 스크립트:
map/scripts/analyze_curb_contact_s9.py

- z: 통과 변화 0.0000m, 정지 변화 -0.0030m(최대상승 +0.0030m). 둘 다
  노이즈 수준. **연석을 타고 오르는(climb) 신호는 없음.**
- 횡편차(차선-1 중심 기준, 부호 있음, 음수=인도 쪽): 통과 최대 0.590m
  (s=80.37), 정지 최대 0.848m (s=91.95). **둘 다 인도 쪽으로 치우침,
  정지구간이 44% 더 큼.**
- 차량진행yaw-도로yaw: 통과 최대편차 +10.19deg, 정지 최대편차
  +31.95deg(s=92.11). 정지구간에서 차량이 도로 진행방향을 훨씬 크게
  벗어남.
- 인도 안쪽 경계까지 거리(차선중심 기준, 양쪽 구간 동일 단면이라 상수):
  1.675m. 차량 반폭 추정 0.85m 적용 시 여유 0.825m(치우침 0 가정 시).
  - 통과구간: 실측 치우침 0.590m 적용 시 차량 가장자리~인도 거리
    = 1.675 - 0.590 - 0.85 = **0.235m 여유(접촉 없음)**
  - 정지구간: 실측 치우침 0.848m 적용 시 = 1.675 - 0.848 - 0.85 =
    **-0.023m (물리적 한계선 통과, 접촉 가능 범위)**
- **제어값(throttle/brake/steer) 컬럼: CSV에 없음.** TM 의도적 제동인지
  물리적 저지인지 직접 구분할 결정적 증거는 이 CSV로는 확보 불가.

### 판정
**판정 불가(제어값 부재) — 단, 정황증거는 물리접촉 쪽으로 기움.**
z는 climb 신호가 없어 "올라탐"은 아니지만, 정지구간에서만 차량이
인도 쪽으로 크게(0.848m) 치우쳐 계산상 차량 가장자리가 인도 경계를
넘어서고(여유 -0.023m), 동시에 도로 진행방향과의 yaw 편차도 급격히
커진다(+31.95deg) — 사이드(측면) 접촉으로 조향/전진이 막혔을 가능성과
부합하는 정황. 통과구간은 같은 계산에서 0.235m 여유가 있어 대조된다.
단, CARLA가 이 인도 차선에 실제 충돌 메시(3D 연석)를 렌더링하는지는
이 CSV·xodr 분석만으로 확인 불가 — 렌더링 안 됐다면 물리접촉 자체가
성립 안 하므로 이 가설도 무효가 된다.

### 다음 필요한 것 (서버 1회 재현, 사용자 제안)
- 정지 지점 spectator로 직접 관찰(인도 연석 메시 존재 여부, 차량과의
  실제 근접/접촉 확인)
- 제어값(throttle/brake/steer), pitch/roll, z를 프레임 단위로 기록
- 세션5 첫 실패 사례(스로틀0.85·제자리·pitch-1.85·roll-1.5)와 같은
  패턴이 이번 정지에서도 나타나는지 대조

### 정정(사용자 지시, 2026-09-22)
"통과 10.19도 vs 정지 31.95도" yaw 편차를 "가장 강한 정황"으로 본 판단은
철회. 이 값은 차체 방향이 아니라 CSV 연속 위치차로 계산한 진행방향이고,
정지 직전(s=92.11)은 tick간 이동이 극히 작아 잡음에 좌우된다. 위치
기반 횡편차(0.590m -> 0.848m)가 더 신뢰할 만한 정황. 차체 방향은
test_drive.py 신규 vehicle_yaw 컬럼(tf.rotation.yaw 실측)으로 대체.

## test_drive.py 기록 보강 (작성 완료, 미실행)
py_compile 통과, --dry-plan 정상(계획 203점, 기존과 동일 - dry_plan()은
신규 코드 경로 영향 없음).

- 궤적 CSV(LOG_FIELDS)에 8컬럼 추가: throttle,brake,steer,hand_brake,
  reverse,pitch,roll,vehicle_yaw. 기존 14컬럼 이름·순서 불변.
- `record_bbox_once()`: 스폰 직후 1회 `vehicle_bbox_<ts>.txt`에
  bounding_box.extent(반길이/반폭/반높이) 기록. 횡편차 계산의 반폭
  가정값(0.85m)을 실측으로 교체하는 용도.
- `setup_collision_sensor()`: sensor.other.collision 부착.
  `collision_events_<ts>.csv`(COLLISION_FIELDS: frame, sim_timestamp,
  t_rel[공란, 스레드간 오차 우려로 비움 - frame으로 메인CSV와 대조],
  other_actor_id, other_actor_type_id, other_actor_semantic_tags[원시
  숫자, ; 구분], impulse_x/y/z, impulse_mag, loc_x/y/z)에 콜백으로 기록.
  근거: CARLA 공식문서(ref_sensors, collision detector) - 서버가 정적
  요소에도 semantic tag 조회용 fake actor를 만들어 충돌이 잡힌다.
- 정리 순서: finally에서 collision_sensor.stop()->destroy() 먼저(각각
  try/except), 그 다음 vehicle. 세션5식 서버측 선제거 대비.

## 서버 재현 계획 (작성만, 실행은 다음 서버 세션)
1. 재부팅 -> `pgrep -fa firefox`로 브라우저 부재 확인 -> 서버 기동
   (`./CarlaUE4.sh -quality-level=Low -windowed -ResX=800 -ResY=600`,
   기본맵 2분 후 avail 2Gi 이상 확인) -> 878 road 로드
   (`map/tests/test_load_map_878.py`)
2. 수정된 `map/tests/test_drive.py` 로 동일 조건(spawn/경로/20km/h) 1회 주행
3. 정지 시 spectator 이동 좌표(계산 방법, 서버에서 실제 정지 x,y,z와
   vehicle_yaw로 재계산):
   - 세션7 참고값: 정지 위치 약 (228.3, -995.8, ~0), 그 지점 도로 진행방향
     약 -38.5도~68.7도 사이(커브 구간이라 s에 따라 변함, s=92.3 부근 heading 사용)
   - **위에서(top-down)**: `carla.Transform(carla.Location(x, y, z+15),
     carla.Rotation(pitch=-90, yaw=0, roll=0))`
   - **옆에서(side)**: 진행방향에 수직인 오프셋 5m, 높이 +2m.
     heading_rad = road_yaw(해당 s) 라디안 값일 때
     `offset_x = -5*sin(heading_rad)`, `offset_y = 5*cos(heading_rad)`
     (진행방향 왼쪽 5m, OpenDRIVE t-축 관례와 동일 부호)
     `carla.Transform(carla.Location(x+offset_x, y+offset_y, z+2),
     carla.Rotation(pitch=-15, yaw=degrees(heading_rad)+180))`
     (차량을 정면에서 비스듬히 내려다보도록 180도 돌려 마주보게 함)
   - 실행 시점에 `world.get_spectator().set_transform(...)`로 적용
4. 판정 기준(사용자 제시)
   - brake>0 이면 TM이 의도적으로 정지
   - throttle>0 인데 속도 0 이면 물리적으로 막힘
   - 충돌 센서가 정적 객체(semantic_tags에 Sidewalk 태그, 공식 태그표
     확인 필요) 충돌을 보고하면 연석 접촉 확정
   - pitch/roll이 세션5 첫 실패(-1.85도/-1.5도)와 비슷하면 같은 현상
5. bounding box 실측값(vehicle_bbox_<ts>.txt)으로 A-1 curb_analysis의
   여유(-0.023m 등) 재계산

판정 결과에 따라: 물리접촉 -> Phase B(평활화) 또는 차선폭 조정 검토 /
TM제동 -> agents(BasicAgent) 전환 검토. 이 세션은 여기까지(서버 미기동).
