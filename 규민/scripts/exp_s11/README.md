# 세션11 Phase B 사전 실험 스크립트 (프로토타입, 정식 도구 아님)
스크래치패드에서 실행하던 것을 유실 방지로 보존. 정식 스크립트는 승인 후 map/scripts/ 에 작성.
실행 환경: ~/campus_mobility_sim/.venv-carla/bin/python (numpy 필요). 같은 폴더에서 실행.
탐색 스크립트는 경로가 절대경로(~/campus_mobility_sim/map/...)이고 중간 산출물(pkl/osm/xodr)은
현재 폴더에 쓴다. 프로젝트 원본 파일은 읽기만 한다.

실행 순서와 목적
exp_lib.py        공통: 격리 OSM 생성, Osm2Odr 변환, 필렛, 2m창 지표(measure2)
e0.py / e0b.py    road1247 OSM 체인 격리 변환 재현 (해석적 지표 / 2m창 지표)
e2.py             단일 꺾임 각도·세그먼트 길이별 변환 응답 (해석적 지표)
e1.py             합성 꺾임 + 필렛 반경 스윕
e4.py             실제 망 통과(s=80)/정지(s=95) 커브 2m창 지표 비교
align.py          xodr 기준선 <-> OSM 폴리라인 ICP 정렬(align.pkl)
sites.py          전 망 문제 구간 스캔(2m창, f 임계별) -> sites.pkl, g03.pkl
sites2.py         문제 구간 -> OSM 노드 대응(rows.pkl)
sel.py            노드 꺾임각별 원인 분포
smooth_trial.py   필렛 적용 프로토타입 (환경변수 TAG, D_MAX, R_ACC; 인자 R_T)
verify_trial.py   프로토타입 OSM 작성 + 변환(trial_*.osm/xodr)
scan_generic.py   임의 xodr 문제 구간 스캔
hist.py           f 구간별 히스토그램
classify_rem.py   잔여 문제 구간 원인 분류(고정노드/호/원본 내부노드)
route_check.py    경로 핵심 두 커브(노드 3957495733/32) 주변 최악 f
bldg.py           원본 대비 최대 이탈(Hausdorff)과 건물 침범(OSM 건물 399개)
최종 후보 재현: TAG=F10.0 D_MAX=2.0 R_ACC=10.0 python smooth_trial.py 11.0 && TAG=F10.0 python verify_trial.py 11.0
(smooth_trial.py 는 rows.pkl, align.pkl 이 먼저 있어야 한다: align.py -> sites.py -> sites2.py)
