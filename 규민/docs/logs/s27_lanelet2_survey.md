# s27 Phase 5. Lanelet2 공수 조사 (2026-10-06, 조사만, 변환 안 함)

분류 표기: [확인] 로컬 파일 또는 공식 문서 원문으로 확인 / [확인-요약] 공식 사이트 검색 결과 요약으로만 봄(원문 미열람)
/ [추정] 근거 약함 / [확인 불가]
Autoware 저장소는 읽기만 했다(~/autoware 1.9.0 태그, 수정 없음). Docker 이미지는 열지 않았다

## 0. 전제 (로컬 확인)
- [확인] Autoware 1.9.0 의 Lanelet2 확장 버전: autoware_lanelet2_extension 1.2.0 (~/autoware/repositories/autoware.repos)
- [확인] Autoware 지도 3종 배치 예: ~/autoware_data/maps/<이름>/{pointcloud_map.pcd, lanelet2_map.osm, map_projector_info.yaml}
- [확인] 우리 맵 규모(frozen_v1, logs/session18_load.log 와 xodr 직접 집계): road 878 = 일반 260 + 교차로 커넥터 618, junction 95,
  주행 차선 878(road 당 lane -1 1개), 인도 차선 504, 높이 전부 0
- [확인] 커넥터 회전 분류(세션27 집계, 끝점 방향 차 기준 |30도| 미만 직진, 120도 이상 U턴): 직진 155 / 좌 154 / 우 155 / U턴 154.
  s23 의 "U턴형 143"(들어오는·나가는 방향 135도 기준)과 정의가 달라 수가 다르다

## 1. xodr -> Lanelet2 자동 변환 도구가 있는가
- [확인] CommonRoad Scenario Designer(TUM, github.com/CommonRoad/commonroad-scenario-designer):
  README 에 "OpenDRIVE => Lanelet/Lanelet2" 변환 기능. CommonRoad 형식을 거쳐 간다. GPL-3.0, BETA(v0.8.5), Python 3.10~3.13
  - README 에 Autoware 호환 언급 없음
  - [확인-요약] CommonRoad 포럼에 OpenDRIVE -> Lanelet2 -> Autoware 경로에서 오류 보고 글이 있다(원문 미열람)
  - 품질: [확인 불가]. 우리 xodr 로 돌려 본 적 없음
- [확인] 실제 사례: CARLA Town01 의 Autoware 용 Lanelet2(~/autoware_data/maps/Town01/lanelet2_map.osm, 출처는
  autoware_carla_interface README 가 가리키는 carla-simulator/autoware-contents, 세션3 기록)
  - generator="lanelet2", lanelet 124 = Town01.xodr 주행 차선 124 (road 122, laneSection 122). 차선 하나 = lanelet 하나
  - 태그는 type=lanelet / subtype=road 뿐. turn_direction·속도 제한·정지선·규제 요소 0. 경계 선(way)에 태그 없음
  - 좌표 local_x / local_y, ele 전부 0, projector_type Local
  - 즉 xodr 에서 기계적으로 뽑은 최소 형식이다. 어떤 도구로 만들었는지는 [확인 불가]
  - 이 최소 지도로 Autoware 가 경로 계획까지 되는지는 [확인 불가]. 우리 세션5~6 은 메모리 중단으로 Town01 연동 주행을 못 했다
- [확인] 비교용 Autoware 샘플 지도(sample-map-planning, generator=JOSM): lanelet 190, turn_direction 114(좌37 우39 직38),
  speed_limit 전 lanelet, regulatory_element 51(신호 등 18·정지 표지 6·우선권 18 등), stop_line 95, crosswalk 4, MGRS 투영.
  사람이 편집기로 만든 완전한 형식
- 결론: 자동 변환 도구는 있다(CommonRoad). Autoware 가 요구하는 태그까지 채워 주는지는 확인 못 했다

## 2. Autoware 가 Lanelet2 지도에 요구하는 것
- [확인] autoware_lanelet2_extension 1.2.0 docs/lanelet2_format_extension.md(원문)
  - 필수: 모든 점에 ele(높이) 태그 / 교차로 안 lanelet 에 turn_direction(left, right, straight) /
    신호등은 LineString + height 태그(우리 캠퍼스는 신호등 없음, odd.md -> 해당 없음)
  - 선택: local_x/local_y, MetaInfo, 정지선·신호 전구, centerline, 금지 구역, direction_change 영역(회차용), 측위 표지 등
- [확인-요약] Autoware 문서 "Vector Map creation requirement specifications": 운행 영역 + 사방 200m 이상 여유,
  lanelet 마다 통행 우선권·속도 제한·진행 방향·신호·정지선·표지 정보. 원문 페이지는 주소 변경으로 이번에 못 열었다
- [확인] 검증 도구 tier4/autoware_lanelet2_map_validator(Apache-2.0): Ubuntu 22.04·Humble, Autoware 작업 공간 안에서 colcon build 필요.
  검사 범주 lane / intersection / traffic_light / crosswalk / stop_line / area
  - 우리 환경: colcon build 는 별도 결정 전까지 금지(CLAUDE.md). Docker 이미지 안에 들어 있는지는 [확인 불가]

## 3. 자동으로 채워지는 것과 수작업
자동(xodr 에서 기계적으로 나옴, Town01 사례 기준) [확인 + 추정]
- lanelet 경계(왼쪽·오른쪽 선), lanelet 사이 앞뒤 연결, 진행 방향(one_way)
- ele: 우리 맵은 전부 0 -> 태그는 채워지지만 값은 평지. 양성재 오르막 미반영 한계 그대로(세션20)
- turn_direction: xodr 에는 없다. 커넥터 회전각으로 계산 가능(위 0절 분류) -> 스크립트로 자동화 가능 [추정, 미구현]
- 속도 제한: 일괄값(예: 20km/h)은 스크립트로 가능 [추정]
수작업(현장 정보, xodr·OSM 에 없음)
- 횡단보도: 고원식 누적 10곳 내외(10-04·10-06 답사), 좌표 없음. OSM 에도 거의 없음
- 정지선·양보(우선권) 관계: 비신호 교차로 95곳의 우선권. 캠퍼스 규칙 [확인 불가]
- 회차 구역(direction_change 영역): 북문·중문·후문 종점 회차
- 금지·특수 구역: 라바콘 구간(정문·후문 중앙선, 좌회전·유턴 불가), 볼라드, 착탈식 볼라드, 게이트 정차
- 지도 좌표계 결정: Local(Town01 방식) vs MGRS(샘플 방식). CARLA 연동과 점군 지도 좌표를 맞춰야 함 [확인 불가: 어느 쪽이 맞는지]

## 4. 편집 도구
- [확인-요약] Vector Map Builder(TIER IV): 웹 기반, 무료. 점군 지도를 배경으로 lane·규제 요소를 그린다. 계정 필요 여부 [확인 불가]
- [확인] JOSM: 샘플 지도의 generator 가 JOSM. 로컬 데스크톱 편집기. Autoware 전용 태그 지원은 플러그인 필요 여부 [확인 불가]
- 점군 지도가 없으면 Vector Map Builder 의 배경이 없다 -> 점군 지도 순서와 엮인다

## 5. 분량 (단계별)
형식: 단계 / 자동·수작업 / 예상 분량 / 근거
- 1 변환 도구 시험(CommonRoad 설치, 우리 xodr 1회 변환, 결과 로드) / 자동 / 반나절~1일 / 근거 없음, 추정. 설치는 venv 안 pip(승인 필요)
- 2 기하 검수(lanelet 878개 생성 여부, 끊김·겹침) / 자동 검사 + 수작업 확인 / 1일 / 근거 없음, 추정. 대상 수는 확인(878)
- 3 turn_direction 부여(커넥터 618개) / 자동(스크립트) / 반나절 / 근거: 회전각 분류는 이미 계산됨(0절). 스크립트는 미작성
- 4 U턴 커넥터 처리(154 또는 143, 정의에 따라) / 수작업 판단 + 자동 / 반나절 / 근거 없음, 추정. Autoware turn_direction 값에 U턴이 없다
- 5 속도 제한·ele·투영 정보 / 자동 / 2시간 / 근거 없음, 추정
- 6 횡단보도 10곳 내외 / 수작업 / 곳당 15~30분 -> 3~5시간 / 근거 없음, 추정. 좌표부터 다시 받아야 함
- 7 교차로 우선권·정지선(비신호 95곳 중 노선 위만) / 수작업 / 노선 위 교차로 수 미집계 -> [확인 불가]. 95곳 전부면 곳당 10분 가정 약 16시간(추정)
- 8 회차 구역·금지 구역 / 수작업 / 1일 / 근거 없음, 추정
- 9 Autoware 로드·경로 계획 확인(planning simulator) / 실행 / 1~2일 / 근거: 세션2 planning simulator 단독 실행 기록은 있으나 우리 지도로 한 적 없음. 메모리 제약(CLAUDE.md)
- 10 검증 도구(map_validator) / 실행 / [확인 불가] colcon build 금지 상태라 지금은 못 씀
합: 단계 1~9 를 다 더하면 약 7~10 작업일(추정). 가장 큰 불확실성은 1(도구가 쓸 만한가)과 7(우선권 범위)이다.
1이 실패하면 878 lanelet 을 손으로 그려야 해 분량이 크게 늘어난다(얼마인지 근거 없음)

## 6. "N주" 로 줄이지 않은 이유와 먼저 할 일
- 근거 있는 것은 대상 개수(878, 618, 95, 횡단보도 10곳 내외)와 필수 태그 목록뿐이다. 시간 값은 전부 추정
- 불확실성을 가장 빨리 줄이는 일: 단계 1(도구 시험) 반나절. 결과로 자동 / 수작업 비율이 정해진다
- 단계 1 은 재변환 맵(v2)이 확정된 뒤에 한다(재변환하면 lanelet 도 다시 만들어야 함)

## 참고 출처
- https://raw.githubusercontent.com/autowarefoundation/autoware_lanelet2_extension/1.2.0/autoware_lanelet2_extension/docs/lanelet2_format_extension.md
- https://github.com/CommonRoad/commonroad-scenario-designer
- https://github.com/tier4/autoware_lanelet2_map_validator
- https://commonroad.in.tum.de/forum/t/openroad-to-lanelet2-conversion-error/1386 (검색 결과로만 봄)
- Autoware 문서 지도 설계 페이지(autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/map/,
  docs.autoware.org 로 이동 후 404. 검색 요약으로만 봄)
