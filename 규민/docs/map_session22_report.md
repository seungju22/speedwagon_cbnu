# 맵세션22 보고서 (2026-10-02) — 발표 자료와 캠퍼스 외관

## 시작
- 원본 xodr sha256 bf835cdfad0cea65 확인(map/maps/cbnu_internal_only_localtm_tags73_smooth.xodr)
- 재변환 실행 안 함. Autoware 저장소·OS·docs/logs 의 /home/gm 손대지 않음
- 순서: Phase 1 -> 2 -> 0 -> 3 -> 4 -> 5 (지시 우선순위대로)

## Phase 1 OSM 에서 CARLA 맵까지 (본체) — map/docs/how_the_map_was_made.md
- 10단계 + 부록(지표, 숫자 모음). 각 단계 무엇/왜/결과, 그림 구조, 예상 질문. 용어 첫 등장 시 풀이. 대본 없음
- 이번 세션에 원본으로 다시 확인한 사실
  - changeset 129142899: 2022-11-20, 편집자 user_17703046(탈퇴 계정 표기), 마지막 편집 way 312개 중 parking_aisle 236개(OSM 전체 425개 중)
  - 보정 73개의 원래 종류: parking_aisle 64 / footway 6 / path 2 / track 1. 그중 66개가 위 changeset 이 마지막 편집
  - xodr 머리말에 netconvert 설정이 통째로 남아 있음(keep-edges.by-type = 기본 12종, service 없음 / lanewidth 3.35 / sidewalk 2.80)
  - Osm2OdrSettings 공개 항목 11개 = 값 9 + 메서드 2(set_osm_way_types 등, 호출 안 함)
  - 투영 배율 이론값 k = 1/√(1−(cos36.63°·sin127.46°)²) = 1.297 = 실측 1.2974 와 일치
- 지시문 수치 정정
  - "벽 대조 실험 1604건 -> 0건": 같은 파일 대조는 세션12 2,139건(벽 1.0) -> 세션13 0건(벽 0). 1,604건은 평활화 전 파일(세션10)의 92m 정지
  - "충격량 방향": 세션10 은 차를 오른쪽으로(접촉 왼쪽), 세션12 는 왼쪽·앞으로(벽 끝이 차 앞 오른쪽에 낌, 추정). 둘 다 중앙선 벽으로 설명
  - "R_T 근거 물리 최소 6.15m": 6.15m 는 차 성능이 아니라 차선 3.35 + 인도 2.80 의 기하 한계(안쪽 인도 접힘)
  - "순간 최소 반경은 예측력이 없다(r1286 R2min 1.29m)": 두 개념이 섞임. 순간(해석적) 반경 = 이음매 스파이크로 기각(세션11),
    R2min = 2m 창 최솟값이 통과를 예측 못 한 사례가 r1286(세션20). 세션21 정정과 같은 내용
  - "캠퍼스 도로가 전부 parking_aisle": 보정 73개 중 64개. 나머지 9개는 footway/path/track
- 새로 드러난 연결: 후문 way 7개 중 2개(392632034, 452870644)는 세션4 후보 검토에서 지명 기반 오판으로 제외된 것, 5개는 50m 미만 필터

## Phase 2 PPT 구성 — map/docs/ppt_outline_2026-10-08.md
- 본 6장: 전경 / 정류장 5곳과 노선 / 먼저 밝히는 두 가지 / 주행 영상 4 / 정량 목표 / 남은 과제
- "먼저 밝히는 두 가지" 를 본 슬라이드 3번에 둠: (1) 최종 목표 정문->후문 유일 미검증 (2) Autoware 미연동 —
  BasicAgent 경로 추종, 위치는 시뮬레이터 참값, 인지·측위 없음, Autoware 는 Docker 구성·1.9.0(10718787) 고정까지
- 백업 8장: 태그 / 투영 / 벽 / 커브 반경 / 기각 가설 9 / 노선 후보 / 외관 / BasicAgent·참값 이유
- 주의 기록: 영상 파일 "정문to중문시험주행실패.webm"(10-02 01:26)은 시각상 세션20 중문 완주 주행. 세션18 실패 영상은 09-28 22:50:56 파일. 캡션 확인 필요

## Phase 0 후문 상태와 노선 후보 — map/docs/logs/s22_routes.md
- 스크립트 map/scripts/s22_routes.py(신규, 읽기전용), 로그 s22_routes.log
- 확인: road1898 은 j106 막다른 끝 회차 커넥터(9.89m, U턴형). road1333(484m) 앞뒤 연결 0. --route back 은 출력 후 exit 2
- 현재 맵, 종점 road1898 s=5.0, 교차로 U턴 금지
  - A 남문 경유 1,221.9m / 28 road / 회전 3회(전부 주행 이력 있는 커브) / 최소 평균R 10.12 / 최소 R2 6.48(종점 회차 제외) / 잔여지점 0
  - B 양성재 경유 1,705.7m / 50 road / 회전 12회 / 최소 평균R 5.54(r1786) / R2 1.32(r1190) / 남문 안 지남 / 잔여 3
  - C: 양성재->남문 2,276m, 남문->양성재 2,701m, 북문 회차 경유 1,902m, 중문 회차 경유 1,807m, k-최단 12개(1,222~1,448m, 전부 남문 경유)
  - 정류장 최다: 4곳(양성재·중문·북문·남문) 3,677m, 회차 2번, 회전 31회
  - 평균R 9m 이상은 A 하나. 방향 오류 전 후보 0
- 재변환 예측(시험 맵, 채택 안 됨): A 1,436m(새 커브 r1415 R2min 3.22·r1981 평균R 9.30), B 1,920m, 4곳 3,876m. 시험 맵에서는 중문 s=70 정차점이 성립 안 함(road 67.8m)
- 결론은 내지 않음. 노선 선택은 사용자·팀

## Phase 3 캠퍼스 외관 설계 — map/docs/props_plan.md
- 전제 정정: Town 레벨 인스턴스는 못 꺼내지만, Content 의 쿠킹된 정적 메시는 static.prop.mesh + mesh_path 로 소환 가능(세션15 실증).
  Building SM_ uasset 628개, Vegetation 195개(파일 수, 전부 메시인지는 미확인)
- map/scripts/s22_buildings.py(신규): OSM 건물 폴리곤 -> 면적 중심 위경도 -> CARLA. 투영 중심·offset 은 xodr 머리말에서 읽음(하드코딩 없음).
  자기검사 junction 90개 중앙값 3.35m. 결과 18동 map/data/processed/s22_buildings.csv
  - "양성재" 이름 폴리곤은 13x13m 1동뿐, 기숙사동은 S17-x(지선관·명덕관·신민관 등)로 따로 있어 함께 넣음
  - 신민관 S17-3 은 폴리곤이 도로·인도와 겹침(중심이 인도 끝 1.9m)
- map/scripts/place_props.py(신규): --list / --dry-run / --apply / --clear / --draw. 위경도를 서버 맵 머리말로 매번 변환.
  침범 검사 4규칙(차선·인도 위 점 금지, 주행 차선 끝 3m 여유, 루트 계획선 3.9m, 정차점 10m), 실패 시 도로 반대쪽 1m 씩 20m 까지 이동
  - 오프라인 검증: 도로 위 대조점·신민관 중심은 거부, 자연과학대학 본관 122x25m 발자국은 도로와 겹쳐 제외

## Phase 4 서버 확인
- 사전: Firefox 14 프로세스 발견 -> 사용자 종료 -> 0건(잡힌 2건은 검사 셸 자신). 서버 기동 전 23:24:40 avail 10Gi / swap 0B / gtt 34MiB
- 서버 사용자 기동, 명령 원문: cd ~/carla/CARLA_0.9.15 && ./CarlaUE4.sh -quality-level=Low -win (PID 5037). 23:24:58 Town10HD_Opt, 0.9.15
- 23:25:12 test_load_map_878.py --wall-height 0 exit 0: 878 / 95 / 1243, 서버 to_opendrive sha256 일치(session22_load.log)
- 메모리: 로드 직후 23:25:18 avail 2.2Gi / swap 904Mi / gtt 3.53GiB -> 2분 후 23:27:23 avail 4.6Gi / swap 893Mi / gtt 1.45GiB / RSS 1.75GiB -> 통과
- 목록 map/docs/logs/s22_prop_list.txt: static.prop 96 / vehicle 41. 나무 0(화분 8), 가로등 gardenlamp 1, 벤치 3, 쓰레기통류 12, 공사 자재 26, 건물 크기 0.
  자동 분류 정규식 오류("street" ⊃ "tree")로 파일 끝 손 분류 절을 정본으로 표시
- 시험 배치 23:28:07: 5/5 소환(도서관 구관 자리 SM_Apartment01_1 1, 나무 SM_Acer_02 2, 자연대 본관 양 끝 bench01 2), 자연대 SM_Block05 는 검사로 제외
  - 보이는가: 예(근접 카메라 사진 figures/25~27). 70m 높이 전경에서는 작아서 거의 안 보임(figures/24)
  - 도로 침범: 검사상 0, 최소 차선 여유 3.6m. 주행 안 함
  - 메모리: avail 4.6 -> 4.5Gi, gtt 1.46 그대로
  - SM_Apartment01_1 은 이름과 달리 2층 집 크기. bbox 는 여전히 믿을 수 없음(집 0.1x7.4x4.6)
- 새 발견: 디버그 상자(draw_box)가 RGB 카메라 센서 영상에 찍힘 + 선 주변 빛 번짐이 화면을 물들임. 지시문 3-4 전제와 반대.
  props_plan.md 3-4 정정. draw_string 은 이 사진들에서 안 보임
- 스크린샷: figures/24~27(디버그 빛 섞임). 깨끗한 사진 28 은 디버그 수명(600s) 만료 뒤 촬영 — 아래 마감 절에 결과 기록

## 하지 않은 것 (지시대로)
- 재변환, 후문 루트 구현, prop 전면 배치, 주행 영상 재촬영, 계획 길이 수정, 고도, Lanelet2·점군

## 마감
- 깨끗한 사진(디버그 수명 만료 후 23:39:55): figures/28_s22_props_wide_clean.png(전경, 집·나무), 29 나무, 30 집, 31 벤치. 발표용은 28·30
- 소품 정리 23:40: place_props.py --clear 5개 제거, 남은 static.prop 0, actor 는 spectator 1 뿐, 비동기(sync False)
- 23:40:32 avail 4.6Gi / swap 835Mi
- 서버 실행 중(사용자 종료)
- 원본 xodr sha256 bf835cdfad0cea65 (시작 bf835cdfad0cea65, 불변). test_drive.py bfdcefb0bb546289 (불변)
- 새 파일: how_the_map_was_made.md, ppt_outline_2026-10-08.md, props_plan.md, map_session22_report.md, logs/s22_routes.md·.log,
  logs/s22_buildings.log, logs/s22_prop_list.txt, logs/s22_props_dryrun.log, logs/s22_props_apply.log, logs/session22_load.log, logs/props_state.json,
  figures/24~31, data/processed/s22_buildings.csv, scripts/s22_routes.py, s22_buildings.py, place_props.py
- 미결: 큰 건물 메시(SM_Block·SM_Mall·SM_Office 계열) 크기 확인과 scale 조정, LiDAR 에 디버그 표시가 잡히는지(미확인),
  영상 파일명 "중문 실패" 캡션 확인(사용자), 노선 선택(팀)
- 다음 시작점: 발표(10/8) 후 10/9 재변환(10 way). 재변환 후 s22_routes.py 종점을 후문 정차점으로 바꿔 후보 재계산
