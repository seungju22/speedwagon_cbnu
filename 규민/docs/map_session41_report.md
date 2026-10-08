# 맵세션41 보고서 — 인계 1 서버 확인 (2026-10-09)

## 0. 선행 확인
- 해시: frozen_v1 bf835cdfad0cea65, 동결본 5240ca8b883e42c0, test_drive.py 2935c016f9315b70 (load_campus_world.py 0abbe948ee192947)
- 첫 확인 00:22: firefox 12건 -> 멈추고 보고. 사용자 종료 뒤 00:22:37 firefox 0, CarlaUE4 0
- 기준선: avail 6.6Gi, swap 343Mi (drop_caches 권고했으나 실행 여부 미확인. 서버 기동 직후 avail 5377Mi)

## 1. 서버 (사용자 기동)
- 명령: cd ~/carla/CARLA_0.9.15 && ./CarlaUE4.sh -quality-level=Low -win, PID 9383
- 기동 직후 00:23:15 avail 5377Mi swap 349Mi / 2분 뒤 00:25:18 avail 2874Mi swap 2486Mi -> 기준(2048Mi / 4096Mi) 안
- 실행 래퍼: map/scripts/s41_run.sh(신규, s38_back_run.sh 방식: 서버 기동·종료 없음, avail < 1024Mi 즉시 kill 감시, 주행 전 기준 확인). 감시 kill 0

## 2. 로더 첫 실행 [확인] — logs/s41_load.log
- python map/scripts/load_campus_world.py (기본값 = 동결본)
- 출력: map_sha256=5240ca8b883e42c0, 생성 인자 7개(wall_height 0.0 등), 서버 0.9.15, 맵 이름 Carla/Maps/OpenDriveMap,
  서버 to_opendrive sha256=5240ca8b883e42c0(파일과 일치), 도로 917 / 교차로 112, 스폰 지점 1302
- 동결 기록(map_frozen_v2.md 917 / 112 / 1302)과 같음
- 메모리: 로더 직후 avail 2650Mi swap 1053Mi, 2분 뒤 주행 전 5171Mi / 1049Mi

## 3. 4 노선 회귀 [확인] — logs/s41_{north,south,middle,yangseong}.log, s41_summary.log
- test_drive.py 2935c016f9315b70, --xodr maps/cbnu_campus_frozen_v2.xodr, 2번 로더로 띄운 월드, 기본 인자. 서버 해시 일치(로그 server_map_sha256)
- 판정: 세션33 개정 기준(reconversion_plan.md 9-3: 완주, 충돌 0, 대기 제외 시간 ±1.0s, 궤적 ±2.0m)
  - 북문: 완주, 622.5m(기준 622.5, 0.0), 123.2s(123.1, +0.1), 대기 아니오, 정차점 0.94m, 충돌 0 -> 통과
  - 남문: 완주, 726.7m(726.8, -0.1), 143.9s(143.8, +0.1), 대기 아니오, 0.98m, 충돌 0 -> 통과
  - 중문: 완주, 690.2m(690.2, 0.0), 136.6s(136.8, -0.2), 대기 아니오, 0.89m, 충돌 0 -> 통과
  - 양성재: 완주, 633.9m(633.9, 0.0), 125.7s(125.6, +0.1), 대기 아니오, 0.93m, 충돌 0 -> 통과
- 세션38 test_drive.py 변경(back 전용 필드) 뒤 첫 4 노선 주행. 어긋남 없음. 값 조정 없음
- 메모리: 주행 후 avail 5162~5170Mi, swap 1049Mi

## 4. 차량 원값 [확인] — logs/s41_vehicle_raw.log (scripts/s39_vehicle_raw.py, 주행 없음, 소환 직후 기록 후 지움)
- 위치: r1392 s=28 lane -1. 바퀴 인덱스 0·1 = 앞(max_steer 70), 2·3 = 뒤(max_steer 0)
- 단위: WheelPhysicsControl.position 은 cm(세계 좌표). cm 로 보면 차 원점까지 1.4~3.3m, m 로 보면 9만 6천 m -> cm 확정
  - z 원값 = 바퀴 반지름 값과 같다(fusorosa 63.00 / radius 63.00, a2 32.99 / 33.00): 바퀴 중심 높이 cm
- fusorosa: wheel[0] (33692.47, -91407.96, 63.00), [1] (33538.76, -91196.20, 63.00), [2] (33225.77, -91723.43, 63.00), [3] (33094.22, -91542.20, 63.00)
  - 앞뒤 투영 앞 +3.015m / 뒤 -2.615m, 좌우 앞 ±1.308m / 뒤 ±1.120m. 축거 = 앞 [1,0] 중점 - 뒤 [2,3] 중점 = 5.630m
  - bbox 길이 10.273 폭 3.944 높이 4.253m, 중심 오프셋 (-0.445, -0.121, 2.127)
- sprinter: 앞 +1.932 / 뒤 -1.729m, 축거 3.662m (앞 [0,1], 뒤 [3,2]), bbox 5.915 x 1.988 x 2.561m
- prius: 앞 +1.420 / 뒤 -1.399m, 축거 2.819m (앞 [1,0], 뒤 [2,3]), bbox 4.514 x 2.007 x 1.525m
- audi.a2: 앞 +1.250 / 뒤 -1.257m, 축거 2.506m (앞 [1,0], 뒤 [3,2]), bbox 3.705 x 1.789 x 1.549m
- 세션38 계산값(축거 5.630 / 3.662 / 2.819 / 2.506)과 같음. fusorosa 축거는 Rosa 공개 축거 3.490~4.550m 보다 1.08m 이상 크다(세션39 기록 그대로)

## 5. GNSS 기준 실측 [확인] — logs/s41_gnss.log (scripts/s41_gnss.py)
- 정문(r1278 s=0.0)·후문(r1151 s=23.1) lane -1 위 a2(physics 끔), GNSS 를 차 원점에 부착. 노이즈 6개 속성 0.0, noise_seed 0 명시. 동기 0.05s 3틱째 값
- 정문: 센서 (36.6370658, 127.4588325) / stops_v2.yaml (36.6326490, 127.4529988) 차 716.44m / transform_to_geolocation 과 0.000m
- 후문: 센서 (36.6292716, 127.4687009) / stops_v2.yaml (36.6248296, 127.4628555) 차 719.18m / transform_to_geolocation 과 0.000m
- 세션40 C(오프라인 계산 정문 716.44m, 후문 719.18m, offset 크기 718.07m)와 같음. GNSS 센서는 transform_to_geolocation 과 같은 기준이다
- 센서 고도 0.500(차 원점 z 0.5 그대로, 지도 고도 0)

## 6. README 갱신 (2·3 통과) — docs/handoff_1_README.md
- 3절 월드 띄우기: load_campus_world.py 로 교체, 출력 예(917/112/1302, 해시 일치), 예전 명령은 한 줄 남김, 생성 인자 7개 값 명시
- 3절 끝 "좌표 기준" 추가: 시뮬레이터 안은 CARLA 좌표 / stops_v2.yaml 위경도는 실제 위경도 / CARLA GNSS 와 약 718m 차(실측 716.44·719.18m)
- 세션41 4 노선 결과 한 줄 추가. 평가어 없음

## 7. 마감
- 남은 액터: 양성재 회차 하네스 차 a2 31(브레이크 유지로 남겨둔 것) 지움 -> 차량·센서·소품 0
  - 참고: s39_vehicle_raw.py·s41_gnss.py 끝의 "남은" 목록에 지운 직후 액터가 잠깐 보였다(조회 시점). 다시 조회해 0 확인(logs/s41_cleanup.log)
- 설정: synchronous_mode False, no_rendering_mode False(비동기 복원, fixed_delta 미설정)
- 해시 재확인: frozen_v1 bf835cdfad0cea65, v2 후보 5240ca8b883e42c0, 동결본 5240ca8b883e42c0, test_drive.py 2935c016f9315b70, load_campus_world.py 0abbe948ee192947 — 변경 없음
- 끝 메모리: avail 5148Mi swap 1048Mi
- 인계 1 조건(세션40 목표 "팀원이 README 한 장만 보고 CARLA 에서 캠퍼스 월드를 띄울 수 있으면 된다"):
  - 로더 실행 통과, README 를 로더 기준으로 갱신, 로더로 띄운 월드에서 4 노선 회귀 통과 -> 이 PC 기준으로는 충족
  - 남은 것: README·stops_v2.yaml·동결본이 sync 대상 밖(세션40 B 안 승인 대기). 저장소로 가기 전까지 팀원은 이 파일들을 못 받는다
  - 인계 선언과 --apply 는 사용자
- 새 파일: map/scripts/s41_run.sh·s41_gnss.py, logs/s41_runs_stdout.log·s41_load.log·s41_{north,south,middle,yangseong}.log·s41_summary.log·s41_vehicle_raw.log·s41_gnss.log·s41_cleanup.log, 주행 CSV 4조
- 수정: docs/handoff_1_README.md(3절). xodr·test_drive.py·test_load_map_878.py·sync_to_repo.sh 수정 없음. git·--apply 없음
- CARLA 종료는 사용자
- 다음 시작점: sync 대상 안(세션40 B) 승인 -> 수정 -> 미리보기 -> 사용자 --apply·인계 1 선언

---

# 세션41 추가 — sync 대상 확장(세션40 B안 승인) + README "v2 의 한계" (2026-10-09, 서버 없음)

## 1. sync_to_repo.sh 수정 [확인]
- 백업 map/scripts/sync_to_repo.sh.bak_s41 (수정 전 888ee0a51fe8dbb9). *.bak* 는 sync 제외 규칙이라 저장소로 안 간다
- 변경(diff 4곳)
  - MAPS 에 cbnu_campus_frozen_v2.xodr 추가(v1 은 그대로 둠)
  - DATA=(stops_v2.yaml): map/data/<목록> -> 규민/data/. data/ 전체는 OSM 원본·중간 파일이 있어 목록만
  - TOPDOCS=(handoff_1_README.md): 작업 폴더 최상위 docs/<목록> -> 규민/docs/. 최상위 docs 에는 개인 환경 기록이 있어 목록만
  - 토큰 검사(grep) 범위: docs·scripts·tests·README 에 MAPS·DATA·TOPDOCS 의 모든 파일 경로를 더함
- bash -n 통과

## 2. ODbL 표기 확인 [확인]
- v1 규칙: maps/NOTICE.md 에 파일 이름·해시, "OpenStreetMap, © OpenStreetMap contributors", "ODbL 1.0, OSM 파생 데이터"
- 확인 결과 NOTICE.md 에 v2 동결본 줄이 없었다 -> v1 과 같은 형식으로 한 줄 추가, "위 두 파일 모두 OSM 파생 데이터다" 로 고침
- stops_v2.yaml 도 OSM 파생(좌표 = OSM 파생 지도 + OSM way 442595850 노드) -> 생성 스크립트 s40_stops.py 머리 주석에
  "OpenStreetMap, (c) OpenStreetMap contributors, ODbL 1.0" 한 줄 추가 후 다시 생성. diff = 그 한 줄만
- handoff README 4절에 출처 한 줄. 동결본 xodr 자체는 수정하지 않음(xodr 수정 금지)

## 3. 미리보기 (--dry-run, logs/s41_sync_preview.log) — --apply 안 함
- docs 295(신규 282), scripts 47(신규 46), tests 1, maps 2(동결본 신규, NOTICE.md 갱신), data 1(stops_v2.yaml 신규, 규민/data 새로 생김), 최상위 docs 1(handoff_1_README.md 신규), README 0
- 크기 합 약 23.3MB(docs 20.7, maps 2.2, scripts 0.2, tests 0.1). 5MB 넘는 파일 0
- 미리보기의 "created directory 규민/data" 는 dry-run 표시일 뿐 실제로 안 생김(확인)
- 기존 파일 갱신(신규 아님): field_survey_2026-10-04·10-06, presentation_2026-10-08, presentation_numbers_locked, reconversion_plan, logs/drive_plan_dry 6개, s23_connectivity(v1), s24_reconv_list, scripts/s27_reconvert.py, tests/test_drive.py, maps/NOTICE.md
- 인계 1 핵심 파일이 목록에 있음: maps/cbnu_campus_frozen_v2.xodr, maps/NOTICE.md, data/stops_v2.yaml, docs/handoff_1_README.md, scripts/load_campus_world.py, docs/map_frozen_v2.md

## 4. README "v2 의 한계" (5-1절 추가) — 지도에서 확인한 뒤 씀
- 확인 스크립트 scripts/s41_v2_limits.py, logs/s41_v2_limits.log, logs/s41_backgate_paths.log
- 캠퍼스 경계를 지나는 주행 차선 12(들어옴 6, 나감 6). 그중 정문·후문 쪽
  - 정문: 들어옴 r1278(정류장에서 25.3m) -> 캠퍼스 안 도달 가능. 나감 r1150(22.7m) 은 있으나 캠퍼스 안(중문 road)에서 도달 불가
  - 후문: 나감 r1151(57.3m) 캠퍼스 안에서 도달 가능. 들어옴 r1279(56.6m) 의 앞 road 는 r1418(게이트 밖 회차 연결로) 하나뿐,
    들어옴 r1368(27.5m) 은 앞 road 없음 -> 바깥 공도에서 후문으로 들어오는 길 없음
  - 지시 문장 "후문 게이트는 나가는 방향만" 과 지도 사이 차이: 게이트 밖 회차(r1151 -> r1418 -> r1279)로 다시 들어오는 길은 있다. README 에 그대로 적음
- 정류장 6지점 30쌍(같은 road 면 s 순서까지): 25/30 경로 있음. 없는 5쌍 = 다른 5지점 -> 정문 전부
  - 정문 정류장(r1278 s=0)은 바깥 공도에서 들어오는 road 의 시작점이라 캠퍼스 안에서 갈 수 없다
- README 에 적은 것: 정문 들어오는 방향만 / 후문 나가는 방향만(+회차로 다시 들어오는 길) / 25 of 30 / v3 에서 양방향·전 경로 맞추고 버전업 공지 / 스크립트는 road id 대신 stops 정류장 이름·좌표
- README 4절 경로를 저장소 기준 data/stops_v2.yaml 로 바꿈(작업 폴더 경로 병기)

## 5. 마감
- 해시: 동결본 5240ca8b883e42c0 불변, frozen_v1 bf835cdfad0cea65 불변. 수정 파일 sync_to_repo.sh, maps/NOTICE.md, data/stops_v2.yaml(주석 1줄), scripts/s40_stops.py(주석 1줄), docs/handoff_1_README.md
- 새 파일: scripts/s41_v2_limits.py, sync_to_repo.sh.bak_s41, logs/s41_sync_preview.log·s41_v2_limits.log·s41_backgate_paths.log
- git·--apply 없음. --apply 는 사용자 지시 대기
- 참고: 사용자가 붙여 준 backlog v2.1 본문은 파일로 저장하지 않았다(지시 없음). docs/backlog_v2.md·backlog_status.md 는 여전히 없다
