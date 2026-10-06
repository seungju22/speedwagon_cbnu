# 규민 — 충북대 캠퍼스 CARLA 시뮬레이션 맵

담당: 맵, 인지 (저장소 루트 `역할` 파일 기준)

## 1. 무엇을 만들었는가

충북대학교 캠퍼스 도로를 CARLA 0.9.15 에서 달릴 수 있는 OpenDRIVE 맵(`.xodr`)으로 만들고,
셔틀 정류장 노선을 BasicAgent 로 주행 검증했다.

- 맵 파일: `maps/cbnu_internal_only_localtm_tags73_smooth.xodr` (frozen_v1)
- sha256 앞 16자: `bf835cdfad0cea65`
- 규모: road 878 / junction 95 / spawn point 1243
- 실제 위치와의 축척 오차: 0.07% (유사변환 스케일 1.000709, 보정점 90개, 잔차 RMS 3.7m)

## 2. 어떻게 만들었는가 (OSM -> OpenDRIVE)

1. OpenStreetMap 캠퍼스 영역 다운로드 (`scripts/download_osm.sh`)
2. 태그 보정 73건: 변환기가 버리는 `highway=service` 등을 `unclassified` 로 (`scripts/fix_tags.py`)
3. 캠퍼스 경계 폴리곤 안·경계 way 만 남김 (`scripts/classify_internal.py`)
4. 급커브 평활화(필렛) (`scripts/smooth_osm_curves.py`)
5. CARLA Osm2Odr(내장 SUMO netconvert)로 변환 (`scripts/osm_to_xodr.py`)
   - 투영 `+proj=tmerc +lat_0=36.627298 +lon_0=127.456394 +ellps=WGS84`, 차선 폭 3.35m, 신호등 생성 끔
6. CARLA 로드 시 `wall_height=0` (반쪽 도로 구조에서 벽이 중앙선 위에 서는 문제. 같은 파일 대조 충돌 2,139건 -> 0건)

자세한 경위: `docs/how_the_map_was_made.md`

## 3. 현재 검증 상태 (정류장별)

조건: 빈 도로·맑음·평지(z=0), CARLA 0.9.15, BasicAgent, 동기 20Hz, 목표 20km/h.
거리는 출발점(정문 밖 공도 교차점)부터 주행 궤적, 시간은 시뮬레이션 시간

- 북문(도서관): 완주, 622.5m, 123.1s
- 남문(도서관): 완주, 726.8m, 143.8s
- 중문: 완주, 690.2m, 136.8s
- 양성재: 완주, 633.9m, 125.6s
- 4노선 충돌 0, 평균 속도 18.2km/h, 최고 속도 23.1~24.0km/h
- 정차 판정 거리 0.93m / 0.92m / 0.90m / 0.96m (판정 순간 정차점까지 거리, 정지 오차 아님)
- 실제 정지 위치 오차 0.33~1.06m (3회 중 2회만 1.0m 이내)
- 후문: 미검증. 지금 맵에서 갈 수 있는 가장 가까운 곳이 217m 떨어져 있다(후문 way 미변환)

연결성(재변환 전): 정문에서 도달 가능한 road 826/878 = 94.1%(차선 기준), 정문까지 왕복 가능 4/878 = 0.5%

숫자 출처: `docs/presentation_numbers_locked.md`

## 4. 알려진 한계

- 고도 미반영: 맵 전체 z=0. 실제 오르막(양성재 방향 등) 없음. 주행 결과는 평지 가정
- 교통 규제 미반영: 변환 입력 OSM 에 회전 제한 relation 0개. 신호등 없음(캠퍼스 실제도 비신호)
- 후문 노선 미검증: 재변환(후문 연결 way 추가) 후 확인 예정. 계획 `docs/reconversion_plan.md`
- 모든 도로가 방향당 1차로(정문 진입로 실제는 4차로)
- 막다른 길 14곳에 회차 연결로가 생성되지 않음(변환기 동작, `docs/logs/s23_deadend_cause.md`)
- 고원식 횡단보도·볼라드·게이트 등 현장 구조물은 OSM 에 거의 없어 맵에 없음(`docs/field_survey_*.md`)

## 5. 재현 방법

환경: Ubuntu 22.04, CARLA 0.9.15 패키지, 파이썬 venv 에 `carla==0.9.15`

**주의: 스크립트에 /home/gyumin 절대 경로가 하드코딩돼 있어 다른 환경에서는 경로 수정이 필요하다.**
(일부 문서·로그에는 이전 환경의 /home/gm 경로도 남아 있다. 기록이라 고치지 않았다)

원래 작업 폴더 구조는 `map/{data,docs,maps,scripts,tests}` 이고, 이 폴더의 `docs/ scripts/ tests/ maps/` 가 그 일부다.
원본 OSM(`data/raw`)과 중간 산출물은 올리지 않았다(`scripts/download_osm.sh` 로 다시 받는다)

순서
1. `scripts/download_osm.sh` -> `data/raw/cbnu_campus.osm`
2. `python scripts/fix_tags.py`
3. `python scripts/classify_internal.py`
4. `python scripts/smooth_osm_curves.py`
5. `python scripts/osm_to_xodr.py --input <평활화 OSM> --output <xodr>`
6. CARLA 서버 실행 후 `python tests/test_load_map_878.py --wall-height 0`
7. 주행: `python tests/test_drive.py --route north` (north / south / middle / yangseong)

주의: OSM 은 계속 편집되므로 오늘 다시 받으면 같은 맵이 나오지 않을 수 있다. 같은 입력이면 파이프라인 결과는 같다(재현성 확인 기록 `docs/logs/s18_D_reconvert.md`)

## 6. 라이선스 고지

- 원본 데이터: OpenStreetMap (© OpenStreetMap contributors)
- 라이선스: ODbL 1.0 (Open Database License). `maps/` 의 xodr 는 OSM 에서 파생된 데이터다
- 변환 도구: CARLA Osm2Odr (내장 SUMO netconvert)
- CARLA 에셋(3D 모델·텍스처 등)은 포함하지 않는다
- 일부 그림(`docs/figures/`)은 CARLA 0.9.15 화면 캡처다
- 일부 그림의 지도 배경: © OpenStreetMap contributors (`docs/figures/22_route_realmap.png`, `docs/figures/23_library_entrances.png`)
