# GitHub 공개 준비 초안 (세션24, 2026-10-04)

상태: 초안만. 업로드·저장소 생성·git init 하지 않음(CLAUDE.md: git push, 원격 저장소 생성 금지).
지금 campus_mobility_sim 은 git 저장소가 아니다(2026-10-04 확인). 아래 문안은 팀이 결정한 뒤 쓴다

## 1. 무엇을 공유하고 무엇을 빼나

공유 (직접 만든 것)
- map/scripts/*.py, download_osm.sh: 변환·분석 스크립트
- map/tests/test_drive.py, test_brake.py, test_load_map*.py: 주행·제동 시험 클라이언트(.bak_* 는 제외)
- map/docs/*.md, map/docs/logs/*.md: 세션 보고서·분석 문서
- map/maps/cbnu_internal_only_localtm_tags73_smooth.xodr: OSM 유래 -> ODbL 표기 필수(2절)
- map/data/processed/*.osm, cbnu_relation_polygon.json: OSM 유래 -> ODbL 표기 필수
- docker/*.yaml: 고정 버전 설정

빼는 것
- CARLA 에셋 전부: CARLA 패키지(~/carla/CARLA_0.9.15) 안의 언리얼 에셋·맵·차량 모델·바이너리. 재배포 불가로 다룬다
  (CARLA 코드는 MIT 지만 에셋은 별도 라이선스(CC-BY). 재배포 조건을 우리가 검토하지 않았으므로 넣지 않는다)
- .venv-carla: 가상환경. requirements 로 대신
- 대용량 로그: map/docs/logs 의 drive_log_*.csv(최대 3.8MB), brake_test_*.csv, *_stdout.log. 요약 .md 만 공유
- 백업: *.bak*, 백업 tar, test_drive.py.bak_s*
- 시험 맵: *_trial_s18_*.xodr, *_trial_*.osm (채택 안 된 산출물)
- 개인 경로: docs/, logs/ 안의 /home/gm, /home/gyumin 경로 표기 54개 파일. 치환 금지 규칙이 있으므로 고치지 않고, 공개 범위에서 판단
- 원본 OSM 덤프(data/raw/cbnu_campus.osm, 3MB): download_osm.sh 로 다시 받게 하고 파일은 뺄지 팀 결정(ODbL 표기만 하면 넣어도 됨)

## 2. ODbL 표기 문안 (OSM 유래 데이터)

README 와 맵 파일 옆 NOTICE 에 넣는다

```
Map data © OpenStreetMap contributors.
OpenStreetMap data is available under the Open Database License (ODbL) v1.0.
https://www.openstreetmap.org/copyright

This repository contains derived data (OpenDRIVE .xodr and processed .osm files)
produced from OpenStreetMap data. These derived databases are made available
under the ODbL v1.0: https://opendatacommons.org/licenses/odbl/1-0/
Modifications: tag changes (73 ways highway=service -> unclassified),
campus-boundary extraction, curve smoothing. See map/docs/how_the_map_was_made.md.
```

한국어 병기
```
지도 데이터 © OpenStreetMap 기여자. ODbL 1.0 으로 제공됨.
이 저장소의 .xodr 과 가공 .osm 은 OpenStreetMap 데이터에서 파생한 데이터베이스이며 ODbL 1.0 을 따른다.
```

- 그림(map/docs/figures, plot_route_realmap_s16.py 출력)에 OSM 타일 이미지를 배경으로 쓴 것은 그림 안이나 캡션에
  "© OpenStreetMap contributors" 를 넣는다
- 근거 URL: https://www.openstreetmap.org/copyright , https://osmfoundation.org/wiki/Licence/Attribution_Guidelines

## 3. .gitignore 초안

```
# 가상환경·캐시
.venv*/
__pycache__/
*.pyc

# CARLA 에셋·바이너리 (재배포 금지)
carla/
CARLA_*/
*.uasset
*.umap
*.pak
*.so
*.a

# 대용량 로그 (요약 .md 만 공유)
map/docs/logs/drive_log_*.csv
map/docs/logs/drive_plan_*.csv
map/docs/logs/collision_events_*.csv
map/docs/logs/brake_test_*.csv
map/docs/logs/*_stdout.log
map/logs/

# 백업·시험 산출물
*.bak
*.bak_*
*.tar
*.tar.gz
map/maps/*_trial_*.xodr
map/data/processed/*_trial_*.osm

# 영상·대용량 미디어
*.mp4
*.mkv
```

- drive_plan_*.csv 는 작으니(10KB) 재현용으로 남길지 팀 결정. 남기면 위 줄을 지운다

## 4. README 초안

```
# campus_mobility_sim

충북대학교 캠퍼스 도로를 OpenStreetMap 에서 가져와 CARLA 0.9.15 용 OpenDRIVE 맵으로
변환하고, 정류장 노선 주행을 시험한 PBL 프로젝트(2026-2).

## 포함
- map/scripts: OSM -> xodr 변환, 태그 보정, 곡선 평활화, 분석 스크립트
- map/tests: CARLA 주행·제동 시험 클라이언트 (BasicAgent, 동기 20Hz)
- map/maps: 변환 결과 xodr (OSM 유래, ODbL)
- map/docs: 세션 보고서, 맵 제작 과정(how_the_map_was_made.md), 현장 답사 기록

## 포함하지 않음
- CARLA 시뮬레이터와 그 에셋(맵·차량·건물 모델). 재배포하지 않는다.
  CARLA 0.9.15 는 공식 릴리스에서 직접 받는다:
  https://github.com/carla-simulator/carla/releases/tag/0.9.15
- 대용량 주행 로그 원본, 백업 파일

## 고정 버전
Ubuntu 22.04 / CARLA 0.9.15 / carla==0.9.15 (pip) / Autoware 1.9.0 (미연동)

## 실행 (요약)
1. CARLA 0.9.15 서버 실행 (별도 설치)
2. python map/tests/test_drive.py --route north

## 한계
맵은 평면(z=0)이다. 고원식 횡단보도·경사·노면 차이가 반영되지 않는다.
교차로 연결로는 변환기(netconvert)가 만든 기하이며 실측이 아니다.

## 라이선스
- 코드: (팀 결정 필요)
- 지도 데이터: © OpenStreetMap contributors, ODbL 1.0
```

## 5. 공개 전 결정할 것
- 코드 라이선스(MIT 등)
- 원본 OSM 덤프 포함 여부
- docs 의 개인 경로 표기 처리 방법(치환 금지 규칙과 충돌)
- 팀원 3명 이름·학번 노출 범위
