# 고지 (인계 3)

이 폴더의 지도 자료와, 이 자료를 돌리는 데 쓰는 남의 소프트웨어의 라이선스를 한 장에 모은 것임.
CARLA 에셋, Autoware 이미지, 가상환경은 이 묶음에 넣지 않았음. 받는 쪽이 각 배포처에서 받음.

## 파일별 라이선스

- MIT(규민/handoff3/ 맨 위 LICENSE): 우리가 쓴 스크립트(.py·.sh·launch.py), 문서(.md·.html), compose 오버레이와 설정 yaml, 목록·로그. 아래 두 묶음에 든 파일은 뺌
- ODbL 1.0: OSM 에서 나온 지도 자료. 다음 절의 목록과 OSM 원본·중간 파일(map/data/raw/cbnu_campus.osm, map/data/processed/ 의 osm·json, map/maps/ 의 xodr)
- Apache 2.0(Autoware 설정 파일을 고친 것). 원본에 라이선스 머리말은 없었고, 바꾼 것은 파일 머리 주석에 적음(Apache 2.0 4(b)). 라이선스 원문 docs/handoff_3/LICENSE-APACHE-2.0.txt(Apache 2.0 4(a))
  - 11/02 더함: integration/config/s73_vehicle_info_starex.param.yaml(원본 autoware_sample_vehicle_description vehicle_info.param.yaml), s73_sensor_kit_calibration_roof.yaml(원본 carla_sensor_kit_description sensor_kit_calibration.yaml), s73_sensor_mapping_pps545k_van.yaml(원본 autoware_carla_interface sensor_mapping_light_weight.yaml). 셋 다 머리 주석에 원본과 고친 값
  - integration/config/sensor_mapping_nocam.yaml, sensor_mapping_light32.yaml, sensor_mapping_nocam_pps545k.yaml: 원본 autoware_carla_interface 0.52.0 의 config/sensor_mapping_light_weight.yaml(이미지 universe-humble-1.9.0)
  - integration/config/diag_autoware-carla_gt.yaml, diag_autoware-carla_gt_nopcd.yaml: 원본 autoware_launch 의 config/system/diagnostics/autoware-carla.yaml(같은 이미지)
- 지도를 그린 그림(점검 페이지 figs·shots, 발표 그림의 지도 부분)은 ODbL Produced Work. 그림 자체는 MIT 로 두고 '© OpenStreetMap contributors' 고지를 붙임
- 이 밖에 남의 코드를 베껴 온 파일은 없음(10/27 에 목록 파일 머리말·출처 주석을 찾아 확인)

## 지도 자료: OpenStreetMap 파생, ODbL 1.0

원본: OpenStreetMap, © OpenStreetMap contributors. https://www.openstreetmap.org/copyright
라이선스: Open Data Commons Open Database License 1.0. https://opendatacommons.org/licenses/odbl/1-0/
내려받은 것: 2026-09-15, OSM API 0.6 map 요청(map/scripts/download_osm.sh), sha256 앞 16자 c383870eb4d1e196

아래는 모두 그 OSM 에서 만든 파생 데이터베이스임. 다시 나눠 줄 때는 같은 ODbL 1.0 조건을 따르고, 위 출처 줄을 남김.
- 도로망 OpenDRIVE: map/maps/cbnu_campus_frozen_v31.xodr(214507af)
- Autoware Lanelet2 지도: autoware_map/cbnu_campus_v31/lanelet2_map.osm(28a57711), map/maps/v32a·v32b 의 lanelet2_map.osm(횡단보도·방지턱 자리는 OSM 태그와 로드뷰로 고름)
- 정류장·승강장·노선: map/data/stops_v3.yaml, map/data/shuttle_platforms_v31_r1·r2.yaml, map/data/routes/*.yaml
- 장식 레이어 배치 자료: map/data/buildings_layout_v3.csv, buildings_layout_tier2_s66.csv, trees_layout_s66.csv 와 배치 결과 csv·json

손으로 고른 값(정류장 자리, 횡단보도 keep, 건물 높이 가정 등)이 들어 있어도 OSM 에서 나온 데이터베이스라 ODbL 을 따름.
ODbL 4.6 에 따라 파생 데이터베이스는 공개할 때 그 데이터베이스나 만드는 방법을 함께 내놓아야 함. 만드는 방법은 docs/handoff_3/map_reproduce.md 와 그 스크립트임.
이 자료로 그린 그림·영상(점검 페이지, 발표 그림의 지도 부분)은 ODbL 의 Produced Work 임. 공개할 때 '© OpenStreetMap contributors' 고지를 붙임(ODbL 4.3).

## 쓰는 소프트웨어

| 이름 | 판 | 라이선스 | 근거 |
|---|---|---|---|
| CARLA 시뮬레이터 코드 | 0.9.15 | MIT | 패키지 LICENSE, https://github.com/carla-simulator/carla/blob/0.9.15/README.md |
| CARLA 에셋 | 0.9.15 | CC-BY | 같은 README 'CARLA specific assets are distributed under CC-BY License' |
| Unreal Engine 4(CARLA 안) | - | Epic 자체 조건 | 같은 README |
| carla 파이썬 패키지 | 0.9.15 | MIT | PyPI 메타데이터 License: MIT License |
| CARLA Osm2Odr(OSM -> xodr 변환기) | 0.9.15 안 | EPL 2.0(SUMO netconvert 를 고친 것) | CARLA Util/BuildTools/BuildOSM2ODR.sh 의 저장소 carla-simulator/sumo, 그 LICENSE |
| Osm2Odr 의존 PROJ·SQLite·Xerces-C | - | X/MIT·공개 도메인·Apache 2.0 | CARLA README 'SUMO integration' 절 |
| Autoware | 1.9.0 | Apache 2.0 | ~/autoware LICENSE, 이미지 안 package.xml |
| autoware_carla_interface(브리지) | 1.9.0 이미지 | Apache 2.0 | 이미지 안 package.xml |
| autoware_lanelet2_extension | 1.9.0 이미지 | Apache 2.0 | 이미지 안 package.xml |
| Lanelet2(lanelet2_core·io) | 1.9.0 이미지 | BSD 3-Clause | 이미지 안 package.xml, https://github.com/fzi-forschungszentrum-informatik/Lanelet2 |
| Autoware Docker 이미지 | universe-humble-1.9.0 | 안의 패키지마다 다름 | ghcr.io/autowarefoundation/autoware |

- 이 묶음은 위 소프트웨어의 소스나 바이너리를 넣지 않음. compose 오버레이·설정 파일·스크립트로 부르기만 함

## 논문

비 조건 모델은 Goodin et al. 2019, Electronics 8(1) 89(CC BY 4.0 공개 논문)의 식을 따라 짠 것임. 논문 본문은 넣지 않음. perception/docs/rain_model.md

## 11/03 더함(v3.3)
- v3.3a·v3.3b Lanelet2(map/maps/v33a·v33b)와 승강장·순환 노선 파일도 OSM 파생 데이터베이스라 ODbL 1.0. 출처 줄 '© OpenStreetMap contributors' 를 남김. 만드는 법은 map/docs/v33b_readme.md 의 스크립트 순서
- OSM 과 다르게 뺀 길(현장 확인)은 map/maps/v33a/v33a_changes.json 에 근거와 함께 적음

## 11/03~11/04 더함(v3.3c·v3.3d, 스프린터)
- map/maps/v33c·v33d 의 Lanelet2, 승강장 파일 shuttle_platforms_v33c_r1·v33d_r1(_sprinter)·순환 노선 circulation_v33c_r1·v33d_r1(_sprinter)도 OSM 파생이라 ODbL 1.0. 만드는 법은 map/docs/v33c_readme.md, v33d_readme.md
- Apache 2.0(Autoware 설정을 고친 것, 머리 주석에 원본과 바꾼 값)
  - integration/config/s75_vehicle_info_sprinter.param.yaml: 원본 autoware_sample_vehicle_description vehicle_info.param.yaml
  - integration/config/s76_raw_vehicle_cmd_converter_sprinter.param.yaml: 원본 autoware_carla_interface config/raw_vehicle_cmd_converter.param.yaml
  - integration/config/s76_mirror_sprinter.param.yaml: 원본 sample_vehicle_description mirror.param.yaml
  - integration/config/s76_sprinter_accel_map.csv, s76_sprinter_brake_map.csv: autoware_carla_interface accel_map.csv·brake_map.csv 와 같은 꼴, 값은 CARLA 에서 새로 잼
  - integration/config/s74_sensor_mapping_van_pick.yaml: 원본 autoware_carla_interface sensor_mapping_light_weight.yaml
- map/docs/figs/v33d_sprinter_door.jpg 는 CARLA 화면을 찍은 것(CARLA 에셋 CC-BY)
