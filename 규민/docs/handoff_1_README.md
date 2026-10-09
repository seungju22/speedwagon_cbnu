
# 인계 1. CARLA 에서 충북대 캠퍼스 월드 띄우기 (2026-10-09)

이 문서대로 하면 CARLA 서버를 켜고 캠퍼스 도로망(v2 동결본)을 올릴 수 있다.
경로는 팀 저장소 `speedwagon_cbnu/규민/` 기준이다. 작업 폴더에서는 앞에 `map/` 이 붙는다.

## 1. 준비물

Ubuntu 22.04 와 CARLA 0.9.15 패키지가 필요하다. 0.9.16, 0.10 은 쓰지 않는다.
https://github.com/carla-simulator/carla/releases/tag/0.9.15

압축은 `~/carla/CARLA_0.9.15` 에 푼다. tests/test_drive.py 가 이 경로의 PythonAPI/carla(agents)를 읽는다(test_drive.py 128행).

파이썬 3.10 가상환경에 `pip install carla==0.9.15` 를 설치한다(https://pypi.org/project/carla/0.9.15/). 작업 PC 는 Python 3.10.12, carla 0.9.15 였다.

지도 파일은 `maps/cbnu_campus_frozen_v2.xodr` 이다. 받은 뒤 `sha256sum maps/cbnu_campus_frozen_v2.xodr` 로 앞 16자가 `5240ca8b883e42c0` 인지 본다.

메모리가 빠듯하다. 작업 PC(가용 약 12.7GiB, 내장 그래픽)에서는 서버만 켜도 가용 메모리가 2GiB 대까지 내려갔다. 가용 2GiB 미만이거나 swap 이 4GiB 를 넘으면 멈춘다.

## 2. 서버 띄우기

```
cd ~/carla/CARLA_0.9.15
./CarlaUE4.sh -quality-level=Low -win
```

처음 뜨는 기본 맵(Town10)은 그대로 두고 3절로 간다. 포트는 2000 이다.
근거: https://carla.readthedocs.io/en/latest/start_quickstart/ , https://carla.readthedocs.io/en/latest/adv_rendering_options/

## 3. 캠퍼스 월드 띄우기

```
python scripts/load_campus_world.py
```

기본 지도는 `maps/cbnu_campus_frozen_v2.xodr` 이다. 다른 파일은 `--xodr PATH`, 다른 서버는 `--host`·`--port` 로 준다.
파일 해시가 `5240ca8b883e42c0` 로 시작하지 않으면 경고를 찍는다.

정상이면 출력에 다음 줄이 나온다.

```
map_sha256=5240ca8b883e42c0 (cbnu_campus_frozen_v2.xodr)
서버 to_opendrive sha256=5240ca8b883e42c0 (파일과 일치)
도로 917 / 교차로 112
스폰 지점 1302
```

예전 명령도 결과가 같다. `python tests/test_load_map_878.py --wall-height 0 --xodr maps/cbnu_campus_frozen_v2.xodr`

생성 인자 7개는 로더 코드에 값으로 적었다. vertex_distance 2.0, max_road_length 50.0, wall_height 0.0, additional_width 0.6, smooth_junctions True, enable_mesh_visibility True, enable_pedestrian_navigation True. wall_height 말고는 CARLA 0.9.15 기본값과 같다.

벽 높이는 0 이어야 한다. 예전 명령에서도 `--wall-height 0` 을 빼지 않는다. 이 지도는 양방향 도로를 단방향 반쪽 도로 둘로 나눈 구조라, 기본값(벽 1m)이면 중앙선 위에 벽이 선다.

월드를 다시 불러오면(generate_opendrive_world, load_world) 그 전에 띄운 차와 소품은 사라진다.

노선 주행 확인은 선택이다. `python tests/test_drive.py --route north --xodr maps/cbnu_campus_frozen_v2.xodr` (노선 north, south, middle, yangseong, back)
2026-10-09 에 로더로 띄운 월드에서 4 노선을 한 번씩 달렸다. 북문 622.5m/123.2s, 남문 726.7m/143.9s, 중문 690.2m/136.6s, 양성재 633.9m/125.7s, 충돌 0 이었다.

### 좌표 기준

시뮬레이터 안은 CARLA 좌표(road id·s·lane, 또는 x·y m)다. 노선과 정류장 위치는 이 좌표로 쓴다.
stops_v2.yaml 의 lat·lon 은 실제 위경도(OSM 과 같은 기준)다.
CARLA GNSS 센서 값(= `transform_to_geolocation`)은 실제 위경도와 약 718m 다르다. 실측은 정문 716.44m, 후문 719.18m 였다(노이즈 0). CARLA 가 xodr 머리말의 `<offset>`(718.07m)을 읽지 않기 때문이다.

## 4. 정류장 파일 읽는 법 (`data/stops_v2.yaml`)

작업 폴더에서는 `map/data/stops_v2.yaml` 이다. 데이터 출처는 OpenStreetMap, 라이선스는 ODbL 1.0 이다(파일 머리 주석).

`stops` 에 6지점이 있다. 정문, 도서관 북문, 도서관 남문, 중문, 양성재, 후문이다. 지점마다 다음 값이 있다.

- `road_id`, `s`(road 시작에서 m), `lane_id`(-1 = 진행 방향 오른쪽 주행 차선). CARLA 에서 `world.get_map().get_waypoint_xodr(road_id, lane_id, s)` 로 위치를 얻는다
- `carla_xyz`: 위 waypoint 의 CARLA 좌표(m)
- `lat`, `lon`: WGS84. xodr 머리말 투영과 offset 으로 역산했다. `carla.Map.transform_to_geolocation` 값과 다르고, 그 함수는 쓰지 않는다
- `inside_campus_boundary`: 정문만 false 다. 노선 출발점이 캠퍼스 경계 밖 약 24m 의 공도 교차점이다
- `stop_zone_length_m`: 아직 비어 있다

`depot_candidates` 에는 박물관 버스 차고지(좌표 있음)와 N14 주차장(좌표 기록 없음, 빈 값)이 있다. 둘 중 하나로 정하지 않았다.

## 5. 알려진 한계

- 고도가 없다. 도로 917개의 elevation 이 모두 0 이다. 오르막과 고원식 횡단보도도 없다
- 교통 규제가 없다. 신호·표지(signal) 0개, 회전 제한 relation 입력 0개다
- 정문에서 나가 정문으로 돌아오는 road 는 917개 중 4개다. 정문 출구 way 를 넣지 않은 판(B안)이다
- 후문 고리 일부(r1368, 1892, 1893, 1894)는 정문에서 갈 수 없다
- 주행 시험은 `vehicle.audi.a2` 로만 했다(tests/test_drive.py 1058행). Autoware 브리지 기본 차는 `vehicle.toyota.prius` 로 알려져 있지만 이 PC 의 브리지 설정 파일로 확인하지는 않았다
- CARLA 조향 범위가 실차보다 크다. fusorosa, sprinter, prius, a2 의 앞바퀴 max_steer_angle API 값이 모두 70도다
- Autoware sample_vehicle(wheel_base 2.79m, max_steer 0.70rad)의 뒤축 최소 반경은 3.312m 다. 차선 중심 반경이 이보다 작은 구간은 북문 1, 남문 0, 중문 5, 양성재 0, 후문 4 곳이다. Autoware 로는 아직 시험하지 않았다
- CARLA fusorosa 바운딩 박스 길이는 10.273m 다. 실차 Rosa 공개 길이는 6,245~7,730mm(FUSO 필리핀 제원표)라고 받았지만 원문은 확인하지 않았다
- 후문 노선은 r1391 에서 노선 전용 설정(감속 8.9km/h, 계획점 간격 1m, 겨냥 1.0m)을 써서 통과했다. 다른 차량이나 플래너에서는 따로 확인해야 한다
- 건물과 나무는 지도에 없다. 소품 소환 시험만 했다

## 5-1. v2 의 한계 (도로망 방향과 경로)

- 정문 게이트는 들어오는 방향만 있다. 바깥 공도에서 들어오는 r1278 만 쓸 수 있다. 나가는 차선 r1150 은 있지만 캠퍼스 안에서 갈 수 없다
- 후문 게이트는 나가는 방향만 있다. 캠퍼스에서 나가는 r1151 은 있고, 바깥 공도에서 들어오는 길은 없다. 게이트 바로 밖 회차 연결로 r1418 로 돌아 다시 들어오는 r1279 는 있다. 그 밖의 들어오는 차선 r1368 은 앞에 이어진 도로가 없다
- 정류장끼리 모든 방향으로 다니지는 못한다. 6지점 30쌍 중 25쌍만 경로가 있다. 안 되는 5쌍은 모두 "다른 정류장 -> 정문"이다

v3 에서 게이트 양방향과 정류장 사이 전 경로를 맞추고, 새 파일과 새 해시로 공지한다.
v3 에서 road id 가 바뀐다. 스크립트는 road id 대신 stops_v2.yaml 의 정류장 이름과 좌표를 쓴다.

## 6. 하지 말 것

xodr 파일을 고치지 않는다. 맵 파일 이름도 바꾸지 않는다. 로그와 노선표가 파일 해시와 이름으로 맵을 구분한다.

## 7. 변경 규칙

도로망은 새 버전 파일과 새 해시로만 바뀐다. 바뀌면 팀에 공지한다.
지금 기준은 frozen_v2 `5240ca8b883e42c0` 이다. 이전 판은 frozen_v1 `bf835cdfad0cea65` 다.
