# 캠퍼스 v3 에서 Autoware 자율주행 돌리기

2026-10-10 I5 기준. CARLA 0.9.15, Autoware 1.9.0 이미지, 브리지 autoware_carla_interface 0.52.0.
경로는 `~/campus_mobility_sim` 을 `C` 로 줄여 씀. Town01 쪽은 howto_town01.md 를 봄.

## 지도 폴더 이름 규칙

- 브리지 런치(autoware_carla_interface.launch.xml 20행)는 `map_path` 의 마지막 폴더 이름을 CARLA 맵 이름으로 씀.
  그 이름으로 `client.load_world_if_different()` 를 부름(carla_autoware.py 138행).
- 캠퍼스는 `generate_opendrive_world` 로 올림. 그 월드의 이름은 `Carla/Maps/OpenDriveMap` 임.
- 그래서 지도 폴더 이름은 `OpenDriveMap` 이어야 함. 다르면 브리지가 다른 맵을 불러와 캠퍼스가 사라짐.
- 폴더는 도커에 이미 마운트된 `~/autoware_data/maps/` 안에 둠(컨테이너 `/home/aw/autoware_data/maps/OpenDriveMap`).
- 링크는 쓰지 않음. 마운트 밖을 가리키면 컨테이너에서 안 풀림. 복사함.
- 원본은 `C/autoware_map/cbnu_campus_v3/` 임. 고친 뒤에는 `bash C/autoware_map/scripts/sync_map_folder.sh` 로 복사함.
- 들어 있는 것: lanelet2_map.osm, map_projector_info.yaml(`projector_type: Local`). 점군 지도는 없음.

## 시뮬레이터 LiDAR (2026-10-14 세션59 바꿈)

- 센서 파일 `C/integration/config/sensor_mapping_nocam.yaml`(컨테이너는 `~/autoware_data/maps/i1_config/` 사본, sha256 b7189e92)의 LiDAR 를
  브리지 기본(1.9.0 이미지 sensor_mapping.yaml)과 같게 바꿨음: 64채널, 초당 30만 점, 범위 100m, 회전 20Hz, 상하 +10~-30도, 장착은 센서 킷 그대로. 카메라는 계속 뺌.
- 이전 값(I1 부터 세션58 까지)은 32채널, 초당 10만 점, 50m 였음(.bak_s59, 0f85dcb0). 세션59 부터 '가벼운 LiDAR' 라고 부름.
- 바꾼 근거(세션59 성능 시험 s59p_ref, p_middle_north, 레이어 732 켬): 실시간 0.998배(같음), 주행 중 최소 avail 2,445MiB(세션58 2,771~2,865),
  컨테이너 CPU 평균 190.7%(세션58 181~183%), 컨테이너 메모리 2.86~2.95GiB(세션58 2.36~2.51), 도착 81.0s·정지 앞뒤 -0.440m(I6-2 81.1s·-0.439m).
  규칙(실시간 0.99배 이상, 최소 avail 2GiB 이상)을 넘었음. 여유가 0.4GiB 쯤 줄었으니 카메라·bag 을 같이 돌리면 감시 로그를 봄.
- 브리지는 range·rotation·channels·fov·pps 만 넣음(carla_wrapper.py 249~256행). 그래서 무작위 버림(CARLA 기본 0.45)은 그대로임.
  버림 0 의 기준 점군이 필요하면 브리지 밖에서 따로 달아 씀(perception/tools/s59_lidar_bench.py 참고).
- 되돌리려면 두 곳 모두 .bak_s59 를 sensor_mapping_nocam.yaml 로 복사함.
- 이 설정으로 백로그 S1 을 닫았음(2026-10-15 세션60). 끝 기준을 15km/h 정지 거리 12.9m 안 보행자 10점 이상으로 바꿨음. 근거·한계는 perception/docs/gt_objects2.md '백로그 S1 닫음'.

## 좌표

- map 좌표 = CARLA 좌표 (x, -y) = xodr 평면 좌표 그대로임. xodr 머리말 offset 은 CARLA 가 읽지 않으므로 지도에도 적용하지 않음.
- 브리지를 0.53.0 이상으로 올리면 geoReference 로 원점을 옮기는 기능이 생기니 좌표부터 다시 확인함.
- spawn_point 인자는 CARLA 좌표임(x, y, z, roll, pitch, yaw 순서). 브리지가 z 에 2m 를 더해 떨어뜨림.

## 실행 순서

규칙: 서버 시험(CARLA·Autoware) 중에는 터미널 하나만 돌림. 2026-10-10 a3 는 맵 터미널과 같이 돌아 최소 avail 1,299MiB, 2026-10-11 p1 은 혼자 4,762MiB 였음.


1. CARLA 서버를 띄움(렌더링 끔, 절전 막음).
   `cd ~/carla/CARLA_0.9.15 && nohup systemd-inhibit --what=sleep:idle --who=i5 --why="CARLA I5" ./CarlaUE4.sh -RenderOffScreen -quality-level=Low > C/integration/logs/carla_i5_1.log 2>&1 &`
2. 자원 감시: `bash C/integration/docs/tools/i4_monitor.sh C/integration/logs/i5_monitor.log i1_aw &`
3. 캠퍼스 월드를 올림: `C/.venv-carla/bin/python C/map/scripts/load_campus_world.py` (기본 v3 동결본, 해시 확인)
   출력의 맵 이름이 `Carla/Maps/OpenDriveMap` 인지 봄.
4. 지도 폴더를 맞춤: `bash C/autoware_map/scripts/sync_map_folder.sh`
5. 한 회차 실행: `bash C/integration/docs/tools/i5_run.sh <tag> <쌍> 0.05`
   - 쌍 이름은 map/tests/test_drive.py ROUTES_PAIRS_S46 의 이름(예 p_south_yang)
   - 출발·목표는 i5_route.py 가 계산함(출발 승강장 차선 중심, 도착 승강장 차선 중심 = 뒷바퀴 축 목표)
   - 보행자 횡단: `PEDARGS="--road 1408 --s 60" bash i5_run.sh <tag> <쌍> 0.05 ped`
6. 요약·그림: `C/.venv-carla/bin/python C/integration/docs/tools/i5_summary.py <tag> <쌍> C/integration/docs/i5_traj.png`
7. 끝나면 CARLA, 감시를 내리고 프로세스 0 을 확인함(컨테이너는 i5_run.sh 가 내림).

i5_run.sh 가 하는 일
- 남은 차량·센서 정리, world.id 기록과 표지 콘 1개(r1366, 노선 밖)
- Autoware 기동(i5_aw_up.sh): 카메라 뺀 센서 파일, `localization:=false` + 정답 측위,
  `use_pointcloud_map:=false` + 진단 그래프 점군 검사 제외, 정답 객체(`use_empty_dynamic_object_publisher:=true`)
- 정답 측위·정답 객체 노드 실행, world.id·표지 확인, empty_objects_publisher 끄기
- CARLA 쪽 궤적 기록(i5_track.py), 주행(i5_drive.sh: 레코더, 경로, 자율 전환, 도착 감시)
- 브리지에 SIGINT(자차·센서를 브리지가 지움, 최대 15s) -> 컨테이너 내림 -> i5_cleanup.py(센서 -> 동기 끄기 -> 남은 액터)
- 경로 생성만 30쌍: `bash C/integration/docs/tools/i5_route30.sh <tag>`(자차를 출발 승강장으로 옮기고 목표, logs/i5_route30_<tag>.csv)

## 지도를 다시 만들 때

`C/autoware_map/docs/l1_report.md` 를 봄. 순서만 적음.
1. `.venv-carla/bin/python autoware_map/scripts/l1_build_lanelet2.py map/maps/cbnu_campus_frozen_v3.xodr autoware_map/build_v3c --chord-fix autoware_map/logs/l1_chord_list.json`
   나누기(뒤집히는 lanelet)와 이음매 보정 15m 가 기본으로 켜짐. --seam-taper 0 이면 v3b, 거기에 --no-split 이면 v3a 와 같은 바이트
2. 실제 로더로 평가(1.9.0 이미지 안 lanelet2): `docker run --rm -v C/autoware_map:/w <이미지> bash -c 'source /opt/ros/humble/setup.bash; cd /w; python3 scripts/l1_eval.py build_v3c build_v3c/pairs30.json'`
3. build 의 lanelet2_map.osm 을 cbnu_campus_v3/ 로 복사하고 sync_map_folder.sh

지도 판(2026-10-11): v3a 45ff2015795f22d2(I5 a1~b2·p1), v3b d7a7122605265142(m1), v3c f06582acf3f3b7be(m2, 지금 폴더)

## 장식 레이어 켜고 돌리기 (2026-10-11 S4)

레이어 = 나(정육면체 상자 건물 721) + 정류장 시설 10 + 지면 판 Plane 1 = 732 액터. map/scripts/place_campus_layers.py 는 실행만 함.

순서(실행 순서 3·4 다음, 5 대신)
1. 레이어 소환(브리지 기동 전, 비동기 상태에서):
   `C/.venv-carla/bin/python C/map/scripts/place_campus_layers.py --layers buildings,busstop,ground --repr 나 --ground-mesh Plane --mesh-sizes C/map/docs/logs/s51_bench/mesh_sizes.json --state C/integration/logs/s4_layers_state.json`
   - 상태 파일은 integration/logs 에 둠(맵 폴더에 안 씀). 이미 있으면 멈추므로 먼저 --remove
   - 서버 맵 sha 가 v3 동결본과 다르면 소환 안 하고 멈춤
2. 한 회차: `LSTATE=C/integration/logs/s4_layers_state.json bash C/integration/docs/tools/s4_run.sh <tag> <쌍> 0.05`
   - i5_run.sh 와 같고, 추적이 s4_track.py(네 바퀴 이탈)로 바뀌고, 표지 콘 옆에서 레이어 수를 기록(mark)·확인(check)함
   - 확인 = world.id 같음 + 레이어별 살아 있는 수 같음. logs/i5_run_<tag>.log 의 "check ... 통과 True"
   - i5_cleanup.py 는 차량·보행자·센서만 지움. 레이어(static.prop)는 회차 사이에 남음
3. 요약: `C/.venv-carla/bin/python C/integration/docs/tools/s4_summary.py <tag> C/integration/logs/s4_monitor.log`
4. 끝: `place_campus_layers.py --remove --state C/integration/logs/s4_layers_state.json`, 그다음 CARLA 를 내림

이탈 판정
- 지면 판이 차를 받쳐서 차도를 벗어나도 떨어지지 않음(세션51). 낙하로는 이탈을 못 봄
- s4_track.py: 매 틱 바퀴 4개 위치(처음 한 번 physics_control 의 바퀴 위치를 차 좌표로 바꿔 둠)에서
  map.get_waypoint(위치, project_to_road=False, lane_type=Driving|Sidewalk) 가 None 이면 그 바퀴는 도로 밖
- 열: wheels_off(도로 밖 바퀴 수), wheels_off_ids. 한 바퀴라도 밖인 틱 수 = 이탈 틱. offroad 열(차 중심, 차도만)은 i5 와 같음
- 레이어 끈 옛 회차(I5)는 s4_summary.py --wheels '<s4_track 로그의 wheels_local>' 로 오프라인 재계산

녹화 규칙(S4 회차부터. 그 전 녹화는 건드리지 않음)
- 레이어 나를 켜면 녹화가 분당 24.7MB(세션51)라 녹화는 켜되 오래 두지 않음
- 성공 회차: s4_summary.py 로 지표를 뽑은 뒤 recordings/i5_<tag>.log 를 지움
- 실패 회차: 녹화를 남김(원인 재생용)
- 영상용 회차: mp4 를 만든 뒤 .log 를 지움
- 지우기 전에 이름·크기·sha256 앞 16자를 integration/logs/recordings_deleted.txt 에 한 줄씩 남김

영상(레이어 켠 회차, S4)
- 레이어 732개가 든 녹화는 레코더 재생에서 자차가 생기지 않았음(S4). 그래서 궤적 기록으로 만듦
  `s4_video_track.py logs/i5_track_<tag>.csv <frames>` -> `s4_label.py <frames> <labeled> 0.1 "<노선>  레이어 켬"`
  -> `docker run --rm --network none --entrypoint python3 -u 1000:1000 -v <dir>:/f -v C/integration:/i <1.9.0 이미지> /i/docs/tools/i2_encode.py /f/<labeled> /i/recordings/<tag>_full.mp4 10`
- 레이어는 미리 소환해 둠. 재생을 시험했다면 남은 소품을 먼저 지움(재생 중지로 안 지워짐)

## v3.1 후보 띄우기 (2026-10-11 I6-1)

xodr map/maps/cbnu_campus_v31_candidate_s53.xodr(b9cc23692441de3a). 동결 전 후보라 위 절차의 기본(v3)과 다른 곳만 적음.

0. 서버 시험 전: 충전기가 실제로 충전 중인지 봄(`cat /sys/class/power_supply/BAT0/status` 가 Charging 또는 Full).
   배터리로 돌면 GNOME 이 20분 유휴 뒤 절전함(sleep-inactive-battery-timeout 1200). systemd-inhibit sleep:idle 로는 안 막혔음(I6 i6r1).
   세션 동안 `nohup gnome-session-inhibit --inhibit suspend:idle --reason "I6" sleep 21600 &` 를 같이 띄우고, 끝에 그 PID 를 kill 함
1. 지도 폴더: `l1_build_lanelet2.py map/maps/cbnu_campus_v31_candidate_s53.xodr integration/maps/build_v31 --chord-fix autoware_map/logs/l1_chord_list.json`
   (b9b3535f80232d9d) 의 lanelet2_map.osm·map_projector_info.yaml 을 ~/autoware_data/maps/OpenDriveMap/ 로 복사. v3c 백업은 integration/maps/OpenDriveMap_v3c_bak_i6/
   - r1317 을 현 목록에서 뺀 판: --chord-fix integration/maps/i6_chord_list_no1317.json -> build_v31_no1317(4ea59920). 주행 미확인
2. 월드: `C/.venv-carla/bin/python C/map/scripts/load_campus_world.py --xodr C/map/maps/cbnu_campus_v31_candidate_s53.xodr` (서버 sha 가 파일과 같은지 봄)
3. 레이어: S4 명령에 `--xodr C/map/maps/cbnu_campus_v31_candidate_s53.xodr --state C/integration/logs/i6_layers_state.json` (없으면 서버 sha 불일치로 멈춤)
4. 한 회차: `LSTATE=C/integration/logs/i6_layers_state.json bash C/integration/docs/tools/i6_run.sh <tag> <쌍> 0.05`
   - s4_run.sh 와 같고 다른 점: 출발·목표를 v3.1 xodr·map/build_v31_check/pairs30.json 으로(i6_route.py), CARLA 녹화 없음(i6_drive.sh),
     정지 감시 i6_stall.sh(자율 전환 뒤 90s 동안 0.05m/s 밑이고 목표 3m 밖이면 i6_diag.py 진단 -> logs/i6_stop_diag_<tag>.txt, 감시 종료)
5. 요약: `s4_summary.py <tag> C/integration/logs/i6_monitor.log` (파일 이름은 i5_* 그대로)
6. 영상: S4 와 같이 s4_video_track.py -> s4_label.py "<노선>  v3.1 후보" -> i2_encode.py

## v3.1 후보 s54 띄우기 (2026-10-12 I6-2)

xodr map/maps/cbnu_campus_v31_candidate_s54.xodr(214507afc87c2523), Lanelet2 83edb7755588184f. 동결 전이라 로더·레이어에 --xodr 로 줌(로더·레이어 스크립트 안 고침).
1. 지도 폴더(한 번만): 현 보정 목록에서 1317·1825 를 뺀 목록으로 만듦
   `.venv-carla/bin/python autoware_map/scripts/l1_build_lanelet2.py map/maps/cbnu_campus_v31_candidate_s54.xodr integration/maps/build_v31_s54 --chord-fix integration/maps/i62_chord_list_no1317_1825.json`
   출력 sha 83edb7755588184f 확인 -> 지금 폴더를 백업한 뒤 lanelet2_map.osm·map_projector_info.yaml 을 ~/autoware_data/maps/OpenDriveMap/ 로 복사
2. 충전 중인지(upower BAT0 state, /sys/class/power_supply/BAT0/status), 전원 프로필 performance 확인. 절전 막기:
   `nohup gnome-session-inhibit --inhibit suspend:idle --reason "I6-2" sleep infinity > C/integration/logs/i62_inhibit.log 2>&1 &`
   (systemd-inhibit sleep:idle 만으로는 GNOME 배터리 유휴 절전을 못 막는다, I6-1)
3. CARLA: `cd ~/carla/CARLA_0.9.15 && nohup systemd-inhibit --what=sleep:idle --who=i62 --why="CARLA I6-2" ./CarlaUE4.sh -RenderOffScreen -quality-level=Low > ~/campus_mobility_sim/integration/logs/carla_i62_1.log 2>&1 &`
4. 월드: `C/.venv-carla/bin/python C/map/scripts/load_campus_world.py --xodr C/map/maps/cbnu_campus_v31_candidate_s54.xodr` -> "서버 to_opendrive sha256=214507afc87c2523 (파일과 일치)"
5. 레이어: `C/.venv-carla/bin/python C/map/scripts/place_campus_layers.py --layers buildings,busstop,ground --repr 나 --ground-mesh Plane --mesh-sizes C/map/docs/logs/s51_bench/mesh_sizes.json --xodr C/map/maps/cbnu_campus_v31_candidate_s54.xodr --state C/integration/logs/i62_layers_state.json` -> 732
6. 한 회차: `LSTATE=C/integration/logs/i62_layers_state.json bash C/integration/docs/tools/i62_run.sh <tag> <쌍> 0.05`, 요약 `i62_summary.py <tag> <monitor.log>`
   30쌍 묶음: `bash C/integration/docs/tools/i62_all.sh [첫 번호]` 가 3~5 와 회차·요약을 다 함(감시 i4_monitor.sh 는 따로 띄움)
