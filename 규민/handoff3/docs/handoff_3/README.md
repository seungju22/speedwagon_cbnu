# 인계 3: 캠퍼스 지도·인지와 실행 도구

v3.1 동결 지도에서 셔틀 한 구간을 Autoware 1.9.0 으로 자율주행시키는 순서와, 인계 2 뒤에 더한 것(v3.2 지도, 장식 레이어 v2, 대체 정차점, LiDAR 클러스터링, 비 조건, 지도 재현)을 적은 것임.
측위는 CARLA 정답 값을 씀. 객체 인식은 정답 객체(다 보기·차 시점)와 LiDAR 클러스터링 가운데 고름.
미션 노드(정류장 사이클, 요청 정차, 출발 조건)와 셔틀 기능 실험은 이 묶음에 없음.
11/04 판: 순환 노선용 지도 v3.3d 와 스프린터 차량 설정을 더함(16절). 새 실행 스크립트 s77_run_loop.sh 는 스프린터가 기본, VEHICLE=prius 로 되돌림. 판단·제어 쪽이 볼 것은 17절.

인계 2 대비 바뀐 것은 `changes_since_handoff2.md`, 지도 재현은 `map_reproduce.md`, 라이선스는 `NOTICE.md`.

## 1. 폴더 자리

handoff3/ 안은 작업 폴더 구조 그대로임. `~/campus_mobility_sim` 으로 복사함. 아래에서는 이 경로를 `C` 로 씀.
- 스크립트가 `~/campus_mobility_sim/...` 경로를 바로 씀
- compose 오버레이가 `${HOME}/campus_mobility_sim/...` 를 컨테이너에 마운트함(정답 측위·정답 객체 노드, 점 지움 노드, 로그 폴더, `.venv-carla` 안 carla 패키지)
- 다른 자리에 두면 컨테이너가 빈 폴더를 마운트하고 브리지가 carla 를 못 찾음

## 2. 설치 순서

1. Ubuntu 22.04, Docker(sudo 없이 docker 를 쓸 수 있게 docker 그룹에 넣음). RAM 16GB 에서 돌렸음. GPU 는 없어도 됨
2. CARLA 0.9.15 패키지를 `~/carla/CARLA_0.9.15` 에 풂. https://github.com/carla-simulator/carla/releases/tag/0.9.15
3. Autoware 저장소를 `~/autoware` 에 두고 1.9.0 으로 맞춤. compose 파일만 씀

       git clone https://github.com/autowarefoundation/autoware.git ~/autoware
       git -C ~/autoware checkout 1.9.0

4. 이미지(CUDA 없는 것, 약 10GB)

       docker pull ghcr.io/autowarefoundation/autoware:universe-humble-1.9.0

5. 파이썬 가상환경은 꼭 `C/.venv-carla` 에 만듦(위 마운트 때문)

       python3 -m venv ~/campus_mobility_sim/.venv-carla
       ~/campus_mobility_sim/.venv-carla/bin/pip install carla==0.9.15 numpy shapely matplotlib

## 3. 한 번만 하는 준비

    cp -r handoff3/. ~/campus_mobility_sim/
    cd ~/campus_mobility_sim
    sha256sum -c map/maps/frozen_v31.sha256          # 10개 OK

    # Autoware 지도 폴더. 이름은 꼭 OpenDriveMap(브리지가 이 이름으로 CARLA 맵을 고름)
    mkdir -p ~/autoware_data/maps/OpenDriveMap ~/autoware_data/maps/i1_config ~/autoware_data/ml_models
    cp autoware_map/cbnu_campus_v31/* ~/autoware_data/maps/OpenDriveMap/
    cp integration/config/sensor_mapping_nocam_pps545k.yaml integration/config/sensor_mapping_nocam.yaml \
       integration/config/sensor_mapping_light32.yaml ~/autoware_data/maps/i1_config/

    # 로그 폴더. 없으면 docker 가 root 소유로 만듦
    mkdir -p integration/logs/s59 integration/logs/s67 perception/logs

## 4. DDS 설정

- 주행용 compose(`docker/planning-simulator.pin.yaml`)가 `CYCLONEDDS_URI` 로 MaxAutoParticipantIndex 1000 을 넘김. 따로 할 것 없음
- 네트워크 없는 컨테이너(`docker run --network none`, bag 오프라인 처리)에서 ROS 노드가 'failed to create domain' 으로 안 뜨면, 이미지 기본 cyclonedds.xml 의 수신 버퍼 10MB 요구가 호스트 net.core.rmem_max 보다 커서 그런 것임. lo 만 쓰는 설정을 넘김

      -e CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General></Domain></CycloneDDS>'

## 5. 메모리

- CARLA 를 띄우기 전에 가용 메모리가 10GiB 이상인지 봄(`free -g` 의 available). 밑이면 브라우저 같은 큰 프로그램을 끄거나 재부팅함
- 같이 띄우면 주행 중 최소 가용이 1.6~2.5GiB 까지 내려감(16GB 노트북). 1GiB 밑이 30초 넘게 가거나 swap 7GiB 를 넘으면 Autoware 컨테이너 -> CARLA 순서로 내림
- 오래 돌릴 때는 절전을 막음. 충전기를 꽂고 `C/integration/docs/tools/s71_keepawake.sh` 를 씀(GNOME 배터리 유휴 절전은 systemd-inhibit 만으로 안 막힘)

      nohup bash ~/campus_mobility_sim/integration/docs/tools/s71_keepawake.sh ~/campus_mobility_sim/integration/logs/keepawake.log > /dev/null 2>&1 &
      cat ~/campus_mobility_sim/integration/logs/keepawake.log.pids     # 끌 때 SELF= 의 PID 에 kill

## 6. 달리기 (한 구간)

터미널 하나로 함.

1. CARLA 를 띄움(화면 없이). 두 줄로 나누고 실제 서버 PID 를 pgrep 으로 봄(`cd ... && nohup ... &` 한 줄이면 `$!` 가 셸 PID 라 안 맞음)

       cd ~/carla/CARLA_0.9.15
       nohup ./CarlaUE4.sh -RenderOffScreen -quality-level=Low > ~/campus_mobility_sim/integration/logs/carla.log 2>&1 &
       sleep 5; pgrep -f CarlaUE4-Linux-Shipping

   줄이 둘 나오면 몇 초 뒤 다시 봄. 띄우는 동안 잠깐 생기는 프로세스가 같이 잡힐 수 있음(10/26 에 한 번 봄)

2. 30초쯤 뒤 캠퍼스 월드를 올림. '파일과 일치' 와 214507af 가 나와야 함

       ~/campus_mobility_sim/.venv-carla/bin/python ~/campus_mobility_sim/map/scripts/load_campus_world.py

3. (선택) 장식 레이어를 놓음. 7절
4. 한 회차. p_middle_north(중문 -> 도서관 북문, 약 270m)를 차 시점 정답 객체로, 장면 없이

       bash ~/campus_mobility_sim/integration/docs/tools/s59_run.sh try1 p_middle_north car none

   Autoware 컨테이너(i1_aw)를 띄우고 정답 측위·객체 노드를 켠 뒤 경로를 넣고 자율 모드로 바꿈. 북문 승강장에 서면 끝나고 컨테이너도 스스로 내림. 3~5분 걸림
5. 결과
   - `C/integration/logs/i5_run_try1.log`: 고른 설정 한 줄(smap, mode, rain, 레이어 판)
   - `C/integration/logs/i5_drive_try1.log`: route success, ROUTE_SAME, engage success
   - `C/integration/logs/i5_watch_try1.csv` 마지막 줄 final: 정지 위치와 앞뒤·옆 오차(앞뒤 -0.44m 쯤이면 정상)
   - `C/integration/logs/i5_track_try1.csv`: 매 틱 위치·속도·road·충돌·바퀴 이탈
6. 끝나면 CARLA 를 1에서 본 PID 로 끔(`kill -INT <PID>`). `pgrep -f CarlaUE4` 와 `docker ps` 가 비었는지 봄

같은 인자로 `s77_run_loop.sh` 를 부르면 스프린터로 달림(16절). 위 s59_run.sh 는 전처럼 프리우스.

s59_run.sh 인자: `<tag> <쌍> <mode> <장면> [cam 0|1] [bag 0|1]`
- 쌍: map/build_v31_s54/pairs30.json 의 30쌍 이름(p_main_middle, p_middle_north, p_north_yang, p_yang_back 등)
- 목표: 기본은 도서관 북문 뒷축 목표(노선 파일). 다른 쌍을 그 쌍 도착 승강장까지 달리려면 `PAIR_GOAL=1` 을 붙임
- 장면: none, ped(돌발 보행자), door(북문 승강하차), cone(차로 콘), pcar(차로에 걸친 차), obst(콘+차). 장면은 북문 쪽 r1415 에 놓이므로 p_middle_north 와 같이 씀. 자세한 것은 `C/integration/scenario/README.md`
- cam 1 이면 추적 카메라 프레임을 `integration/logs/s59/cam_<tag>` 에 남김

## 7. 고를 것

LiDAR(시뮬레이터 LiDAR, 앞에 SMAP= 를 붙임)
- 기본: `sensor_mapping_nocam_pps545k.yaml`. 64채널, 초당 54.5만 점, 100m. 브리지가 무작위로 버리는 점(CARLA 기본 0.45)만큼 늘려 기준 센서와 점 수를 맞춘 것. 세 인지 모드 모두 이것이 기본
- 30만: `SMAP=sensor_mapping_nocam.yaml`(인계 2 기본). 가벼운: `SMAP=sensor_mapping_light32.yaml`(32채널, 10만, 50m)
- 54.5만은 30만보다 브리지 CPU 가 약 13%p 더 들고 최소 가용 메모리는 거의 같음(-17MiB). 실시간 0.998배 같음

인지 모드(셋째 인자)
- car(차 시점 정답): 지붕 LiDAR 자리에서 보이는 것만 냄. 보행자 17.5m, 차 35m, 소품 12.5m, 가려지면 안 냄
- all(다 보기 정답): 80m 안을 거리·가려짐 없이 다 냄. 인지 탓인지 계획 탓인지 가를 때 비교용
- clustering: Autoware LiDAR 클러스터링(지면 분할 + 유클리드 클러스터 + 추적·예측)이 객체 토픽을 냄. 정답은 비교용 /perception/gt/objects 로만 나감
  - 정류장 시설 점·건물 바닥 사각형 점을 클러스터 앞에서 지움(기본 켬, `FACCROP=0`·`BLDCROP=0` 이면 끔), 클러스터 최소 점 5(`CLMIN=10` 이면 Autoware 기본)
  - 객체 bag 이 `integration/logs/s67/objbag_<tag>` 에 자동으로 남음(p_middle_north 장면 없음 약 50MB)
  - 근거·한계: `C/perception/docs/interface.md`, 거리별 검출률은 발표 재료 tab11·tab12·tab14
- 정답 객체 근거와 인자: `C/perception/docs/gt_objects2.md`

비 조건(앞에 RAIN= 를 붙임, 기본 맑음)
- `RAIN=1` 약한 비 9mm/h, `RAIN=2` 강한 비 25mm/h
- car 는 정답 범위를 모델 최대 거리로 줄임(비1 차 21.8m, 비2 보행자·차 16.2m). clustering 은 점 지움 노드가 점군을 열화함(거리에 따라 점 버림, 거리 잡음 2%, 시드 RAIN_SEED 기본 0). all 은 그대로
- CARLA 날씨는 안 바꿈(0.9.15 LiDAR 에 날씨 연결이 없음)
- 모델은 Goodin 2019, 표적 반사율 약 22% 가정(기본값). 어두운 옷(7%)은 최악 조건으로 문서에만 둠. 7%·강한 비는 모델 최대 거리 12.1m 로 15km/h 정지 거리 12.9m 보다 짧음. 근거와 민감도 `C/perception/docs/rain_model.md`

      RAIN=2 bash ~/campus_mobility_sim/integration/docs/tools/s59_run.sh rain2 p_middle_north clustering door

장식 레이어(건물·정류장 시설·지면·나무, 선택)
- v2(기본) = 건물 1순위 721 상자 + 정류장 시설 10 + 지면 판 1 + 건물 2순위 164 + 나무 82 = 978 액터. v1 = 732(나무·2순위 없음)
- 월드를 올린 뒤, 회차 전에 한 번 놓고 상태 파일을 LSTATE= 로 넘김

      ~/campus_mobility_sim/.venv-carla/bin/python ~/campus_mobility_sim/map/scripts/place_campus_layers_s66.py --state ~/campus_mobility_sim/integration/logs/try_layers_state.json
      LSTATE=~/campus_mobility_sim/integration/logs/try_layers_state.json bash ~/campus_mobility_sim/integration/docs/tools/s59_run.sh try2 p_middle_north car none

- v1 로 놓으려면 `--set v1`. 지울 때 `--remove --state <같은 파일>`. 상태 파일이 이미 있으면 멈추므로 먼저 지움
- 로그 i5_run_<tag>.log 에 'layer_set v2 978' 이 남음. 레이어는 회차 사이에 남음(CARLA 를 끄거나 --remove 할 때까지)
- clustering 의 건물 점 지움 사각형(`perception/config/s69_building_rects.json`)은 v1·v2 건물 자리임. 레이어를 안 놓아도 그대로 둬도 됨

## 8. 여러 구간 이어 달리기

최종 확장 노선은 정문(이쪽) -> 중문 -> 도서관 북문 -> 양성재(이쪽) -> 후문(이쪽)임(3,314m). CARLA 회차는 한 구간마다 컨테이너를 새로 띄움.
구간 출발점은 쌍의 출발 승강장이라, 앞 구간이 선 자리와 다음 구간 출발이 수 m 다를 수 있음.

    T=~/campus_mobility_sim/integration/docs/tools
    PAIR_GOAL=1 bash $T/s59_run.sh leg1 p_main_middle car none
    PAIR_GOAL=1 bash $T/s59_run.sh leg2 p_middle_north car none
    PAIR_GOAL=1 bash $T/s59_run.sh leg3 p_north_yang car none
    PAIR_GOAL=1 bash $T/s59_run.sh leg4 p_yang_back car none

- 한 CARLA 안에서 차례로 돌림. 회차마다 i5_cleanup.py 가 남은 차·센서를 지움. 레이어는 남음
- 한 컨테이너 안에서 차를 세우지 않고 이어 가는 것은 플래닝 시뮬레이터 쪽 순서를 봄(`C/docs/handoff_2/planning_sim_dev.md`): 도착(routing state 3) -> `/api/routing/clear_route` -> 새 목표 -> 자율 전환

## 9. 지도 판: v3.1 과 v3.2

- 팀 기본은 v3.1(Lanelet2 28a57711). 도로(xodr 214507af)와 CARLA 월드는 v3.1·v3.2 가 같음
- v3.2 = v3.1 + 횡단보도·과속방지턱. 판은 v32a(42곳, OSM 그대로)로 정했음. v32b 는 X30·X44 두 곳을 뺀 대안
- 바꾸기(CARLA·Autoware 를 끈 뒤)

      cp -a ~/autoware_data/maps/OpenDriveMap ~/autoware_data/maps/OpenDriveMap_v31_bak
      cp ~/campus_mobility_sim/map/maps/v32a/lanelet2_map.osm ~/autoware_data/maps/OpenDriveMap/lanelet2_map.osm
      sha256sum ~/autoware_data/maps/OpenDriveMap/lanelet2_map.osm   # 4719e5e5a4f4588bc0e8a48c53d9bc71beaade7c446389e0f5a4ee3c47cfc789

- 방지턱 옵션: 1.9.0 런치는 방지턱 모듈을 이름 불일치로 못 켬. `SPEED_BUMP=1` 을 붙이면 뜬 뒤 load_plugin 으로 켬. 로그에 'speed_bump on params=7'. 횡단보도 모듈은 원래 켜져 있음

      SPEED_BUMP=1 bash ~/campus_mobility_sim/integration/docs/tools/s59_run.sh v32try p_middle_north car none

- 방지턱 위 약 9km/h, 구간 시간 +4.6~9.5%. CARLA 도로에는 방지턱 모양이 없음. 자세한 것 `C/map/docs/v32_switch_howto.md`, `C/map/docs/v32_candidate.md`

## 10. 정류장 대체 정차점

- 승강장 2판 `map/data/shuttle_platforms_v31_r2.yaml`: 1판 그대로 + 승강장마다 alt_stop(같은 차로에서 들어오는 쪽으로 6.89~11.39m 뒤, 정차 구역·문 구역·뒷축 목표). 도서관 남문(이쪽)은 원래 정차 불가라 없음
- 노선 3판 `map/data/routes/pilot_route_r3.yaml`: 2판 그대로 + 정류장마다 alt_goal
- 쓰임: 원래 정차 자리가 막혔을 때 그 앞에 서는 자리. 들어오는 길에서 바로 대체 자리에 서는 경로는 26쌍 모두 원래 경로 앞부분과 같음
- 정문·후문은 원래 자리에 선 뒤 대체 자리로 돌아올 길이 지도 안에 없음(캠퍼스 밖으로 나가는 길)
- 이 묶음의 s59_run.sh 는 목표가 북문 원래 자리 또는 쌍 목표로 정해져 있음. 대체 자리로 달려 보려면 플래닝 시뮬레이터 스크립트에 alt_goal 값을 그대로 줌(예: 북문)

      bash ~/campus_mobility_sim/integration/docs/tools/s62_ps_goal.sh alt1 646.872 691.028 -0.302194 0.953246 619.358 907.515 -0.684987 0.728555

- 그림 `map/docs/figs/s70_alt_stops.png`, 설명 `map/docs/shuttle_route_file.md` 대체 정차점 절
- CARLA 로 대체 자리까지 달린 기록은 아직 없음

## 11. 되돌리는 법

- 지도: `cp ~/autoware_data/maps/OpenDriveMap_v31_bak/lanelet2_map.osm ~/autoware_data/maps/OpenDriveMap/` 뒤 sha256 28a57711 확인
- LiDAR 를 인계 2 처럼: `SMAP=sensor_mapping_nocam.yaml`
- 클러스터링을 10/23 처럼: `SMAP=sensor_mapping_nocam.yaml BLDCROP=0 CLMIN=10`. 시설 점 지움까지 끄려면 `FACCROP=0`
- 비: RAIN 을 안 붙이면 맑음
- 레이어: `place_campus_layers_s66.py --remove --state <상태 파일>`, 또는 CARLA 를 다시 띄움
- 방지턱 모듈: SPEED_BUMP 를 빼면 꺼진 채로 뜸
- 스크립트는 인자를 안 주면 위 기본(54.5만, 차 시점, 맑음, 북문 목표, 방지턱 꺼짐)으로 돎

## 12. CARLA 없이 개발할 때

플래닝 시뮬레이터 순서와 스크립트는 `C/docs/handoff_2/planning_sim_dev.md`(이어 달리기, 더미 보행자, 가상 문). 같은 지도 폴더를 씀.

## 13. 자주 막히는 곳

- 브리지가 다른 맵을 불러와 캠퍼스가 사라짐: 지도 폴더 이름이 OpenDriveMap 이 아님
- 경로가 안 들어감(route success 없음): `~/autoware_data/maps/OpenDriveMap/lanelet2_map.osm` 해시가 28a57711(v3.1) 또는 4719e5e5(v32a)인지 봄
- 브리지가 carla 를 못 찾음: `.venv-carla` 가 `~/campus_mobility_sim/.venv-carla` 에 없음
- 로그 폴더가 root 소유가 됨: 3절 mkdir 를 안 하고 먼저 돌렸음. `sudo chown -R $USER` 로 되돌리고 mkdir 부터 함
- 'failed to create domain': 4절
- 월드 해시가 안 맞음: CARLA 가 덜 떴을 때 불렀음. 30초 더 기다려 다시 부름
- clustering 로그에 'faccrop on' 이 없음: 컨테이너가 뜨기 전에 점 지움이 실패한 것. `integration/logs/s67/fac_swap_<tag>.log` 를 봄
- 배터리로 돌다 20분 뒤 절전됨: 충전기와 5절 keepawake
- 끌 때 `pkill -f` 를 쓰면 자기 셸까지 끊길 수 있음. 띄울 때 본 PID 로 끔
- 가용 메모리가 모자람: 5절. SMAP=sensor_mapping_light32.yaml 로 가볍게 하거나 레이어를 v1 로

## 14. 지도를 다시 만들 때

- Lanelet2 만: `bash C/map/scripts/s57_make_v31_lanelet2.sh <새 폴더>`(동결 xodr -> 83edb775 -> 28a57711)
- OSM 부터 전부: `bash C/map/scripts/s71_reproduce.sh <빈 임시 HOME>`. 단계·해시·손으로 정한 것은 `map_reproduce.md`

## 15. 넣지 않은 것

CARLA 에셋, Autoware 이미지, 가상환경, 주행 bag·프레임·영상, 작업 기록과 점검 페이지, 판단·제어 코드와 셔틀 기능 실험. 작업 폴더 백업에만 있음.

스크립트·문서는 MIT(맨 위 `LICENSE`). 지도 데이터는 © OpenStreetMap contributors, ODbL 1.0. Autoware 설정을 고친 파일(yaml·지도 csv)은 Apache 2.0. 파일별로는 `docs/handoff_3/NOTICE.md` 를 봄.

## 16. 순환 노선과 차량 (지도 v3.3d)

캠퍼스 안을 도는 순환 노선은 v3.3 지도에서만 됨(개성재 승강장, 유턴 두 곳, N14 통로). 지도 설명은 `C/map/docs/v33d_readme.md`.
팀 기본 지도(v3.1, ~/autoware_data/maps/OpenDriveMap)는 그대로 두고, v3.3d 사본을 다른 경로로 마운트해서 씀.

    cd ~/campus_mobility_sim
    mkdir -p integration/maps/s77_v33d/OpenDriveMap
    cp map/maps/v33d/lanelet2_map.osm map/maps/v33d/map_projector_info.yaml integration/maps/s77_v33d/OpenDriveMap/
    cp integration/config/s74_sensor_mapping_van_pick.yaml ~/autoware_data/maps/i1_config/

- 한 바퀴 약 3,406m, 6곳: 정문 안쪽 > 양성재(이쪽) > 후문 안쪽 > (후문 앞 섬 유턴) > 도서관 남문(건너편) > 도서관 북문 > (모서리 유턴) > 개성재 > (N14 통로) > 정문 안쪽
- 노선 파일: 프리우스 `map/data/routes/circulation_v33d_r1.yaml`, 스프린터 `circulation_v33d_r1_sprinter.yaml`. 정류장마다 via(요청 없으면 지나감)와 goal(요청 있으면 섬)이 있음. 운행 논리는 판단 쪽 몫이라 여기 없음
- 구간을 이어 달리는 스크립트는 `s77_run_loop.sh`. 회차 목록(LOOP_SEGS) 한 줄 = 구간 이름, 목표 x y qz qw, 안 쓰는 칸 다섯 개(-), 예상 초. 예상의 2배를 넘기거나 바퀴 이탈이 10초 이어지면 끊음. 예시 목록 `C/integration/config/s77_segments_loop_sprinter.txt`

      C=~/campus_mobility_sim
      LOOP_SEGS=$C/integration/config/s77_segments_loop_sprinter.txt \
      MAP_COMPOSE=$C/integration/config/s77_v33d_map.override.yaml MAP_PATH=/home/aw/s77_maps/OpenDriveMap \
      SPAWN_OVR=379.399,-885.908,0.30,0.0,0.0,-144.18 \
      bash $C/integration/docs/tools/s77_run_loop.sh loop1 p_middle_north car none

차량
- 기본 차는 스프린터(11/04 순환 6/6 뒤 바꿈). s77_run_loop.sh 에서만 해당하고, s59_run.sh·s75_run_loop.sh 는 전처럼 프리우스
- `VEHICLE=prius` 를 붙이면 프리우스(sample_vehicle 설정, 인계 2 와 같음), `VEHICLE=sprinter` 면 스프린터
- 스프린터로 고르면 s77_run_loop.sh 가 아래를 채움(따로 주면 그 값이 먼저)
  - VEHICLE_TYPE=vehicle.mercedes.sprinter, VEH_COMPOSE=integration/config/s76_sprinter.override.yaml(vehicle_info, 지붕 LiDAR, 가속·제동 지도, max_throttle 0.6, 미러 자르기)
  - SMAP=s74_sensor_mapping_van_pick.yaml, 정답 객체 노드 `--lidar=-0.229,0.0,2.130`
  - 한 목표 회차의 목표는 스프린터 문 기준 북문 목표, 문 구역 장면은 스프린터 승강장 파일
- 스프린터 문은 오른쪽 슬라이딩 문. 문 가운데가 뒷축 앞 1.52m 라 정차 목표가 프리우스(1.345m)보다 약 0.17m 뒤임(승강장 파일 `_sprinter` 판)
- 스프린터 가속·제동 지도는 CARLA 에서 계단 시험으로 잰 것(`integration/docs/tools/s76_pedal_bench.py`, `s76_make_maps.py`). 브리지 기본 지도는 프리우스용이라 3톤 차는 출발을 못 함
- `PAIR_GOAL=1` 의 쌍 목표는 프리우스 문 기준 그대로임

## 17. 알려진 점

판단·제어 쪽에서 볼 것. 잰 값과 자리만 적음.

- 조향 되돌림 값: 브리지가 Autoware 에 돌려주는 조향(/vehicle/status/steering_status)은 앞 왼바퀴 각임. 왼 회전에서 실제 등가 조향보다 약 16% 큼(스프린터 명령 0.7 에서 돌려준 값 0.832rad, 실제 등가 0.718rad).
- 스프린터 저속 정지: 정차 마지막 0.5초 감속이 -4~-5m/s2 임(브레이크 0.07~0.33). CARLA 브레이크가 저속에서 셈. 가속·제동 지도의 저속 칸은 정지 평균 감속으로 잡았음. 프리우스도 -3.9~-4.0m/s2
- 스프린터 인지: 정지 장면 거리별 검출(클러스터링, 54.5만)에서 보행자 90% 를 넘는 가장 먼 거리가 13m. 15km/h 정지 거리 12.9m 를 겨우 넘음. 프리우스는 15m. 차 25m, 콘 10m(프리우스 25·13m)
- 순환 노선의 유턴 두 곳: 도서관 길 모서리(개신문화관 정면, 반경 4.1m)와 후문 앞 섬 둘레(반경 6.9m). 둘 다 맞은편 차가 없을 때 도는 자리로 잡았음. 실제로 언제 돌지는 판단 쪽 몫
- 결과마다 잰 차가 다름
  - 30쌍 경로·차체 검사, I6 이어 달리기, 대체 정차점, 인지 비교(10/23~10/25), 정답 객체 기본 거리: 프리우스
  - 순환 노선 CARLA 주행(11/04), 스프린터 인지 거리별(11/04): 스프린터
  - 플래닝 시뮬레이터 순환: 두 차 다
