# 인계 2: 캠퍼스 한 구간을 Autoware 로 달려 보기

v3.1 동결 지도에서 셔틀 한 구간을 Autoware 1.9.0 으로 자율주행시키는 데까지의 순서임.
측위와 객체 인식은 CARLA 정답 값을 씀. LiDAR 클러스터링은 이 순서에 없음.

## 폴더 자리

handoff2/ 안은 작업 폴더 구조 그대로임. 반드시 `~/campus_mobility_sim` 으로 복사함. 아래에서는 이 경로를 `C` 로 씀.
- 스크립트가 `~/campus_mobility_sim/...` 경로를 바로 씀(경로 계산, 노선 파일, 로그)
- compose 오버레이가 `${HOME}/campus_mobility_sim/...` 를 컨테이너에 마운트함. 정답 측위 노드, 로그 폴더, 진단 설정,
  그리고 `.venv-carla` 안의 carla 파이썬 패키지(브리지가 컨테이너 안에서 이걸로 CARLA 에 붙음)
- 다른 자리에 두면 컨테이너가 빈 폴더를 마운트하고 브리지가 carla 를 못 찾음

## 필요한 것

- Ubuntu 22.04, Docker, RAM 16GB 이상. GPU 는 없어도 됨(내장 그래픽 노트북에서 돌렸음)
- CARLA 0.9.15 패키지를 `~/carla/CARLA_0.9.15` 에 풂
- Autoware 저장소 1.9.0 을 `~/autoware` 에 둠. compose 파일만 씀. `git -C ~/autoware checkout 1.9.0`
- 이미지 `ghcr.io/autowarefoundation/autoware:universe-humble-1.9.0`(CUDA 없는 것, 약 10GB)
- 파이썬 가상환경은 꼭 `C/.venv-carla` 에 만듦(위 마운트 때문)

      python3 -m venv ~/campus_mobility_sim/.venv-carla
      ~/campus_mobility_sim/.venv-carla/bin/pip install carla==0.9.15 numpy shapely matplotlib

## 한 번만 하는 준비

    cp -r handoff2/. ~/campus_mobility_sim/
    cd ~/campus_mobility_sim
    sha256sum -c map/maps/frozen_v31.sha256          # 10개 OK

    # Autoware 지도 폴더. 이름은 꼭 OpenDriveMap(브리지가 이 이름으로 CARLA 맵을 고른다)
    mkdir -p ~/autoware_data/maps/OpenDriveMap ~/autoware_data/maps/i1_config ~/autoware_data/ml_models
    cp autoware_map/cbnu_campus_v31/* ~/autoware_data/maps/OpenDriveMap/
    cp integration/config/sensor_mapping_nocam.yaml integration/config/sensor_mapping_light32.yaml ~/autoware_data/maps/i1_config/

    # 로그 폴더. 없으면 docker 가 root 소유로 만든다
    mkdir -p integration/logs/s59 perception/logs

## 달리기

터미널 하나로 함. CARLA 와 Autoware 를 같이 띄우면 여유 메모리가 1.7~2GB 까지 내려가니 브라우저 같은 큰 프로그램은 끔.
오래 돌릴 때는 화면 꺼짐·절전을 막음(예: `gnome-session-inhibit --inhibit suspend:idle sleep 7200 &`).

1. CARLA 를 띄움(화면 없이). 실제 서버 프로세스 PID 를 적어 둠.
   `cd ... && nohup ... &` 한 줄로 쓰면 `$!` 가 셸 PID 라 끌 때 안 맞음. 두 줄로 나누고 PID 는 pgrep 으로 봄.

       cd ~/carla/CARLA_0.9.15
       nohup ./CarlaUE4.sh -RenderOffScreen -quality-level=Low > ~/campus_mobility_sim/integration/logs/carla.log 2>&1 &
       sleep 5; pgrep -f CarlaUE4-Linux-Shipping

2. 30초쯤 뒤 캠퍼스 월드를 올림. 해시 214507af 가 '파일과 일치'로 나와야 함.

       ~/campus_mobility_sim/.venv-carla/bin/python ~/campus_mobility_sim/map/scripts/load_campus_world.py

3. 한 회차. p_middle_north(중문 -> 도서관 북문, 약 270m)를 차 시점으로, 장면 없이.

       bash ~/campus_mobility_sim/integration/docs/tools/s59_run.sh try1 p_middle_north car none

   Autoware 컨테이너(i1_aw)를 띄우고 정답 측위·정답 객체 노드를 켠 뒤 경로를 넣고 자율 모드로 바꿈.
   북문 승강장에 서면 끝나고 컨테이너도 스스로 내림. 3~5분 걸림.
4. 결과
   - `C/integration/logs/i5_drive_try1.log`: route success, ROUTE_SAME(Autoware 경로가 노선과 같음), engage success
   - `C/integration/logs/i5_watch_try1.csv` 마지막 줄 final: 정지 위치, 앞뒤·옆 오차(앞뒤 -0.44m 쯤이면 정상)
   - `C/integration/logs/i5_track_try1.csv`: 매 틱 위치·속도·road·충돌
5. CARLA 를 1에서 적은 PID 로 끔(`kill -INT <PID>`, 감싼 CarlaUE4.sh 도 같이 끝남). `pgrep -f CarlaUE4` 와 `docker ps` 가 비었는지 봄.

## 고를 것

정답 객체 모드(셋째 인자)
- car: 지붕 LiDAR 자리에서 보이는 것만 냄. 보행자 17.5m, 차 35m, 소품 12.5m, 가려지면 안 냄. 발표 장면은 이걸로 돌렸음
- all: 80m 안을 거리·가려짐 없이 다 냄. 인지 탓인지 계획 탓인지 가를 때 비교용
- 근거와 인자: `C/perception/docs/gt_objects2.md`

LiDAR 설정(시뮬레이터 LiDAR, 앞에 SMAP= 를 붙임)
- 기준(기본): `sensor_mapping_nocam.yaml`. 64채널, 초당 30만 점, 100m. 브리지 기본값과 같음
- 가벼운: `SMAP=sensor_mapping_light32.yaml bash .../s59_run.sh ...`. 32채널, 초당 10만 점, 50m
- 이 노트북(16GB)에서 기준 설정으로 주행 중 최소 여유 메모리가 약 2.0GB, Autoware 가 뜨는 순간은 1.7GB 였음(카메라·bag 켜면 더 줌). 그보다 적게 남는 PC 는 가벼운 쪽을 씀
- 정답 객체 노드의 거리(17.5/35/12.5m)는 기준 설정으로 잰 값임. LiDAR 를 가볍게 해도 이 거리는 그대로라, 클러스터링을 붙일 때만 차이가 남

장면(넷째 인자). 자세한 것은 `C/integration/scenario/README.md`
- none: 장면 없음. ped: 돌발 보행자. door: 북문 승강장 승하차. cone: 차로의 콘. pcar: 차로에 걸친 차
- 다섯째 인자 1 이면 추적 카메라 프레임을 `integration/logs/s59/cam_<tag>` 에 남김

장식 레이어(건물 상자·정류장·지면, 선택). 켜려면 회차 전에 한 번 놓고 `LSTATE=` 로 넘김. 방법은 `C/integration/docs/howto_campus.md` 장식 레이어 절, `--xodr C/map/maps/cbnu_campus_frozen_v31.xodr` 를 꼭 줌

## CARLA 없이 개발할 때

미션 노드(정류장 도착 -> 문 -> 출발, 경유점, 요청 정차)는 Autoware 플래닝 시뮬레이터로 만들 수 있음. 같은 지도 폴더를 쓰고 CARLA 는 안 띄움.
순서와 스크립트는 `C/docs/handoff_2/planning_sim_dev.md` 를 봄.

## 막힐 때

- 브리지가 다른 맵을 불러와 캠퍼스가 사라짐: 지도 폴더 이름이 OpenDriveMap 이 아님
- 경로가 안 들어감(route success 없음): `~/autoware_data/maps/OpenDriveMap/lanelet2_map.osm` 해시가 28a57711 인지 봄
- 네트워크 없는 컨테이너(`docker run --network none`, bag 오프라인 처리 등)에서 ROS 노드가 'failed to create domain' 으로 안 뜬다:
  이미지 기본 cyclonedds.xml 이 수신 버퍼 10MB 이상을 요구하는데 호스트 net.core.rmem_max 가 기본값이라 그럼. 아래처럼 lo 만 쓰는 설정을 넘김.
  주행용 compose 는 privileged 라 상관없음.

      -e CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General></Domain></CycloneDDS>'

## 지도를 다시 만들 때

`bash C/map/scripts/s57_make_v31_lanelet2.sh` 가 동결 xodr 에서 lanelet2_map.osm 을 다시 만들고 해시(빌더 출력 83edb775, 최종 28a57711)를 맞춰 봄.

## 넣지 않은 것

CARLA 에셋, Autoware 이미지, 가상환경, 주행 bag·프레임·영상, 세션 보고서, 실험·요약 스크립트. 작업 폴더 백업에만 있음.

지도 데이터 © OpenStreetMap contributors, ODbL 1.0. `map/maps/NOTICE.md` 를 봄.
