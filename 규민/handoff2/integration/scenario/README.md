# 시나리오 도구 (세션58~59)

`C` = `~/campus_mobility_sim`. 파이썬은 `C/.venv-carla/bin/python`. 모두 CARLA 클라이언트로 따로 띄우고, 브리지가 동기 모드로 틱을 돌리는 동안
`wait_for_tick` 으로 따라감. Autoware·브리지는 안 고침. 회차 하나를 묶은 스크립트는 `C/integration/docs/tools/s59_run.sh`.

## 보행자 도구 ped_tool.py
- 승강장(노선 파일 `map/data/shuttle_platforms_v31_r1.yaml`) 기준 위치 이름으로 보행자를 만들고 걷게 하고 지움
- 위치 이름: door_out(문 앞 0.45m), dz_center(문 구역 무게중심), walk_back·walk_fwd(dz_center 에서 차 뒤·앞 6m),
  corr_center(전방 통로 무게중심), corr_side(그 바깥 3.0m), 또는 지도 좌표 [x, y]
- 단계: wait_ego_stopped [s], wait_ego_near <m>, spawn <id> <위치>, walk <id> <위치> [속도], wait <s>, destroy <id>
- 단계는 YAML(steps) 또는 `--steps "a;b;c"`. 청사진은 `--seed` 로 고정
- 기록 `--log`: 틱마다 보행자 위치, 문 구역·전방 통로 안 여부, 단계
- 서버 없이 위치만 확인: `python ped_tool.py door_lib_north.yaml --dry`
- .venv-carla 에 PyYAML 이 없어 YAML 은 시스템 python3 로 읽음(pip 안 함)
- 예: `python ped_tool.py door_lib_north.yaml --log out.csv --timeout 1500`

## 장면 셋
- 가. 돌발 보행자 `dart_out.py <out.csv> --road 1415 --s 45 --trigger 20`: 차로 밖 3.1m 에 밴(sprinter)을 세우고, 그 앞에서
  보행자가 자차 20m 안에 들어오면 1.4m/s 로 차로를 건넘. 기록: 앞 범퍼-보행자 gap, 상자 사이 최소 거리, 충돌
- 나. 문 구역 `ped_tool.py door_lib_north.yaml`: 북문 정차 뒤 하차 -> 문 구역 5초 서 있기 -> 전방 통로 들어왔다 나가기 -> 탑승
- 다. 정차 차량·콘 `obstacles.py <out.csv> --road 1415 --car-s 55 --cone-s 30`: 콘(role obstacle, 차선 중심 오른쪽 0.9m)과
  차로에 0.7m 걸친 audi.a2. `--no-car` 면 콘만(다1), `--no-cone` 이면 걸친 차만(다2). 기록: 자차 road s, 옆 벗어남
- s59_run.sh 장면 이름: ped(가), door(나), obst(다, 둘 다), cone(다1), pcar(다2), none

## 추적 카메라 chase_cam.py
- `python chase_cam.py <frame_dir> [--w 640 --h 360 --fps 10 --dist 7 --height 3.2 --pitch -14 --fov 90]`
- 자차(role ego_vehicle) 뒤에 RGB 카메라를 Rigid 로 달아 주행 중 jpg 를 남김. 자차가 없어지거나 SIGINT 면 끝
- 영상: `python3 C/integration/docs/tools/s4_label.py <frame_dir> <글자 폴더> 0.1 "<장면>  <모드>"` ->
  `docker run --rm --network none --entrypoint python3 -u 1000:1000 -v <글자 폴더>:/f:ro -v C/integration:/i <1.9.0 이미지> /i/docs/tools/i2_encode.py /f /i/recordings/<이름>.mp4 10`
- 카메라를 켜면 주행 중 최소 avail 이 약 200MiB 더 내려감(세션59 2,175MiB)

## 요약 스크립트
- `s59_summary_ped.py <tag>`: 가. 보행자 출발, 처음 넘긴 틱(시각·거리), 섰는지, 상자 사이 최소 거리, 충돌
- `s59_summary_door.py <tag>`: 나. 문 구역·전방 통로 안 보행자 틱을 정답 전체와 낸 것으로 세어 놓친 시간·늦게 잡은 시간
- `s59_summary_obst.py <tag> <monitor.log>`: 다. 콘·걸친 차를 처음 넘긴 거리, 섰는지, 남은 길이(차로 따라 근사), 충돌, 도착
- s58_* 은 같은 내용의 세션58 판(로그 경로만 다름)
- 회차 공통 값(도착·정지 오차·실시간 배율·최소 avail·충돌)은 `C/integration/docs/tools/i62_summary.py <tag> <monitor.log>`
