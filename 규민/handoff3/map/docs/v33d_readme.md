# 지도 v3.3d (11/04)

v3.3c 에서 후문 앞 유턴만 바꾼 판. 도로(xodr 214507af)·CARLA 월드·장식 레이어는 v3.1 그대로이고 Lanelet2 지도만 다름.
v3.1 동결 묶음과 ~/autoware_data/maps/OpenDriveMap 은 손대지 않음. 새 판은 다른 경로의 OpenDriveMap 폴더로 마운트해서 고름.
그 앞 판에서 바뀐 것은 map/docs/v33b_readme.md, map/docs/v33c_readme.md 에 있음.

## 파일
- map/maps/v33d/lanelet2_map.osm(fe9ee6ff), 바뀐 것 v33d_changes.json. 바탕은 map/maps/v33c
- 승강장: 프리우스 map/data/shuttle_platforms_v33d_r1.yaml(값은 v33c 와 같음), 스프린터 shuttle_platforms_v33d_r1_sprinter.yaml
- 순환 노선: 프리우스 map/data/routes/circulation_v33d_r1.yaml, 스프린터 circulation_v33d_r1_sprinter.yaml(목표·경유점만 다름)
- 그림: map/docs/figs/v33d_back_uturn.png(후문 섬 둘레 유턴과 스프린터 차체 자취), v33d_sprinter_door.jpg(문 자리 확인 사진)
- 만든 순서

      .venv-carla/bin/python map/scripts/s77_uturn_search.py --src map/maps/v33c/lanelet2_map.osm --a 34923 --b 34884 \
          --box 920,1010,240,330 --irange 0,20 --jrange 0,17 --rmin 4.1 --rmax 7.0 --lmax 50 --emit 0 \
          --out integration/logs/s77/uturn_island_search.json
      python3 map/scripts/s77_make_v33d.py --ext 4
      ~/lanelet2_trial_venv/bin/python map/scripts/s77_check_v33d.py map/maps/v33d/lanelet2_map.osm integration/logs/s77/v33d_route_check.json
      (cd map/scripts && ../../.venv-carla/bin/python s77_platforms_sprinter.py --veh sprinter)
      python3 map/scripts/s77_write.py --loop-len 3406.1 --seg3-len 544.9
      (cd map/scripts && ../../.venv-carla/bin/python s77_m20.py)

## 후문 앞 유턴
후문 앞 길 끝은 막다른 곳이 아니고 섬(약 20 x 9m)을 둘러싼 고리임. 현장 확인(11/04)으로 실제로 돌 공간이 있다고 해서 스프린터가 도는 반경으로 다시 잡음.
- 지운 것: 52800(v3.3b 에서 넣은 열쇠 구멍 모양 유턴, 반경 약 2.9m), 49898(섬 동쪽 끝 연결, 반경 2.94m)
- 넣은 것: 52806. 섬 남쪽 34923(r1391) 에서 섬 북쪽 34884(r1390) 로 섬 동쪽 끝을 돎. 반경 6.9m, 39.3m(끝 곧은 4m 포함), 경로 점을 centerline 으로 넣음
- 필요한 조향: 스프린터 0.49rad, 프리우스 0.38rad. 스프린터 최대 0.77rad(뒷축 반경 3.77m) 안
- 포장 면 레이어는 필요 없었음. 바퀴는 지금 CARLA 도로면 안(바퀴 검사 7점이 남지만 모두 면 끝에서 0.05~0.10m, 타이어 반폭 0.12m 를 더한 값)
- 34923·34884 를 유턴 자리에서 나눔(앞쪽이 원래 id). 후문을 지나는 30쌍 10개는 레인렛 id 만 갈리고 경로·길이 같음

## 스프린터 문 기준 정차
- CARLA 스프린터(vehicle.mercedes.sprinter)는 오른쪽 슬라이딩 문. 옆 사진에서 문짝 이음매 두 줄 사이를 재면 문 가운데가 뒷축 앞 1.52m(문짝 1.55m, 열림 폭은 1.3m 로 봄)
  - 처음엔 앞축 기준으로 2.11m 로 잡았다가 사진을 보고 고침
- 표지 점(문이 설 차로 가운데 점)은 프리우스 판과 같게 두고, 차 치수·문 자리만 바꿔 승강장 검사(문 구역 2.0m2 이상, 문 옆 깊이 0.8m, 인도 안 90%, 앞 통로 8m, 차체가 레인렛 안, 교차로·횡단보도 없음)를 다시 함
- 16곳 모두 통과. 14곳은 표지 점 그대로라 목표(뒷축)가 프리우스보다 0.14~0.23m 뒤. 개성재 두 곳은 차 앞이 교차로에 걸려 1m 뒤로 옮김(목표 1.20m 뒤)
- 스프린터 판에는 대체 정차점(alt_stop)이 없음

## 순환 노선
- 한 바퀴 약 3,406m, 6곳: 정문 안쪽 > 양성재(이쪽) > 후문 안쪽 > (후문 앞 섬 유턴) > 도서관 남문(건너편) > 도서관 북문 > (모서리 유턴) > 개성재 > (N14 통로) > 정문 안쪽
- 후문 안쪽 -> 남문 구간이 544.9m 로 51m 늘어남(섬을 돌아 나옴)
- 사범대 앞 대신 하차 자리는 안 씀(11/04 판정)

## 쓰는 법
- CARLA: integration/docs/tools/s77_run_loop.sh 에 MAP_COMPOSE=integration/config/s77_v33d_map.override.yaml MAP_PATH=/home/aw/s77_maps/OpenDriveMap. 지도 사본은 integration/maps/s77_v33d/OpenDriveMap 에 둠(폴더 이름은 OpenDriveMap)

      mkdir -p integration/maps/s77_v33d/OpenDriveMap
      cp map/maps/v33d/lanelet2_map.osm map/maps/v33d/map_projector_info.yaml integration/maps/s77_v33d/OpenDriveMap/

- 차는 VEHICLE 로 고름. 자세한 것은 인계 3 README

## 한계
- 섬 둘레 유턴은 맞은편에서 오는 차가 없을 때만 도는 것으로 봄. 그 판단은 판단 쪽 몫
- 스프린터 30쌍 차체 검사는 v3.3c 와 같음(389점 24건, 좁은 꺾임에서 몸체가 레인렛 밖)
- 순환 전체 바퀴 검사: 스프린터 25점, 프리우스 13점(v3.3c 37·31점)
