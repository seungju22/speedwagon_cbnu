# 캠퍼스 장식 레이어 (10/11 정리, 10/26 문체 고침)

레이어 v2(10/23, 건물 2순위·나무)는 map/scripts/place_campus_layers_s66.py 이고 이 스크립트 함수를 불러 씀. 쓰는 법은 integration/docs/howto_campus.md '레이어 v2'.

스크립트: map/scripts/place_campus_layers.py. 측정 묶음: map/scripts/layers_bench.py(10/11 실행용).
맵: v3 동결본(a17ab012e04c664f). xodr·stops·test_drive.py 는 읽기만 함.

## 레이어와 기본값
- buildings: 기본 표현 나(정육면체 덩어리). 가(패키지 건물 메시)·다(혼합)는 --repr 로 고름
  - 다 는 메시 실제 크기를 잰 뒤(layers_bench.py b) 다시 비교함. 지금 가·다 의 침범 수는 메시 폭을 길이와 같게 둔 가정 값임
- busstop: 승강장 11곳 중 10곳. 도서관 남문(이쪽)은 뺐음(아래 한계 3)
- ground: 지면 판. 기본 끔. --layers 에 ground 를 넣어야 켜짐
- trees, parked, props: 빈 레이어(0개)
- 기본 실행(--layers buildings,busstop): 정육면체 721 + 정류장 시설 10 = 731, 경로 차로 침범 0, 그 밖 차로 침범 0(10/11 dry-run)

## 메시 경로 (10/11 대조)
- 쓰는 경로는 모두 10/08 B1 에서 소환 확인한 8종 안임
  - House01, House02, Apartment03(가), GroundCube(나), static.prop.busstop(정류장)
- GroundCube 크기: 패키지 파일(GroundCube.uexp)의 경계값이 반폭 50x50x50cm, 중심 0 임. 엔진 기본 Cube(1m)와 같은 값임
  -> 한 변 1.0m, 피벗 가운데. 10/10 가정과 같음. 서버 측정 전이라 [파일 값]으로 둠
  - B1 API bbox(0.222x1.397x1.000)는 z 만 맞음. 가 메시의 bbox 도 x 축이 화면과 안 맞았음(B1)
- B1 밖 경로(10/11 에서 소환부터 확인): 아래 지면 판 후보와 정육면체 대체 후보
  - /Engine/BasicShapes/Plane.Plane: 1m 판(두께 0), 충돌 상자 있음 [파일 값]
  - /Engine/BasicShapes/Cube.Cube: 1m 정육면체, 충돌 상자 [파일 값]
  - /Game/Carla/Static/SideWalk/SideWalkCube.SideWalkCube: 1m 정육면체, 볼록 충돌 [파일 값]
  - 대체 후보: Plane 이 안 되면 GroundCube·SideWalkCube, SideWalkCube 가 안 되면 GroundCube·Cube, Cube 가 안 되면 GroundCube·SideWalkCube
- /Engine 경로가 static.prop.mesh 로 열리는지는 확인한 적 없음

## ground 레이어
- 차선 조각 외곽 사각형 + 100m 를 덮는 정사각형 하나. scale 은 세 축 같은 값이라 긴 변 기준(지금 1378m)
- 윗면을 도로면(z 0)보다 0.15m 아래에 둠(GROUND_DROP). 0.1~0.2m 범위에서 깜빡임을 피하려는 값임
- 기본 메시 GroundCube(B1 확인 경로). 정육면체라 땅속으로 1378m 내려감. 보이지 않는 쪽임
- 켜면 차가 차도 밖으로 나가도 떨어지지 않음. 그래서 이 레이어를 켠 주행에서 차도 이탈은 낙하로 판정하지 않음
  이탈 판정은 차선 중심에서 옆으로 벗어난 거리로 함(test_drive 의 궤적 이탈과 같은 방식)
- 침범 검사 대상이 아님(차로 아래)
- 10/11 에서 잴 것: LiDAR 점 수 변화, 차도 밖으로 내보낸 차가 서는지, 화면 한 장

## 메시 크기 반영
- layers_bench.py 가 mesh_sizes.json 을 남김. place_campus_layers.py --mesh-sizes <파일> 로 넣음
- 넣으면 가 바닥이 잰 길이 x 폭 사각형이 됨(정사각형 가정이 풀림). 정육면체 한 변·정류장 바닥·지면 판 두께도 바뀜
- 형식 확인(10/11, 지어낸 값): House01 폭만 10m 로 넣으면 가 의 경로 침범이 10건에서 7건으로 줄었음. 형식 확인용이고 실제 값이 아님

## 한계
1. 중문 2동은 깎기만 했음. 물리지 않았음
   - way765358663: 차로 밖으로 빼려면 20.7m 를 옮겨야 함(한도 5m). 옮기면 인문사회관과 86.8m2 겹침
   - 경영대학 본관: 3.9m 를 옮겨야 함. 옮기면 way765358663 깎은 폴리곤과 7.8m2 겹침
   - 깎은 뒤 상자의 경로 차선 겹침은 둘 다 0m2 임
   - way765358663 은 중문 노선 차로가 폴리곤 가운데를 지남. 도로 위를 건너는 구조(지붕 있는 통로 등)인지 현장 확인 필요
2. 층고 4.0m, 기본 4층은 가정값임
   - 층고 = OSM 에 height 와 building:levels 가 둘 다 있는 32동의 height/levels 중앙값. 범위 3.6~7.0m, 사분위 4.0~4.7m
   - 기본 층수 = building:levels 가 있는 96동의 중앙값. 범위 1~14층, 사분위 2~5층
   - 10/11 에 data/raw/cbnu_campus.osm 에서 다시 세어 같은 값을 얻었음
   - 30동 중 levels 태그 있는 21동, 기본값 9동
3. 도서관 남문(이쪽) 승강장에는 정류장 시설을 놓지 않음
   - 원래 자리와 앞뒤 2m 간격 ±20m(21곳)에 걸리지 않는 자리가 없었음
   - 원래 자리는 경로 밖 차로 r1369 와 1.379m2 겹침(10/10 dry-run). 승강장 자체는 stops_v3.yaml 그대로임
4. 이름 없는 5동(way765358663, way471400965, way442760317, way442760303, way471400915)은 이름 칸을 비워 둠. 나중에 채움
5. 여유 0.5m 는 급커브 추종 이탈(r1391 0.626m) 을 덮지 못함(10/10와 같음)
6. 정육면체 721개, 정류장 10개 규모의 메모리·틱 시간은 그때 안 쟀음. 10/11 layers_bench.py c 에서 쟀음(아래 절)
7. 레이어 v2 나무 1그루(SM_Acer_02)의 수관이 건물 상자와 0.11m 겹침(10/23 서버에서 잰 반폭 4.52m 기준, 배치 때 가정 4.32m). 그대로 둠
   - 건물을 OSM 윤곽 대신 직사각형으로 근사한 탓임. 나무는 OSM 위치를 그대로 지킴
   - 차도·인도 쪽 여유는 잰 반폭으로도 최소 0.49m(침범 0). 근거 integration/logs/s66/step5/tree_size_check.txt

## 10/11 측정 반영 (2026-10-11)
- 메시 8후보 모두 소환 확인. GroundCube 1.000m 정육면체(LiDAR), Plane 1m 판. 크기는 logs/s51_bench/mesh_sizes.json
- 지면 판 기본 메시를 Plane 으로(mesh_sizes.json ground_choice). 판 위 차는 빈 곳·차로 밖 20m 에서 z -0.11m 로 섬, 판 없으면 떨어짐
- 정류장 시설: 긴 축 x 를 차로와 나란히(yaw = 차로 방향). 깊이 1.884m 라 인도 안. 열린 면 방향은 확인 필요
- 가·다 경로 침범(잰 폭): 가 6건(3개), 다 4건(2개). 나 0
- 부하(렌더링 켬): 틱 평균 5.0~6.2ms, CARLA RSS +70MiB 이하, get_actors 0.3ms 이하. 녹화 1분 나 24.7MB·다 8.3MB·가 1.9MB
- 실행할 때 --mesh-sizes map/docs/logs/s51_bench/mesh_sizes.json 을 줌
