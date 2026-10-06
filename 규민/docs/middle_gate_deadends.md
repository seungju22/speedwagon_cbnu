# 중문 직전 878 road 끝점(dead end) 조사 (맵세션7 추가, 읽기전용)

- 끝점 = 변환된 way 의 첫/끝 노드 중 다른 변환 way 와 접점이 없는 막다른 OSM 노드(xodr 에서는 U턴 junction 으로 표현되어 링크 유무로는 안 잡힘). 중문 정류장(36.633602,127.460506) 직선거리 순 5개
- (참고) xodr 에서 pred/succ 링크 자체가 없는 road 끝은 2개이며 중문 정류장에서 1000m 이상 떨어진 병원 권역·서남측뿐이라 이번 대상이 아님
- 좌표는 OSM 노드 위경도 기준. xodr road 는 보정 잔차 최대 10.6m 오차. 이어지는 way = 끝 노드에서 30m 이내에 닿는 다른 way
- 중문 정류장 최근접 OSM 그래프 노드 2261340224 (11m)

- 중문 게이트 추정점(폴리곤 경계에서 정류장에 가장 가까운 점): (36.633680,127.458398) ENU (200,639), 정류장까지 189m. OSM 에 게이트 지물이 없어 추정
- 게이트 추정점 60m 내 way: way293271139(highway=residential,name=내수동로102번길,oneway=yes,297m,밖,미변환,4m), way169523944(highway=residential,name=내수동로,1211m,밖,미변환,12m), way471401000(highway=residential,59m,밖,미변환,12m), way471401004(highway=service,16m,밖,미변환,17m), way471401003(highway=service,99m,밖,미변환,24m), way293271115(highway=residential,name=사직대로112번길,147m,밖,미변환,35m), way293271133(highway=residential,name=1순환로672번길,oneway=yes,361m,밖,미변환,41m), way293271116(highway=residential,name=사직대로104번길,153m,밖,미변환,44m)

## 끝점 1: OSM 노드 4437127634 (junction87, 들어오는 road 1327)
- 좌표 (36.631753,127.458015) ENU (166.5,426.4), 중문 정류장까지 303m, 폴리곤 안
- 대응 자기 way: way446440351 highway=unclassified,service=parking_aisle 84m 안 (이 끝에서 끝나는 way)
- 이어지는 way446440352: highway=service,service=parking_aisle 75m 안(안쪽 75m) 미변환 [노드 공유]
- 이어지는 way446440353: highway=service,service=parking_aisle 38m 안(안쪽 38m) 미변환 [노드 공유]
- 이어지는 way472037987: highway=service 8m 밖(안쪽 0m) 미변환 [26m 이격]
- 중문 정류장 도달(차량 way 만·방향 준수(진출)): 총 1375m, way 8개: way446440351(unclassified,84m,안) -> way442595034(unclassified,11m,안) -> way442595029(unclassified,174m,안) -> way774924319(unclassified,175m,안) -> way442595017(unclassified,127m,안) -> way442595019(service,157m,횡단) -> way169523944(내수동로,599m,밖) -> way792211896(1순환로,49m,밖)
  - 폴리곤 경계 통과 지점 약 (36.632528,127.454057), 중문 정류장까지 직선 588m
  - 경로가 중문 게이트 추정점에 가장 가까이 지나는 거리: 12m
- 중문 정류장 도달(모든 highway·무방향(보행로 포함)): 총 546m, way 5개: way446440351(unclassified,84m,안) -> way442595034(unclassified,8m,안) -> way442710666(footway,58m,횡단) -> way293271134(1순환로674번길,381m,밖) -> way792211896(1순환로,14m,밖)
  - 폴리곤 경계 통과 지점 약 (36.631691,127.457079), 중문 정류장까지 직선 373m
  - 경로가 중문 게이트 추정점에 가장 가까이 지나는 거리: 124m

## 끝점 2: OSM 노드 4402717760 (junction 없음)
- 좌표 (36.631161,127.458932) ENU (248.4,360.4), 중문 정류장까지 306m, 폴리곤 안
- 대응 자기 way: way442595040 highway=unclassified,service=parking_aisle 69m 안 (이 끝에서 끝나는 way)
- 이어지는 way442595042: highway=service,service=parking_aisle 190m 횡단(안쪽 157m) 미변환 [노드 공유]
- 중문 정류장 도달(차량 way 만·방향 준수(진출)): 총 1558m, way 8개: way442595040(unclassified,69m,안) -> way442595037(unclassified,142m,안) -> way442595034(unclassified,139m,안) -> way774924319(unclassified,277m,안) -> way442595017(unclassified,127m,안) -> way442595019(service,157m,횡단) -> way169523944(내수동로,599m,밖) -> way792211896(1순환로,49m,밖)
  - 폴리곤 경계 통과 지점 약 (36.632528,127.454057), 중문 정류장까지 직선 588m
  - 경로가 중문 게이트 추정점에 가장 가까이 지나는 거리: 12m
- 중문 정류장 도달(모든 highway·무방향(보행로 포함)): 총 740m, way 6개: way442595040(unclassified,69m,안) -> way442595037(unclassified,142m,안) -> way442595034(unclassified,77m,안) -> way442710666(footway,58m,횡단) -> way293271134(1순환로674번길,381m,밖) -> way792211896(1순환로,14m,밖)
  - 폴리곤 경계 통과 지점 약 (36.631691,127.457079), 중문 정류장까지 직선 373m
  - 경로가 중문 게이트 추정점에 가장 가까이 지나는 거리: 124m

## 끝점 3: OSM 노드 4400580300 (junction108, 들어오는 road 1294)
- 좌표 (36.629483,127.459614) ENU (309.3,173.7), 중문 정류장까지 465m, 폴리곤 안
- 대응 자기 way: way442361829 highway=unclassified,service=parking_aisle 79m 안 (이 끝에서 끝나는 way)
- 이어지는 way815864977: highway=service,service=parking_aisle 35m 안(안쪽 35m) 미변환 [노드 공유]
- 이어지는 way442361830: highway=service,service=parking_aisle,oneway=yes 63m 안(안쪽 63m) 미변환 [9m 이격]
- 이어지는 way442361832: highway=service,service=parking_aisle 154m 안(안쪽 154m) 미변환 [9m 이격]
- 이어지는 way471667673: highway=footway 135m 안(안쪽 135m) 미변환 [14m 이격]
- 이어지는 way471667675: highway=footway 70m 안(안쪽 70m) 미변환 [14m 이격]
- 중문 정류장 도달(차량 way 만·방향 준수(진출)): 총 1504m, way 9개: way815864977(service,10m,안) -> way442595043(service,31m,안) -> way442595044(service,27m,안) -> way648686772(unclassified,62m,안) -> way774924319(unclassified,443m,안) -> way442595017(unclassified,127m,안) -> way442595019(service,157m,횡단) -> way169523944(내수동로,599m,밖) -> way792211896(1순환로,49m,밖)
  - 폴리곤 경계 통과 지점 약 (36.632528,127.454057), 중문 정류장까지 직선 588m
  - 경로가 중문 게이트 추정점에 가장 가까이 지나는 거리: 12m
- 중문 정류장 도달(모든 highway·무방향(보행로 포함)): 총 605m, way 4개: way442361829(unclassified,9m,안) -> way442361832(service,115m,안) -> way815864979(footway,106m,횡단) -> way792211896(1순환로,376m,밖)
  - 폴리곤 경계 통과 지점 약 (36.630104,127.461624), 중문 정류장까지 직선 402m
  - 경로가 중문 게이트 추정점에 가장 가까이 지나는 거리: 199m

## 끝점 4: OSM 노드 4655491158 (junction102, 들어오는 road 1212)
- 좌표 (36.629850,127.458204) ENU (183.4,214.5), 중문 정류장까지 466m, 폴리곤 안
- 대응 자기 way: way471400900 highway=unclassified 68m 안 (이 끝에서 끝나는 way)
- 이어지는 way471400896: highway=service,service=parking_aisle 53m 안(안쪽 53m) 미변환 [노드 공유]
- 이어지는 way471400882: highway=path 27m 안(안쪽 27m) 미변환 [5m 이격]
- 이어지는 way471400878: highway=footway 17m 안(안쪽 17m) 미변환 [15m 이격]
- 이어지는 way471400899: highway=path 38m 안(안쪽 38m) 미변환 [16m 이격]
- 이어지는 way471400904: highway=path 38m 안(안쪽 38m) 미변환 [20m 이격]
- 이어지는 way471400903: highway=path 82m 안(안쪽 82m) 미변환 [27m 이격]
- 중문 정류장 도달(차량 way 만·방향 준수(진출)): 총 1555m, way 9개: way471400896(service,53m,안) -> way442595043(service,38m,안) -> way442595044(service,27m,안) -> way648686772(unclassified,62m,안) -> way774924319(unclassified,443m,안) -> way442595017(unclassified,127m,안) -> way442595019(service,157m,횡단) -> way169523944(내수동로,599m,밖) -> way792211896(1순환로,49m,밖)
  - 폴리곤 경계 통과 지점 약 (36.632528,127.454057), 중문 정류장까지 직선 588m
  - 경로가 중문 게이트 추정점에 가장 가까이 지나는 거리: 12m
- 중문 정류장 도달(모든 highway·무방향(보행로 포함)): 총 641m, way 6개: way471400900(unclassified,38m,안) -> way815864978(unclassified,65m,안) -> way442595034(unclassified,85m,안) -> way442710666(footway,58m,횡단) -> way293271134(1순환로674번길,381m,밖) -> way792211896(1순환로,14m,밖)
  - 폴리곤 경계 통과 지점 약 (36.631691,127.457079), 중문 정류장까지 직선 373m
  - 경로가 중문 게이트 추정점에 가장 가까이 지나는 거리: 124m

## 끝점 5: OSM 노드 4402715664 (junction41, 들어오는 road 1282)
- 좌표 (36.632250,127.455270) ENU (-78.7,481.7), 중문 정류장까지 491m, 폴리곤 안
- 대응 자기 way: way442595017 highway=unclassified,service=parking_aisle 161m 안 (이 끝에서 끝나는 way)
- 이어지는 way442595018: highway=service,service=parking_aisle 143m 안(안쪽 143m) 미변환 [노드 공유]
- 이어지는 way442595022: highway=service,service=parking_aisle 35m 안(안쪽 35m) 미변환 [3m 이격]
- 이어지는 way442595025: highway=service,service=parking_aisle 30m 안(안쪽 30m) 미변환 [14m 이격]
- 이어지는 way442595023: highway=service,service=parking_aisle 10m 안(안쪽 10m) 미변환 [20m 이격]
- 중문 정류장 도달(차량 way 만·방향 준수(진출)): 총 813m, way 5개: way442595018(service,143m,안) -> way442595027(service,109m,횡단) -> way293271139(내수동로102번길,166m,밖) -> way293271134(1순환로674번길,381m,밖) -> way792211896(1순환로,14m,밖)
  - 폴리곤 경계 통과 지점 약 (36.632993,127.457385), 중문 정류장까지 직선 287m
  - 경로가 중문 게이트 추정점에 가장 가까이 지나는 거리: 117m
- 중문 정류장 도달(모든 highway·무방향(보행로 포함)): 총 614m, way 5개: way442595018(service,143m,안) -> way442595027(service,109m,횡단) -> way293271139(내수동로102번길,132m,밖) -> way169523944(내수동로,182m,밖) -> way792211896(1순환로,49m,밖)
  - 폴리곤 경계 통과 지점 약 (36.632993,127.457385), 중문 정류장까지 직선 287m
  - 경로가 중문 게이트 추정점에 가장 가까이 지나는 거리: 4m

## 공통 관찰
- 5개 끝점 모두 이어지는 OSM way 가 원본 service(parking_aisle) 등 미변환 way 다. 끊긴 원인은 '연결 실패'가 아니라 세션4 fix_tags 73건에 들어가지 않아 변환기에서 빠진 것이다(중문 주변 도로 전체가 같은 패턴).
- 끝점 5는 앞서 주황 후보(way442595018)의 시작점과 같다. 사용자 판정으로 주황은 중문이 아니라 별개 출구다.
