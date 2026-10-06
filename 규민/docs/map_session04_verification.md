# 맵세션4 Phase C-5 검증 (자동생성, 읽기전용)

## 1. road / junction 개수
- 기존(19건판): road 134 / junction 25
- 신규(73건판): road 878 / junction 95

## 2. 정문-도서관 경로
- 기존: 존재, 거리가중 길이 529.86m(dijkstra합산 529.86m), road수 5, road id=['300', '402', '319', '405', '318']
- 신규: 존재, 거리가중 길이 529.86m(dijkstra합산 529.86m), road수 15, road id=['1247', '1914', '1356', '1917', '1355', '1447', '1373', '1673', '1374', '1591', '1375', '1689', '1376', '1625', '1377']

## 3. junction 쌍 거리 비율 (공통 이름 junction 15개)
- 쌍 수: 105
- 비율 min/median/mean/max: 0.9633/1.0002/1.0018/1.1186

## 4. road id 변경 여부
- road 요소에 name 속성이 비어있어(name="") id로 직접 대응 불가 (osm2odr가 원본 OSM way id를 road name에 보존하지 않음)
- id 범위: 기존 280~413, 신규 1120~1997
- 결론: 개수 자체가 다르므로(134->878) id 체계가 전면 재부여됨. 1:1 대응표 작성 불가(대응 근거 없음), 사실만 기록

## 5. 기하 이상 개수 (analyze_geometry.py 함수 재사용)
- 기존: connection 90건 중 gap이상(>=0.01m) 40건, 종단(along)이상 1건, heading_diff median=0.0000deg max=37.8597deg
- 신규: connection 618건 중 gap이상(>=0.01m) 131건, 종단(along)이상 1건, heading_diff median=0.0000deg max=76.4957deg

