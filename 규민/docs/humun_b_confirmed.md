# 후문B(병원측) 채택 후보 확정 표 (맵세션7 추가)

사용자 판정: 후문=B(병원측). 캠퍼스 안쪽 구간만 채택, 폴리곤 밖 공도(1순환로 등) 제외. 재변환은 보류(이 표는 후보 확정까지).

| 구분 | way id | 태그 | 길이(m) | 폴리곤 판정 | 안쪽/밖 길이(m) | 변환 여부 | 방향 | 채택 |
|---|---|---|---|---|---|---|---|---|
| 번호48 | 452870644 | highway=service,service=parking_aisle | 52 | 안 | 52/0 | 미변환(원본 service) | 양방향 | 채택(안쪽 구간) |
| 번호1 | 392632034 | highway=service,service=parking_aisle | 105 | 횡단 | 81/24 | 미변환(원본 service) | 양방향 | 채택(way 전체 105m 중 안쪽 81m; 밖 24m 처리는 아래) |
| 서비스way(도로망 접속) | 481945510 | highway=service,service=parking_aisle,oneway=yes | 9 | 안 | 9/0 | 미변환(원본 service) | 일방통행(way 방향) | 채택(안쪽 구간) |
| 서비스way | 481943505 | highway=service,service=parking_aisle,oneway=yes | 24 | 안 | 24/0 | 미변환(원본 service) | 일방통행(way 방향) | 채택(안쪽 구간) |
| 서비스way | 481475019 | highway=service,service=parking_aisle | 37 | 안 | 37/0 | 미변환(원본 service) | 양방향 | 채택(안쪽 구간) |
| 진입 쌍 | 481943503 | highway=service,service=parking_aisle,oneway=yes | 25 | 안 | 25/0 | 미변환(원본 service) | 일방통행(way 방향) | 채택(안쪽 구간) |
| 진입 쌍 | 481945509 | highway=service,service=parking_aisle,oneway=yes | 5 | 안 | 5/0 | 미변환(원본 service) | 일방통행(way 방향) | 채택(안쪽 구간) |
| 연결/공도 | 662321292 | highway=service | 28 | 밖 | 0/28 | 미변환(폴리곤 밖) | 양방향 | 제외(폴리곤 밖) |
| 연결/공도 | 792211884 | highway=secondary_link,oneway=yes | 31 | 밖 | 0/31 | 미변환(폴리곤 밖) | 일방통행(way 방향) | 제외(폴리곤 밖) |
| 연결/공도 | 774924320 | highway=secondary_link,name=1순환로,oneway=yes | 407 | 밖 | 0/407 | 미변환(폴리곤 밖) | 일방통행(way 방향) | 제외(폴리곤 밖) |

이미 변환된 이웃: way452870641+way452870648 = road1333(485.6m, 고립 성분). 번호48 시작 노드와 번호1 끝 노드가 이 way들과 노드를 공유한다.

## 번호1(way392632034) 밖으로 나가는 부분 분석
- 노드 5개, 순서대로 안/밖: OIIII
- 경계 횡단 구간: 노드3958352335(밖) -> 노드4727599760(안), 횡단 부근 (36.624701,127.463656)
- 밖쪽 끝 노드 3958352335 (36.624701,127.463656) 태그: 없음
- 밖 구간 길이(경계 횡단 세그먼트 포함 개략): 24m ; 이 way 자체에는 name 태그 없음, service=parking_aisle
- 밖 끝 노드 주변 way(공유 노드):
  - way293211022 highway=secondary_link,name=1순환로,oneway=yes 428m 밖
  - way662321292 highway=service 28m 밖
  - way774924322 highway=secondary_link,name=1순환로,oneway=yes 26m 밖
- 판정: 밖 끝 노드가 곧바로 공도(2개 way: way293211022, way774924322)와 접속한다. 즉 way392632034 의 밖 부분은 폴리곤 경계를 넘어 1순환로에 닿는 마지막 한 세그먼트(최대 24m, 이름 없음)뿐으로, 게이트 통과(접속)부다. 공도 구간은 그 너머(1순환로 계열 way)다.
- 제안: 번호1은 way 단위로만 변환되므로 way 전체(105m, 밖 최대 24m)를 채택하고 1순환로 계열은 제외. 접속 세그먼트를 잘라내려면 OSM way 분할 편집이 필요(재변환 단계에서 사용자 결정).
