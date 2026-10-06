# OSM 조사 기록 (Phase 1-4)

원본: `data/raw/cbnu_campus.osm` (읽기전용, 2026-09-15 22:31 다운로드)
node 11,579 / way 2,067 / relation 41
(1-3 대조: Overpass 실측 대비 node+1,582·relation+3 —
OSM기본API의 문서화된 동작 차이, 오류 아님)

## 범위 판단 정정 (2026-09-15, 사용자 지시)

다운로드 범위(캠퍼스+300m 여유, 사직대로·1순환로·서부로·
성봉로 포함)는 그대로 유지한다 — 정문 진입 구간이 포함돼
있어 유용하기 때문.

다만 "셔틀 노선 대상"의 의미가 정정됨: 사용자가 말한 "큰길"은
교내 간선도로를 뜻하며, 캠퍼스 바깥 간선도로(사직대로·
1순환로·서부로·성봉로)는 셔틀 노선 대상이 아니다.

**판단 근거**:
1. 셔틀은 교내 이동 수단이라 순수 교외 구간을 운행할 이유가 없음
2. 외부 간선 4개 도로는 highway 태그 분석 결과 전부(또는
   거의 전부) 캠퍼스 폴리곤 밖으로 분류됨(secondary/
   secondary_link/tertiary/tertiary_link/residential/
   unclassified/road 태그가 external 100%) → 태그 체계상으로도
   이미 "교내 도로"와 분리되어 있어 별도 판단 기준을 새로
   만들 필요가 없었음
3. 정문 진입 구간처럼 캠퍼스 경계에 걸친(boundary로 분류된)
   짧은 구간은 셔틀이 실제로 지나갈 가능성이 있어 예외로 남김

**변환 단계 기본 정책**: 캠퍼스 외부(external)로 분류된 way는
변환 대상에서 기본 제외. boundary(경계걸침)로 분류된 way는
개별 검토 후 포함 여부 결정.

## highway 값별 way 개수 (전체 2,067건)

| highway | 개수 |
|---|---|
| service | 517 |
| footway | 279 |
| residential | 216 |
| path | 43 |
| secondary | 22 |
| steps | 17 |
| tertiary | 15 |
| secondary_link | 15 |
| track | 9 |
| unclassified | 4 |
| cycleway | 3 |
| tertiary_link | 3 |
| corridor | 2 |
| construction | 2 |
| road | 1 |
| rest_area | 1 |

### service= 하위태그 (517건 중)
parking_aisle 425 / 태그없음 89 / driveway 3

## 캠퍼스 안/밖 분리 (relation 6705106 실제 폴리곤 기준)

방법: Nominatim lookup(osm_ids=R6705106, polygon_geojson=1)로
받은 Polygon(외곽링+홀 50개 링)에 대해, way의 각 노드좌표를
even-odd 교차판정(모든 링을 합쳐 판정 — GeoJSON Polygon의
홀 처리에 표준적으로 쓰이는 방식)으로 내부/외부 판정.
way의 노드가 전부 내부면 internal, 전부 외부면 external,
섞이면 boundary(경계걸침, 정문 진입 구간 등).

highway 태그 있는 way 1,149건 기준:
- internal: 447
- external: 664
- boundary: 38

highway값별 분류:
| highway | internal | boundary | external |
|---|---|---|---|
| service | 195 | 24 | 298 |
| footway | 199 | 13 | 67 |
| path | 30 | 0 | 13 |
| residential | 0 | 0 | 216 |
| secondary | 0 | 0 | 22 |
| secondary_link | 0 | 0 | 15 |
| tertiary | 0 | 0 | 15 |
| tertiary_link | 0 | 0 | 3 |
| unclassified | 0 | 0 | 4 |
| road | 0 | 0 | 1 |
| track | 8 | 1 | 0 |
| steps | 11 | 0 | 6 |
| cycleway | 1 | 0 | 2 |
| corridor | 2 | 0 | 0 |
| rest_area | 1 | 0 | 0 |
| construction | 0 | 0 | 2 |

→ 외부 간선 4개 도로가 쓰는 태그(secondary·secondary_link·
tertiary·tertiary_link·residential·unclassified·road)는
전부 100% external. 이는 위 "범위 판단 정정"의 근거 2번을
실측으로 뒷받침한다.

## parking_aisle 간선후보 (판별기준 적용)

기준(제시·검증됨): 길이≥200m AND 양끝 각각 다른 highway way
2개 이상과 연결(막다른길 아님). 425건 중 24건 해당.
기지 사례 2건(446440340, 774924319) 모두 포함되어 기준 타당성
확인.

24건의 안/밖 분류: internal 13 / boundary 3 / external 8

| way id | 길이(m) | 분류 |
|---|---|---|
| 481950067 | 520 | internal |
| 774924319 | 442 | internal |
| 648686770 | 431 | internal |
| 392632036 | 428 | internal |
| 392632035 | 419 | internal |
| 481943504 | 395 | internal |
| 452421706 | 368 | boundary |
| 452870642 | 342 | external |
| 469888363 | 324 | external |
| 470097495 | 296 | external |
| 446440340 | 282 | internal |
| 493198759 | 272 | external |
| 442595037 | 261 | internal |
| 442708395 | 260 | internal |
| 452870648 | 252 | boundary |
| 473537465 | 251 | external |
| 469652320 | 248 | external |
| 452870641 | 245 | boundary |
| 442595034 | 218 | internal |
| 442595030 | 215 | internal |
| 469652294 | 209 | external |
| 473926892 | 205 | internal |
| 442708402 | 204 | internal |
| 452870654 | 200 | external |

→ internal 13건 + boundary 3건 = 16건이 "범위 판단 정정"
정책상 태그 보정 우선 후보(외부 8건은 기본 제외 후보).

### 판별기준의 한계 (2026-09-15 확인)

정문-도서관 최단경로를 그래프(BFS, 내부+경계 차도후보
228건)로 직접 계산한 결과, 경로를 구성하는 4개 way 중
3개(392532207 147.5m·481950063 48.7m·481950062 64.5m)가
길이<200m라는 이유만으로 16건 후보에서 빠져 있었음이
확인됨.

**길이 기준은 도로 중요도의 대리 지표일 뿐이며, 짧아도
경로 연결에 필수인 구간을 놓친다. 실제 주행 경로를
계산해 대조하는 절차가 필요하다는 것을 392532207/
481950063/481950062 사례로 확인.**

→ 보정 대상을 16건에서 19건으로 확장(사용자 승인,
2026-09-15). fix_tags.py TARGET_WAY_IDS_ROUTE_BASED 참고.

## 연결성 점검 (차도 후보: service/residential/track/
unclassified/tertiary/tertiary_link/secondary/
secondary_link/road, internal+boundary만)

대상 228건 → 연결요소(공유노드 기준) 5개:
- 주요 요소 222건 (97.4%)
- 고립 요소 4개(각 1~2건):
  - way 442709214,471667849 (약 36.62767,127.44992)
  - way 665834148,665834149 (약 36.62824,127.46198)
  - way 471400720 (약 36.63021,127.45186, track)
  - way 473927737 (약 36.62543,127.46073)

→ "정문–도서관 7개 way" 구간의 구체적 way id는 이번 집계에
쓰지 않았으나(사전조사에서 이미 연결 확인됨), 내부+경계
차도 후보의 97.4%가 하나의 연결요소로 묶여 있어 전반적
연결성은 양호. 고립 4곳은 소규모(각 1~2 way)로 셔틀 노선
설계 시 개별 확인 필요.

## 속성 결손

| 범위 | 총 | lanes | width | oneway |
|---|---|---|---|---|
| 전체 highway | 1,149 | 17 | 2 | 62 |
| internal만 | 447 | 0 | 2 | 14 |

횡단보도(crossing) 노드: 14
신호등(traffic_signals) 노드: 4

## CARLA Osm2Odr 변환 기본 동작 (공식 문서 확인, 2026-09-15)

출처: carla-simulator/carla 0.9.15 태그, Docs/python_api.md,
`carla.Osm2OdrSettings.set_osm_way_types` 설명

> By default the road types imported are `motorway,
> motorway_link, trunk, trunk_link, primary, primary_link,
> secondary, secondary_link, tertiary, tertiary_link,
> unclassified, residential`.

**핵심**: 기본값에 `service`, `track`, `footway`는 없다.
즉 highway=service로 남아있는 도로(parking_aisle 포함,
위 16건의 간선후보 포함)는 태그를 바꾸지 않으면 변환 시
자동으로 빠진다 — 태그 보정이 "정리"가 아니라 변환에
필수적인 단계임을 의미한다.

`default_lane_width` 기본값 4.0m(문서 확인).

이 사실은 1-6(HARD STOP C) "보정 후 태그 값" 결정에
직접 반영: 간선후보를 기본 목록에 있는 값(예: unclassified)
으로 바꾸면 `set_osm_way_types`를 커스텀할 필요 없이 변환됨.

## 2-2 검증 후 road 출처 분해 (2026-09-15, 재변환 3회로 실측)

방법: 동일 Osm2OdrSettings(default_lane_width=3.35,
generate_traffic_lights=False)로 (a) 보정 전 원본
(b) 내부+경계(485way)만 추출한 사본 (c) 내부+경계에서
19건까지 제외한 사본(466way), 총 3회 추가 변환해 비교.

- 전체(19건 보정 반영) road: 4,200개
- 내부(캠퍼스 폴리곤 기준) road: **134개(3.2%)**
- 외부 road: 4,048개(96.8%, 보정 전 원본과 정확히 일치 —
  외부 도로는 19건 보정과 무관하게 그대로)
- 최대길이 road: 3,029.8m, 이름 "산단로"(id 5134/4541,
  방향별 쌍) → 캠퍼스 폴리곤 밖, 다운로드범위 안의 이름
  있는 외부 간선도로. 사용자가 확인한 4개 외부간선(사직대로
  등)과는 별개 도로지만 마찬가지로 external 분류와 일치.

**핵심 발견**: 캠퍼스 내부/경계로 분류된 way 485건 중,
Osm2Odr가 도로로 인식하는 태그를 가진 것은 보정한 19건
뿐이다. 나머지 466건(footway/path/service=parking_aisle/
track/steps 등)은 도로 기여가 0이다 — 19건을 제외하고
내부+경계 사본만 변환을 시도하면 **"No edges loaded"로
변환 자체가 실패**한다(실측 확인, 2026-09-15). 즉 캠퍼스
내부 도로망은 전적으로 이 19건에 의존한다. 참고 파일:
`map/maps/cbnu_internal_only.xodr`(road134/junction25),
`map/maps/cbnu_unfixed_reference.xodr`(보정 전 진단용).

## 내부+경계 485-way 사본 재구성 (2026-09-17, 맵세션3)

**유실 경위**: 이 절에서 서술한 internal/boundary/external 분류와
485-way 사본은 2026-09-15 세션에서 일회성 코드로만 실행되고
스크립트로 저장되지 않았음. 그 결과 `cbnu_internal_only.xodr`
(134road/25junction)의 산출 경로가 한동안 재현 불가능한
상태였음(2026-09-17 맵세션3에서 발견).

**복원**: `map/scripts/classify_internal.py`. 입력은
`data/processed/cbnu_campus_fixed.osm` + `data/processed/
cbnu_relation_polygon.json`(Nominatim relation 6705106 폴리곤,
재조회해 저장), 출력은 `data/processed/cbnu_internal_boundary.osm`
(485way). 판정 기준(모든 node 내부→internal, 모든 node 외부→external,
섞임→boundary)과 GeoJSON 홀 처리(전체 링 XOR)는 스크립트 자체의
주석에 상세 기재. 재실행 결과 이 문서의 기록값(internal447/
boundary38/external664, 합계1149, 485way)과 **완전히 일치**함을
확인(map/logs/classify_internal.log).

공식문서(Docs/python_api.md)는 `Osm2OdrSettings.
default_lane_width` 기본값을 4.0으로 기재하나, 설치된
carla==0.9.15 패키지를 직접 인스턴스화해 실측하면
실제 런타임 기본값은 3.35였음(소스 코드까지는 미확인).
문서가 최신 상태가 아닐 수 있다는 경고 — Osm2OdrSettings의
다른 옵션(center_map, proj_string, generate_traffic_lights
등, 문서에 기본값 자체가 기재 안 된 것들 포함)이나 이후
다른 CARLA 기능에서도 문서값과 실측값이 다를 수 있으니,
근거 제시 시 "문서 기재값"과 "설치판 실측값"을 구분해서
확인할 것.

## 인도(sidewalk) lane 확인 (2026-09-18, 맵세션4)

`maps/cbnu_internal_only_localtm.xodr`의 lane type을 전수
집계(grep 직접 카운트, 134개 laneSection 기준):
none 134 / driving 134 / **sidewalk 86**(전체 134 중 64%) /
solid 224 / broken 8 / town 134.

**결론: 인도 lane이 이미 존재함. 별도 생성 작업 불필요로 확정.**
Osm2Odr가 변환 시점에 OSM 원본 태그 기반으로 자동 생성한
것으로 추정(내부 로직은 SUMO netconvert 영역이라 CARLA
문서 범위 밖, 미확인). 공식문서(adv_opendrive, 2026-09-18
조회) 인용: "Pedestrians will navigate over the sidewalks
and crosswalks that appear in the map." — CARLA가 이 sidewalk
lane을 보행 가능 영역으로 실제 사용함을 확인.

Osm2OdrSettings 옵션 전수(설치판 실측,
`help(carla.Osm2OdrSettings)`): all_junctions_with_traffic_lights,
center_map, default_lane_width, elevation_layer_height,
generate_traffic_lights, offset_x, offset_y, proj_string,
use_offsets. 인도 폭을 별도 지정하는 옵션은 없음 — 폭 조정이
필요해지면 default_lane_width(현재 전체 3.35m 단일값, 인도에도
동일 적용되는지는 별도 확인 필요) 조정 외에 경로가 없음.
오늘(Phase B/C) 범위에서는 "인도 있음, 추가작업 불필요"로
결론짓고 다음 단계 진행.
