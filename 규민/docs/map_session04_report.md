# 맵세션4 보고 (2026-09-18)

## 0. 시작 점검

- CLAUDE.md 고정버전 복창 완료: OS/ROS2 Ubuntu22.04/Humble,
  CARLA 0.9.15 패키지(0.9.16·0.10 금지), CARLA 파이썬 클라이언트
  carla==0.9.15(venv 내), Autoware 1.9.0 태그(main 금지),
  Autoware Docker CUDA없는 humble 1.9.0 고정. 변경 없음
- 직전 기록 3줄 요약(setup_log.md, map_session03_report.md):
  1) 맵세션3 마감: Phase A~E 전부 완료. 재변환본
     cbnu_internal_only_localtm.xodr(road134/junction25) 스케일
     문제(1.30배) 해소, 정문(junction2)-도서관(junction11)
     539.75m 경로 연결 확인, CARLA 로드+spectator 육안검증 통과
  2) 메모리: 로드직후avail2.6Gi/swap917MiB → 2분후avail4.9Gi/
     swap909MiB, 중단기준(avail2GiB미만/swap4GiB초과) 미도달
  3) 다음 시작점(세션3 마감 기록): 24-road 컴포넌트(S계열
     건물군)를 정문-도서관 13-road 컴포넌트와 잇는 way 조사
     (오늘 지시서 범위 밖 — Phase B가 우선)
- 읽기전용 점검(2026-09-18): df 여유88G(38%/전 세션 89G에서
  거의 동일), free avail 10Gi/used2.4Gi/swap0B(정상),
  docker ps -a 없음, CarlaUE4 미기동(pgrep 자기명령매칭 1줄만,
  실프로세스 없음) → 세션3 마감 상태와 일치, 정상 시작 조건

## Phase A — 현황 점검과 가능성 조사

### A-1. 세션3 CARLA 측정값

세션3(2026-09-17) Phase E, 실행 명령 원문(map_session03_report.md
그대로 인용):
`~/carla/CARLA_0.9.15/CarlaUE4.sh -quality-level=Low -windowed
-ResX=800 -ResY=600`

| 시점 | avail | swap |
|---|---|---|
| 기준선(서버 실행 전) | 10Gi | 0B |
| 서버 기동 직후 | 4.7Gi | 4MiB |
| 자연 하락 후(로드 스크립트 실행 전) | 2.6Gi | 2.1Gi |
| 맵 로드 직후 | 2.6Gi | 917MiB |
| 로드 2분 후 | 4.9Gi | 909MiB |
| 서버 종료 후 | 7.1Gi | 497MiB |

**CARLA RSS·gtt_used: 세션3에는 기록이 없음.** 세션3은
`free -h`(avail/swap)만 측정했고, RSS·gtt_used 측정은
세션2(2026-09-16) Town10HD_Opt 대조군 실험에서만 있었음
(별개 세션·별개 맵). 이 항목은 세션3 자체에서 누락된 것이지
화면 잘림으로 유실된 게 아님 — setup_log.md 121~135행 전수
확인, RSS·gtt_used 키워드가 09-17 기록에 없음을 grep으로 확인.

세션2 Town10HD_Opt 대조군(2026-09-16, 참고용 비교 대상):
avail1.0Gi/swap3.1Gi, RSS 1466288KiB(약1.4GiB), gtt_used
2776629248B(약2.6GiB).

**비교**: 세션3 캠퍼스맵(134road)은 로드직후avail2.6Gi→2분후
4.9Gi로 세션2 Town10HD_Opt(2분후avail1.0Gi)보다 훨씬 여유 있음.
캠퍼스맵이 Town10HD_Opt보다 훨씬 단순한 메시라는 추정과 부합.
단 RSS·gtt_used가 없어 "무엇이 얼마나 덜 쓰였는지" 구성비 비교는
세션3만으로는 불가.

**중단기준 근접도**: 세션3 전 구간에서 avail최저 2.6Gi(기준
2GiB미만과 0.6Gi 여유), swap최고 2.1Gi(기준 4GiB초과와 1.9Gi
여유) — 두 기준 모두 여유 있게 미도달.

### A-2. 인도(sidewalk) 유무

`maps/cbnu_internal_only_localtm.xodr` lane type 집계
(grep 직접 카운트, 134개 laneSection 기준):

| type | 개수 |
|---|---|
| none(중심기준선) | 134 |
| driving | 134 |
| **sidewalk** | **86** |
| solid(도로표시선) | 224 |
| broken(도로표시선) | 8 |
| town(도로기능 태그) | 134 |

**결론: 인도 lane이 이미 있음(86개 laneSection에 존재, 전체
134개 중 64%).** 추가 생성 작업 불필요.

근거(공식문서, adv_opendrive.readthedocs.io, 오늘 조회):
"Pedestrians will navigate over the sidewalks and crosswalks
that appear in the map." — CARLA가 xodr의 sidewalk lane을
렌더링·보행 가능 영역으로 사용함을 확인.

Osm2OdrSettings 옵션 전수 확인(설치된 carla==0.9.15,
`help(carla.Osm2OdrSettings)` 직접 실행, 공식 문서 아닌 실제
설치 패키지 API 실측 — 세션1 "0-3 휠 확인" 선례와 동일 방식):
all_junctions_with_traffic_lights, center_map,
default_lane_width, elevation_layer_height,
generate_traffic_lights, offset_x, offset_y, proj_string,
use_offsets. **인도 폭을 별도 지정하는 옵션 없음** — 인도는
변환기가 OSM 원본 태그(sidewalk=*, highway=footway 등) 기반으로
자동 판단해 생성한 것으로 추정(단, 변환기 내부 로직까지는
미확인 — SUMO netconvert 쪽 로직이라 CARLA 문서 범위 밖).

CARLA OpenDRIVE 스탠드얼론 모드의 건물·소품 처리 (공식문서
adv_opendrive, 오늘 조회, 원문 인용):
"The resulting mesh describes the road definition in a
minimalistic manner. All the elements will correspond with the
OpenDRIVE file, but besides that, there will be only void."
→ **건물·소품은 xodr 스탠드얼론 모드에서 원천적으로 렌더링
안 됨**(세션 시작 시 사용자 문제제기의 원인 진단 1번과 정확히
일치, 공식문서로 재확인).

### A-3. 건물·소품 배치 가능성 조사

**1) static.prop.mesh 속성 (공식문서 content_authoring_props,
오늘 조회)**

원문 코드 예제 그대로 인용:
```
mesh_path = '/Game/Carla/Static/Car/4Wheeled/ParkedVehicles/Charger/SM_ChargerParked.SM_ChargerParked'
parked_vehicle_bp = bp_lib.find('static.prop.mesh')
parked_vehicle_bp.set_attribute('mesh_path', mesh_path)
parked_vehicle = world.spawn_actor(parked_vehicle_bp, spawn_point)
```
- 메시 경로 지정: `set_attribute('mesh_path', ...)`, 값은
  `/Game/...`로 시작하는 전체 UE 패키지 경로 + `.자산명` 반복
- **크기 조절(scale)**: 공식문서에 언급 없음 — 미확인. 스폰 후
  `actor.set_scale()` 류가 static prop actor에도 통하는지는
  Phase E 실제 소환 시험에서 확인 필요
- 패키지판 동작 조건(원문): "as long as the package contains the
  nominated static mesh in the correct location" — 즉 경로만
  맞으면 패키지판에서도 동작 (공식 확인됨)
- 사전 준비: "Static meshes already included in the CARLA content
  library" — 새 에셋 임포트 얘기 아니라 **기존 콘텐츠 재사용**
  전제. 새 자산 추가 방법은 이 문서 범위 밖

**2) static.prop.* 블루프린트 전수 (공식문서 bp_library, 오늘
조회)**

건물처럼 큰 것은 없음. 확인된 것: 버스정류장(busstop,
busstoplb), 키오스크(kiosk_01), 퍼걸러(pergola), 가로등
(gardenlamp), 벤치(bench01~03), 쓰레기통류(bin, trashcan01~05,
garbage01~06), 유리·의류수거함, 바리케이드류(streetbarrier,
chainbarrier 등), 라바콘(trafficcone01/02), 표지판(streetsign류),
분수(fountain, streetfountain). **나무는 static.prop 카테고리에
없음**(bp_library 문서에 미포함, 별도 카테고리로 추정).

**3) 메시 경로를 Unreal 에디터 없이 알아내는 방법 — 가능,
확인됨(중요 발견)**

이 CARLA 0.9.15 패키지는 `.pak`으로 암호화·병합되어 있지 않고,
`Content/` 아래 `.uasset`/`.uexp` 파일이 UE 프로젝트 폴더 구조
그대로 노출돼 있음(로컬 파일시스템 직접 확인,
`~/carla/CARLA_0.9.15/CarlaUE4/Content/`, 16GB). 공식문서 예제의
`/Game/Carla/Static/Car/.../SM_ChargerParked.SM_ChargerParked`
경로가 로컬 파일
`Content/Carla/Static/Car/4Wheeled/ParkedVehicles/Charger/
SM_ChargerParked.uasset`과 정확히 대응함을 확인(공식문서 예제와
로컬 파일구조가 1:1로 맞아떨어짐 — `Content/`를 `/Game/`으로
치환, 확장자 떼고 자산명 반복하면 mesh_path가 됨).
→ **에디터 없이 `find` 명령만으로 메시 경로 목록화 가능.**

**4) Town10HD 등 기본맵 건물 메시 경로**

`Content/Carla/Static/Building/` 아래 단일 정적메시(SM_*.uasset,
여러 부품 블루프린트가 아니라 병합된 단일 메시라 스폰이 단순함)
다수 확인: SM_House01~06, SM_Apartment03/04(v01/v02_merged),
SM_Mansion01~03, SM_Skyscraper_01~05, SM_SkyScraperNY_1/2,
Hotel, Museum, Station, Factory, GasStation01,
Building/00_Opt/ 아래 각 건물의 최적화(Opt) 버전도 별도 존재
(예 SM_House01_Opt). 도심형(스카이스크레이퍼·아파트) 자산이
풍부해 "Town10HD 계열 도심 건물"이라는 지시서의 사전 판단과
부합. **단, Town10HD_Opt 맵 자체가 정확히 어떤 자산을 쓰는지는
`.umap`이 바이너리라 미확인** — Building 폴더에 도심형 자산군이
존재한다는 것만 확인, Town10HD 전용 자산인지는 별개 확인 필요.
나무: `Content/Carla/Static/Vegetation/Trees/`에 SM_*.uasset
167개 확인(SM_Acer_02, SM_Ash_01, SM_Cypress 등 수종 다양).
가로등: `Content/Carla/Static/StreetLight/`에
BP_Lamppost_short.uasset 등 존재(단, 확장자가 .uasset인 것은
블루프린트일 가능성 있음 — 순수 SM_이 아니라 BP_ 접두. static.
prop.mesh가 받는 게 StaticMesh 애셋이라는 점에서 BP_는 별도
액터로 스폰해야 할 가능성, Phase E에서 확인 필요).

**5) OpenDRIVE `<object>` 요소 렌더링 여부 — 확인: 안 됨**

A-2에서 인용한 것과 동일 문서(adv_opendrive)의 "besides that,
there will be only void" — object 요소를 포함한 도로 외
모든 것이 렌더링 안 됨. 이 문서에 object 요소를 별도로 언급하는
문장 자체가 없음(전수조사 결과 무언급) → object 태그를 xodr에
넣어도 스탠드얼론 모드에서 그려질 근거 없음. static.prop.mesh
파이썬 소환 방식이 유일한 경로로 판단.

**6) ImportAssets.sh — 실측**

내용 전문(11줄): `Import/` 폴더의 `*.tar.gz`를 전부 압축 해제할
뿐. 로컬 `Import/` 폴더는 비어 있음(전수조사). 이 스크립트만으로
새 `.fbx`를 패키지에 넣을 수 없음 — tar.gz 자체가 이미 UE로
쿠킹된 콘텐츠여야 하고, 그 쿠킹 과정은 소스빌드 워크플로우의
`make import`(공식 문서, 오늘 세션 범위 밖이라 원문 재확인
안 함)에서 나오는 것으로 알려짐. **결론: 이 노트북(소스빌드
불가 환경, 세션 배경에서 이미 확정)에서는 이 경로로 새 자산을
넣을 수 없음. 기존 콘텐츠 재사용(static.prop.mesh)만 현실적
경로.**

### A-3 종합 판정

| 항목 | 상태 |
|---|---|
| static.prop.mesh 블루프린트 | 확인됨(존재) |
| 크기 조절 | 미확인(공식문서 무언급, 실측 필요) |
| 메시 경로 확인 방법(에디터 불필요) | 확인됨(파일시스템 대조) |
| 건물류 기존 메시 | 확인됨(House/Apartment/Mansion/Skyscraper 등) |
| 나무 기존 메시 | 확인됨(167종) |
| 가로등 기존 메시 | 존재하나 BP_ 접두 — 소환 방식 미확인 |
| OpenDRIVE object 렌더링 | 확인됨(안 됨) |
| 새 자산(.fbx) 추가 | 확인됨(이 환경에서 불가) |

**실제 소환 성공 여부는 미확인.** 경로·문법은 문서와 로컬
파일구조 대조로 확인됐으나, CARLA 서버에 실제로 spawn_actor가
성공하는지는 Phase E 1회 시험 전까지 미확정.

## Phase B 부가조사 — 후보 1/41/48 상세 (사용자 지시, 2026-09-18)

### 좌표 산출 방법 변경 (지시와 다르게 처리, 이유 명시)

지시는 geo_calibrate 역변환(Umeyama) 사용이었으나, 이 세 후보는
아직 xodr로 변환되지 않은 OSM 원본 단계(`data/processed/
cbnu_campus_fixed.osm`)라 원본 위경도를 그대로 갖고 있음. Umeyama
역변환은 근사치(잔차 RMS2.68m/최대6.01m, 세션3 A-1)라, 이미
정확한 위경도가 있는 데이터에 적용하면 오차만 추가됨. **원본
위경도를 오차0으로 그대로 사용.**

### 시작/중간/끝점 위경도 (소수점6자리)

**번호1 (way392632034, service/parking_aisle, 104.5m)**
- 시작: 36.624701, 127.463656 (node 3958352335)
- 중간: 36.624756, 127.463339 (node 4727599759)
- 끝: 36.624909, 127.462514 (node 4496854212)

**번호41 (way442709226, service/parking_aisle, 218.9m)**
- 시작: 36.626260, 127.459975 (node 4403766754)
- 중간: 36.627004, 127.461084 (node 4403766758)
- 끝: 36.626843, 127.461894 (node 4403766761)

**번호48 (way452870644, service/parking_aisle, 52.0m)**
- 시작: 36.624909, 127.462514 (node 4496854212)
- 중간: 36.624955, 127.462293 (node 3958352338)
- 끝: 36.625092, 127.461983 (node 4744097595)

(번호1의 끝점과 번호48의 시작점이 같은 node4496854212 —
두 후보가 직접 이어져 있음, 우연 아님)

### 60m 반경 전수 조사 (중간점 기준, node+way 전부)

**번호1: 20건.** 이름있는 building은 없음(nameless building=yes
1건, 53.2m). 대신: 자전거도로·횡단보도·인도(footway류) 다수,
과속방지턱(traffic_calming), **외부 간선도로명**(1순환로
37.8m·57.8m, 모충로 53.5m), **버스정류장 name=충북대학교병원**
(58.0m). → 표의 "인접건물명" 빈칸 사유는 "건물 자체가 없어서"가
아니라 "이름있는 building 태그가 없어서"이고, 실제로는 병원
버스정류장·외부간선도로 접점이라는 정보가 있었음(표 설계가
building만 봐서 놓침).

**번호41: 4건뿐(가장 한적한 구역).** 진짜로 60m 안에 건물이
없음(0건). 대신 **leisure=pitch, name=스포츠센터**(42.2m,
node12458854373) — 운동장. → "다른 종류의 지물만 있어서"
케이스. barrier=cycle_barrier(30.4m)도 있음.

**번호48: 17건.** building 없음. 대신 **amenity=hospital,
name=교육인재관 H13**(55.6m, node11911748888) — **노드**에
amenity+name 태그가 있는 경우. → **표 생성 스크립트
(gen_candidates_table.py)의 설계 결함 확인**: `nearest_building()`
함수가 "building 태그+name 태그가 같이 있는 way"만 검색하고
node는 아예 순회 안 함 — 이 지물이 60m 반경(55.6m) 안에 있는데도
"-"로 나온 것은 데이터 부재가 아니라 **검색 범위 설계 누락**.
나머지는 전부 highway=service/footway(횡단보도·과속방지턱 포함).

### 종합 판정

64건 표의 "인접건물명 -" 63건 중, 이번에 확인한 3건 기준으로는
1건(41번)만 "진짜로 60m 안에 이름있는 지물이 드묾"(운동장 1개
뿐)이고, 2건(1번·48번)은 **표 생성 스크립트가 building+name way만
찾고 amenity/leisure/버스정류장 등 이름있는 node를 놓쳐서** 빈칸이
된 경우. 나머지 61건에도 같은 설계 누락이 적용됐을 가능성 높음 —
표 전체를 이 기준(building 외 amenity/leisure/name있는 node 포함)
으로 재생성할지는 사용자 결정 필요(다음 턴에 제안).

## 검산 불일치 (Phase C 진입 전, 사용자 지시로 확인)

지시서의 "채택52건/제외12건" 표기를 map/docs/road_candidates.md와
python 검산으로 직접 대조 → **실제 채택54건/제외10건**(합64는
일치, 번호 누락·중복 없음, road_candidates.md와도 일치). 사용자
확인: 개수 라벨만 오기, 번호 목록 자체는 정확. Phase C 대상을
기존19+신규54=**73개**로 정정(지시서 원문 71개 아님). 상세 근거는
docs/setup_log.md 2026-09-18 기록.

## Phase C — 태그 보정·재변환·검증

### C-1. fix_tags.py 대상 갱신 (73개)

`map/scripts/fix_tags.py` 수정: `TARGET_WAY_IDS_NEW_CANDIDATES`
(54개, road_candidates.md 채택번호 → way id 기계적 추출) 추가,
`TARGET_WAY_IDS = LENGTH_BASED(16) + ROUTE_BASED(3) + NEW(54)`.
원본(`data/raw/cbnu_campus.osm`) 미수정. 출력을 새 파일명
`cbnu_campus_fixed_tags73.osm`으로 분리해 기존 19건판
(`cbnu_campus_fixed.osm`)을 대조군으로 보존.

실행 결과: 73건 전부 매칭·보정(찾지 못한 id 0건).

### C-2. 차선 폭 개별화 가능성 — 가능함(단, 이번 세션엔 미적용)

CARLA 0.9.15의 Osm2Odr은 `Util/BuildTools/BuildOSM2ODR.sh`(0.9.15
태그, 원문 확인)가 지정하는 `carla-simulator/sumo.git` 포크,
고정커밋 `1835e1e9538d0778971acc8b19b111834aae7261`
(브랜치 `aaron/defaultsidewalkwidth`)의 SUMO netconvert
코드베이스를 그대로 사용함을 공식 빌드스크립트 원문으로 확인.

그 커밋의 `src/netimport/NIImporter_OpenStreetMap.cpp` 원문
직접 확인 결과:
- 일반 `width` 태그는 way 단위로 **파싱되지 않음**(타입맵 병합
  로직에서만 쓰이고, OSM way 태그값으로는 무시됨)
- `width:lanes` / `width:lanes:forward` / `width:lanes:backward`
  태그는 **way마다 개별 파싱**되어(`|`구분 리스트) 차선별
  `NBEdge::setLaneWidth()`로 반영됨(1377~1393행)
- `src/netwrite/NWWriter_OpenDrive.cpp` 원문에서 그 폭 값이
  그대로 xodr `<width>` 요소에 쓰이는 것도 확인(544/739행,
  `e->getLaneWidth(j)`)

**결론: 메커니즘은 가능**(OSM 원본 way에 `width:lanes` 태그를
넣으면 Osm2Odr이 읽어 차선별 폭을 xodr에 반영한다). 단, 이번
세션은 **적용하지 않음** — 실측 근거가 없음(원본 OSM에 애초
width류 태그가 전혀 없고, 도로별 실폭 측량 데이터도 없음).
값을 임의로 지어넣는 것은 근거 없는 데이터 조작이라 판단.
**단일값(3.35m) 유지, 값 자체는 재검토했으나 변경 근거 없어
그대로 둠**(3.35m = CARLA 0.9.15 런타임 실측 기본값, 세션3
확정 근거 그대로 유효). 차선 폭 개별화는 실측 데이터 확보 후
재추진 가능한 항목으로 남김.

### C-3. 인도 생성 — 불필요 (재확인 없음, 기존 결론 유지)

Phase A-2에서 86/134 sidewalk lane 이미 확인됨. 추가 조사 없음.

### C-4. 파이프라인 재실행

스크립트 3개 전부 존재 확인 후 진행(세션3 유실 사고 재발 없음):
`fix_tags.py` → `classify_internal.py` → `osm_to_xodr.py`
(지시서 문구 순서 "485추출→태그보정→변환"과 달리, 실제 스크립트
의존관계상 태그보정이 485추출보다 먼저 — classify_internal.py의
입력이 태그보정 결과물이기 때문. 순서는 기존 파이프라인 설계
그대로, 변경 없음).

실행 결과:
1. `fix_tags.py`: 73건 보정 →
   `data/processed/cbnu_campus_fixed_tags73.osm`
2. `classify_internal.py`: internal447/boundary38/external664
   (기록값과 완전 일치) → 485way 추출 →
   `data/processed/cbnu_internal_boundary_tags73.osm`
3. `osm_to_xodr.py` (`.venv-carla` 파이썬으로 실행, carla 모듈은
   프로젝트 venv 안에만 있음): 정상 변환, 출력크기 2,069,577
   bytes → `maps/cbnu_internal_only_localtm_tags73.xodr`

**대조군 보존 확인**: 기존 `cbnu_internal_only_localtm.xodr`
(440,218 bytes) mtime·크기 불변 확인.

stderr 캡처(세션3에서 고친 기능, 이번에 처음 자동 적용됨) 요약:
`Could not compute smooth shape` 10건, `Not joining junctions`
8건, `Found sharp turn` 2건, `Found angle` 2건 등 — proj.db
경고는 기존과 동일하게 여전히 남음(미해결 변수, 기존 기록과 동일).

### C-5. 검증

`map/scripts/verify_session4.py` 작성·실행(신규 스크립트,
analyze_geometry.py의 parse_roads/parse_junctions/analyze_junction
함수를 import해 재사용). 결과 전문:
`map/docs/map_session04_verification.md`.

**1) road/junction 개수**
- 기존(19건판): road 134 / junction 25
- 신규(73건판): **road 878 / junction 95**
- 판단: 급증이나 원인 설명됨 — 54건이 `highway=unclassified`로
  승격되며 기존에는 단순 통과 취급되던 교차점들이 전부 정식
  junction으로 분리된 것으로 추정(osm2odr가 unclassified를
  service/footway보다 훨씬 적극적으로 junction 처리한다는
  간접 증거). 근본 원인(osm2odr 내부 로직)까지는 미확인.

**2) 정문-도서관 경로**
- 존재 여부: 기존/신규 둘 다 존재(끊김 없음)
- 길이: 거리가중 최단경로(Dijkstra, road pred/succ elementType=
  road 링크만 사용, junction 슈퍼노드로 뭉개지 않는 방식 — 세션3
  Phase C 방식 재구현)로 기존/신규 둘 다 재계산한 결과 **양쪽
  다 529.86m로 완전 일치**(기존 5road → 신규 15road로 세분화
  됐으나 물리적 경로 총길이는 보존됨)
- **주의**: 세션3 기록값 539.75m/6road와는 다름(약 9.89m 차이).
  원인: 세션3의 원 그래프탐색 스크립트가 저장되지 않아(유실,
  setup_log.md 2026-09-17 기록) 그 알고리즘을 정확히 재현할 수
  없음 — 오늘 것은 독립적으로 새로 구현한 Dijkstra 최단경로.
  절대값 539.75m과의 정합성은 검증 불가, 대신 "동일 방법을
  기존판/신규판에 똑같이 적용했을 때 일치하는가"로 대체 검증함
  (그 결과는 완전 일치 — 신뢰할 수 있는 상대 비교)

**3) junction 쌍 거리 비율 (축척 유지 확인)**
- 이름(OSM node id) 공통 junction 15개, 105개 쌍 비교
- 비율(신규/기존) min 0.9633 / median 1.0002 / mean 1.0018 /
  max 1.1186 — **median·mean은 1.000 근처로 축척 유지 확인**.
  max 1.1186(11.86% 차이) 1건은 이상치, 어느 쌍인지는 미조사
  (다음 단계 필요시 추적)

**4) road id 변경 여부**
- road name 속성이 전부 빈 문자열이라 원본 OSM way id 보존 안 됨
  → id 기준 1:1 대응표 작성 불가(대응 근거 자체가 없음)
- id 범위만 사실 기록: 기존 280~413(134개) / 신규
  1120~1997(878개) — 전면 재부여, 대응표는 작성하지 않음(근거 없음)

**5) 기하 이상 개수**
- 기존: connection 90건, gap이상(>=0.01m) 40건, 종단(along)
  이상 1건, heading_diff median 0.00° max 37.86°
- 신규: connection 618건, gap이상 131건, 종단이상 **1건(동일)**,
  heading_diff median 0.00° max **76.50°(신규 발생, 악화)**
- 판단: along(실제 진행방향 단절) 이상은 늘지 않음(1건 그대로) —
  실제 단절 증가 없음. gap이상 비율은 오히려 낮아짐(44%→21%,
  connection 수 대비 상대적으로 개선). 단 heading_diff 최댓값이
  37.86°→76.50°로 커진 신규 사례 1건이 새로 나타남 — 어느
  junction인지 미조사, Phase D 육안검증에서 우선 확인 대상으로
  남김.

### 남은 미해결/후속 항목
- junction쌍 거리비율 이상치(max1.1186) 원인 조사
- 세션3 정문-도서관 539.75m 원 산출 스크립트는 영구 유실 상태로
  확정(재현 시도 실패) — 앞으로는 verify_session4.py가 대체 표준

### C-5 이후 사용자 재확인 2건 (2026-09-18)

사용자가 road/junction "134/25로 동일"이라 이의 제기 → grep/stat
독립 재확인(스크립트 미경유) 4중으로 878/95 입증, 사용자 오독으로
확인·승인. netconvert 내부 소스 추가확인은 시간대비효용 낮다는
사용자 판단으로 중단 — junction승격 가설은 "추정"으로 명시 유지.

**정문-도서관 재검증**: 73건판에서도 경로 존재. 옛경로(539.75m)와
신규 다이크스트라 재계산(529.86m) 차이 9.89m는 정확히 road404
길이와 일치 — junction26의 기존 템플릿아티팩트 커넥터 경유 여부
차이일 뿐, 단절 아님. 신규판도 같은 방식 재계산시 **529.86m로
완전 일치**, 9개 junction 경유(옛4개→신규9개, 중간5개 신규 삽입:
3958352394/4402717711/4402717729/4402724104/4402724110).

**junction26 대응**: 이름(4748296088) 매칭으로 옛id26→신규id107
확정, heading_diff 37.8596507933935°로 **신구 완전 동일**(악화
없음). 신규 최댓값 76.4957°는 **별도 지점**(신규id20,
name=4397573144) — stderr경고("Reducing junction cluster
4397573143,4397573144,4397573186")와 매칭, 신규54건 중 번호5
(way442065044, 자연대5호관 인근)와 기존16건 중 way392632036의
교차점. **셔틀경로(15road/9junction) 위에 없음, 최단거리
402.59m로 무관 확인.**

작은 대응표(요청분만):
| 항목 | 옛 | 신규 |
|---|---|---|
| junction26(name4748296088) | id26 | id107 |
| 정문junction(2261340221) | id2 | id2 |
| 도서관junction(4402717742) | id11 | id51 |
| road402(4.292m) | 402 | 1914 |
| road405(2.969m) | 405 | 1917 |
| road404(9.892m,아티팩트) | 404 | 1916 |
| road300/319/299/318 | 4개 | 세분화(1:1 대응 없음, route단위로만 대응) |

좌표계 발견: 73건판이 osm2odr center_map 재중심으로 19건판 대비
(dx=86.5034,dy=70.5446) **평행이동**(회전·축척 없음). 오프셋
보정 후 road402/405가 new1914/1917과 오차0.0001m로 완전일치
확인 — 같은 물리도로임을 좌표로도 검증.

## Phase D — 오버레이 재생성 (2026-09-18)

`map/scripts/plot_overlay_expanded.py` 신규 작성(`plot_overlay.py`
사본, 원본은 19건판 재현용으로 무수정 보존). 주요 설계:

- geo_calibrate.fit_similarity를 **73건판 xodr 자신의 junction
  위치**로 매번 새로 적합(Umeyama) — 위에서 발견한 (86.50,70.54)
  평행이동을 수동 보정하지 않고, 애초에 파일마다 독립 적합하는
  기존 설계를 그대로 재사용해 우회(도로망·배경 모두 같은
  ref_lat/ref_lon로 변환되므로 자동으로 정합)
- 정합성 자체점검 추가: 정문 OSM위경도 위치와 도로망 junction2
  위치 거리 계산 → **2.321m(정상)**, 어긋남 없음 확인 후 저장
- 축척·범위: 19건판(옛)의 bbox+250m를 그대로 재사용해 신규
  플롯에 고정 적용(같은 x[-663.5,883.5]/y[-778.3,739.6]) —
  세션3 04_overlay.png와 나란히 비교 가능
- 경로: 신규 15-road(529.86m, junction9개 경유)를 빨강으로 표시
- 76.50° junction(신규id20)을 주황 X로 표시, "셔틀경로 아님" 라벨
- 건물 라벨 18개(20개 이내, road_candidates.md 등장순 + 사용자가
  강조한 농대권역·S17기숙사권역 큐레이션, 자연대5호관은 76.50°
  지점 맥락 확인용으로 포함) — 18/18 전부 매칭 성공

실행 결과: 배경 건물368개·보행로278개(옛판과 정확히 동일 개수,
같은 bbox 재사용 검증됨), 신규판 junction 매칭 90/95(회전0.065°/
스케일1.000709/잔차RMS3.732m, 옛판 회전0.155°/스케일1.001446/
RMS3.420m과 같은 수준), 정합성점검 2.321m 정상.

출력: `map/docs/figures/06_overlay_expanded.png`. 육안검증은
사용자 판단 대기(사용자 지시대로 이미지 제시 후 HARD STOP).

## 사용자 육안검증 결과 — 도서관 종점 위치 문제 발견 (2026-09-18)

사용자가 06_overlay_expanded.png 확인 중 파란 별(junction51) 위치가
실제 중앙도서관과 다름을 지적. 조사 결과 세션3부터 있던 문제로
확인(Phase C와 무관). 상세는 setup_log.md 참고. 도서관 관련 OSM
지물 전수(amenity=library 4건): 중앙도서관 구관(way442066135,
N12), 신관(way648686778, N12, 구관과 노드공유), 과학기술도서관
(way452267231, S1-7, 다른 건물·자연대권역), 청주신율봉어린이도서관
(way494839488, 캠퍼스 밖·무관).

### 종점 후보 3안 비교 (읽기전용 조사만, 결정은 다음 세션)

| 안 | 도서관(구관)까지 거리 | 정문→종점 경로길이 | 비고 |
|---|---|---|---|
| **A. junction51(현재)** | 118.7m(도로망 기준)/116.8m(OSM 직접) | 529.86m | 세션3부터 사용 중. 선정근거 약함(반경150m내 5개 지물 중 최원거리) |
| **B. junction35** | 87.5m | 818.89m(+289m) | 도서관에 더 가까운 최근접 junction(95개 중 전체 1위 후보 아님, 5위 안 항목 중 최상위). 경로가 훨씬 돌아감 |
| **C. road1278 경유(junction 아님)** | 65.4m(전체 도로망 중 최근접) | 744.33m(+214m) | road1278(junction36→35, 138.045m)의 76.46m 지점(junction36쪽 기준). 현재경로와 첫5road(1247/1914/1356/1917/1355) 공유, junction107(옛junction26)에서 분기 |

**미확인 항목(다음 세션 조사 필요)**:
- C안의 정확한 CARLA/Autoware 적용 방법 — 셔틀 종점이 반드시
  junction(교차로)이어야 하는지, 아니면 도로 위 임의 좌표(waypoint)를
  목표로 잡을 수 있는지 미조사(사용자 질문, 답변 보류)
- B/C안 채택 시 오버레이(06번 유사) 재생성 필요
- A/B/C 전부 "건물 정문/출입구"가 아니라 "건물 중심(centroid)"
  기준 거리 — 실제 도서관 출입구 위치는 별도 확인 안 됨
- junction107(옛junction26) 분기점이 두 후보 경로(현재/C안) 공통
  경유지인데, 이 junction 자체가 이미 알려진 기하결함
  (heading_diff 37.86°, 템플릿아티팩트 커넥터) 지점 — C안 채택 시
  이 문제도 함께 검토 필요

### 다음 세션 시작점
1. 셔틀 종점 확정 — road1278 조사 결과 기반, junction 제약 해제
   여부(CARLA/Autoware waypoint 지정 가능성) 확인 포함
2. 종점 변경 시 오버레이 재생성(06번 유사 스크립트 재사용)
3. Phase E: CARLA 로드·주행·소품 시험(878 road, RSS/gtt_used
   측정 반드시 포함 — 세션3 누락 재발방지)
4. 미결 항목: 번호49(footway 191.2m) 정체 미확인, 24-road
   컴포넌트(S계열) 연결 조사, junction20(76.50°) 원인 추가조사
   여부

[HARD STOP] 맵세션4 마감.
