#!/usr/bin/env python3
# 10/09 2번: stops_v3.yaml (v3 동결본 정류장). stops_v3_draft_s46.yaml 의 값(road·s·좌표·위경도)은 그대로 두고
# 머리말·note·source 만 10/09 규칙(평서문, 기록 이름·세션 번호 없이, source 는 만든 스크립트·OSM 근거)으로 정리한다
# 입력 data/stops_v3_draft_s46.yaml(읽기만), 출력 data/stops_v3.yaml
import hashlib
import re
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
src = (BASE / "data/stops_v3_draft_s46.yaml").read_text()
sha16 = hashlib.sha256((BASE / "maps/cbnu_campus_frozen_v3.xodr").read_bytes()).hexdigest()[:16]
assert sha16 == "a17ab012e04c664f"
hdr_old = re.match(r"(#[^\n]*\n)+", src).group(0)
hdr = ("# 충북대 캠퍼스 맵 v3 정류장과 차고지 후보다\n"
       "# 맵은 maps/cbnu_campus_frozen_v3.xodr 이다(sha256 앞 16자 a17ab012e04c664f)\n"
       "# 정류장마다 승강장(platforms)을 둔다. 양방향 길 위 정류장은 건너편에도 승강장이 있다. 도서관 북문은 막다른 끝이라 하나다\n"
       "# 건너편 승강장은 같은 자리 옆 15m 안 반대 방향 차로 중 가장 가까운 곳이다. 도서관 남문만 보행로와 겹치는 차로를 피해 같은 길 반대쪽을 골랐다\n"
       "# road/s/lane 은 OpenDRIVE 좌표다. lane -1 은 진행 방향 오른쪽 주행 차선이다\n"
       "# 위경도는 WGS84 다. xodr 머리말의 투영과 offset 으로 역산했다(s22_buildings.Frame). CARLA GNSS 값과는 다르다\n"
       "# direction 은 진행 방위(북 0, 동 90)를 8방위로 적은 것이다\n"
       "# stop_zone_length_m 는 아직 정하지 않아 비워 두었다\n"
       "# 데이터 출처는 OpenStreetMap 이다. (c) OpenStreetMap contributors, ODbL 1.0\n"
       "# 좌표는 OSM 파생 지도와 OSM 노드에서 계산했다\n")
body = hdr + src[len(hdr_old):]
body = body.replace("map_sha256_16: a17ab012e04c664f", f"map_sha256_16: {sha16}")
REP = [
    ('note: "정문 게이트(경비실)를 지나 첫 사거리 전, 들어오는 차로 가운데쯤(사용자 질문 1 답)"',
     'note: "정문 게이트(경비실)를 지나 첫 사거리 전, 들어오는 차로 가운데쯤이다"'),
    ('source: "map/scripts/s43_verify.py [D], OSM 정문 노드 6394475285 ~ 사거리 노드 3958352394 들어오는 경로 가운데"',
     'source: "map/scripts/s43_verify.py, OSM 정문 노드 6394475285 와 사거리 노드 3958352394 사이 들어오는 경로의 가운데"'),
    ('note: "세션46 사용자 결정(후보 1): 막다른 끝 대신 j49 앞 관통 도로로 옮김. 문 위치는 추정(보고서 map_session46_report.md 1절, 추정점 2개)"',
     'note: "중문 길이 갈라지는 교차로 앞 관통 도로다. 예전 막다른 끝 정류장은 previous_end_stop 에 남겼다. OSM 에 문 지물이 없어 문까지 거리는 추정이다"'),
    ('source: "map/scripts/s46_middle_choice.py 후보 1, s46_stops.py"',
     'source: "map/scripts/s46_middle_choice.py, s46_stops.py, OSM way 442595034"'),
    ('note: "기존 정류장 자리"', 'note: "정류장 기준 자리다"'),
    ('note: "나가는 쪽 차로(게이트 방향)"', 'note: "게이트로 나가는 쪽 차로다"'),
    ('note: "같은 길 반대쪽 반쪽 도로(r1235 는 보행로 way 468616604 와 0.74m 겹쳐 뺌)"',
     'note: "같은 길 반대쪽 차로다. 더 가까운 r1235 는 보행로 way 468616604 와 0.74m 겹쳐 쓰지 않았다"'),
    ('note: "중문 길(446440351)이 갈라지는 j49 앞 관통 도로(way 442595034), 미술관(N16-2) 북동쪽 7m"',
     'note: "중문 길(way 446440351)이 갈라지는 교차로 앞 관통 도로(way 442595034)이고, 미술관(N16-2) 북동쪽 7m 다"'),
    ('note: "같은 길 반대쪽 차로"', 'note: "같은 길 반대쪽 차로다"'),
    ('    previous_end_stop:  # 세션44까지의 중문 정류장(막다른 끝 5.7m 앞, 승강장 하나). 정문 -> 중문 회귀 비교용(test_drive.py middle 노선 종점)',
     '    previous_end_stop:  # 예전 중문 정류장(막다른 끝 5.7m 앞). test_drive.py middle 노선 종점이고 회귀 비교에 쓴다'),
    ('    platforms:  # 세션45 승강장. side 이쪽 = 위 road_id·s 와 같은 자리', '    platforms:  # side 이쪽 = 위 road_id·s 와 같은 자리'),
    ('    platforms:  # 세션46 승강장. side 이쪽 = 위 road_id·s 와 같은 자리', '    platforms:  # side 이쪽 = 위 road_id·s 와 같은 자리'),
]
for a, b in REP:
    n = body.count(a)
    assert n >= 1, a[:40]
    body = body.replace(a, b)
assert "세션" not in body and "보고서" not in body, [l for l in body.splitlines() if "세션" in l or "보고서" in l]
(BASE / "data/stops_v3.yaml").write_text(body)
print("출력 data/stops_v3.yaml")
