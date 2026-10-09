#!/usr/bin/env python3
# 맵세션40 3번(M5): 정류장 6지점·차고지 후보 2곳을 기존 노선 정의에서 모아 map/data/stops_v2.yaml 로 쓴다(읽기전용 입력, 서버 없음)
# 새로 정하지 않는다
#   정류장 = test_drive.py ROUTES_V2 의 출발점(정문, 실제 주행 로그 spawn road1278 lane-1 s=0.00)과 각 노선 종점(마지막 road, end_s, lane -1)
#   위경도 = s22_buildings.Frame(xodr 머리말 투영 + offset) 정변환을 뉴턴법으로 역산. 왕복 차를 같이 적는다
#   (carla.Map.transform_to_geolocation 은 쓰지 않는다: 세션40 시험에서 약 716m 어긋남 = xodr offset 미반영으로 보임 [추정])
#   캠퍼스 경계 = data/processed/cbnu_relation_polygon.json(OSM relation 6705106, classify_internal.py 와 같은 판정)
#   차고지 = 기록에 있는 것만. 박물관 버스 차고지: way 442595850(logs/s24_reconv_list.md 49~52행) 노드 평균 + E-j4 답사점
#            N14 주차장: 좌표 기록 없음 -> 빈 값
# 정차 구역 길이는 빈 값(지시문)
# 사용: .venv-carla/bin/python map/scripts/s40_stops.py
import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import carla

HERE = Path(__file__).resolve().parent
BASE = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(BASE / "tests"))
from classify_internal import load_polygon_rings, point_in_polygon  # noqa: E402
from s22_buildings import RAW_OSM, Frame  # noqa: E402

sys.argv = sys.argv[:1]
import test_drive as td  # noqa: E402

XODR = BASE / "maps/cbnu_campus_frozen_v2.xodr"
POLY = BASE / "data/processed/cbnu_relation_polygon.json"
OUT = BASE / "data/stops_v2.yaml"
START = (1278, -1, 0.0)             # logs/s38_v2_back_run1.log "spawn: ... road1278 lane-1 s=0.00" (find_plain_road_spawn_on_path)
STOPS = [("main_gate", "정문", "start", None), ("lib_north", "도서관 북문", "north", None),
         ("lib_south", "도서관 남문", "south", None), ("middle_gate", "중문", "middle", None),
         ("yangseong", "양성재", "yangseong", None), ("back_gate", "후문", "back", None)]
# 2026-10-09: note 는 정류장 위치를 한 줄 평서문으로 적는다(보고서 이름 뺌). 예전 문구는 test_drive.py ROUTES_V2 의 stop
NOTES = {
    "main_gate": "정문 바로 밖 공도 교차점으로, 모든 노선이 여기서 출발한다",
    "lib_north": "도서관 구관 북동쪽 면 앞 도로로, 면 중점에서 46.1m 떨어져 있다",
    "lib_south": "자연대와 도서관 사이 도로로, 도서관 남서쪽 면 중점에서 약 57.5m 떨어져 있다",
    "middle_gate": "중문 앞 도로로, road1327 막다른 끝 5.7m 전이고 문까지 약 40~80m 다",
    "yangseong": "양성재(서문) 앞 도로로, 건물 외곽선에서 45.7m 떨어져 있다",
    "back_gate": "OSM way 392632034 위 후문B 추정 게이트로, 기준점(36.624844, 127.462860)에서 1.65m 떨어져 있다",
}


def inverse(frame, x, y, lat=36.63, lon=127.46):
    """CARLA (x, y) -> 위경도. Frame.carla 를 수치 미분으로 뉴턴 반복"""
    for _ in range(20):
        fx, fy = frame.carla(lat, lon)
        ex, ey = x - fx, y - fy
        if math.hypot(ex, ey) < 1e-6:
            break
        d = 1e-6
        a = [(frame.carla(lat + d, lon)[i] - (fx, fy)[i]) / d for i in (0, 1)]
        b = [(frame.carla(lat, lon + d)[i] - (fx, fy)[i]) / d for i in (0, 1)]
        det = a[0] * b[1] - a[1] * b[0]
        lat += (ex * b[1] - ey * b[0]) / det
        lon += (a[0] * ey - a[1] * ex) / det
    return lat, lon


def main():
    text = XODR.read_text()
    sha = hashlib.sha256(text.encode()).hexdigest()[:16]
    cmap = carla.Map(XODR.stem, text)
    frame = Frame(text)
    rings = load_polygon_rings(POLY)
    routes = td.ROUTES_V2
    lines = ["# 정류장과 차고지 후보. map/scripts/s40_stops.py 가 기존 노선 정의에서 모아 만들었다. 새로 정한 값은 없다",
             f"# 맵은 maps/cbnu_campus_frozen_v2.xodr 이다(sha256 앞 16자 {sha})",
             "# road/s/lane 은 OpenDRIVE 좌표다. lane -1 은 진행 방향 오른쪽 주행 차선이다",
             "# 위경도는 WGS84 다. xodr 머리말의 투영과 offset 으로 역산했다(s22_buildings.Frame)",
             "# stop_zone_length_m 는 아직 정하지 않아 비워 두었다",
             "# 데이터 출처는 OpenStreetMap 이다. (c) OpenStreetMap contributors, ODbL 1.0",
             "# 좌표는 OSM 파생 지도와 OSM 노드에서 계산했다",
             f"map_sha256_16: {sha}", "stops:"]
    for key, name, route, _ in STOPS:
        if route == "start":
            rid, lane, s = START
            src = "tests/test_drive.py ROUTES_V2 첫 road(모든 노선 공통 출발)"
        else:
            r = routes[route]
            rid, lane, s = r["roads"][-1], -1, r["end_s"]
            src = f"tests/test_drive.py ROUTES_V2['{route}'] 마지막 road·end_s"
        wp = cmap.get_waypoint_xodr(rid, lane, s)
        loc = wp.transform.location
        glat, glon = inverse(frame, loc.x, loc.y)
        bx, by = frame.carla(glat, glon)
        rt = math.hypot(bx - loc.x, by - loc.y)
        inside = point_in_polygon(glon, glat, rings)
        lines += [f"  - id: {key}", f"    name: {name}", f"    route: {route if route != 'start' else 'all'}",
                  f"    road_id: {rid}", f"    s: {s}", f"    lane_id: {lane}",
                  f"    carla_xyz: [{loc.x:.3f}, {loc.y:.3f}, {loc.z:.3f}]",
                  f"    lat: {glat:.7f}", f"    lon: {glon:.7f}",
                  f"    latlon_roundtrip_m: {rt:.3f}",
                  f"    inside_campus_boundary: {'true' if inside else 'false'}",
                  "    stop_zone_length_m:",
                  f"    note: \"{NOTES[key]}\"", f"    source: \"{src}\""]
        print(f"{key} r{rid} s={s} lane {lane} ({glat:.7f},{glon:.7f}) 왕복차 {rt:.3f}m 경계안 {inside}")
    # 차고지
    nodes, way = {}, None
    for _, el in ET.iterparse(str(RAW_OSM)):
        if el.tag == "node":
            nodes[el.get("id")] = (float(el.get("lat")), float(el.get("lon")))
        elif el.tag == "way" and el.get("id") == "442595850":
            way = [n.get("ref") for n in el.findall("nd")]
    pts = [nodes[n] for n in dict.fromkeys(way)]       # 고리라 시작=끝 노드 중복 제거
    lat, lon = sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)
    lines += ["depot_candidates:  # 고르지 않음",
              "  - id: museum_bus_depot", "    name: 박물관 버스 차고지",
              f"    lat: {lat:.7f}", f"    lon: {lon:.7f}",
              f"    note: \"박물관 버스 차고지 주차 통로 고리(OSM way 442595850) 노드 {len(pts)}개의 평균 위치이고, 진입 답사점은 36.627598, 127.454728 이다\"",
              "    source: \"OSM way 442595850\"",
              "  - id: n14_parking", "    name: N14 주차장",
              "    lat:", "    lon:",
              "    note: \"N14 주차장은 건물을 ㄷ자로 감싼 형태로만 기록돼 있어 좌표를 비워 두었다\"",
              "    source: \"\""]
    print(f"museum_bus_depot way 442595850 노드 {len(pts)} 평균 ({lat:.7f},{lon:.7f}) / n14_parking 빈 값")
    OUT.write_text("\n".join(lines) + "\n")
    print(f"-> {OUT}")


if __name__ == "__main__":
    main()
