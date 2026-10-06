#!/usr/bin/env python3
# 캠퍼스 폴리곤(Nominatim relation 6705106) 기준으로
# data/processed/cbnu_campus_fixed.osm 안의 highway 태그 있는 way를
# internal(전부폴리곤안)/external(전부밖)/boundary(걸침)로 분류하고,
# internal+boundary way만 담은 사본(data/processed/cbnu_internal_boundary.osm)을
# 만든다. 읽기전용 입력, 새 파일만 생성. 원본(raw/cbnu_campus.osm,
# processed/cbnu_campus_fixed.osm) 미수정.
#
# 유실 경위: 이 판정 로직은 2026-09-15 세션에서 일회성으로 실행만 되고
# 스크립트로 저장되지 않아, cbnu_internal_only.xodr(134road/25junction)의
# 정확한 재현 경로가 끊겨 있었음. 2026-09-17 맵세션3에서 발견,
# 이 스크립트로 복원.
#
# 판정 기준 (osm_survey.md 원 서술과 동일하게 복원):
#   way를 이루는 "모든 node"의 위경도 각각에 대해 폴리곤 내부 여부를
#   판정한다(way의 한쪽 끝점이나 중심점 1개만 보는 게 아니라 전체 node).
#   - 전체 node가 내부 -> internal
#   - 전체 node가 외부 -> external
#   - 내부/외부가 섞임  -> boundary (캠퍼스 경계에 걸친 way, 정문
#     진입로 등)
# 내부 판정 자체는 even-odd(짝홀) 규칙을 "폴리곤의 50개 링 전체를
# 합쳐서" 적용한다: GeoJSON Polygon은 [외곽링, 홀1, 홀2, ...] 구조라
# 각 링에 대해 개별적으로 even-odd 판정(홀수 교차=내부)을 한 뒤
# 전체 결과를 XOR로 합친다 — 홀 안에 들어가면 다시 "외부" 판정으로
# 뒤집히는, 홀 있는 폴리곤의 표준적 처리 방식.
import json
import xml.etree.ElementTree as ET
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent.parent
POLY_PATH = BASE_DIR / "data" / "processed" / "cbnu_relation_polygon.json"
# 맵세션4 Phase C-1(2026-09-18): fix_tags.py 대상 73건 갱신에 맞춰
# 입출력을 새 파일명으로 전환. 기존 cbnu_campus_fixed.osm(19건판)과
# cbnu_internal_boundary.osm(485way, 대조군 xodr의 원천)은 손대지
# 않고 그대로 둔다(사본을 새로 만든다, C-1 지시).
INPUT_OSM = BASE_DIR / "data" / "processed" / "cbnu_campus_fixed_tags73.osm"
OUTPUT_OSM = BASE_DIR / "data" / "processed" / "cbnu_internal_boundary_tags73.osm"
LOG_DIR = BASE_DIR / "logs"

# osm_survey.md에 기록된 이전 판정 결과(검증 대조용). 근거:
# map/docs/osm_survey.md "## 캠퍼스 안/밖 분리" 절.
EXPECTED_INTERNAL = 447
EXPECTED_BOUNDARY = 38
EXPECTED_EXTERNAL = 664
EXPECTED_TOTAL = 1149  # highway 태그 있는 way 전체


def load_polygon_rings(path):
    data = json.loads(path.read_text())
    geojson = data[0]["geojson"]
    assert geojson["type"] == "Polygon", f"예상과 다른 geometry 타입: {geojson['type']}"
    # GeoJSON 좌표는 [lon, lat] 순서
    return geojson["coordinates"]  # [ring0(외곽), ring1..ringN(홀)]


def point_in_ring(lon, lat, ring):
    # 표준 ray-casting even-odd 판정(단일 링).
    inside = False
    n = len(ring)
    x, y = lon, lat
    for i in range(n):
        x1, y1 = ring[i]
        x2, y2 = ring[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            x_int = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < x_int:
                inside = not inside
    return inside


def point_in_polygon(lon, lat, rings):
    inside = False
    for ring in rings:
        if point_in_ring(lon, lat, ring):
            inside = not inside  # 홀 처리: 링을 지날 때마다 뒤집음
    return inside


def load_osm(path):
    nodes = {}
    ways = []
    cur = None
    for ev, elem in ET.iterparse(path, events=("start", "end")):
        if ev == "start" and elem.tag == "way":
            cur = {"id": elem.get("id"), "nds": [], "tags": {}}
        elif ev == "end" and elem.tag == "node":
            nodes[elem.get("id")] = {
                "lat": elem.get("lat"), "lon": elem.get("lon"),
                "tags": {c.get("k"): c.get("v") for c in elem if c.tag == "tag"},
            }
        elif ev == "end" and elem.tag == "nd" and cur is not None:
            cur["nds"].append(elem.get("ref"))
        elif ev == "end" and elem.tag == "tag" and cur is not None:
            cur["tags"][elem.get("k")] = elem.get("v")
        elif ev == "end" and elem.tag == "way":
            ways.append(cur)
            cur = None
            elem.clear()
    return nodes, ways


def classify(nodes, ways, rings):
    result = {}  # way_id -> "internal"|"boundary"|"external"
    cache = {}   # node_id -> bool(inside)
    for way in ways:
        if "highway" not in way["tags"]:
            continue
        flags = []
        for nid in way["nds"]:
            if nid not in nodes:
                continue
            if nid not in cache:
                n = nodes[nid]
                cache[nid] = point_in_polygon(float(n["lon"]), float(n["lat"]), rings)
            flags.append(cache[nid])
        if not flags:
            continue
        if all(flags):
            result[way["id"]] = "internal"
        elif not any(flags):
            result[way["id"]] = "external"
        else:
            result[way["id"]] = "boundary"
    return result


def write_subset_osm(nodes, ways, classification, output_path):
    keep_way_ids = {wid for wid, cls in classification.items() if cls in ("internal", "boundary")}
    keep_ways = [w for w in ways if w["id"] in keep_way_ids]
    keep_node_ids = set()
    for w in keep_ways:
        keep_node_ids.update(w["nds"])

    root = ET.Element("osm", version="0.6", generator="classify_internal.py")
    for nid in sorted(keep_node_ids, key=int):
        n = nodes[nid]
        node_el = ET.SubElement(root, "node", id=nid, lat=n["lat"], lon=n["lon"])
        for k, v in n["tags"].items():
            ET.SubElement(node_el, "tag", k=k, v=v)
    for w in keep_ways:
        way_el = ET.SubElement(root, "way", id=w["id"])
        for nid in w["nds"]:
            ET.SubElement(way_el, "nd", ref=nid)
        for k, v in w["tags"].items():
            ET.SubElement(way_el, "tag", k=k, v=v)

    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_path, encoding="UTF-8", xml_declaration=True)
    return len(keep_ways), len(keep_node_ids)


def main():
    rings = load_polygon_rings(POLY_PATH)
    print(f"폴리곤 링 수: {len(rings)} (osm_survey.md 기록: 50링)")

    nodes, ways = load_osm(INPUT_OSM)
    print(f"입력 OSM: node {len(nodes)} / way {len(ways)}")

    classification = classify(nodes, ways, rings)
    counts = {"internal": 0, "boundary": 0, "external": 0}
    for cls in classification.values():
        counts[cls] += 1
    total = sum(counts.values())

    print(f"\n분류 결과: internal={counts['internal']} "
          f"boundary={counts['boundary']} external={counts['external']} "
          f"합계={total}")
    print(f"대조 기록값: internal={EXPECTED_INTERNAL} "
          f"boundary={EXPECTED_BOUNDARY} external={EXPECTED_EXTERNAL} "
          f"합계={EXPECTED_TOTAL}")

    match = (counts["internal"] == EXPECTED_INTERNAL and
             counts["boundary"] == EXPECTED_BOUNDARY and
             counts["external"] == EXPECTED_EXTERNAL and
             total == EXPECTED_TOTAL)

    if not match:
        print("\n*** 검증 실패: 기록값과 불일치. 사본을 만들지 않고 중단합니다. ***")
        return 1

    print("\n검증 통과: 기록값과 완전히 일치.")
    n_ways, n_nodes = write_subset_osm(nodes, ways, classification, OUTPUT_OSM)
    print(f"사본 생성: {OUTPUT_OSM}")
    print(f"  포함 way(internal+boundary) 수: {n_ways} (기대값 485)")
    print(f"  포함 node 수: {n_nodes}")

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = LOG_DIR / "classify_internal_tags73.log"
    with open(log_path, "w") as f:
        f.write(f"입력: {INPUT_OSM}\n출력: {OUTPUT_OSM}\n")
        f.write(f"internal={counts['internal']} boundary={counts['boundary']} "
                f"external={counts['external']} 합계={total}\n")
        f.write(f"기록값과 일치: {match}\n")
        f.write(f"사본 way수={n_ways} node수={n_nodes}\n")
    print(f"로그: {log_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
