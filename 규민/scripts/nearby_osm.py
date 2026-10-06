#!/usr/bin/env python3
# 질의점 위경도 근처의 OSM 지물(건물/도로명) 조회. 읽기전용, osm 파일 미수정.
# data/raw/cbnu_campus.osm 안의 node(건물 포인트, amenity 등)와
# way(building, highway name)를 대상으로 질의점까지의 거리를 계산해
# 가까운 순으로 출력한다. 거리 계산은 geo_calibrate.py와 동일한
# 상수(111,320m/도, 위도로 cos보정)를 재사용한다.
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

OSM_RAW_PATH = Path(__file__).resolve().parent.parent / "data" / "raw" / "cbnu_campus.osm"
METERS_PER_DEG_LAT = 111320

QUERY_POINTS = [
    ("junction1(4748296080, 강조구간 자유단)", 36.631513, 127.453602),
    ("junction11(4402717742, 강조구간 자유단)", 36.629468, 127.457141),
    ("북쪽 고립덩어리 중심", 36.624250, 127.462499),
    ("도로망 전체 중심", 36.627298, 127.456394),
]

TOP_N = 5
RADIUS_M = 150


def dist_m(lat1, lon1, lat2, lon2):
    ref_lat = (lat1 + lat2) / 2
    dlat = (lat2 - lat1) * METERS_PER_DEG_LAT
    dlon = (lon2 - lon1) * METERS_PER_DEG_LAT * math.cos(math.radians(ref_lat))
    return math.hypot(dlat, dlon)


def load(path):
    nodes = {}   # id -> (lat, lon, tags)
    ways = []    # (id, [node_ids], tags)
    way_nodes = {}
    cur_way = None
    for event, elem in ET.iterparse(path, events=("start", "end")):
        if event == "start" and elem.tag == "way":
            cur_way = {"id": elem.get("id"), "nds": [], "tags": {}}
        elif event == "end" and elem.tag == "node":
            tags = {c.get("k"): c.get("v") for c in elem if c.tag == "tag"}
            nodes[elem.get("id")] = (float(elem.get("lat")), float(elem.get("lon")), tags)
        elif event == "end" and elem.tag == "nd" and cur_way is not None:
            cur_way["nds"].append(elem.get("ref"))
        elif event == "end" and elem.tag == "tag" and cur_way is not None:
            cur_way["tags"][elem.get("k")] = elem.get("v")
        elif event == "end" and elem.tag == "way":
            ways.append(cur_way)
            cur_way = None
            elem.clear()
    return nodes, ways


def way_centroid(way, nodes):
    pts = [nodes[n][:2] for n in way["nds"] if n in nodes]
    if not pts:
        return None
    return (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))


def main():
    nodes, ways = load(OSM_RAW_PATH)
    print(f"로드: node {len(nodes)} / way {len(ways)}")

    for label, qlat, qlon in QUERY_POINTS:
        print(f"\n=== {label} ===")
        print(f"질의 위경도: {qlat:.6f}, {qlon:.6f}")
        candidates = []

        for nid, (lat, lon, tags) in nodes.items():
            if not tags:
                continue
            d = dist_m(qlat, qlon, lat, lon)
            if d <= RADIUS_M:
                candidates.append((d, "node", nid, tags))

        for way in ways:
            if not way["tags"]:
                continue
            c = way_centroid(way, nodes)
            if c is None:
                continue
            d = dist_m(qlat, qlon, c[0], c[1])
            if d <= RADIUS_M:
                candidates.append((d, "way", way["id"], way["tags"]))

        candidates.sort(key=lambda x: x[0])
        if not candidates:
            print(f"  반경{RADIUS_M}m 내 태그있는 지물 없음")
            continue
        for d, kind, oid, tags in candidates[:TOP_N]:
            tag_str = ", ".join(f"{k}={v}" for k, v in tags.items() if k in
                                 ("name", "building", "amenity", "highway", "name:en"))
            if not tag_str:
                tag_str = ", ".join(f"{k}={v}" for k, v in list(tags.items())[:3])
            print(f"  {d:6.1f}m {kind} {oid}: {tag_str}")


if __name__ == "__main__":
    main()
