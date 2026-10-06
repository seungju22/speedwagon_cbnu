#!/usr/bin/env python3
# Phase B-1 필터 기준 설계를 위한 읽기전용 통계 스크립트.
# data/processed/cbnu_internal_boundary.osm(485way, 19way는 이미
# highway=unclassified로 보정됨)에서 19way를 제외한 466way 후보군의
# highway 태그·길이·기존 도로망(19way) 연결 여부를 집계한다.
# 파일을 생성하지 않는다(화면 출력만).
import sys
from math import radians, sin, cos, asin, sqrt
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import classify_internal as ci
from fix_tags import TARGET_WAY_IDS

BASE_DIR = Path(__file__).resolve().parent.parent
INPUT_OSM = BASE_DIR / "data" / "processed" / "cbnu_campus_fixed.osm"
POLY_PATH = BASE_DIR / "data" / "processed" / "cbnu_relation_polygon.json"


def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p1, p2 = radians(lat1), radians(lat2)
    dphi = radians(lat2 - lat1)
    dlambda = radians(lon2 - lon1)
    a = sin(dphi / 2) ** 2 + cos(p1) * cos(p2) * sin(dlambda / 2) ** 2
    return 2 * R * asin(sqrt(a))


def way_length_m(way, nodes):
    total = 0.0
    nds = way["nds"]
    for a, b in zip(nds, nds[1:]):
        if a not in nodes or b not in nodes:
            continue
        na, nb = nodes[a], nodes[b]
        total += haversine_m(float(na["lat"]), float(na["lon"]),
                              float(nb["lat"]), float(nb["lon"]))
    return total


def main():
    rings = ci.load_polygon_rings(POLY_PATH)
    nodes, ways = ci.load_osm(INPUT_OSM)
    classification = ci.classify(nodes, ways, rings)

    ways_by_id = {w["id"]: w for w in ways}
    existing_ids = set(TARGET_WAY_IDS)
    existing_nodes = set()
    for wid in existing_ids:
        w = ways_by_id.get(wid)
        if w:
            existing_nodes.update(w["nds"])

    candidates = []
    for wid, cls in classification.items():
        if cls == "external":
            continue
        if wid in existing_ids:
            continue
        w = ways_by_id[wid]
        highway = w["tags"].get("highway")
        service = w["tags"].get("service")
        length = way_length_m(w, nodes)
        touches = bool(set(w["nds"]) & existing_nodes)
        candidates.append({
            "id": wid, "highway": highway, "service": service,
            "length": length, "cls": cls, "touches_existing": touches,
            "name": w["tags"].get("name"),
            "width": w["tags"].get("width"),
        })

    print(f"후보 총수(19way 제외, external 제외): {len(candidates)}")

    from collections import Counter
    hw_counter = Counter(c["highway"] for c in candidates)
    print("\nhighway별 개수:")
    for k, v in hw_counter.most_common():
        print(f"  {k}: {v}")

    print(f"\ncls별: internal={sum(1 for c in candidates if c['cls']=='internal')} "
          f"boundary={sum(1 for c in candidates if c['cls']=='boundary')}")

    print(f"\n기존 19way와 직접 노드 공유(touches_existing=True): "
          f"{sum(1 for c in candidates if c['touches_existing'])}")

    steps = [c for c in candidates if c["highway"] == "steps"]
    print(f"\nsteps(계단, 차도 불가): {len(steps)}건")

    lengths = sorted(c["length"] for c in candidates)
    import statistics
    print(f"\n길이 분포: min={lengths[0]:.1f} max={lengths[-1]:.1f} "
          f"median={statistics.median(lengths):.1f}")
    for thresh in (10, 20, 30, 50, 100, 150, 200):
        n = sum(1 for l in lengths if l < thresh)
        print(f"  길이<{thresh}m: {n}건")

    width_tagged = [c for c in candidates if c["width"]]
    print(f"\nwidth 태그 있는 후보: {len(width_tagged)}건")
    for c in width_tagged:
        print(f"  way{c['id']}: width={c['width']} highway={c['highway']}")

    parking_aisle = [c for c in candidates if c["service"] == "parking_aisle"]
    print(f"\nparking_aisle: {len(parking_aisle)}건 "
          f"(touches_existing={sum(1 for c in parking_aisle if c['touches_existing'])})")

    # 길이>=30m AND steps 아님 AND 기존망과 직접 연결
    prelim = [c for c in candidates
              if c["highway"] != "steps" and c["length"] >= 30
              and c["touches_existing"]]
    print(f"\n[예시 필터] steps제외 AND 길이>=30m AND 기존망 직접연결: "
          f"{len(prelim)}건")


if __name__ == "__main__":
    main()
