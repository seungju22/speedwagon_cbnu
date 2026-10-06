#!/usr/bin/env python3
# Phase B-1에서 승인된 1차 필터(steps 제외 / width<2.5m 제외 /
# 기존19way와 직접 노드공유 안하면 제외 / 길이<50m 제외)를 적용해
# 64건 후보 목록을 만든다. B-2(그림)·B-3(표)이 공통으로 이 모듈을
# import해서 쓴다(번호-way id 대응이 그림과 표에서 어긋나지 않도록).
# 읽기전용(파일 생성 없음, import되어 쓰이는 라이브러리 모듈).
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import classify_internal as ci
from fix_tags import TARGET_WAY_IDS
from analyze_road_candidates import way_length_m, INPUT_OSM, POLY_PATH

MIN_LENGTH_M = 50.0
MIN_WIDTH_M = 2.5


def _passes_width(tags):
    width = tags.get("width")
    if width is None:
        return True
    try:
        return float(width) >= MIN_WIDTH_M
    except ValueError:
        return True


def get_candidates():
    """번호(1부터, way id 오름차순 정렬로 고정) -> 후보 dict 리스트 반환.
    각 dict: number, id, highway, service, length, name, nds, cls
    """
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

    picked = []
    for wid, cls in classification.items():
        if cls == "external" or wid in existing_ids:
            continue
        w = ways_by_id[wid]
        tags = w["tags"]
        if tags.get("highway") == "steps":
            continue
        if not _passes_width(tags):
            continue
        if not (set(w["nds"]) & existing_nodes):
            continue
        length = way_length_m(w, nodes)
        if length < MIN_LENGTH_M:
            continue
        picked.append({
            "id": wid, "highway": tags.get("highway"),
            "service": tags.get("service"), "length": length,
            "name": tags.get("name"), "nds": w["nds"], "cls": cls,
        })

    picked.sort(key=lambda c: int(c["id"]))
    for i, c in enumerate(picked, start=1):
        c["number"] = i
    return picked, nodes


if __name__ == "__main__":
    candidates, _ = get_candidates()
    print(f"후보 수: {len(candidates)}")
    from collections import Counter
    print(Counter(c["highway"] for c in candidates))
