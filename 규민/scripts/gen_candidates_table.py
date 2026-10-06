#!/usr/bin/env python3
# Phase B-3: road_candidates.py의 64건에 번호/wayid/태그/길이/
# 기존망연결여부/인접건물명을 표로 정리해 map/docs/road_candidates.md에
# 쓴다. plot_candidates.py와 번호가 같은 road_candidates.get_candidates()를
# 그대로 재사용하므로 그림-표 번호가 항상 일치한다. 읽기전용 입력,
# 문서 1개만 생성.
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from road_candidates import get_candidates
from nearby_osm import load as load_nearby, dist_m, OSM_RAW_PATH

OUT_PATH = Path(__file__).resolve().parent.parent / "docs" / "road_candidates.md"
BUILDING_RADIUS_M = 60


def _is_named_feature(tags):
    # 버그 수정(맵세션4, 2026-09-18): 기존엔 building+name way만 봐서
    # amenity/leisure를 가진 node(예: 버스정류장, 병원 지물)를 놓쳤다.
    # 번호48 사례 — 55.6m 거리의 amenity=hospital,name=교육인재관 H13
    # (node 11911748888)이 표에서 "-"로 나온 원인. setup_log.md 기록.
    return "name" in tags and (
        "building" in tags or "amenity" in tags or "leisure" in tags
    )


def nearest_building(mid_lat, mid_lon, nodes, ways):
    best = None
    for way in ways:
        tags = way["tags"]
        if not _is_named_feature(tags):
            continue
        pts = [nodes[n][:2] for n in way["nds"] if n in nodes]
        if not pts:
            continue
        cy = sum(p[0] for p in pts) / len(pts)
        cx = sum(p[1] for p in pts) / len(pts)
        d = dist_m(mid_lat, mid_lon, cy, cx)
        if d <= BUILDING_RADIUS_M and (best is None or d < best[0]):
            best = (d, tags["name"])
    for nid, ndata in nodes.items():
        tags = ndata[2] if len(ndata) > 2 else {}
        if not _is_named_feature(tags):
            continue
        ny, nx = ndata[0], ndata[1]
        d = dist_m(mid_lat, mid_lon, ny, nx)
        if d <= BUILDING_RADIUS_M and (best is None or d < best[0]):
            best = (d, tags["name"])
    return best


def main():
    candidates, nodes = get_candidates()
    nb_nodes, nb_ways = load_nearby(OSM_RAW_PATH)

    lines = []
    lines.append("# 도로 후보 목록 (Phase B, 맵세션4)")
    lines.append("")
    lines.append(f"필터: highway=steps 제외, width<2.5m 제외, 기존19way와 "
                  f"직접 노드공유 안하면 제외, 길이<50m 제외. 임계값 조정 없이 "
                  f"확정(사용자 승인, 2026-09-18) — 총 {len(candidates)}건.")
    lines.append("")
    lines.append("그림: `map/docs/figures/05a~05d_candidates_*.png` "
                  "(번호가 이 표와 동일)")
    lines.append("")
    lines.append("| 번호 | way id | 태그 | 길이(m) | 기존망직접연결 | 인접건물명(60m내) |")
    lines.append("|---|---|---|---|---|---|")

    for c in candidates:
        mid_nid = c["nds"][len(c["nds"]) // 2]
        mid_lat = float(nodes[mid_nid]["lat"])
        mid_lon = float(nodes[mid_nid]["lon"])
        nb = nearest_building(mid_lat, mid_lon, nb_nodes, nb_ways)
        nb_str = f"{nb[1]}({nb[0]:.0f}m)" if nb else "-"
        tag = c["highway"]
        if c["service"]:
            tag += f"(service={c['service']})"
        name = c["name"] or "-"
        lines.append(f"| {c['number']} | {c['id']} | {tag} | "
                      f"{c['length']:.1f} | 예 | {nb_str} |")

    OUT_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"저장: {OUT_PATH} ({len(candidates)}행)")


if __name__ == "__main__":
    main()
