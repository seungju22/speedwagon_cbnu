#!/usr/bin/env python3
# 맵세션4 Phase C-5 검증. analyze_geometry.py의 파싱/분석 함수를
# import해 재사용(요청대로 "세션2 분석 스크립트 재사용"). 대조군
# cbnu_internal_only_localtm.xodr(19건판)과 신규
# cbnu_internal_only_localtm_tags73.xodr(73건판)을 둘 다 읽어
# 비교만 한다. 두 xodr 모두 읽기전용, 이 스크립트는 문서만 생성.
import statistics
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPTS_DIR))
import analyze_geometry as ag  # noqa: E402

BASE_DIR = SCRIPTS_DIR.parent
OLD_XODR = BASE_DIR / "maps" / "cbnu_internal_only_localtm.xodr"
NEW_XODR = BASE_DIR / "maps" / "cbnu_internal_only_localtm_tags73.xodr"
OUT_PATH = BASE_DIR / "docs" / "map_session04_verification.md"

GATE_OSM_ID = "2261340221"   # junction2(정문) 이름, plot_overlay.py 재사용
LIB_OSM_ID = "4402717742"    # junction11(도서관) 이름


def load(path):
    root = ET.parse(path).getroot()
    roads = ag.parse_roads(root)
    junctions = ag.parse_junctions(root)
    return roads, junctions


def junction_locations(roads, junctions):
    locs = {}
    for jid, junction in junctions.items():
        pts = []
        for conn in junction["connections"]:
            connecting = roads.get(conn["connectingRoad"])
            if connecting:
                pts.append(ag.road_end_state(connecting, "start")[:2])
        if pts:
            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
            locs[jid] = (cx, cy)
    return locs


def name_to_jid(junctions):
    m = {}
    for jid, j in junctions.items():
        if j["name"]:
            m[j["name"]] = jid
    return m


def build_road_graph(roads):
    # "road pred/succ elementType=road 링크만 union, junction
    # 슈퍼노드로 뭉개지 않음" — 세션3 Phase C 노선충분성 판단과
    # 동일 방식(setup_log.md 125행). connecting road(junction!=-1)의
    # pred/succ는 항상 elementType=road로 실제 도로를 직접 가리키므로
    # 이것만 모아도 base road<->connecting road<->base road 경로가
    # 끊기지 않고 이어진다.
    adj = {rid: set() for rid in roads}
    for rid, r in roads.items():
        for link in (r["pred"], r["succ"]):
            if link and link.get("elementType") == "road":
                other = link.get("elementId")
                if other in roads:
                    adj[rid].add(other)
                    adj[other].add(rid)
    return adj


def roads_touching_junction(roads, jid):
    out = set()
    for rid, r in roads.items():
        for link in (r["pred"], r["succ"]):
            if link and link.get("elementType") == "junction" and link.get("elementId") == jid:
                out.add(rid)
    return out


def dijkstra_shortest(adj, roads, starts, goals):
    # 거리가중 최단경로(hop수 최소가 아니라 실제 길이 최소).
    # "최단경로"라는 용어와 일치시키기 위해 BFS(홉수)에서 전환
    # (맵세션4 Phase C-5 재조사, 2026-09-18 — 아래 3번 항목 참고).
    import heapq
    dist = {s: roads[s]["length"] for s in starts}
    prev = {s: None for s in starts}
    pq = [(roads[s]["length"], s) for s in starts]
    heapq.heapify(pq)
    visited = set()
    while pq:
        d, cur = heapq.heappop(pq)
        if cur in visited:
            continue
        visited.add(cur)
        if cur in goals:
            path = [cur]
            while prev[path[-1]] is not None:
                path.append(prev[path[-1]])
            path.reverse()
            return path, d
        for nxt in adj.get(cur, ()):
            nd = d + roads[nxt]["length"]
            if nxt not in dist or nd < dist[nxt]:
                dist[nxt] = nd
                prev[nxt] = cur
                heapq.heappush(pq, (nd, nxt))
    return None, None


def path_length(roads, path):
    return sum(roads[rid]["length"] for rid in path)


def geometry_anomaly_counts(roads, junctions):
    all_rows = {}
    for jid, junction in junctions.items():
        all_rows[jid] = ag.analyze_junction(junction, roads)
    gaps_all = [row["gap_m"] for rows in all_rows.values() for row in rows]
    alongs_all = [row["along_m"] for rows in all_rows.values() for row in rows]
    hdiffs_all = [row["heading_diff_deg"] for rows in all_rows.values() for row in rows]
    n_gap_bad = sum(1 for g in gaps_all if g >= ag.GAP_DISCONTINUITY_M)
    n_along_bad = sum(1 for a in alongs_all if abs(a) >= ag.GAP_DISCONTINUITY_M)
    return {
        "n_connections": len(gaps_all),
        "n_gap_bad": n_gap_bad,
        "n_along_bad": n_along_bad,
        "hdiff_max": max(hdiffs_all) if hdiffs_all else None,
        "hdiff_median": statistics.median(hdiffs_all) if hdiffs_all else None,
    }


def main():
    old_roads, old_junctions = load(OLD_XODR)
    new_roads, new_junctions = load(NEW_XODR)

    lines = []
    lines.append("# 맵세션4 Phase C-5 검증 (자동생성, 읽기전용)")
    lines.append("")
    lines.append("## 1. road / junction 개수")
    lines.append(f"- 기존(19건판): road {len(old_roads)} / junction {len(old_junctions)}")
    lines.append(f"- 신규(73건판): road {len(new_roads)} / junction {len(new_junctions)}")
    lines.append("")

    # 2. 정문-도서관 경로
    old_name2jid = name_to_jid(old_junctions)
    new_name2jid = name_to_jid(new_junctions)
    lines.append("## 2. 정문-도서관 경로")
    for label, roads, name2jid in (("기존", old_roads, old_name2jid), ("신규", new_roads, new_name2jid)):
        gate_jid = name2jid.get(GATE_OSM_ID)
        lib_jid = name2jid.get(LIB_OSM_ID)
        if gate_jid is None or lib_jid is None:
            lines.append(f"- {label}: 정문/도서관 junction 이름 매칭 실패 "
                         f"(gate_jid={gate_jid}, lib_jid={lib_jid})")
            continue
        starts = roads_touching_junction(roads, gate_jid)
        goals = roads_touching_junction(roads, lib_jid)
        adj = build_road_graph(roads)
        path, dist = dijkstra_shortest(adj, roads, starts, goals)
        if path is None:
            lines.append(f"- {label}: 경로 없음(끊김)")
        else:
            length = path_length(roads, path)
            lines.append(f"- {label}: 존재, 거리가중 길이 {length:.2f}m"
                         f"(dijkstra합산 {dist:.2f}m), road수 {len(path)}, "
                         f"road id={path}")
    lines.append("")

    # 3. junction 쌍 거리 비율 (이름 공통 junction만)
    old_locs = junction_locations(old_roads, old_junctions)
    new_locs = junction_locations(new_roads, new_junctions)
    common_names = set(old_name2jid) & set(new_name2jid)
    common_names = {n for n in common_names
                    if old_name2jid[n] in old_locs and new_name2jid[n] in new_locs}
    lines.append(f"## 3. junction 쌍 거리 비율 (공통 이름 junction {len(common_names)}개)")
    ratios = []
    names_sorted = sorted(common_names)
    for i in range(len(names_sorted)):
        for j in range(i + 1, len(names_sorted)):
            n1, n2 = names_sorted[i], names_sorted[j]
            import math
            p1o = old_locs[old_name2jid[n1]]
            p2o = old_locs[old_name2jid[n2]]
            p1n = new_locs[new_name2jid[n1]]
            p2n = new_locs[new_name2jid[n2]]
            do = math.hypot(p1o[0] - p2o[0], p1o[1] - p2o[1])
            dn = math.hypot(p1n[0] - p2n[0], p1n[1] - p2n[1])
            if do > 1.0:  # 1m 미만 쌍은 오차민감이라 제외
                ratios.append(dn / do)
    if ratios:
        lines.append(f"- 쌍 수: {len(ratios)}")
        lines.append(f"- 비율 min/median/mean/max: "
                     f"{min(ratios):.4f}/{statistics.median(ratios):.4f}/"
                     f"{statistics.mean(ratios):.4f}/{max(ratios):.4f}")
    else:
        lines.append("- 비교 가능한 공통 junction 쌍 없음")
    lines.append("")

    # 4. road id 변경 여부
    lines.append("## 4. road id 변경 여부")
    lines.append("- road 요소에 name 속성이 비어있어(name=\"\") id로 직접 대응 불가"
                 " (osm2odr가 원본 OSM way id를 road name에 보존하지 않음)")
    lines.append(f"- id 범위: 기존 {min(int(r) for r in old_roads)}~"
                 f"{max(int(r) for r in old_roads)}, "
                 f"신규 {min(int(r) for r in new_roads)}~"
                 f"{max(int(r) for r in new_roads)}")
    lines.append("- 결론: 개수 자체가 다르므로(134->878) id 체계가 전면 재부여됨."
                 " 1:1 대응표 작성 불가(대응 근거 없음), 사실만 기록")
    lines.append("")

    # 5. 기하 이상 개수
    lines.append("## 5. 기하 이상 개수 (analyze_geometry.py 함수 재사용)")
    for label, roads, junctions in (("기존", old_roads, old_junctions),
                                     ("신규", new_roads, new_junctions)):
        stats = geometry_anomaly_counts(roads, junctions)
        lines.append(f"- {label}: connection {stats['n_connections']}건 중 "
                     f"gap이상(>=0.01m) {stats['n_gap_bad']}건, "
                     f"종단(along)이상 {stats['n_along_bad']}건, "
                     f"heading_diff median={stats['hdiff_median']:.4f}deg "
                     f"max={stats['hdiff_max']:.4f}deg")
    lines.append("")

    OUT_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"완료: {OUT_PATH}")
    for line in lines:
        print(line)


if __name__ == "__main__":
    main()
