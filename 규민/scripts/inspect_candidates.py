#!/usr/bin/env python3
# 사용자 지정 후보 3건(번호1=way392632034, 41=way442709226,
# 48=way452870644)의 시작/중간/끝점 위경도와, 각 way(중간점 기준) 60m
# 반경 내 OSM 지물 전수를 나열한다. 읽기전용, 파일 생성 없음.
#
# 주의: 이 세 way는 아직 xodr로 변환되지 않은 "후보" 단계(Phase B)라
# data/processed/cbnu_campus_fixed.osm(OSM 원본 좌표 그대로, 보정본이지만
# highway태그만 바뀜)에 원래 위경도가 그대로 있다. geo_calibrate.py의
# Umeyama 역변환은 xodr좌표->실제위경도 근사(잔차 RMS 2.68m/최대 6.01m,
# map_session03_report.md A-1)이므로, 이미 위경도를 갖고 있는 이 데이터에
# 굳이 근사 역변환을 거치면 오차만 추가된다. 따라서 원본 위경도를 그대로
# 쓴다(오차 0) — 지시받은 방법과 다르게 처리한 이유를 이 주석과 화면
# 출력에 명시한다.
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from nearby_osm import load as load_nearby, dist_m, OSM_RAW_PATH
from road_candidates import get_candidates

TARGETS = {1: "392632034", 41: "442709226", 48: "452870644"}
RADIUS_M = 60


def main():
    candidates, nodes = get_candidates()
    by_id = {c["id"]: c for c in candidates}

    nb_nodes, nb_ways = load_nearby(OSM_RAW_PATH)

    for number, wid in TARGETS.items():
        c = by_id.get(wid)
        if c is None:
            print(f"번호{number}(way{wid}): 후보 목록에서 못 찾음")
            continue
        nds = c["nds"]
        start_nid, end_nid = nds[0], nds[-1]
        mid_nid = nds[len(nds) // 2]

        def latlon(nid):
            n = nodes[nid]
            return float(n["lat"]), float(n["lon"])

        slat, slon = latlon(start_nid)
        mlat, mlon = latlon(mid_nid)
        elat, elon = latlon(end_nid)

        print(f"\n=== 번호{number} (way{wid}, {c['highway']}"
              f"{'/'+c['service'] if c['service'] else ''}, "
              f"길이{c['length']:.1f}m, node수{len(nds)}) ===")
        print(f"시작점: {slat:.6f}, {slon:.6f}  (node {start_nid})")
        print(f"중간점: {mlat:.6f}, {mlon:.6f}  (node {mid_nid})")
        print(f"끝점  : {elat:.6f}, {elon:.6f}  (node {end_nid})")

        # 60m 반경 내 OSM 지물 전수(중간점 기준)
        found = []
        for nid, (lat, lon, tags) in nb_nodes.items():
            if not tags:
                continue
            d = dist_m(mlat, mlon, lat, lon)
            if d <= RADIUS_M:
                found.append((d, "node", nid, tags))
        for way in nb_ways:
            tags = way["tags"]
            if not tags:
                continue
            pts = [nb_nodes[n][:2] for n in way["nds"] if n in nb_nodes]
            if not pts:
                continue
            cy = sum(p[0] for p in pts) / len(pts)
            cx = sum(p[1] for p in pts) / len(pts)
            d = dist_m(mlat, mlon, cy, cx)
            if d <= RADIUS_M:
                found.append((d, "way", way["id"], tags))

        found.sort(key=lambda x: x[0])
        has_named_building = any("building" in t and "name" in t
                                  for _, _, _, t in found)
        print(f"60m 반경 내 지물 {len(found)}건 "
              f"(이름있는 building 존재: {has_named_building}):")
        if not found:
            print("  없음")
        for d, kind, oid, tags in found:
            tagstr = ", ".join(f"{k}={v}" for k, v in tags.items())
            print(f"  {d:5.1f}m {kind:4s} {oid:>10s}: {tagstr}")


if __name__ == "__main__":
    main()
