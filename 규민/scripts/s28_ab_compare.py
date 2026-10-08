#!/usr/bin/env python3
# 맵세션28 Phase 3: 재변환 A/B 안 비교용 요약 (읽기전용, 서버 없음, xodr 수정 없음).
# 맵 하나당 출력
#   road 수 / junction 수 (spawn 은 오프라인에서 0 이 나와 서버 로드 값을 쓴다)
#   정문 junction 2개(j1 name=4748296080, j107 name=4748296088)의 연결로 개수·id·들어오는 road·길이
#     -> 번호는 재변환 때 바뀐다(세션18 시험 0/95). 그래서 junction name(= OSM 노드 id)으로 찾는다
#   정의 없는 junction 참조 수(회차 없는 막다른 끝, s27_turnfix scan 과 같은 정의)
#   (선택) 기존 4개 노선 road 를 id 대응표로 번역한 뒤 lane -1 next() 로 이어지는지
# 도달 가능 비율·정문 왕복 비율은 s23_connectivity.py 를 따로 돌린다(이 스크립트에 복제하지 않음)
# 사용: python s28_ab_compare.py <xodr> [<id map csv: old_id,new_id|trial_id>]
import csv
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests"))
from s19_scope_common import load_map, road_lengths, successors  # noqa: E402

GATE_JUNCTIONS = {"4748296080": "j1(frozen_v1)", "4748296088": "j107(frozen_v1)"}
# test_drive.py ROUTES 원문(2026-10-07 시점)에서 옮김. 바뀌면 여기도 맞춘다
ROUTES = {
    "north": [1247, 1914, 1356, 1917, 1355, 1447, 1373, 1673, 1374, 1591, 1375, 1689, 1376, 1625, 1377, 1640, 1378],
    "south": [1247, 1914, 1356, 1917, 1355, 1446, 1172, 1428, 1240, 1695, 1239, 1426, 1238, 1563, 1237],
    "middle": [1247, 1914, 1356, 1917, 1355, 1447, 1373, 1673, 1374, 1592, 1283, 1617, 1284, 1606, 1285, 1597,
               1286, 1838, 1327],
    "yangseong": [1247, 1914, 1356, 1917, 1355, 1446, 1172, 1427, 1127, 1699, 1126, 1726, 1125, 1733, 1124, 1826,
                  1123],
}


def summary(xodr):
    root = ET.parse(str(xodr)).getroot()
    roads = {r.get("id"): r for r in root.iter("road")}
    juncs = {j.get("id"): j for j in root.iter("junction")}
    cmap = load_map(xodr)
    print(f"맵 {Path(xodr).name}")
    # 오프라인 carla.Map 은 spawn 을 만들지 않는다(frozen_v1 에서 0, 서버 로드 값은 1243 session18_load.log).
    # spawn 은 2-6 서버 로드(test_load_map_878.py) 값을 쓴다
    print(f"road {len(roads)} / junction {len(juncs)} / spawn(오프라인, 비교에 쓰지 않음) {len(cmap.get_spawn_points())}")
    for j in juncs.values():
        if j.get("name") in GATE_JUNCTIONS:
            cs = j.findall("connection")
            print(f"정문 junction name={j.get('name')} ({GATE_JUNCTIONS[j.get('name')]}) -> 이 맵 j{j.get('id')}, 연결로 {len(cs)}")
            for c in cs:
                cr = roads[c.get("connectingRoad")]
                lanes = ",".join(f"{ll.get('from')}>{ll.get('to')}" for ll in c.findall("laneLink"))
                print(f"  연결 {c.get('id')}: 들어옴 r{c.get('incomingRoad')} -> 연결로 r{c.get('connectingRoad')}"
                      f" 길이 {float(cr.get('length')):.2f} contact {c.get('contactPoint')} lane {lanes}")
    ref = set()
    for r in roads.values():
        link = r.find("link")
        if link is None:
            continue
        for e in link:
            if e.get("elementType") == "junction":
                ref.add(e.get("elementId"))
    undef = sorted((j for j in ref if j not in juncs or not juncs[j].findall("connection")), key=int)
    print(f"정의 없는 junction 참조(회차 없는 막다른 끝) {len(undef)}: {' '.join('j' + j for j in undef)}")
    return cmap


def check_routes(cmap, xodr, idmap_csv):
    lens = road_lengths(xodr)
    idmap = {}
    with open(idmap_csv) as fh:
        rd = csv.DictReader(fh)
        col = "new_id" if "new_id" in rd.fieldnames else "trial_id"
        for r in rd:
            if r[col]:
                idmap[int(r["old_id"])] = (int(r[col]), float(r["match_cost"]))
    for name, rs in ROUTES.items():
        miss = [r for r in rs if r not in idmap]
        if miss:
            print(f"노선 {name}: 대응 없음 {miss} -> 끊김으로 판정")
            continue
        tr = [idmap[r][0] for r in rs]
        bad_cost = [(o, idmap[o][1]) for o in rs if idmap[o][1] >= 0.05]
        breaks = [(a, b) for a, b in zip(tr, tr[1:]) if b not in successors(cmap, a, lens[a][0])]
        print(f"노선 {name}: {len(rs)} road, 형상 비용 0.05 이상 {bad_cost or '없음'}, next() 끊김 {breaks or '없음'}"
              f" -> {'연결 유지' if not breaks and not bad_cost else '확인 필요'}")
        print(f"  번역 {tr}")


def main():
    xodr = Path(sys.argv[1])
    cmap = summary(xodr)
    if len(sys.argv) > 2:
        check_routes(cmap, xodr, sys.argv[2])


if __name__ == "__main__":
    main()
