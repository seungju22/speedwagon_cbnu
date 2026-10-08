#!/usr/bin/env python3
# 맵세션31 Phase 3: 정문 j107 커넥터(frozen_v1 r1917) 재변환 전후 비교 (읽기전용, carla 오프라인 Map, 서버 없음)
# 새 맵에서는 id 가 바뀌므로 좌표로 찾는다: frozen_v1 r1917 lane -1 의 s=중간 점에서 가장 가까운 driving waypoint 의 road
# 출력: 길이, junction, laneOffset, 차선 구성, pred/succ, 앞 road 끝 -> 이 road 시작 거리·방위 차, 이 road 끝 -> 뒤 road 시작
# 사용: .venv-carla/bin/python s31_r1917_probe.py <frozen_v1 xodr> <새 xodr> [<새 xodr> ...]
import math
import sys
import xml.etree.ElementTree as ET

import carla


def info(path, loc=None, rid=None):
    txt = open(path).read()
    m = carla.Map("p", txt)
    root = ET.fromstring(txt.encode())
    if rid is None:
        rid = m.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving).road_id
    r = root.find(f".//road[@id='{rid}']")
    L = float(r.get("length"))
    lk = r.find("link")
    get = lambda k: (lk.find(k).get("elementType"), lk.find(k).get("elementId"), lk.find(k).get("contactPoint"))  # noqa: E731
    lo = [(x.get("s"), x.get("a"), x.get("b"), x.get("c"), x.get("d")) for x in r.findall("lanes/laneOffset")]
    lanes = [(l.get("id"), l.get("type"), l.find("width").get("a") if l.find("width") is not None else None) for l in r.iter("lane")]
    geo = [(g.get("x"), g.get("y"), g.get("hdg"), g.get("length"), [c.tag for c in g][0]) for g in r.findall("planView/geometry")]
    jn = r.get("junction")
    jname = root.find(f".//junction[@id='{jn}']").get("name") if jn != "-1" else None
    print(f"  road {rid} name={r.get('name')!r} 길이 {L:.4f} junction {jn} (name {jname})")
    print(f"  laneOffset {lo or '없음'} / 차선 {lanes}")
    print(f"  planView {geo}")
    print(f"  pred {get('predecessor')} succ {get('successor')}")
    wp0 = m.get_waypoint_xodr(rid, -1, 0.001)
    wp1 = m.get_waypoint_xodr(rid, -1, L - 0.001)
    pr, su = get("predecessor"), get("successor")
    for tag, other, mine in (("앞", pr, wp0), ("뒤", su, wp1)):
        if other[0] != "road":
            continue
        o = int(other[1])
        oL = float(root.find(f".//road[@id='{o}']").get("length"))
        s = oL - 0.001 if other[2] == "end" else 0.001
        ow = m.get_waypoint_xodr(o, -1, s)
        d = ow.transform.location.distance(mine.transform.location)
        dy = abs((ow.transform.rotation.yaw - mine.transform.rotation.yaw + 180) % 360 - 180)
        print(f"  {tag} road {o}({other[2]}) lane -1 점 <-> 이 road 끝점 거리 {d:.3f}m, 방위 차 {dy:.1f}도")
    # 중심선 방위 변화(지그재그)
    ys = [m.get_waypoint_xodr(rid, -1, max(0.001, min(L - 0.001, L * k / 10))).transform.rotation.yaw for k in range(11)]
    print(f"  lane -1 방위(s 0~L 11점) {[round(y, 1) for y in ys]}")
    mid = m.get_waypoint_xodr(rid, -1, L / 2).transform.location
    return mid


def main():
    print(f"## {sys.argv[1]} (frozen_v1, r1917)")
    mid = info(sys.argv[1], rid=1917)
    for p in sys.argv[2:]:
        print(f"## {p} (frozen_v1 r1917 중간점 {mid.x:.2f},{mid.y:.2f} 최근접)")
        info(p, loc=mid)


if __name__ == "__main__":
    main()
