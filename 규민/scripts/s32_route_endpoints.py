#!/usr/bin/env python3
# 맵세션32 Phase 0-4: 4 노선 출발·종점을 v2 에서 확정 (읽기전용, 서버 없음, test_drive.py 수정 없음)
# 노선 road 는 test_drive.ROUTES(frozen_v1 id)를 id 대응표로 번역. 출발 = 첫 road lane -1 s=0, 종점 = 끝 road end_s.
# 좌표는 xodr_point_at_s 로 두 맵에서 계산해 거리 비교. next() 체인은 오프라인 carla.Map(lane -1).
# 커넥터 junction 번호는 junction name 으로 번역. 출발 spawn 은 test_drive 가 좌표(221.40,-1086.84)로 찾음 -> 서버에서만.
# 사용: .venv-carla/bin/python s32_route_endpoints.py <v1 xodr> <v2 xodr> <id map csv>
import csv
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(BASE / "tests"))
import test_drive as td  # noqa: E402
import carla  # noqa: E402


def roads(p):
    root = ET.parse(str(p)).getroot()
    return root, {int(r.get("id")): r for r in root.iter("road")}


def jnames(root):
    return {int(j.get("id")): j.get("name") for j in root.iter("junction")}


def main():
    v1, v2, idmap = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3]
    mp = {int(r["old_id"]): (int(r["new_id"]), float(r["match_cost"])) for r in csv.DictReader(open(idmap))}
    r1, e1 = roads(v1)
    r2, e2 = roads(v2)
    jn1, jn2 = jnames(r1), jnames(r2)
    name2j2 = {}
    for j, n in jn2.items():
        name2j2.setdefault(n, []).append(j)
    m2 = carla.Map("v2", v2.read_text())
    fail = 0
    for k in ["north", "south", "middle", "yangseong"]:
        R = td.ROUTES[k]
        new = [mp[r][0] for r in R["roads"]]
        cost = max(mp[r][1] for r in R["roads"])
        missing = [n for n in new if n not in e2]
        sx1, sy1 = td.xodr_point_at_s(e1[R["roads"][0]], 0.0)
        sx2, sy2 = td.xodr_point_at_s(e2[new[0]], 0.0)
        ex1, ey1 = td.xodr_point_at_s(e1[R["roads"][-1]], R["end_s"])
        ex2, ey2 = td.xodr_point_at_s(e2[new[-1]], R["end_s"])
        ds, de = math.hypot(sx1 - sx2, sy1 - sy2), math.hypot(ex1 - ex2, ey1 - ey2)
        brk = []
        for a, b in zip(new, new[1:]):
            L = float(e2[a].get("length"))
            wp = m2.get_waypoint_xodr(a, -1, max(L - 0.05, 0.0))
            nxt = {w.road_id for w in wp.next(1.0)} if wp else set()
            if b not in nxt:
                brk.append((a, b))
        jbad = []
        conn2 = {}
        for rid, j in R["connectors"].items():
            nrid = mp[rid][0]
            cand = name2j2.get(jn1[j], [])
            jr = int(e2[nrid].get("junction"))
            conn2[nrid] = jr
            if cand != [jr]:
                jbad.append((rid, j, nrid, jr, cand))
        sharp2 = {mp[r][0]: v for r, v in R["sharp"].items()}
        ok = not (missing or brk or jbad or ds > 0.01 or de > 0.01 or cost >= 0.05)
        fail += 0 if ok else 1
        print(f"{k}: {'대응 성공' if ok else '대응 실패'}")
        print(f"  v1 출발 r{R['roads'][0]} s=0 ({sx1:.3f},{sy1:.3f}) -> v2 r{new[0]} ({sx2:.3f},{sy2:.3f}) 차 {ds:.4f}m")
        print(f"  v1 종점 r{R['roads'][-1]} s={R['end_s']} ({ex1:.3f},{ey1:.3f}) -> v2 r{new[-1]} ({ex2:.3f},{ey2:.3f}) 차 {de:.4f}m")
        print(f"  최대 형상 비용 {cost:.4f}, v2 없는 road {missing}, next() 끊김 {brk}")
        print(f"  v2 roads {new}")
        print(f"  v2 connectors {conn2} (junction name 대응 불일치 {jbad})")
        print(f"  v2 sharp {sharp2}")
    print(f"대응 실패 노선 {fail}")


if __name__ == "__main__":
    main()
