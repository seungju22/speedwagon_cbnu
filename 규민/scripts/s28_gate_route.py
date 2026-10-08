#!/usr/bin/env python3
# 맵세션28 Phase 2-라. 392632034 를 넣으면 후문 노선이 어떻게 달라지는가 (읽기전용, 서버 없음)
# 세션18 시험 맵(_trial_s18_gates.xodr, 채택 안 된 산출물, 입력에 392632034 포함)을 읽기만 한다.
# 대상점 3개(고리 접점 / 추정 게이트 / 경계 횡단점)까지 정문 r1247(시험 번호)에서 최단 경로,
# 각 대상점 road 에서 정문으로 돌아오는 경로 존재 여부.
# 실행: ~/campus_mobility_sim/.venv-carla/bin/python map/scripts/s28_gate_route.py > map/docs/logs/s28_gate_route.log
import csv
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from s19_scope_common import XODR_TRIAL, load_map, road_lengths, build_graph, dijkstra, path_to  # noqa: E402
from s27_route_checks import LOGS, ll_carla, path_len, fmt  # noqa: E402

TARGETS = [("고리 접점", (36.624909, 127.462514)), ("추정 게이트", (36.624844, 127.462860)),
           ("경계 횡단점", (36.624701, 127.463656))]


def main():
    idmap = {}
    with open(LOGS / "s18_D_road_id_map.csv") as fh:
        for r in csv.DictReader(fh):
            idmap[int(r["old_id"])] = int(r["trial_id"])
    inv = {v: k for k, v in idmap.items()}
    tmap = load_map(XODR_TRIAL)
    Lt = road_lengths(XODR_TRIAL)
    g = build_graph(tmap, Lt)
    src = idmap[1247]
    dist, prev = dijkstra(g, Lt, src, src_rest=Lt[src][0])
    print(f"시험 맵 road {len(Lt)}, 정문 r1247 -> 시험 r{src}, 정문에서 도달 road {len(dist)}")
    for label, ll in TARGETS:
        cx, cy = ll_carla(ll)
        best = None
        for rid in dist:
            L = Lt[rid][0]
            ss = 0.0
            while ss <= L:
                wp = tmap.get_waypoint_xodr(rid, -1, ss)
                if wp is not None:
                    loc = wp.transform.location
                    d = math.hypot(loc.x - cx, loc.y - cy)
                    if best is None or d < best[0]:
                        best = (d, rid, ss)
                ss += 1.0
        d, rid, ss = best
        p = path_to(prev, src, rid)
        new = [r for r in p if r not in inv]
        print(f"[{label} {fmt(ll)}] 최근접 시험 r{rid}(junction={Lt[rid][1]}) s={ss:.0f} 거리 {d:.1f}m")
        print(f"  정문 -> {len(p)} road, {path_len(Lt, p, ss):.0f}m, 새 road {len(new)}개 {new}")
        back, bprev = dijkstra(g, Lt, rid, src_rest=Lt[rid][0] - ss)
        # 정문은 들어오기만 하는 road 라(정문 출구 way 없는 시험 맵) 복귀 확인은 남문 정차 road(옛 1237)로 한다
        for name, old in (("정문", 1247), ("남문 정차 road", 1237)):
            t = idmap[old]
            if t in back:
                q = path_to(bprev, rid, t)
                print(f"  복귀 -> {name} 옛 r{old}(시험 r{t}) {len(q)} road, {back[t]:.0f}m")
            else:
                print(f"  복귀 -> {name} 옛 r{old}(시험 r{t}) 경로 없음")


if __name__ == "__main__":
    main()
