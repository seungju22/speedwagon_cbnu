#!/usr/bin/env python3
# 맵세션34 3~5: r1391 영향 범위·우회 경로·새 road 전수(읽기전용, 서버 없음)
# 새 road = v2 road 중 대응표(s28_road_id_map_B.csv) new_id 에 없는 것(재변환으로 생김) + 회차 후처리 추가분(r2054~r2066)
# 형상이 바뀐 road(대응 비용 >= 0.05)도 따로 센다. 곡률 값은 s34_curvature_all_*.csv
import csv
import sys
sys.path.insert(0, "/home/gyumin/campus_mobility_sim/map/scripts")
sys.path.insert(0, "/home/gyumin/campus_mobility_sim/map/tests")
from s19_scope_common import load_map, road_lengths, build_graph, dijkstra, path_to  # noqa: E402
import test_drive as td  # noqa: E402

B = "/home/gyumin/campus_mobility_sim/map/"
V2 = B + "maps/cbnu_internal_only_localtm_tags81_B_smooth_turnfix.xodr"
idm = list(csv.DictReader(open(B + "docs/logs/s28_road_id_map_B.csv")))
mapped = {int(r["new_id"]) for r in idm}
changed = {int(r["new_id"]): (int(r["old_id"]), float(r["match_cost"])) for r in idm if float(r["match_cost"]) >= 0.05}
cur = {int(r["road"]): r for r in csv.DictReader(open(B + "docs/logs/s34_curvature_all_v2.csv"))}
cur1 = {int(r["road"]): r for r in csv.DictReader(open(B + "docs/logs/s34_curvature_all_v1.csv"))}
new = sorted(set(cur) - mapped)
routes = {k: set(v["roads"]) for k, v in td.ROUTES_V2.items()}
r4 = set().union(*(routes[k] for k in ["north", "south", "middle", "yangseong"]))
print(f"[3] r1391 포함 노선: {[k for k, v in routes.items() if 1391 in v]}. 4 노선 포함: {1391 in r4}")
cm, L = load_map(V2), road_lengths(V2)
g = build_graph(cm, L)
pred = sorted(u for u, vs in g.items() if 1391 in vs)
print(f"    r1391 앞 {pred} 뒤 {sorted(g[1391])}")
d, p = dijkstra(g, L, 1278)
base = d[1151] - L[1151][0] + 23.1
d2, p2 = dijkstra(g, L, 1278, banned=frozenset({1391}))
print(f"[4] 기본 경로(정문 -> r1151 s=23.1) {base:.1f}m")
if 1151 in d2:
    pp = path_to(p2, 1278, 1151)
    print(f"    r1391 제외 우회: {d2[1151] - L[1151][0] + 23.1:.1f}m, {len(pp)} road: {pp}")
    sus = [r for r in pp if r in cur and float(cur[r]["min_inner_r"]) <= 3.0]
    print(f"    우회 경로 중 안쪽 가장자리 반경 3m 이하 road: {[(r, cur[r]['min_inner_r']) for r in sus]}")
    print(f"    우회 경로 중 새 road: {[r for r in pp if r in set(new)]}")
else:
    print("    r1391 제외 시 r1151 도달 불가")


def f(r):
    return float(r["min_inner_r"])


def report(label, ids, table):
    le0 = [r for r in ids if f(table[r]) <= 0]
    le1 = [r for r in ids if 0 < f(table[r]) <= 1.0]
    print(f"{label}: {len(ids)} road, 안쪽 가장자리 <=0 {len(le0)} {[(r, table[r]['min_inner_r'], table[r]['s_at']) for r in le0]}")
    print(f"   0<x<=1m {len(le1)} {[(r, table[r]['min_inner_r'], table[r]['s_at'], 'j' + table[r]['junction']) for r in le1]}")
    return le0, le1


print("[5] lane -1 안쪽 가장자리 반경 최솟값")
a0, a1 = report("  새 road(재변환+후처리)", new, cur)
print(f"    새 road 목록: {new}")
print(f"    그중 4 노선 경로 포함: {[r for r in a0 + a1 if r in r4]}, back 포함: {[r for r in a0 + a1 if r in routes['back']]}")
c0, c1 = report("  형상 바뀐 road(비용>=0.05)", sorted(changed), cur)
report("  v2 전체", sorted(cur), cur)
v0, v1 = report("  참고 frozen_v1 전체", sorted(cur1), cur1)
all0 = [r for r in cur if f(cur[r]) <= 1.0]
print(f"  v2 전체 <=1m 중 4 노선 포함: {sorted(r for r in all0 if r in r4)} / back 포함: {sorted(r for r in all0 if r in routes['back'])}")
for k in ["north", "south", "middle", "yangseong", "back"]:
    worst = min(routes[k], key=lambda r: f(cur[r]))
    print(f"  {k}: 경로 최소 안쪽 가장자리 r{worst} {cur[worst]['min_inner_r']}m (s={cur[worst]['s_at']}, j{cur[worst]['junction']})")
sw = [r for r in cur if float(cur[r]["min_with_sidewalk_r"]) <= 0]
print(f"  인도 포함 바깥 끝 <=0(인도 메시 자기교차) v2 {len(sw)} road, 그중 새 road {len([r for r in sw if r in set(new)])}, "
      f"4 노선 포함 {sorted(r for r in sw if r in r4)}, back 포함 {sorted(r for r in sw if r in routes['back'])}")
