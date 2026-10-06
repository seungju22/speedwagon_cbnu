#!/usr/bin/env python3
# 맵세션27: 옛 xodr road id -> 새 xodr road id 형상 대응표 (읽기전용, 서버 없음).
# 세션18 D 의 id_map_s18.py 는 scratchpad 에만 있어 사라졌다. 같은 목적으로 다시 만든다.
# 비교 기준(같은 road 의 정의): 참조선 시작점(planView 첫 geometry x,y), 시작 방위 hdg, 길이.
#   비용 = 시작점 거리(m) + 방위 차(rad) + 길이 차(m). 비용 < THRESH 를 "형상 일치" 로 본다(세션18 과 같은 0.05)
# 검증: frozen_v1 -> 세션18 시험 맵으로 돌려 s18_D_road_id_map.csv 와 같은지 대조(--check-s18)
# 사용: python s27_id_map.py <옛 xodr> <새 xodr> <출력 csv>
#       python s27_id_map.py --check-s18
import csv
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

BASE = Path("/home/gyumin/campus_mobility_sim/map")
THRESH = 0.05


def roads(path):
    out = {}
    for r in ET.parse(str(path)).getroot().iter("road"):
        g = r.find("planView/geometry")
        out[int(r.get("id"))] = (float(g.get("x")), float(g.get("y")), float(g.get("hdg")),
                                 float(r.get("length")), int(r.get("junction")))
    return out


def cost(a, b):
    dh = abs((a[2] - b[2] + math.pi) % (2 * math.pi) - math.pi)
    return math.hypot(a[0] - b[0], a[1] - b[1]) + dh + abs(a[3] - b[3])


def build(old_path, new_path):
    old, new = roads(old_path), roads(new_path)
    # 시작점 격자로 후보를 좁힌다(전수 비교와 결과 같음, 속도만)
    grid = {}
    for nid, v in new.items():
        grid.setdefault((int(v[0] // 20), int(v[1] // 20)), []).append(nid)
    rows = []
    for oid, v in sorted(old.items()):
        gx, gy = int(v[0] // 20), int(v[1] // 20)
        cand = [n for dx in (-1, 0, 1) for dy in (-1, 0, 1) for n in grid.get((gx + dx, gy + dy), [])]
        if not cand:
            cand = list(new)
        best = min(cand, key=lambda n: cost(v, new[n]))
        c = cost(v, new[best])
        if c >= 1.0:   # 근처에 없으면 전수
            best = min(new, key=lambda n: cost(v, new[n]))
            c = cost(v, new[best])
        rows.append((oid, best, round(c, 3), v[3], new[best][3]))
    return rows, old, new


def write(rows, out):
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["old_id", "new_id", "match_cost", "old_length", "new_length"])
        w.writerows(rows)


def summary(rows, new):
    ok = [r for r in rows if r[2] < THRESH]
    dup = len(ok) - len({r[1] for r in ok})
    print(f"옛 {len(rows)} / 새 {len(new)}: 형상 일치(비용<{THRESH}) {len(ok)}, 불일치 {len(rows) - len(ok)}, "
          f"일치 중 새 id 중복 {dup}")
    bad = sorted([r for r in rows if r[2] >= THRESH], key=lambda r: r[2])
    print("불일치(비용 순):", [(r[2], r[0], r[1]) for r in bad])
    return ok, bad


def check_s18():
    rows, old, new = build(BASE / "maps/cbnu_internal_only_localtm_tags73_smooth.xodr",
                           BASE / "maps/cbnu_internal_only_localtm_tags73_smooth_trial_s18_gates.xodr")
    ok, bad = summary(rows, new)
    ref = {}
    with open(BASE / "docs/logs/s18_D_road_id_map.csv") as f:
        for r in csv.DictReader(f):
            ref[int(r["old_id"])] = (int(r["trial_id"]), float(r["match_cost"]))
    same_ok = sum(1 for r in ok if ref[r[0]][0] == r[1])
    ref_ok = {k for k, v in ref.items() if v[1] < THRESH}
    mine_ok = {r[0] for r in ok}
    print(f"세션18 대응표 대조: 일치 판정 집합 같음={ref_ok == mine_ok} (세션18 {len(ref_ok)} / 이번 {len(mine_ok)}), "
          f"일치 road 중 새 id 같음 {same_ok}/{len(ok)}")
    diff = [(r[0], r[1], ref[r[0]][0]) for r in rows if ref[r[0]][0] != r[1]]
    print(f"새 id 가 세션18 과 다른 road {len(diff)}: {diff[:20]}")


if __name__ == "__main__":
    if sys.argv[1:] == ["--check-s18"]:
        check_s18()
    elif len(sys.argv) == 4:
        rows, old, new = build(Path(sys.argv[1]), Path(sys.argv[2]))
        summary(rows, new)
        write(rows, Path(sys.argv[3]))
        print(f"출력 {sys.argv[3]}")
    else:
        sys.exit(__doc__ or "인자: <옛 xodr> <새 xodr> <출력 csv> | --check-s18")
