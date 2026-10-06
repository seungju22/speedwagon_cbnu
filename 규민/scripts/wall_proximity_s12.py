"""세션12 읽기 전용: CARLA 0.9.15 MeshFactory 규칙으로 벽 위치를 재구성하고
충돌 시점 차량 bbox 와의 거리를 잰다(서버 불필요).

벽 규칙(LibCarla/source/carla/road/MeshFactory.cpp 0.9.15, GenerateWalls):
- junction 이 아닌 road 에만 생성
- 가장 바깥 오른쪽 차선(min_lane)의 오른쪽 가장자리 -> 오른쪽 벽
- max_lane 의 왼쪽 가장자리 -> 왼쪽 벽. 차선이 오른쪽뿐이면 max_lane=-1 이고
  그 왼쪽 가장자리 = 기준선(t=0)
- 높이 wall_height(기본 1.0m), 두께 0(단면 삼각형 띠)
"""
import csv, math, sys
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests"))
import test_drive as td  # xodr_point_at_s 재사용

XODR = ROOT / "maps" / "cbnu_internal_only_localtm_tags73_smooth.xodr"
TS = "20260924_113337"
FIRST_COLL_FRAME = 165683
HALF_L, HALF_W = 1.853, 0.894   # vehicle_bbox_20260924_113337.txt
OUTER_T = -(3.35 + 2.80)       # 오른쪽 벽 t
RADIUS = 40.0


def wall_polylines(road, step=0.5):
    L = float(road.get("length"))
    pts0, ptsR = [], []
    n = max(2, int(L / step) + 1)
    for i in range(n):
        s = min(L - 1e-3, max(1e-3, i * L / (n - 1)))
        x, y = td.xodr_point_at_s(road, s)
        x2, y2 = td.xodr_point_at_s(road, min(L - 1e-4, s + 0.05))
        x1, y1 = td.xodr_point_at_s(road, max(1e-4, s - 0.05))
        h = math.atan2(y2 - y1, x2 - x1)
        # xodr 좌표 -> CARLA 좌표(y 반전)
        pts0.append((x, -y))
        nx, ny = -math.sin(h), math.cos(h)          # xodr 왼쪽 법선
        ptsR.append((x + OUTER_T * nx, -(y + OUTER_T * ny)))
    return pts0, ptsR


def seg_dist(p, a, b):
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx - ax, by - ay
    t = 0 if dx == dy == 0 else max(0, min(1, ((px-ax)*dx + (py-ay)*dy) / (dx*dx + dy*dy)))
    return math.hypot(px - ax - t*dx, py - ay - t*dy), (ax + t*dx, ay + t*dy)


def main():
    rows = {int(r["frame"]): r for r in csv.DictReader(
        open(ROOT / "docs" / "logs" / f"drive_log_{TS}.csv"))}
    frames = sorted(rows)
    r = rows[FIRST_COLL_FRAME]
    cx, cy, yaw = float(r["x"]), float(r["y"]), math.radians(float(r["vehicle_yaw"]))
    fx, fy = math.cos(yaw), math.sin(yaw)
    rx, ry = -math.sin(yaw), math.cos(yaw)          # CARLA 오른쪽
    corners = {k: (cx + a*HALF_L*fx + b*HALF_W*rx, cy + a*HALF_L*fy + b*HALF_W*ry)
               for k, (a, b) in {"FL": (1, -1), "FR": (1, 1), "RL": (-1, -1), "RR": (-1, 1)}.items()}
    print(f"car at first collision: ({cx:.2f},{cy:.2f}) yaw {math.degrees(yaw):.1f}")
    root = ET.parse(XODR).getroot()
    hits = []
    for road in root.iter("road"):
        if road.get("junction") != "-1":
            continue
        pts0, ptsR = wall_polylines(road, step=2.0)
        if min(math.hypot(x - cx, y - cy) for x, y in pts0 + ptsR) > RADIUS:
            continue
        pts0, ptsR = wall_polylines(road)
        for name, pts in (("center(t=0)", pts0), ("outer(t=-6.15)", ptsR)):
            for k, c in corners.items():
                d, q = min(seg_dist(c, a, b) for a, b in zip(pts, pts[1:]))
                # 벽점 q 가 차의 어느 쪽인지(차량 좌표)
                lat = (q[0] - cx) * rx + (q[1] - cy) * ry
                lon = (q[0] - cx) * fx + (q[1] - cy) * fy
                hits.append((d, road.get("id"), name, k, lat, lon, pts[0], pts[-1]))
    hits.sort()
    print("closest walls to bbox corners (d, road, wall, corner, wall-pt lateral[+right], longitudinal[+fwd]):")
    seen = set()
    for d, rid, name, k, lat, lon, p0, p1 in hits:
        if (rid, name) in seen:
            continue
        seen.add((rid, name))
        print(f"{d:6.2f}m road{rid:<5} {name:<15} {k} lat{lat:+.2f} lon{lon:+.2f} "
              f"ends ({p0[0]:.1f},{p0[1]:.1f})-({p1[0]:.1f},{p1[1]:.1f})")
        if len(seen) >= 8:
            break


if __name__ == "__main__":
    main()
