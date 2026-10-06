#!/usr/bin/env python3
# 맵세션11 Phase A-3 보조. 읽기전용, CARLA 불필요.
# 질문: 세션10 2차 재현 궤적(회전한 차량 bbox 4모서리)이 xodr 해석적 연석선(t=-3.35)을
#       충돌 시작 시점(frame 167820)에 실제로 넘는가?
#   - 넘는다 -> 연석은 공칭 위치에 있고 차가 바깥으로 쏠려 부딪힌 것(메시 쐐기 불필요)
#   - 안 넘는다 -> 메시가 해석적 형상과 다르다(쐐기/코드 근사 등)
# 가정: 차량 액터 위치 = bbox 중심(bbox 위치 오프셋 미기록). CARLA->xodr 는 y,yaw 부호 반전.
import csv
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_geometry import parse_roads, geom_point, geom_heading

BASE = Path(__file__).resolve().parent.parent
XODR = BASE / "maps" / "cbnu_internal_only_localtm_tags73.xodr"
LOG = BASE / "docs" / "logs" / "drive_log_20260922_213844.csv"
COL = BASE / "docs" / "logs" / "collision_events_20260922_213844.csv"
HALF_L, HALF_W = 1.8527, 0.8943
CURB_T = -3.35
FIRST_COLLISION_FRAME = 167820


def ref_line(road, step=0.02):
    pts = []
    n = int(road["length"] / step)
    for i in range(n + 1):
        s = min(i * step, road["length"])
        g = road["geoms"][0]
        for gg in road["geoms"]:
            if gg["s"] - 1e-9 <= s:
                g = gg
        p = min(max((s - g["s"]) / g["length"], 0.0), 1.0)
        x, y = geom_point(g, p)
        pts.append((s, x, y, geom_heading(g, p)))
    return pts


def project(pts, x, y, hint):
    lo = max(hint - 400, 0)
    hi = min(hint + 400, len(pts))
    best = min(range(lo, hi), key=lambda i: (pts[i][1] - x) ** 2 + (pts[i][2] - y) ** 2)
    s, px, py, h = pts[best]
    t = -(x - px) * math.sin(h) + (y - py) * math.cos(h)
    return best, s, t


def main():
    roads = parse_roads(ET.parse(XODR).getroot())
    pts = ref_line(roads["1247"])
    rows = [r for r in csv.DictReader(open(LOG)) if r["road_id"] == "1247"]
    out = []
    hint = 0
    for r in rows:
        xc, yc, yawc = float(r["x"]), float(r["y"]), float(r["vehicle_yaw"])
        x, y, yaw = xc, -yc, -math.radians(yawc)   # CARLA -> xodr
        cs, sn = math.cos(yaw), math.sin(yaw)
        corners = []
        for lx, ly, name in ((HALF_L, -HALF_W, "FR"), (HALF_L, HALF_W, "FL"),
                             (-HALF_L, -HALF_W, "RR"), (-HALF_L, HALF_W, "RL")):
            # xodr 는 우수계: 전방=(cos,sin), 차체 왼쪽=(-sin,cos). FR 은 ly=-W.
            cx = x + lx * cs - ly * sn
            cy = y + lx * sn + ly * cs
            corners.append((name, cx, cy))
        res = []
        for name, cx, cy in corners:
            hint, s, t = project(pts, cx, cy, hint)
            res.append((t, name, s))
        tmin = min(res)
        tmax = max(res)
        center_hint, sc, tc = project(pts, x, y, hint)
        hint = center_hint
        out.append({"frame": int(r["frame"]), "t": float(r["t"]), "s": float(r["s"]),
                    "t_center": tc, "t_min": tmin[0], "corner": tmin[1], "t_max": tmax[0], "corner_l": tmax[1],
                    "clear": tmin[0] - CURB_T, "spd": float(r["speed_mps"]),
                    "steer": float(r["steer"]), "yaw_rel": math.degrees(
                        (yaw - pts[center_hint][3] + math.pi) % (2 * math.pi) - math.pi)})
    print(f"차량 bbox 반길이 {HALF_L} 반폭 {HALF_W}, 연석선 t={CURB_T}")
    print("첫 충돌 frame", FIRST_COLLISION_FRAME)
    first_neg = next((o for o in out if o["clear"] < 0), None)
    if first_neg:
        print(f"연석선 해석적 침범 첫 시점: frame={first_neg['frame']} t={first_neg['t']:.3f} "
              f"s={first_neg['s']:.2f} 모서리={first_neg['corner']} 여유={first_neg['clear']:+.3f}")
    print("frame   t     s     t_center t_min corner clear  spd  steer yaw_rel")
    for o in out:
        if FIRST_COLLISION_FRAME - 60 <= o["frame"] <= FIRST_COLLISION_FRAME + 40 \
                and (o["frame"] - FIRST_COLLISION_FRAME) % 5 == 0:
            print(f"{o['frame']} {o['t']:6.2f} {o['s']:6.2f} {o['t_center']:+.3f} "
                  f"{o['t_min']:+.3f} {o['corner']} {o['clear']:+.3f} {o['spd']:.2f} "
                  f"{o['steer']:+.2f} {o['yaw_rel']:+.1f}")
    f0 = next(o for o in out if o["frame"] >= FIRST_COLLISION_FRAME)
    print(f"첫 충돌 시점 frame={f0['frame']} s={f0['s']:.2f}: 왼쪽 모서리 최대 t={f0['t_max']:+.3f}"
          f"({f0['corner_l']}), 오른쪽 모서리 최소 t={f0['t_min']:+.3f}, 중심 t={f0['t_center']:+.3f}")
    impulse_directions(pts)
    for lo, hi, name in ((78, 86, "통과 s=78~86"), (86, 93, "정지 전 s=86~93")):
        seg = [o for o in out if lo <= o["s"] <= hi]
        if seg:
            m = min(seg, key=lambda o: o["clear"])
            mx = max(seg, key=lambda o: o["t_max"])
            print(f"{name}: 왼쪽 모서리 최대 t={mx['t_max']:+.3f}m(s={mx['s']:.2f}, 기준선 t=0 넘으면 +) "
                  f"모서리={mx['corner_l']}, 중심 t 범위 {min(o['t_center'] for o in seg):+.3f}"
                  f"~{max(o['t_center'] for o in seg):+.3f}")
            print(f"{name}: 최소 여유 {m['clear']:+.3f}m (s={m['s']:.2f} frame={m['frame']} "
                  f"모서리={m['corner']}), 중심 횡편차 최대 "
                  f"{min(o['t_center'] for o in seg) + 1.675:+.3f}m(우측 -)")


def impulse_directions(pts):
    """충돌 임펄스(차량이 받은 힘, CARLA 월드)를 도로 좌표(along/left)로 분해."""
    rows = list(csv.DictReader(open(COL)))
    print(f"\n충돌 이벤트 {len(rows)}건. 임펄스를 도로 기준 along(+진행)/left(+왼쪽)로 분해")
    print("  left 성분 음수 = 차가 오른쪽으로 밀림 = 장애물이 차의 왼쪽에 있음")
    hint = int(80.0 / 0.02)   # 충돌은 s≈89 부근에서 시작. 투영 창(±8m) 시작점
    acc = []
    for i, r in enumerate(rows):
        x, y = float(r["loc_x"]), -float(r["loc_y"])
        ix, iy = float(r["impulse_x"]), -float(r["impulse_y"])   # CARLA->xodr
        hint, s, t = project(pts, x, y, hint)
        h = pts[hint][3]
        along = ix * math.cos(h) + iy * math.sin(h)
        left = -ix * math.sin(h) + iy * math.cos(h)
        mag = math.hypot(ix, iy)
        acc.append((s, t, along / mag, left / mag, mag))
        if i < 6 or i in (50, 200, 800, 1500):
            print(f"  #{i:<4} frame={r['frame']} s={s:.2f} t={t:+.2f} along={along/mag:+.2f} "
                  f"left={left/mag:+.2f} |imp|={mag:.0f}")
    lefts = [a[3] for a in acc]
    print(f"  전체 left 성분 평균 {sum(lefts)/len(lefts):+.2f} "
          f"(음수 비율 {sum(1 for v in lefts if v < 0)/len(lefts):.0%})")


if __name__ == "__main__":
    main()
