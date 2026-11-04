#!/usr/bin/env python3
# 11/03: 도서관 길 꺾이는 모서리('가', 11/03 '도서관 지나 개신문화관이 정면, 좌회전만 되는 곳에서 좌회전하는 척 유턴')
#   유턴 중심선(뒷축 경로) 찾기. 서버 없음, 지도 파일 읽기만
#   CARLA 면 = 동결 xodr 오프라인 carla.Map, get_waypoint(project_to_road=False, Driving|Sidewalk) (s4_track.py 바퀴 이탈 판정과 같음)
#   후보 = r1415(37801, 동남행) 중심선 i 점 -> r1409(36955, 북서행) 중심선 j 점, Dubins 경로(반경 r, 6가지 꼴)
#   점수 = 두 차(프리우스 sample_vehicle, 스프린터) 바퀴 4개(타이어 반폭 0.12m 더함)가 CARLA 면 밖으로 나간 최대 거리. 작을수록 좋음
# 출력: integration/logs/s75/uturn_search.json(위 후보 20개), 경로 점은 --emit 로 고른 후보만
# 사용: .venv-carla/bin/python map/scripts/s75_uturn_search.py [--rmin 3.9] [--rmax 5.5] [--emit 0]
import argparse, json, math, re, sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent; C = HERE.parents[1]
sys.path.insert(0, str(HERE))
ap = argparse.ArgumentParser(); ap.add_argument("--rmin", type=float, default=3.9); ap.add_argument("--rmax", type=float, default=5.5)
ap.add_argument("--src", default=str(C / "map/maps/v33b/lanelet2_map.osm")); ap.add_argument("--emit", type=int, default=-1)
ap.add_argument("--out", default=str(C / "integration/logs/s75/uturn_search.json")); ap.add_argument("--tm", type=float, default=0.12)
a = ap.parse_args()
X0, X1, Y0, Y1, S = 670, 770, 610, 700, 0.25
RAS = C / "integration/logs/s75/v_raster_670_770_610_700.npy"
if RAS.exists():
    G = np.load(RAS)
else:
    import s72_common as K, carla
    m = K.cmap(); RD = carla.LaneType.Driving | carla.LaneType.Sidewalk
    nx = int((X1 - X0) / S) + 1; ny = int((Y1 - Y0) / S) + 1; G = np.zeros((ny, nx), bool)
    for iy in range(ny):
        for ix in range(nx):
            G[iy, ix] = m.get_waypoint(carla.Location(X0 + ix * S, -(Y0 + iy * S), 0), project_to_road=False, lane_type=RD) is not None
    np.save(RAS, G)
ny, nx = G.shape; ON = np.argwhere(G)


def outd(x, y):
    """CARLA 면 밖이면 가장 가까운 면 칸까지 거리(m), 안이면 0"""
    ix = int(round((x - X0) / S)); iy = int(round((y - Y0) / S))
    if 0 <= ix < nx and 0 <= iy < ny and G[iy, ix]: return 0.0
    w = 16; sub = G[max(0, iy - w):iy + w + 1, max(0, ix - w):ix + w + 1]
    pts = np.argwhere(sub)
    if len(pts) == 0: return 9.9
    d = np.hypot(pts[:, 0] + max(0, iy - w) - iy, pts[:, 1] + max(0, ix - w) - ix).min() * S
    return float(d)


txt = Path(a.src).read_text()
NODE_RE = r'<node id="(\d+)"[^>]*>\s*<tag k="ele" v="[-\d.]+" />\s*<tag k="local_x" v="([-\d.]+)" />\s*<tag k="local_y" v="([-\d.]+)" />'
nodes = {i: (float(x), float(y)) for i, x, y in re.findall(NODE_RE, txt)}
ways = {m_.group(1): re.findall(r'<nd ref="(\d+)" />', m_.group(2)) for m_ in re.finditer(r'<way id="(\d+)"[^>]*>(.*?)</way>', txt, re.S)}


def lan(rid):
    rel = re.search(rf'<relation id="{rid}"[^>]*>(.*?)</relation>', txt, re.S).group(1)
    mem = dict((r, ref) for ref, r in re.findall(r'<member type="way" ref="(\d+)" role="(\w+)" />', rel))
    return ways[mem["left"]], ways[mem["right"]]


LA, RA = lan("37801"); LB, RB = lan("36955")
cA = [((nodes[p][0] + nodes[q][0]) / 2, (nodes[p][1] + nodes[q][1]) / 2) for p, q in zip(LA, RA)]
cB = [((nodes[p][0] + nodes[q][0]) / 2, (nodes[p][1] + nodes[q][1]) / 2) for p, q in zip(LB, RB)]
hd = lambda c, i: math.atan2(c[min(len(c) - 1, i + 1)][1] - c[max(0, i - 1)][1], c[min(len(c) - 1, i + 1)][0] - c[max(0, i - 1)][0])
mod = lambda x: x % (2 * math.pi)


def dubins(q0, q1, r):
    dx, dy = q1[0] - q0[0], q1[1] - q0[1]; D = math.hypot(dx, dy) / r; th = mod(math.atan2(dy, dx)); al = mod(q0[2] - th); be = mod(q1[2] - th)
    sa, sb, ca, cb, cab = math.sin(al), math.sin(be), math.cos(al), math.cos(be), math.cos(al - be); out = []
    p2 = 2 + D * D - 2 * cab + 2 * D * (sa - sb)
    if p2 >= 0: g = math.atan2(cb - ca, D + sa - sb); out.append(("LSL", mod(-al + g), math.sqrt(p2), mod(be - g)))
    p2 = 2 + D * D - 2 * cab + 2 * D * (sb - sa)
    if p2 >= 0: g = math.atan2(ca - cb, D - sa + sb); out.append(("RSR", mod(al - g), math.sqrt(p2), mod(-be + g)))
    p2 = -2 + D * D + 2 * cab + 2 * D * (sa + sb)
    if p2 >= 0:
        p = math.sqrt(p2); g = math.atan2(-ca - cb, D + sa + sb) - math.atan2(-2, p); out.append(("LSR", mod(-al + g), p, mod(-mod(be) + g)))
    p2 = D * D - 2 + 2 * cab - 2 * D * (sa + sb)
    if p2 >= 0:
        p = math.sqrt(p2); g = math.atan2(ca + cb, D - sa - sb) - math.atan2(2, p); out.append(("RSL", mod(al - g), p, mod(be - g)))
    t_ = (6 - D * D + 2 * cab + 2 * D * (sa - sb)) / 8
    if abs(t_) <= 1:
        p = mod(2 * math.pi - math.acos(t_)); t = mod(al - math.atan2(ca - cb, D - sa + sb) + p / 2); out.append(("RLR", t, p, mod(al - be - t + p)))
    t_ = (6 - D * D + 2 * cab + 2 * D * (-sa + sb)) / 8
    if abs(t_) <= 1:
        p = mod(2 * math.pi - math.acos(t_)); t = mod(-al - math.atan2(ca - cb, D + sa - sb) + p / 2); out.append(("LRL", t, p, mod(mod(be) - al - t + p)))
    return out


def sample(q0, typ, segs, r, step=0.2):
    x, y, h = q0; pts = [(x, y, h)]
    for c, L in zip(typ, segs):
        n = max(1, int(L * r / step))
        for _ in range(n):
            d = L * r / n
            if c == "S": x += d * math.cos(h); y += d * math.sin(h)
            else:
                k = 1 if c == "L" else -1; dh = k * d / r
                x += r * k * (math.sin(h + dh) - math.sin(h)); y += -r * k * (math.cos(h + dh) - math.cos(h)); h += dh
            pts.append((x, y, h))
    return pts


VEH = {"prius_sample": dict(wb=2.79, tr=1.64), "sprinter": dict(wb=3.661, tr=1.676)}


def worst(pts, v):
    w = 0.0
    for x, y, h in pts[::2]:
        c, s = math.cos(h), math.sin(h)
        for aa in (0.0, v["wb"]):
            for b in (-v["tr"] / 2 - a.tm, v["tr"] / 2 + a.tm):
                w = max(w, outd(x + aa * c - b * s, y + aa * s + b * c))
    return w


cands = []
for i in range(96, 162, 2):
    for j in range(56, 130, 2):
        q0 = (*cA[i], hd(cA, i)); q1 = (*cB[j], hd(cB, j))
        for r in np.arange(a.rmin, a.rmax + 1e-6, 0.2):
            for typ, t, p, q in dubins(q0, q1, r):
                L = (t + p + q) * r
                if L > 45: continue
                pts = sample(q0, typ, (t, p, q), r)
                if math.dist(pts[-1][:2], q1[:2]) > 0.2: continue
                ws = {k: round(worst(pts, v), 2) for k, v in VEH.items()}
                cands.append(dict(score=max(ws.values()), worst=ws, r=round(float(r), 2), i=i, j=j, typ=typ, segs=[round(t, 3), round(p, 3), round(q, 3)], L=round(L, 1),
                                  start=[round(v, 2) for v in cA[i]], end=[round(v, 2) for v in cB[j]]))
cands.sort(key=lambda d: (d["score"], -d["r"], d["L"]))
Path(a.out).write_text(json.dumps(dict(raster=str(RAS), box=[X0, X1, Y0, Y1, S], n=len(cands), top=cands[:20]), ensure_ascii=False, indent=1))
for d in cands[:8]: print(d)
if a.emit >= 0:
    d = cands[a.emit]; q0 = (*cA[d["i"]], hd(cA, d["i"]))
    pts = sample(q0, d["typ"], d["segs"], d["r"], 0.25)
    Path(a.out.replace(".json", "_path.json")).write_text(json.dumps(dict(pick=d, pts=[[round(x, 3), round(y, 3), round(h, 4)] for x, y, h in pts]), indent=0))
