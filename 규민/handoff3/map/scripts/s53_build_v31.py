#!/usr/bin/env python3
# 10/11 2번: v3 동결본 복사 -> v3.1 후보. 문제 요소만 바꾸고 나머지 road 는 바이트 그대로
# 방법(10/09 과 같은 틀): road 안 이어진 planView 요소 묶음(창)을 line+arc+line 또는 3차 Hermite(paramPoly3 normalized) 하나로 바꾼다
#   - 창 양끝 위치·방위 그대로(이음매 수정 r1972 만 예외: 아래 FIX 표의 move_end)
#   - road ID·link·lanes 그대로. 길이는 바뀔 수 있다(바뀐 만큼 보고). 새 paramPoly3 의 length = 실제 호 길이(CARLA 가 호 길이로 읽음, s53_geom)
#   - 목표: 바뀐 road 의 차선 중심 최소 반경 >= R_TARGET(4.0m). 근거: Autoware 기본 차 최소 회전 반경 3.312m(wheel_base 2.79 / tan 0.70)에 20% 여유
#   - 10/09 의 '옛 기준선 0.2m 안' 은 이음매·접힘에서는 못 지킨다. 대신 벗어남 최대값과, 새 차선(중심 ± 폭/2)이 옛 차선+인도 띠 안인지 적는다
# 입력: map/maps/cbnu_campus_frozen_v3.xodr (읽기만)
# 출력: map/maps/cbnu_campus_v31_candidate_s53.xodr, map/docs/logs/s53_build_v31.log, .json
# 사용: .venv-carla/bin/python map/scripts/s53_build_v31.py [--only r1972,r1317 ...]
import argparse
import hashlib
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
BASE = HERE.parent
sys.path.insert(0, str(HERE))
import s53_geom as G  # noqa: E402

SRC = BASE / "maps/cbnu_campus_frozen_v3.xodr"
OUT = BASE / "maps/cbnu_campus_v31_candidate_s53.xodr"
LOG = BASE / "docs/logs/s53_build_v31.log"
R_TARGET = 4.0
GEO_RE = re.compile(r'( *)<geometry s="([\d.e+-]+)" x="([-\d.e+]+)" y="([-\d.e+]+)" hdg="([-\d.e+]+)" length="([\d.e+-]+)">\s*'
                    r'(<line/>|<arc [^>]*/>|<paramPoly3 [^>]*/>)\s*</geometry>\n')
ROAD_RE = re.compile(r'^ *<road name="[^"]*" length="([^"]*)" id="(\d+)" junction="(-?\d+)">.*?</road>\n', re.S | re.M)
log = []


def P(*a):
    s = " ".join(str(x) for x in a)
    log.append(s)
    print(s, flush=True)


# ---------------- 요소 ----------------

def kind_tuple(xml):
    if xml == "<line/>":
        return ("line", None)
    if xml.startswith("<arc"):
        return ("arc", float(re.search(r'curvature="([-\d.e+]+)"', xml).group(1)))
    return ("pp3", {k: float(v) for k, v in re.findall(r'(\w+)="([-\d.e+]+)"', xml) if k != "pRange"})


def end_pose(x, y, h, L, xml):
    e = G.geo_eval((0.0, x, y, h, L, kind_tuple(xml)), L)
    return e[0], e[1], e[2]


def herm_coef(A, B, a, b):
    dx, dy = B[0] - A[0], B[1] - A[1]
    ex, ey = dx * math.cos(A[2]) + dy * math.sin(A[2]), -dx * math.sin(A[2]) + dy * math.cos(A[2])
    d1 = B[2] - A[2]
    t1u, t1v = b * math.cos(d1), b * math.sin(d1)
    return dict(aU=0.0, bU=a, cU=3 * ex - 2 * a - t1u, dU=-2 * ex + a + t1u,
                aV=0.0, bV=0.0, cV=3 * ey - t1v, dV=-2 * ey + t1v)


def pp3_xml(c):
    return '<paramPoly3 ' + " ".join(f'{k}="{c[k]:.10f}"' for k in ("aU", "bU", "cU", "dU", "aV", "bV", "cV", "dV")) + ' pRange="normalized"/>'


def pp3_arclen(c, n=4000):
    L, px, py = 0.0, 0.0, 0.0
    for i in range(1, n + 1):
        q = i / n
        x = c["bU"] * q + c["cU"] * q * q + c["dU"] * q ** 3
        y = c["bV"] * q + c["cV"] * q * q + c["dV"] * q ** 3
        L += math.hypot(x - px, y - py)
        px, py = x, y
    return L


def pp3_lane_rmin(c, off, n=800):
    """paramPoly3 의 차선 중심(옆 off) 최소 반경. 접히면 0"""
    r = math.inf
    for i in range(n + 1):
        q = i / n
        du, dv = c["bU"] + 2 * c["cU"] * q + 3 * c["dU"] * q * q, c["bV"] + 2 * c["cV"] * q + 3 * c["dV"] * q * q
        ddu, ddv = 2 * c["cU"] + 6 * c["dU"] * q, 2 * c["cV"] + 6 * c["dV"] * q
        sp = math.hypot(du, dv)
        if sp < 1e-9:
            return 0.0
        k = (du * ddv - dv * ddu) / sp ** 3
        f = 1 - k * off
        if f <= 0:
            return 0.0
        if abs(k) > 1e-12:
            r = min(r, abs(f / k))
    return r


def herm_dev(c, A, old_line, n=40):
    from shapely.geometry import Point
    d = 0.0
    for i in range(n + 1):
        q = i / n
        u = c["bU"] * q + c["cU"] * q * q + c["dU"] * q ** 3
        v = c["bV"] * q + c["cV"] * q * q + c["dV"] * q ** 3
        x, y = A[0] + u * math.cos(A[2]) - v * math.sin(A[2]), A[1] + u * math.sin(A[2]) + v * math.cos(A[2])
        d = max(d, old_line.distance(Point(x, y)))
    return d


def herm_mindev(A, B, off, old_line, n=40, lo=0.1, hi=2.5):
    """차선 반경 >= R_TARGET*1.02 인 Hermite 중 옛 기준선에서 가장 덜 벗어나는 것. 없으면 None"""
    T = math.dist(A[:2], B[:2])
    best = None
    for ia in range(n + 1):
        for ib in range(n + 1):
            a, b = T * (lo + (hi - lo) * ia / n), T * (lo + (hi - lo) * ib / n)
            c = herm_coef(A, B, a, b)
            if pp3_lane_rmin(c, off, 150) < R_TARGET * 1.02:
                continue
            d = herm_dev(c, A, old_line, 25)
            if best is None or d < best[0]:
                best = (d, c)
    if best is None:
        return None
    c = best[1]
    return pp3_lane_rmin(c, off, 2000), c, pp3_arclen(c)


def herm_best(A, B, off, n=40, lo=0.15, hi=2.5):
    """A->B Hermite 중 차선 중심 최소 반경이 가장 큰 것(접선 크기 a·b = 직선거리 x [lo, hi] 격자). (반경, 계수, 호 길이)"""
    T = math.dist(A[:2], B[:2])
    best = None
    for ia in range(n + 1):
        for ib in range(n + 1):
            a = T * (lo + (hi - lo) * ia / n)
            b = T * (lo + (hi - lo) * ib / n)
            c = herm_coef(A, B, a, b)
            r = pp3_lane_rmin(c, off, 200)
            if best is None or r > best[0]:
                best = (r, c)
    # 가장 좋은 점 주변을 촘촘히
    r0, c0 = best
    a0, b0 = c0["bU"], math.hypot(*(lambda d: (d[0], d[1]))((c0["bU"] + 2 * c0["cU"] + 3 * c0["dU"], c0["bV"] + 2 * c0["cV"] + 3 * c0["dV"])))
    step = T * (hi - lo) / n
    for ia in range(-10, 11):
        for ib in range(-10, 11):
            a, b = a0 + step * ia / 10, b0 + step * ib / 10
            if a <= 0 or b <= 0:
                continue
            c = herm_coef(A, B, a, b)
            r = pp3_lane_rmin(c, off, 400)
            if r > best[0]:
                best = (r, c)
    r, c = best
    return pp3_lane_rmin(c, off, 2000), c, pp3_arclen(c)


def lal_any(A, B, off, lane_r=None):
    """A->B line+arc+line(호 하나, 양쪽 직선 >= 0) 가운데 차선 반경 최대. 해 없으면 None. (반경, 요소 목록)"""
    D = G.wrap(B[2] - A[2])
    if abs(D) < math.radians(0.5):
        return None
    dx, dy = B[0] - A[0], B[1] - A[1]
    u = dx * math.cos(A[2]) + dy * math.sin(A[2])
    v = -dx * math.sin(A[2]) + dy * math.cos(A[2])
    sg = 1 if D > 0 else -1
    v *= sg
    aD = abs(D)
    # 두 직선(A 방향, B 방향)의 교점까지 거리 d1(A 에서), d2(B 까지). 호 반경 R 이면 접선 길이 t = R tan(aD/2), L1 = d1 - t, L2 = d2 - t
    sD = math.sin(aD)
    if abs(sD) < 1e-9:
        return None
    d2 = v / sD
    d1 = u - d2 * math.cos(aD)
    if d1 <= 0 or d2 <= 0:
        return None
    tmax = min(d1, d2)
    R = tmax / math.tan(aD / 2)
    if lane_r is not None:
        Rn = lane_r + sg * off              # 차선 반경 = 기준선 반경 - sg*off (왼쪽 sg=+1)
        if Rn <= 0 or Rn > R:
            return None
        R = Rn
    k = sg / R
    t = R * math.tan(aD / 2)
    L1, L2 = d1 - t, d2 - t
    x1, y1 = A[0] + L1 * math.cos(A[2]), A[1] + L1 * math.sin(A[2])
    arc = f'<arc curvature="{k:.10f}"/>'
    x2, y2, h2 = end_pose(x1, y1, A[2], R * aD, arc)
    els = ([(A[0], A[1], A[2], L1, "<line/>")] if L1 > 1e-6 else []) + [(x1, y1, A[2], R * aD, arc)] + \
          ([(x2, y2, h2, L2, "<line/>")] if L2 > 1e-6 else [])
    f = 1 - k * off
    rl = abs(f / k) if f > 0 else 0.0
    e = end_pose(*els[-1][:4], els[-1][4])
    assert math.dist(e[:2], B[:2]) < 1e-5, (e, B)
    return rl, els


# ---------------- road 블록 ----------------

class RoadBlock:
    def __init__(self, m):
        self.m = m
        self.text = m.group(0)
        self.len_str = m.group(1)
        self.id = int(m.group(2))
        self.junction = int(m.group(3))
        self.geos = [dict(raw=g.group(0), ind=g.group(1), s=float(g.group(2)), x=float(g.group(3)), y=float(g.group(4)),
                          h=float(g.group(5)), L=float(g.group(6)), xml=g.group(7)) for g in GEO_RE.finditer(self.text)]
        el = ET.fromstring(self.text)
        self.road = G.Road(el)

    def start_pose(self, i):
        g = self.geos[i]
        return g["x"], g["y"], g["h"]

    def end_pose_of(self, j):
        g = self.geos[j]
        return end_pose(g["x"], g["y"], g["h"], g["L"], g["xml"])

    def rebuild(self, i, j, els):
        """요소 i..j 를 els[(x,y,h,L,xml)] 로 바꾼 새 블록 텍스트와 새 길이"""
        ind = self.geos[i]["ind"]
        s_ = self.geos[i]["s"]
        out = ""
        for x, y, h, L, xml in els:
            out += (f'{ind}<geometry s="{s_:.8f}" x="{x:.8f}" y="{y:.8f}" hdg="{h:.8f}" length="{L:.8f}">\n'
                    f'{ind}    {xml}\n{ind}</geometry>\n')
            s_ += L
        for g in self.geos[j + 1:]:
            out += g["raw"].replace(f's="{g["raw"].split(chr(34))[1]}"', f's="{s_:.8f}"', 1)
            s_ += g["L"]
        a0 = self.text.index(self.geos[i]["raw"])
        a1 = self.text.index(self.geos[-1]["raw"]) + len(self.geos[-1]["raw"])
        nb = self.text[:a0] + out + self.text[a1:]
        nb = nb.replace(f'length="{self.len_str}" id="{self.id}"', f'length="{s_:.8f}" id="{self.id}"', 1)
        return nb, s_


def road_of(text):
    return G.Road(ET.fromstring(text))


def lane_rmin(road):
    r = math.inf
    for s, x, y, h, k, fo in road.samples():
        r = min(r, 0.0 if fo else (abs(1 / k) if abs(k) > 1e-12 else math.inf))
    return r


def band_poly(road, step=0.25):
    """road 의 차선+인도 띠(기준선 옆 t 범위) 다각형"""
    from shapely.geometry import Polygon
    left_t = road.off + road.width / 2
    right_t = road.off - road.width / 2 - road.sidewalk_w
    bl, br = [], []
    for s, *_ in road.samples(step):
        x, y, h, k = road.ref_at(s)
        bl.append((x - left_t * math.sin(h), y + left_t * math.cos(h)))
        br.append((x - right_t * math.sin(h), y + right_t * math.cos(h)))
    return Polygon(bl + br[::-1]).buffer(0)


def deviation(old, new, step=0.25, extra_band=None):
    """옛/새 차선 중심 사이 최대 거리(양쪽 방향)와 새 차선 가장자리가 옛 차선+인도 띠 밖으로 나간 최대 거리"""
    from shapely.geometry import LineString, Point, Polygon
    so = [(x, y) for s, x, y, *_ in old.samples(step)]
    sn = [(x, y) for s, x, y, *_ in new.samples(step)]
    lo, ln = LineString(so), LineString(sn)
    dmax = max(max(lo.distance(Point(p)) for p in sn), max(ln.distance(Point(p)) for p in so))
    # 옛 띠: 차선 왼쪽 가장자리 ~ 인도 바깥(오른쪽). 기준선 옆 t: 차선 off±w/2, 인도는 그 오른쪽 sidewalk_w
    w = old.width
    left_t = old.off + w / 2
    right_t = old.off - w / 2 - old.sidewalk_w
    band_l, band_r = [], []
    for s, *_ in old.samples(step):
        x, y, h, k = old.ref_at(s)
        band_l.append((x - left_t * math.sin(h), y + left_t * math.cos(h)))
        band_r.append((x - right_t * math.sin(h), y + right_t * math.cos(h)))
    band = Polygon(band_l + band_r[::-1]).buffer(0)
    if extra_band is not None:
        band = band.union(extra_band).buffer(0.01).buffer(-0.01)
    out = 0.0
    for s, x, y, h, k, fo in new.samples(step):
        for sg in (1, -1):
            px, py = x - sg * new.width / 2 * math.sin(h), y + sg * new.width / 2 * math.cos(h)
            pt = Point(px, py)
            if not band.contains(pt):
                out = max(out, band.boundary.distance(pt) if not band.is_empty else 0)
    return dmax, out


# ---------------- 고칠 목록(정한 순서) ----------------
STAGES = [
    ("2 정문->후문 경로 나머지", [1391, 1948, 1253]),
    ("3 r1317 중문 진입", [1317]),
    ("4 북문 출발", [1415, 1312, 1672]),
    ("5 r1471", [1471]),
    ("6 나머지 30쌍 경로", [1190, 1624, 1727, 1220, 1398, 1436, 1480, 1729, 1825, 1946, 1281, 1159, 1812]),
]
SKIP = {}
ROUTE_ROADS = set()
STOP_S = {1278: 138.0, 1150: 15.1, 1415: 78.0, 1268: 47.0, 1405: 83.6, 1191: 31.9, 1320: 15.0, 1153: 22.0, 1281: 31.1, 1151: 23.1, 1279: 73.0}
BAND_TOL = 0.3      # 새 차선 가장자리가 옛 차선+인도 띠 밖으로 나가도 되는 거리(m)
DEV_SLACK = 0.3     # 벗어남 최소값에서 이만큼 더 벗어나는 것까지는 반경 큰 쪽을 고른다
GAP_MIN = 3.2       # 이어지지 않은 이웃 차선 중심까지 최소 거리. 차선 폭 3.35m 에서 0.15m 까지 겹침 허용(이보다 줄면 적용 안 함, 원래 더 가까웠으면 그대로 둠)


# ---------------- 고치기 ----------------

def fit_window(rb, i, j, A=None, B=None, prefer="radius"):
    """창 i..j 를 A->B 로. 후보 목록 [(반경, els, 방법)]: line-arc-line(호 최대, 차선 반경 목표+2% 호), Hermite(반경 최대, 덜 벗어남)"""
    from shapely.geometry import LineString
    A = A or rb.start_pose(i)
    B = B or rb.end_pose_of(j)
    off = rb.road.off
    old_line = LineString([rb.road.ref_at(s)[:2] for s in [rb.geos[i]["s"] + q * (rb.geos[j]["s"] + rb.geos[j]["L"] - rb.geos[i]["s"]) / 200 for q in range(201)]])
    cands = []
    la = lal_any(A, B, off)
    if la:
        cands.append((la[0], la[1], "line+arc+line(호 최대)"))
    for lr in (R_TARGET * 1.02, R_TARGET * 1.02 + 2 * abs(off)):   # 둘째 = 짝(안쪽 차선)도 목표를 넘기는 호
        lb = lal_any(A, B, off, lane_r=lr)
        if lb:
            cands.append((lb[0], lb[1], f"line+arc+line(차선 반경 {lr:.2f}m 호)"))
    r, c, L = herm_best(A, B, off)
    cands.append((r, [(A[0], A[1], A[2], L, pp3_xml(c))], "3차 Hermite(반경 최대)"))
    hm = herm_mindev(A, B, off, old_line)
    if hm:
        cands.append((hm[0], [(A[0], A[1], A[2], hm[2], pp3_xml(hm[1]))], "3차 Hermite(덜 벗어남)"))
    return cands


def find_pair(roads_all, rid):
    """같은 기준선을 거꾸로 쓰는 짝 반쪽 도로(일반 road, 양끝 0.5m 안)"""
    r = roads_all[rid]
    if r.junction != -1:
        return None
    a, b = r.ref_at(0)[:2], r.ref_at(r.length)[:2]
    for oid, o in roads_all.items():
        if oid != rid and o.junction == -1 and math.dist(o.ref_at(0)[:2], b) < 0.5 and math.dist(o.ref_at(o.length)[:2], a) < 0.5:
            return oid
    return None


def pair_fit(texts, pid, my_new, my_s0, my_s1, need_r):
    """짝 road pid 를 새 기준선(my_new 의 [my_s0, my_s1] 구간, 거꾸로)에 맞춰 고친다.
    짝 창 = 그 구간 양끝을 짝 기준선에 투영한 s 를 품는 요소 묶음. 후보 중 차선 반경 조건(need_r: 숫자면 그 이상, None 이면 안 접힘·안 나빠짐)을
    만족하고 새 기준선에서 가장 덜 벗어나는 것. 반환 dict 또는 None"""
    from shapely.geometry import LineString, Point
    rb = RoadBlock(ROAD_RE.search(texts[pid]))
    po = rb.road
    tgt = LineString([my_new.ref_at(my_s0 + (my_s1 - my_s0) * q / 300)[:2] for q in range(301)])
    def proj(pt):
        best = None
        for s in [po.length * q / 2000 for q in range(2001)]:
            d = math.dist(po.ref_at(s)[:2], pt)
            if best is None or d < best[0]:
                best = (d, s)
        return best[1]
    sa = proj(my_new.ref_at(my_s1)[:2])
    sb = proj(my_new.ref_at(my_s0)[:2])
    if sa > sb:
        sa, sb = sb, sa
    n = len(rb.geos)
    i = max([q for q in range(n) if rb.geos[q]["s"] <= sa + 1e-6] or [0])
    j = min([q for q in range(n) if rb.geos[q]["s"] + rb.geos[q]["L"] >= sb - 1e-6] or [n - 1])
    a_s, b_s = rb.geos[i]["s"], rb.geos[j]["s"] + rb.geos[j]["L"]
    loc_old = min([0.0 if fo else (abs(1 / k) if abs(k) > 1e-12 else math.inf) for s, x, y, h, k, fo in po.samples() if a_s - 1e-6 <= s <= b_s + 1e-6] or [math.inf])
    A, B = rb.start_pose(i), rb.end_pose_of(j)
    off = po.off
    opts = []
    for lr in (None, R_TARGET * 1.02, R_TARGET * 1.02 + 2 * abs(off), R_TARGET * 1.02 + 4 * abs(off)):
        la = lal_any(A, B, off, lane_r=lr)
        if la:
            opts.append((la[1], "line+arc+line"))
    T = math.dist(A[:2], B[:2])
    for ia in range(31):
        for ib in range(31):
            c = herm_coef(A, B, T * (0.1 + 2.4 * ia / 30), T * (0.1 + 2.4 * ib / 30))
            opts.append(([(A[0], A[1], A[2], None, c)], "3차 Hermite"))
    best = None
    for els, how in opts:
        if els[0][3] is None:          # Hermite: 길이·xml 채우기(나중에)
            c = els[0][4]
            rr = pp3_lane_rmin(c, off, 150)
            ok_r = (rr >= need_r) if need_r else (rr > 0 and rr >= min(loc_old, R_TARGET) - 1e-6)
            if not ok_r:
                continue
            d = herm_dev(c, A, tgt, 30)
            cand = (d, [(A[0], A[1], A[2], pp3_arclen(c), pp3_xml(c))], how)
        else:
            nb, _ = rb.rebuild(i, j, els)
            nw = road_of(nb)
            rr = min([0.0 if fo else (abs(1 / k) if abs(k) > 1e-12 else math.inf) for s, x, y, h, k, fo in nw.samples() if a_s - 1e-6 <= s <= a_s + sum(e[3] for e in els) + 1e-6] or [math.inf])
            ok_r = (rr >= need_r) if need_r else (rr > 0 and rr >= min(loc_old, R_TARGET) - 1e-6)
            if not ok_r:
                continue
            d = max(tgt.distance(Point(nw.ref_at(a_s + sum(e[3] for e in els) * q / 40)[:2])) for q in range(41))
            cand = (d, els, how)
        if best is None or cand[0] < best[0]:
            best = cand
    if best is None:
        return None
    d, els, how = best
    nb, nl = rb.rebuild(i, j, els)
    nw = road_of(nb)
    dv, outb = deviation(po, nw)
    rr_loc = min([0.0 if fo else (abs(1 / k) if abs(k) > 1e-12 else math.inf) for s, x, y, h, k, fo in nw.samples() if a_s - 1e-6 <= s <= a_s + sum(e[3] for e in els) + 1e-6] or [math.inf])
    return dict(rid=pid, i=i, j=j, how=how + "(짝 기준선 따라감)", text=nb, len_old=po.length, len_new=nl, dev=dv, out_band=outb,
                loc_old=loc_old, loc_new=rr_loc, r_old=lane_rmin(po), r_new=lane_rmin(nw), tgt_dev=d, s0=a_s, s1=b_s)


def try_road(texts, rid, windows=None, A=None, B_override=None, maxw=4, note="", min_s0=None, exclude=(), pair_ctx=None):
    """road rid 의 반경 미만 요소를 품는 창을 찾아 고친다. 결과 dict 또는 None"""
    rb = RoadBlock(ROAD_RE.search(texts[rid]))
    old = rb.road
    r_old = lane_rmin(old)
    n = len(rb.geos)
    # 나쁜 요소
    bad = []
    for q, g in enumerate(rb.geos):
        sub = road_of(texts[rid])
        rr = math.inf
        for s, x, y, h, k, fo in sub.samples():
            if g["s"] - 1e-6 <= s <= g["s"] + g["L"] + 1e-6:
                rr = min(rr, 0.0 if fo else (abs(1 / k) if abs(k) > 1e-12 else math.inf))
        if rr < R_TARGET:
            bad.append(q)
    if B_override is not None:
        bad = sorted(set(bad) | {n - 1})
    if not bad:
        return None
    # 나쁜 요소를 이어진 묶음으로 나눠 가장 나쁜(반경 최소) 묶음 하나만 고친다. 호출하는 쪽이 반복한다
    groups, cur = [], [bad[0]]
    for q in bad[1:]:
        if q == cur[-1] + 1:
            cur.append(q)
        else:
            groups.append(cur)
            cur = [q]
    groups.append(cur)
    if B_override is not None:
        groups = [g for g in groups if n - 1 in g] or groups
    def grp_r(gg):
        sub = road_of(texts[rid])
        a, b = rb.geos[gg[0]]["s"], rb.geos[gg[-1]]["s"] + rb.geos[gg[-1]]["L"]
        return min([0.0 if fo else (abs(1 / k) if abs(k) > 1e-12 else math.inf) for s, x, y, h, k, fo in sub.samples() if a - 1e-6 <= s <= b + 1e-6] or [math.inf])
    if min_s0 is not None:
        groups = [g for g in groups if rb.geos[g[0]]["s"] >= min_s0 - 1e-6 or any(rb.geos[q]["s"] >= min_s0 for q in g)] or groups
    groups = [g for g in groups if round(rb.geos[g[0]]["s"], 2) not in exclude]
    if not groups:
        return None
    gsel = min(groups, key=grp_r)
    gkey = round(rb.geos[gsel[0]]["s"], 2)
    lo, hi = min(gsel), max(gsel)
    wins = windows or [(i, j) for i in range(max(0, lo - maxw), lo + 1) for j in range(hi, min(n - 1, hi + maxw) + 1)]
    if min_s0 is not None:
        wins = [(i, j) for i, j in wins if rb.geos[i]["s"] >= min_s0]       # 승강장 s 를 안 옮기게 창은 승강장 뒤에서만
        if not wins:
            return None
    allc = []
    for i, j in wins:
        Bw = B_override if (B_override is not None and j == n - 1) else None
        for r, els, how in fit_window(rb, i, j, A=None, B=Bw):
            nb, nl = rb.rebuild(i, j, els)
            new = road_of(nb)
            rr = lane_rmin(new)
            dv, outb = deviation(old, new)
            a_s, b_s = rb.geos[i]["s"], rb.geos[j]["s"] + rb.geos[j]["L"]
            loc_old = min([0.0 if fo else (abs(1 / k) if abs(k) > 1e-12 else math.inf) for s, x, y, h, k, fo in old.samples() if a_s - 1e-6 <= s <= b_s + 1e-6] or [math.inf])
            nb_s = a_s + sum(e[3] for e in els)
            loc_new = min([0.0 if fo else (abs(1 / k) if abs(k) > 1e-12 else math.inf) for s, x, y, h, k, fo in new.samples() if a_s - 1e-6 <= s <= nb_s + 1e-6] or [math.inf])
            cand = dict(rid=rid, i=i, j=j, how=how, r_new=rr, r_old=r_old, loc_old=loc_old, loc_new=loc_new, dev=dv, out_band=outb,
                        len_old=old.length, len_new=nl, text=nb, s0=rb.geos[i]["s"], s1=rb.geos[j]["s"] + rb.geos[j]["L"], gkey=gkey)
            cand["meets"] = loc_new >= R_TARGET and rr >= min(r_old, R_TARGET) - 1e-6
            if pair_ctx and cand["meets"]:
                pid, pair_on_route = pair_ctx
                pc = pair_fit(texts, pid, new, a_s, nb_s, R_TARGET * 1.02 if pair_on_route else None)
                if pc is None:
                    cand["meets"] = False
                    cand["pair_fail"] = True
                else:
                    pb = band_poly(road_of(texts[pid]))
                    cand["dev"] = max(dv, pc["dev"])
                    _, cand["out_band"] = deviation(old, new, extra_band=pb)
                    _, pc["out_band"] = deviation(pc_old := road_of(texts[pid]), road_of(pc["text"]), extra_band=band_poly(old))
                    cand["out_band"] = max(cand["out_band"], pc["out_band"])
                    cand["pair"] = pc
            allc.append(cand)
    # 고르기: 목표 반경을 넘고 띠 안인 것 중 벗어남 최소 + 0.3m 안에서 반경이 가장 큰 것. 없으면 띠 안 -> 반경 큰 것
    good = [c for c in allc if c["meets"] and c["out_band"] <= BAND_TOL]
    if good:
        dmin = min(c["dev"] for c in good)
        return max([c for c in good if c["dev"] <= dmin + DEV_SLACK], key=lambda c: (min(c["loc_new"], 50.0), -c["dev"]))
    meets = [c for c in allc if c["meets"]]
    if meets:
        return min(meets, key=lambda c: (c["out_band"], c["dev"]))
    return max(allc, key=lambda c: (min(c["loc_new"], 50.0), -c["out_band"]))


def report(c, extra=""):
    P(f"  r{c['rid']}: 창 요소 {c['i']}~{c['j']}(s {c['s0']:.2f}~{c['s1']:.2f}) -> {c['how']}. 창 안 차선 반경 {c.get('loc_old', 0):.3f} -> {c.get('loc_new', 0):.3f}m, road 전체 {c['r_old']:.3f} -> {c['r_new']:.3f}m, "
      f"길이 {c['len_old']:.3f} -> {c['len_new']:.3f}m({c['len_new'] - c['len_old']:+.3f}), 옛 차선 중심에서 최대 {c['dev']:.2f}m, "
      f"새 차선이 옛 차선+인도 띠 밖 {c['out_band']:.2f}m{extra}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default="")
    ap.add_argument("--out", type=Path, default=OUT)
    args = ap.parse_args()
    text = SRC.read_text()
    assert hashlib.sha256(text.encode()).hexdigest().startswith("a17ab012e04c664f")
    ms = {int(m.group(2)): m for m in ROAD_RE.finditer(text)}
    texts = {rid: m.group(0) for rid, m in ms.items()}
    changed, results = {}, []
    only = {int(x.strip().lstrip("r")) for x in args.only.split(",") if x.strip()}

    # ---- 1) r1972 이음매(정문 출발 전체). r1393 끝 = r1972 시작 점만 r1392 기준선 연장 위로 옮긴다. 그 점에 붙은 것은 r1972 하나(j109)
    if not only or 1972 in only:
        P("\n[1] r1972 이음매 (r1393 끝 -> r1972 -> r1392)")
        r92 = road_of(texts[1392])
        x2, y2, h2, _ = r92.ref_at(0.0)
        rb72 = RoadBlock(ROAD_RE.search(texts[1972]))
        Lc = rb72.road.length
        p1 = (x2 - Lc * math.cos(h2), y2 - Lc * math.sin(h2), h2)
        old1 = road_of(texts[1393]).ref_at(road_of(texts[1393]).length)
        P(f"  옮긴 점(r1393 끝 = r1972 시작): ({old1[0]:.3f}, {old1[1]:.3f}) 방위 {math.degrees(old1[2]):.1f} -> ({p1[0]:.3f}, {p1[1]:.3f}) "
          f"방위 {math.degrees(h2):.1f}, 이동 {math.dist(old1[:2], p1[:2]):.3f}m")
        c93 = try_road(texts, 1393, B_override=p1)
        report(c93)
        nb72, nl72 = rb72.rebuild(0, 0, [(p1[0], p1[1], h2, Lc, "<line/>")])
        n72 = road_of(nb72)
        dv, outb = deviation(rb72.road, n72)
        P(f"  r1972: line {Lc:.3f}m 방위 {math.degrees(h2):.1f}(r1392 와 같음). 길이 그대로, 옛 차선 중심에서 최대 {dv:.2f}m, 띠 밖 {outb:.2f}m")
        texts[1393], texts[1972] = c93["text"], nb72
        changed[1393], changed[1972] = c93, dict(rid=1972, how="line(방위 r1392)", r_old=0, r_new=math.inf, dev=dv, out_band=outb,
                                                  len_old=Lc, len_new=Lc)
    # ---- 2) 나머지: 단계 순서대로 road 안 창 고치기(끝점 그대로). 30쌍·5노선 경로 위 4m 미만 road
    import s53_check as CK
    global ROUTE_ROADS
    ROUTE_ROADS = {r for v in G.routes().values() for r in v["roads"]}
    _, roads_all, juncs = G.load(SRC)
    for rid in changed:
        roads_all[rid] = road_of(texts[rid])
    for stage, rids in STAGES:
        P(f"\n[{stage}]")
        for rid in rids:
            if only and rid not in only:
                continue
            if rid in SKIP:
                P(f"  r{rid}: 안 고침 - {SKIP[rid]}")
                continue
            tried = set()
            for rep in range(6):          # 묶음마다 한 번씩
                pid = find_pair(roads_all, rid)
                pctx = (pid, pid in ROUTE_ROADS) if pid else None
                c = try_road(texts, rid, min_s0=STOP_S.get(rid, -1) + 2.0 if rid in STOP_S else None, exclude=tried, pair_ctx=pctx)
                if c is None:
                    if rep == 0:
                        P(f"  r{rid}: {R_TARGET}m 미만 요소 없음(앞 단계에서 풀림)")
                    break
                tried.add(c["gkey"])
                g0 = CK.min_gap(roads_all, juncs, rid)
                trial = dict(roads_all)
                trial[rid] = road_of(c["text"])
                if c.get("pair"):
                    trial[c["pair"]["rid"]] = road_of(c["pair"]["text"])
                g1 = CK.min_gap(trial, juncs, rid)
                ok = (c["loc_new"] > c["loc_old"] + 1e-6 and c["r_new"] >= c["r_old"] - 1e-6 and c["out_band"] <= BAND_TOL
                      and not (g1[0] < GAP_MIN and g1[0] < g0[0] - 0.05))
                extra = f", 이웃 차선 간격 {g0[0]:.2f} -> {g1[0]:.2f}m(r{g1[1]})" + ("" if ok else " -> 적용 안 함")
                report(c, extra)
                if c.get("pair"):
                    pc = c["pair"]
                    P(f"    짝 r{pc['rid']}: 창 요소 {pc['i']}~{pc['j']}(s {pc['s0']:.2f}~{pc['s1']:.2f}) -> {pc['how']}. 창 안 차선 반경 {pc['loc_old']:.3f} -> {pc['loc_new']:.3f}m, "
                      f"road 전체 {pc['r_old']:.3f} -> {pc['r_new']:.3f}m, 길이 {pc['len_old']:.3f} -> {pc['len_new']:.3f}m, 옛 차선 중심에서 최대 {pc['dev']:.2f}m, "
                      f"새 기준선과 차 {pc['tgt_dev']:.2f}m, 두 반쪽 띠 밖 {pc['out_band']:.2f}m")
                if ok:
                    texts[rid] = c["text"]
                    roads_all[rid] = trial[rid]
                    if c.get("pair"):
                        pc = c["pair"]
                        texts[pc["rid"]] = pc["text"]
                        roads_all[pc["rid"]] = trial[pc["rid"]]
                        if pc["rid"] in changed:
                            pc["r_old"] = changed[pc["rid"]]["r_old"]
                            pc["len_old"] = changed[pc["rid"]]["len_old"]
                        changed[pc["rid"]] = pc
                    if rid in changed:
                        c["r_old"] = changed[rid]["r_old"]
                        c["parts"] = changed[rid].get("parts", 1) + 1
                        c["len_old"] = changed[rid]["len_old"]
                    changed[rid] = c
                else:
                    results.append(dict(rid=rid, reason="조건 밖", r_best=c["r_new"], dev=c["dev"], out_band=c["out_band"], gap=g1[0]))
    out = []
    pos = 0
    for rid, m in sorted(ms.items(), key=lambda kv: kv[1].start()):
        out.append(text[pos:m.start()])
        out.append(texts[rid])
        pos = m.end()
    out.append(text[pos:])
    v = "".join(out)
    args.out.write_text(v)
    sha = hashlib.sha256(v.encode()).hexdigest()
    P(f"\n출력 {args.out.name}: {len(v.encode())} B, sha256 {sha[:16]}, 바뀐 road {sorted(changed)}")
    # 바이트 확인: 안 바뀐 road 는 원본과 같은가
    ms2 = {int(m.group(2)): m.group(0) for m in ROAD_RE.finditer(v)}
    same = sum(1 for rid in ms if ms2[rid] == ms[rid].group(0))
    P(f"road {len(ms2)}, 원본과 바이트 같은 road {same}, 다른 road {len(ms) - same}")
    LOG.write_text("\n".join(log) + "\n")
    (LOG.with_suffix(".json")).write_text(json.dumps({str(k): {kk: vv for kk, vv in c.items() if kk != "text"} for k, c in changed.items()},
                                                     ensure_ascii=False, indent=1, default=str))


if __name__ == "__main__":
    main()
