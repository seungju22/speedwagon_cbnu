"""10/11 공통: xodr planView 를 직접 읽어 기준선·lane -1 차선 중심의 위치·방위·곡률을 표본으로 낸다(서버 불필요).
- 요소: line, arc, paramPoly3(pRange normalized). 이 맵에 spiral·poly3 없음(10/09 GEO_RE 와 같은 세 가지)
- lane -1 중심 옆 거리 off = laneOffset.a - width(-1).a / 2 (왼쪽 +). 이 맵은 laneOffset·폭이 상수(a 만)이고 laneSection 1개
  laneOffset 3.35 road -> off +1.675(기준선 왼쪽), 없는 road -> off -1.675(오른쪽)
- 차선 중심 곡률 k_l = k / (1 - k*off). (1 - k*off) <= 0 이면 차선 중심이 뒤로 접힌다(10/08 Rc <= 0, '뒤집힘'): 반경 0 으로 센다
"""
import math
import re
import xml.etree.ElementTree as ET


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Road:
    def __init__(self, el):
        self.el = el
        self.id = int(el.get("id"))
        self.length = float(el.get("length"))
        self.junction = int(el.get("junction"))
        lo = el.find("lanes/laneOffset")
        a = float(lo.get("a")) if lo is not None else 0.0
        for k in "bcd":
            if lo is not None and abs(float(lo.get(k))) > 1e-12:
                raise ValueError(f"r{self.id} laneOffset 가 상수가 아님")
        w = el.find("lanes/laneSection/right/lane[@id='-1']/width")
        self.width = float(w.get("a")) if w is not None else 0.0
        self.off = a - self.width / 2
        self.has_sidewalk = el.find("lanes/laneSection/right/lane[@id='-2']") is not None
        sw = el.find("lanes/laneSection/right/lane[@id='-2']/width")
        self.sidewalk_w = float(sw.get("a")) if sw is not None else 0.0
        self.geos = []
        for g in el.findall("planView/geometry"):
            s, x, y, h, L = (float(g.get(k)) for k in ("s", "x", "y", "hdg", "length"))
            c = list(g)[0]
            if c.tag == "line":
                kind = ("line", None)
            elif c.tag == "arc":
                kind = ("arc", float(c.get("curvature")))
            elif c.tag == "paramPoly3":
                if c.get("pRange") != "normalized":
                    raise ValueError("pRange")
                kind = ("pp3", {k: float(c.get(k)) for k in ("aU", "bU", "cU", "dU", "aV", "bV", "cV", "dV")})
            else:
                raise ValueError(c.tag)
            self.geos.append((s, x, y, h, L, kind))
        lk = el.find("link")
        self.pred = self.succ = None
        if lk is not None:
            p, q = lk.find("predecessor"), lk.find("successor")
            self.pred = (p.get("elementType"), int(p.get("elementId")), p.get("contactPoint")) if p is not None else None
            self.succ = (q.get("elementType"), int(q.get("elementId")), q.get("contactPoint")) if q is not None else None

    def ref_at(self, s):
        """기준선 (x, y, hdg, k) at s"""
        s = min(max(s, 0.0), self.length)
        i = 0
        for j, g in enumerate(self.geos):
            if g[0] <= s + 1e-9:
                i = j
        return geo_eval(self.geos[i], s - self.geos[i][0])

    def lane_at(self, s):
        """lane -1 중심 (x, y, hdg, k_lane, folded)"""
        x, y, h, k = self.ref_at(s)
        o = self.off
        f = 1 - k * o
        return x - o * math.sin(h), y + o * math.cos(h), h, (k / f if abs(f) > 1e-12 else math.inf), f <= 0

    def samples(self, step=0.05):
        """[(s, x, y, h, k_lane, folded)] 기준선 s 표본(요소 경계 포함)"""
        ss = set()
        for g in self.geos:
            n = max(1, int(math.ceil(g[4] / step)))
            for q in range(n + 1):
                ss.add(round(g[0] + g[4] * q / n, 6))
        out = []
        for s in sorted(ss):
            if s > self.length + 1e-6:
                continue
            x, y, h, k, fo = self.lane_at(s)
            out.append((s, x, y, h, k, fo))
        return out


def geo_eval(g, t):
    s0, x, y, h, L, (kind, p) = g
    t = min(max(t, 0.0), L)
    if kind == "line":
        return x + t * math.cos(h), y + t * math.sin(h), h, 0.0
    if kind == "arc":
        k = p
        if abs(k) < 1e-12:
            return x + t * math.cos(h), y + t * math.sin(h), h, 0.0
        return x + (math.sin(h + k * t) - math.sin(h)) / k, y - (math.cos(h + k * t) - math.cos(h)) / k, h + k * t, k
    q = _p_of_s(p, L, t)
    u = p["aU"] + p["bU"] * q + p["cU"] * q * q + p["dU"] * q ** 3
    v = p["aV"] + p["bV"] * q + p["cV"] * q * q + p["dV"] * q ** 3
    du, dv = p["bU"] + 2 * p["cU"] * q + 3 * p["dU"] * q * q, p["bV"] + 2 * p["cV"] * q + 3 * p["dV"] * q * q
    ddu, ddv = 2 * p["cU"] + 6 * p["dU"] * q, 2 * p["cV"] + 6 * p["dV"] * q
    sp = math.hypot(du, dv)
    k = (du * ddv - dv * ddu) / sp ** 3 if sp > 1e-12 else 0.0
    return x + u * math.cos(h) - v * math.sin(h), y + u * math.sin(h) + v * math.cos(h), h + math.atan2(dv, du), k


_LUT = {}
STRETCH = False   # 대조 결과로 정함(아래 _p_of_s)


def _p_of_s(p, L, t, n=2000):
    """paramPoly3 의 호 길이 t -> 매개 p. CARLA 도 s 를 실제 호 길이로 본다(표본 대조로 확인, 2026-10-11).
    길이 L 은 xodr 의 length(호 길이와 다를 수 있음): 호 길이 표를 L 로 늘려 맞춘다(CARLA 와 같은지는 대조로 본다)"""
    key = (id(p), L)
    if key not in _LUT:
        cum = [0.0]
        px, py = p["aU"], p["aV"]
        for i in range(1, n + 1):
            q = i / n
            x = p["aU"] + p["bU"] * q + p["cU"] * q * q + p["dU"] * q ** 3
            y = p["aV"] + p["bV"] * q + p["cV"] * q * q + p["dV"] * q ** 3
            cum.append(cum[-1] + math.hypot(x - px, y - py))
            px, py = x, y
        _LUT[key] = cum
    cum = _LUT[key]
    tot = cum[-1]
    if tot <= 0:
        return 0.0
    target = min(t, tot) if STRETCH is False else (t / L * tot if L > 0 else 0.0)
    import bisect
    i = bisect.bisect_left(cum, target)
    if i <= 0:
        return 0.0
    if i >= len(cum):
        return 1.0
    a, b = cum[i - 1], cum[i]
    return (i - 1 + (target - a) / (b - a if b > a else 1)) / (len(cum) - 1)


def load(path):
    text = open(path).read()
    root = ET.fromstring(text)
    roads = {}
    for el in root.iter("road"):
        r = Road(el)
        roads[r.id] = r
    juncs = {}
    for j in root.iter("junction"):
        juncs[int(j.get("id"))] = [(int(c.get("incomingRoad")), int(c.get("connectingRoad")), c.get("contactPoint")) for c in j.iter("connection")]
    return text, roads, juncs


def routes():
    """test_drive.py ROUTES_V3S46 (읽기만). {이름: dict(roads, start_s, end_s)}"""
    import sys
    from pathlib import Path
    base = Path(__file__).resolve().parent.parent
    sys.path.insert(0, str(base / "tests"))
    argv, sys.argv = sys.argv, sys.argv[:1]
    try:
        import test_drive as td
    finally:
        sys.argv = argv
    return td.ROUTES_V3S46
