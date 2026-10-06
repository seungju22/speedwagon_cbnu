#!/usr/bin/env python3
# 맵세션11 공용 라이브러리 (평활화 단계·검증 공용). CARLA 서버·carla 모듈 불필요, 읽기전용.
#
# 지표 (설계: map/docs/map_session11_design_smoothing.md)
#   해석적 순간 곡률은 변환기(paramPoly3) 접합부 스파이크라 판정에 부적합하다.
#   CARLA 메시는 vertex_distance=2.0m 로 샘플링하므로 기준선 곡률을 2m 창 평균으로 잰다:
#     kappa_w(s) = (theta(s+1m) - theta(s-1m)) / 2m,   R2 = 1/|kappa_w|
#   안쪽 인도 바깥 가장자리(기준선에서 t_far)의 접힘 계수 f = 1 - kappa_w * t_far
#   (f<=0 접힘, 안쪽 = kappa_w*t_far > 0). 기준선~인도 바깥 6.15m = 차선 3.35 + 인도 2.80.
#   수용 기준 f >= 0.30 (R2 >= 6.15/0.70 = 8.79m) 은 공학적 선택값이다.
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_geometry import parse_roads, parse_junctions          # noqa: E402
from analyze_wedge_s11 import parse_lanes, lane_edges, sample_road, find_twins  # noqa: E402
import geo_calibrate as G                                            # noqa: E402

WINDOW_M = 2.0
STEP_M = 0.25
F_MIN = 0.30
SIDEWALK_FAR_M = 6.15

# OSM 위경도 -> ENU(m) 등장방형 근사 기준점 (캠퍼스 중심 부근, 어느 값이든 상대 형상만 쓴다)
LAT0, LON0 = 36.6318, 127.4532
M_LAT = 110574.0
M_LON = 111320.0 * math.cos(math.radians(LAT0))


def to_en(lat, lon):
    return (lon - LON0) * M_LON, (lat - LAT0) * M_LAT


def to_ll(e, n):
    return LAT0 + n / M_LAT, LON0 + e / M_LON


# ---------------- OSM ----------------
def load_osm(path_or_text, is_text=False):
    root = ET.fromstring(path_or_text) if is_text else ET.parse(path_or_text).getroot()
    nodes = {}
    tagged = set()
    for n in root.findall("node"):
        nodes[n.get("id")] = to_en(float(n.get("lat")), float(n.get("lon")))
        if n.findall("tag"):
            tagged.add(n.get("id"))
    ways = {}
    for w in root.findall("way"):
        ways[w.get("id")] = [nd.get("ref") for nd in w.findall("nd")]
    return root, nodes, ways, tagged


def build_segs(nodes, ways):
    segs = []
    for wid in sorted(ways):
        nds = ways[wid]
        for i in range(len(nds) - 1):
            a, b = nodes[nds[i]], nodes[nds[i + 1]]
            segs.append((wid, i, a[0], a[1], b[0], b[1]))
    S = np.array([[s[2], s[3], s[4], s[5]] for s in segs], dtype=float)
    return segs, S


def nearest_on_segs(S, P):
    """점 P(N,2) -> (거리, 세그먼트 index, 세그먼트 위 매개변수 u)"""
    A, B = S[:, 0:2], S[:, 2:4]
    D = B - A
    L2 = (D ** 2).sum(1) + 1e-12
    bd = np.full(len(P), 1e18)
    bi = np.zeros(len(P), int)
    bu = np.zeros(len(P))
    for k0 in range(0, len(P), 1000):
        p = P[k0:k0 + 1000][:, None, :]
        u = np.clip(((p - A) * D).sum(2) / L2, 0, 1)
        q = A + u[..., None] * D
        d = np.linalg.norm(p - q, axis=2)
        j = d.argmin(1)
        ar = np.arange(len(j))
        bd[k0:k0 + 1000], bi[k0:k0 + 1000], bu[k0:k0 + 1000] = d[ar, j], j, u[ar, j]
    return bd, bi, bu


def path_cum(nodes, nds):
    c = [0.0]
    for k in range(1, len(nds)):
        c.append(c[-1] + math.dist(nodes[nds[k]], nodes[nds[k - 1]]))
    return c


def turn_deg(p0, p1, p2):
    a = (p1[0] - p0[0], p1[1] - p0[1])
    b = (p2[0] - p1[0], p2[1] - p1[1])
    return math.degrees(math.atan2(a[0] * b[1] - a[1] * b[0], a[0] * b[0] + a[1] * b[1]))


# ---------------- xodr <-> OSM 정렬 ----------------
class Alignment:
    """xodr 좌표 -> OSM ENU 유사변환(회전·등방스케일·평행이동). ICP 로 추정."""

    def __init__(self, R, s, t, median):
        self.R, self.s, self.t, self.median = R, s, t, median
        self.Ri = np.linalg.inv(R)

    def fwd(self, X):
        return (self.s * (self.R @ np.asarray(X, float).T)).T + self.t

    def inv_pt(self, e, n):
        return self.Ri @ ((np.array([e, n]) - self.t) / self.s)


def align_icp(roads, junctions, nodes, S, iters=10):
    jxy = G.junction_xodr_locations(roads, junctions)
    src, dst = [], []
    for jid, j in junctions.items():
        if j["name"] in nodes and jid in jxy:
            src.append(jxy[jid])
            dst.append(nodes[j["name"]])
    R, s, t, _, _ = G.fit_similarity(src, dst)
    pts = []
    for rid in sorted(roads, key=int):
        r = roads[rid]
        if r["junction"] != "-1":
            continue
        smp = sample_road(r, STEP_M)
        for i in range(0, len(smp), 8):
            pts.append((smp[i]["x"], smp[i]["y"]))
    X = np.array(pts)
    d = None
    for _ in range(iters):
        P = (s * (R @ X.T)).T + t
        d, bi, bu = nearest_on_segs(S, P)
        keep = d < np.percentile(d, 80)
        Q = S[bi][:, 0:2] + bu[:, None] * (S[bi][:, 2:4] - S[bi][:, 0:2])
        R, s, t, _, _ = G.fit_similarity(X[keep], Q[keep])
    P = (s * (R @ X.T)).T + t
    d, _, _ = nearest_on_segs(S, P)
    return Alignment(R, s, t, float(np.median(d)))


# ---------------- 2m 창 스캔 ----------------
def load_xodr(text_or_path, is_text=False):
    root = ET.fromstring(text_or_path) if is_text else ET.parse(text_or_path).getroot()
    roads = parse_roads(root)
    juncs = parse_junctions(root)
    elems = {r.get("id"): r for r in root.iter("road")}
    return root, roads, juncs, elems


def scan_sites(roads, elems, win=WINDOW_M):
    """인도 lane 이 있는 plain road 마다 2m 창 지표. 반환: [(rid, s, f, R2, x, y)] """
    m = int(round(win / 2 / STEP_M))
    out = []
    for rid in sorted(roads, key=int):
        r = roads[rid]
        if r["junction"] != "-1":
            continue
        offs, secs = parse_lanes(elems[rid])
        if not any(e["type"] == "sidewalk" for e in lane_edges(offs, secs, 0.0)):
            continue
        S_ = sample_road(r, STEP_M)
        for i in range(m, len(S_) - m):
            ds = S_[i + m]["s"] - S_[i - m]["s"]
            kw = (S_[i + m]["hdg"] - S_[i - m]["hdg"]) / ds
            e = [e for e in lane_edges(offs, secs, S_[i]["s"]) if e["type"] == "sidewalk"]
            if not e:
                continue
            tf = e[0]["t_out"]
            f = 1 - kw * tf if kw * tf > 0 else 1.0
            R2 = 1 / abs(kw) if abs(kw) > 1e-9 else 1e9
            out.append((rid, S_[i]["s"], f, R2, S_[i]["x"], S_[i]["y"]))
    return out


def group_sites(sites, fmax=F_MIN):
    """f < fmax 인 연속 샘플을 road 별 구간으로 묶는다. 최악점(smin, f, R2, x, y)을 대표로."""
    out, cur = [], None
    for rid, s_, f, R2, x, y in sites:
        if f < fmax:
            if cur and cur["road"] == rid and s_ - cur["s1"] <= 0.6:
                cur["s1"] = s_
                if f < cur["f"]:
                    cur.update(f=f, R2=R2, smin=s_, x=x, y=y)
            else:
                if cur:
                    out.append(cur)
                cur = {"road": rid, "s0": s_, "s1": s_, "f": f, "R2": R2, "smin": s_, "x": x, "y": y}
        elif cur:
            out.append(cur)
            cur = None
    if cur:
        out.append(cur)
    return out


def worst_near(sites, point_xodr, radius=5.0):
    """xodr 좌표 점 반경 안의 최악 f 샘플. 없으면 None."""
    near = [(f, rid, s_, R2) for rid, s_, f, R2, x, y in sites
            if (x - point_xodr[0]) ** 2 + (y - point_xodr[1]) ** 2 <= radius ** 2]
    return min(near) if near else None
