#!/usr/bin/env python3
# 10/09 3번: v3 후보 s46 만들기. s45 후보에서 꺾임 9곳(10/09 [F] 목록)만 r1329(10/09)와 같은 방법으로 고친다
# 방법: 꺾임 요소를 포함하는 이어진 planView 요소 묶음(창)을 line + arc + line 하나로 바꾼다
#   창 양끝 위치·방위 그대로, 창 밖 요소는 바이트 그대로, road 길이 변화 ±0.1m 안(DL_MAX), link·lanes 그대로
#   창은 꺾임 요소를 품는 것 중 고친 뒤 road 전체 차선 중심 최소 반경이 가장 큰 것(창 크기는 꺾임 요소 앞뒤 MAXW 개까지)
#   길이는 old-0.099 / old / old+0.099 중 반경이 가장 큰 것
#   회차 r1879 는 고치지 않는다(회차 형상 수정 금지)
# 입력: map/maps/cbnu_campus_v3_candidate_s45.xodr (읽기만)
# 출력: map/maps/cbnu_campus_v3_candidate_s46.xodr, map/docs/logs/s46_build_v3.log
import hashlib
import math
import re
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
SRC = BASE / "maps/cbnu_campus_v3_candidate_s45.xodr"
OUT = BASE / "maps/cbnu_campus_v3_candidate_s46.xodr"
LOG = BASE / "docs/logs/s46_build_v3.log"
KINKS = [1190, 1220, 1317, 1320, 1367, 1415, 1825, 1871, 1879]
SKIP = {1879: "회차 연결로(j88 중문 막다른 끝). 지시 '회차 형상 수정 금지'"}
OFF = -1.675          # lane -1 중심 = 기준선 오른쪽 1.675m(이 맵 lanes: laneOffset 없음, 폭 3.35)
DL_MAX = 0.099
MAXW = 4
log = []


def P(*a):
    s = " ".join(str(x) for x in a)
    log.append(s)
    print(s)


text = SRC.read_text()
assert hashlib.sha256(text.encode()).hexdigest().startswith("d9b9c504e93e2255")
road_re = re.compile(r'^ *<road name="[^"]*" length="([^"]*)" id="(\d+)" junction="(-?\d+)">.*?</road>\n', re.S | re.M)
roads = {int(m.group(2)): m for m in road_re.finditer(text)}
GEO_RE = re.compile(r'( *)<geometry s="([\d.]+)" x="([-\d.]+)" y="([-\d.]+)" hdg="([-\d.]+)" length="([\d.]+)">\s*(<line/>|<arc [^>]*/>|<paramPoly3 [^>]*/>)\s*</geometry>\n')


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def pp_of(kind):
    return {k: float(v) for k, v in re.findall(r'(\w+)="([-\d.e]+)"', kind) if k != "pRange"}


def pose_on(kind, x, y, h, L, t):
    if kind == "<line/>":
        return x + t * math.cos(h), y + t * math.sin(h), h
    if kind.startswith("<arc"):
        k = float(re.search(r'curvature="([-\d.e]+)"', kind).group(1))
        if abs(k) < 1e-12:
            return x + t * math.cos(h), y + t * math.sin(h), h
        return x + (math.sin(h + k * t) - math.sin(h)) / k, y - (math.cos(h + k * t) - math.cos(h)) / k, h + k * t
    pp = pp_of(kind)
    p = t / L
    u = pp["aU"] + pp["bU"] * p + pp["cU"] * p * p + pp["dU"] * p ** 3
    v = pp["aV"] + pp["bV"] * p + pp["cV"] * p * p + pp["dV"] * p ** 3
    du, dv = pp["bU"] + 2 * pp["cU"] * p + 3 * pp["dU"] * p * p, pp["bV"] + 2 * pp["cV"] * p + 3 * pp["dV"] * p * p
    return x + u * math.cos(h) - v * math.sin(h), y + u * math.sin(h) + v * math.cos(h), h + math.atan2(dv, du)


def lane_r_el(kind, L):
    """요소의 차선 중심(기준선 오른쪽 1.675m) 최소 반경(해석식, s42_common.lane_radius 와 같은 식)"""
    if kind == "<line/>":
        return math.inf
    if kind.startswith("<arc"):
        ks = [float(re.search(r'curvature="([-\d.e]+)"', kind).group(1))]
    else:
        pp = pp_of(kind)
        ks = []
        for i in range(2001):
            p = i / 2000
            du = pp["bU"] + 2 * pp["cU"] * p + 3 * pp["dU"] * p * p
            dv = pp["bV"] + 2 * pp["cV"] * p + 3 * pp["dV"] * p * p
            ddu, ddv = 2 * pp["cU"] + 6 * pp["dU"] * p, 2 * pp["cV"] + 6 * pp["dV"] * p
            sp = math.hypot(du, dv)
            if sp > 1e-9:
                ks.append((du * ddv - dv * ddu) / sp ** 3)
    r = math.inf
    for k in ks:
        if abs(k) > 1e-12:
            r = min(r, abs((1 - k * OFF) / k))
    return r


def lal(A, B, Lt):
    """A·B=(x,y,h) 를 잇는 line-arc-line, 전체 길이 Lt. (L1, R, sign, L2) 또는 None"""
    D = wrap(B[2] - A[2])
    if abs(D) < math.radians(1.0):
        return None
    dx, dy = B[0] - A[0], B[1] - A[1]
    u = dx * math.cos(A[2]) + dy * math.sin(A[2])
    v = -dx * math.sin(A[2]) + dy * math.cos(A[2])
    sg = 1 if D > 0 else -1
    v *= sg
    aD = abs(D)
    sD, cD = math.sin(aD), math.cos(aD)
    a2 = (1 - cD) / sD
    den = aD - sD - a2 * (1 - cD)
    if abs(den) < 1e-12:
        return None
    R = (Lt - u - v * (1 - cD) / sD) / den
    L2 = v / sD - R * a2
    L1 = u - R * sD - L2 * cD
    if R <= 0 or L1 < 0 or L2 < 0:
        return None
    return L1, R, sg, L2


def lal_els(A, sol):
    L1, R, sg, L2 = sol
    k = sg / R
    x1, y1 = A[0] + L1 * math.cos(A[2]), A[1] + L1 * math.sin(A[2])
    D = R * abs(wrap(0)) if False else None
    return k, (x1, y1)


def herm_coef(A, B, a, b):
    """A->B 3차 Hermite(paramPoly3 normalized, A 기준 좌표). 접선 크기 a·b(m)"""
    dx, dy = B[0] - A[0], B[1] - A[1]
    ex, ey = dx * math.cos(A[2]) + dy * math.sin(A[2]), -dx * math.sin(A[2]) + dy * math.cos(A[2])
    d1 = B[2] - A[2]
    t0u, t0v, t1u, t1v = a, 0.0, b * math.cos(d1), b * math.sin(d1)
    return dict(aU=0.0, bU=t0u, cU=3 * ex - 2 * t0u - t1u, dU=-2 * ex + t0u + t1u,
                aV=0.0, bV=t0v, cV=3 * ey - 2 * t0v - t1v, dV=-2 * ey + t0v + t1v)


def herm_len(c, n=800):
    L, prev = 0.0, (0.0, 0.0)
    for i in range(1, n + 1):
        p = i / n
        u = c["bU"] * p + c["cU"] * p * p + c["dU"] * p ** 3
        v = c["bV"] * p + c["cV"] * p * p + c["dV"] * p ** 3
        L += math.dist(prev, (u, v))
        prev = (u, v)
    return L


def herm_kind(c):
    return ('<paramPoly3 ' + " ".join(f'{k}="{c[k]:.8f}"' for k in ("aU", "bU", "cU", "dU", "aV", "bV", "cV", "dV"))
            + ' pRange="normalized"/>')


def herm_fit(A, B, Lt_lo, Lt_hi):
    """길이가 [Lt_lo, Lt_hi] 안인 Hermite 중 차선 중심 최소 반경이 가장 큰 것. (반경, 계수, 길이) 또는 None"""
    T = math.dist(A[:2], B[:2])
    best = None
    for ia in range(1, 61):
        for ib in range(1, 61):
            c = herm_coef(A, B, T * ia / 20, T * ib / 20)
            L = herm_len(c, 200)
            if not (Lt_lo - 0.05 <= L <= Lt_hi + 0.05):
                continue
            L = herm_len(c)
            if not (Lt_lo <= L <= Lt_hi):
                continue
            r = lane_r_el(herm_kind(c), L)
            if best is None or r > best[0]:
                best = (r, c, L)
    return best


DEV_MAX = 0.2          # 고친 기준선이 옛 기준선에서 벗어나는 최대 거리(m). 짝 반쪽 도로와 차선이 겹치지 않게(1.0m 로 하면 r1367·r1320 이 짝과 0.6~1.0m 겹침, 10/09 확인)
OK_R = 1.0             # 고친 뒤 road 전체 차선 중심 최소 반경이 이 이상이어야 적용(10/09 목록 기준 1m)


def sample_els(els, step=0.25):
    pts = []
    for x, y, h, L, kind in els:
        n = max(2, int(L / step) + 1)
        for q in range(n):
            pts.append(pose_on(kind, x, y, h, L, L * q / (n - 1))[:2])
    return pts


def dev(old_pts, new_pts):
    from shapely.geometry import LineString, Point
    a, b = LineString(old_pts), LineString(new_pts)
    return max(max(a.distance(Point(p)) for p in new_pts), max(b.distance(Point(p)) for p in old_pts))


fixed = {}
summary = []
for rid in KINKS:
    m = roads[rid]
    blk = m.group(0)
    old_len = float(m.group(1))
    gl = [(g.group(0), float(g.group(2)), float(g.group(3)), float(g.group(4)), float(g.group(5)), float(g.group(6)), g.group(7), g.group(1))
          for g in GEO_RE.finditer(blk)]
    radii = [lane_r_el(g[6], g[5]) for g in gl]
    before = min(radii)
    bad = [i for i, r in enumerate(radii) if r < 1.0]
    P(f"\nr{rid} junction {m.group(3)}, 길이 {old_len:.3f}m, 요소 {len(gl)}, 차선 중심 최소 반경 {before:.3f}m(이 스크립트 식), 1m 미만 요소 {[(i, round(gl[i][1], 2)) for i in bad]}")
    if rid in SKIP:
        P(f"  고치지 않음: {SKIP[rid]}")
        summary.append((rid, before, None, 0.0, "못 고침: " + SKIP[rid]))
        continue
    lo, hi = min(bad), max(bad)
    cands = []          # (반경, -dev, -창크기, 새 요소 목록, i, j, dl, 방법)
    best_any = None     # 거리 제한 없이 가장 큰 반경(보고용)
    for i in range(max(0, lo - MAXW), lo + 1):
        for j in range(hi, min(len(gl) - 1, hi + MAXW) + 1):
            A = (gl[i][2], gl[i][3], gl[i][4])
            last = gl[j]
            B = pose_on(last[6], last[2], last[3], last[4], last[5], last[5])
            seg = sum(g[5] for g in gl[i:j + 1])
            old_pts = sample_els([(g[2], g[3], g[4], g[5], g[6]) for g in gl[i:j + 1]])
            rest = min([radii[q] for q in range(len(gl)) if q < i or q > j] + [math.inf])
            opts = []
            for dl in (-DL_MAX, 0.0, DL_MAX):
                sol = lal(A, B, seg + dl)
                if not sol:
                    continue
                L1, R, sg, L2 = sol
                k = sg / R
                D = wrap(B[2] - A[2])
                x1, y1 = A[0] + L1 * math.cos(A[2]), A[1] + L1 * math.sin(A[2])
                x2, y2, h2 = pose_on(f'<arc curvature="{k}"/>', x1, y1, A[2], R * abs(D), R * abs(D))
                els = ([(A[0], A[1], A[2], L1, "<line/>")] if L1 > 1e-6 else []) + [(x1, y1, A[2], R * abs(D), f'<arc curvature="{k:.8f}"/>')] \
                    + ([(x2, y2, h2, L2, "<line/>")] if L2 > 1e-6 else [])
                e = pose_on(els[-1][4], *els[-1][:4], els[-1][3])
                assert math.dist(e[:2], B[:2]) < 1e-6 and abs(wrap(e[2] - B[2])) < 1e-6
                opts.append((abs((1 - k * OFF) / k), els, dl, f"line {L1:.3f} + arc 기준선 반경 {R:.3f}({'왼' if sg > 0 else '오른'}쪽) + line {L2:.3f}"))
            if not opts:
                hf = herm_fit(A, B, seg - DL_MAX, seg + DL_MAX)
                if hf:
                    rh, c, Lh = hf
                    e = pose_on(herm_kind(c), A[0], A[1], A[2], Lh, Lh)
                    assert math.dist(e[:2], B[:2]) < 1e-6 and abs(wrap(e[2] - B[2])) < 1e-6
                    opts.append((rh, [(A[0], A[1], A[2], Lh, herm_kind(c))], Lh - seg, "3차 Hermite(line-arc-line 해 없음)"))
            for rl, els, dl, how in opts:
                sc = min(rl, rest)
                dv = dev(old_pts, sample_els(els))
                if best_any is None or sc > best_any[0]:
                    best_any = (sc, dv, i, j, how)
                if dv <= DEV_MAX:
                    cands.append((sc, -dv, -(j - i), els, i, j, dl, how, seg))
    if not cands or max(c[0] for c in cands) < OK_R:
        why = f"길이 ±0.1m·옛 기준선 {DEV_MAX}m 안에서 차선 반경 1m 이상 해 없음"
        if best_any:
            why += f" (거리 제한 없이 최대 {best_any[0]:.3f}m, 그때 벗어남 {best_any[1]:.2f}m, {best_any[4]})"
        if cands:
            cb = max(cands)
            why += f"; {DEV_MAX}m 안 최대 {cb[0]:.3f}m"
        P(f"  못 고침: {why}")
        summary.append((rid, before, None, 0.0, "못 고침: " + why))
        continue
    sc, ndv, _, els, i, j, dl, how, seg = max(cands)
    ind = gl[i][7]
    out_geo, s_ = "", gl[i][1]
    for x, y, h, L, kind in els:
        out_geo += (f'{ind}<geometry s="{s_:.8f}" x="{x:.8f}" y="{y:.8f}" hdg="{h:.8f}" length="{L:.8f}">\n'
                    f'{ind}    {kind}\n{ind}</geometry>\n')
        s_ += L
    tail = "".join(g[0] if abs(dl) < 1e-12 else g[0].replace(f's="{g[1]:.8f}"', f's="{g[1] + dl:.8f}"', 1) for g in gl[j + 1:])
    a0 = blk.index(gl[i][0])
    a1 = blk.index(gl[-1][0]) + len(gl[-1][0])
    nb = blk[:a0] + out_geo + tail + blk[a1:]
    new_len = old_len + dl
    nb = nb.replace(f'length="{m.group(1)}" id="{rid}"', f'length="{new_len:.8f}" id="{rid}"', 1)
    A = (gl[i][2], gl[i][3], gl[i][4])
    last = gl[j]
    B = pose_on(last[6], last[2], last[3], last[4], last[5], last[5])
    P(f"  창 요소 {i}~{j} (s={gl[i][1]:.3f}~{gl[i][1] + seg:.3f}, {seg:.3f}m, {j - i + 1}개) 방위 변화 {math.degrees(wrap(B[2] - A[2])):+.1f}도")
    P(f"  -> {how}, 길이 변화 {dl:+.3f}m, 옛 기준선에서 최대 {-ndv:.2f}m")
    P(f"  차선 중심 최소 반경 {before:.3f} -> {sc:.3f}m(road 전체), 길이 {old_len:.3f} -> {new_len:.3f}m")
    fixed[rid] = nb
    summary.append((rid, before, sc, dl, f"옛 기준선에서 최대 {-ndv:.2f}m"))

out, pos = [], 0
for rid, m in sorted(roads.items(), key=lambda kv: kv[1].start()):
    out.append(text[pos:m.start()])
    out.append(fixed.get(rid, m.group(0)))
    pos = m.end()
out.append(text[pos:])
v = "".join(out)
OUT.write_text(v)
sha = hashlib.sha256(v.encode()).hexdigest()
P(f"\n출력 {OUT.name}: road {len(re.findall(r'<road name=', v))}, junction {len(re.findall(r'<junction name=', v))}, {len(v.encode())} B, sha256 {sha}")
P("요약(road, 전, 후, 길이 차)")
for rid, b, a, dl, note in summary:
    P(f"  r{rid}: {b:.3f} -> {('%.3f' % a) if a is not None else '-'}m, {dl:+.3f}m {note}")
LOG.write_text("\n".join(log) + "\n")
