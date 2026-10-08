#!/usr/bin/env python3
# 맵세션34: xodr 기준선 곡률과 차선 가장자리 반경(읽기전용, 서버 없음, xodr 수정 없음)
# 정의(OpenDRIVE 1.4 좌표: t 는 기준선 왼쪽이 +, lane -1 은 t in [-w, 0])
#   곡률 k(s): 부호 있음(+ = 왼쪽으로 꺾임). 기준선 반경 R_ref = 1/|k|
#   기준선에서 t 만큼 떨어진 평행선의 곡률 중심까지 거리 r(t) = (1 - k t) / |k|.  1 - k t <= 0 이면 그 선이 뒤집힘(메시 자기교차)
#   lane -1 안쪽 가장자리 반경 = min(r(0), r(-w)) = 지시문의 "차선 중심 R - 반차선폭" 과 같은 값
#   (차선 중심 반경 r(-w/2) 에서 w/2 를 뺀 값. k<0 이면 r(-w), k>0 이면 r(0))
#   인도까지 포함한 바깥 끝 t=-(w+ws) 도 따로 계산(CARLA 는 인도 메시도 만든다)
# 폭은 각 road 의 첫 laneSection lane -1 / -2 width 레코드(a + b ds + c ds^2 + d ds^3)에서 읽는다. laneOffset 도 읽어 t 에 더한다
# 기하: line / arc / spiral(곡률 선형) / paramPoly3(normalized 또는 arcLength). poly3 는 이 맵에 없음(있으면 경고)
# 기하 이음매: 앞 기하 끝 위치·방위와 다음 기하 시작(x, y, hdg) 차이도 기록(방위 계단 = 곡률 무한대)
# 사용: python3 s34_curvature.py <xodr> [road id ...]   (id 없으면 전 road 요약 CSV 를 stdout)
import math
import sys
import xml.etree.ElementTree as ET

DS = 0.02   # 표본 간격(m, 근사 호장)


def poly(c, ds):
    return c[0] + c[1] * ds + c[2] * ds * ds + c[3] * ds ** 3


def recs(el, tag):
    out = []
    for w in el.findall(tag):
        so = float(w.get("sOffset", w.get("s", 0)))
        out.append((so, [float(w.get(k)) for k in "abcd"]))
    return sorted(out)


def at(rs, s):
    cur = None
    for so, c in rs:
        if so <= s + 1e-9:
            cur = (so, c)
    if cur is None:
        return 0.0
    return poly(cur[1], s - cur[0])


def geom_samples(g):
    """한 geometry 의 (s_local, x, y, hdg, k) 표본. paramPoly3 는 실제 호장도 함께 반환"""
    s0, x0, y0, h0, L = (float(g.get(k)) for k in ("s", "x", "y", "hdg", "length"))
    e = list(g)[0]
    tag = e.tag
    out = []
    ch, sh = math.cos(h0), math.sin(h0)
    if tag == "line":
        n = max(2, int(L / DS) + 1)
        for i in range(n + 1):
            u = L * i / n
            out.append((u, x0 + u * ch, y0 + u * sh, h0, 0.0))
        return tag, out, L
    if tag == "arc":
        k = float(e.get("curvature"))
        n = max(2, int(L / DS) + 1)
        for i in range(n + 1):
            u = L * i / n
            h = h0 + k * u
            if abs(k) < 1e-12:
                x, y = x0 + u * ch, y0 + u * sh
            else:
                x = x0 + (math.sin(h) - sh) / k
                y = y0 - (math.cos(h) - ch) / k
            out.append((u, x, y, h, k))
        return tag, out, L
    if tag == "spiral":
        k0, k1 = float(e.get("curvStart")), float(e.get("curvEnd"))
        n = max(2, int(L / DS) + 1)
        x, y, h = x0, y0, h0
        du = L / n
        out.append((0.0, x, y, h, k0))
        for i in range(1, n + 1):
            u = du * i
            km = k0 + (k1 - k0) * (u - du / 2) / L
            hm = h + km * du / 2
            x += du * math.cos(hm)
            y += du * math.sin(hm)
            h += km * du
            out.append((u, x, y, h, k0 + (k1 - k0) * u / L))
        return tag, out, L
    if tag == "paramPoly3":
        U = [float(e.get(k)) for k in ("aU", "bU", "cU", "dU")]
        V = [float(e.get(k)) for k in ("aV", "bV", "cV", "dV")]
        pmax = 1.0 if e.get("pRange", "normalized") == "normalized" else L
        n = max(50, int(L / DS) * 4)
        pts = []
        arc = 0.0
        prev = None
        for i in range(n + 1):
            p = pmax * i / n
            u, v = poly(U, p), poly(V, p)
            du = U[1] + 2 * U[2] * p + 3 * U[3] * p * p
            dv = V[1] + 2 * V[2] * p + 3 * V[3] * p * p
            ddu, ddv = 2 * U[2] + 6 * U[3] * p, 2 * V[2] + 6 * V[3] * p
            sp = math.hypot(du, dv)
            k = (du * ddv - dv * ddu) / sp ** 3 if sp > 1e-12 else float("inf")
            x, y = x0 + u * ch - v * sh, y0 + u * sh + v * ch
            if prev is not None:
                arc += math.hypot(x - prev[0], y - prev[1])
            prev = (x, y)
            pts.append((arc, x, y, h0 + math.atan2(dv, du), k))
        # s_local 은 선언 길이에 비례 배분(CARLA 와 같은 가정인지는 미확인). 실제 호장은 따로 반환
        out = [(a / arc * L if arc > 0 else 0.0, x, y, h, k) for a, x, y, h, k in pts]
        return tag, out, arc
    raise ValueError(f"미지원 기하 {tag}")


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def analyze(road):
    rid = int(road.get("id"))
    L = float(road.get("length"))
    lanes = road.find("lanes")
    lo = recs(lanes, "laneOffset")
    sec = lanes.find("laneSection")
    w1 = w2 = None
    for ln in sec.iter("lane"):
        if ln.get("id") == "-1":
            w1 = recs(ln, "width")
        if ln.get("id") == "-2":
            w2 = recs(ln, "width")
    nsec = len(lanes.findall("laneSection"))
    rows, joints, mism = [], [], []
    last = None
    for g in road.find("planView"):
        tag, smp, arc = geom_samples(g)
        gs, gl = float(g.get("s")), float(g.get("length"))
        if tag == "paramPoly3" and abs(arc - gl) > 1e-3:
            mism.append((round(gs, 3), round(gl, 4), round(arc, 4)))
        if last is not None:
            gx, gy, gh = float(g.get("x")), float(g.get("y")), float(g.get("hdg"))
            joints.append((round(gs, 3), math.hypot(gx - last[1], gy - last[2]), math.degrees(wrap(gh - last[3]))))
        for u, x, y, h, k in smp:
            s = gs + u
            off = at(lo, s)
            w = at(w1, s) if w1 else 0.0
            ws = at(w2, s) if w2 else 0.0
            rows.append((s, x, y, h, k, off, w, ws, tag))
        last = smp[-1]
    return rid, L, rows, joints, mism, nsec


def edge_r(k, t):
    """t 위치 평행선의 곡률 중심까지 거리(부호: <=0 이면 뒤집힘). 직선이면 inf"""
    if abs(k) < 1e-9:
        return float("inf")
    return (1 - k * t) / abs(k)


def summarize(rid, L, rows):
    best = (float("inf"), None)
    best_sw = (float("inf"), None)
    best_ref = (float("inf"), None)
    for s, x, y, h, k, off, w, ws, tag in rows:
        if abs(k) < 1e-9:
            continue
        ri = min(edge_r(k, off), edge_r(k, off - w))
        ro = edge_r(k, off - w - ws)
        rr = 1 / abs(k)
        if ri < best[0]:
            best = (ri, (s, k, w, off, tag))
        if min(ri, ro) < best_sw[0]:
            best_sw = (min(ri, ro), (s, k, w + ws))
        if rr < best_ref[0]:
            best_ref = (rr, s)
    return best, best_sw, best_ref


def main():
    root = ET.parse(sys.argv[1]).getroot()
    roads = {int(r.get("id")): r for r in root.iter("road")}
    ids = [int(a) for a in sys.argv[2:]]
    if ids:
        for rid in ids:
            rid, L, rows, joints, mism, nsec = analyze(roads[rid])
            print(f"r{rid} 길이 {L:.3f} junction={roads[rid].get('junction')} laneSection {nsec}")
            print(f"  기하 이음매(시작 s, 위치 차 m, 방위 차 도): {[(a, round(b, 4), round(c, 2)) for a, b, c in joints]}")
            print(f"  paramPoly3 선언 길이 vs 실제 호장: {mism}")
            print("  s, k(1/m), R_ref, w(lane-1), 차선중심 R, 안쪽가장자리 R(=중심R - w/2), 인도 끝 r")
            step = max(1, int(0.25 / DS))
            for i, (s, x, y, h, k, off, w, ws, tag) in enumerate(rows):
                if i % step:
                    continue
                rc = edge_r(k, off - w / 2)
                ri = min(edge_r(k, off), edge_r(k, off - w))
                ro = edge_r(k, off - w - ws)
                f = lambda v: "inf" if v == float("inf") else f"{v:.2f}"
                print(f"  s={s:6.2f} k={k:+.4f} R_ref={f(1/abs(k) if abs(k) > 1e-9 else float('inf'))} w={w:.2f} "
                      f"Rc={f(rc)} Rin={f(ri)} Rsw={f(ro)} {tag}")
            best, best_sw, best_ref = summarize(rid, L, rows)
            print(f"  최소 안쪽 가장자리 반경(lane -1) {best[0]:.3f}m at s={best[1][0]:.2f} (k={best[1][1]:+.4f}, w={best[1][2]:.2f}, {best[1][4]})")
            print(f"  최소 R_ref {best_ref[0]:.3f}m at s={best_ref[1]:.2f} / 인도 포함 최소 {best_sw[0]:.3f}m at s={best_sw[1][0]:.2f}")
        return
    print("road,junction,length,min_inner_r,s_at,k_at,w_at,min_ref_r,min_with_sidewalk_r,max_joint_gap_m,max_joint_hdg_deg,pp3_len_mismatch_max")
    for rid, road in sorted(roads.items()):
        rid, L, rows, joints, mism, nsec = analyze(road)
        best, best_sw, best_ref = summarize(rid, L, rows)
        mg = max((j[1] for j in joints), default=0.0)
        mh = max((abs(j[2]) for j in joints), default=0.0)
        mm = max((abs(a[2] - a[1]) for a in mism), default=0.0)
        if best[1] is None:
            print(f"{rid},{road.get('junction')},{L:.3f},inf,,,,inf,inf,{mg:.4f},{mh:.3f},{mm:.4f}")
        else:
            print(f"{rid},{road.get('junction')},{L:.3f},{best[0]:.4f},{best[1][0]:.2f},{best[1][1]:.5f},{best[1][2]:.2f},"
                  f"{best_ref[0]:.4f},{best_sw[0]:.4f},{mg:.4f},{mh:.3f},{mm:.4f}")


if __name__ == "__main__":
    main()
