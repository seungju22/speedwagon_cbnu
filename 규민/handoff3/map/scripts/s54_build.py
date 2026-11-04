#!/usr/bin/env python3
# 10/11 2번: v3.1 후보 s53 복사 -> s54. p_yang_back 이 막히는 r1220(짝 r1352)·r1825 둘레만 바꾸고 나머지 road 는 바이트 그대로
# 방법은 s53_build_v31.py 와 같다(그 함수를 가져다 씀): 요소 묶음을 line+arc+line 또는 3차 Hermite 로, road ID·link·lanes 그대로, 목표 차선 반경 4m(+2%)
# [A] r1220 s0~40.50(요소 0~3) 삼각형 돌출: 양끝(시작점, s40.50) 위치·방위 그대로 새 기준선 C. 짝 r1352 같은 구간(요소 3~5)은 C 를 거꾸로 쓴다(같은 곡선)
#     두 반쪽 차선 반경(기준선 옆 -1.675, +1.675) 모두 목표 이상, 새 차선 가장자리가 옛 두 반쪽 띠(차선+인도) 안
# [B] r1825: r1219·r1351(0.2m 토막)의 두 끝 P2·P3 를 r1219 방위 반대로 DELTA 만큼 옮긴다(53a r1972 처럼 끝점 예외).
#     붙은 연결로 r1913·r1914(j77), r1824·r1825·r1826·r1827(j78)은 옮긴 끝 하나만 바뀐 채 다시 맞춘다(다른 끝 위치·방위 그대로)
#     '도로 폭 안' = 옛 둘레 road(j77·j78 연결로, 토막, 앞뒤 road)의 차선+인도 띠 합 안
# 입력: maps/cbnu_campus_v31_candidate_s53.xodr(읽기만). 출력: maps/cbnu_campus_v31_candidate_s54.xodr, docs/logs/s54_build.log·.json
# 사용: .venv-carla/bin/python map/scripts/s54_build.py [--delta 4.0] [--scan] [--out PATH]
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
BASE = HERE.parent
sys.path.insert(0, str(HERE))
import s53_build_v31 as B  # noqa: E402
import s53_check as CK  # noqa: E402
import s53_geom as G  # noqa: E402

SRC = BASE / "maps/cbnu_campus_v31_candidate_s53.xodr"
OUT = BASE / "maps/cbnu_campus_v31_candidate_s54.xodr"
LOG = BASE / "docs/logs/s54_build.log"
RT = B.R_TARGET * 1.02
log = []


def P(*a):
    s = " ".join(str(x) for x in a)
    log.append(s)
    print(s, flush=True)


def loc_r(road, a, b):
    return min([0.0 if fo else (abs(1 / k) if abs(k) > 1e-12 else math.inf) for s, x, y, h, k, fo in road.samples() if a - 1e-6 <= s <= b + 1e-6] or [math.inf])


def rev_els(els):
    """요소 목록을 거꾸로(끝 -> 시작). line·arc 만"""
    out = []
    for x, y, h, L, xml in reversed(els):
        ex, ey, eh = B.end_pose(x, y, h, L, xml)
        if xml == "<line/>":
            out.append((ex, ey, eh + math.pi, L, "<line/>"))
        else:
            k = float(xml.split('"')[1])
            out.append((ex, ey, eh + math.pi, L, f'<arc curvature="{-k:.10f}"/>'))
    return [(x, y, G.wrap(h), L, xml) for x, y, h, L, xml in out]


def union_band(roads):
    from shapely.ops import unary_union
    return unary_union([B.band_poly(r) for r in roads]).buffer(0.01).buffer(-0.01)


# ---------------- [A] r1220 + r1352 ----------------
def fix_1220(texts):
    rb = B.RoadBlock(B.ROAD_RE.search(texts[1220]))
    rp = B.RoadBlock(B.ROAD_RE.search(texts[1352]))
    i, j = 0, 3
    A, Bp = rb.start_pose(i), rb.end_pose_of(j)
    pi_, pj_ = 3, len(rp.geos) - 1
    Ap, Bpp = rp.start_pose(pi_), rp.end_pose_of(pj_)
    P(f"  r1220 창 요소 {i}~{j} s {rb.geos[i]['s']:.2f}~{rb.geos[j]['s'] + rb.geos[j]['L']:.2f}: ({A[0]:.2f},{A[1]:.2f}) {math.degrees(A[2]):.1f}도 -> "
      f"({Bp[0]:.2f},{Bp[1]:.2f}) {math.degrees(Bp[2]):.1f}도")
    P(f"  r1352 창 요소 {pi_}~{pj_}: ({Ap[0]:.2f},{Ap[1]:.2f}) {math.degrees(Ap[2]):.1f}도 -> ({Bpp[0]:.2f},{Bpp[1]:.2f}) {math.degrees(Bpp[2]):.1f}도 "
          f"(r1220 창과 끝점 차 {math.dist(A[:2], Bpp[:2]):.3f}·{math.dist(Bp[:2], Ap[:2]):.3f}m)")
    old, oldp = rb.road, rp.road
    off = old.off
    band_all = union_band([old, oldp])
    from shapely.geometry import LineString
    s_end = rb.geos[j]["s"] + rb.geos[j]["L"]
    old_line = LineString([old.ref_at(s_end * q / 300)[:2] for q in range(301)])     # 옛 기준선(창 s0~s_end)
    cands = []
    # line+arc+line: 차선 반경 목표(+2%)와 짝 쪽(+2*|off|) 두 가지, 호 최대
    for lr in (None, RT, RT + 2 * abs(off)):
        la = B.lal_any(A, Bp, off, lane_r=lr)
        if la:
            cands.append(("line+arc+line" + (f"(차선 반경 {lr:.2f}m 호)" if lr else "(호 최대)"), la[1], rev_els(la[1])))
    T = math.dist(A[:2], Bp[:2])
    for ia in range(41):
        for ib in range(41):
            a, b = T * (0.1 + 2.4 * ia / 40), T * (0.1 + 2.4 * ib / 40)
            c = B.herm_coef(A, Bp, a, b)
            if min(B.pp3_lane_rmin(c, off, 150), B.pp3_lane_rmin(c, -off, 150)) < RT:
                continue
            cr = B.herm_coef((Bp[0], Bp[1], Bp[2] + math.pi), (A[0], A[1], A[2] + math.pi), b, a)
            cands.append((f"3차 Hermite(a {a / T:.2f}T b {b / T:.2f}T)", [(A[0], A[1], A[2], B.pp3_arclen(c), B.pp3_xml(c))],
                          [(Bp[0], Bp[1], G.wrap(Bp[2] + math.pi), B.pp3_arclen(cr), B.pp3_xml(cr))]))
    P(f"  후보 {len(cands)}")
    res = []
    for how, els, rels in cands:
        nb, nl = rb.rebuild(i, j, els)
        nbp, nlp = rp.rebuild(pi_, pj_, rels)
        new, newp = B.road_of(nb), B.road_of(nbp)
        s1 = sum(e[3] for e in els)
        r_a = loc_r(new, 0, s1)
        s0p = rp.geos[pi_]["s"]
        r_b = loc_r(newp, s0p, s0p + sum(e[3] for e in rels))
        if min(r_a, r_b) < B.R_TARGET:
            continue
        dv, ob = B.deviation(old, new, extra_band=band_all)
        dvp, obp = B.deviation(oldp, newp, extra_band=band_all)
        same = max(math.dist(new.ref_at(s1 * q / 50)[:2], newp.ref_at(s0p + sum(e[3] for e in rels) * (1 - q / 50))[:2]) for q in range(51))
        res.append(dict(how=how, text=nb, textp=nbp, len_old=old.length, len_new=nl, lenp_old=oldp.length, lenp_new=nlp, r_a=r_a, r_b=r_b,
                        r_new=B.lane_rmin(new), rp_new=B.lane_rmin(newp), dev=max(dv, dvp), out_band=max(ob, obp), same=same,
                        dev_line=max(old_line.distance(__import__("shapely.geometry", fromlist=["Point"]).Point(new.ref_at(s1 * q / 60)[:2])) for q in range(61))))
    good = [c for c in res if c["out_band"] <= B.BAND_TOL]
    P(f"  목표 반경 넘는 것 {len(res)}, 그중 띠 안(0.3m) {len(good)}")
    if not good:
        return None
    dmin = min(c["dev"] for c in good)
    best = max([c for c in good if c["dev"] <= dmin + B.DEV_SLACK], key=lambda c: (min(c["r_a"], c["r_b"], 50.0), -c["dev"]))
    return best


# ---------------- [B] r1825 둘레 ----------------
def refit(texts, rid, A=None, Bend=None, band=None):
    """road rid 전체(요소 0~n-1)를 시작 A·끝 Bend(None 이면 그대로)로 다시 맞춘다. 53a 고르기 규칙"""
    rb = B.RoadBlock(B.ROAD_RE.search(texts[rid]))
    n = len(rb.geos)
    old = rb.road
    A = A or rb.start_pose(0)
    Bq = Bend or rb.end_pose_of(n - 1)
    allc = []
    straight = abs(G.wrap(Bq[2] - A[2])) < math.radians(0.05) and abs(G.wrap(math.atan2(Bq[1] - A[1], Bq[0] - A[0]) - A[2])) < math.radians(0.05)
    if straight:
        cl = [("line", [(A[0], A[1], A[2], math.dist(A[:2], Bq[:2]), "<line/>")])]
    else:
        cl = [(how, els) for r, els, how in B.fit_window(rb, 0, n - 1, A=A, B=Bq)]
        T = math.dist(A[:2], Bq[:2])            # 반경이 목표에 못 미쳐도 띠 안에서 옛것보다 나은 것을 찾게 Hermite 격자를 더함(53a pair_fit 과 같은 격자)
        for ia in range(31):
            for ib in range(31):
                c = B.herm_coef(A, Bq, T * (0.1 + 2.4 * ia / 30), T * (0.1 + 2.4 * ib / 30))
                if B.pp3_lane_rmin(c, old.off, 150) > 0:
                    cl.append(("3차 Hermite(격자)", [(A[0], A[1], A[2], B.pp3_arclen(c), B.pp3_xml(c))]))
    for how, els in cl:
        nb, nl = rb.rebuild(0, n - 1, els)
        new = B.road_of(nb)
        dv, ob = B.deviation(old, new, extra_band=band)
        allc.append(dict(rid=rid, how=how, text=nb, r_old=B.lane_rmin(old), r_new=B.lane_rmin(new), len_old=old.length, len_new=nl, dev=dv, out_band=ob))
    tgt = min(B.R_TARGET, allc[0]["r_old"]) if allc else B.R_TARGET
    good = [c for c in allc if c["r_new"] >= B.R_TARGET and c["out_band"] <= B.BAND_TOL]
    if good:
        dmin = min(c["dev"] for c in good)
        c = max([c for c in good if c["dev"] <= dmin + B.DEV_SLACK], key=lambda c: (min(c["r_new"], 50.0), -c["dev"]))
        c["meets"] = True
        return c
    ok = [c for c in allc if c["r_new"] >= tgt - 1e-6 and c["out_band"] <= B.BAND_TOL]
    c = max(ok or allc, key=lambda c: (c["r_new"] if c in ok else -c["out_band"], -c["dev"]))
    c["meets"] = c in ok
    return c


def fix_1825(texts, roads_all, delta):
    r19 = B.road_of(texts[1219])
    r51 = B.road_of(texts[1351])
    P2, P3 = r19.ref_at(0), r19.ref_at(r19.length)
    h = P2[2]
    assert math.dist(r51.ref_at(0)[:2], P3[:2]) < 1e-3 and math.dist(r51.ref_at(r51.length)[:2], P2[:2]) < 1e-3
    dx, dy = -delta * math.cos(h), -delta * math.sin(h)
    P2n, P3n = (P2[0] + dx, P2[1] + dy, h), (P3[0] + dx, P3[1] + dy, h)
    hb = r51.ref_at(0)[2]
    out = {}
    rb = B.RoadBlock(B.ROAD_RE.search(texts[1219]))
    nb, nl = rb.rebuild(0, 0, [(P2n[0], P2n[1], h, r19.length, "<line/>")])
    out[1219] = dict(rid=1219, how=f"옮김 {delta:.2f}m(방위 그대로)", text=nb, r_old=math.inf, r_new=math.inf, len_old=r19.length, len_new=nl, dev=delta, out_band=0.0, meets=True)
    rb = B.RoadBlock(B.ROAD_RE.search(texts[1351]))
    nb, nl = rb.rebuild(0, 0, [(P3n[0], P3n[1], hb, r51.length, "<line/>")])
    out[1351] = dict(rid=1351, how=f"옮김 {delta:.2f}m(방위 그대로)", text=nb, r_old=math.inf, r_new=math.inf, len_old=r51.length, len_new=nl, dev=delta, out_band=0.0, meets=True)
    near = [1219, 1351, 1220, 1352, 1249, 1250, 1385, 1386]      # 토막과 앞뒤 road. j77·j78 연결로는 아래에서 더함
    band = union_band([roads_all[r] for r in near] + [roads_all[r] for r in roads_all if roads_all[r].junction in (77, 78)])
    out[1914] = refit(texts, 1914, Bend=P2n, band=band)
    out[1913] = refit(texts, 1913, A=(P2n[0], P2n[1], hb), band=band)
    out[1825] = refit(texts, 1825, A=P3n, band=band)
    out[1826] = refit(texts, 1826, A=P3n, band=band)
    out[1824] = refit(texts, 1824, Bend=(P3n[0], P3n[1], hb), band=band)
    out[1827] = refit(texts, 1827, Bend=(P3n[0], P3n[1], hb), band=band)
    return out


def report(c):
    P(f"  r{c['rid']}: {c['how']}, 차선 반경(road 전체) {c['r_old']:.3f} -> {c['r_new']:.3f}m, 길이 {c['len_old']:.3f} -> {c['len_new']:.3f}m({c['len_new'] - c['len_old']:+.3f}), "
      f"옛 차선 중심에서 최대 {c['dev']:.2f}m, 둘레 띠 밖 {c['out_band']:.2f}m, 조건 {'맞음' if c.get('meets') else '못 맞춤'}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--delta", type=float, default=4.0)
    ap.add_argument("--scan", action="store_true", help="DELTA 여러 값으로 [B] 만 비교(쓰지 않음)")
    ap.add_argument("--out", type=Path, default=OUT)
    a = ap.parse_args()
    text = SRC.read_text()
    assert hashlib.sha256(text.encode()).hexdigest().startswith("b9cc23692441de3a")
    ms = {int(m.group(2)): m for m in B.ROAD_RE.finditer(text)}
    texts = {rid: m.group(0) for rid, m in ms.items()}
    _, roads_all, juncs = G.load(SRC)
    if a.scan:
        for d in (3.2, 3.5, 4.0, 4.5, 5.0, 5.5):
            out = fix_1825(dict(texts), roads_all, d)
            P(f"DELTA {d}: " + ", ".join(f"r{r} {c['r_new']:.2f}m/벗어남 {c['dev']:.2f}/띠밖 {c['out_band']:.2f}{'' if c.get('meets') else ' X'}" for r, c in out.items() if r not in (1219, 1351)))
        return
    changed = {}
    P("[A] r1220 + 짝 r1352 (삼각형 돌출)")
    c = fix_1220(texts)
    if c is None:
        P("  조건 맞는 후보 없음 -> 안 고침")
    else:
        g0 = CK.min_gap(roads_all, juncs, 1220)
        trial = dict(roads_all)
        trial[1220], trial[1352] = B.road_of(c["text"]), B.road_of(c["textp"])
        g1 = CK.min_gap(trial, juncs, 1220)
        P(f"  고름: {c['how']}. 창 안 차선 반경 r1220 {c['r_a']:.3f}m, r1352 {c['r_b']:.3f}m, road 전체 r1220 {c['r_new']:.3f}m r1352 {c['rp_new']:.3f}m, "
          f"두 반쪽 기준선 차 {c['same']:.4f}m, 옛 차선 중심에서 최대 {c['dev']:.2f}m(옛 기준선 기준 {c['dev_line']:.2f}m), 두 반쪽 띠 밖 {c['out_band']:.2f}m, "
          f"길이 r1220 {c['len_old']:.3f} -> {c['len_new']:.3f}, r1352 {c['lenp_old']:.3f} -> {c['lenp_new']:.3f}, 이웃 차선 간격 {g0[0]:.2f} -> {g1[0]:.2f}m(r{g1[1]})")
        if g1[0] < B.GAP_MIN and g1[0] < g0[0] - 0.05:
            P("  이웃 간격 조건 밖 -> 안 고침")
        else:
            texts[1220], texts[1352] = c["text"], c["textp"]
            roads_all.update(trial)
            changed[1220] = dict(rid=1220, how=c["how"], r_old=0.0, r_new=c["r_new"], len_old=c["len_old"], len_new=c["len_new"], dev=c["dev"], out_band=c["out_band"])
            changed[1352] = dict(rid=1352, how=c["how"] + "(거꾸로, 같은 기준선)", r_old=B.lane_rmin(G.load(SRC)[1][1352]), r_new=c["rp_new"],
                                 len_old=c["lenp_old"], len_new=c["lenp_new"], dev=c["dev"], out_band=c["out_band"])
    P(f"\n[B] r1825 둘레: P2·P3 를 {a.delta:.2f}m 옮김")
    out = fix_1825(texts, roads_all, a.delta)
    for rid, c in out.items():
        report(c)
    gaps = []
    trial = dict(roads_all)
    for rid, c in out.items():
        trial[rid] = B.road_of(c["text"])
    for rid in out:
        g0, g1 = CK.min_gap(roads_all, juncs, rid), CK.min_gap(trial, juncs, rid)
        gaps.append(f"r{rid} {g0[0]:.2f}->{g1[0]:.2f}(r{g1[1]})")
    P("  이웃 차선 간격 " + ", ".join(gaps))
    if all(c.get("meets") for c in out.values()) and all(c["out_band"] <= B.BAND_TOL for c in out.values()):
        for rid, c in out.items():
            texts[rid] = c["text"]
            changed[rid] = c
        roads_all.update(trial)
    else:
        P("  조건 못 맞춘 road 가 있어 [B] 적용 안 함")
    parts, pos = [], 0
    for rid, m in sorted(ms.items(), key=lambda kv: kv[1].start()):
        parts.append(text[pos:m.start()])
        parts.append(texts[rid])
        pos = m.end()
    parts.append(text[pos:])
    v = "".join(parts)
    a.out.write_text(v)
    sha = hashlib.sha256(v.encode()).hexdigest()
    ms2 = {int(m.group(2)): m.group(0) for m in B.ROAD_RE.finditer(v)}
    same = sum(1 for rid in ms if ms2[rid] == ms[rid].group(0))
    P(f"\n출력 {a.out.name}: {len(v.encode())} B, sha256 {sha[:16]}, 바뀐 road {sorted(changed)}, road {len(ms2)}, s53 과 바이트 같은 road {same}")
    LOG.write_text("\n".join(log) + "\n")
    LOG.with_suffix(".json").write_text(json.dumps({str(k): {kk: vv for kk, vv in c.items() if not kk.startswith("text")} for k, c in changed.items()},
                                                  ensure_ascii=False, indent=1, default=str))


if __name__ == "__main__":
    main()
