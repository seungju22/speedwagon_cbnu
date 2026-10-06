#!/usr/bin/env python3
# 맵세션11 Phase A. 읽기전용, CARLA 불필요. xodr 는 수정하지 않는다.
# 가설: 인도가 커브 "안쪽"에 오고 반경이 (기준선~인도 바깥 가장자리 거리)보다 작으면
#       안쪽 오프셋 곡선이 접혀(fold) 인도 메시가 쐐기로 튀어나온다.
#
# 판정식(OpenDRIVE t 양수=진행방향 왼쪽, 곡률 kappa 양수=좌회전):
#   기준선에서 횡거리 t 만큼 떨어진 오프셋 곡선의 진행 배율 f = 1 - kappa * t
#   f <= 0 이면 접힘(fold). 안쪽 여부는 kappa*t > 0 (곡률중심과 같은 쪽).
#   독립 검증: 오프셋 폴리라인을 직접 만들어 기준선 진행과 반대로 가는 구간 검출.
#
# analyze_geometry.py 의 세션9 수정판(poly_deriv 3*d*p**2)을 사용한다.
# 샘플 간격 0.25m(<=0.5m), 각도는 unwrap 처리.
import csv
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_geometry import parse_roads, geom_point, geom_heading

BASE = Path(__file__).resolve().parent.parent
XODR_PATH = BASE / "maps" / "cbnu_internal_only_localtm_tags73.xodr"
OUT_MD = BASE / "docs" / "map_session11_wedge_analysis.md"
OUT_CSV = BASE / "docs" / "logs" / "session11_wedge_scan.csv"

# 세션9 A-2 확정경로 15 road (analyze_curves_s9.py 와 동일)
ROUTE_ROADS = [1247, 1914, 1356, 1917, 1355, 1446, 1172, 1428, 1240,
               1695, 1239, 1426, 1238, 1564, 1278]

STEP = 0.25          # m
PLAIN_TOL = 0.5      # 구간 병합 허용 간격


# ---------------- lane 파싱 ----------------
def _poly(a, b, c, d, x):
    return a + b * x + c * x * x + d * x ** 3


def parse_lanes(road_elem):
    lanes = road_elem.find("lanes")
    offsets = []
    for lo in lanes.findall("laneOffset"):
        offsets.append((float(lo.get("s")), float(lo.get("a")), float(lo.get("b")),
                        float(lo.get("c")), float(lo.get("d"))))
    offsets.sort()
    sections = []
    for ls in lanes.findall("laneSection"):
        sec = {"s": float(ls.get("s")), "left": [], "right": []}
        for side in ("left", "right"):
            node = ls.find(side)
            if node is None:
                continue
            for ln in node.findall("lane"):
                widths = []
                for w in ln.findall("width"):
                    widths.append((float(w.get("sOffset")), float(w.get("a")), float(w.get("b")),
                                   float(w.get("c")), float(w.get("d"))))
                widths.sort()
                sec[side].append({"id": int(ln.get("id")), "type": ln.get("type"),
                                  "widths": widths})
            sec[side].sort(key=lambda l: abs(l["id"]))
        sections.append(sec)
    sections.sort(key=lambda x: x["s"])
    return offsets, sections


def lane_width(lane, ds):
    ws = [w for w in lane["widths"] if w[0] <= ds + 1e-9]
    if not ws:
        return 0.0
    so, a, b, c, d = ws[-1]
    return max(_poly(a, b, c, d, ds - so), 0.0)


def lane_edges(offsets, sections, s):
    """s 위치의 lane 별 (t_inner, t_outer, type, side). t 는 진행방향 왼쪽이 +."""
    off = 0.0
    for o in offsets:
        if o[0] <= s + 1e-9:
            off = _poly(o[1], o[2], o[3], o[4], s - o[0])
    sec = sections[0]
    for sc in sections:
        if sc["s"] <= s + 1e-9:
            sec = sc
    ds = s - sec["s"]
    out = []
    for side, sign in (("left", 1.0), ("right", -1.0)):
        t = off
        for ln in sec[side]:
            w = lane_width(ln, ds)
            t_in, t_out = t, t + sign * w
            out.append({"id": ln["id"], "type": ln["type"], "side": side,
                        "t_in": t_in, "t_out": t_out})
            t = t_out
    return out


# ---------------- 기하 샘플링 ----------------
def kappa_analytic(geom, p):
    """paramPoly3 정확 곡률(부호: +좌회전). 곡률은 매개변수화에 무관."""
    if geom["type"] == "line":
        return 0.0
    c = geom["coeffs"]
    du = c["bU"] + 2 * c["cU"] * p + 3 * c["dU"] * p ** 2
    dv = c["bV"] + 2 * c["cV"] * p + 3 * c["dV"] * p ** 2
    ddu = 2 * c["cU"] + 6 * c["dU"] * p
    ddv = 2 * c["cV"] + 6 * c["dV"] * p
    den = (du * du + dv * dv) ** 1.5
    return (du * ddv - dv * ddu) / den if den > 1e-12 else 0.0


def sample_road(road, step=STEP):
    n = max(int(math.ceil(road["length"] / step)), 1)
    rows = []
    for i in range(n + 1):
        s = min(i * road["length"] / n, road["length"])
        geom = road["geoms"][0]
        for g in road["geoms"]:
            if g["s"] - 1e-9 <= s:
                geom = g
        p = (s - geom["s"]) / geom["length"] if geom["length"] > 0 else 0.0
        p = min(max(p, 0.0), 1.0)
        x, y = geom_point(geom, p)
        hdg = geom_heading(geom, p)
        rows.append({"s": s, "x": x, "y": y, "hdg": hdg, "k": kappa_analytic(geom, p)})
    # heading unwrap + heading-diff 기반 곡률(세션9와 비교용)
    for i in range(1, len(rows)):
        while rows[i]["hdg"] - rows[i - 1]["hdg"] > math.pi:
            rows[i]["hdg"] -= 2 * math.pi
        while rows[i]["hdg"] - rows[i - 1]["hdg"] < -math.pi:
            rows[i]["hdg"] += 2 * math.pi
    for i in range(len(rows)):
        j0, j1 = max(i - 1, 0), min(i + 1, len(rows) - 1)
        ds = rows[j1]["s"] - rows[j0]["s"]
        rows[i]["k_hd"] = (rows[j1]["hdg"] - rows[j0]["hdg"]) / ds if ds > 0 else 0.0
    return rows


def offset_xy(r, t):
    return r["x"] - t * math.sin(r["hdg"]), r["y"] + t * math.cos(r["hdg"])


# ---------------- 스캔 ----------------
def scan_road(rid, road, offsets, sections):
    """반환: 샘플별 레코드(인도 lane 이 있는 경우). 인도 lane 마다 한 줄."""
    samples = sample_road(road)
    recs = []
    prev_off = {}
    for r in samples:
        edges = lane_edges(offsets, sections, r["s"])
        for e in edges:
            if e["type"] != "sidewalk":
                continue
            k = r["k"]
            t_far = e["t_out"]
            t_near = e["t_in"]
            # 인도 lane 의 "기준선에서 가장 먼 가장자리" = t_out
            f_far = 1.0 - k * t_far
            f_near = 1.0 - k * t_near
            inside = (k * t_far) > 0
            # 독립 검증: 오프셋 폴리라인 진행 방향 vs 기준선 진행 방향
            ox, oy = offset_xy(r, t_far)
            key = (e["id"],)
            folded_direct = False
            if key in prev_off:
                px, py, rx, ry = prev_off[key]
                dxo, dyo = ox - px, oy - py
                dxr, dyr = r["x"] - rx, r["y"] - ry
                folded_direct = (dxo * dxr + dyo * dyr) < 0
            prev_off[key] = (ox, oy, r["x"], r["y"])
            recs.append({
                "road": rid, "s": r["s"], "lane": e["id"], "side": e["side"],
                "t_near": t_near, "t_far": t_far, "kappa": k, "kappa_hd": r["k_hd"],
                "radius": (1.0 / abs(k)) if abs(k) > 1e-9 else float("inf"),
                "inside": inside, "f_far": f_far, "f_near": f_near,
                "fold_far": f_far <= 0, "fold_near": f_near <= 0,
                "fold_direct": folded_direct,
            })
    return recs, samples


def merge_intervals(recs, flag):
    """flag(rec) 가 True 인 연속 샘플을 (road, lane)별 구간으로 병합."""
    out = []
    cur = None
    for r in recs:
        if flag(r):
            if cur and cur["road"] == r["road"] and cur["lane"] == r["lane"] \
                    and r["s"] - cur["s1"] <= PLAIN_TOL + 1e-6:
                cur["s1"] = r["s"]
                cur["min_radius"] = min(cur["min_radius"], r["radius"])
                cur["min_f"] = min(cur["min_f"], r["f_far"])
                cur["t_far"] = max(cur["t_far"], abs(r["t_far"]))
            else:
                if cur:
                    out.append(cur)
                cur = {"road": r["road"], "lane": r["lane"], "s0": r["s"], "s1": r["s"],
                       "min_radius": r["radius"], "min_f": r["f_far"],
                       "t_far": abs(r["t_far"]), "side": r["side"]}
        else:
            if cur:
                out.append(cur)
                cur = None
    if cur:
        out.append(cur)
    return out



def find_twins(roads):
    """plain road 의 쌍둥이: 기준선이 일치하고 방향만 반대인 road(osm2odr 가 양방향 way 를
    반대방향 half-road 두 개로 만든다). 끝점 일치 + 1m 간격 샘플 최대 이격 0.01m 이하로 확정."""
    def pt(road, s):
        g = road["geoms"][0]
        for gg in road["geoms"]:
            if gg["s"] - 1e-9 <= s:
                g = gg
        p = min(max((s - g["s"]) / g["length"], 0.0), 1.0)
        return geom_point(g, p)
    by_start = {}
    for rid, q in roads.items():
        if q["junction"] != "-1":
            continue
        x, y = pt(q, 0.0)
        by_start.setdefault((round(x, 1), round(y, 1)), []).append(rid)
    twins = {}
    for rid, q in roads.items():
        if q["junction"] != "-1":
            continue
        ex, ey = pt(q, q["length"])
        for cand in by_start.get((round(ex, 1), round(ey, 1)), []):
            c = roads[cand]
            if cand == rid or abs(c["length"] - q["length"]) > 0.01:
                continue
            sep = max(math.hypot(pt(q, s)[0] - pt(c, c["length"] * s / q["length"] * 0 + (q["length"] - s))[0],
                                 pt(q, s)[1] - pt(c, (q["length"] - s))[1])
                      for s in [i * 1.0 for i in range(int(q["length"]) + 1)])
            if sep <= 0.01:
                twins[rid] = cand
    return twins

def main():
    root = ET.parse(XODR_PATH).getroot()
    roads = parse_roads(root)
    road_elems = {r.get("id"): r for r in root.iter("road")}
    L = []
    P = L.append
    P("# 맵세션11 Phase A 쐐기 원리 검증 (읽기전용, 자동생성)")
    P("")
    P(f"- xodr: {XODR_PATH.name}, road {len(roads)}개, 샘플 간격 {STEP}m")
    P("- 판정식: f=1-kappa*t (t=기준선 횡거리, 좌+). f<=0 접힘. 안쪽=kappa*t>0")
    P("")

    all_recs = []
    lane_struct = {}
    for rid, road in roads.items():
        offsets, sections = parse_lanes(road_elems[rid])
        # lane 구조 요약(첫 laneSection, 전체 road 집계용)
        sig = []
        for sec in sections:
            sig.append(tuple((l["type"], "R" if l["id"] < 0 else "L")
                             for side in ("left", "right") for l in sec[side]))
        lane_struct.setdefault(tuple(sig[:1]), []).append(rid)
        recs, _ = scan_road(rid, road, offsets, sections)
        all_recs.extend(recs)

    # ---- A-1: road1247 ----
    P("## A-1. road1247 lane 구조")
    r1247 = road_elems["1247"]
    offsets, sections = parse_lanes(r1247)
    P(f"- laneOffset: {offsets if offsets else '없음'}")
    P(f"- laneSection 수: {len(sections)}")
    for e in lane_edges(offsets, sections, 0.0):
        P(f"- lane {e['id']:>2} {e['type']:<9} 측={e['side']} "
          f"t: {e['t_in']:+.3f} -> {e['t_out']:+.3f}")
    P("- 좌측 lane 없음이면 기준선(t=0)은 주행차선(-1)의 왼쪽 끝이다")
    P("")

    # ---- A-2: 커브 회전 방향 ----
    P("## A-2. road1247 커브 회전 방향 / 인도 위치")
    rec1247 = [r for r in all_recs if r["road"] == "1247" and r["lane"] == -2]
    for lo, hi, name in ((78.0, 84.0, "s=80 커브(통과)"), (92.0, 97.0, "s=95 커브(정지)")):
        seg = [r for r in rec1247 if lo <= r["s"] <= hi]
        mk = max(seg, key=lambda r: abs(r["kappa"]))
        turn = "좌회전" if mk["kappa"] > 0 else "우회전"
        pos = "안쪽" if mk["inside"] else "바깥쪽"
        P(f"- {name}: s={lo}~{hi} 최대곡률 s={mk['s']:.2f} kappa={mk['kappa']:+.4f} "
          f"반경={mk['radius']:.2f}m {turn}, 인도(우측 t={mk['t_far']:+.2f}) {pos}, "
          f"f_far={mk['f_far']:.3f} f_near={mk['f_near']:.3f}")
        P(f"  - 구간 내 kappa 부호: 양 {sum(1 for r in seg if r['kappa'] > 1e-6)}개 / "
          f"음 {sum(1 for r in seg if r['kappa'] < -1e-6)}개, "
          f"안쪽 판정 샘플 {sum(1 for r in seg if r['inside'])}/{len(seg)}, "
          f"직접 접힘 검출 {sum(1 for r in seg if r['fold_direct'])}개")
        P(f"  - heading-diff 기준 최소반경(세션9 비교용) "
          f"{min(1/abs(r['kappa_hd']) for r in seg if abs(r['kappa_hd'])>1e-9):.2f}m")
    P("")

    # ---- A-3/A-4: 전수 ----
    fold_far = merge_intervals(all_recs, lambda r: r["fold_far"] and r["inside"])
    fold_near = merge_intervals(all_recs, lambda r: r["fold_near"] and r["inside"])
    fold_dir = merge_intervals(all_recs, lambda r: r["fold_direct"])
    # 안쪽 인도 + 반경이 t_far 에 근접(f_far<0.3)한 위험 구간(참고)
    risky = merge_intervals(all_recs, lambda r: r["inside"] and r["f_far"] < 0.3)
    outer_tight = merge_intervals(all_recs, lambda r: (not r["inside"]) and r["radius"] < 6.15)

    route_set = {str(x) for x in ROUTE_ROADS}

    def count(lst):
        on = [x for x in lst if x["road"] in route_set]
        return len(on), len(lst)

    P("## A-4. 878 road 전수 스캔")
    P(f"- 인도 lane 이 있는 road/샘플: "
      f"{len({r['road'] for r in all_recs})} road, {len(all_recs)} 샘플")
    inside_ct = sum(1 for r in all_recs if r["inside"] and abs(r["kappa"]) > 1e-3)
    outside_ct = sum(1 for r in all_recs if (not r["inside"]) and abs(r["kappa"]) > 1e-3)
    P(f"- 곡률 있는 샘플 중 인도 안쪽 {inside_ct} / 바깥쪽 {outside_ct}")
    for name, lst in (("안쪽 인도 바깥가장자리 접힘(f_far<=0)", fold_far),
                      ("안쪽 인도 안쪽가장자리(연석선) 접힘(f_near<=0)", fold_near),
                      ("직접검증 오프셋 진행 역전(안/바깥 무관)", fold_dir),
                      ("참고: 안쪽 인도 f_far<0.3(접힘 임박)", risky),
                      ("참고: 바깥쪽 인도인데 반경<6.15m", outer_tight)):
        a, b = count(lst)
        P(f"- {name}: 경로 위 {a}곳 / 전체 {b}곳")
    P("")
    for name, lst in (("접힘(f_far<=0, 안쪽)", fold_far),
                      ("직접 검증 진행 역전", fold_dir),
                      ("접힘 임박(f_far<0.3, 안쪽)", risky)):
        P(f"### 목록: {name}")
        if not lst:
            P("- 없음")
        for x in sorted(lst, key=lambda x: (x["road"] not in route_set, x["road"], x["s0"]))[:60]:
            tag = "경로" if x["road"] in route_set else "  - "
            P(f"- {tag} road{x['road']} lane{x['lane']} s={x['s0']:.2f}~{x['s1']:.2f} "
              f"최소반경={x['min_radius']:.2f}m 인도바깥거리={x['t_far']:.2f}m "
              f"min_f={x['min_f']:.3f}")
        P("")


    # ---- 쌍둥이(역방향 half-road) 매핑: 물리적 도로 기준 문제 구간 ----
    twins = find_twins(roads)
    plain_with_twin = [r for r in twins]
    P("## 쌍둥이 road (기준선 일치, 방향 반대)")
    P(f"- plain road 중 쌍둥이 확정: {len(twins)}개 (전체 plain "
      f"{sum(1 for q in roads.values() if q['junction']=='-1')}개)")
    P("- 물리적 도로 = [쌍둥이 인도][쌍둥이 차선][내 차선][내 인도]. 좌회전이면 "
      "내 왼쪽(안쪽)은 쌍둥이의 인도")
    P(f"- 예: road1247 <-> road{twins.get('1247')}, road1356 <-> road{twins.get('1356')}")
    P("")
    P("### 경로 road 기준 물리 문제 구간(내 인도 접힘 + 쌍둥이 인도 접힘을 내 s 로 환산)")
    phys = []
    for x in fold_far:
        if x["road"] in route_set:
            phys.append(("자기 인도(안쪽)", x["road"], x["road"], x["s0"], x["s1"], x))
    for x in fold_far:
        for mine, tw in twins.items():
            if tw == x["road"] and mine in route_set:
                Lm = roads[mine]["length"]
                phys.append(("쌍둥이 인도(안쪽)", mine, tw, Lm - x["s1"], Lm - x["s0"], x))
    phys.sort(key=lambda z: (z[1], z[3]))
    for kind, mine, src_r, a, b, x in phys:
        P(f"- road{mine} s={a:.2f}~{b:.2f} [{kind}, 원천 road{src_r}] "
          f"최소반경={x['min_radius']:.2f}m min_f={x['min_f']:.3f}")
    P(f"- 합계(경로 15 road 위 물리 접힘 구간): {len(phys)}곳")
    # 전체: 쌍둥이 쌍 단위로 물리 접힘 지점(쌍둥이 s 환산 후 병합)
    P("- 전체 접힘 구간(238)은 road 단위 집계. 쌍둥이 쌍은 서로 다른 half-road 이므로 "
      "물리적으로는 한 쌍의 반대쪽 인도가 각각 자기 안쪽 커브에서 접히는 것이라 중복 없음")
    P("")
    # 세션9 급커브 6곳 대조(경로 15 road 위 rate>=6deg/m 구간): 인도 위치 표기
    P("## 세션9 급커브 6곳 대조")
    s9 = [("1247", 79.5, 81.0), ("1247", 93.5, 95.5), ("1356", 11.5, 12.0),
          ("1564", 1.0, 9.0), ("1428", 0.0, 5.0), ("1446", 2.0, 8.5)]
    for rid, lo, hi in s9:
        seg = [r for r in all_recs if r["road"] == rid and lo - 0.25 <= r["s"] <= hi + 0.25]
        if not seg:
            P(f"- road{rid} s={lo}~{hi}: 인도 lane 없음(커넥터 등) — 인도 접힘 판정 대상 아님")
            continue
        segs = {}
        for r in seg:
            segs.setdefault(r["lane"], []).append(r)
        for ln, rr in segs.items():
            mk = max(rr, key=lambda r: abs(r["kappa"]))
            P(f"- road{rid} s={lo}~{hi} lane{ln}: 최대곡률 반경 {mk['radius']:.2f}m "
              f"{'좌' if mk['kappa']>0 else '우'}회전, 인도 "
              f"{'안쪽' if mk['inside'] else '바깥쪽'}, min f_far={min(r['f_far'] for r in rr):.3f}")
    P("")

    # lane 구조 분포
    P("## lane 구조 분포(첫 laneSection 기준)")
    for sig, ids in sorted(lane_struct.items(), key=lambda kv: -len(kv[1])):
        P(f"- {len(ids)} road: {sig[0] if sig else '()'}")
    P("")

    OUT_MD.write_text("\n".join(L), encoding="utf-8")
    with open(OUT_CSV, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["kind", "road", "lane", "s0", "s1", "min_radius", "t_far", "min_f", "on_route"])
        for kind, lst in (("fold_inside", fold_far), ("fold_direct", fold_dir),
                          ("risky_inside", risky), ("outer_tight", outer_tight)):
            for x in lst:
                w.writerow([kind, x["road"], x["lane"], f"{x['s0']:.2f}", f"{x['s1']:.2f}",
                            f"{x['min_radius']:.3f}", f"{x['t_far']:.3f}", f"{x['min_f']:.4f}",
                            int(x["road"] in route_set)])
    print(f"완료: {OUT_MD}")
    print(f"CSV: {OUT_CSV}")


if __name__ == "__main__":
    main()
