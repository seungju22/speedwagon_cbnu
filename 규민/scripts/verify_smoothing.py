#!/usr/bin/env python3
# 맵세션11 Phase C-3: 평활화 후 재변환 검증. 읽기전용(입력 파일 미수정), CARLA 서버 불필요.
# 목적: 원본 xodr(대조군)과 평활화 xodr 를 같은 방법으로 재측정해 비교한다.
# 검증 항목
#   V1 위상: road/junction 수, junction 이름 집합, 길이 0.2m plain road 수
#   V2 문제 구간(2m 창 f<0.30, f<0) 재측정 + 잔여 목록 CSV(중문·후문 경로 설계 시 대조용)
#   V3 경로 핵심 두 커브(OSM 노드 3957495733/3957495732) 주변 최악 f·R2 전/후
#   V4 축척: junction 쌍 거리 비율(신/구)
#   V5 road id 대응표(시작·끝점 좌표 매칭) + 확정 경로 15 road 새 번호
#   V6 정문->종점 경로 재탐색(방향 검사, phase_e_route_check 의 함수 재사용), 길이 비교
#   V7 종점 출입구 거리(구/신 같은 방법), 옛 정지 지점(road1247 s=89.45)의 새 위치
#   V8 원본 대비 이탈(Hausdorff)과 건물 침범(OSM 건물 폴리곤)
import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import curve_metrics as cm                                        # noqa: E402
import geo_calibrate as G                                         # noqa: E402
from analyze_geometry import road_end_state, geom_point, geom_heading  # noqa: E402
from phase_e_route_check import lane_dirs, build, dijkstra, path_to   # noqa: E402

BASE = Path(__file__).resolve().parent.parent
D = BASE / "data" / "processed"
ROUTE_OLD = ["1247", "1914", "1356", "1917", "1355", "1446", "1172", "1428", "1240",
             "1695", "1239", "1426", "1238", "1564", "1278"]
END_S_OLD = 100.38
ENTR_LIB_WAY, ENTR_TARGET_WAY = "442066135", "446440345"      # find_library_entrances.py 출입구1
CRIT_NODES = ("3957495733", "3957495732")                     # road1247 s=80 / s=95 커브 원인 노드
STOP_S_OLD = 89.45                                            # 옛 정지(충돌 시작) 지점 s
ARC_ID_BASE = 9_000_000_000_000


def road_end_enu(roads, al):
    ids = sorted(roads, key=int)
    A = np.array([road_end_state(roads[r], "start")[:2] + road_end_state(roads[r], "end")[:2] for r in ids])
    s = al.fwd(A[:, 0:2])
    e = al.fwd(A[:, 2:4])
    return ids, s, e


def match_roads(old, new):
    """(시작,끝) ENU 좌표 합 거리 최소 일대일 매칭. old/new = (roads, al). 반환 {old_id: (new_id, cost)}"""
    io, so, eo = road_end_enu(*old)
    inn, sn, en = road_end_enu(*new)
    cost = (np.linalg.norm(so[:, None, :] - sn[None, :, :], axis=2)
            + np.linalg.norm(eo[:, None, :] - en[None, :, :], axis=2))
    kind_o = np.array([old[0][r]["junction"] != "-1" for r in io])
    kind_n = np.array([new[0][r]["junction"] != "-1" for r in inn])
    cost = cost + 1e6 * (kind_o[:, None] != kind_n[None, :])
    pairs = sorted((cost[i, j], i, j) for i in range(len(io)) for j in range(len(inn)) if cost[i, j] < 8.0)
    used_o, used_n, m = set(), set(), {}
    for c, i, j in pairs:
        if i in used_o or j in used_n:
            continue
        used_o.add(i)
        used_n.add(j)
        m[io[i]] = (inn[j], float(c))
    return m, cost, io, inn


def project_on_road(road, al_pt_xodr, step=0.05):
    """xodr 점 -> road 기준선 최근접 s, 거리"""
    n = int(road["length"] / step)
    best = (1e18, 0.0)
    for i in range(n + 1):
        s = min(i * step, road["length"])
        g = road["geoms"][0]
        for gg in road["geoms"]:
            if gg["s"] - 1e-9 <= s:
                g = gg
        p = min(max((s - g["s"]) / g["length"], 0.0), 1.0)
        x, y = geom_point(g, p)
        d = math.hypot(x - al_pt_xodr[0], y - al_pt_xodr[1])
        if d < best[0]:
            best = (d, s)
    return best[1], best[0]


def point_on_road(road, s):
    g = road["geoms"][0]
    for gg in road["geoms"]:
        if gg["s"] - 1e-9 <= s:
            g = gg
    p = min(max((s - g["s"]) / g["length"], 0.0), 1.0)
    return geom_point(g, p)


def seg_dist(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy + 1e-12
    u = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
    return math.hypot(p[0] - a[0] - u * dx, p[1] - a[1] - u * dy), (a[0] + u * dx, a[1] + u * dy)


def route_search(roads, juncs, src_road, tgt_road, tgt_s):
    ld = lane_dirs_from(roads_root_cache[id(roads)])
    adj = build(roads, juncs, ld)
    dist, prev = dijkstra(adj, roads, (src_road, 1), 0.0)
    tg = (tgt_road, 1)
    if tg not in dist:
        return None, None, adj
    seq = path_to(prev, tg)
    return dist[tg] - roads[tgt_road]["length"] + tgt_s, seq, adj


roots_cache = {}
roads_root_cache = {}


def lane_dirs_from(root):
    return lane_dirs(root)


def main():
    ap = argparse.ArgumentParser(description="평활화 재변환 검증")
    ap.add_argument("--osm-orig", type=Path, default=D / "cbnu_internal_boundary_tags73.osm")
    ap.add_argument("--osm-smooth", type=Path, default=D / "cbnu_internal_boundary_tags73_smooth.osm")
    ap.add_argument("--osm-raw", type=Path, default=BASE / "data" / "raw" / "cbnu_campus.osm")
    ap.add_argument("--xodr-old", type=Path, default=BASE / "maps" / "cbnu_internal_only_localtm_tags73.xodr")
    ap.add_argument("--xodr-new", type=Path, default=BASE / "maps" / "cbnu_internal_only_localtm_tags73_smooth.xodr")
    ap.add_argument("--out-md", type=Path, default=BASE / "docs" / "map_session11_verification.md")
    ap.add_argument("--out-prefix", default="session11")
    a = ap.parse_args()
    logs = BASE / "docs" / "logs"
    L = []
    P = L.append
    P("# 맵세션11 Phase C-3 평활화 재변환 검증 (자동생성, 읽기전용)")
    P("")
    P(f"- 원본 OSM: {a.osm_orig.name} / 사본 OSM: {a.osm_smooth.name}")
    P(f"- 대조군 xodr: {a.xodr_old.name} / 신규 xodr: {a.xodr_new.name}")
    P("")

    # ---- 로딩 ----
    _, n_orig, w_orig, tg_orig = cm.load_osm(a.osm_orig)
    _, n_new, w_new, tg_new = cm.load_osm(a.osm_smooth)
    o = {}
    for tag, path, nodes, ways in (("old", a.xodr_old, n_orig, w_orig), ("new", a.xodr_new, n_new, w_new)):
        root, roads, juncs, elems = cm.load_xodr(path)
        roads_root_cache[id(roads)] = root
        segs, S = cm.build_segs(nodes, ways)
        al = cm.align_icp(roads, juncs, nodes, S)
        sites = cm.scan_sites(roads, elems)
        o[tag] = dict(root=root, roads=roads, juncs=juncs, elems=elems, segs=segs, S=S, al=al,
                      sites=sites, nodes=nodes, ways=ways)
    old, new = o["old"], o["new"]
    P(f"- 정렬(xodr->OSM ENU) 중앙 잔차: 구 {old['al'].median:.2f}m / 신 {new['al'].median:.2f}m")
    P("")

    # ---- V1 위상 ----
    P("## V1 위상")
    def topo(x):
        pl = [r for r in x["roads"].values() if r["junction"] == "-1"]
        return len(x["roads"]), len(x["juncs"]), len(pl), sum(1 for r in pl if r["length"] < 0.5)
    to, tn = topo(old), topo(new)
    P(f"- road: 구 {to[0]} / 신 {tn[0]} (이전 878)")
    P(f"- junction: 구 {to[1]} / 신 {tn[1]} (이전 95)")
    P(f"- plain road: 구 {to[2]} / 신 {tn[2]}, 길이<0.5m plain: 구 {to[3]} / 신 {tn[3]}")
    jn_o = {j["name"] for j in old["juncs"].values()}
    jn_n = {j["name"] for j in new["juncs"].values()}
    P(f"- junction 이름 집합 동일: {jn_o == jn_n}" + ("" if jn_o == jn_n else f" 차이 {sorted(jn_o ^ jn_n)}"))
    P("")

    # ---- V2 문제 구간 ----
    P("## V2 문제 구간 재측정 (인도 안쪽 2m 창 f, 접힘 f<0 / 수용 미달 f<0.30)")
    go = {f: cm.group_sites(old["sites"], f) for f in (0.0, cm.F_MIN)}
    gn = {f: cm.group_sites(new["sites"], f) for f in (0.0, cm.F_MIN)}
    P(f"- f<0.30: 구 {len(go[cm.F_MIN])}곳 -> 신 {len(gn[cm.F_MIN])}곳")
    P(f"- f<0 (접힘): 구 {len(go[0.0])}곳 -> 신 {len(gn[0.0])}곳")
    # 잔여 분류
    fixed_new = set()
    occ = {}
    for nds in new["ways"].values():
        for n in nds:
            occ[n] = occ.get(n, 0) + 1
        fixed_new.add(nds[0])
        fixed_new.add(nds[-1])
    fixed_new |= {n for n, c in occ.items() if c > 1} | set(tg_new)
    Pn = new["al"].fwd([[g["x"], g["y"]] for g in gn[cm.F_MIN]]) if gn[cm.F_MIN] else np.zeros((0, 2))
    d, bi, bu = cm.nearest_on_segs(new["S"], Pn) if len(Pn) else ([], [], [])
    cats = {}
    rows = []
    for g, dist, si, ui, pen in zip(gn[cm.F_MIN], d, bi, bu, Pn):
        wid, i = new["segs"][si][0], new["segs"][si][1]
        nds = new["ways"][wid]
        cum = cm.path_cum(new["nodes"], nds)
        pos = cum[i] + ui * (cum[i + 1] - cum[i])
        near = [k for k in range(len(nds)) if abs(cum[k] - pos) <= 8]
        has_arc = any(int(nds[k]) >= ARC_ID_BASE for k in near)
        has_fixed = any(nds[k] in fixed_new for k in near)
        if dist > 3.0:
            cat = "OSM 대응 불가(정렬오차>3m)"
        else:
            cat = ("호(신규노드) 근처 " if has_arc else "") + ("고정노드(공유/끝점) 근처" if has_fixed else "")
            cat = cat.strip() or "원본 내부노드만(이탈상한 초과 등)"
        cats[cat] = cats.get(cat, 0) + 1
        lat, lon = cm.to_ll(*pen)
        rows.append({"road_new": g["road"], "s": round(g["smin"], 2), "f": round(g["f"], 3),
                     "R2_m": round(g["R2"], 2), "way_id": wid, "near_node_ids": ";".join(nds[k] for k in near),
                     "category": cat, "lat": f"{lat:.6f}", "lon": f"{lon:.6f}"})
    P("- 잔여 분류: " + ", ".join(f"{k} {v}" for k, v in sorted(cats.items(), key=lambda kv: -kv[1])))
    with open(logs / f"{a.out_prefix}_residual_sites.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else ["none"])
        w.writeheader()
        w.writerows(rows)
    P(f"- 잔여 목록 CSV: docs/logs/{a.out_prefix}_residual_sites.csv (중문·후문 경로 설계 시 먼저 대조)")
    P("")

    # ---- V3 경로 핵심 두 커브 ----
    P("## V3 경로 핵심 두 커브(road1247 s=80 / s=95, OSM 원인 노드)")
    crit = {}
    for nid in CRIT_NODES:
        en = n_orig[nid]
        res = []
        for tag in ("old", "new"):
            x = o[tag]
            pt = x["al"].inv_pt(*en)
            w_ = cm.worst_near(x["sites"], pt, 5.0)
            res.append(w_)
        crit[nid] = res
        fo, fn = res
        P(f"- 노드 {nid}: 구 f={fo[0]:+.3f} R2={fo[3]:.2f}m (road{fo[1]} s={fo[2]:.1f}) -> "
          f"신 f={fn[0]:+.3f} R2={fn[3]:.2f}m (road{fn[1]} s={fn[2]:.1f})")
    ok = all(v[1][0] >= cm.F_MIN for v in crit.values())
    P(f"- 경로 위 문제 2곳 해소(신 f>=0.30): {'예' if ok else '아니오'}")
    P(f"- 기준선~인도 바깥 거리: 6.15m (차선 3.35 + 인도 2.80, laneSection 불변)")
    P("")

    # ---- V4 축척 ----
    P("## V4 축척 (junction 쌍 거리 비율 신/구, 구 거리 50m 이상 쌍)")
    lo = G.junction_xodr_locations(old["roads"], old["juncs"])
    ln = G.junction_xodr_locations(new["roads"], new["juncs"])
    no = {old["juncs"][j]["name"]: lo[j] for j in lo}
    nn = {new["juncs"][j]["name"]: ln[j] for j in ln}
    com = sorted(set(no) & set(nn))
    ratios = []
    for i in range(len(com)):
        for k in range(i + 1, len(com)):
            do = math.dist(no[com[i]], no[com[k]])
            if do >= 50:
                ratios.append(math.dist(nn[com[i]], nn[com[k]]) / do)
    P(f"- 공통 junction {len(com)}개, 쌍 {len(ratios)}개: 중앙값 {np.median(ratios):.5f} 평균 {np.mean(ratios):.5f} "
      f"최소 {min(ratios):.4f} 최대 {max(ratios):.4f}")
    P("")

    # ---- V5 road id 대응 ----
    P("## V5 road id 대응 (시작·끝점 ENU 좌표 매칭)")
    m, cost, io, inn = match_roads((old["roads"], old["al"]), (new["roads"], new["al"]))
    P(f"- 대응 성공 {len(m)}/{len(old['roads'])}, 비용(시작+끝 이격) 중앙 {np.median([c for _, c in m.values()]):.2f}m "
      f"최대 {max(c for _, c in m.values()):.2f}m")
    same_id = sum(1 for k, (v, _) in m.items() if k == v)
    P(f"- id 가 그대로인 road {same_id}개 (나머지는 재부여)")
    with open(logs / f"{a.out_prefix}_road_id_map.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["old_id", "new_id", "match_cost_m", "old_length", "new_length", "kind"])
        for k in sorted(m, key=int):
            v, c = m[k]
            w.writerow([k, v, f"{c:.3f}", f"{old['roads'][k]['length']:.3f}", f"{new['roads'][v]['length']:.3f}",
                        "connector" if old["roads"][k]["junction"] != "-1" else "plain"])
    P(f"- 전체 대응표: docs/logs/{a.out_prefix}_road_id_map.csv")
    route_new = []
    for r in ROUTE_OLD:
        v, c = m.get(r, (None, None))
        route_new.append(v)
        amb = sorted(cost[io.index(r)])[:2]
        P(f"- 경로 road{r} -> 신 road{v} (매칭비용 {c:.2f}m, 2순위와 격차 {amb[1] - amb[0]:.2f}m, "
          f"길이 {old['roads'][r]['length']:.2f} -> {new['roads'][v]['length']:.2f})")
    P("- 신 경로 시퀀스: " + " ".join(str(x) for x in route_new))
    P("")

    # ---- V7 종점 출입구 (V6 보다 먼저 계산: 종점 s 필요) ----
    _, n_raw, w_raw, _ = cm.load_osm(a.osm_raw)
    def poly(wid):
        return [n_raw[n] for n in w_raw[wid] if n in n_raw]
    lib = poly(ENTR_LIB_WAY)
    tgt = poly(ENTR_TARGET_WAY)
    cen = (sum(p[0] for p in tgt) / len(tgt), sum(p[1] for p in tgt) / len(tgt))
    best = (1e18, None)
    for p0, p1 in zip(lib, lib[1:]):
        dd, q = seg_dist(cen, p0, p1)
        if dd < best[0]:
            best = (dd, q)
    entr = best[1]
    P("## V7 종점 출입구")
    def end_metrics(x, road_id, s_hint=None):
        pt = x["al"].inv_pt(*entr)
        s_, dd = project_on_road(x["roads"][road_id], pt)
        return s_, dd
    s_old, d_old = end_metrics(old, "1278")
    new_end_road = route_new[-1]
    s_new, d_new = end_metrics(new, new_end_road)
    # 이전 기록: 종점 road1278 s=100.38, 출입구까지 16.68m (같은 방법 재현 여부 확인)
    P(f"- 같은 방법 재현(구 xodr, road1278): 출입구 최근접 s={s_old:.2f}m, 거리 {d_old * old['al'].s:.2f}m "
      f"(기록 s=100.38 / 16.68m)")
    P(f"- 신 xodr road{new_end_road}: 출입구 최근접 s={s_new:.2f}m (길이 {new['roads'][new_end_road]['length']:.2f}), "
      f"출입구까지 {d_new * new['al'].s:.2f}m")
    # 옛 종점(s=100.38) 위치를 신 road 에 투영
    xo, yo = point_on_road(old["roads"]["1278"], END_S_OLD)
    en_o = old["al"].fwd([[xo, yo]])[0]
    s_map, d_map = project_on_road(new["roads"][new_end_road], new["al"].inv_pt(*en_o))
    xn, yn = point_on_road(new["roads"][new_end_road], s_map)
    en_n = new["al"].fwd([[xn, yn]])[0]
    P(f"- 옛 종점 위치(road1278 s=100.38)를 신 road{new_end_road} 에 투영: s={s_map:.2f}m (이격 {d_map:.2f}m)")
    P(f"- 신 종점(s={s_map:.2f}) 출입구 거리: {math.dist(en_n, entr):.2f}m "
      f"(옛 종점 출입구 거리 {math.dist(en_o, entr):.2f}m, 같은 방법)")
    END_S_NEW = s_map
    P("")

    # ---- V6 경로 재탐색 ----
    P("## V6 정문 -> 종점 경로 재탐색 (driving lane 방향 검사)")
    c_old, seq_old, adj_old = route_search(old["roads"], old["juncs"], "1247", "1278", END_S_OLD)
    P(f"- 구 xodr 같은 방법 재현: {'존재' if c_old else '끊김'}, "
      f"{c_old:.2f}m, road {len(seq_old)}개 (이전 기록 781.22m / 15 road)")
    src_new = route_new[0]
    c_new, seq_new, adj_new = route_search(new["roads"], new["juncs"], src_new, new_end_road, END_S_NEW)
    if c_new is None:
        P("- 신 xodr: 끊김")
    else:
        P(f"- 신 xodr: 존재, {c_new:.2f}m, road {len(seq_new)}개 (출발 road{src_new} s=0 -> 종점 road{new_end_road} "
          f"s={END_S_NEW:.2f})")
        ids_new = [r for r, _ in seq_new]
        P("- 신 최단 시퀀스: " + " ".join(f"{r}{'+' if d == 1 else '-'}" for r, d in seq_new))
        P(f"- 대응 경로와 동일: {ids_new == route_new}")
        if ids_new != route_new:
            P(f"  (대응 경로 {route_new})")
    # 대응 경로 자체의 방향 연결성
    chain_ok = []
    for u, v in zip(route_new, route_new[1:]):
        chain_ok.append(((v, 1) in adj_new.get((u, 1), [])))
    P(f"- 대응 15 road 연속 연결(정방향) 모두 성립: {all(chain_ok)} " + ("" if all(chain_ok) else f"{chain_ok}"))
    P("")

    # ---- V7b 옛 정지 지점 ----
    P("## 옛 정지 지점(road1247 s=89.45, 충돌 시작)의 새 위치")
    x89, y89 = point_on_road(old["roads"]["1247"], STOP_S_OLD)
    en89 = old["al"].fwd([[x89, y89]])[0]
    r1247n = route_new[0]
    s89n, d89 = project_on_road(new["roads"][r1247n], new["al"].inv_pt(*en89))
    P(f"- 신 road{r1247n} s={s89n:.2f}m (이격 {d89:.2f}m). 주행 검증 시 s≈{s89n - 2:.0f}~{s89n + 8:.0f} 구간 관찰")
    # 구간 s=89~96(옛) 대응 구간 지표
    sites_new = new["sites"]
    twin_new = None
    P("")

    # ---- V8 이탈 / 건물 ----
    P("## V8 원본 대비 이탈과 건물 침범")
    changed = [w for w in w_new if w_new[w] != w_orig.get(w)]
    building = []
    _, _, _, _ = None, None, None, None
    import xml.etree.ElementTree as ET
    raw_root = ET.parse(a.osm_raw).getroot()
    for w in raw_root.findall("way"):
        if any(t.get("k") == "building" for t in w.findall("tag")):
            pts = [n_raw[n.get("ref")] for n in w.findall("nd") if n.get("ref") in n_raw]
            if len(pts) >= 4:
                building.append(np.array(pts))
    E = np.array([(*pg[i], *pg[i + 1]) for pg in building for i in range(len(pg) - 1)])

    def dist_edges(pts):
        A_, B_ = E[:, 0:2], E[:, 2:4]
        D_ = B_ - A_
        L2 = (D_ ** 2).sum(1) + 1e-12
        out = []
        for p in pts:
            u = np.clip(((p - A_) * D_).sum(1) / L2, 0, 1)
            out.append(np.linalg.norm(p - (A_ + u[:, None] * D_), axis=1).min())
        return np.array(out)

    def inside(p):
        x, y = p
        for pg in building:
            if pg[:, 0].min() <= x <= pg[:, 0].max() and pg[:, 1].min() <= y <= pg[:, 1].max():
                c = False
                for i in range(len(pg) - 1):
                    x1, y1 = pg[i]
                    x2, y2 = pg[i + 1]
                    if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1 + 1e-12) + x1:
                        c = not c
                if c:
                    return True
        return False

    def pdist(p, pl):
        return min(seg_dist(p, a_, b_)[0] for a_, b_ in zip(pl, pl[1:]))

    def dense(pl, step=0.5):
        out = []
        for a_, b_ in zip(pl, pl[1:]):
            n = max(int(math.dist(a_, b_) / step), 1)
            out += [(a_[0] + (b_[0] - a_[0]) * k / n, a_[1] + (b_[1] - a_[1]) * k / n) for k in range(n)]
        out.append(pl[-1])
        return out

    hds, worse, ins_new, ins_old, near_new, near_old = [], [], 0, 0, 0, 0
    for wid in changed:
        po = w_orig[wid]
        pn = w_new[wid]
        i0 = next(i for i, (x, y) in enumerate(zip(po, pn)) if x != y)
        j0 = next(i for i, (x, y) in enumerate(zip(reversed(po), reversed(pn))) if x != y)
        so = [n_orig[n] for n in po][max(i0 - 1, 0):len(po) - j0 + 1]
        sn = [n_new[n] for n in pn][max(i0 - 1, 0):len(pn) - j0 + 1]
        do, dn = dense(so), dense(sn)
        hd = max(max(pdist(p, so) for p in dn), max(pdist(p, sn) for p in do))
        hds.append((hd, wid))
        dno, doo = dist_edges(np.array(dn)).min(), dist_edges(np.array(do)).min()
        ins_new += any(inside(p) for p in dn)
        ins_old += any(inside(p) for p in do)
        near_new += dno < cm.SIDEWALK_FAR_M
        near_old += doo < cm.SIDEWALK_FAR_M
        if dno < cm.SIDEWALK_FAR_M and dno < doo - 0.05:
            worse.append((wid, dno, doo, hd))
    P(f"- 변경 way {len(changed)}개, 신규 노드 {sum(1 for n in n_new if int(n) >= ARC_ID_BASE)}개")
    P(f"- 원본 대비 최대 이탈(양방향 Hausdorff): 중앙 {np.median([h for h, _ in hds]):.2f}m "
      f"최대 {max(h for h, _ in hds):.2f}m (way{max(hds)[1]}), >1m {sum(1 for h, _ in hds if h > 1)}개, "
      f">2m {sum(1 for h, _ in hds if h > 2)}개")
    P(f"- 건물 폴리곤 {len(building)}개. 변경 구간 중심선이 건물 내부: 신 {ins_new}곳 / 원본 {ins_old}곳")
    P(f"- 건물까지 <6.15m(인도 포함 반폭): 신 {near_new}곳 / 원본 {near_old}곳, 새로 0.05m 이상 악화 {len(worse)}곳")
    for wid, dn_, do_, hd in sorted(worse, key=lambda z: z[1]):
        P(f"  - way{wid} 건물거리 {do_:.2f} -> {dn_:.2f}m (이탈 {hd:.2f}m)")
    P("")
    a.out_md.write_text("\n".join(L), encoding="utf-8")
    print("\n".join(L))


if __name__ == "__main__":
    main()
