#!/usr/bin/env python3
# 맵세션11 Phase C: OSM 곡선 평활화 단계 (파이프라인: classify_internal -> 이 단계 -> osm_to_xodr)
# 설계: map/docs/map_session11_design_smoothing.md
#
# 목적: 인도 바깥 가장자리 오프셋이 접히는(2m 창 지표 f < 0.30) 커브만 골라, 원인 OSM 노드를
#       반경 R 원호로 모따기(fillet)한 OSM 사본과 변경 로그를 만든다. 원본 OSM 은 열기만 한다.
# 흐름: 입력 OSM -> [반복 최대 3회]
#         (1) Osm2Odr 변환(임시)   (2) 2m 창 스캔 -> 문제 구간(f<0.30)
#         (3) ICP 로 xodr 점 -> OSM 세그먼트/노드 대응   (4) 후보 노드 선정
#         (5) 필렛 계획(접선 겹침 축소, R_ACC 미달 건너뜀)   (6) 사본 갱신
#       -> 사본 OSM + 변경/건너뜀 CSV + JSON 보고서
# 근거·파라미터: 설계문서 B-1~B-3 (사용자 승인 2026-09-24)
#   R_TARGET 11.0 / R_ACC 10.0 / 이탈 상한 2.0m / 고정 노드 여유 2.0m / 호 노드 간격 2.0m
#   f>=0.30 은 공학적 선택값이며 충돌 방지 충분성은 주행 검증으로만 확인된다.
# 실행: ~/campus_mobility_sim/.venv-carla/bin/python map/scripts/smooth_osm_curves.py
import argparse
import collections
import copy
import csv
import datetime
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import curve_metrics as cm                                    # noqa: E402
from osm_to_xodr import convert, build_settings, capture_stderr  # noqa: E402

BASE = Path(__file__).resolve().parent.parent
DEFAULT_IN = BASE / "data" / "processed" / "cbnu_internal_boundary_tags73.osm"
DEFAULT_OUT = BASE / "data" / "processed" / "cbnu_internal_boundary_tags73_smooth.osm"
LOG_DIR = BASE / "docs" / "logs"

ARC_ID_BASE = 9_000_000_000_000   # 신규 노드 id 시작(실제 OSM id 는 약 1.3e10 이하)


class Params:
    def __init__(self, a):
        self.r_target = a.r_target
        self.r_acc = a.r_acc
        self.d_max = a.d_max
        self.th_cause = a.th_cause
        self.th_absorb = a.th_absorb
        self.arc_step = a.arc_step
        self.m_fixed = a.m_fixed
        self.near_m = a.near_m
        self.max_iter = a.max_iter
        self.max_align_err = a.max_align_err

    def as_dict(self):
        return dict(self.__dict__)


# ---------------- 상태 ----------------
class State:
    def __init__(self, path):
        self.orig_root, self.nodes, self.ways, self.tagged = cm.load_osm(path)
        self.orig_ways = {w: list(n) for w, n in self.ways.items()}
        self.new_nodes = {}          # id -> (e, n)
        self.next_id = ARC_ID_BASE
        self.changed_ways = set()

    def occ(self):
        return collections.Counter(n for nds in self.ways.values() for n in nds)

    def fixed_set(self):
        """다른 way 와 공유되거나 way 끝점이거나 태그가 있는 노드는 건드리지 않는다."""
        occ = self.occ()
        fixed = {n for n, c in occ.items() if c > 1} | set(self.tagged)
        for nds in self.ways.values():
            fixed.add(nds[0])
            fixed.add(nds[-1])
        return fixed

    def to_xml(self):
        root = copy.deepcopy(self.orig_root)
        used = set()
        for w in root.findall("way"):
            wid = w.get("id")
            if wid in self.changed_ways:
                tags = w.findall("tag")
                for nd in w.findall("nd"):
                    w.remove(nd)
                for t in tags:
                    w.remove(t)
                for ref in self.ways[wid]:
                    ET.SubElement(w, "nd", ref=ref)
                for t in tags:
                    w.append(t)
            used.update(nd.get("ref") for nd in w.findall("nd"))
        first_way = next(i for i, ch in enumerate(root) if ch.tag == "way")
        for k, (nid, (e, n)) in enumerate(sorted(self.new_nodes.items(), key=lambda kv: int(kv[0]))):
            lat, lon = cm.to_ll(e, n)
            el = ET.Element("node", id=nid, lat=f"{lat:.9f}", lon=f"{lon:.9f}", version="1")
            root.insert(first_way + k, el)
        for n in list(root.findall("node")):
            if n.get("id") not in used and not n.findall("tag"):
                root.remove(n)
        return root


# ---------------- 필렛 ----------------
def fillet_arc(p0, p1, p2, R, h):
    """꼭짓점 p1 을 반경 R 원호로 모따기. 반환 (호 위 점 리스트, 접선길이 T, 꼭짓점~호 중점 이탈)."""
    a = (p1[0] - p0[0], p1[1] - p0[1])
    b = (p2[0] - p1[0], p2[1] - p1[1])
    la, lb = math.hypot(*a), math.hypot(*b)
    u1 = (a[0] / la, a[1] / la)
    u2 = (b[0] / lb, b[1] / lb)
    th = math.atan2(u1[0] * u2[1] - u1[1] * u2[0], u1[0] * u2[0] + u1[1] * u2[1])
    ath = abs(th)
    T = R * math.tan(ath / 2)
    A = (p1[0] - u1[0] * T, p1[1] - u1[1] * T)
    sgn = 1 if th > 0 else -1
    nrm = (-u1[1] * sgn, u1[0] * sgn)
    C = (A[0] + nrm[0] * R, A[1] + nrm[1] * R)
    n = max(int(math.ceil(R * ath / h)), 1)
    a0 = math.atan2(A[1] - C[1], A[0] - C[0])
    pts = []
    for k in range(n + 1):
        ang = a0 + sgn * ath * k / n
        pts.append((C[0] + R * math.cos(ang), C[1] + R * math.sin(ang)))
    return pts, T, R * (1 / math.cos(ath / 2) - 1)


def predicted_dev(theta_deg, R):
    return R * (1 / math.cos(math.radians(abs(theta_deg)) / 2) - 1)


def plan_way(st, wid, targets, fixed, P):
    """way 안의 목표 노드 index 집합 -> {k: ('apply'|'skip', R)}. 접선 길이가 이웃 blocking
    노드(고정/목표/꺾임 th_absorb 이상)와 겹치면 R 을 축소하고, R_ACC 미만이면 건너뛴다."""
    nds = st.ways[wid]
    cum = cm.path_cum(st.nodes_all, nds)
    th = {}
    for k in range(1, len(nds) - 1):
        th[k] = cm.turn_deg(st.nodes_all[nds[k - 1]], st.nodes_all[nds[k]], st.nodes_all[nds[k + 1]])
    active = set(targets)
    decisions = {}
    Rk = {k: P.r_target for k in active}

    def blocker(k, d):
        j = k + d
        while 0 <= j < len(nds):
            if j in (0, len(nds) - 1) or nds[j] in fixed:
                return j, "fixed"
            if j in active:
                return j, "target"
            if abs(th[j]) >= P.th_absorb:
                return j, "kink"
            j += d
        return None, None

    while True:
        for _ in range(60):
            changed = False
            for k in sorted(active):
                for d in (-1, 1):
                    j, kind = blocker(k, d)
                    if j is None:
                        continue
                    dist = abs(cum[j] - cum[k]) - (P.m_fixed if kind == "fixed" else 0.0)
                    Tk = Rk[k] * math.tan(math.radians(abs(th[k])) / 2)
                    Tj = Rk[j] * math.tan(math.radians(abs(th[j])) / 2) if kind == "target" else 0.0
                    if Tk + Tj > dist + 1e-9:
                        f = 0.0 if dist <= 0 else dist / (Tk + Tj) * 0.999
                        Rk[k] *= f
                        if kind == "target":
                            Rk[j] *= f
                        changed = True
            if not changed:
                break
        low = sorted(k for k in active if Rk[k] < P.r_acc)
        if not low:
            break
        for k in low:
            decisions[k] = ("skip", Rk[k])
            active.discard(k)
        for k in active:
            Rk[k] = P.r_target
    for k in active:
        decisions[k] = ("apply", Rk[k])
    return decisions, th, cum


def apply_way(st, wid, decisions, th, cum, P, it, change_rows):
    nds = st.ways[wid]
    applied = {k: R for k, (kind, R) in decisions.items() if kind == "apply"}
    if not applied:
        return False
    zones = [(cum[k] - R * math.tan(math.radians(abs(th[k])) / 2),
              cum[k] + R * math.tan(math.radians(abs(th[k])) / 2), k) for k, R in applied.items()]
    absorbed = {k for k in range(1, len(nds) - 1) if k not in applied
                and any(lo < cum[k] < hi for lo, hi, _ in zones) and abs(th[k]) < P.th_absorb}
    out = [nds[0]]
    for k in range(1, len(nds) - 1):
        if k in absorbed:
            continue
        if k in applied:
            pts, T, dev = fillet_arc(st.nodes_all[nds[k - 1]], st.nodes_all[nds[k]],
                                     st.nodes_all[nds[k + 1]], applied[k], P.arc_step)
            ids = []
            for p in pts:
                st.next_id += 1
                nid = str(st.next_id)
                st.new_nodes[nid] = p
                st.nodes_all[nid] = p
                ids.append(nid)
            out.extend(ids)
            change_rows.append({
                "iteration": it, "way_id": wid, "node_id": nds[k], "theta_deg": round(th[k], 2),
                "R_m": round(applied[k], 2), "T_m": round(T, 2), "dev_vertex_arc_m": round(dev, 2),
                "n_new_nodes": len(ids), "first_new_id": ids[0],
                "absorbed_nodes": ";".join(nds[x] for x in sorted(absorbed)
                                           if abs(cum[x] - cum[k]) <= T)})
        else:
            out.append(nds[k])
    out.append(nds[-1])
    st.ways[wid] = out
    st.changed_ways.add(wid)
    return True


# ---------------- 측정 -> 대상 선정 ----------------
def measure(st, P):
    text = st.to_xml()
    box = []
    with capture_stderr(box):
        xodr = convert(ET.tostring(text, encoding="unicode"), build_settings())
    root, roads, juncs, elems = cm.load_xodr(xodr, is_text=True)
    sites = cm.scan_sites(roads, elems)
    return xodr, roads, juncs, sites


def select_targets(st, roads, juncs, sites, P, tried):
    """문제 구간 -> OSM 위치 -> 원인 후보 노드. 반환 (way->set(k), 제외 기록)"""
    groups = cm.group_sites(sites, cm.F_MIN)
    segs, S = cm.build_segs(st.nodes_all, st.ways)
    al = cm.align_icp(roads, juncs, st.nodes_all, S)
    fixed = st.fixed_set()
    targets = collections.defaultdict(set)
    excluded = []
    if not groups:
        return targets, excluded, groups, al
    P_en = al.fwd([[g["x"], g["y"]] for g in groups])
    d, bi, bu = cm.nearest_on_segs(S, P_en)
    for g, dist, si, ui in zip(groups, d, bi, bu):
        wid, i = segs[si][0], segs[si][1]
        if dist > P.max_align_err:
            excluded.append({"way_id": wid, "node_id": "", "theta_deg": "", "reason": f"정렬오차 {dist:.2f}m",
                             "R_m": ""})
            continue
        nds = st.ways[wid]
        cum = cm.path_cum(st.nodes_all, nds)
        pos = cum[i] + ui * (cum[i + 1] - cum[i])
        for k in range(1, len(nds) - 1):
            if abs(cum[k] - pos) > P.near_m or nds[k] in fixed or nds[k] in st.new_nodes:
                continue
            if (wid, nds[k]) in tried:
                continue
            th = cm.turn_deg(st.nodes_all[nds[k - 1]], st.nodes_all[nds[k]], st.nodes_all[nds[k + 1]])
            if abs(th) < P.th_cause:
                continue
            if predicted_dev(th, P.r_target) > P.d_max:
                excluded.append({"way_id": wid, "node_id": nds[k], "theta_deg": round(th, 2),
                                 "reason": f"이탈 상한 초과 예상 {predicted_dev(th, P.r_target):.2f}m>{P.d_max}m",
                                 "R_m": ""})
                tried.add((wid, nds[k]))
                continue
            targets[wid].add(k)
    return targets, excluded, groups, al


# ---------------- 메인 ----------------
def main():
    ap = argparse.ArgumentParser(description="인도 오프셋 접힘 커브의 OSM 노드를 필렛한 사본 생성")
    ap.add_argument("--input", type=Path, default=DEFAULT_IN)
    ap.add_argument("--output", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--log-prefix", default="session11_smoothing")
    ap.add_argument("--r-target", type=float, default=11.0)
    ap.add_argument("--r-acc", type=float, default=10.0)
    ap.add_argument("--d-max", type=float, default=2.0)
    ap.add_argument("--th-cause", type=float, default=8.0)
    ap.add_argument("--th-absorb", type=float, default=10.0)
    ap.add_argument("--arc-step", type=float, default=2.0)
    ap.add_argument("--m-fixed", type=float, default=2.0)
    ap.add_argument("--near-m", type=float, default=8.0)
    ap.add_argument("--max-iter", type=int, default=3)
    ap.add_argument("--max-align-err", type=float, default=3.0)
    a = ap.parse_args()
    if a.output.resolve() == a.input.resolve():
        sys.exit("입력과 출력이 같다: 원본은 덮어쓰지 않는다")
    P = Params(a)
    st = State(a.input)
    st.nodes_all = dict(st.nodes)          # 원본 + 신규 노드 좌표
    change_rows, skip_rows, history = [], [], []
    tried = set()
    for it in range(1, P.max_iter + 2):
        xodr, roads, juncs, sites = measure(st, P)
        g03 = cm.group_sites(sites, cm.F_MIN)
        g0 = cm.group_sites(sites, 0.0)
        n_roads = len(roads)
        n_j = len(juncs)
        history.append({"iteration": it, "road": n_roads, "junction": n_j,
                        "sites_f_lt_0.30": len(g03), "sites_f_lt_0": len(g0)})
        print(f"[iter {it}] road {n_roads} junction {n_j} 문제구간 f<0.30: {len(g03)} f<0: {len(g0)}")
        if it > P.max_iter:
            break
        targets, excluded, groups, al = select_targets(st, roads, juncs, sites, P, tried)
        for e in excluded:
            e["iteration"] = it
            skip_rows.append(e)
        if not targets:
            print(f"  신규 대상 없음 -> 종료 (정렬 median {al.median:.2f}m)")
            break
        fixed = st.fixed_set()
        n_apply = 0
        for wid in sorted(targets):
            decisions, th, cum = plan_way(st, wid, targets[wid], fixed, P)
            for k, (kind, R) in decisions.items():
                tried.add((wid, st.ways[wid][k]))
                if kind == "skip":
                    skip_rows.append({"iteration": it, "way_id": wid, "node_id": st.ways[wid][k],
                                      "theta_deg": round(th[k], 2), "R_m": round(R, 2),
                                      "reason": f"접선 제약으로 R {R:.1f}m < R_ACC {P.r_acc}m"})
            # 노드 index 는 apply 로 바뀌므로 way 별 1회 적용
            before = len(change_rows)
            apply_way(st, wid, decisions, th, cum, P, it, change_rows)
            n_apply += len(change_rows) - before
        print(f"  대상 way {len(targets)}개, 필렛 적용 {n_apply}개 (정렬 median {al.median:.2f}m)")
        if n_apply == 0:
            # 더 할 수 있는 것이 없다. 현재 상태가 마지막 측정과 같으므로 종료
            break
    # ---- 출력 ----
    root = st.to_xml()
    a.output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(a.output, encoding="UTF-8", xml_declaration=True)
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    ch = LOG_DIR / f"{a.log_prefix}_changes.csv"
    sk = LOG_DIR / f"{a.log_prefix}_skipped.csv"
    with open(ch, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["iteration", "way_id", "node_id", "theta_deg", "R_m", "T_m",
                                          "dev_vertex_arc_m", "n_new_nodes", "first_new_id", "absorbed_nodes"])
        w.writeheader()
        w.writerows(change_rows)
    with open(sk, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["iteration", "way_id", "node_id", "theta_deg", "R_m", "reason"])
        w.writeheader()
        for r in skip_rows:
            w.writerow({k: r.get(k, "") for k in w.fieldnames})
    devs = [r["dev_vertex_arc_m"] for r in change_rows]
    report = {
        "실행시각": datetime.datetime.now().isoformat(timespec="seconds"),
        "입력": str(a.input), "출력": str(a.output), "params": P.as_dict(),
        "history": history, "필렛수": len(change_rows), "변경way수": len(st.changed_ways),
        "신규노드수": len(st.new_nodes), "건너뜀": len(skip_rows),
        "꼭짓점_호_이탈_최대": max(devs) if devs else 0.0,
    }
    (LOG_DIR / f"{a.log_prefix}_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"필렛 {len(change_rows)}개, 변경 way {len(st.changed_ways)}개, 신규 노드 {len(st.new_nodes)}개, "
          f"건너뜀 {len(skip_rows)}건")
    print(f"출력 OSM: {a.output}")
    print(f"로그: {ch.name}, {sk.name}, {a.log_prefix}_report.json")


if __name__ == "__main__":
    main()
