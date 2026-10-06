#!/usr/bin/env python3
# 맵세션23 Phase 0 추가: 막다른 끝 회차 커넥터 누락 원인 비교 (읽기전용, 서버 없음, xodr/OSM 미수정).
# 회차를 받은 막다른 끝(정의된 junction, 커넥터 1개, U턴형) vs 못 받은 끝(참조만 있고 정의 없는 junction)
# 을 변환 입력 OSM(CONV_OSM) 기준 특성으로 나란히 놓는다.
# 노드 대응: 정의된 junction 은 name = OSM 노드 id. 정의 없는 junction 은 그 junction 을 참조하는
#   road 의 참조선 끝점(planView)에서 가장 가까운 변환 way 노드.
# 실행: .venv-carla/bin/python map/scripts/s23_deadend_cause.py [xodr] > map/docs/logs/s23_deadend_cause.log
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, "/home/gyumin/campus_mobility_sim/map/scripts")
from s19_scope_common import RAW_OSM, CONV_OSM, XODR_V1, load_osm, ll_to_carla  # noqa: E402

XODR = Path(sys.argv[1]) if len(sys.argv) > 1 else XODR_V1


def ref_end(road, at_end):
    """road 참조선의 시작/끝 점 (xodr 좌표 -> CARLA y 반전)."""
    geos = road.find("planView").findall("geometry")
    g = geos[-1] if at_end else geos[0]
    x, y, h, L = (float(g.get(k)) for k in ("x", "y", "hdg", "length"))
    if not at_end:
        return x, -y
    if g.find("line") is not None:
        return x + L * math.cos(h), -(y + L * math.sin(h))
    if g.find("arc") is not None:
        k = float(g.find("arc").get("curvature"))
        return x + (math.sin(h + k * L) - math.sin(h)) / k, -(y - (math.cos(h + k * L) - math.cos(h)) / k)
    return x + L * math.cos(h), -(y + L * math.sin(h))   # 그 밖(spiral/poly) 근사


def main():
    root = ET.parse(str(XODR)).getroot()
    roads = {r.get("id"): r for r in root.iter("road")}
    jdef = {j.get("id"): j for j in root.iter("junction")}
    jref = {}
    for rd in roads.values():
        link = rd.find("link")
        if link is None:
            continue
        for e in link:
            if e.get("elementType") == "junction":
                jref.setdefault(e.get("elementId"), []).append((rd.get("id"), e.tag))
    cn, cw = load_osm(CONV_OSM)
    rn, rw = load_osm(RAW_OSM)
    conv = {w: d for w, d in cw.items() if d["tags"].get("highway") == "unclassified"}
    nxy = {n: ll_to_carla(*ll) for n, ll in cn.items()}
    at_conv, at_all, at_raw = {}, {}, {}
    for wid, w in cw.items():
        if "highway" not in w["tags"]:
            continue
        for i, n in enumerate(w["nds"]):
            at_all.setdefault(n, []).append(wid)
            if wid in conv:
                at_conv.setdefault(n, []).append((wid, i))
    for wid, w in rw.items():
        if "highway" in w["tags"]:
            for n in w["nds"]:
                at_raw.setdefault(n, set()).add(wid)

    # 막다른 끝 = 정의된 junction 중 커넥터 1개(회차) / 정의 없는 junction
    got, miss = [], []
    for jid, j in jdef.items():
        cs = j.findall("connection")
        if len(cs) == 1 and cs[0].get("incomingRoad") and len(jref.get(jid, [])) <= 3:
            inc = cs[0].get("incomingRoad")
            got.append((jid, j.get("name"), inc))
    for jid in set(jref) - set(jdef):
        refs = jref[jid]
        inc = [r for r, t in refs if t == "successor"]
        rid = inc[0] if inc else refs[0][0]
        at_end = bool(inc)
        p = ref_end(roads[rid], at_end)
        n = min(at_conv, key=lambda n: math.dist(nxy[n], p))
        miss.append((jid, n, rid, math.dist(nxy[n], p)))

    def feats(node):
        ws = at_conv.get(node, [])
        out = []
        for wid, i in ws:
            w = conv[wid]
            nd = w["nds"]
            L = sum(math.dist(nxy[a], nxy[b]) for a, b in zip(nd, nd[1:]))
            term = i in (0, len(nd) - 1)
            if term:
                prev = nd[1] if i == 0 else nd[-2]
                last = math.dist(nxy[node], nxy[prev])
            else:
                last = float("nan")
            t = w["tags"]
            tagset = ",".join(f"{k}={v}" for k, v in sorted(t.items()) if k != "highway")
            out.append(dict(wid=wid, L=L, nn=len(nd), term=term, last=last, tags=tagset,
                            oneway=t.get("oneway", "-"), width=t.get("width", "-"),
                            lanes=t.get("lanes", "-"), ring=(nd[0] == nd[-1])))
        return out

    def show(label, rows):
        print(f"\n## {label} {len(rows)}")
        stats = []
        for jid, node, inc, extra in rows:
            fs = feats(node)
            deg_conv = len(at_conv.get(node, []))
            deg_all = len(at_all.get(node, []))
            deg_raw = len(at_raw.get(node, set()))
            raw_node = node in rn
            negid = node.startswith("-")
            sw = any(l.get("type") == "sidewalk" for l in roads[inc].iter("lane")) if inc in roads else None
            ln = float(roads[inc].get("length")) if inc in roads else float("nan")
            for f in fs:
                print(f"j{jid} node{node} 입력road r{inc} {ln:.1f}m 보도 {'O' if sw else 'X'} | way{f['wid']}"
                      f" {f['L']:.1f}m 노드{f['nn']} 끝노드 {'O' if f['term'] else 'X'} 끝구간 {f['last']:.1f}m"
                      f" oneway {f['oneway']} width {f['width']} lanes {f['lanes']} 고리 {'O' if f['ring'] else 'X'}"
                      f" | 차수 변환way {deg_conv} / 입력OSM highway {deg_all} / 원본OSM {deg_raw}"
                      f" | 원본노드 {'O' if raw_node else 'X'}{' 음수id' if negid else ''} {extra}")
                print(f"    태그 {f['tags']}")
                stats.append((f, deg_conv, deg_all, deg_raw, ln))
        return stats

    print(f"# 막다른 끝 회차 비교 — {XODR.name}")
    sg = show("회차 받음(정의된 junction, 커넥터 1개)", [(j, n, i, "") for j, n, i in sorted(got, key=lambda x: int(x[0]))])
    sm = show("회차 못 받음(정의 없는 junction)",
              [(j, n, i, f"(노드 대응 {d:.1f}m)") for j, n, i, d in sorted(miss, key=lambda x: int(x[0]))])

    def summ(label, st):
        import statistics as S
        if not st:
            return
        f = [s[0] for s in st]
        print(f"\n### 요약 {label} (way 행 {len(st)})")
        print(f"way 길이 중앙값 {S.median(x['L'] for x in f):.1f} 범위 {min(x['L'] for x in f):.1f}~{max(x['L'] for x in f):.1f}")
        print(f"way 노드 수 중앙값 {S.median(x['nn'] for x in f)} 범위 {min(x['nn'] for x in f)}~{max(x['nn'] for x in f)}")
        lasts = [x['last'] for x in f if not math.isnan(x['last'])]
        if lasts:
            print(f"끝 구간 길이 중앙값 {S.median(lasts):.1f} 범위 {min(lasts):.1f}~{max(lasts):.1f}")
        print(f"끝 노드가 way 끝: {sum(x['term'] for x in f)}/{len(f)}")
        print(f"oneway=yes: {sum(x['oneway'] == 'yes' for x in f)}, width 태그: {sum(x['width'] != '-' for x in f)},"
              f" lanes 태그: {sum(x['lanes'] != '-' for x in f)}, 고리 way: {sum(x['ring'] for x in f)}")
        from collections import Counter
        print(f"태그 구성: {dict(Counter(x['tags'] for x in f))}")
        print(f"차수(변환way): {dict(Counter(s[1] for s in st))}")
        print(f"차수(입력OSM highway 전체): {dict(Counter(s[2] for s in st))}")
        print(f"차수(원본OSM highway 전체): {dict(Counter(s[3] for s in st))}")
        print(f"입력 road 길이 중앙값 {S.median(s[4] for s in st):.1f} 범위 {min(s[4] for s in st):.1f}~{max(s[4] for s in st):.1f}")
    summ("회차 받음", sg)
    summ("회차 못 받음", sm)


if __name__ == "__main__":
    main()
