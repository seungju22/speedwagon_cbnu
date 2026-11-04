#!/usr/bin/env python3
# 10/16: v3.2 후보 Lanelet2 만들기(서버 없음). 28a57711 사본에 요소를 더하기만 함
#   횡단보도 = crosswalk 레인렛(subtype crosswalk, 경계 pedestrian_marking) + crosswalk_polygon 면 + crosswalk 규제 요소
#   과속방지턱 = speed_bump 면(area yes, height) + speed_bump 규제 요소
#   두 규제 요소 모두 겹치는 도로 레인렛이 regulatory_element 멤버로 참조(behavior_velocity 모듈이 경로 레인렛에서 찾음)
# 형식 근거: autoware_lanelet2_extension docs/extra_regulatory_elements.md(Crosswalk, Speed Bump),
#   1.9.0 이미지 include/.../regulatory_elements/crosswalk.hpp·speed_bump.hpp(RuleName crosswalk/speed_bump, 역할 crosswalk_polygon)
# 넣을 지점: crossings_v3_candidates.csv. keep 이 차 있으면 따름, 비면 OSM 태그(footway=crossing -> 횡단보도, traffic_calming -> 방지턱)만
# 자리: CSV carla_x, -carla_y(= map 좌표). 방향은 가장 가까운 도로 레인렛 진행 방향, 폭은 그 단면에서 나란한 레인렛을 이은 차도 폭
# 기존 줄은 지우거나 바꾸지 않음. 도로 레인렛 relation 에 member 한 줄씩 더하고, 새 node·way·relation 을 각 묶음 끝에 붙임
# 사용: .venv-carla/bin/python map/scripts/s63_v32_candidate.py
import csv
import hashlib
import math
import re
import shutil
import sys
from pathlib import Path

from shapely.geometry import LineString, Point, Polygon

C = Path(__file__).resolve().parents[2]
SRC = C / "autoware_map/cbnu_campus_v31/lanelet2_map.osm"
SRC_SHA = "28a57711094effaf3e0523cfd103e94a010c2bac01d37feebd0de90438dd02f9"
CSV = C / "map/data/crossings_v3_candidates.csv"
OUT = C / "map/maps/v32_candidate"

# 가정값(OSM 에 없음)
CW_WIDTH = 4.0      # 횡단보도 폭(차 진행 방향) m
CW_EXT = 1.0        # 횡단보도를 차도 가장자리 밖으로 늘리는 길이(보도 쪽 대기 자리) m
SB_DEPTH = 3.6      # 방지턱 길이(차 진행 방향) m, 원호형 기준
SB_HEIGHT = 0.10    # 방지턱 높이 m
NEAR_M = 1.5        # 후보 점에서 도로 레인렛까지 이 안이어야 넣음
PAR_COS = 0.8       # 단면에서 나란한 레인렛으로 볼 방향 cos
GAP_M = 0.3         # 단면 구간 잇기 허용 틈
ID0 = {"node": 900001, "way": 910001, "rel": 920001}
DEG_PER_M = 1.0 / 111319.49  # l1_build_lanelet2.py 와 같은 참고용 근사(Local 투영은 local_x/local_y 를 씀)
KEEP_Y = {"1", "y", "yes", "o", "keep", "남김", "true"}
KEEP_N = {"0", "n", "no", "x", "drop", "뺌", "false"}

log = []


def P(*a):
    s = " ".join(str(x) for x in a)
    log.append(s)
    print(s)


def parse(text):
    nodes, ways, lls = {}, {}, {}
    for m in re.finditer(r'<node id="(\d+)".*?</node>', text, re.S):
        b = m.group(0)
        nodes[int(m.group(1))] = (float(re.search(r'k="local_x" v="([-\d.]+)"', b).group(1)),
                                  float(re.search(r'k="local_y" v="([-\d.]+)"', b).group(1)))
    for m in re.finditer(r'<way id="(\d+)".*?</way>', text, re.S):
        ways[int(m.group(1))] = [int(x) for x in re.findall(r'<nd ref="(\d+)"', m.group(0))]
    for m in re.finditer(r'<relation id="(\d+)".*?</relation>', text, re.S):
        b = m.group(0)
        if 'k="subtype" v="road"' not in b:
            continue
        L = int(re.search(r'ref="(\d+)" role="left"', b).group(1))
        R = int(re.search(r'ref="(\d+)" role="right"', b).group(1))
        road = int(re.search(r'k="xodr_road" v="(\d+)"', b).group(1))
        lls[int(m.group(1))] = (L, R, road)
    return nodes, ways, lls


def seg_dir(pts, p):
    """pts 선에서 p 에 가장 가까운 조각의 단위 방향."""
    best = None
    for a, b in zip(pts[:-1], pts[1:]):
        d = LineString([a, b]).distance(Point(p))
        if best is None or d < best[0]:
            best = (d, a, b)
    _, a, b = best
    L = math.hypot(b[0] - a[0], b[1] - a[1]) or 1.0
    return ((b[0] - a[0]) / L, (b[1] - a[1]) / L)


def choose(rows):
    out = []
    for r in rows:
        k = r["keep"].strip().lower()
        src = r["sources"].split()
        kinds = []
        if "footway_crossing" in src or "osm_crossing" in src:
            kinds.append("crosswalk")
        if "calming" in src:
            kinds.append("speed_bump")
        if k in KEEP_Y:
            how = f"keep={r['keep']}"
            if not kinds:
                kinds = ["crosswalk"]  # 태그 없이 남김이면 횡단보도로 봄
        elif k in KEEP_N:
            how, kinds = f"keep={r['keep']}(뺌)", []
        elif k:
            how, kinds = f"keep={r['keep']}(읽지 못함, 넣지 않음)", []
        else:
            how = "keep 빈칸 -> OSM 태그(" + ",".join(s for s in src if s in ("footway_crossing", "osm_crossing", "calming")) + ")" if kinds \
                else "keep 빈칸 -> 태그 없음(" + ",".join(src) + "), 추정 후보라 안 넣음"
        out.append((r, kinds, how))
    return out


def main():
    raw = SRC.read_bytes()
    assert hashlib.sha256(raw).hexdigest() == SRC_SHA, "원본 해시 다름"
    text = raw.decode("utf-8")
    nodes, ways, lls = parse(text)
    for kind, base in ID0.items():
        pass
    maxid = max(max(nodes), max(ways), max(int(x) for x in re.findall(r'<relation id="(\d+)"', text)))
    assert maxid < ID0["node"], maxid
    P(f"원본 {SRC.relative_to(C)} sha256 {SRC_SHA[:8]}, node {len(nodes)} way {len(ways)} 도로 레인렛 {len(lls)}, 최대 id {maxid}")

    poly, bound = {}, {}
    for i, (L, R, road) in lls.items():
        lp = [nodes[n] for n in ways[L]]
        rp = [nodes[n] for n in ways[R]]
        poly[i] = Polygon(lp + rp[::-1]).buffer(0)
        bound[i] = (lp, rp)

    rows = list(csv.DictReader(CSV.open(encoding="utf-8")))
    sel = choose(rows)
    nid, wid, rid = ID0["node"], ID0["way"], ID0["rel"]
    new_nodes, new_ways, new_rels = [], [], []
    add_member = {}  # 도로 레인렛 id -> [규제 요소 id]
    table = []

    def mk_node(x, y):
        nonlocal nid
        i = nid
        nid += 1
        new_nodes.append(f'  <node id="{i}" visible="true" version="1" lat="{y * DEG_PER_M:.11f}" lon="{x * DEG_PER_M:.11f}">\n'
                         f'    <tag k="ele" v="0.000000" />\n    <tag k="local_x" v="{x:.6f}" />\n    <tag k="local_y" v="{y:.6f}" />\n  </node>\n')
        return i

    def mk_way(nids, tags):
        nonlocal wid
        i = wid
        wid += 1
        s = f'  <way id="{i}" visible="true" version="1">\n' + "".join(f'    <nd ref="{n}" />\n' for n in nids)
        s += "".join(f'    <tag k="{k}" v="{v}" />\n' for k, v in tags) + "  </way>\n"
        new_ways.append(s)
        return i

    def mk_rel(members, tags):
        nonlocal rid
        i = rid
        rid += 1
        s = f'  <relation id="{i}" visible="true" version="1">\n'
        s += "".join(f'    <member type="{t}" ref="{r}" role="{ro}" />\n' for t, r, ro in members)
        s += "".join(f'    <tag k="{k}" v="{v}" />\n' for k, v in tags) + "  </relation>\n"
        new_rels.append(s)
        return i

    for r, kinds, how in sel:
        if not kinds:
            table.append((r["id"], "-", how, "", "", "", ""))
            continue
        p = (float(r["carla_x"]), -float(r["carla_y"]))
        pt = Point(p)
        near = sorted((poly[i].distance(pt), i) for i in lls if poly[i].distance(pt) < 15)
        if not near or near[0][0] > NEAR_M:
            P(f"{r['id']} 도로 레인렛이 {NEAR_M}m 안에 없음(가장 가까운 {near[0][0] if near else '-'}), 안 넣음")
            table.append((r["id"], "+".join(kinds), how + " / 자리 못 찾음", "", "", "", ""))
            continue
        i0 = near[0][1]
        t = seg_dir(bound[i0][0], p)
        n = (-t[1], t[0])
        cross = LineString([(p[0] - 20 * n[0], p[1] - 20 * n[1]), (p[0] + 20 * n[0], p[1] + 20 * n[1])])
        iv = []
        for d, i in near:
            ti = seg_dir(bound[i][0], p)
            if abs(ti[0] * t[0] + ti[1] * t[1]) < PAR_COS:
                continue
            g = poly[i].intersection(cross)
            for part in getattr(g, "geoms", [g]):
                if part.is_empty or part.length == 0:
                    continue
                us = [(c[0] - p[0]) * n[0] + (c[1] - p[1]) * n[1] for c in part.coords]
                iv.append((min(us), max(us), i))
        iv.sort()
        # u=0 을 품는 이어진 구간
        lo = hi = 0.0
        changed = True
        while changed:
            changed = False
            for a, b, _ in iv:
                if a <= hi + GAP_M and b >= lo - GAP_M and (a < lo or b > hi):
                    lo, hi = min(lo, a), max(hi, b)
                    changed = True
        road_w = hi - lo
        for kind in kinds:
            ext = CW_EXT if kind == "crosswalk" else 0.0
            half = (CW_WIDTH if kind == "crosswalk" else SB_DEPTH) / 2
            u0, u1 = lo - ext, hi + ext

            def at(u, v):
                return (p[0] + u * n[0] + v * t[0], p[1] + u * n[1] + v * t[1])
            rect = Polygon([at(u0, -half), at(u1, -half), at(u1, half), at(u0, half)])
            hits = sorted(i for i in lls if poly[i].intersection(rect).area > 0.05)
            src_ref = r["source_refs"].split(";")[0].strip()
            if kind == "crosswalk":
                # 레인렛 진행 = 사람 건너는 방향(u). 왼쪽 경계 v=+half, 오른쪽 v=-half
                a1, a2 = mk_node(*at(u0, half)), mk_node(*at(u1, half))
                b1, b2 = mk_node(*at(u0, -half)), mk_node(*at(u1, -half))
                lw = mk_way([a1, a2], [("type", "pedestrian_marking")])
                rw = mk_way([b1, b2], [("type", "pedestrian_marking")])
                pw = mk_way([b1, b2, a2, a1], [("type", "crosswalk_polygon"), ("area", "yes")])
                cl = mk_rel([("way", lw, "left"), ("way", rw, "right")],
                            [("type", "lanelet"), ("subtype", "crosswalk"), ("location", "urban"), ("one_way", "no"),
                             ("participant:pedestrian", "yes"), ("speed_limit", "10"), ("s63_src", r["id"])])
                reg = mk_rel([("relation", cl, "refers"), ("way", pw, "crosswalk_polygon")],
                             [("type", "regulatory_element"), ("subtype", "crosswalk"), ("s63_src", r["id"])])
                elem = f"crosswalk ll {cl} reg {reg}"
            else:
                c = [mk_node(*at(u0, -half)), mk_node(*at(u1, -half)), mk_node(*at(u1, half)), mk_node(*at(u0, half))]
                pw = mk_way(c, [("type", "speed_bump"), ("area", "yes"), ("height", f"{SB_HEIGHT:.2f}")])
                reg = mk_rel([("way", pw, "refers")], [("type", "regulatory_element"), ("subtype", "speed_bump"), ("s63_src", r["id"])])
                elem = f"speed_bump way {pw} reg {reg}"
            for i in hits:
                add_member.setdefault(i, []).append(reg)
            roads = sorted({lls[i][2] for i in hits})
            P(f"{r['id']} {kind}: map ({p[0]:.2f},{p[1]:.2f}) r{r['road_id']} 가장 가까운 레인렛 {i0}(r{lls[i0][2]}) {near[0][0]:.2f}m, "
              f"차도 폭 {road_w:.2f}m, 참조 레인렛 {len(hits)}개 roads {roads}, {elem}, OSM {src_ref}")
            table.append((r["id"], kind, how, f"{p[0]:.2f}", f"{p[1]:.2f}", f"{road_w:.2f}",
                          " ".join(str(i) for i in hits), elem, src_ref))

    # 원본 텍스트에 더하기만
    out = text
    for i, regs in add_member.items():
        m = re.search(rf'  <relation id="{i}" visible="true" version="1">\n(.*?)  </relation>\n', out, re.S)
        body = m.group(1)
        last = [mm for mm in re.finditer(r'    <member [^\n]*\n', body)][-1]
        ins = "".join(f'    <member type="relation" ref="{g}" role="regulatory_element" />\n' for g in regs)
        start = m.start(1) + last.end()
        out = out[:start] + ins + out[start:]
    k = out.rfind("  </node>\n") + len("  </node>\n")
    out = out[:k] + "".join(new_nodes) + out[k:]
    k = out.rfind("  </way>\n") + len("  </way>\n")
    out = out[:k] + "".join(new_ways) + out[k:]
    k = out.rfind("</osm>")
    out = out[:k] + "".join(new_rels) + out[k:]

    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "lanelet2_map.osm").write_text(out, encoding="utf-8")
    shutil.copy2(SRC.parent / "map_projector_info.yaml", OUT / "map_projector_info.yaml")
    with (OUT / "v32_elements.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["id", "kind", "how", "map_x", "map_y", "road_width_m", "ref_lanelets", "element", "osm_ref"])
        w.writerows(table)
    ncw = sum(1 for t in table if t[1] == "crosswalk" and len(t) > 7)
    nsb = sum(1 for t in table if t[1] == "speed_bump" and len(t) > 7)
    P(f"넣음: 횡단보도 {ncw}, 방지턱 {nsb}, 참조를 더한 도로 레인렛 {len(add_member)}개, 새 node {len(new_nodes)} way {len(new_ways)} relation {len(new_rels)}")
    P(f"가정값: 횡단보도 폭 {CW_WIDTH}m(+차도 밖 {CW_EXT}m 씩), 방지턱 길이 {SB_DEPTH}m 높이 {SB_HEIGHT}m")
    P(f"출력 {OUT.relative_to(C)}/lanelet2_map.osm sha256 {hashlib.sha256(out.encode()).hexdigest()}")
    (OUT / "s63_build.log").write_text("\n".join(log) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
