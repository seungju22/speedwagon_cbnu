#!/usr/bin/env python3
# 11/04: v3.3d = v3.3c(map/maps/v33c, d2935e5b) 에서 후문 앞 유턴만 바꿈. xodr·CARLA 월드는 v3.1 그대로, Lanelet2 만
#   11/04 판정: 후문 앞은 실제 공간 있음 -> 스프린터가 도는 반경(뒷축 4.1m 이상)으로 다시 잡음
#   후문 앞 길 끝은 섬(약 20 x 9m)을 두른 고리. 섬 동쪽 끝 49898(r1953, 반경 2.94m)과 열쇠 구멍 유턴 52800 을 지우고
#   34923(r1391, 섬 남쪽 남동행) i 점 -> 34884(r1390, 섬 북쪽 북서행) j 점을 Dubins 경로(s77_uturn_search.py 가 고름)로 잇는 유턴 레인렛 하나를 더함
#   나누기·경계 벌리기·끝 늘림(--ext)·centerline 은 s75_make_v33c.py 와 같은 방법(그 2·3단계를 그대로 씀)
# 출력: map/maps/v33d/{lanelet2_map.osm, map_projector_info.yaml, v33d_changes.json}
# 사용: python3 map/scripts/s77_make_v33d.py [--path integration/logs/s77/uturn_island_search_path.json] [--wl 3.0] [--wr 3.0] [--ext 8]
import argparse, hashlib, json, math, re, shutil
from pathlib import Path

C = Path(__file__).resolve().parents[2]
ap = argparse.ArgumentParser(); ap.add_argument("--src", default=str(C / "map/maps/v33c"))
ap.add_argument("--path", default=str(C / "integration/logs/s77/uturn_island_search_path.json"))
ap.add_argument("--wl", type=float, default=3.0); ap.add_argument("--wr", type=float, default=3.0)
ap.add_argument("--n14-sig", type=float, default=6.0); ap.add_argument("--n14-d0", type=float, default=2.5); ap.add_argument("--n14-w", type=float, default=20.0)
ap.add_argument("--ext", type=int, default=8); ap.add_argument("--wpow", type=float, default=0.4)
ap.add_argument("--n14-c0", type=float, default=-4.0); ap.add_argument("--n14-e", type=float, default=0.0)
ap.add_argument("--out", default=str(C / "map/maps/v33d")); a = ap.parse_args()
SRC = Path(a.src); OUT = Path(a.out); OUT.mkdir(parents=True, exist_ok=True)
P = json.loads(Path(a.path).read_text()); pick = P["pick"]; pts = P["pts"]
text = (SRC / "lanelet2_map.osm").read_text()
NODE_RE = r'(<node id="(\d+)" visible="true" version="1" lat=")([-\d.]+)(" lon=")([-\d.]+)(">\s*<tag k="ele" v="[-\d.]+" />\s*<tag k="local_x" v=")([-\d.]+)(" />\s*<tag k="local_y" v=")([-\d.]+)(" />)'
nodes = {m.group(2): (float(m.group(7)), float(m.group(9))) for m in re.finditer(NODE_RE, text)}
ll_ = {m.group(2): (float(m.group(3)), float(m.group(5))) for m in re.finditer(NODE_RE, text)}
ks = list(nodes)[:2000]
ky = sum(ll_[k][0] * nodes[k][1] for k in ks) / sum(nodes[k][1] ** 2 for k in ks)
kx = sum(ll_[k][1] * nodes[k][0] for k in ks) / sum(nodes[k][0] ** 2 for k in ks)
WAY_RE = r'  <way id="(\d+)" visible="true" version="1">\n(.*?)  </way>\n'
REL_RE = r'  <relation id="(\d+)" visible="true" version="1">\n(.*?)  </relation>\n'
ways = {m.group(1): re.findall(r'<nd ref="(\d+)" />', m.group(2)) for m in re.finditer(WAY_RE, text, re.S)}
rel_body = {m.group(1): m.group(2) for m in re.finditer(REL_RE, text, re.S)}
mem = lambda rid: dict((r, ref) for ref, r in re.findall(r'<member type="way" ref="(\d+)" role="(\w+)" />', rel_body[rid]))
nid = max(int(x) for x in nodes) + 1; wid = max(int(x) for x in ways) + 1; rid_n = max(int(x) for x in rel_body) + 1
info = {}

# ---- 1) 52800(열쇠 구멍 유턴)·49898(섬 동쪽 끝, 반경 2.94m) 지움 ----
info["removed"] = []
for OLD, where in (("52800", "후문 앞 열쇠 구멍 유턴(v3.3b, 반경 약 2.9m)"), ("49898", "섬 동쪽 끝 r1953(반경 2.94m)")):
    om = mem(OLD); drop_w = [om["left"], om["right"]]
    assert all(f'ref="{OLD}"' not in b for r, b in rel_body.items() if r != OLD), f"{OLD} 를 다른 relation 이 씀"
    assert all(f'ref="{w}"' not in b for r, b in rel_body.items() if r != OLD for w in drop_w), f"{OLD} 경계를 다른 relation 이 씀"
    used_elsewhere = {n for w, nd in ways.items() if w not in drop_w for n in nd}
    drop_n = sorted({n for w in drop_w for n in ways[w]} - used_elsewhere)
    text = re.sub(rf'  <relation id="{OLD}" visible="true" version="1">\n.*?  </relation>\n', "", text, flags=re.S)
    for w in drop_w:
        text = re.sub(rf'  <way id="{w}" visible="true" version="1">\n.*?  </way>\n', "", text, flags=re.S); del ways[w]
    del rel_body[OLD]
    dn = set(drop_n)
    text = re.sub(NODE_RE + r'\n  </node>\n', lambda m: "" if m.group(2) in dn else m.group(0), text)
    info["removed"].append(dict(relation=OLD, ways=drop_w, nodes=len(drop_n), where=where))


# ---- 2) 나누기 ----
def way_xml(w, refs): return f'  <way id="{w}" visible="true" version="1">\n' + "".join(f'    <nd ref="{r}" />\n' for r in refs) + "  </way>\n"


def set_way(w, refs):
    global text
    text = re.sub(rf'  <way id="{w}" visible="true" version="1">\n.*?  </way>\n', lambda m: way_xml(w, refs), text, count=1, flags=re.S)


new_ways, new_rels = [], []


def split(rid, k, keep_front):
    """rid 레인렛을 경계 점 k 에서 나눔. keep_front 면 앞 조각이 원래 id. 돌려줌: (앞 id, 뒤 id)"""
    global wid, rid_n
    mm = mem(rid); L, R = ways[mm["left"]], ways[mm["right"]]; assert len(L) == len(R) and 0 < k < len(L) - 1
    fL, bL, fR, bR = L[:k + 1], L[k:], R[:k + 1], R[k:]
    keepL, keepR, newL, newR = (fL, fR, bL, bR) if keep_front else (bL, bR, fL, fR)
    set_way(mm["left"], keepL); set_way(mm["right"], keepR)
    ways[mm["left"]], ways[mm["right"]] = keepL, keepR
    wl, wr = str(wid), str(wid + 1); wid += 2; new_ways.append((wl, newL)); new_ways.append((wr, newR)); ways[wl], ways[wr] = newL, newR
    body = rel_body[rid].replace(f'ref="{mm["left"]}" role="left"', f'ref="{wl}" role="left"').replace(f'ref="{mm["right"]}" role="right"', f'ref="{wr}" role="right"')
    body += f'    <tag k="s77_split_from" v="{rid}" />\n'
    nr = str(rid_n); rid_n += 1; new_rels.append((nr, body)); rel_body[nr] = body
    return (rid, nr) if keep_front else (nr, rid)


i, j0 = pick["i"], pick["j"]; j = j0 + a.ext
A_front, A_back = split("34923", i, keep_front=True)    # 34923 앞 조각 = 후문 안쪽 쪽
B_front, B_back = split("34884", j, keep_front=False)   # 34884 뒤 조각 = 북서행(49817 -> 34514 쪽)
info["split"] = dict(r1391=dict(src="34923", at=i, front=A_front, back=A_back), r1390=dict(src="34884", at=j, front=B_front, back=B_back))

# ---- 3) 유턴 레인렛 ----
LA, RA = ways[mem(A_front)["left"]], ways[mem(A_front)["right"]]; LB, RB = ways[mem(B_back)["left"]], ways[mem(B_back)["right"]]
pL0, pR0, pL1, pR1 = nodes[LA[-1]], nodes[RA[-1]], nodes[LB[0]], nodes[RB[0]]
w0 = math.dist(pL0, pR0); w1 = math.dist(pL1, pR1)
c0 = ((pL0[0] + pR0[0]) / 2, (pL0[1] + pR0[1]) / 2); c1 = ((pL1[0] + pR1[0]) / 2, (pL1[1] + pR1[1]) / 2)
# 늘림: r1390 중심선 j0+1 .. j 점(나누기 전 34884 의 중심선 = 앞 조각과 뒤 조각 앞부분)
LBf, RBf = ways[mem(B_front)["left"]], ways[mem(B_front)["right"]]
LBall, RBall = LBf + LB[1:], RBf + RB[1:]
cBall = [((nodes[p][0] + nodes[q][0]) / 2, (nodes[p][1] + nodes[q][1]) / 2) for p, q in zip(LBall, RBall)]
assert math.dist(cBall[j0], pts[-1][:2]) < 0.25 and math.dist(cBall[j], c1) < 0.01, (cBall[j0], pts[-1], cBall[j], c1)
for k_ in range(j0 + 1, j + 1):
    hh_ = math.atan2(cBall[k_][1] - cBall[k_ - 1][1], cBall[k_][0] - cBall[k_ - 1][0])
    pts.append([cBall[k_][0], cBall[k_][1], hh_])
assert math.dist(c0, pts[0][:2]) < 0.05 and math.dist(c1, pts[-1][:2]) < 0.05, (c0, pts[0], c1, pts[-1])
n_ = len(pts); cl = [(x, y) for x, y, _ in pts]; cl[-1] = c1


def offset(sign, wmax):
    out = []
    for k_, (x, y) in enumerate(cl):
        h = pts[k_][2]; f = k_ / (n_ - 1); wend = w0 / 2 * (1 - f) + w1 / 2 * f
        d = wend + (wmax - wend) * max(0.0, math.sin(math.pi * f)) ** a.wpow
        out.append((x - sign * d * math.sin(h), y + sign * d * math.cos(h)))
    return out


left, right = offset(1, a.wl), offset(-1, a.wr)
new_nodes = []; lref = [LA[-1]]; rref = [RA[-1]]
for seq, refs in ((left[1:-1], lref), (right[1:-1], rref)):
    for x, y in seq[::3]: new_nodes.append((str(nid), x, y)); refs.append(str(nid)); nid += 1
lref.append(LB[0]); rref.append(RB[0])
cref = []
for k_, (x, y, _) in enumerate(pts):
    if k_ % 2 == 0 or k_ == len(pts) - 1: new_nodes.append((str(nid), x, y)); cref.append(str(nid)); nid += 1
wl, wr = str(wid), str(wid + 1); wc_ = str(wid + 2); wid += 3; new_ways += [(wl, lref), (wr, rref), (wc_, cref)]
tA = dict(re.findall(r'<tag k="([^"]+)" v="([^"]+)" />', rel_body[A_front]))
tags = {k: v for k, v in tA.items() if not k.startswith("xodr_") and not k.startswith("s75_") and not k.startswith("s77_")}
tags.update({"turn_direction": "left", "s77_uturn": "back_gate_island",
             "s77_note": "field check 2026-11-04: space in front of back gate; turn around the island when no oncoming car. Radius for Sprinter (rear axle 4.1 m or more), inside CARLA road surface"})
ur = str(rid_n); rid_n += 1
new_rels.append((ur, f'    <member type="way" ref="{wl}" role="left" />\n    <member type="way" ref="{wr}" role="right" />\n    <member type="way" ref="{wc_}" role="centerline" />\n'
                     + "".join(f'    <tag k="{k}" v="{v}" />\n' for k, v in tags.items())))
nl = "".join(f'  <node id="{i_}" visible="true" version="1" lat="{ky * y:.11f}" lon="{kx * x:.11f}">\n    <tag k="ele" v="0.000000" />\n'
             f'    <tag k="local_x" v="{x:.6f}" />\n    <tag k="local_y" v="{y:.6f}" />\n  </node>\n' for i_, x, y in new_nodes)
wx = "".join(way_xml(w, r) for w, r in new_ways)
rx = "".join(f'  <relation id="{r}" visible="true" version="1">\n{b}  </relation>\n' for r, b in new_rels)
i_way = text.find("  <way "); i_rel = text.find("  <relation "); i_end = text.rfind("</osm>")
text = text[:i_way] + nl + text[i_way:i_rel] + wx + text[i_rel:i_end] + rx + text[i_end:]
(OUT / "lanelet2_map.osm").write_text(text); shutil.copy(SRC / "map_projector_info.yaml", OUT / "map_projector_info.yaml")
h = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
info["uturn"] = dict(relation=ur, where="후문 앞 섬 둘레", start=[round(v, 2) for v in c0], end=[round(v, 2) for v in c1], typ=pick["typ"], r=pick["r"],
                     segs_rad=pick["segs"], length_m=pick["L"], wl=a.wl, wr=a.wr, lane_w=[round(w0, 3), round(w1, 3)], nodes=len(new_nodes),
                     wheel_worst_out_m=pick["worst"], ext_pts=a.ext, centerline_pts=len(cref), wpow=a.wpow, length_with_ext_m=round(sum(math.dist(p[:2], q[:2]) for p, q in zip(pts, pts[1:])), 1), steer_need_rad=dict(prius_sample=round(math.atan(2.79 / pick["r"]), 3), sprinter=round(math.atan(3.661 / pick["r"]), 3)),
                     from_lanelet=A_front, to_lanelet=B_back)
info.update(source_sha256=h(SRC / "lanelet2_map.osm"), out_sha256=h(OUT / "lanelet2_map.osm"), path_json=str(Path(a.path).resolve().relative_to(C)) if Path(a.path).resolve().is_relative_to(C) else a.path)
(OUT / "v33d_changes.json").write_text(json.dumps(info, ensure_ascii=False, indent=1))
print(json.dumps(info["removed"], ensure_ascii=False)); print(json.dumps(info["split"]), json.dumps(info["uturn"], ensure_ascii=False)); print("out", info["out_sha256"][:16])
