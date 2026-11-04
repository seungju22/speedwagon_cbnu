#!/usr/bin/env python3
# 11/03: v3.3c = v3.3b(map/maps/v33b, bf0d421e) 에서 유턴 자리를 옮김. xodr·CARLA 월드는 v3.1 그대로, Lanelet2 만
#   11/03 오후: 실제로 도는 곳은 '도서관 지나 개신문화관이 정면에 보이는 곳, 좌회전만 되는 길에서 좌회전하는 척하다 차 없으면 유턴'
#   = 도서관 길이 남동 -> 북동으로 꺾이는 모서리('가', 약 722, 641). v3.3a·b 의 북문 길 끝 유턴(52799, '나', 750, 700)은 자리를 잘못 잡은 것
#   (s74 CARLA 에서 그 자리 도로면이 없어 8분 헤맴). 실제 포장 범위는 그림으로 못 고름 -> 지금 CARLA 도로면 안에서만 잡음
#   1) 52799 레인렛과 그 경계 way 둘, 그 경계에만 쓰인 점을 지움
#   2) r1415(37801, 동남행) 을 중심선 i 번째 점에서, r1409(36955, 북서행) 을 j 번째 점에서 둘로 나눔(앞 조각이 원래 id 를 지님 /
#      36955 는 뒤 조각(모서리 -> 서쪽)이 원래 id 를 지님: 도서관 북문 쪽 승강장·30쌍 레인렛 id 가 그대로이게)
#   3) 유턴 레인렛 하나: 중심선 = s75_uturn_search.py 가 고른 Dubins 경로(꼴·반경·i·j 는 --path json), 경계 = 중심선 좌우로 벌림
#      좌우 같게 최대 --wl·--wr(3.0, 다르게 하면 가운데선이 밀림), 양 끝은 차로 폭 절반(사인 꼴로 이어짐)
#      --ext: 유턴 끝을 r1409 중심선을 따라 곧게 --ext 점(약 1m 간격) 더 늘림(그만큼 r1409 를 뒤에서 나눔). 폭 꼴은 sin^--wpow(끝 가까이까지 넓게)
#      (플래닝 시뮬레이터 스프린터가 유턴 끝에서 앞 왼 모서리가 주행 영역 밖 0.35m 로 path_optimizer 가 세움 -> 고침, 11/03)
#      가운데선: 넓힌 경계의 가운데는 Dubins 경로와 어긋나서, 경로 점(약 0.5m)을 role=centerline way 로 레인렛에 넣음
#      (Autoware lanelet2_map_loader use_waypoints: true -> centerline 을 waypoints 로 씀, 1.9.0 autoware_launch config/map/lanelet2_map_loader.param.yaml)
#   4) N14 모서리(r1318, 레인렛 24245): s74 의 바깥 경계만 넓히기(가운데선이 CARLA 면 밖으로 밀림)를 되돌리고(v3.3a 점 자리),
#      가운데선을 가우스(--n14-sig 점)로 고른 뒤 꺾임 자리(경계 점 79번 둘레, 바퀴가 나가던 534~540, 940~944) 앞뒤 --n14-w m 에서
#      왼쪽(맞은편 차로 쪽)으로 최대 --n14-d0 m 비켜 감(코사인 제곱). 경계 = 새 가운데선 좌우 차로 폭 절반(+ --n14-e). 점 수·id 는 그대로, 자리만 옮김
#      값은 바퀴 4개가 CARLA 면 안에 들게 찾은 것(integration/logs/s75/n14_search2.json)
# 출력: map/maps/v33c/{lanelet2_map.osm, map_projector_info.yaml, v33c_changes.json}
# 사용: python3 map/scripts/s75_make_v33c.py [--path integration/logs/s75/uturn_search_path.json] [--wl 2.2] [--wr 3.3]
import argparse, hashlib, json, math, re, shutil
from pathlib import Path

C = Path(__file__).resolve().parents[2]
ap = argparse.ArgumentParser(); ap.add_argument("--src", default=str(C / "map/maps/v33b"))
ap.add_argument("--path", default=str(C / "integration/logs/s75/uturn_search_path.json"))
ap.add_argument("--wl", type=float, default=3.0); ap.add_argument("--wr", type=float, default=3.0)
ap.add_argument("--n14-sig", type=float, default=6.0); ap.add_argument("--n14-d0", type=float, default=2.5); ap.add_argument("--n14-w", type=float, default=20.0)
ap.add_argument("--ext", type=int, default=8); ap.add_argument("--wpow", type=float, default=0.4)
ap.add_argument("--n14-c0", type=float, default=-4.0); ap.add_argument("--n14-e", type=float, default=0.0)
ap.add_argument("--out", default=str(C / "map/maps/v33c")); a = ap.parse_args()
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

# ---- 1) 52799 지움 ----
OLD = "52799"; om = mem(OLD); drop_w = [om["left"], om["right"]]
used_elsewhere = {n for w, nd in ways.items() if w not in drop_w for n in nd}
drop_n = sorted({n for w in drop_w for n in ways[w]} - used_elsewhere)
assert all(f'ref="{OLD}"' not in b for r, b in rel_body.items() if r != OLD), "52799 를 다른 relation 이 씀"
text = re.sub(rf'  <relation id="{OLD}" visible="true" version="1">\n.*?  </relation>\n', "", text, flags=re.S)
for w in drop_w: text = re.sub(rf'  <way id="{w}" visible="true" version="1">\n.*?  </way>\n', "", text, flags=re.S)
dn = set(drop_n)
text = re.sub(NODE_RE + r'\n  </node>\n', lambda m: "" if m.group(2) in dn else m.group(0), text)
info["removed"] = dict(relation=OLD, ways=drop_w, nodes=len(drop_n), where="북문 길 끝(750, 695), v3.3a·b 유턴")


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
    body += f'    <tag k="s75_split_from" v="{rid}" />\n'
    nr = str(rid_n); rid_n += 1; new_rels.append((nr, body)); rel_body[nr] = body
    return (rid, nr) if keep_front else (nr, rid)


i, j0 = pick["i"], pick["j"]; j = j0 + a.ext
A_front, A_back = split("37801", i, keep_front=True)    # 37801 앞 조각 = 도서관 쪽(승강장 lib_north), 뒤 조각 = 모서리 -> 북동
B_front, B_back = split("36955", j, keep_front=False)   # 36955 뒤 조각 = 모서리 -> 서쪽(도서관 쪽), 앞 조각 = 북동 -> 모서리
info["split"] = dict(r1415=dict(src="37801", at=i, front=A_front, back=A_back), r1409=dict(src="36955", at=j, front=B_front, back=B_back))

# ---- 3) 유턴 레인렛 ----
LA, RA = ways[mem(A_front)["left"]], ways[mem(A_front)["right"]]; LB, RB = ways[mem(B_back)["left"]], ways[mem(B_back)["right"]]
pL0, pR0, pL1, pR1 = nodes[LA[-1]], nodes[RA[-1]], nodes[LB[0]], nodes[RB[0]]
w0 = math.dist(pL0, pR0); w1 = math.dist(pL1, pR1)
c0 = ((pL0[0] + pR0[0]) / 2, (pL0[1] + pR0[1]) / 2); c1 = ((pL1[0] + pR1[0]) / 2, (pL1[1] + pR1[1]) / 2)
# 늘림: r1409 중심선 j0+1 .. j 점(나누기 전 36955 의 중심선 = 뒤 조각 앞부분과 앞 조각 끝)
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
tags = {k: v for k, v in tA.items() if not k.startswith("xodr_") and not k.startswith("s75_")}
tags.update({"turn_direction": "left", "s75_uturn": "library_corner",
             "s75_note": "user field 2026-11-03: past library, facing Gaesin culture hall, left-only road; pretend left turn and U-turn when no car. Paved extent unknown -> inside CARLA road surface"})
ur = str(rid_n); rid_n += 1
new_rels.append((ur, f'    <member type="way" ref="{wl}" role="left" />\n    <member type="way" ref="{wr}" role="right" />\n    <member type="way" ref="{wc_}" role="centerline" />\n'
                     + "".join(f'    <tag k="{k}" v="{v}" />\n' for k, v in tags.items())))
nl = "".join(f'  <node id="{i_}" visible="true" version="1" lat="{ky * y:.11f}" lon="{kx * x:.11f}">\n    <tag k="ele" v="0.000000" />\n'
             f'    <tag k="local_x" v="{x:.6f}" />\n    <tag k="local_y" v="{y:.6f}" />\n  </node>\n' for i_, x, y in new_nodes)
wx = "".join(way_xml(w, r) for w, r in new_ways)
rx = "".join(f'  <relation id="{r}" visible="true" version="1">\n{b}  </relation>\n' for r, b in new_rels)
i_way = text.find("  <way "); i_rel = text.find("  <relation "); i_end = text.rfind("</osm>")
text = text[:i_way] + nl + text[i_way:i_rel] + wx + text[i_rel:i_end] + rx + text[i_end:]
# ---- 4) N14 모서리 ----
V33A = (C / "map/maps/v33a/lanelet2_map.osm").read_text()
na = {m.group(2): (float(m.group(7)), float(m.group(9))) for m in re.finditer(NODE_RE, V33A)}
m14 = mem("24245"); L14, R14 = ways[m14["left"]], ways[m14["right"]]; assert len(L14) == len(R14)
PL = [na[q] for q in L14]; PR = [na[q] for q in R14]
cl14 = [((p[0] + q[0]) / 2, (p[1] + q[1]) / 2) for p, q in zip(PL, PR)]; n14 = len(cl14)
hw = sorted(math.dist(p, q) for p, q in zip(PL, PR))[n14 // 2] / 2   # 차로 폭 절반(가운데값, 꺾임 자리 지그재그 빼고)
acc = [0.0]
for p, q in zip(cl14, cl14[1:]): acc.append(acc[-1] + math.dist(p, q))
def gs(i, sig):
    if sig <= 0: return cl14[i]
    k = int(3 * sig); lo, hi = max(0, i - k), min(n14, i + k + 1); ws = [math.exp(-((t - i) ** 2) / (2 * sig * sig)) for t in range(lo, hi)]; sw = sum(ws)
    return (sum(w_ * cl14[t][0] for w_, t in zip(ws, range(lo, hi))) / sw, sum(w_ * cl14[t][1] for w_, t in zip(ws, range(lo, hi))) / sw)
Q = [gs(i, a.n14_sig) for i in range(n14)]
J14 = 79; I0, I1 = 20, n14 - 20; moved14 = 0; newpos = {}
for i in range(I0, I1):
    hh = math.atan2(Q[min(n14 - 1, i + 1)][1] - Q[max(0, i - 1)][1], Q[min(n14 - 1, i + 1)][0] - Q[max(0, i - 1)][0])
    u = (acc[i] - acc[J14] - a.n14_c0) / a.n14_w; b = math.cos(math.pi / 2 * u) ** 2 if abs(u) < 1 else 0.0
    cx, cy = Q[i][0] - a.n14_d0 * b * math.sin(hh), Q[i][1] + a.n14_d0 * b * math.cos(hh); d = hw + a.n14_e * b
    newpos[L14[i]] = (cx - d * math.sin(hh), cy + d * math.cos(hh)); newpos[R14[i]] = (cx + d * math.sin(hh), cy - d * math.cos(hh))
for i in list(range(0, I0)) + list(range(I1, n14)):   # 양 끝 20점은 v3.3a 자리로(s74 가 옮긴 점 되돌림)
    newpos[L14[i]] = na[L14[i]]; newpos[R14[i]] = na[R14[i]]
assert all(sum(1 for w_, nd in ways.items() if q in nd) == 1 for q in newpos if q not in (L14[0], L14[-1], R14[0], R14[-1])), "N14 경계 점을 다른 way 도 씀"
for q in (L14[0], L14[-1], R14[0], R14[-1]): newpos[q] = nodes[q]   # 끝 점(이웃 레인렛과 같이 씀)은 그대로
def put14(m):
    q = m.group(2)
    if q not in newpos: return m.group(0)
    x, y = newpos[q]
    return f"{m.group(1)}{ky * y:.11f}{m.group(4)}{kx * x:.11f}{m.group(6)}{x:.6f}{m.group(8)}{y:.6f}{m.group(10)}"
text = re.sub(NODE_RE, put14, text)
info["n14"] = dict(lanelet="24245", road=1318, reverted_s74=True, sig=a.n14_sig, d0=a.n14_d0, w=a.n14_w, c0=a.n14_c0, e=a.n14_e, half_width=round(hw, 3),
                   jog_idx=J14, jog_xy=[round(v, 2) for v in cl14[J14]], moved_nodes=len(newpos), search="integration/logs/s75/n14_search2.json")
(OUT / "lanelet2_map.osm").write_text(text); shutil.copy(SRC / "map_projector_info.yaml", OUT / "map_projector_info.yaml")
h = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
info["uturn"] = dict(relation=ur, where="도서관 길 꺾이는 모서리('가')", start=[round(v, 2) for v in c0], end=[round(v, 2) for v in c1], typ=pick["typ"], r=pick["r"],
                     segs_rad=pick["segs"], length_m=pick["L"], wl=a.wl, wr=a.wr, lane_w=[round(w0, 3), round(w1, 3)], nodes=len(new_nodes),
                     wheel_worst_out_m=pick["worst"], ext_pts=a.ext, centerline_pts=len(cref), wpow=a.wpow, length_with_ext_m=round(sum(math.dist(p[:2], q[:2]) for p, q in zip(pts, pts[1:])), 1), steer_need_rad=dict(prius_sample=round(math.atan(2.79 / pick["r"]), 3), sprinter=round(math.atan(3.661 / pick["r"]), 3)),
                     from_lanelet=A_front, to_lanelet=B_back)
info.update(source_sha256=h(SRC / "lanelet2_map.osm"), out_sha256=h(OUT / "lanelet2_map.osm"), path_json=a.path)
(OUT / "v33c_changes.json").write_text(json.dumps(info, ensure_ascii=False, indent=1))
print(json.dumps(info["split"]), json.dumps(info["uturn"], ensure_ascii=False)); print(json.dumps(info["n14"], ensure_ascii=False)); print("out", info["out_sha256"][:16])
