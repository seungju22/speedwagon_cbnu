#!/usr/bin/env python3
# 11/03: v3.3b = v3.3a(map/maps/v33a, af536a84) + 두 가지. xodr·CARLA 월드는 v3.1 그대로, Lanelet2 만
#   1) N14 주차장 길 r1318 ㄷ자 서북쪽 모서리: 바깥(오른쪽) 경계 점을 모서리 둘레 반경 --wr m 안에서만 최대 --wd m 바깥으로(코사인 제곱으로 줄임)
#      근거: 바로 바깥에 OSM 주차장 면 471401019·471401020·471401021 이 3.5~4.3m 거리(포장된 면). 11/03 '보통 N14 통로로 돌아 나옴'
#      11/02 차체 검사에서 지금 차 1.35m·스타렉스 1.61m 가 이 바깥으로 나감. 10/12 p_north_middle 이 r1318 s=75.7 에서 'outside drivable area' 로 선 자리와 같은 모퉁이인지 기록에 적음
#   2) 후문 앞 유턴 레인렛: r1251(동남행) 마지막 레인렛 끝 -> r1384(북서행) 첫 레인렛 처음. 북문 것과 같은 열쇠 구멍(s73_make_v33.py 와 같은 식)
#      근거: 11/03 '후문 앞 공간 있음, 차 안 올 때 돌림'(맞은편 차를 보고 기다리는 것은 판단 쪽 몫)
# 출력: map/maps/v33b/{lanelet2_map.osm, map_projector_info.yaml, v33b_changes.json}
# 사용: python3 map/scripts/s74_make_v33b.py [--wd 2.0] [--wr 12] [--out 폴더]
import argparse, hashlib, json, math, re, shutil
from pathlib import Path

C = Path(__file__).resolve().parents[2]
SRC = C / "map/maps/v33a"
ap = argparse.ArgumentParser(); ap.add_argument("--wd", type=float, default=2.0); ap.add_argument("--wr", type=float, default=12.0)
ap.add_argument("--r-u", type=float, default=5.0); ap.add_argument("--r-out", type=float, default=5.0); ap.add_argument("--uw", type=float, default=4.0)
ap.add_argument("--out", default=str(C / "map/maps/v33b")); a = ap.parse_args(); OUT = Path(a.out); OUT.mkdir(parents=True, exist_ok=True)
meta = json.loads((C / "map/build_s57_regen/build/l1_meta.json").read_text())
text = (SRC / "lanelet2_map.osm").read_text()
NODE_RE = r'(<node id="(\d+)" visible="true" version="1" lat=")([-\d.]+)(" lon=")([-\d.]+)(">\s*<tag k="ele" v="[-\d.]+" />\s*<tag k="local_x" v=")([-\d.]+)(" />\s*<tag k="local_y" v=")([-\d.]+)(" />)'
nodes = {m.group(2): (float(m.group(7)), float(m.group(9))) for m in re.finditer(NODE_RE, text)}
ll_ = {m.group(2): (float(m.group(3)), float(m.group(5))) for m in re.finditer(NODE_RE, text)}
ks = list(nodes)[:2000]
ky = sum(ll_[k][0] * nodes[k][1] for k in ks) / sum(nodes[k][1] ** 2 for k in ks)
kx = sum(ll_[k][1] * nodes[k][0] for k in ks) / sum(nodes[k][0] ** 2 for k in ks)
ways = {m.group(1): re.findall(r'<nd ref="(\d+)" />', m.group(2)) for m in re.finditer(r'<way id="(\d+)"[^>]*>(.*?)</way>', text, re.S)}
rels = {}
for m in re.finditer(r'<relation id="(\d+)"[^>]*>(.*?)</relation>', text, re.S):
    tags = dict(re.findall(r'<tag k="([^"]+)" v="([^"]+)" />', m.group(2)))
    if tags.get("type") == "lanelet":
        rels[m.group(1)] = (tags, dict((role, ref) for ref, role in re.findall(r'<member type="way" ref="(\d+)" role="(\w+)" />', m.group(2))))
ref_count = {}
for w, nds in ways.items():
    for n in nds: ref_count[n] = ref_count.get(n, 0) + 1

# ---- 1) N14 모서리 넓히기 ----
rid1318 = str(meta["lanelet"]["1318,0,-1"]); t, mem = rels[rid1318]
L, R = ways[mem["left"]], ways[mem["right"]]
APEX = (537.8, 940.7)   # 11/02 차체 검사가 걸린 점들 가운데(지금 차·스타렉스)
moved = []
for i, n in enumerate(R):
    if ref_count.get(n, 0) > 1: continue   # 다른 레인렛과 같이 쓰는 점(끝 점)은 안 옮김
    x, y = nodes[n]; d = math.dist((x, y), APEX)
    if d >= a.wr: continue
    # 바깥 방향 = 왼 경계 가장 가까운 점 -> 오른 경계 점
    lx, ly = min((nodes[q] for q in L), key=lambda p: math.dist(p, (x, y)))
    ux, uy = x - lx, y - ly; s = math.hypot(ux, uy); ux, uy = ux / s, uy / s
    k = a.wd * math.cos(math.pi / 2 * d / a.wr) ** 2
    nodes[n] = (x + ux * k, y + uy * k); moved.append((n, round(k, 3)))
def put(m):
    n = m.group(2)
    if n not in dict(moved): return m.group(0)
    x, y = nodes[n]
    return f"{m.group(1)}{ky * y:.11f}{m.group(4)}{kx * x:.11f}{m.group(6)}{x:.6f}{m.group(8)}{y:.6f}{m.group(10)}"
text = re.sub(NODE_RE, put, text)
print("N14 모서리: 옮긴 점", len(moved), "최대", max(k for _, k in moved))


# ---- 2) 후문 앞 유턴(s73_make_v33.py 와 같은 열쇠 구멍) ----
def keyhole(alpha, Ru, Ro, step=0.25):
    pts = [(0.0, 0.0)]; x = y = h = 0.0
    def arc(R_, ang, left):
        nonlocal x, y, h
        n = max(2, int(abs(ang) * R_ / step)); sgn = 1 if left else -1
        for _ in range(n):
            cx = x - R_ * math.sin(h) * sgn; cy = y + R_ * math.cos(h) * sgn; h2 = h + ang / n * sgn
            x = cx + R_ * math.sin(h2) * sgn; y = cy - R_ * math.cos(h2) * sgn; h = h2; pts.append((x, y))
    arc(Ro, alpha, False); arc(Ru, math.pi + 2 * alpha, True); arc(Ro, alpha, False)
    return pts, x, y, h


ra, rb = 1251, 1384
tA, mA = rels[str(meta["lanelet_last"][f"{ra},0,-1"])]; tB, mB = rels[str(meta["lanelet"][f"{rb},0,-1"])]
LA, RA = ways[mA["left"]], ways[mA["right"]]; LB, RB = ways[mB["left"]], ways[mB["right"]]
pL0, pR0, pL1, pR1 = nodes[LA[-1]], nodes[RA[-1]], nodes[LB[0]], nodes[RB[0]]
c0 = ((pL0[0] + pR0[0]) / 2, (pL0[1] + pR0[1]) / 2); c1 = ((pL1[0] + pR1[0]) / 2, (pL1[1] + pR1[1]) / 2)
qa, qb = nodes[LA[-2]], nodes[RA[-2]]
h0 = math.atan2(c0[1] - (qa[1] + qb[1]) / 2, c0[0] - (qa[0] + qb[0]) / 2); w = math.dist(pL0, pR0)
dx, dy = c1[0] - c0[0], c1[1] - c0[1]
lat_d = -dx * math.sin(h0) + dy * math.cos(h0); lon_d = dx * math.cos(h0) + dy * math.sin(h0)
lo, hi = 0.0, math.pi / 2
for _ in range(60):
    mid = (lo + hi) / 2; _, ex, ey, _ = keyhole(mid, a.r_u, a.r_out)
    lo, hi = (mid, hi) if ey > lat_d else (lo, mid)
alpha = (lo + hi) / 2; pts, ex, ey, eh = keyhole(alpha, a.r_u, a.r_out)
shift = lon_d - ex
if shift > 0:
    n = int(shift / 0.25) + 1; pts = [(shift * i / n, 0.0) for i in range(n)] + [(px + shift, py) for px, py in pts]
elif shift < 0:
    n = int(-shift / 0.25) + 1; pts = pts + [(ex - (-shift) * i / n, ey) for i in range(1, n + 1)]
cl = [(c0[0] + px * math.cos(h0) - py * math.sin(h0), c0[1] + px * math.sin(h0) + py * math.cos(h0)) for px, py in pts]
def offset(d):
    out = []; n_ = len(cl)
    for i, p in enumerate(cl):
        a0 = cl[max(0, i - 1)]; b0 = cl[min(n_ - 1, i + 1)]; hh = math.atan2(b0[1] - a0[1], b0[0] - a0[0])
        dd = d * (1 + (a.uw / w - 1) * math.sin(math.pi * i / (n_ - 1)))
        out.append((p[0] - dd * math.sin(hh), p[1] + dd * math.cos(hh)))
    return out
left, right = offset(w / 2), offset(-w / 2)
nid = max(int(x) for x in re.findall(r'<node id="(\d+)"', text)) + 1
wid = max(int(x) for x in re.findall(r'<way id="(\d+)"', text)) + 1
rid = max(int(x) for x in re.findall(r'<relation id="(\d+)"', text)) + 1
new_nodes = []; lref = [LA[-1]]; rref = [RA[-1]]
for seq, refs in ((left[1:-1], lref), (right[1:-1], rref)):
    for x, y in seq[::4]: new_nodes.append((nid, x, y)); refs.append(str(nid)); nid += 1
lref.append(LB[0]); rref.append(RB[0])
nl = "".join(f'  <node id="{i}" visible="true" version="1" lat="{ky * y:.11f}" lon="{kx * x:.11f}">\n    <tag k="ele" v="0.000000" />\n'
             f'    <tag k="local_x" v="{x:.6f}" />\n    <tag k="local_y" v="{y:.6f}" />\n  </node>\n' for i, x, y in new_nodes)
wl = "".join(f'  <way id="{w_}" visible="true" version="1">\n' + "".join(f'    <nd ref="{r}" />\n' for r in refs) + "  </way>\n" for w_, refs in ((wid, lref), (wid + 1, rref)))
tags = {k: v for k, v in tA.items() if k not in ("xodr_road", "xodr_section", "xodr_lane")}
tags.update({"turn_direction": "left", "s73_uturn": "back_gate_front", "s74_note": "user field: space in front of back gate, turn when no oncoming car (2026-11-03)"})
rl = (f'  <relation id="{rid}" visible="true" version="1">\n    <member type="way" ref="{wid}" role="left" />\n    <member type="way" ref="{wid + 1}" role="right" />\n'
      + "".join(f'    <tag k="{k}" v="{v}" />\n' for k, v in tags.items()) + "  </relation>\n")
i_way = text.find("  <way "); i_rel = text.find("  <relation "); i_end = text.rfind("</osm>")
text = text[:i_way] + nl + text[i_way:i_rel] + wl + text[i_rel:i_end] + rl + text[i_end:]
(OUT / "lanelet2_map.osm").write_text(text); shutil.copy(SRC / "map_projector_info.yaml", OUT / "map_projector_info.yaml")
ys = [p[1] for p in pts]; xs = [p[0] for p in pts]
h = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
info = dict(n14=dict(apex=APEX, wd=a.wd, wr=a.wr, moved=len(moved), max_m=max(k for _, k in moved), parking_ways=["471401019", "471401020", "471401021"]),
            back_uturn=dict(relation=rid, from_road=ra, to_road=rb, r_u=a.r_u, r_out=a.r_out, uw=a.uw, alpha_deg=round(math.degrees(alpha), 2), lane_w=round(w, 3),
                            lat_gap=round(lat_d, 3), lon_gap=round(lon_d, 3), forward_max=round(max(xs), 2), right_max=round(-min(ys), 2), left_max=round(max(ys), 2)),
            source_sha256=h(SRC / "lanelet2_map.osm"), out_sha256=h(OUT / "lanelet2_map.osm"))
(OUT / "v33b_changes.json").write_text(json.dumps(info, ensure_ascii=False, indent=1))
print(json.dumps(info["back_uturn"], ensure_ascii=False)); print("out", info["out_sha256"][:16])
