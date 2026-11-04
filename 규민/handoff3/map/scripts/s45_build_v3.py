#!/usr/bin/env python3
# 10/09: v3 후보 s45 만들기. OSM 재변환 없이 s43 후보 텍스트에 덧붙이고, r1329 한 road 의 꺾임만 고친다
# 입력: map/maps/cbnu_campus_v3_candidate_s43.xodr (읽기만), OSM 원본 data/raw/cbnu_campus.osm (노드 좌표·태그만)
# 출력: map/maps/cbnu_campus_v3_candidate_s45.xodr, map/docs/logs/s45_build_v3.log
# 1) 북문 고리 길(10/09 [E] 5): j112(노드 4402717775) -> E20 442595044 -> 442595043(노드 4403782532~4402717774 구간)
#    -> 815864977 -> j110(노드 4400580300, 맵 도로 442361829 = r1312·r1325 의 끝)
#    양방향(oneway 태그 없음) -> 반쪽 도로 2개(3004 j112->j110, 3005 j110->j112). 기준선 = OSM 노드 꺾은선 + 모서리 원호(필렛)
#    차로는 기존 반쪽 도로와 같다(-1 주행 3.35, -2 보도 2.80, r1415 의 lanes 블록 그대로)
#    양끝: 기존 도로 r1415·1409·1312·1325 는 노드에서 끝난다(막다른 끝). 새 도로는 노드에서 GAP112·GAP110 떨어진 곳에서 시작·끝,
#    연결로 4개(3006 j112 r1415->3004, 3007 j112 3005->r1409, 3008 j110 3004->r1312, 3009 j110 r1325->3005)는
#    10/09 과 같은 3차 Hermite. 기존 junction 에는 connection 만 더한다. 기존 회차 r1703·r1622 그대로
# 2) r1329: s=16.77~60.10 의 line + 뾰족 paramPoly3 + paramPoly3 (3 요소)를 line-arc-line 으로 바꾼다
#    양끝(s=16.77, s=60.10) 위치·방위 그대로, 그 앞뒤 요소는 바이트 그대로. 길이 변화 R1329_DL(±0.1m 안)
# 사용: python3 map/scripts/s45_build_v3.py
import hashlib
import math
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from s22_buildings import Frame                    # noqa: E402
from s42_common import OSM_RAW, parse_osm          # noqa: E402

BASE = Path(__file__).resolve().parent.parent
SRC = BASE / "maps/cbnu_campus_v3_candidate_s43.xodr"
OUT = BASE / "maps/cbnu_campus_v3_candidate_s45.xodr"
LOG = BASE / "docs/logs/s45_build_v3.log"
F_ID, B_ID = 3004, 3005
CF112, CB112, CF110, CB110 = 3006, 3007, 3008, 3009
LOOP_WAYS = ["442595044", "442595043", "815864977"]
GAP112 = float(sys.argv[1]) if len(sys.argv) > 1 else 14.0    # 노드 4402717775 에서 새 도로 끝까지(m). 첫 구간 26.7m - 필렛 8.9m 안
GAP110 = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0    # 노드 4400580300 쪽. 마지막 구간 11.9m - 필렛 0.7m 안
R_BIG, R_SMALL = 8.0, 20.0     # 모서리 필렛 기준선 반경: 45도 넘는 꺾임 / 나머지
R1329_DL = -0.09               # r1329 길이 변화(m)

log = []


def P(*a):
    s = " ".join(str(x) for x in a)
    log.append(s)
    print(s)


text = SRC.read_text()
assert hashlib.sha256(text.encode()).hexdigest().startswith("dae3f058ed86f907")
road_re = re.compile(r'^ *<road name="[^"]*" length="[^"]*" id="(\d+)" junction="(-?\d+)">.*?</road>\n', re.S | re.M)
junc_re = re.compile(r'^ *<junction name="([^"]*)" id="(\d+)">.*?</junction>\n', re.S | re.M)
conn_re = re.compile(r'<connection id="(\d+)" incomingRoad="(\d+)" connectingRoad="(\d+)" contactPoint="[^"]*">.*?</connection>', re.S)
roads = {int(m.group(1)): m for m in road_re.finditer(text)}
juncs = {int(m.group(2)): m for m in junc_re.finditer(text)}
P(f"s43 후보 road {len(roads)}, junction {len(juncs)}")
GEO_RE = r'<geometry s="([\d.]+)" x="([-\d.]+)" y="([-\d.]+)" hdg="([-\d.]+)" length="([\d.]+)">\s*(<line/>|<arc [^>]*/>|<paramPoly3 [^>]*/>)\s*</geometry>'


def pose_on(kind, x, y, h, L, t):
    """요소 안 s=t 위치·방위"""
    if kind == "<line/>":
        return x + t * math.cos(h), y + t * math.sin(h), h
    if kind.startswith("<arc"):
        k = float(re.search(r'curvature="([-\d.e]+)"', kind).group(1))
        if abs(k) < 1e-12:
            return x + t * math.cos(h), y + t * math.sin(h), h
        return x + (math.sin(h + k * t) - math.sin(h)) / k, y - (math.cos(h + k * t) - math.cos(h)) / k, h + k * t
    pp = {k: float(v) for k, v in re.findall(r'(\w+)="([-\d.e]+)"', kind) if k != "pRange"}
    p = t / L
    u = pp["aU"] + pp["bU"] * p + pp["cU"] * p * p + pp["dU"] * p ** 3
    v = pp["aV"] + pp["bV"] * p + pp["cV"] * p * p + pp["dV"] * p ** 3
    du, dv = pp["bU"] + 2 * pp["cU"] * p + 3 * pp["dU"] * p * p, pp["bV"] + 2 * pp["cV"] * p + 3 * pp["dV"] * p * p
    return x + u * math.cos(h) - v * math.sin(h), y + u * math.sin(h) + v * math.cos(h), h + math.atan2(dv, du)


def geoms(blk):
    return [(float(s), float(x), float(y), float(h), float(L), k) for s, x, y, h, L, k in re.findall(GEO_RE, blk)]


def end_pose(blk):
    s, x, y, h, L, k = geoms(blk)[-1]
    return pose_on(k, x, y, h, L, L)


def start_pose(blk):
    s, x, y, h, L, k = geoms(blk)[0]
    return x, y, h


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def hermite(p0, p1):
    """10/09 과 같은 3차 Hermite(paramPoly3 normalized). 계수, 호 길이, 기준선 최소 반경"""
    x0, y0, h0 = p0
    x1, y1, h1 = p1
    dx, dy = x1 - x0, y1 - y0
    ex, ey = dx * math.cos(h0) + dy * math.sin(h0), -dx * math.sin(h0) + dy * math.cos(h0)
    T = math.hypot(dx, dy)
    t0u, t0v = T, 0.0
    t1u, t1v = T * math.cos(h1 - h0), T * math.sin(h1 - h0)
    c = dict(aU=0.0, bU=t0u, cU=3 * ex - 2 * t0u - t1u, dU=-2 * ex + t0u + t1u,
             aV=0.0, bV=t0v, cV=3 * ey - 2 * t0v - t1v, dV=-2 * ey + t0v + t1v)
    L, prev = 0.0, (0.0, 0.0)
    for i in range(1, 4001):
        p = i / 4000
        u = c["bU"] * p + c["cU"] * p * p + c["dU"] * p ** 3
        v = c["bV"] * p + c["cV"] * p * p + c["dV"] * p ** 3
        L += math.dist(prev, (u, v))
        prev = (u, v)
    kmax = 0.0
    for i in range(0, 1001):
        p = i / 1000
        du = c["bU"] + 2 * c["cU"] * p + 3 * c["dU"] * p * p
        dv = c["bV"] + 2 * c["cV"] * p + 3 * c["dV"] * p * p
        ddu, ddv = 2 * c["cU"] + 6 * c["dU"] * p, 2 * c["cV"] + 6 * c["dV"] * p
        kmax = max(kmax, abs(du * ddv - dv * ddu) / math.hypot(du, dv) ** 3)
    return c, L, (1 / kmax if kmax else math.inf)


def pp_xml(c):
    return ('<paramPoly3 ' + " ".join(f'{k}="{c[k]:.8f}"' for k in ("aU", "bU", "cU", "dU", "aV", "bV", "cV", "dV"))
            + ' pRange="normalized"/>')


def geom_xml(s, x, y, h, L, kind):
    return (f'            <geometry s="{s:.8f}" x="{x:.8f}" y="{y:.8f}" hdg="{h:.8f}" length="{L:.8f}">\n'
            f'                {kind}\n            </geometry>\n')


LANES_NORMAL = re.search(r'        <lanes>.*?</lanes>\n', roads[1415].group(0), re.S).group(0)
LANES_CONN = re.search(r'        <lanes>.*?</lanes>\n', roads[1969].group(0), re.S).group(0)


def fillet_path(pts):
    """꺾은선 -> line/arc 요소 목록 [(x, y, h, L, kind)]. 모서리마다 원호, 반경은 꺾임 45도 초과 R_BIG, 나머지 R_SMALL"""
    hs = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(pts, pts[1:])]
    cut = [0.0] * len(pts)          # 꼭짓점에서 양쪽으로 잘라 내는 길이
    arcs = {}
    for i in range(1, len(pts) - 1):
        d = wrap(hs[i] - hs[i - 1])
        R = R_BIG if abs(d) > math.radians(45) else R_SMALL
        t = R * math.tan(abs(d) / 2)
        cut[i] = t
        arcs[i] = (R, d)
    out = []
    cur = pts[0]
    for i in range(len(pts) - 1):
        a, b = pts[i], pts[i + 1]
        seg = math.dist(a, b)
        ln = seg - (cut[i] if i > 0 else 0.0) - (cut[i + 1] if i + 1 < len(pts) - 1 else 0.0)
        assert ln > 0.5, ("모서리 원호가 겹침", i, seg, ln)
        out.append((cur[0], cur[1], hs[i], ln, "<line/>"))
        cur = (cur[0] + ln * math.cos(hs[i]), cur[1] + ln * math.sin(hs[i]))
        if i + 1 < len(pts) - 1:
            R, d = arcs[i + 1]
            k = math.copysign(1 / R, d)
            out.append((cur[0], cur[1], hs[i], R * abs(d), f'<arc curvature="{k:.8f}"/>'))
            x, y, h = pose_on(f'<arc curvature="{k}"/>', cur[0], cur[1], hs[i], R * abs(d), R * abs(d))
            cur = (x, y)
    return out, arcs


def road_xml(rid, pred_j, succ_j, els):
    s, body = 0.0, ""
    for x, y, h, L, kind in els:
        body += geom_xml(s, x, y, h, L, kind)
        s += L
    return (f'    <road name="" length="{s:.8f}" id="{rid}" junction="-1">\n'
            f'        <link>\n            <predecessor elementType="junction" elementId="{pred_j}"/>\n'
            f'            <successor elementType="junction" elementId="{succ_j}"/>\n        </link>\n'
            f'        <type s="0" type="town"/>\n        <planView>\n' + body
            + '        </planView>\n        <elevationProfile>\n            <elevation s="0" a="0.00" b="0" c="0" d="0"/>\n'
            '        </elevationProfile>\n        <lateralProfile/>\n' + LANES_NORMAL + '        <objects/>\n    </road>\n'), s


def conn_xml(rid, name, jid, pred, succ, p0, c, L):
    return (f'    <road name="{name}" length="{L:.8f}" id="{rid}" junction="{jid}">\n'
            f'    <link>\n        <predecessor elementType="road" elementId="{pred}" contactPoint="end"/>\n'
            f'        <successor elementType="road" elementId="{succ}" contactPoint="start"/>\n    </link>\n'
            f'    <type s="0" type="town"/>\n    <planView>\n'
            f'        <geometry s="0.00000000" x="{p0[0]:.8f}" y="{p0[1]:.8f}" hdg="{p0[2]:.8f}" length="{L:.8f}">\n'
            f'            {pp_xml(c)}\n        </geometry>\n    </planView>\n'
            '        <elevationProfile>\n            <elevation s="0" a="0.00" b="0" c="0" d="0"/>\n        </elevationProfile>\n'
            '        <lateralProfile/>\n' + LANES_CONN + '        <objects/>\n        <signals>\n        </signals>\n</road>\n')


def add_conn(blk, inc, con):
    ids = [int(c.group(1)) for c in conn_re.finditer(blk)]
    nid = max(ids) + 1
    add = (f'        <connection id="{nid}" incomingRoad="{inc}" connectingRoad="{con}" contactPoint="start">\n'
           f'            <laneLink from="-1" to="-1"/>\n            <laneLink from="-2" to="-2"/>\n        </connection>\n')
    k = blk.rindex("    </junction>")
    return blk[:k] + add + blk[k:], nid


# ---- 1) 북문 고리 길
rn, rnt, rw = parse_osm(OSM_RAW)
for w in LOOP_WAYS + ["442361829"]:
    t = rw[w][1]
    P(f"OSM way {w}: 태그 {t}, 노드 {len(rw[w][0])}")
    assert t.get("highway") == "service" and "oneway" not in t and "access" not in t and "motor_vehicle" not in t
fr = Frame(text)
w43, w977 = rw["442595043"][0], rw["815864977"][0]
assert rw["442595044"][0] == ["4403782532", "4402717775"] and w977[-1] == "4400580300" and w43[-1] == w977[0]
chain = ["4402717775", "4403782532"] + w43[w43.index("4403782532") + 1:] + w977[1:]
P(f"고리 노드 {chain}")
P(f"  442595043 의 노드 4402717772~4403782532 구간({math.dist(fr.xodr(*rn['4402717772']), fr.xodr(*rn['4403782532'])):.1f}m)은 넣지 않음(고리 밖, 끝 노드는 맵 junction 아님)")
pts = [fr.xodr(*rn[n]) for n in chain]
j112p, j110p = pts[0], pts[-1]
P(f"  노드 4402717775 xodr ({j112p[0]:.3f}, {j112p[1]:.3f}) / r1409 시작 {tuple(round(v, 3) for v in start_pose(roads[1409].group(0))[:2])} / r1415 끝 {tuple(round(v, 3) for v in end_pose(roads[1415].group(0))[:2])}")
P(f"  노드 4400580300 xodr ({j110p[0]:.3f}, {j110p[1]:.3f}) / r1312 시작 {tuple(round(v, 3) for v in start_pose(roads[1312].group(0))[:2])} / r1325 끝 {tuple(round(v, 3) for v in end_pose(roads[1325].group(0))[:2])}")
# 노드 대신 기존 도로 끝점을 쓴다(맵 노드 = 변환 입력의 평활화 뒤 좌표)
p_a = start_pose(roads[1409].group(0))[:2]
p_b = start_pose(roads[1312].group(0))[:2]
pts[0], pts[-1] = p_a, p_b


def trim(pl, d, from_start):
    """꺾은선 끝에서 d 만큼 잘라 낸 점"""
    a, b = (pl[0], pl[1]) if from_start else (pl[-1], pl[-2])
    L = math.dist(a, b)
    assert d < L
    q = (a[0] + (b[0] - a[0]) * d / L, a[1] + (b[1] - a[1]) * d / L)
    return [q] + pl[1:] if from_start else pl[:-1] + [q]


fpts = trim(trim(pts, GAP112, True), GAP110, False)
P(f"  새 도로 끝 간격: j112 쪽 {GAP112}m, j110 쪽 {GAP110}m")
f_els, arcs = fillet_path(fpts)
for i, (R, d) in arcs.items():
    P(f"  모서리 {chain[i]}: 꺾임 {math.degrees(d):+.1f}도, 기준선 반경 {R}m, 차선 중심 반경 {R + (1.675 if d > 0 else -1.675):.3f}m(3004 기준)")
f_xml, f_len = road_xml(F_ID, 112, 110, f_els)
b_els, _ = fillet_path(fpts[::-1])
b_xml, b_len = road_xml(B_ID, 110, 112, b_els)
P(f"3004(j112->j110) 길이 {f_len:.3f}m, 3005(j110->j112) {b_len:.3f}m, 요소 {len(f_els)}, 노드 꺾은선 길이 {sum(math.dist(a, b) for a, b in zip(pts, pts[1:])):.3f}m")
f_start = (f_els[0][0], f_els[0][1], f_els[0][2])
fe = f_els[-1]
f_end = pose_on(fe[4], fe[0], fe[1], fe[2], fe[3], fe[3])
b_start = (b_els[0][0], b_els[0][1], b_els[0][2])
be = b_els[-1]
b_end = pose_on(be[4], be[0], be[1], be[2], be[3], be[3])
e1415, s1409 = end_pose(roads[1415].group(0)), start_pose(roads[1409].group(0))
e1325, s1312 = end_pose(roads[1325].group(0)), start_pose(roads[1312].group(0))
conns = [(CF112, ":4402717775_s45a", 112, 1415, F_ID, e1415, f_start),
         (CB112, ":4402717775_s45b", 112, B_ID, 1409, b_end, s1409),
         (CF110, ":4400580300_s45a", 110, F_ID, 1312, f_end, s1312),
         (CB110, ":4400580300_s45b", 110, 1325, B_ID, e1325, b_start)]
c_xml = ""
for rid, name, jid, pred, succ, p0, p1 in conns:
    c, L, R = hermite(p0, p1)
    turn = math.degrees(wrap(p1[2] - p0[2]))
    P(f"연결로 {rid}(j{jid}) r{pred}->r{succ}: 방위 변화 {turn:+.1f}도, 길이 {L:.3f}m, 기준선 최소 반경 {R:.2f}m")
    c_xml += conn_xml(rid, name, jid, pred, succ, p0, c, L)
new_junc = {}
for jid, inc, con in ((112, 1415, CF112), (112, B_ID, CB112), (110, F_ID, CF110), (110, 1325, CB110)):
    base = new_junc.get(jid, juncs[jid].group(0))
    nb, nid = add_conn(base, inc, con)
    new_junc[jid] = nb
    P(f"junction {jid}: connection {nid} 추가 (incoming {inc} -> connecting {con})")

# ---- 2) r1329 꺾임
blk = roads[1329].group(0)
gs = geoms(blk)
i0 = [k for k, g in enumerate(gs) if abs(g[0] - 16.77092075) < 1e-6][0]
i1 = [k for k, g in enumerate(gs) if abs(g[0] - 60.10273423) < 1e-6][0]
A = (gs[i0][1], gs[i0][2], gs[i0][3])
B = (gs[i1][1], gs[i1][2], gs[i1][3])
old_seg = gs[i1][0] - gs[i0][0]
Lt = old_seg + R1329_DL
D = wrap(B[2] - A[2])
dx, dy = B[0] - A[0], B[1] - A[1]
u = dx * math.cos(A[2]) + dy * math.sin(A[2])
v = -dx * math.sin(A[2]) + dy * math.cos(A[2])
sD, cD = math.sin(abs(D)), math.cos(abs(D))
sg = 1 if D > 0 else -1
v = v * sg
# u = L1 + R sinD + L2 cosD ; v = R(1-cosD) + L2 sinD ; L1 + R D + L2 = Lt
# L2 = (v - R(1-cD))/sD ; L1 = u - R sD - L2 cD ; u - R sD - L2 cD + R|D| + L2 = Lt
a2 = (1 - cD) / sD
R = (Lt - u - v / sD * (1 - cD)) / (abs(D) - sD - (-a2) * (1 - cD) + a2 * 0) if False else None
# 풀이: L2 = v/sD - R a2 ; L1 + L2 = u - R sD + L2 (1 - cD)
#  Lt = u - R sD + (v/sD - R a2)(1 - cD) + R|D|  ->  R (|D| - sD - a2 (1 - cD)) = Lt - u - v (1 - cD)/sD
R = (Lt - u - v * (1 - cD) / sD) / (abs(D) - sD - a2 * (1 - cD))
L2 = v / sD - R * a2
L1 = u - R * sD - L2 * cD
P(f"r1329: 바꾸는 구간 s={gs[i0][0]:.3f}~{gs[i1][0]:.3f}({old_seg:.3f}m, 요소 {i1 - i0}개), 방위 변화 {math.degrees(D):+.1f}도")
P(f"  line {L1:.3f} + arc 반경 {R:.3f}({R * abs(D):.3f}m) + line {L2:.3f} = {L1 + R * abs(D) + L2:.3f}m (변화 {R1329_DL:+.3f})")
assert L1 > 0 and L2 > 0 and R > 0
k = sg / R
x1, y1 = A[0] + L1 * math.cos(A[2]), A[1] + L1 * math.sin(A[2])
x2, y2, h2 = pose_on(f'<arc curvature="{k}"/>', x1, y1, A[2], R * abs(D), R * abs(D))
x3, y3, h3 = x2 + L2 * math.cos(h2), y2 + L2 * math.sin(h2), h2
P(f"  끝 맞춤: 위치 차 {math.dist((x3, y3), B[:2]):.2e}m, 방위 차 {math.degrees(wrap(h3 - B[2])):.2e}도")
new_els = [(A[0], A[1], A[2], L1, "<line/>"), (x1, y1, A[2], R * abs(D), f'<arc curvature="{k:.8f}"/>'), (x2, y2, h2, L2, "<line/>")]
els = [(g[1], g[2], g[3], g[4], g[5]) for g in gs[:i0]] + new_els + [(g[1], g[2], g[3], g[4], g[5]) for g in gs[i1:]]
s, body = 0.0, ""
for x, y, h, L, kind in els:
    body += geom_xml(s, x, y, h, L, kind)
    s += L
old_len = float(re.search(r'length="([\d.]+)" id="1329"', blk).group(1))
pv0, pv1 = blk.index("        <planView>\n") + len("        <planView>\n"), blk.index("        </planView>")
nb = blk[:pv0] + body + blk[pv1:]
nb = nb.replace(f'length="{old_len:.8f}" id="1329"', f'length="{s:.8f}" id="1329"', 1)
assert nb != blk and f'length="{s:.8f}"' in nb
e_old, e_new = end_pose(blk), end_pose(nb)
P(f"  r1329 길이 {old_len:.3f} -> {s:.3f}m ({s - old_len:+.3f}), 끝 위치 차 {math.dist(e_old[:2], e_new[:2]):.2e}m, 끝 방위 차 {math.degrees(wrap(e_new[2] - e_old[2])):.2e}도")
P(f"  r1329 lanes·link 블록은 그대로(planView 와 length 만 바뀜): {blk[pv1:] == nb[nb.index('        </planView>'):]}")
road_edit = {1329: nb}

# ---- 조립(s43 와 같은 방식)
out, pos = [], 0
spans = sorted([(m.start(), m.end(), "road", rid) for rid, m in roads.items()] + [(m.start(), m.end(), "junc", j) for j, m in juncs.items()])
last_road_end = max(m.end() for m in roads.values())
for a, b, kind, i in spans:
    out.append(text[pos:a])
    out.append((road_edit if kind == "road" else new_junc).get(i, text[a:b]))
    if b == last_road_end:
        out.append(f_xml + b_xml + c_xml)
    pos = b
out.append(text[pos:])
v3 = "".join(out)
OUT.write_text(v3)
sha = hashlib.sha256(v3.encode()).hexdigest()
P(f"출력 {OUT.name}: road {len(re.findall(r'<road name=', v3))}, junction {len(re.findall(r'<junction name=', v3))}, {len(v3.encode())} B, sha256 {sha}")
LOG.write_text("\n".join(log) + "\n")
