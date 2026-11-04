#!/usr/bin/env python3
# 10/09: v3 후보 xodr 만들기. OSM 재변환 없이 v2 동결본 텍스트에 덧붙이고 뺀다(10/07 회차 후처리와 같은 방식)
# 입력: map/maps/cbnu_campus_frozen_v2.xodr (읽기만), A안 map/maps/cbnu_internal_only_localtm_tags82_A_smooth.xodr (060 형상만 읽음)
# 출력: map/maps/cbnu_campus_v3_candidate_s43.xodr, map/docs/logs/s43_build_v3.log
# 1) 지우기: 검토 시트 삭제 7 way 의 road 14개(s42_way_dir.csv, way 위 92~100% 확인)
#    + 그 road 를 앞·뒤로 가리키는 교차로 연결로 + 그 연결로를 쓰는 junction connection
#    connection 이 하나도 안 남은 junction 은 지우고, 그 junction 을 가리키는 남은 road 의 link 줄만 지운다
# 2) 정문 출구: 새 일반 road X(3001) = A안 060 도로(r1392) planView 의 s 2.0~33.395 구간(형상 그대로, s 만 당김)
#    j109(노드 4748296088)에 연결로 C1(3002): v2 r1255 끝 -> X 시작
#    j1(노드 4748296080)에 연결로 C2(3003): X 끝 -> v2 r1150 시작
#    C1·C2 형상 = 양끝 위치·방위를 맞춘 3차 Hermite(paramPoly3, normalized). 차로는 양끝과 같은 1차로(-1 주행 3.35, -2 보도 2.80)
#    기존 road·연결로·connection 은 그대로 둔다(바이트 비교로 확인)
# 사용: python3 map/scripts/s43_build_v3.py
import csv
import hashlib
import math
import re
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
V2 = BASE / "maps/cbnu_campus_frozen_v2.xodr"
AAN = BASE / "maps/cbnu_internal_only_localtm_tags82_A_smooth.xodr"
OUT = BASE / "maps/cbnu_campus_v3_candidate_s43.xodr"
LOG = BASE / "docs/logs/s43_build_v3.log"
DEL_WAYS = {"E24": "442361807", "B34": "815864978", "B37": "471400900", "D30": "472252990",
            "E27": "442595852", "E36": "468616606", "I01": "468616610"}
X_ID, C1_ID, C2_ID = 3001, 3002, 3003
X_S0, X_S1 = 2.0, 33.39482487            # A안 r1392 에서 쓰는 구간(s)

log = []


def P(*a):
    s = " ".join(str(x) for x in a)
    log.append(s)
    print(s)


text = V2.read_text()
assert hashlib.sha256(text.encode()).hexdigest().startswith("5240ca8b883e42c0")
road_re = re.compile(r'^ *<road name="[^"]*" length="[^"]*" id="(\d+)" junction="(-?\d+)">.*?</road>\n', re.S | re.M)
junc_re = re.compile(r'^ *<junction name="([^"]*)" id="(\d+)">.*?</junction>\n', re.S | re.M)
roads = {int(m.group(1)): m for m in road_re.finditer(text)}
juncs = {int(m.group(2)): m for m in junc_re.finditer(text)}
P(f"v2 road {len(roads)}, junction {len(juncs)}")
assert len(roads) == 917 and len(juncs) == 112


def links(block):
    out = {}
    for tag in ("predecessor", "successor"):
        m = re.search(rf'<{tag} elementType="(road|junction)" elementId="(\d+)"', block.split("<planView>")[0])
        if m:
            out[tag] = (m.group(1), int(m.group(2)))
    return out


# ---- 1) 지우기
wd = {r["way"]: r for r in csv.DictReader(open(BASE / "docs/logs/s42_way_dir.csv"))}
del_roads = set()
for lab, w in DEL_WAYS.items():
    rs = set(map(int, (wd[w]["roads_fwd"] + " " + wd[w]["roads_bwd"]).split()))
    P(f"삭제 way {lab} {w}: road {sorted(rs)}")
    del_roads |= rs
for rid in list(del_roads):
    assert roads[rid].group(2) == "-1", rid
conn_del = set()
for rid, m in roads.items():
    if m.group(2) != "-1":
        lk = links(m.group(0))
        if any(v[0] == "road" and v[1] in del_roads for v in lk.values()):
            conn_del.add(rid)
P(f"같이 지우는 교차로 연결로 {len(conn_del)}: {sorted(conn_del)}")
gone = del_roads | conn_del
conn_re = re.compile(r'(\s*)<connection id="(\d+)" incomingRoad="(\d+)" connectingRoad="(\d+)" contactPoint="[^"]*">.*?</connection>', re.S)
new_junc = {}
junc_removed = []
for jid, m in juncs.items():
    blk = m.group(0)
    removed = [c for c in conn_re.finditer(blk) if int(c.group(3)) in gone or int(c.group(4)) in gone]
    if not removed:
        continue
    for c in removed:
        assert int(c.group(4)) in gone, (jid, c.group(0))     # 남는 연결로를 쓰는 connection 은 지우지 않는다
    nb = blk
    for c in removed:
        nb = nb.replace(c.group(0), "", 1)
    if not conn_re.search(nb):
        junc_removed.append(jid)
        new_junc[jid] = ""
    else:
        new_junc[jid] = nb
    P(f"junction {jid}({m.group(1)}): connection {len(removed)} 지움" + (", 남은 connection 0 -> junction 지움" if jid in junc_removed else ""))
# 지운 연결로가 속한 junction 확인
for rid in conn_del:
    assert int(roads[rid].group(2)) in new_junc
# 남은 road 중 지운 junction 을 가리키는 link
link_edit = {}
for rid, m in roads.items():
    if rid in gone:
        continue
    blk = m.group(0)
    nb = blk
    for tag in ("predecessor", "successor"):
        mm = re.search(rf'\n\s*<{tag} elementType="junction" elementId="(\d+)"[^>]*/>', nb.split("<planView>")[0])
        if mm and int(mm.group(1)) in junc_removed:
            nb = nb.replace(mm.group(0), "", 1)
    if nb != blk:
        link_edit[rid] = nb
P(f"지운 junction 을 가리켜 link 줄만 지운 road {len(link_edit)}: {sorted(link_edit)}")
# 삭제 뒤 새 막다른 끝: 남은 일반 road 끝 중, 끝 쪽 junction 에서 나가는 connection 이 v2 에는 있었고 v3 에는 없는 것
def out_conn(jblocks, jid, rid):
    b = jblocks.get(jid, "")
    return [c for c in conn_re.finditer(b) if int(c.group(3)) == rid]


v2j = {j: m.group(0) for j, m in juncs.items()}
v3j = dict(v2j)
v3j.update(new_junc)
dead = []
for rid, m in roads.items():
    if rid in gone or m.group(2) != "-1":
        continue
    lk = links(m.group(0))
    s = lk.get("successor")
    if s and s[0] == "junction":
        a, b = out_conn(v2j, s[1], rid), out_conn(v3j, s[1], rid)
        if a and not b:
            dead.append((rid, s[1]))
        elif len(b) < len(a):
            P(f"  r{rid} 끝 junction {s[1]}: 나가는 연결 {len(a)} -> {len(b)}")
P(f"삭제로 새로 생긴 막다른 끝(road 끝에서 나갈 연결이 0 이 됨) {len(dead)}: " + ", ".join(f"r{r}(junction {j})" for r, j in dead))

# ---- 2) 정문 출구
atext = AAN.read_text()
ma = re.search(r'    <road name="" length="[^"]*" id="1392" junction="-1">.*?</road>\n', atext, re.S)
ablk = ma.group(0)
assert 'elementId="109"' in ablk and 'elementId="1"' in ablk
geoms = re.findall(r'<geometry s="([\d.]+)" x="([-\d.]+)" y="([-\d.]+)" hdg="([-\d.]+)" length="([\d.]+)">\s*(<line/>|<paramPoly3 [^>]*/>)\s*</geometry>', ablk)
alen = float(re.search(r'length="([\d.]+)" id="1392"', ablk).group(1))
P(f"A안 r1392(060) geometry {len(geoms)}개, 길이 {alen:.3f}")
xg = []
for s, x, y, h, L, kind in geoms:
    s, x, y, h, L = map(float, (s, x, y, h, L))
    if s + L <= X_S0 + 1e-9 or s >= X_S1 - 1e-9:
        continue
    if s < X_S0:                                   # 첫 line 을 X_S0 에서 자른다
        assert kind == "<line/>"
        d = X_S0 - s
        x, y, L, s = x + d * math.cos(h), y + d * math.sin(h), L - d, X_S0
    assert s + L <= X_S1 + 1e-6, (s, L)
    xg.append((s - X_S0, x, y, h, L, kind))
X_LEN = sum(g[4] for g in xg)
X_START = (xg[0][1], xg[0][2], xg[0][3])
mend = re.search(rf'<geometry s="{X_S1:.8f}" x="([-\d.]+)" y="([-\d.]+)" hdg="([-\d.]+)"', ablk)
X_END = (float(mend.group(1)), float(mend.group(2)), float(mend.group(3)))
P(f"X(3001): geometry {len(xg)}개, 길이 {X_LEN:.3f}m, 시작 ({X_START[0]:.3f}, {X_START[1]:.3f}) hdg {X_START[2]:.5f}, 끝 ({X_END[0]:.3f}, {X_END[1]:.3f}) hdg {X_END[2]:.5f}")


def end_pose(blk):
    gs = re.findall(r'<geometry s="([\d.]+)" x="([-\d.]+)" y="([-\d.]+)" hdg="([-\d.]+)" length="([\d.]+)">\s*(<line/>|<paramPoly3 [^>]*/>)', blk)
    s, x, y, h, L, kind = gs[-1]
    x, y, h, L = map(float, (x, y, h, L))
    if kind == "<line/>":
        return x + L * math.cos(h), y + L * math.sin(h), h
    pp = {k: float(v) for k, v in re.findall(r'(\w+)="([-\d.e]+)"', kind) if k != "pRange"}
    u = pp["aU"] + pp["bU"] + pp["cU"] + pp["dU"]
    v = pp["aV"] + pp["bV"] + pp["cV"] + pp["dV"]
    du, dv = pp["bU"] + 2 * pp["cU"] + 3 * pp["dU"], pp["bV"] + 2 * pp["cV"] + 3 * pp["dV"]
    return x + u * math.cos(h) - v * math.sin(h), y + u * math.sin(h) + v * math.cos(h), h + math.atan2(dv, du)


def start_pose(blk):
    g = re.search(r'<geometry s="0.00000000" x="([-\d.]+)" y="([-\d.]+)" hdg="([-\d.]+)"', blk)
    return float(g.group(1)), float(g.group(2)), float(g.group(3))


e1255 = end_pose(roads[1255].group(0))
s1150 = start_pose(roads[1150].group(0))
P(f"v2 r1255 끝 ({e1255[0]:.3f}, {e1255[1]:.3f}) hdg {e1255[2]:.5f}; v2 r1150 시작 ({s1150[0]:.3f}, {s1150[1]:.3f}) hdg {s1150[2]:.5f}")


def hermite(p0, p1):
    """p0·p1 = (x, y, hdg). paramPoly3(normalized) 계수와 호 길이"""
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


c1, L1, R1 = hermite(e1255, X_START)
c2, L2, R2 = hermite(X_END, s1150)
P(f"C1(3002, j109): 길이 {L1:.3f}m, 기준선 최소 반경 {R1:.1f}m; C2(3003, j1): 길이 {L2:.3f}m, 기준선 최소 반경 {R2:.1f}m")

LANES_NORMAL = re.search(r'        <lanes>.*?</lanes>\n', roads[1255].group(0), re.S).group(0)
LANES_CONN = re.search(r'        <lanes>.*?</lanes>\n', roads[1969].group(0), re.S).group(0)


def pp_xml(c):
    return ('<paramPoly3 ' + " ".join(f'{k}="{c[k]:.8f}"' for k in ("aU", "bU", "cU", "dU", "aV", "bV", "cV", "dV"))
            + ' pRange="normalized"/>')


def geom_xml(s, x, y, h, L, kind):
    return (f'            <geometry s="{s:.8f}" x="{x:.8f}" y="{y:.8f}" hdg="{h:.8f}" length="{L:.8f}">\n'
            f'                {kind}\n            </geometry>\n')


x_xml = (f'    <road name="" length="{X_LEN:.8f}" id="{X_ID}" junction="-1">\n'
         f'        <link>\n            <predecessor elementType="junction" elementId="109"/>\n'
         f'            <successor elementType="junction" elementId="1"/>\n        </link>\n'
         f'        <type s="0" type="town"/>\n        <planView>\n'
         + "".join(geom_xml(*g) for g in xg)
         + '        </planView>\n        <elevationProfile>\n            <elevation s="0" a="0.00" b="0" c="0" d="0"/>\n'
         '        </elevationProfile>\n        <lateralProfile/>\n' + LANES_NORMAL + '        <objects/>\n    </road>\n')


def conn_xml(rid, name, jid, pred, succ, p0, c, L):
    return (f'    <road name="{name}" length="{L:.8f}" id="{rid}" junction="{jid}">\n'
            f'    <link>\n        <predecessor elementType="road" elementId="{pred}" contactPoint="end"/>\n'
            f'        <successor elementType="road" elementId="{succ}" contactPoint="start"/>\n    </link>\n'
            f'    <type s="0" type="town"/>\n    <planView>\n'
            f'        <geometry s="0.00000000" x="{p0[0]:.8f}" y="{p0[1]:.8f}" hdg="{p0[2]:.8f}" length="{L:.8f}">\n'
            f'            {pp_xml(c)}\n        </geometry>\n    </planView>\n'
            '        <elevationProfile>\n            <elevation s="0" a="0.00" b="0" c="0" d="0"/>\n        </elevationProfile>\n'
            '        <lateralProfile/>\n' + LANES_CONN + '        <objects/>\n        <signals>\n        </signals>\n</road>\n')


c1_xml = conn_xml(C1_ID, ":4748296088_s43", 109, 1255, X_ID, e1255, c1, L1)
c2_xml = conn_xml(C2_ID, ":4748296080_s43", 1, X_ID, 1150, X_END, c2, L2)


def add_conn(blk, inc, con):
    ids = [int(c.group(2)) for c in conn_re.finditer(blk)]
    nid = max(ids) + 1
    add = (f'        <connection id="{nid}" incomingRoad="{inc}" connectingRoad="{con}" contactPoint="start">\n'
           f'            <laneLink from="-1" to="-1"/>\n            <laneLink from="-2" to="-2"/>\n        </connection>\n')
    k = blk.rindex("    </junction>")
    return blk[:k] + add + blk[k:], nid


for jid, inc, con in ((109, 1255, C1_ID), (1, X_ID, C2_ID)):
    base = new_junc.get(jid, v2j[jid])
    nb, nid = add_conn(base, inc, con)
    new_junc[jid] = nb
    P(f"junction {jid}: connection {nid} 추가 (incoming {inc} -> connecting {con})")

# ---- 조립(원래 순서 유지, 새 road 는 마지막 road 뒤)
out, pos = [], 0
spans = []
for rid, m in roads.items():
    spans.append((m.start(), m.end(), "road", rid))
for jid, m in juncs.items():
    spans.append((m.start(), m.end(), "junc", jid))
spans.sort()
last_road_end = max(m.end() for m in roads.values())
for a, b, kind, i in spans:
    out.append(text[pos:a])
    if kind == "road":
        if i in gone:
            pass
        else:
            out.append(link_edit.get(i, text[a:b]))
    else:
        out.append(new_junc.get(i, text[a:b]))
    if b == last_road_end:
        out.append(x_xml + c1_xml + c2_xml)
    pos = b
out.append(text[pos:])
v3 = "".join(out)
OUT.write_text(v3)
sha = hashlib.sha256(v3.encode()).hexdigest()
nr = len(re.findall(r'<road name=', v3))
nj = len(re.findall(r'<junction name=', v3))
P(f"출력 {OUT.name}: road {nr} (v2 917 - 지움 {len(gone)} + 새 3), junction {nj} (v2 112 - 지움 {len(junc_removed)}), sha256 {sha}")
P(f"지운 road {len(gone)}: 일반 {sorted(del_roads)} / 연결로 {sorted(conn_del)}")
P(f"지운 junction {sorted(junc_removed)}")
LOG.write_text("\n".join(log) + "\n")
