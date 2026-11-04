#!/usr/bin/env python3
# 11/02: v3.3a Autoware Lanelet2 지도 만들기(xodr·CARLA 월드는 v3.1 동결본 그대로, Lanelet2 에서만 빼고 더함)
# 입력(읽기만): v3.1 빌더 출력 map/build_s57_regen/build(83edb775, r1318·r1188 이 아직 있음) + 그 l1_meta.json
# 1) 뺄 road(근거 표는 DROP): s55_drop_lanelets.py 와 같은 방법(줄 단위로 relation·way·혼자 쓰는 node 를 뺌, 남은 줄은 바이트 그대로)
#    더해서 뺀 road 에만 붙은 교차로 연결로와, 빼고 나서 본선과 아예 안 이어지는 조각(약 연결 성분)도 뺌
# 2) 북문 길(r1415) 끝 유턴 레인렛 하나를 더함: r1415 마지막 레인렛 끝 -> r1409 첫 레인렛 처음. 열쇠 구멍 모양(오른쪽 R_OUT 로 a 만큼 벌렸다가
#    왼쪽 R_U 로 180+2a 돌고 오른쪽 R_OUT 로 a 돌아옴). 중심선 반경 R_U 는 스타렉스 뒷축 최소 반경 3.77m 보다 넉넉하게
#    넓이 근거: 현장 확인(11/2, '그 자리는 넓어서 버스들도 많이 돌림'). OSM 에는 그 넓이를 그린 면이 없음
# 3) r1318·r1188(N14 주차장 길) 은 빼지 않음 = 되살림(10/12 에서 뺐던 것). : 'N14 건물을 둘러싼 주차장을 활용해 차 돌려서 나옴'
# 출력: map/maps/v33a/{lanelet2_map.osm, map_projector_info.yaml, v33a_changes.json}
# 사용: python3 map/scripts/s73_make_v33.py [--r-u 5.0] [--r-out 5.0]
import argparse, collections, hashlib, json, math, re, shutil, subprocess, sys
from pathlib import Path

C = Path(__file__).resolve().parents[2]
SRC = C / "map/build_s57_regen/build"
OUT = C / "map/maps/v33a"
ap = argparse.ArgumentParser(); ap.add_argument("--r-u", type=float, default=5.0); ap.add_argument("--r-out", type=float, default=5.0)
ap.add_argument("--uw", type=float, default=4.0, help="유턴 레인렛 가운데 폭(양 끝은 기존 차로 폭, 사인으로 이음)")
ap.add_argument("--out", default=str(OUT)); a = ap.parse_args()
OUT = Path(a.out)

# road, OSM way, OSM 태그, 근거, 갈래(OSM대로 / 현장 확인으로 OSM 과 다르게)
DROP = [
    (1320, "442595034", "service/parking_aisle", "볼라드: 지금 중문 승강장 바로 위(중문 쪽)에 볼라드, 차 못 지나감(11/02, mid_check.png ✕)", "다르게"),
    (1191, "442595034", "service/parking_aisle", "같은 조각 반대 방향", "다르게"),
    (1317, "442595034", "service/parking_aisle", "개성재 자동문 게이트 G5(관리동 앞 교차로)로 들어가는 조각(11/02: 게이트 = G5, 셔틀은 개성재 안 안 감)", "다르게"),
    (1190, "442595034", "service/parking_aisle", "같은 조각 반대 방향", "다르게"),
    (1360, "446440351", "service/parking_aisle", "개성재 안쪽 관리동 앞 길, 자동문 게이트 안(밤에 닫힘, 11/02)", "다르게"),
    (1229, "446440351", "service/parking_aisle", "같은 길 반대 방향", "다르게"),
    (1194, "442595037", "service/parking_aisle", "정의관·미술과 남쪽 동서 길: 개성재 게이트 안(11/02), 기억으로는 지도처럼 안 이어짐", "다르게"),
    (1323, "442595037", "service/parking_aisle", "같은 길 반대 방향", "다르게"),
    (1195, "442595037", "service/parking_aisle", "같은 way 동쪽 조각(동쪽 고리로 이어짐)", "다르게"),
    (1324, "442595037", "service/parking_aisle", "같은 조각 반대 방향", "다르게"),
    (1312, "442595037", "service/parking_aisle", "동쪽 고리(두 번째 좌회전 뒤 도는 길): 11/02 '좌회전 두 번으로 돌아 나가지 말고 차만 돌려 내려옴'", "다르게"),
    (1325, "442361829", "service/parking_aisle", "동쪽 고리 반대 방향", "다르게"),
    (3004, "(10/09 연결)", "-", "10/09 에 이은 북문 고리 연결: 두 번째 좌회전(11/02 안 씀)", "다르게"),
    (3005, "(10/09 연결)", "-", "같은 연결 반대 방향", "다르게"),
    (3006, "(10/09 교차로)", "-", "북문 길 끝 -> 연결 교차로", "다르게"),
    (3007, "(10/09 교차로)", "-", "연결 -> 북문 길 교차로", "다르게"),
    (3008, "(10/09 교차로)", "-", "연결 -> 동쪽 고리 교차로", "다르게"),
    (3009, "(10/09 교차로)", "-", "동쪽 고리 -> 연결 교차로", "다르게"),
    (1235, "468616604", "footway(lit=yes)", "OSM 인도(footway)가 태그 보정 때 차도로 바뀐 것. 남문(이쪽) 문 구역 자리", "OSM대로"),
    (1369, "468616604", "footway(lit=yes)", "같은 인도 반대 방향", "OSM대로"),
]
KEEP_NOTE = {1318: "N14 주차장 길(way 442595030) 되살림: 10/12 에서 Lanelet2 에서만 뺐던 것. 11/02 'N14 를 둘러싼 주차장으로 차 돌림'(10/4 답사: ㄷ자, 양쪽 열림, 양방향)",
             1188: "같은 길 반대 방향 되살림"}

meta = json.loads((SRC / "l1_meta.json").read_text())
LANES = {int(k.split(",")[0]) for k in meta["lanelet"]}
E = collections.defaultdict(set); ER = collections.defaultdict(set)
for x, y in meta["edges"]:
    rx, ry = int(x.split(",")[0]), int(y.split(",")[0])
    if rx in LANES and ry in LANES: E[rx].add(ry); ER[ry].add(rx)
drop = {d[0] for d in DROP}
JUNC = {}
import xml.etree.ElementTree as ET
for r in ET.parse(C / "map/maps/cbnu_campus_frozen_v31.xodr").getroot().iter("road"):
    JUNC[int(r.get("id"))] = int(r.get("junction", "-1"))
# 뺀 road 에 붙은 교차로 연결로(앞이나 뒤가 뺀 road)
conn = {r for r in LANES if JUNC.get(r, -1) != -1 and ((E[r] & drop) or (ER[r] & drop))}
drop2 = drop | conn
# 본선과 안 이어지는 조각(약 연결 성분). 본선 = 정문 쪽 본선 r1392 가 든 성분
rem = LANES - drop2
U = collections.defaultdict(set)
for r in rem:
    for v in E[r]:
        if v in rem: U[r].add(v); U[v].add(r)
seen = {1392}; st = [1392]
while st:
    u = st.pop()
    for v in U[u]:
        if v not in seen: seen.add(v); st.append(v)
iso = rem - seen
drop3 = drop2 | iso
print("직접 뺌", len(drop), "붙은 연결로", len(conn), "떨어진 조각", len(iso), sorted(r for r in iso if JUNC.get(r, -1) == -1))
OUT.mkdir(parents=True, exist_ok=True)
tmp = OUT / "_drop"
if tmp.exists(): shutil.rmtree(tmp)
subprocess.run([sys.executable, str(C / "map/scripts/s55_drop_lanelets.py"), str(SRC), str(tmp)] + [str(r) for r in sorted(drop3)], check=True,
               stdout=subprocess.DEVNULL)
text = (tmp / "lanelet2_map.osm").read_text()

# ---- 유턴 레인렛 ----
node_xy = {m.group(1): (float(m.group(3)), float(m.group(4))) for m in re.finditer(
    r'<node id="(\d+)"[^>]*lat="([-\d.]+)"[^>]*>\s*<tag k="ele" v="[-\d.]+" />\s*<tag k="local_x" v="([-\d.]+)" />\s*<tag k="local_y" v="([-\d.]+)" />', text)}
latlon = {m.group(1): (float(m.group(2)), float(m.group(3))) for m in re.finditer(r'<node id="(\d+)" visible="true" version="1" lat="([-\d.]+)" lon="([-\d.]+)"', text)}
# 위경도 = local 에 비례(원점 0,0). 기존 노드로 맞춤
ks = list(node_xy)[:2000]
ky = sum(latlon[k][0] * node_xy[k][1] for k in ks) / sum(node_xy[k][1] ** 2 for k in ks)
kx = sum(latlon[k][1] * node_xy[k][0] for k in ks) / sum(node_xy[k][0] ** 2 for k in ks)
err = max(abs(latlon[k][0] - ky * node_xy[k][1]) + abs(latlon[k][1] - kx * node_xy[k][0]) for k in ks)
ways = {m.group(1): re.findall(r'<nd ref="(\d+)" />', m.group(2)) for m in re.finditer(r'<way id="(\d+)"[^>]*>(.*?)</way>', text, re.S)}
rels = {}
for m in re.finditer(r'<relation id="(\d+)"[^>]*>(.*?)</relation>', text, re.S):
    body = m.group(2); tags = dict(re.findall(r'<tag k="([^"]+)" v="([^"]+)" />', body))
    if tags.get("type") != "lanelet": continue
    mem = dict((role, ref) for ref, role in re.findall(r'<member type="way" ref="(\d+)" role="(\w+)" />', body))
    rels[m.group(1)] = (tags, mem)
last1415 = str(meta["lanelet_last"]["1415,0,-1"]); first1409 = str(meta["lanelet"]["1409,0,-1"])
tA, mA = rels[last1415]; tB, mB = rels[first1409]
LA, RA = ways[mA["left"]], ways[mA["right"]]; LB, RB = ways[mB["left"]], ways[mB["right"]]
pL0, pR0 = node_xy[LA[-1]], node_xy[RA[-1]]      # r1415 끝 왼·오
pL1, pR1 = node_xy[LB[0]], node_xy[RB[0]]        # r1409 처음 왼·오
c0 = ((pL0[0] + pR0[0]) / 2, (pL0[1] + pR0[1]) / 2); c1 = ((pL1[0] + pR1[0]) / 2, (pL1[1] + pR1[1]) / 2)
q = node_xy[LA[-2]], node_xy[RA[-2]]
h0 = math.atan2(c0[1] - (q[0][1] + q[1][1]) / 2, c0[0] - (q[0][0] + q[1][0]) / 2)
w = math.dist(pL0, pR0)
# r1409 처음 중심이 r1415 끝 중심에서 왼쪽으로 얼마나(옆), 앞뒤로 얼마나
dx, dy = c1[0] - c0[0], c1[1] - c0[1]
lat_d = -dx * math.sin(h0) + dy * math.cos(h0); lon_d = dx * math.cos(h0) + dy * math.sin(h0)


def keyhole(alpha, Ru, Ro, step=0.25):
    """중심선 점 열(r1415 끝 기준 좌표 앞 x, 왼 y)과 끝의 옆 이동"""
    pts = [(0.0, 0.0)]; x = y = 0.0; h = 0.0
    def arc(R, ang, left):
        nonlocal x, y, h
        n = max(2, int(abs(ang) * R / step))
        for _ in range(n):
            dh = ang / n * (1 if left else -1)
            # 원 위로 한 칸
            cx = x - R * math.sin(h) * (1 if left else -1); cy = y + R * math.cos(h) * (1 if left else -1)
            h2 = h + dh
            x = cx + R * math.sin(h2) * (1 if left else -1); y = cy - R * math.cos(h2) * (1 if left else -1); h = h2
            pts.append((x, y))
    arc(Ro, alpha, False); arc(Ru, math.pi + 2 * alpha, True); arc(Ro, alpha, False)
    return pts, x, y, h


# 옆 이동 = lat_d 가 되게 a 를 찾음
lo, hi = 0.0, math.pi / 2
for _ in range(60):
    mid = (lo + hi) / 2
    _, ex, ey, _ = keyhole(mid, a.r_u, a.r_out)
    if ey > lat_d: lo = mid
    else: hi = mid
alpha = (lo + hi) / 2
pts, ex, ey, eh = keyhole(alpha, a.r_u, a.r_out)
# 앞뒤 차 맞추기: 끝 점 앞뒤(x)가 r1409 처음과 같게. 모자라면 처음에 곧게 늘리고, 넘치면 끝에서 곧게 더 감(끝 방향은 거의 뒤쪽)
shift = lon_d - ex
if shift > 0:
    n = int(shift / 0.25) + 1
    pts = [(shift * i / n, 0.0) for i in range(n)] + [(px + shift, py) for px, py in pts]
elif shift < 0:
    n = int(-shift / 0.25) + 1
    pts = pts + [(ex + math.cos(eh) * (-shift) * i / n / abs(math.cos(eh)), ey) for i in range(1, n + 1)]
# 지도 좌표로
def to_map(px, py):
    return c0[0] + px * math.cos(h0) - py * math.sin(h0), c0[1] + px * math.sin(h0) + py * math.cos(h0)
cl = [to_map(*p) for p in pts]
# 경계: 중심선에서 왼·오 w/2(방향은 앞뒤 점)
def offset(cl, d):
    out = []; n_ = len(cl)
    for i, p in enumerate(cl):
        dd = d * (1 + (a.uw / w - 1) * math.sin(math.pi * i / (n_ - 1)))   # 11/02: 가운데로 갈수록 넓게(넓이 근거: 현장 확인)
        a0 = cl[max(0, i - 1)]; b0 = cl[min(len(cl) - 1, i + 1)]
        hh = math.atan2(b0[1] - a0[1], b0[0] - a0[0])
        out.append((p[0] - dd * math.sin(hh), p[1] + dd * math.cos(hh)))
    return out
left = offset(cl, w / 2); right = offset(cl, -w / 2)
# 양 끝은 기존 노드를 그대로 씀(끝 점 공유 = 이어짐)
nid = max(int(x) for x in re.findall(r'<node id="(\d+)"', text)) + 1
wid = max(int(x) for x in re.findall(r'<way id="(\d+)"', text)) + 1
rid = max(int(x) for x in re.findall(r'<relation id="(\d+)"', text)) + 1
new_nodes = []; lref = [LA[-1]]; rref = [RA[-1]]
for seq, refs in ((left[1:-1], lref), (right[1:-1], rref)):
    for x, y in seq[::4]:
        new_nodes.append((nid, x, y)); refs.append(str(nid)); nid += 1
lref.append(LB[0]); rref.append(RB[0])
nl = "".join(f'  <node id="{i}" visible="true" version="1" lat="{ky * y:.11f}" lon="{kx * x:.11f}">\n    <tag k="ele" v="0.000000" />\n'
             f'    <tag k="local_x" v="{x:.6f}" />\n    <tag k="local_y" v="{y:.6f}" />\n  </node>\n' for i, x, y in new_nodes)
wl = "".join(f'  <way id="{w_}" visible="true" version="1">\n' + "".join(f'    <nd ref="{r}" />\n' for r in refs) + "  </way>\n"
             for w_, refs in ((wid, lref), (wid + 1, rref)))
tags = {k: v for k, v in tA.items() if k not in ("xodr_road", "xodr_section", "xodr_lane")}
tags.update({"turn_direction": "left", "s73_uturn": "north_gate_end", "s73_note": "user field: wide paved area, buses turn here (2026-11-02)"})
rl = (f'  <relation id="{rid}" visible="true" version="1">\n    <member type="way" ref="{wid}" role="left" />\n    <member type="way" ref="{wid + 1}" role="right" />\n'
      + "".join(f'    <tag k="{k}" v="{v}" />\n' for k, v in tags.items()) + "  </relation>\n")
# 노드는 node 블록 끝에, way 는 way 끝에, relation 은 </osm> 앞에
i_way = text.find("  <way "); i_rel = text.find("  <relation "); i_end = text.rfind("</osm>")
text = text[:i_way] + nl + text[i_way:i_rel] + wl + text[i_rel:i_end] + rl + text[i_end:]
(OUT / "lanelet2_map.osm").write_text(text)
shutil.copy(SRC / "map_projector_info.yaml", OUT / "map_projector_info.yaml")
shutil.rmtree(tmp)
# 유턴 넓이: 중심선에서 차로 밖으로 얼마나 나가나(r1415 끝 기준 오른쪽 -, 왼쪽 +)
ys = [p[1] for p in pts]; xs = [p[0] for p in pts]
info = dict(uw=a.uw, r_u=a.r_u, r_out=a.r_out, alpha_deg=round(math.degrees(alpha), 2), lane_w=round(w, 3), lat_gap=round(lat_d, 3), lon_gap=round(lon_d, 3),
            center_right_max=round(-min(ys), 2), center_left_max=round(max(ys), 2), center_forward_max=round(max(xs), 2),
            need_right_of_r1415_center=round(-min(ys) + w / 2, 2), need_left_of_r1409_center=round(max(ys) - lat_d + w / 2, 2),
            relation=rid, nodes=len(new_nodes), latlon_fit_err=err, last1415=last1415, first1409=first1409)
h = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
res = dict(source=str(SRC.relative_to(C)), source_sha256=h(SRC / "lanelet2_map.osm"), drop=[dict(road=r, way=w_, osm_tag=t, why=y, kind=k) for r, w_, t, y, k in DROP],
           drop_connectors=sorted(conn), drop_isolated=sorted(iso), keep_restored={str(k): v for k, v in KEEP_NOTE.items()}, uturn=info,
           out_sha256=h(OUT / "lanelet2_map.osm"), projector_sha256=h(OUT / "map_projector_info.yaml"))
(OUT / "v33a_changes.json").write_text(json.dumps(res, ensure_ascii=False, indent=1))
print(json.dumps(info, ensure_ascii=False)); print("out", res["out_sha256"][:16], "projector", res["projector_sha256"][:16])
