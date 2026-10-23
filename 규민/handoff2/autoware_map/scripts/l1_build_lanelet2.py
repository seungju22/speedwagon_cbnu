#!/usr/bin/env python3
# L1/A1: xodr -> Autoware 용 Lanelet2 지도(lanelet2_map.osm + map_projector_info.yaml). 서버 없이 carla.Map(오프라인)으로 만든다.
# 왜 crdesigner(세션29~31) 대신 이것인가: 세션30·31 에서 crdesigner 출력은 원인 A(laneOffset 커넥터 끝점 교체)·
#   원인 B(경계 뒤집혀 읽힘)·r1917 이음매(1.1m) 로 끊김 31~33 이 남았다. 세션31 10-2 의 "대안"(점 공유를 연결 관계와
#   위치로 직접 정함)을 CARLA 가 실제로 쓰는 차선 기하로 구현한 것이다. 같은 입력을 crdesigner 로도 돌려 대조한다(l1_crd_baseline.py).
# 흐름
#   1 carla.Map(name, xodr 문자열).get_topology() = (차선 시작 wp, 다음 차선 시작 wp) 쌍 = 차선 연결 간선. Driving 차선만 나온다
#   2 차선마다 시작 wp 에서 next_until_lane_end(STEP) 로 중심선 표본. 경계 = 중심 ± 차선폭/2 (차선 진행 방향 왼쪽 법선)
#   3 좌표: ROS(map) = (x, -y). CARLA 는 xodr y 를 뒤집어 읽으므로 이것이 xodr 평면 좌표 그대로다. header offset 은 적용하지 않는다
#   4 연결 간선 A->B 마다 A 끝 왼/오 점과 B 시작 왼/오 점을 같은 점으로 합친다(union-find, 위치는 평균). 이동 거리 기록
#   5 회차 연결로(차선 방위 변화 >= 150도)는 기본으로 뺀다(--keep-uturn 이면 넣는다)
# 출력: <out>/lanelet2_map.osm, map_projector_info.yaml, l1_meta.json(차선 키 -> lanelet id, 간선, 뺀 목록, 점 이동)
# 사용: .venv-carla/bin/python l1_build_lanelet2.py <xodr> <out_dir> [--keep-uturn] [--speed 20] [--name OpenDriveMap] [--chord-fix JSON] [--no-split] [--max-pieces 3]
#   2026-10-11: 나누기 기본 켬(로더 align 에 걸리는 lanelet 을 조각으로). --no-split 이면 v3a(45ff2015795f22d2) 와 같은 바이트
import argparse
import collections
import hashlib
import json
import math
from pathlib import Path

import carla

from l1_align_sim import flips      # 같은 폴더. lanelet2 로더 align 판정을 똑같이 따라 한 것

STEP = 1.0           # 중심선 표본 간격 m
UTURN_DEG = 150.0
TAPER = 5.0          # 끝점 이동을 안쪽으로 나눠 주는 길이 m    # 세션39 s39_turn_radius.py 와 같은 회차 정의(방위 변화)
DEG_PER_M = 1.0 / 111319.49   # lat/lon 은 Local 투영에서 쓰이지 않는다(local_x/local_y 가 덮어씀). 참고용 근사값


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def key(wp):
    return (wp.road_id, wp.section_id, wp.lane_id)


def ros(loc):
    return (loc.x, -loc.y)


def lane_samples(wp, m, road_len):
    # next_until_lane_end 의 마지막 점은 차선 끝에서 최대 STEP 앞이라 정확한 끝 점을 더한다(이 맵은 road 마다 lane section 1개, 주행 차선 lane -1 만)
    pts = [wp] + list(wp.next_until_lane_end(STEP))
    if wp.lane_id < 0:
        end = m.get_waypoint_xodr(wp.road_id, wp.lane_id, road_len[wp.road_id] - 1e-3)
        if end is not None:
            pts.append(end)
    out = [pts[0]]
    for p in pts[1:]:
        a, b = out[-1].transform.location, p.transform.location
        if math.hypot(a.x - b.x, a.y - b.y) > 0.05 and key(p) == key(wp):
            out.append(p)
    return out


def bounds(samples):
    L, R, C = [], [], []
    for w in samples:
        x, y = ros(w.transform.location)
        yaw = -math.radians(w.transform.rotation.yaw)
        nx, ny = -math.sin(yaw), math.cos(yaw)
        h = w.lane_width / 2
        C.append((x, y))
        L.append((x + h * nx, y + h * ny))
        R.append((x - h * nx, y - h * ny))
    return L, R, C


def turn_deg(C):
    t = 0.0
    hs = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(C, C[1:]) if a != b]
    for a, b in zip(hs, hs[1:]):
        d = (b - a + math.pi) % (2 * math.pi) - math.pi
        t += d
    return math.degrees(t)


SEAM_MOVE = 0.5      # 끝점 이동이 이보다 크면 이음매로 보고 SEAM_TAPER 로 길게 나눈다(2026-10-11 m1: r1972 1.087m 를 5m 안에 몰아 path_optimizer 가 drivable area 밖으로 보고 섬)
SEAM_TAPER = [15.0]  # --seam-taper 로 바꾼다. 0 이면 끔(v3b 와 같은 출력)


def taper(pts, ns, ne):
    ds, de = math.dist(pts[0], ns), math.dist(pts[-1], ne)
    if SEAM_TAPER[0] > 0 and max(ds, de) > SEAM_MOVE:
        return taper_seam(pts, ns, ne, ds, de)
    return taper_old(pts, ns, ne)


def taper_seam(pts, ns, ne, ds, de):
    # 큰 쪽 끝은 SEAM_TAPER m, 작은 쪽은 TAPER m. 둘 합이 차선 길이보다 길면 시작 이동 -> 끝 이동을 차선 전체에 직선 보간
    acc = [0.0]
    for p, q in zip(pts, pts[1:]):
        acc.append(acc[-1] + math.dist(p, q))
    tot = acc[-1] or 1.0
    Ts = SEAM_TAPER[0] if ds > SEAM_MOVE else TAPER
    Te = SEAM_TAPER[0] if de > SEAM_MOVE else TAPER
    dsx, dsy = ns[0] - pts[0][0], ns[1] - pts[0][1]
    dex, dey = ne[0] - pts[-1][0], ne[1] - pts[-1][1]
    out = []
    for p, t in zip(pts, acc):
        if Ts + Te >= tot:
            ws, we = 1 - t / tot, t / tot
        else:
            ws, we = max(0.0, 1 - t / Ts), max(0.0, 1 - (tot - t) / Te)
        out.append((p[0] + ws * dsx + we * dex, p[1] + ws * dsy + we * dey))
    out[0], out[-1] = ns, ne
    return out


def taper_old(pts, ns, ne):
    # 끝점을 공유 위치로 옮긴 만큼 안쪽 점도 TAPER m 에 걸쳐 줄여 가며 옮긴다(끝 한 점만 꺾이지 않게). 차선 길이 절반을 넘지 않는다
    acc = [0.0]
    for p, q in zip(pts, pts[1:]):
        acc.append(acc[-1] + math.dist(p, q))
    tot = acc[-1]
    T = min(TAPER, tot / 2) if tot > 0 else 1.0
    dsx, dsy = ns[0] - pts[0][0], ns[1] - pts[0][1]
    dex, dey = ne[0] - pts[-1][0], ne[1] - pts[-1][1]
    out = []
    for p, t in zip(pts, acc):
        ws, we = max(0.0, 1 - t / T), max(0.0, 1 - (tot - t) / T)
        out.append((p[0] + ws * dsx + we * dex, p[1] + ws * dsy + we * dey))
    out[0], out[-1] = ns, ne
    return out


def split_cuts(T, C, max_pieces):
    # 나눌 표본 번호 [0, c1, (c2,) n-1]. 조각마다 점 2개 이상이고 flips() 가 둘 다 False 인 것 중 곡선 중간에 가장 가까운 것.
    # 곡선 중간 = 중심선 방위 변화 누적이 전체의 1/2(2조각), 1/3·2/3(3조각) 이 되는 표본
    n = len(T["L"])
    acc = [0.0, 0.0]
    for o, p, q in zip(C, C[1:], C[2:]):
        d = (math.atan2(q[1] - p[1], q[0] - p[0]) - math.atan2(p[1] - o[1], p[0] - o[0]) + math.pi) % (2 * math.pi) - math.pi
        acc.append(acc[-1] + abs(d))
    acc += [acc[-1]] * (n - len(acc))
    tot = acc[-1] or 1.0

    def at(f):
        return min(range(n), key=lambda i: abs(acc[i] - f * tot))

    def ok(cs):
        return all(c1 - c0 >= 1 and not any(flips(T["L"][c0:c1 + 1], T["R"][c0:c1 + 1])) for c0, c1 in zip(cs, cs[1:]))
    for k in range(2, max_pieces + 1):
        aims = [at(j / k) for j in range(1, k)]
        if k == 2:
            cands = [[c] for c in range(1, n - 1)]
        else:
            cands = [[c1, c2] for c1 in range(1, n - 2) for c2 in range(c1 + 1, n - 1)]
        cands.sort(key=lambda cs: sum(abs(c - t) for c, t in zip(cs, aims)))
        for cs in cands:
            if ok([0] + cs + [n - 1]):
                return [0] + cs + [n - 1]
    return None


def split_chord(ch, ot, cs, C):
    # 현(점 2개) 쪽에 현 위 점 q 를 넣고 다른 쪽은 표본 c 에서 잘라 2조각. 두 조각 모두 flips() False 인 (c, q) 중
    # c 가 곡선 중간(방위 변화 누적 1/2)에 가깝고 q 가 ot[c] 를 현에 투영한 자리에 가까운 것
    n = len(ot)
    acc = [0.0, 0.0]
    for o, p, q in zip(C, C[1:], C[2:]):
        acc.append(acc[-1] + abs((math.atan2(q[1] - p[1], q[0] - p[0]) - math.atan2(p[1] - o[1], p[0] - o[0]) + math.pi) % (2 * math.pi) - math.pi))
    acc += [acc[-1]] * (n - len(acc))
    mid = min(range(n), key=lambda i: abs(acc[i] - acc[-1] / 2))
    (x0, y0), (x1, y1) = ch
    L2 = (x1 - x0) ** 2 + (y1 - y0) ** 2 or 1.0
    best = None
    for c in range(1, n - 1):
        u0 = ((ot[c][0] - x0) * (x1 - x0) + (ot[c][1] - y0) * (y1 - y0)) / L2
        for j in range(1, 40):
            u = j / 40
            q = (x0 + u * (x1 - x0), y0 + u * (y1 - y0))
            p1 = ([ch[0], q], ot[:c + 1]); p2 = ([q, ch[1]], ot[c:])
            if cs == "R":
                p1, p2 = p1[::-1], p2[::-1]
            if not any(flips(*p1)) and not any(flips(*p2)):
                sc = (abs(c - mid), abs(u - u0))
                if best is None or sc < best[0]:
                    best = (sc, c, q)
    return (best[1], best[2]) if best else None


class UF:
    def __init__(self):
        self.p = {}

    def find(self, a):
        self.p.setdefault(a, a)
        while self.p[a] != a:
            self.p[a] = self.p[self.p[a]]
            a = self.p[a]
        return a

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.p[max(ra, rb)] = min(ra, rb)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("xodr")
    ap.add_argument("out")
    ap.add_argument("--keep-uturn", action="store_true")
    ap.add_argument("--speed", default="20")
    ap.add_argument("--name", default="OpenDriveMap")
    ap.add_argument("--chord-fix", help="l1_eval_pairs.json. 그 inverted 목록 lanelet 의 안쪽 경계를 양 끝 직선(현)으로 바꾼다."
                    " lanelet2 로더(geometry::align)가 급커브 안쪽 경계 가운데 점 때문에 경계를 뒤집어 끊기는 것을 막는다")
    ap.add_argument("--no-split", action="store_true",
                    help="나누기 끔(v3a 와 같은 출력). 기본은 켬: 로더 align 판정(l1_align_sim.py)에 걸리는 lanelet 을 곡선 중간에서"
                    " 2~MAX_PIECES 조각으로 나눈다. 점 좌표는 그대로(현 바꾸기도 그 lanelet 에는 쓰지 않는다), 나눈 자리 점은 앞뒤 조각이 공유")
    ap.add_argument("--max-pieces", type=int, default=3)
    ap.add_argument("--seam-taper", type=float, default=15.0,
                    help="끝점 이동 0.5m 초과 이음매의 보정을 나눠 주는 길이 m(기본 15). 0 이면 끔 = v3b(d7a7122605265142)")
    a = ap.parse_args()
    SEAM_TAPER[0] = a.seam_taper
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    h = sha(a.xodr)
    print(f"입력 {a.xodr} sha256 {h[:16]}")
    m = carla.Map(a.name, Path(a.xodr).read_text())
    import xml.etree.ElementTree as ET
    xr = ET.parse(a.xodr).getroot()
    road_len = {int(r.get("id")): float(r.get("length")) for r in xr.iter("road")}
    nsec = sum(len(r.findall("lanes/laneSection")) for r in xr.iter("road"))
    assert nsec == len(road_len), "lane section 이 둘 이상인 road 가 있다. 끝 점 계산을 고쳐야 한다"
    topo = m.get_topology()
    lanes = {}
    edges = set()
    selfe = []
    for w1, w2 in topo:
        lanes.setdefault(key(w1), w1)
        lanes.setdefault(key(w2), w2)
        if key(w1) == key(w2):       # 막다른 끝: CARLA 가 (시작, 같은 차선 끝) 을 준다. 간선 아님
            selfe.append(key(w1))
            continue
        edges.add((key(w1), key(w2)))
    print(f"topology 쌍 {len(topo)}, 차선 {len(lanes)}, 간선 {len(edges)}, 막다른 끝(자기 쌍) {selfe}")
    geo = {}
    for k, w in lanes.items():
        s = lane_samples(w, m, road_len)
        if len(s) < 2:      # 아주 짧은 차선: 끝점 하나 더(같은 차선 끝 = 다음 차선 시작이 아니므로 0.5*길이 앞으로)
            nx = [p for p in w.next(0.05) if key(p) == k]
            s = s + nx[:1]
        L, R, C = bounds(s)
        geo[k] = dict(L=L, R=R, C=C, turn=turn_deg(C), junction=w.is_junction, width=w.lane_width,
                      s0=s[0].s, s1=s[-1].s, n=len(s))
    # 회차 판정: junction 커넥터 차선에 대해 "들어오는 차선 끝 방위 -> 나가는 차선 시작 방위" 차가 UTURN_DEG 이상.
    # 차선 자체 방위 변화로 재면 회차 틀(laneOffset 3.35, 9.89m)이 127도로 나와 놓친다(앞뒤 이음매에서 26도씩 더 꺾임)
    pred = collections.defaultdict(list)
    succ = collections.defaultdict(list)
    for x, y in edges:
        succ[x].append(y)
        pred[y].append(x)

    def hd(C, end):
        p, q = (C[-2], C[-1]) if end else (C[0], C[1])
        return math.atan2(q[1] - p[1], q[0] - p[0])
    for k, g in geo.items():
        g["lane_turn"], g["turn"] = g["turn"], 0.0      # 회차 판정은 커넥터만
        if g["junction"] and pred[k] and succ[k]:
            t = max(abs(math.degrees((hd(geo[y]["C"], False) - hd(geo[x]["C"], True) + math.pi) % (2 * math.pi) - math.pi))
                    for x in pred[k] for y in succ[k])
            g["turn"] = t
    uturn = sorted(k for k, g in geo.items() if abs(g["turn"]) >= UTURN_DEG)
    drop = set() if a.keep_uturn else set(uturn)
    print(f"회차 연결로(방위 변화 >= {UTURN_DEG}도) {len(uturn)}, 뺌 {len(drop)}")
    keep = [k for k in sorted(lanes) if k not in drop and len(geo[k]["C"]) >= 2]
    short = [k for k in sorted(lanes) if len(geo[k]["C"]) < 2]
    # 점: 끝점 4개는 union-find 로 공유, 안쪽 점은 고유
    uf = UF()
    for k in keep:
        for e in ("sL", "sR", "eL", "eR"):
            uf.find((k, e))
    kept_edges = [(x, y) for x, y in edges if x in keep and y in keep]
    for x, y in kept_edges:
        uf.union((x, "eL"), (y, "sL"))
        uf.union((x, "eR"), (y, "sR"))
    pos = {}
    for k in keep:
        g = geo[k]
        pos[(k, "sL")], pos[(k, "sR")], pos[(k, "eL")], pos[(k, "eR")] = g["L"][0], g["R"][0], g["L"][-1], g["R"][-1]
    groups = collections.defaultdict(list)
    for n in pos:
        groups[uf.find(n)].append(n)
    gpos, moves = {}, []
    for r, ns in groups.items():
        cx = sum(pos[n][0] for n in ns) / len(ns)
        cy = sum(pos[n][1] for n in ns) / len(ns)
        gpos[r] = (cx, cy)
        for n in ns:
            moves.append((math.hypot(pos[n][0] - cx, pos[n][1] - cy), n))
    moves.sort(reverse=True)
    big = [(round(d, 3), list(n[0]), n[1]) for d, n in moves if d > 0.05]
    print(f"끝점 {len(pos)} -> 공유 점 {len(groups)}, 이동 최대 {moves[0][0]:.3f}m, 0.05m 넘게 이동 {len(big)}")
    # OSM 쓰기
    nid = [1000]
    nodes, ways, rels = [], [], []

    def new_node(x, y):
        nid[0] += 1
        nodes.append((nid[0], x, y))
        return nid[0]

    gid = {r: new_node(*p) for r, p in gpos.items()}
    chord = {}
    if a.chord_fix:
        for kk in json.loads(Path(a.chord_fix).read_text())["inverted"]:
            k = tuple(int(v) for v in kk.split(","))
            if k in geo and k in keep:
                t = sum((math.atan2(q[1] - p[1], q[0] - p[0]) - math.atan2(p[1] - o[1], p[0] - o[0]) + math.pi) % (2 * math.pi) - math.pi
                        for o, p, q in zip(geo[k]["C"], geo[k]["C"][1:], geo[k]["C"][2:]))
                chord[k] = "L" if t > 0 else "R"     # 왼쪽으로 돌면 안쪽은 왼쪽 경계
        print(f"현으로 바꾼 안쪽 경계 {len(chord)}: {sorted((list(k), v) for k, v in chord.items())}")
    ll_of, ll_last, pieces_of, split_log = {}, {}, {}, []
    for k in keep:
        g = geo[k]
        T = {s: taper(g[s], gpos[uf.find((k, "s" + s))], gpos[uf.find((k, "e" + s))]) for s in ("L", "R")}
        if k in chord:
            T[chord[k]] = [T[chord[k]][0], T[chord[k]][-1]]
        n = len(g["L"])
        cuts = {"L": [0, n - 1], "R": [0, n - 1]}
        if not a.no_split and any(flips(T["L"], T["R"])):
            # 나누기 1: 현 바꾸기를 되돌린 원래 점(T0)을 같은 표본 번호에서 자른다. 조각마다 align 판정이 안 뒤집히는 자리를 찾는다
            T0 = {s: taper(g[s], gpos[uf.find((k, "s" + s))], gpos[uf.find((k, "e" + s))]) for s in ("L", "R")}
            found, how = split_cuts(T0, g["C"], a.max_pieces), "원래 점"
            if found:
                T, cuts = T0, {"L": found, "R": found}
            elif k in chord:
                # 나누기 2: 현(v3a 와 같은 점)을 그대로 두고 현 위에 점 1개를 넣어 2조각. 현 직선 위라 모양은 같다
                cs = chord[k]; ot = "R" if cs == "L" else "L"
                r2 = split_chord(T[cs], T[ot], cs, g["C"])
                if r2:
                    c, q = r2
                    T[cs] = [T[cs][0], q, T[cs][1]]
                    cuts = {cs: [0, 1, 2], ot: [0, c, n - 1]}
                    found, how = [0, c, n - 1], f"현 위 점 1개({cs})"
            split_log.append(dict(lane=list(k), n=n, chord=chord.get(k), cuts=found, how=how if found else None,
                                  s=[round(g["s0"] + c * STEP, 1) for c in found] if found else None))
        ids = {}
        for side in ("L", "R"):
            pts = T[side]
            g["T" + side] = pts
            seq = [gid[uf.find((k, "s" + side))]] + [new_node(*p) for p in pts[1:-1]] + [gid[uf.find((k, "e" + side))]]
            ids[side] = []
            for c0, c1 in zip(cuts[side], cuts[side][1:]):
                nid[0] += 1
                ways.append((nid[0], seq[c0:c1 + 1]))
                ids[side].append(nid[0])
        pieces_of[k] = []
        for j, (wl, wr) in enumerate(zip(ids["L"], ids["R"])):
            nid[0] += 1
            rels.append((nid[0], wl, wr, k, j if len(ids["L"]) > 1 else None))
            pieces_of[k].append(nid[0])
        ll_of[k], ll_last[k] = pieces_of[k][0], pieces_of[k][-1]
    if not a.no_split:
        print(f"나눈 lanelet {sum(1 for x in split_log if x['cuts'])}/{len(split_log)}: "
              + "; ".join(f"{x['lane'][0]} 조각 {len(x['cuts']) - 1 if x['cuts'] else '실패'} {x['how'] or ''} s={x['s']}" for x in split_log))
    lines = ['<?xml version="1.0" encoding="UTF-8"?>', '<osm version="0.6" generator="l1_build_lanelet2">']
    for i, x, y in nodes:
        lines.append(f'  <node id="{i}" visible="true" version="1" lat="{y * DEG_PER_M:.11f}" lon="{x * DEG_PER_M:.11f}">')
        lines.append('    <tag k="ele" v="0.000000" />')
        lines.append(f'    <tag k="local_x" v="{x:.6f}" />')
        lines.append(f'    <tag k="local_y" v="{y:.6f}" />')
        lines.append('  </node>')
    for i, seq in ways:
        lines.append(f'  <way id="{i}" visible="true" version="1">')
        lines += [f'    <nd ref="{n}" />' for n in seq]
        lines.append('    <tag k="type" v="line_thin" />')
        lines.append('    <tag k="subtype" v="solid" />')
        lines.append('  </way>')
    for i, l, r, k, piece in rels:
        lines.append(f'  <relation id="{i}" visible="true" version="1">')
        lines.append(f'    <member type="way" ref="{l}" role="left" />')
        lines.append(f'    <member type="way" ref="{r}" role="right" />')
        for kk, vv in (("type", "lanelet"), ("subtype", "road"), ("location", "urban"), ("one_way", "yes"),
                       ("speed_limit", a.speed), ("participant:vehicle", "yes"),
                       ("xodr_road", str(k[0])), ("xodr_section", str(k[1])), ("xodr_lane", str(k[2]))):
            lines.append(f'    <tag k="{kk}" v="{vv}" />')
        if piece is not None:
            lines.append(f'    <tag k="xodr_piece" v="{piece}" />')
        lines.append('  </relation>')
    lines.append('</osm>')
    (out / "lanelet2_map.osm").write_text("\n".join(lines) + "\n")
    (out / "map_projector_info.yaml").write_text("projector_type: Local\n")
    meta = dict(xodr=str(a.xodr), sha256=h, keep_uturn=a.keep_uturn, n_topology=len(topo), n_lanes=len(lanes),
                n_edges=len(edges), dead_end_self=[list(x) for x in selfe], n_kept=len(keep), n_kept_edges=len(kept_edges),
                uturn=[list(k) + [round(geo[k]["turn"], 1)] for k in uturn], dropped=[list(k) for k in sorted(drop)],
                short=[list(k) for k in short],
                lanelet={f"{k[0]},{k[1]},{k[2]}": ll_of[k] for k in keep},
                lanelet_last={f"{k[0]},{k[1]},{k[2]}": ll_last[k] for k in keep},
                pieces={f"{k[0]},{k[1]},{k[2]}": v for k, v in pieces_of.items() if len(v) > 1}, split=split_log,
                lane_len={f"{k[0]},{k[1]},{k[2]}": sum(math.dist(p, q) for p, q in zip(geo[k]["C"], geo[k]["C"][1:])) for k in keep},
                edges=[[f"{x[0]},{x[1]},{x[2]}", f"{y[0]},{y[1]},{y[2]}"] for x, y in sorted(edges)],
                move_max=round(moves[0][0], 4), moved_over_5cm=big[:200], n_moved_over_5cm=len(big),
                chord_fix=[[*k, v] for k, v in sorted(chord.items())], n_nodes=len(nodes), n_ways=len(ways), n_lanelets=len(rels))
    (out / "l1_meta.json").write_text(json.dumps(meta, ensure_ascii=False, indent=1))
    print(f"lanelet {len(rels)}, way {len(ways)}, node {len(nodes)}, 짧아서 뺀 차선 {len(short)}")
    print(f"출력 {out}/lanelet2_map.osm sha256 {sha(out / 'lanelet2_map.osm')[:16]}")


if __name__ == "__main__":
    main()
