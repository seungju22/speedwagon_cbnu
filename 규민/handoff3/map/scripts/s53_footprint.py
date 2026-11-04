#!/usr/bin/env python3
# 10/11 3번: Autoware 기본 차 차체를 경로 lanelet 중심선을 따라 옮겨 경로 lanelet 안에 들어가는지(서버 없음)
# path_optimizer 'outside drivable area' 정지를 미리 거르는 근사다
# - 차체: 1.9.0 이미지 autoware_sample_vehicle_description vehicle_info.param.yaml(읽기만):
#   wheel_base 2.79, front_overhang 1.0, rear_overhang 1.1, wheel_tread 1.64, left/right_overhang 0.128
#   base_link = 뒷바퀴 축 가운데 -> 모서리 (x 3.79 | -1.1, y ±0.948)
# - 주행 가능 영역 = 그 쌍 경로의 lanelet 다각형 합(옆 차선·반대 차선 안 씀)
# - 경로 중심선 = lanelet 왼/오 경계를 같은 비율로 다시 표본해 가운데(0.5m 간격). 첫 lanelet 은 start_s, 끝은 end_s 비율로 자름
# - 판정: 중심선 점마다 옆으로 e(-1.0~+1.0m, 0.05m 간격, 방위는 중심선 방위)만큼 옮겨 네 모서리가 모두 영역 안인 e 가 있으면 통과.
#   path_optimizer 가 경로를 옆으로 옮겨 풀 수 있는 정도를 흉내낸 것(차체 방위 최적화는 안 함)
# 사용: .venv-carla/bin/python map/scripts/s53_footprint.py <lanelet2_map.osm> <pairs.json> <tag> [쌍 ...]
import json
import math
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

from shapely.geometry import Point, Polygon
from shapely.ops import unary_union
from shapely.prepared import prep

BASE = Path(__file__).resolve().parent.parent
CORNERS = [(3.79, 0.948), (3.79, -0.948), (-1.1, 0.948), (-1.1, -0.948)]
MIN_START = 1.2   # 10/13: 출발 s 하한. 옛 5노선은 r1278 s=0(맵 가장자리)이라 차 뒤 1.1m 가 첫 레인렛 밖이 된다(검사 기준 바꿈, 30쌍 출발은 15m 이상이라 영향 없음)
OFFS = sorted([round(-1.0 + 0.05 * i, 2) for i in range(41)], key=abs)


def load(osm):
    r = ET.parse(osm).getroot()
    nodes = {}
    for n in r.iter("node"):
        t = {x.get("k"): x.get("v") for x in n.findall("tag")}
        nodes[n.get("id")] = (float(t["local_x"]), float(t["local_y"]))
    ways = {w.get("id"): [nd.get("ref") for nd in w.findall("nd")] for w in r.iter("way")}
    lls = defaultdict(list)
    for rel in r.iter("relation"):
        t = {x.get("k"): x.get("v") for x in rel.findall("tag")}
        if t.get("type") != "lanelet":
            continue
        mem = {m.get("role"): m.get("ref") for m in rel.findall("member")}
        L = [nodes[q] for q in ways[mem["left"]]]
        R = [nodes[q] for q in ways[mem["right"]]]
        lls[int(t["xodr_road"])].append((int(t.get("xodr_piece", 0)), L, R))
    for k in lls:
        lls[k].sort()
    return lls


def resample(pts, n):
    d = [0.0]
    for a, b in zip(pts, pts[1:]):
        d.append(d[-1] + math.dist(a, b))
    tot = d[-1]
    out = []
    j = 0
    for i in range(n):
        t = tot * i / (n - 1)
        while j < len(d) - 2 and d[j + 1] < t:
            j += 1
        seg = d[j + 1] - d[j]
        u = (t - d[j]) / seg if seg > 0 else 0
        out.append((pts[j][0] + u * (pts[j + 1][0] - pts[j][0]), pts[j][1] + u * (pts[j + 1][1] - pts[j][1])))
    return out, tot


def check(lls, pair):
    polys, center, owner = [], [], []
    rs = pair["roads"]
    for k, rid in enumerate(rs):
        for piece, L, R in lls.get(rid, []):
            polys.append(Polygon(L + R[::-1]).buffer(0))
        segs = lls.get(rid, [])
        cl = []
        for piece, L, R in segs:
            lenL = sum(math.dist(a, b) for a, b in zip(L, L[1:]))
            n = max(3, int(lenL / 0.5) + 1)
            a, _ = resample(L, n)
            b, _ = resample(R, n)
            cl += [((p[0] + q[0]) / 2, (p[1] + q[1]) / 2) for p, q in zip(a, b)]
        if not cl:
            continue
        # 첫·끝 road 자르기(길이 비율)
        tot = sum(math.dist(a, b) for a, b in zip(cl, cl[1:])) or 1
        acc = [0.0]
        for a, b in zip(cl, cl[1:]):
            acc.append(acc[-1] + math.dist(a, b))
        lo = max(pair["start_s"], MIN_START) if k == 0 else -1
        hi = pair["end_s"] if k == len(rs) - 1 else math.inf
        xl = pair.get("_len", {}).get(rid)
        sc = tot / xl if xl else 1.0
        for p, s in zip(cl, acc):
            if (k == 0 and s < lo * sc) or (k == len(rs) - 1 and s > hi * sc):
                continue
            if center and math.dist(center[-1], p) < 0.25:
                continue
            center.append(p)
            owner.append(rid)
    area = unary_union([p.buffer(0.02) for p in polys]).buffer(-0.02)
    pa = prep(area)
    bad = []
    emax = 0.0
    for i, p in enumerate(center):
        a = center[max(0, i - 2)]
        b = center[min(len(center) - 1, i + 2)]
        h = math.atan2(b[1] - a[1], b[0] - a[0])
        c, s = math.cos(h), math.sin(h)
        ok = None
        for e in OFFS:
            bx, by = p[0] - e * s, p[1] + e * c
            if all(pa.contains(Point(bx + x * c - y * s, by + x * s + y * c)) for x, y in CORNERS):
                ok = e
                break
        if ok is None:
            bad.append((owner[i], round(p[0], 2), round(p[1], 2)))
        else:
            emax = max(emax, abs(ok))
    return len(center), bad, emax


def main():
    osm, pj, tag = sys.argv[1], sys.argv[2], sys.argv[3]
    want = sys.argv[4:]
    lls = load(osm)
    pairs = json.loads(Path(pj).read_text())
    out = []
    tot_bad = 0
    for name, p in pairs.items():
        if want and name not in want:
            continue
        n, bad, emax = check(lls, p)
        tot_bad += bool(bad)
        where = defaultdict(list)
        for rid, x, y in bad:
            where[rid].append((x, y))
        s = f"{name}: 점 {n}, 못 들어감 {len(bad)}, 필요한 옆 이동 최대 {emax:.2f}m"
        if bad:
            s += " | " + "; ".join(f"r{rid} {len(v)}점 첫 ({v[0][0]}, {v[0][1]})" for rid, v in where.items())
        out.append(s)
        print(s, flush=True)
    out.append(f"못 들어가는 점이 있는 쌍 {tot_bad}/{len(out)}")
    print(out[-1])
    (BASE / f"docs/logs/s53_footprint_{tag}.log").write_text(f"lanelet {osm}\n" + "\n".join(out) + "\n")


if __name__ == "__main__":
    main()
