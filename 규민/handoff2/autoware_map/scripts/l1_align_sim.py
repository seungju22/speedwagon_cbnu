#!/usr/bin/env python3
# L1: lanelet2 로더의 경계 맞추기(geometry::align) 판정을 파이썬으로 똑같이 따라 한다(2026-10-11)
# 근거: Autoware 1.9.0 이미지 /opt/ros/humble/include/lanelet2_core/geometry/impl/LineString.h (lanelet2 1.2.3)
#   align(left, right): 오른쪽 경계 가운데 점(점 3개 이상이면 ls[size/2], 아니면 양 끝 평균)이 왼쪽 경계의 오른쪽(signedDistance<0)이
#   아니면 왼쪽을 뒤집고, 왼쪽 경계 가운데 점이 오른쪽 경계의 왼쪽(>0)이 아니면 오른쪽을 뒤집는다
#   signedDistance: closestSegment(가장 가까운 선분, 같으면 앞) 에 투영, 부호는 isLeftOf(투영점이 선분 끝점이면 다음 선분도 본다)
# l1_build_lanelet2.py 가 import 해서 쓴다. 단독 실행: python l1_align_sim.py <lanelet2_map.osm> -> 뒤집힐 lanelet 목록
import math
import sys


def _left(a, b, p):
    return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) > 0


def _seg(ls, p):
    best, bi, bq = None, 0, None
    for i in range(len(ls) - 1):
        a, b = ls[i], ls[i + 1]
        dx, dy = b[0] - a[0], b[1] - a[1]
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
        q = (a[0] + t * dx, a[1] + t * dy)
        d = math.hypot(p[0] - q[0], p[1] - q[1])
        if best is None or d < best:
            best, bi, bq = d, i, q
    return best, bi, bq


def signed_distance(ls, p):
    d, i, q = _seg(ls, p)
    a, b = ls[i], ls[i + 1]
    isl = _left(a, b, p)
    if q == b and i + 2 < len(ls):
        n = ls[i + 2]
        if isl != _left(b, n, p) and isl == _left(a, b, n):
            isl = not isl
    return d if isl else -d


def middle(ls):
    if len(ls) > 2:
        return ls[len(ls) // 2]
    return ((ls[0][0] + ls[-1][0]) / 2, (ls[0][1] + ls[-1][1]) / 2)


def flips(left, right):
    """(왼쪽 뒤집힘, 오른쪽 뒤집힘). 둘 다 False 여야 로더가 그대로 읽는다"""
    if len(left) < 2 or len(right) < 2:
        return False, False
    fl = not signed_distance(left, middle(right)) < 0
    if fl:
        left = left[::-1]
    fr = not signed_distance(right, middle(left)) > 0
    return fl, fr


def main():
    import xml.etree.ElementTree as ET
    r = ET.parse(sys.argv[1]).getroot()
    P = {}
    for n in r.iter("node"):
        t = {x.get("k"): x.get("v") for x in n.iter("tag")}
        P[n.get("id")] = (float(t["local_x"]), float(t["local_y"]))
    W = {w.get("id"): [P[x.get("ref")] for x in w.iter("nd")] for w in r.iter("way")}
    bad = []
    for rel in r.iter("relation"):
        mem = {m.get("role"): m.get("ref") for m in rel.iter("member")}
        t = {x.get("k"): x.get("v") for x in rel.iter("tag")}
        f = flips(W[mem["left"]], W[mem["right"]])
        if any(f):
            bad.append((t.get("xodr_road"), t.get("xodr_piece", "-"), f))
    print(f"뒤집힐 lanelet {len(bad)}: {bad}")


if __name__ == "__main__":
    main()
