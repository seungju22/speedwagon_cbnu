#!/usr/bin/env python3
# 맵세션27: 회차 없는 막다른 끝 탐지·채움 후처리 (설계 s23_deadend_cause.md 4절 + 사용자 수정 지시)
# 원칙: 막다른 끝 목록·개수를 코드에 넣지 않는다. 입력 xodr 를 읽어 스스로 찾는다
#      (회차 생성은 망 전체에 따라 달라짐, s23 실험 E0~E2).
# 모드
#   scan <xodr>                 읽기전용. 정의 없는 junction(회차 없는 막다른 끝), 일방통행 막다른 끝,
#                                기존 회차 연결로(틀) 와 "들어오는 차선 끝 기준 상대 위치" 를 출력
#   write <입력 xodr> <출력 xodr>  정의 없는 junction 마다 기존 틀 복제 커넥터 road + junction 정의 추가.
#                                출력은 새 파일만. 입력·frozen_v1 경로로는 쓰지 않는다
#   verify <출력 xodr>           읽기전용. 추가한 커넥터가 들어가는 반쪽 -> 커넥터 -> 나가는 반쪽 으로 next() 이어지는지
# 상태: scan 은 세션27 에 frozen_v1 로 검증. write/verify 는 세션27 에 실행하지 않음(새 xodr 생성 금지) = 미검증
import copy
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import carla

FROZEN = Path("/home/gyumin/campus_mobility_sim/map/maps/cbnu_internal_only_localtm_tags73_smooth.xodr")
TPL_LEN = 9.89240086          # 기존 회차 연결로 길이(s23: 24개 전부 같음)
TPL_TOL = 1e-3
GAP_MAX = 0.05                # 들어가는 반쪽 끝 - 나가는 반쪽 시작 거리 허용(m). s23 실측 0.00
ANG_TOL = 1.0                 # 180도 에서 허용 차(도)
NAME_PREFIX = ":turnfix_"


def links(r):
    out = {}
    for k in ("predecessor", "successor"):
        e = r.find(f"link/{k}")
        if e is not None:
            out[k] = (e.get("elementType"), int(e.get("elementId")))
    return out


def lane_ids(r):
    return {int(l.get("id")) for l in r.iter("lane")}


def lane_end_pose(cmap, rid, at_end):
    """lane -1 끝(또는 시작) 차선 중심 자세를 xodr 좌표계(x, y, hdg rad)로."""
    L = cmap_len[rid]
    wp = cmap.get_waypoint_xodr(rid, -1, max(L - 0.001, 0.0) if at_end else 0.001)
    if wp is None:
        return None
    t = wp.transform
    return t.location.x, -t.location.y, math.radians(-t.rotation.yaw)


def rel(pose, x, y, h):
    px, py, ph = pose
    dx, dy = x - px, y - py
    c, s = math.cos(-ph), math.sin(-ph)
    return dx * c - dy * s, dx * s + dy * c, (h - ph + math.pi) % (2 * math.pi) - math.pi


def apply_rel(pose, rx, ry, rh):
    px, py, ph = pose
    c, s = math.cos(ph), math.sin(ph)
    return px + rx * c - ry * s, py + rx * s + ry * c, (ph + rh + math.pi) % (2 * math.pi) - math.pi


cmap_len = {}


def scan(path, quiet=False):
    text = Path(path).read_text()
    root = ET.fromstring(text.encode())
    roads = {int(r.get("id")): r for r in root.iter("road")}
    juncs = {int(j.get("id")): j for j in root.iter("junction")}
    cmap_len.clear()
    cmap_len.update({k: float(r.get("length")) for k, r in roads.items()})
    cmap = carla.Map(Path(path).stem, text)
    refs = {}
    for rid, r in roads.items():
        if int(r.get("junction")) != -1:
            continue
        for k, (t, eid) in links(r).items():
            if t == "junction":
                refs.setdefault(eid, {"predecessor": [], "successor": []})[k].append(rid)
    undefined = sorted(set(refs) - set(juncs))
    # 기존 회차 연결로(틀): connection 1개 junction 의 커넥터가 paramPoly3 이고 길이 TPL_LEN
    tpl = []
    for jid, j in juncs.items():
        cs = j.findall("connection")
        if len(cs) != 1:
            continue
        cr = roads[int(cs[0].get("connectingRoad"))]
        g = cr.find("planView/geometry")
        if g.find("paramPoly3") is None or abs(float(cr.get("length")) - TPL_LEN) > TPL_TOL:
            continue
        inc = int(cs[0].get("incomingRoad"))
        pose = lane_end_pose(cmap, inc, True)
        tpl.append((jid, int(cr.get("id")), inc, rel(pose, float(g.get("x")), float(g.get("y")), float(g.get("hdg")))))
    # 정의 없는 junction 의 쌍 검사
    cand = []
    for jid in undefined:
        ins, outs = refs[jid]["successor"], refs[jid]["predecessor"]
        ok, note = False, ""
        if len(ins) == 1 and len(outs) == 1:
            a = lane_end_pose(cmap, ins[0], True)
            b = lane_end_pose(cmap, outs[0], False)
            if a is None or b is None:
                note = "waypoint 없음"
            else:
                gap = math.hypot(a[0] - b[0], a[1] - b[1])
                ang = abs(math.degrees((b[2] - a[2]) % (2 * math.pi)) - 180)
                ok = gap <= 3.35 + GAP_MAX and ang <= ANG_TOL   # 차선 중심끼리는 차선 폭만큼 떨어짐
                note = f"차선중심 간격 {gap:.2f}m 방향차 180{'+' if ang >= 0 else ''}{ang:.2f}도"
        else:
            note = f"참조 road 수 들어감 {len(ins)} 나옴 {len(outs)}"
        cand.append((jid, ins, outs, ok, note))
    # 일방통행 막다른 끝: 일반 road 인데 successor 링크 없음
    oneway_dead = [rid for rid, r in roads.items()
                   if int(r.get("junction")) == -1 and "successor" not in links(r)]
    if not quiet:
        print(f"xodr {Path(path).name}: road {len(roads)} / 정의된 junction {len(juncs)}")
        print(f"정의 없는 junction(회차 없는 막다른 끝) {len(undefined)}: {undefined}")
        for jid, ins, outs, ok, note in cand:
            print(f"  j{jid} 들어감 {ins} 나옴 {outs} 채움가능={ok} ({note})")
        print(f"successor 없는 일반 road(일방통행 막다른 끝 후보) {len(oneway_dead)}: {sorted(oneway_dead)}")
        print(f"기존 회차 연결로(틀, connection 1개·paramPoly3·길이 {TPL_LEN}) {len(tpl)}개")
        if tpl:
            rs = [t[3] for t in tpl]
            for i, nm in enumerate(("앞(m)", "왼(m)", "방위(rad)")):
                v = [r[i] for r in rs]
                print(f"  들어오는 차선 끝 기준 상대 {nm}: 최소 {min(v):.4f} 최대 {max(v):.4f}")
    return text, roads, juncs, cmap, cand, tpl


def write(src, dst):
    src, dst = Path(src).resolve(), Path(dst).resolve()
    if dst == src or dst == FROZEN.resolve() or dst.exists():
        sys.exit(f"출력 거부: {dst} (입력·frozen_v1 이거나 이미 있음)")
    text, roads, juncs, cmap, cand, tpl = scan(src)
    if not tpl:
        sys.exit("틀 커넥터 0개: 채우지 않음")
    rs = [t[3] for t in tpl]
    spread = max(max(r[i] for r in rs) - min(r[i] for r in rs) for i in range(3))
    if spread > 0.01:
        sys.exit(f"틀 상대 위치가 일정하지 않음(최대 폭 {spread:.4f}): 복제 근거 깨짐, 중단")
    rx, ry, rh = (sum(r[i] for r in rs) / len(rs) for i in range(3))
    tpl_road = roads[tpl[0][1]]
    next_id = max(roads) + 1
    new_roads, new_juncs, log = [], [], []
    for jid, ins, outs, ok, note in cand:
        if not ok:
            log.append(f"건너뜀 j{jid}: {note}")
            continue
        inc, out = ins[0], outs[0]
        pose = lane_end_pose(cmap, inc, True)
        x, y, h = apply_rel(pose, rx, ry, rh)
        r = copy.deepcopy(tpl_road)
        r.set("id", str(next_id))
        r.set("junction", str(jid))
        r.set("name", f"{NAME_PREFIX}{jid}_0")
        r.find("link/predecessor").set("elementId", str(inc))
        r.find("link/successor").set("elementId", str(out))
        g = r.find("planView/geometry")
        g.set("x", f"{x:.8f}")
        g.set("y", f"{y:.8f}")
        g.set("hdg", f"{h:.8f}")
        sidewalk = -2 in lane_ids(roads[inc]) and -2 in lane_ids(roads[out])
        if not sidewalk:
            right = r.find("lanes/laneSection/right")
            for ln in list(right):
                if ln.get("id") == "-2":
                    right.remove(ln)
        j = ET.Element("junction", {"name": f"{NAME_PREFIX}{jid}", "id": str(jid)})
        c = ET.SubElement(j, "connection", {"id": "0", "incomingRoad": str(inc),
                                            "connectingRoad": str(next_id), "contactPoint": "start"})
        ET.SubElement(c, "laneLink", {"from": "-1", "to": "-1"})
        if sidewalk:
            ET.SubElement(c, "laneLink", {"from": "-2", "to": "-2"})
        new_roads.append(ET.tostring(r, encoding="unicode"))
        new_juncs.append(ET.tostring(j, encoding="unicode"))
        log.append(f"추가 j{jid}: road{next_id} {inc}->{out} 인도={sidewalk}")
        next_id += 1
    # 원문 텍스트를 보존하고 삽입만 한다(머리말 주석·서식 유지)
    k = text.index("<junction ")
    k = text.rfind("\n", 0, k) + 1
    out_text = text[:k] + "".join("    " + s.strip() + "\n" for s in new_roads) + text[k:]
    e = out_text.rindex("</OpenDRIVE>")
    out_text = out_text[:e] + "".join("    " + s.strip() + "\n" for s in new_juncs) + out_text[e:]
    dst.write_text(out_text)
    print("\n".join(log))
    print(f"출력 {dst}: 커넥터 {len(new_roads)} 추가, 건너뜀 {len(cand) - len(new_roads)}")


def verify(path):
    text = Path(path).read_text()
    root = ET.fromstring(text.encode())
    roads = {int(r.get("id")): r for r in root.iter("road")}
    cmap_len.clear()
    cmap_len.update({k: float(r.get("length")) for k, r in roads.items()})
    cmap = carla.Map(Path(path).stem, text)
    bad = 0
    added = [r for r in roads.values() if (r.get("name") or "").startswith(NAME_PREFIX)]
    for r in added:
        rid = int(r.get("id"))
        lk = links(r)
        inc, out = lk["predecessor"][1], lk["successor"][1]
        wp = cmap.get_waypoint_xodr(inc, -1, cmap_len[inc] - 0.02)
        hit = {n.road_id for n in wp.next(0.3)} if wp else set()
        w2 = cmap.get_waypoint_xodr(rid, -1, cmap_len[rid] - 0.02)
        hit2 = {n.road_id for n in w2.next(0.3)} if w2 else set()
        ok = rid in hit and out in hit2
        bad += not ok
        print(f"road{rid} j{r.get('junction')}: {inc} -> {sorted(hit)} / {rid} -> {sorted(hit2)} 기대 {out} => {'통과' if ok else '실패'}")
    print(f"추가 커넥터 {len(added)}, 실패 {bad}")
    return bad == 0


if __name__ == "__main__":
    a = sys.argv[1:]
    if a[:1] == ["scan"] and len(a) == 2:
        scan(a[1])
    elif a[:1] == ["write"] and len(a) == 3:
        write(a[1], a[2])
    elif a[:1] == ["verify"] and len(a) == 2:
        sys.exit(0 if verify(a[1]) else 1)
    else:
        sys.exit("사용: scan <xodr> | write <입력> <출력> | verify <출력>")
