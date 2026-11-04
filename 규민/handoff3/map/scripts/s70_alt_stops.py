#!/usr/bin/env python3
# 10/25 3단계: 승강장 11곳의 대체 정차점('정류장 막힘' 장면용). 서버 없음, 지도 파일은 읽기만
# 규칙: 같은 차로에서 승강장 뒤쪽(들어오는 쪽)으로 문 점을 D 만큼 옮김. D 는 정차 구역 길이 6.89m 부터 0.25m 씩 늘려
#   아래 검사를 모두 통과하는 가장 가까운 값(최대 D_MAX). 못 찾으면 '없음'과 걸린 검사
#   1) 대체 정차 구역이 원래 정차 구역과 안 겹침(D >= 6.89 이면 저절로, 넓이로 다시 봄)
#   2) 차체가 같은 방향 lanelet(28a57711) 안
#   3) 대체 정차 구역이 v32a 횡단보도 다각형, 교차로 연결로(xodr junction != -1)에 안 걸림
#   4) 문 구역(s56_shuttle.py 와 같은 식: 문 앞뒤 3.0m, 차선 가장자리 바깥 1.5m 에서 시설·차로·건물 뺌)이
#      넓이 2.0m2 이상·문 옆 깊이 0.8m 이상·인도(lane -2) 안 90% 이상. 원래 승강장 문 구역(시설 있으면 0.8 x 3.0 = 2.4m2)과 비슷한 크기
#      빼는 것: 정류장 시설 11곳, 건물 1순위 상자(buildings_layout_v3), 레이어 v2 건물 정육면체(s69_building_rects.json, 1·2순위)
# 차로 따라가기: 갈림길에서는 이 승강장으로 끝나는 30쌍(pairs30)의 road 를 먼저(들어오는 길)
# 출력: map/docs/logs/s70_alt_stops.json, s70_alt_stops.log
# 사용: .venv-carla/bin/python map/scripts/s70_alt_stops.py
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely.strtree import STRtree

HERE = Path(__file__).resolve().parent
BASE = HERE.parent
ROOT = BASE.parent
sys.path.insert(0, str(HERE))
import s56_shuttle as s56  # noqa: E402  (함수·상수만 씀, 파일 안 고침)
import place_campus_layers as pcl  # noqa: E402
from s22_buildings import Frame  # noqa: E402

XODR = BASE / "maps/cbnu_campus_frozen_v31.xodr"
LL31 = ROOT / "autoware_map/cbnu_campus_v31/lanelet2_map.osm"
V32A = BASE / "maps/v32a/lanelet2_map.osm"
PLAT = BASE / "data/shuttle_platforms_v31_r1.yaml"
PAIRS = BASE / "build_v31_s54/pairs30.json"
BRECT = ROOT / "perception/config/s69_building_rects.json"
TREES = BASE / "data/trees_layout_s66.csv"  # 참고만(검사 기준 아님): 문 구역 안 나무 줄기(반지름 0.3m) 수
OUT = BASE / "docs/logs/s70_alt_stops.json"
D_STEP, D_MAX = 0.25, 30.0
DZ_AREA_MIN, DZ_DEPTH_MIN, DZ_SW_MIN = 2.0, 0.8, 0.9


def crosswalks(path):
    r = ET.parse(path).getroot()
    nodes = {}
    for n in r.iter("node"):
        t = {x.get("k"): x.get("v") for x in n.findall("tag")}
        nodes[n.get("id")] = (float(t["local_x"]), float(t["local_y"]))
    out = []
    for w in r.iter("way"):
        t = {x.get("k"): x.get("v") for x in w.findall("tag")}
        if t.get("type") == "crosswalk_polygon":
            out.append((w.get("id"), Polygon([nodes[nd.get("ref")] for nd in w.findall("nd")]).buffer(0)))
    return out


def sq(cx, cy, yaw, side):
    h = side / 2
    return Polygon([s56.off(cx, cy, yaw, a, b) for a, b in ((h, h), (h, -h), (-h, -h), (-h, h))])


def main():
    import carla
    import subprocess
    text = XODR.read_text()
    cmap = carla.Map("OpenDriveMap", text)
    root = ET.fromstring(text)
    junc = {int(r.get("id")) for r in root.iter("road") if r.get("junction", "-1") != "-1"}
    frame = Frame(text)
    plats = json.loads(subprocess.run(["/usr/bin/python3", "-c", "import json,sys,yaml;print(json.dumps(yaml.safe_load(open(sys.argv[1]))))", str(PLAT)],
                                      capture_output=True, text=True, check=True).stdout)["platforms"]  # venv 에 yaml 이 없어 시스템 python3 로 읽음
    pairs = json.loads(PAIRS.read_text())
    quads = [(rid, s56.flip(q)) for rid, q in pcl.lane_quads(cmap, root)]
    qtree = STRtree([q for _, q in quads])
    blds = []
    for r in pcl.read_layout(pcl.LAYOUT):
        cx, cy = frame.carla(float(r["center_lat"]), float(r["center_lon"]))
        blds.append(s56.flip(pcl.rect(cx, cy, float(r["yaw_deg"]), float(r["length_m"]), float(r["width_m"]))))
    br = json.loads(BRECT.read_text())["rects"]
    blds += [sq(b["cx"], b["cy"], b["yaw_deg"], b["side"]) for b in br]
    facs = {p["id"]: Polygon(p["facility"]) for p in plats if p.get("facility")}
    cws = crosswalks(V32A)
    import csv
    trees = [s56.Point(float(r["carla_x"]), -float(r["carla_y"])).buffer(0.3) for r in csv.DictReader(open(TREES))]
    s56.OSM = LL31
    lls = s56.load_lanelets()
    ltree = STRtree([l["poly"] for l in lls])
    log = []

    def P(*a):
        s = " ".join(str(x) for x in a)
        print(s)
        log.append(s)

    P(f"xodr {XODR.name} {s56.h16(XODR)} / lanelet2 {s56.h16(LL31)} / v32a {s56.h16(V32A)} / 승강장 {s56.h16(PLAT)}")
    P(f"나무 {len(trees)}(참고만)")
    P(f"교차로 연결로 {len(junc)}, 횡단보도 {len(cws)}, 건물 상자 {len(blds)}(1순위 상자 + 레이어 정육면체 {len(br)}), 시설 {len(facs)}, lanelet {len(lls)}")

    def same_dir_ll(g, yaw):
        c = g.centroid
        return [lls[j] for j in ltree.query(g.buffer(1.0)) if s56.angdiff(s56.ll_heading(lls[j], c.x, c.y), yaw) < 60]

    def zone(wr, prefer):
        wps = s56.span(wr, s56.REAR - s56.STOP_MARGIN_REAR, s56.FRONT + s56.STOP_MARGIN_FRONT, prefer)
        A, B = [], []
        for w in wps:
            x, y, yaw = s56.mpose(w)
            A.append(s56.off(x, y, yaw, 0, w.lane_width / 2))
            B.append(s56.off(x, y, yaw, 0, -w.lane_width / 2))
        return Polygon(A + B[::-1]).buffer(0), wps

    def door_zone(wd, prefer):
        dz_wps = s56.span(wd, -s56.DOOR_ZONE_ALONG / 2, s56.DOOR_ZONE_ALONG / 2, prefer)
        raw = s56.strip(dz_wps, 0.0, s56.DOOR_ZONE_DEPTH, half=True)
        near_q = [quads[j][1] for j in qtree.query(raw.buffer(0.5))]
        cut = near_q + [f for f in facs.values() if f.intersects(raw)] + [b for b in blds if b.intersects(raw)]
        raw_bld = sum(raw.intersection(b).area for b in blds if b.intersects(raw))
        raw_fac = sum(raw.intersection(f).area for f in facs.values() if f.intersects(raw))
        dz = raw.difference(unary_union(cut)) if cut else raw
        if dz.geom_type == "MultiPolygon":
            dz = max(dz.geoms, key=lambda g: g.area)
        ray = s56.strip(s56.span(wd, -0.05, 0.05, prefer), 0.0, s56.DOOR_ZONE_DEPTH, half=True)
        depth = dz.intersection(ray).area / 0.1
        sw_wps = []
        for w in dz_wps:
            sw = cmap.get_waypoint_xodr(w.road_id, -2, w.s)
            if sw is not None and sw.lane_type == carla.LaneType.Sidewalk:
                sw_wps.append(sw)
        swp = s56.strip(sw_wps, -sw_wps[0].lane_width / 2, sw_wps[0].lane_width / 2) if len(sw_wps) > 1 else None
        swf = dz.intersection(swp).area / dz.area if (swp is not None and dz.area > 0) else 0.0
        return dz, dict(area=round(dz.area, 3), depth=round(depth, 3), sidewalk_frac=round(swf, 3),
                        raw_bld_m2=round(raw_bld, 3), raw_fac_m2=round(raw_fac, 3))

    res = []
    for p in plats:
        pid = p["id"]
        rd, s, lane = p["stops_v3"]["road_id"], p["stops_v3"]["s"], p["stops_v3"]["lane_id"]
        wp_p = cmap.get_waypoint_xodr(rd, lane, s)
        inc = [n for n, v in pairs.items() if v["roads"][-1] == rd and abs(v["end_s"] - s) < 0.5]
        prefer = {r for n in inc for r in pairs[n]["roads"]} | {rd}
        wr0 = s56.walk(wp_p, s56.DOOR_X, prefer, back=True)[0]
        sz0, _ = zone(wr0, prefer)
        rec = dict(id=pid, name=p["name"], side=p["side"], incoming_pairs=inc, stoppable=p.get("stoppable", True))
        if p.get("stoppable", True) is False:
            rec.update(status="없음", reason="원래 승강장이 정차 불가(문 구역 없음, 결정 1). 대체 정차점도 두지 않음")
            res.append(rec)
            P(f"{pid}: 없음(원래 정차 불가)")
            continue
        D, found, fails = s56.STOP_MARGIN_FRONT + s56.STOP_MARGIN_REAR + s56.FRONT - s56.REAR, None, {}
        while D <= D_MAX + 1e-6:
            back = s56.walk(wp_p, D, prefer, back=True)
            if len(back) < int(round(D / s56.STEP)) + 1:
                fails["차로 끝"] = fails.get("차로 끝", 0) + 1
                break
            wd = back[0]
            wr = s56.walk(wd, s56.DOOR_X, prefer, back=True)[0]
            bx, by, byaw = s56.mpose(wr)
            dx, dy = s56.off(bx, by, byaw, s56.DOOR_X, s56.DOOR_Y)
            sz, sz_wps = zone(wr, prefer)
            why = []
            if sz.intersection(sz0).area > 1e-3:
                why.append("원래 정차 구역과 겹침")
            bd = s56.body(bx, by, byaw)
            bll = same_dir_ll(bd, byaw)
            b_out = bd.difference(unary_union([l["poly"] for l in bll]).buffer(0.01)).area if bll else bd.area
            if b_out > 0.01:
                why.append("차체 lanelet 밖")
            jr = sorted({w.road_id for w in sz_wps} & junc)
            if jr:
                why.append("교차로 연결로")
            cw = [cid for cid, g in cws if g.intersects(sz)]
            if cw:
                why.append("횡단보도")
            dz, dzi = door_zone(wd, prefer)
            if dzi["area"] < DZ_AREA_MIN or dzi["depth"] < DZ_DEPTH_MIN or dzi["sidewalk_frac"] < DZ_SW_MIN:
                why.append("문 구역 좁음/인도 밖")
            if not why:
                q = (round(math.sin(math.radians(byaw) / 2), 6), round(math.cos(math.radians(byaw) / 2), 6))
                found = dict(shift_back_m=round(D, 2),
                             door_pose=dict(x=round(dx, 3), y=round(dy, 3), yaw_deg=round(byaw, 2)),
                             base_link_goal=dict(x=round(bx, 3), y=round(by, 3), z=0.0, yaw_deg=round(byaw, 2), qz=q[0], qw=q[1]),
                             base_link_ref=dict(road_id=wr.road_id, s=round(wr.s, 2), lanelet_id=sorted({l["id"] for l in bll if l["poly"].contains(s56.Point(bx, by))})),
                             stop_zone=s56.pts(sz), door_zone=s56.pts(dz), door_zone_info=dzi,
                             front_corridor=s56.pts(s56.strip(s56.span(wr, s56.FRONT, s56.FRONT + s56.CORRIDOR_LEN, prefer),
                                                              s56.CORRIDOR_W / 2, -s56.CORRIDOR_W / 2)),  # 앞 8m 는 원래 정차 구역(막은 것) 쪽
                             body=s56.pts(bd, 3), body_out_of_lanelets_m2=round(b_out, 4),
                             stop_zone_roads=sorted({w.road_id for w in sz_wps}),
                             gap_to_original_stop_zone_m=round(sz.distance(sz0), 2),
                             trees_in_door_zone=sum(1 for t in trees if t.intersects(dz)), trees_in_stop_zone=sum(1 for t in trees if t.intersects(sz)))
                break
            for w in why:
                fails[w] = fails.get(w, 0) + 1
            D += D_STEP
        rec.update(original_stop_zone=s56.pts(sz0), original_base_link_goal=p["base_link_goal"], fails_before=fails)
        if found:
            rec.update(status="있음", alt=found)
            P(f"{pid}: 있음 D {found['shift_back_m']} r{found['base_link_ref']['road_id']} s{found['base_link_ref']['s']} "
              f"문구역 {found['door_zone_info']} 나무 문{found['trees_in_door_zone']}·정차{found['trees_in_stop_zone']} 차체밖 {found['body_out_of_lanelets_m2']} 원래와 틈 {found['gap_to_original_stop_zone_m']} 앞에서 걸린 것 {fails}")
        else:
            rec.update(status="없음", reason=f"{D_MAX}m 안에서 못 찾음: {fails}")
            P(f"{pid}: 없음 {fails}")
        res.append(rec)
    OUT.write_text(json.dumps(dict(xodr_sha16=s56.h16(XODR), lanelet2_sha16=s56.h16(LL31), v32a_sha16=s56.h16(V32A),
                                   platforms_sha16=s56.h16(PLAT), rules=dict(d_step=D_STEP, d_max=D_MAX, dz_area_min=DZ_AREA_MIN,
                                   dz_depth_min=DZ_DEPTH_MIN, dz_sidewalk_min=DZ_SW_MIN), platforms=res), ensure_ascii=False, indent=1))
    (BASE / "docs/logs/s70_alt_stops.log").write_text("\n".join(log) + "\n")
    P(f"-> {OUT}")


if __name__ == "__main__":
    main()
