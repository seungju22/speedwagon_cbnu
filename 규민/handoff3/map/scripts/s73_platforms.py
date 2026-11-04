#!/usr/bin/env python3
# 11/02 3단계: v3.3a 새 승강장(10/12 검사 기준, s70_alt_stops.py 와 같은 식). 서버 없음, 지도 파일은 읽기만
#   개성재(middle_gate_s·middle_gate_n 둘 다 같은 자리, 화면 이름 '개성재'): 대각선 길 r1316(북동행), 개성재 게이트 G5 쪽 끝
#   남문(이쪽) lib_south_se: r1268 s=47.0 그대로, footway r1235·r1369 를 차로에서 빼고 다시 셈
#   정문 안쪽(순환용) 두 방향 r1410·r1272, 후문 안쪽(순환용) 두 방향 r1251·r1384, 사범대 앞(교대 학생 하차, 선택) r1415 모퉁이 뒤
# 차 치수: 기본 sample_vehicle(지금 기본값). --veh starex 이면 스타렉스 치수로 문 위치·차체(내일 시험 뒤 기본값을 바꿈)
# 검사: 문 구역 2.0m2 이상·문 옆 깊이 0.8m 이상·인도 안 90% 이상, 전방 통로 8m 가 같은 방향 레인렛 안, 차체가 레인렛(v3.3a) 안, 정차 구역이 교차로 연결로·v32a 횡단보도에 안 걸림
# 출력 integration/logs/s73/platforms_<veh>.json
import argparse, json, math, subprocess
from pathlib import Path
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union
from shapely.strtree import STRtree
import s72_common as K
import s56_shuttle as s56
import s70_alt_stops as s70
import place_campus_layers as pcl

ap = argparse.ArgumentParser(); ap.add_argument("--veh", default="sample"); ap.add_argument("--osm", default=str(K.C / "map/maps/v33a/lanelet2_map.osm"))
a = ap.parse_args()
OUT = K.C / "integration/logs/s73"
if a.veh == "starex":   # 스타렉스: 축거 3.2, 앞뒤 오버행 0.877/1.073(가정), 폭 1.92, 윤거 뒤 1.66
    s56.WB, s56.FO, s56.RO, s56.TREAD = 3.2, 0.877, 1.073, 1.66
    s56.LO = s56.RGO = (1.92 - 1.66) / 2
    s56.FRONT, s56.REAR = s56.WB + s56.FO, -s56.RO
    s56.HALF_W = 1.92 / 2; s56.DOOR_X = (s56.FRONT + s56.REAR) / 2; s56.DOOR_Y = -s56.HALF_W; s56.CORRIDOR_W = 1.92 + 0.6
DROP_Q = {1235, 1369}   # 차로 조각에서 뺄 road(footway)
# 승강장: id, 화면 이름, 쪽, road, 찾을 s 범위, 가까이 둘 목표점(지도 좌표), 들어오는 길(갈림길에서 먼저 따라갈 road)
TARGETS = [
    ("middle_gate_s", "개성재", "게이트 앞(G5 쪽)", 1316, (40, 72), (620, 925), {1316, 1315}),
    ("lib_south_se", "도서관 남문", "이쪽", 1268, (47, 47), None, {1268, 1601, 1269}),
    ("main_gate_inner_se", "정문 안쪽(순환)", "동남행", 1410, (8, 52), (370, 889), {1410, 1485, 1392}),
    ("main_gate_inner_nw", "정문 안쪽(순환)", "북서행", 1272, (8, 52), (372, 891), {1272, 1478}),
    ("back_gate_inner_se", "후문 안쪽(순환)", "동남행", 1251, (60, 130), (965, 285), {1251, 1860}),
    ("back_gate_inner_nw", "후문 안쪽(순환)", "북서행", 1384, (8, 80), (968, 287), {1384, 1946}),
    ("edu_front", "사범대 앞(교대)", "북문 길 끝 쪽", 1415, (165, 205), (800, 660), {1415}),
]


def main():
    cmap = K.cmap(); root = K.ROOT
    quads = [(rid, s56.flip(q)) for rid, q in pcl.lane_quads(cmap, root) if rid not in DROP_Q]
    qtree = STRtree([q for _, q in quads])
    blds = []
    for r in pcl.read_layout(pcl.LAYOUT):
        cx, cy = K.FR.carla(float(r["center_lat"]), float(r["center_lon"]))
        blds.append(s56.flip(pcl.rect(cx, cy, float(r["yaw_deg"]), float(r["length_m"]), float(r["width_m"]))))
    blds += [s70.sq(b["cx"], b["cy"], b["yaw_deg"], b["side"]) for b in json.loads(s70.BRECT.read_text())["rects"]]
    PL = json.loads(subprocess.check_output(["/usr/bin/python3", "-c", "import sys,json,yaml;print(json.dumps(yaml.safe_load(open(sys.argv[1]))))",
                                             str(K.BASE / "data/shuttle_platforms_v31_r2.yaml")]))["platforms"]
    facs = [Polygon(p["facility"]) for p in PL if p.get("facility")]
    cws = [g for _, g in s70.crosswalks(s70.V32A)]
    s56.OSM = Path(a.osm)
    lls = s56.load_lanelets(); ltree = STRtree([l["poly"] for l in lls])

    def same_dir(g, yaw):
        c = g.centroid
        return [lls[j] for j in ltree.query(g.buffer(1.0)) if s56.angdiff(s56.ll_heading(lls[j], c.x, c.y), yaw) < 60]

    res = {}
    for pid, name, side, rid, (s0, s1), tgt, prefer in TARGETS:
        rows = []
        s = float(s0)
        while s <= s1 + 1e-6:
            wp = cmap.get_waypoint_xodr(rid, -1, s)
            if wp is None: s += 2; continue
            wr = s56.walk(wp, s56.DOOR_X, prefer, back=True)[0]
            bx, by, byaw = s56.mpose(wr); px, py, pyaw = s56.mpose(wp)
            dx, dy = s56.off(bx, by, byaw, s56.DOOR_X, s56.DOOR_Y)
            sz_wps = s56.span(wr, s56.REAR - s56.STOP_MARGIN_REAR, s56.FRONT + s56.STOP_MARGIN_FRONT, prefer)
            A_, B_ = [], []
            for w in sz_wps:
                x, y, yaw = s56.mpose(w); A_.append(s56.off(x, y, yaw, 0, w.lane_width / 2)); B_.append(s56.off(x, y, yaw, 0, -w.lane_width / 2))
            sz = Polygon(A_ + B_[::-1]).buffer(0)
            dz_wps = s56.span(wp, -s56.DOOR_ZONE_ALONG / 2, s56.DOOR_ZONE_ALONG / 2, prefer)
            raw = s56.strip(dz_wps, 0.0, s56.DOOR_ZONE_DEPTH, half=True)
            cut = [quads[j][1] for j in qtree.query(raw.buffer(0.5))] + [f for f in facs if f.intersects(raw)] + [b for b in blds if b.intersects(raw)]
            dz = raw.difference(unary_union(cut)) if cut else raw
            if dz.geom_type == "MultiPolygon": dz = max(dz.geoms, key=lambda g: g.area)
            ray = s56.strip(s56.span(wp, -0.05, 0.05, prefer), 0.0, s56.DOOR_ZONE_DEPTH, half=True)
            depth = dz.intersection(ray).area / 0.1
            sw_wps = [x for x in (cmap.get_waypoint_xodr(w.road_id, -2, w.s) for w in dz_wps) if x is not None and x.lane_type == __import__("carla").LaneType.Sidewalk]
            swp = s56.strip(sw_wps, -sw_wps[0].lane_width / 2, sw_wps[0].lane_width / 2) if len(sw_wps) > 1 else None
            swf = dz.intersection(swp).area / dz.area if (swp is not None and dz.area > 0) else 0.0
            bd = s56.body(bx, by, byaw); bll = same_dir(bd, byaw)
            b_out = bd.difference(unary_union([l["poly"] for l in bll]).buffer(0.01)).area if bll else bd.area
            cw = s56.span(wr, s56.FRONT, s56.FRONT + s56.CORRIDOR_LEN, prefer)
            corr = s56.strip(cw, s56.CORRIDOR_W / 2, -s56.CORRIDOR_W / 2) if len(cw) > 1 else Polygon()
            cl = same_dir(corr, byaw) if not corr.is_empty else []
            corr_out = corr.difference(unary_union([l["poly"] for l in cl]).buffer(0.01)).area if cl else corr.area
            junc = sorted({w.road_id for w in sz_wps if K.RJUNC.get(w.road_id, -1) != -1})
            cwk = [i for i, g in enumerate(cws) if g.intersects(sz)]
            ok = dz.area >= 2.0 and depth >= 0.8 and swf >= 0.9 and b_out <= 0.01 and corr_out <= 0.5 and not junc and not cwk
            q = (round(math.sin(math.radians(byaw) / 2), 6), round(math.cos(math.radians(byaw) / 2), 6))
            rows.append(dict(s=round(s, 1), ok=ok, why=[k for k, v in (("문 구역", dz.area < 2.0 or depth < 0.8), ("인도 밖", swf < 0.9), ("차체", b_out > 0.01),
                                                                        ("통로", corr_out > 0.5), ("교차로", bool(junc)), ("횡단보도", bool(cwk))) if v],
                             marker=dict(x=round(px, 3), y=round(py, 3), yaw_deg=round(pyaw, 2)),
                             door_pose=dict(x=round(dx, 3), y=round(dy, 3), yaw_deg=round(byaw, 2)),
                             base_link_goal=dict(x=round(bx, 3), y=round(by, 3), z=0.0, yaw_deg=round(byaw, 2), qz=q[0], qw=q[1]),
                             base_link_ref=dict(road_id=wr.road_id, s=round(wr.s, 2), lanelet_id=sorted({l["id"] for l in bll if l["poly"].contains(Point(bx, by))})),
                             stop_zone=s56.pts(sz), door_zone=s56.pts(dz), front_corridor=s56.pts(corr), body=s56.pts(bd, 3),
                             check=dict(door_zone_area_m2=round(dz.area, 3), depth_at_door_m=round(depth, 3), sidewalk_frac=round(swf, 3),
                                        body_out_of_lanelets_m2=round(b_out, 4), corridor_out_of_lanelets_m2=round(corr_out, 3), junction_roads=junc, crosswalk_v32a=cwk),
                             dist_target=round(math.dist((dx, dy), tgt), 1) if tgt else None))
            s += 2.0
        okr = [r for r in rows if r["ok"]]
        best = (min(okr, key=lambda r: r["dist_target"]) if tgt else okr[0]) if okr else None
        res[pid] = dict(name=name, side=side, road=rid, n=len(rows), n_ok=len(okr), best=best, fails=sorted({w for r in rows for w in r["why"]}))
        print(pid, name, side, f"r{rid}", "통과", len(okr), "/", len(rows), "고른 s", best and best["s"], "목표까지", best and best["dist_target"], "막힌 까닭", res[pid]["fails"],
              best and best["check"]["door_zone_area_m2"])
    (OUT / f"platforms_{a.veh}.json").write_text(json.dumps(dict(veh=a.veh, osm=a.osm, door_x=round(s56.DOOR_X, 3), front=s56.FRONT, rear=s56.REAR,
                                                                half_w=s56.HALF_W, platforms=res), ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
