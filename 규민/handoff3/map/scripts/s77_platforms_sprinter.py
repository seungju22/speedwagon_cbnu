#!/usr/bin/env python3
# 11/04 2단계: 스프린터 슬라이딩 문 기준 승강장 목표. s76_dropoff.py 사본(10/12 검사 식 그대로)
#   승강장마다 표지 점(marker = 문이 설 차로 중심 점)은 그대로 두고, 차 치수·문 자리만 스프린터로 바꿔 뒷축 목표·정차 구역·통로·차체를 다시 셈
#   스프린터 문(오른쪽 슬라이딩 문): 문 가운데 = 뒷축 앞 --door-x m(기본 1.52. CARLA 옆 사진 integration/logs/s77/carla/door_photo.jpg 에서 잰 문짝 -0.98~0.57m(차 중심 기준) 가운데. 처음 가정 2.11 은 0.59m 앞이었음)
#   문 옆 깊이는 0.795 이상(10/12 시설이 차선 끝에서 딱 0.8m 라 계산 반올림으로 0.798 이 나옴)
#   표지 점에서 검사가 안 되면 같은 길에서 앞뒤 1m 씩 최대 6m 옮겨 가장 가까운 통과 자리를 고름
#   지도 v3.3d. 출력 integration/logs/s77/platforms_sprinter.json(+ --veh sample 이면 같은 식으로 프리우스 값을 다시 셈: 기존 파일과 맞는지 확인용)
# 사용: .venv-carla/bin/python map/scripts/s77_platforms_sprinter.py [--veh sprinter|sample] [--door-x 1.52]
import argparse, json, math, subprocess
from pathlib import Path
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union
from shapely.strtree import STRtree
import s72_common as K
import s56_shuttle as s56
import s70_alt_stops as s70
import place_campus_layers as pcl
import carla

ap = argparse.ArgumentParser(); ap.add_argument("--veh", default="sprinter"); ap.add_argument("--osm", default=str(K.C / "map/maps/v33d/lanelet2_map.osm"))
ap.add_argument("--door-x", type=float, default=1.52); ap.add_argument("--src", default=str(K.C / "map/data/shuttle_platforms_v33c_r1.yaml"))
a = ap.parse_args()
OUT = K.C / "integration/logs/s77"
if a.veh == "sprinter":   # 11/03 CARLA 실측(s75_vehicle_info_sprinter): 축거 3.661, 앞뒤 1.015/1.239, 폭 1.988, 윤거 1.676
    s56.WB, s56.FO, s56.RO, s56.TREAD = 3.661, 1.015, 1.239, 1.676
    s56.LO = s56.RGO = (1.988 - 1.676) / 2
    s56.FRONT, s56.REAR = s56.WB + s56.FO, -s56.RO
    s56.HALF_W = 1.988 / 2; s56.DOOR_X = a.door_x; s56.DOOR_Y = -s56.HALF_W; s56.CORRIDOR_W = 1.988 + 0.6
DROP_Q = {1235, 1369}


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

    SRC = json.loads(subprocess.check_output(["/usr/bin/python3", "-c", "import sys,json,yaml;print(json.dumps(yaml.safe_load(open(sys.argv[1]))))", a.src]))["platforms"]
    res = {}
    for P0 in SRC:
        pid, name, side = P0["id"], P0["name"], P0.get("side", "")
        mk = P0["marker"]; w0 = cmap.get_waypoint(carla.Location(mk["x"], -mk["y"], 0.0), project_to_road=True, lane_type=carla.LaneType.Driving)
        rid = w0.road_id; tgt = (mk["x"], mk["y"]); prefer = {rid, P0["base_link_ref"]["road_id"]}
        rows = []
        for ds in (0, -1, 1, -2, 2, -3, 3, -4, 4, -5, 5, -6, 6):
            wp = w0.next(ds)[0] if ds > 0 else (w0.previous(-ds)[0] if ds < 0 else w0)
            s = ds
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
            ok = dz.area >= 2.0 and depth >= 0.795 and swf >= 0.9 and b_out <= 0.01 and corr_out <= 0.5 and not junc and not cwk
            q = (round(math.sin(math.radians(byaw) / 2), 6), round(math.cos(math.radians(byaw) / 2), 6))
            rows.append(dict(s=round(s, 1), ok=ok, why=[k for k, v in (("문 구역", dz.area < 2.0 or depth < 0.795), ("인도 밖", swf < 0.9), ("차체", b_out > 0.01),
                                                                        ("통로", corr_out > 0.5), ("교차로", bool(junc)), ("횡단보도", bool(cwk))) if v],
                             marker=dict(x=round(px, 3), y=round(py, 3), yaw_deg=round(pyaw, 2)),
                             door_pose=dict(x=round(dx, 3), y=round(dy, 3), yaw_deg=round(byaw, 2)),
                             base_link_goal=dict(x=round(bx, 3), y=round(by, 3), z=0.0, yaw_deg=round(byaw, 2), qz=q[0], qw=q[1]),
                             base_link_ref=dict(road_id=wr.road_id, s=round(wr.s, 2), lanelet_id=sorted({l["id"] for l in bll if l["poly"].contains(Point(bx, by))})),
                             stop_zone=s56.pts(sz), door_zone=s56.pts(dz), front_corridor=s56.pts(corr), body=s56.pts(bd, 3),
                             check=dict(door_zone_area_m2=round(dz.area, 3), depth_at_door_m=round(depth, 3), sidewalk_frac=round(swf, 3),
                                        body_out_of_lanelets_m2=round(b_out, 4), corridor_out_of_lanelets_m2=round(corr_out, 3), junction_roads=junc, crosswalk_v32a=cwk),
                             dist_target=round(math.dist((px, py), tgt), 2), shift_m=ds))
        okr = [r for r in rows if r["ok"]]
        best = okr[0] if okr else None   # 옮김이 가장 작은 통과 자리(0 -> -1 -> +1 ... 순서)
        old_goal = P0["base_link_goal"]
        if best: best["goal_moved_m"] = round(math.dist((old_goal["x"], old_goal["y"]), (best["base_link_goal"]["x"], best["base_link_goal"]["y"])), 3)
        res[pid] = dict(name=name, side=side, road=rid, n=len(rows), n_ok=len(okr), best=best, at_marker=rows[0], rows_fail=[dict(shift=r["s"], why=r["why"]) for r in rows if not r["ok"]])
        print(pid, f"r{rid}", "표지 점", "통과" if rows[0]["ok"] else "막힘 " + ",".join(rows[0]["why"]), "고른 옮김", best and best["s"], "문 구역", best and best["check"]["door_zone_area_m2"],
              "목표 옮김", best and best["goal_moved_m"])
    (OUT / f"platforms_{a.veh}.json").write_text(json.dumps(dict(veh=a.veh, osm=a.osm, door_x=round(s56.DOOR_X, 3), front=s56.FRONT, rear=s56.REAR,
                                                                half_w=s56.HALF_W, src=a.src, platforms=res), ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
