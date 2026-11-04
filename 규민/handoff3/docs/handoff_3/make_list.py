"""인계 3 목록 만들기(10/26). 올리지 않음. 원본 경로 -> 저장소 규민/handoff3/ 아래 같은 상대 경로, 크기, sha256 앞 16자.
범위: 맵·인지와 그것을 돌리는 통합 실행 도구·문서(인계 2와 같은 범위) + 발표 재료(영상 빼고). 판단·제어 코드와 셔틀 기능 실험은 안 넣음.
인계 2 목록(docs/handoff_2/handoff_2_list.tsv) 69개는 다 넣고(경로 같음), 그 뒤 새로 생긴 것을 묶음별로 더함.
100MB 넘는 파일은 빼고 따로 적음. 출력: docs/handoff_3/handoff_3_list.tsv, 화면에 묶음별 개수·바이트"""
import csv, glob, hashlib, os
C = os.path.expanduser("~/campus_mobility_sim")
H2 = [r for r in csv.DictReader(open(os.path.join(C, "docs/handoff_2/handoff_2_list.tsv")), delimiter="\t")]
G = [(r["묶음"], r["원본"]) for r in H2]
ms = "map/scripts/"
G += [("지도 재현(OSM -> xodr -> Lanelet2)", p) for p in [
    "docs/handoff_3/map_reproduce.md", ms + "s71_reproduce.sh", ms + "download_osm.sh", ms + "compute_bbox.py",
    ms + "fix_tags.py", ms + "classify_internal.py", ms + "smooth_osm_curves.py", ms + "curve_metrics.py", ms + "osm_to_xodr.py",
    ms + "s27_reconvert.py", ms + "s27_turnfix.py", ms + "s42_common.py", ms + "s43_build_v3.py", ms + "s45_build_v3.py",
    ms + "s46_build_v3.py", ms + "s53_build_v31.py", ms + "s53_geom.py", ms + "s53_check.py", ms + "s54_build.py",
    ms + "s22_buildings.py", ms + "s48_stops.py", ms + "s49_b2_v3.py", ms + "analyze_geometry.py", ms + "analyze_wedge_s11.py",
    ms + "geo_calibrate.py", ms + "s34_curvature.py",
    "map/data/raw/cbnu_campus.osm", "map/data/processed/cbnu_relation_polygon.json",
    "map/maps/cbnu_internal_only_localtm_tags73_smooth.xodr", "map/data/processed/cbnu_campus_fixed_tags73.osm",
    "map/data/processed/cbnu_internal_boundary_tags73.osm", "map/data/processed/cbnu_internal_boundary_tags73_smooth.osm",
    "map/data/stops_v3_draft_s46.yaml", "map/data/stops_v2.yaml", "map/docs/logs/s25_buildings.csv",
    "map/data/processed/s40_b2_cut.csv", "map/docs/logs/s42_way_dir.csv",
    "integration/logs/s71/step2/reproduce.log"]]
G += [("지도 v3.2(횡단보도·방지턱)", p) for p in [
    "map/maps/v32a/lanelet2_map.osm", "map/maps/v32a/map_projector_info.yaml",
    "map/maps/v32b/lanelet2_map.osm", "map/maps/v32b/map_projector_info.yaml", "map/maps/v32b/v32b_dropped.txt",
    "map/maps/v32_candidate/v32_elements.csv", "map/data/crossings_v3_candidates.csv",
    ms + "s63_v32_candidate.py", ms + "s66_make_v32ab.py", ms + "s63_route_check.py", ms + "s66_route_check.py",
    "integration/docs/tools/load_speed_bump_plugin.sh", "map/docs/v32_switch_howto.md", "map/docs/v32_candidate.md"]]
G += [("장식 레이어 v2(건물 2순위·나무)", p) for p in [
    ms + "place_campus_layers_s66.py", ms + "s66_b2_tier2.py", ms + "s66_trees.py",
    "map/data/buildings_layout_tier2_s66.csv", "map/data/trees_layout_s66.csv", "map/docs/campus_layers.md"]]
G += [("정류장 대체 정차점", p) for p in [
    "map/data/shuttle_platforms_v31_r2.yaml", "map/data/routes/pilot_route_r3.yaml",
    ms + "s56_shuttle.py", ms + "s70_alt_stops.py", ms + "s70_alt_route_check.py", ms + "s70_write.py", ms + "s70_alt_figs.py",
    "map/docs/logs/s70_alt_stops.json", "map/docs/logs/s70_alt_route_check.json", "map/docs/figs/s70_alt_stops.png"]]
G += [("지도 v3.3(개성재·남문·순환·유턴·차량, 11/02~11/03)", p) for p in [
    "map/maps/v33a/lanelet2_map.osm", "map/maps/v33a/map_projector_info.yaml", "map/maps/v33a/v33a_changes.json",
    "map/maps/v33b/lanelet2_map.osm", "map/maps/v33b/map_projector_info.yaml", "map/maps/v33b/v33b_changes.json",
    "map/data/shuttle_platforms_v33b_r1.yaml", "map/data/routes/circulation_v33b_r1.yaml", "map/docs/v33b_readme.md",
    "map/docs/figs/v33b_loop.png", "map/docs/figs/v33b_loop_cover.png", "map/docs/figs/v33b_gaeseong_check.png",
    ms + "s72_common.py", ms + "s72_net.py", ms + "s73_make_v33.py", ms + "s74_make_v33b.py", ms + "s73_check_v33.py", ms + "s74_check_v33b.py",
    ms + "s73_platforms.py", ms + "s73_loop.py", ms + "s74_write.py", ms + "s73_m20.py", ms + "s74_m20.py", ms + "s59_m20.py", ms + "s53_footprint.py", ms + "s70_alt_stops.py",
    "integration/config/s73_starex.override.yaml", "integration/config/s73_vehicle_info_starex.param.yaml", "integration/config/s73_sensor_kit_calibration_roof.yaml",
    "integration/config/s73_sensor_mapping_pps545k_van.yaml", "integration/config/s74_v33b_map.override.yaml",
    "integration/docs/tools/s74_run_loop.sh", "integration/docs/tools/s73_ps_goal.sh", "integration/docs/tools/s74_ps_run.sh",
    "docs/handoff_3/LICENSE-APACHE-2.0.txt"]]
G += [("지도 v3.3c(유턴 자리·N14·스프린터, 11/03)", p) for p in [
    "map/maps/v33c/lanelet2_map.osm", "map/maps/v33c/map_projector_info.yaml", "map/maps/v33c/v33c_changes.json",
    "map/data/shuttle_platforms_v33c_r1.yaml", "map/data/routes/circulation_v33c_r1.yaml", "map/docs/v33c_readme.md",
    "map/docs/figs/v33c_loop.png", "map/docs/figs/v33c_uturn_lib.png", "map/docs/figs/v33c_n14.png",
    ms + "s75_uturn_search.py", ms + "s75_make_v33c.py", ms + "s75_write.py", ms + "s75_check_v33c.py", ms + "s75_m20.py",
    ms + "s75_uturn_fig.py", ms + "s75_loop_fig.py",
    "integration/config/s75_v33c_map.override.yaml", "integration/config/s75_sprinter.override.yaml", "integration/config/s75_vehicle_info_sprinter.param.yaml",
    "integration/docs/tools/s75_run_loop.sh", "integration/docs/tools/s75_offroad_watch.py", "integration/docs/tools/s75_track.py",
    "integration/docs/tools/s75_wheel_recheck.py", "integration/docs/tools/s75_steer_eval.py"]]
pt = "perception/tools/"
G += [("인지: 클러스터링·시설/건물 점 지움·비", p) for p in [
    pt + "s67_facility_crop.py", pt + "s67_fac_swap.sh", "perception/config/s67_facility_polygons.json",
    "perception/config/s69_building_rects.json", pt + "s69_building_polys.py",
    "integration/config/sensor_mapping_nocam_pps545k.yaml", "perception/docs/rain_model.md", pt + "s71_rain_sens.py"]]
G += [("인지 평가(재현용)", p) for p in [
    pt + "s67_objects_dump.py", pt + "s67_false_objects.py", pt + "s67_door_compare.py", pt + "s67_cluster_offline.sh",
    pt + "s68_mock/carla.py", pt + "s68_mock/run_mock.sh", pt + "s68_static_run.sh", pt + "s68_spawn_lat.py",
    pt + "s68_spawn_lat_eval.py", pt + "s68_dist_scene.py", pt + "s68_bag_points.py", pt + "s68_dist_eval.py",
    pt + "s69_sweep.launch.py", pt + "s69_sweep_offline.sh", pt + "s69_sweep_dump.py", pt + "s69_dist_eval.py",
    pt + "s69_sweep_table.py", pt + "s69_bld_check.py", pt + "s70_sweep.launch.py", pt + "s70_sweep_offline.sh",
    pt + "s70_minpts_judge.py", pt + "s70_rain_table.py",
    "map/docs/logs/campus_layers_dry_s66_all_actors.csv"]]  # 평가 도구가 레이어 v2 자리를 읽음(s69_building_rects.json 의 출처)
G += [("실행 보조", p) for p in [
    "integration/docs/tools/s71_keepawake.sh", "integration/docs/tools/s71_door_timing.py"]]
G += [("문서(인계 3)", p) for p in [
    "docs/handoff_3/LICENSE", "docs/handoff_3/README.md", "docs/handoff_3/NOTICE.md", "docs/handoff_3/changes_since_handoff2.md",
    "docs/handoff_3/make_list.py"]]
G += [("차체 검사 도구", p) for p in [
    "map/scripts/s53_footprint.py", "map/scripts/s59_m20.py", "map/build_v31_s54/pairs35.json",
    "map/docs/logs/s56b_footprint_pairs.json", "map/docs/logs/s56c_footprint_pairs.json"]]  # 11/04: 점검 페이지(docs/map_review)와 그 생성기는 뺌
G += [("지도 v3.3d(후문 섬 유턴·스프린터 문, 11/04)", p) for p in [
    "map/maps/v33d/lanelet2_map.osm", "map/maps/v33d/map_projector_info.yaml", "map/maps/v33d/v33d_changes.json",
    "map/data/shuttle_platforms_v33d_r1.yaml", "map/data/shuttle_platforms_v33d_r1_sprinter.yaml",
    "map/data/routes/circulation_v33d_r1.yaml", "map/data/routes/circulation_v33d_r1_sprinter.yaml", "map/docs/v33d_readme.md",
    "map/docs/figs/v33d_back_uturn.png", "map/docs/figs/v33d_sprinter_door.jpg", "map/docs/figs/v33d_loop.png",
    ms + "s77_uturn_search.py", ms + "s77_loop_fig.py", ms + "s77_make_v33d.py", ms + "s77_check_v33d.py", ms + "s77_m20.py", ms + "s77_platforms_sprinter.py", ms + "s77_write.py",
    ms + "s56_shuttle.py", ms + "place_campus_layers.py",
    "integration/config/s77_v33d_map.override.yaml", "integration/config/s77_segments_loop_sprinter.txt", "integration/docs/tools/s77_run_loop.sh"]]
G += [("스프린터 차량 설정(11/04)", p) for p in [
    "integration/config/s76_sprinter.override.yaml", "integration/config/s76_sprinter_accel_map.csv", "integration/config/s76_sprinter_brake_map.csv",
    "integration/config/s76_raw_vehicle_cmd_converter_sprinter.param.yaml", "integration/config/s76_mirror_sprinter.param.yaml",
    "integration/config/s74_sensor_mapping_van_pick.yaml",
    "integration/docs/tools/s76_pedal_bench.py", "integration/docs/tools/s76_make_maps.py", "integration/docs/tools/s76_loop_eval.py"]]
G += [("발표 재료(영상 뺌)", p) for p in ["docs/midterm_material/*", "docs/midterm_material/stills/*"]]
SKIP_EXT = (".mp4", ".webm", ".avi", ".mkv")
REPO = {"docs/handoff_3/LICENSE": "LICENSE"}  # 10/27: MIT 전문은 저장소 규민/handoff3/ 맨 위에 둠
rows, big, miss, seen = [], [], [], set()
for grp, pat in G:
    fs = sorted(f for f in glob.glob(os.path.join(C, pat)) if os.path.isfile(f))
    if not fs: miss.append(pat)
    for f in fs:
        rel = os.path.relpath(f, C)
        if ".bak" in rel or "__pycache__" in rel or rel.endswith(SKIP_EXT) or rel in seen or rel.startswith("docs/map_review") or "handoff_3_list" in rel: continue
        seen.add(rel)
        sz = os.path.getsize(f)
        if sz > 100 * 1024 * 1024: big.append((rel, sz)); continue
        h = hashlib.sha256(open(f, "rb").read()).hexdigest()[:16]
        rows.append((grp, rel, "규민/handoff3/" + REPO.get(rel, rel), sz, h))
with open(os.path.join(C, "docs/handoff_3/handoff_3_list.tsv"), "w") as o:
    o.write("묶음\t원본\t저장소\t바이트\tsha256_16\n")
    for r in rows: o.write("\t".join(map(str, r)) + "\n")
print("files", len(rows), "bytes", sum(r[3] for r in rows), "big", big, "missing", miss)
by = {}
for r in rows: by.setdefault(r[0], [0, 0]); by[r[0]][0] += 1; by[r[0]][1] += r[3]
for k, v in by.items(): print(k, v[0], v[1])
# 요약 문서(목록 파일 자체와 이 요약은 표 밖. 바이트·해시는 tsv 에 있음)
NOTE = """# 인계 3 목록 (10/26 처음 만듦, 11/04 마감)

저장소 자리는 `규민/handoff3/` 아래에 작업 폴더와 같은 상대 경로로 둠. 스크립트와 compose 오버레이가 `~/campus_mobility_sim` 경로를 쓰기 때문임.
범위: 맵·인지와 그것을 돌리는 통합 실행 도구·문서(인계 2와 같은 범위) + 발표 재료(영상 뺌). 판단·제어 코드와 셔틀 기능 실험, 작업 기록, 점검 페이지는 안 넣음.
파일 {n}개, 합계 {b:,} 바이트. 100MB 넘는 파일 없음(가장 큰 것 {big} {bigb:,} B). 이 요약과 handoff_3_list.tsv 는 표 밖에 더 올라감.
전체 목록(원본·저장소 경로·바이트·sha256 앞 16자) handoff_3_list.tsv, 다시 만들기 make_list.py

| 묶음 | 파일 수 | 바이트 |
|---|---|---|
"""
mx = max(rows, key=lambda r: r[3])
with open(os.path.join(C, "docs/handoff_3/handoff_3_list.md"), "w") as o:
    o.write(NOTE.format(n=len(rows), b=sum(r[3] for r in rows), big=mx[1], bigb=mx[3]))
    for k, v in by.items(): o.write(f"| {k} | {v[0]} | {v[1]:,} |\n")
    o.write("\n올리기 전 검사: 개인 경로, 규민/ 밖, 범위 밖, 문체, 크기.\n")
