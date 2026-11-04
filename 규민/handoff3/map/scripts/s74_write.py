#!/usr/bin/env python3
# 11/03(s73_write.py 사본, 판 이름·입력만 v3.3b 로) 11/02 3·4단계 파일 쓰기(새 판만, 원래 판은 안 건드림). 시스템 python3(yaml) 로 돎
#   map/data/shuttle_platforms_v33b_r1.yaml: 승강장 2판(v31_r2) 에서 개성재(middle_gate_s·n)·남문(이쪽) 고치고 새 승강장 5개 더함(지금 차 기준 값,
#     스타렉스 값은 integration/logs/s73/platforms_starex.json, 기본값은 내일 시험 뒤)
#   map/data/routes/circulation_v33b_r1.yaml: 순환 노선(정류장 순서, 요청 없으면 통과하도록 via, 정차 goal), 덮기 노선
#   map/build_v33a/pairs30_v33a.json: 30쌍(중문 쌍의 승강장을 개성재 자리로, 나머지 그대로) road 열은 road 그래프 최단 경로(v3.3a)
# 이미 있으면 덮지 않음
import hashlib, json, sys
from pathlib import Path
import yaml

C = Path(__file__).resolve().parents[2]
P2 = C / "map/data/shuttle_platforms_v31_r2.yaml"
OUTP = C / "map/data/shuttle_platforms_v33b_r1.yaml"
OUTR = C / "map/data/routes/circulation_v33b_r1.yaml"
LL = C / "map/maps/v33b/lanelet2_map.osm"
for p in (OUTP, OUTR):
    if p.exists(): sys.exit(f"{p} 가 이미 있음")
h16 = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()[:16]
NEW = json.loads((C / "integration/logs/s74/platforms_sample_v33b.json").read_text())["platforms"]
LOOP = json.loads((C / "integration/logs/s73/loop_v33b.json").read_text())
d = yaml.safe_load(P2.read_text())
FIELDS = ("marker", "door_pose", "base_link_goal", "base_link_ref", "stop_zone", "door_zone", "front_corridor")


def from_new(pid, base=None):
    b = NEW[pid]["best"]; e = dict(base or {})
    for k in FIELDS: e[k] = b[k]
    e["door_zone_area_m2"] = b["check"]["door_zone_area_m2"]; e["facility"] = e.get("facility", []) if pid == "lib_south_se" else []
    e["check"] = dict(ok=True, **b["check"])
    return e


plats = []
for p in d["platforms"]:
    q = dict(p)
    if p["id"] in ("middle_gate_s", "middle_gate_n"):
        q = from_new("middle_gate_s", p)
        q.update(name="개성재", side="게이트 앞(같은 자리)", stops_v3={"road_id": 1316, "s": NEW["middle_gate_s"]["best"]["s"], "lane_id": -1}, facility=[])
        q["note"] = ("11/02: 옛 '중문' 자리(r1320·r1191)는 볼라드로 막혀 v3.3a 에서 뺌. 화면 이름 '개성재', 내부 ID 는 그대로. "
                     "middle_gate_s·n 는 같은 자리(대각선 길 r1316 북동행, 게이트 G5 앞 약 33m). 나갈 때는 N14 주차장 길(r1318)로 돌아 대각선 길 남서행으로")
        q["alt_stop"] = {"status": "없음", "reason": "11/02 새 자리, 대체 정차점은 다시 안 셈"}
        q.pop("stoppable", None); q.pop("status", None)
    elif p["id"] == "lib_south_se":
        q = from_new("lib_south_se", p)
        q.pop("stoppable", None); q.pop("status", None)
        q["note"] = "11/02: footway r1235·r1369(OSM 인도)를 차로에서 빼고 다시 셈 -> 문 구역 4.5m2, 정차 가능(하차). 대체 정차점은 2판 값 그대로가 아니므로 다시 셈 필요"
        q["alt_stop"] = {"status": "없음", "reason": "11/02 다시 안 셈"}
    plats.append(q)
for pid, (nm, side) in {"main_gate_inner_se": ("정문 안쪽(순환)", "동남행"), "main_gate_inner_nw": ("정문 안쪽(순환)", "북서행"),
                        "back_gate_inner_se": ("후문 안쪽(순환)", "동남행"), "back_gate_inner_nw": ("후문 안쪽(순환)", "북서행"),
                        "edu_front": ("사범대 앞(교대)", "북문 길 끝 쪽")}.items():
    b = NEW[pid]["best"]
    e = dict(id=pid, stop_id=pid.rsplit("_", 1)[0] if "inner" in pid else pid, name=nm, side=side,
             stops_v3={"road_id": NEW[pid]["road"], "s": b["s"], "lane_id": -1})
    e.update(from_new(pid)); e["note"] = "11/02 새 승강장(순환용). 원래 정문·후문 승강장은 30쌍 시험용으로 남김" if "inner" in pid else "11/02 새 승강장(선택): 유턴 바로 앞, 교대 학생 하차(11/02)"
    plats.append(e)
out = dict(d)
out["platforms"] = plats
out["lanelet2"] = "map/maps/v33b/lanelet2_map.osm"; out["lanelet2_sha256_16"] = h16(LL)
out["vehicle_note"] = "값은 지금 차(sample_vehicle) 기준. 스타렉스 값 integration/logs/s73/platforms_starex.json(문 위치 DOOR_X 1.502). 기본값은 내일 CARLA 시험 뒤"
head = (f"# 승강장 파일 v3.3b 1판(11/03, v3.3a 1판과 값 같고 지도만 v3.3b, 2026-11-02): 2판(shuttle_platforms_v31_r2.yaml {h16(P2)})에서 개성재(옛 중문)·남문(이쪽)을 고치고 새 승강장 5개를 더함\n"
        "# 만든 것 map/scripts/s73_platforms.py -> s73_write.py. 지도 v3.3b(map/maps/v33b). 설명 map/docs/v33a_readme.md\n")
OUTP.write_text(head + yaml.safe_dump(out, allow_unicode=True, sort_keys=False, width=200))

# 순환 노선
T = LOOP["best"]; CV = LOOP["cover"]
byid = {p["id"]: p for p in plats}
NM = {"main_gate_inner_nw": "정문 안쪽", "main_gate_inner_se": "정문 안쪽", "yangseong_se": "양성재(이쪽)", "yangseong_nw": "양성재(건너편)",
      "lib_south_nw": "도서관 남문(건너편)", "back_gate_inner_se": "후문 안쪽", "back_gate_inner_nw": "후문 안쪽", "lib_south_se": "도서관 남문(이쪽)", "lib_north": "도서관 북문", "edu_front": "사범대 앞", "middle_gate_s": "개성재"}
stops = []
for pid in T["stops"]:
    p = byid[pid]
    stops.append(dict(platform=pid, name=NM.get(pid, p["name"]), action="stop_if_requested",
                      via={"x": p["marker"]["x"], "y": p["marker"]["y"], "yaw_deg": p["marker"]["yaw_deg"]}, goal=p["base_link_goal"]))
route = dict(id="circulation_v33b", platforms_file="map/data/shuttle_platforms_v33b_r1.yaml", platforms_sha256_16=h16(OUTP),
             lanelet2="map/maps/v33b/lanelet2_map.osm", lanelet2_sha256_16=h16(LL), xodr_sha256_16="214507afc87c2523",
             note=("순환 노선 후보(맵 몫, 운행 논리는 판단 쪽). 요청이 없으면 via 를 경유점으로 넣고 지나가고, 요청이 있으면 goal 로 섬. "
                   "한 바퀴 끝 = 처음 정류장. 북문 다음 사범대 앞에서 북문 길 끝 유턴 레인렛으로 돌아 나옴. 개성재 뒤는 N14 주차장 길로 돌아 나옴. "
                   "후문 안쪽 다음 후문 앞 유턴 레인렛으로 돌아 나옴(11/03: 공간 있음, 맞은편 차가 안 올 때 돌림 = 판단 쪽 몫)"),
             stops=stops, length_m=T["length"],
             segments=[dict(frm=s["frm"], to=s["to"], roads=s["roads"], length_m=s["length"]) for s in T["segs"]],
             uturn=[{"lanelet": 52799, "where": "북문 길 r1415 끝(개신문화관 지나 사범대 앞)"}, {"lanelet": json.loads((C / "map/maps/v33b/v33b_changes.json").read_text())["back_uturn"]["relation"], "where": "후문 앞 r1251 끝 -> r1384"}],
             cover_route=dict(note="양방향·고리 안 큰 도로를 모두 한 번 지나는 판(경유점 via_r<road>)", length_m=CV["length"], order=CV["stops"],
                              missed_big_ways=CV["missed"]))
OUTR.write_text("# 순환 노선 v3.3b 1판(2026-11-02). 만든 것 map/scripts/s73_loop.py -> s73_write.py\n" + yaml.safe_dump(route, allow_unicode=True, sort_keys=False, width=200))
print(OUTP, h16(OUTP)); print(OUTR, h16(OUTR))
