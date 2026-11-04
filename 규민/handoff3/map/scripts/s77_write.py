#!/usr/bin/env python3
# 11/04: v3.3d 승강장·순환 노선 파일. 서버 없음
#   프리우스(기본): shuttle_platforms_v33d_r1.yaml = v33c_r1 과 값 같고 지도만 v3.3d. circulation_v33d_r1.yaml = 후문 앞 구간 road 열만 섬 둘레로
#   스프린터: shuttle_platforms_v33d_r1_sprinter.yaml = s77_platforms_sprinter.py 결과(슬라이딩 문 기준), circulation_v33d_r1_sprinter.yaml = 목표·경유점만 스프린터 값
#   (alt_stop 대체 정차점은 프리우스 판에만 있음. 스프린터 판은 다시 안 셈)
# 사용: python3 map/scripts/s77_write.py --loop-len <m> --seg3-len <m>
import argparse, copy, hashlib, json
from pathlib import Path
import yaml

C = Path(__file__).resolve().parents[2]
ap = argparse.ArgumentParser(); ap.add_argument("--loop-len", type=float, required=True); ap.add_argument("--seg3-len", type=float, required=True); a = ap.parse_args()
h16 = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()[:16]
LL = C / "map/maps/v33d/lanelet2_map.osm"; old16 = "d2935e5ba33f9513"; new16 = h16(LL)
P0 = C / "map/data/shuttle_platforms_v33c_r1.yaml"; P1 = C / "map/data/shuttle_platforms_v33d_r1.yaml"; P1S = C / "map/data/shuttle_platforms_v33d_r1_sprinter.yaml"
R0 = C / "map/data/routes/circulation_v33c_r1.yaml"; R1 = C / "map/data/routes/circulation_v33d_r1.yaml"; R1S = C / "map/data/routes/circulation_v33d_r1_sprinter.yaml"
SEG3 = [1251, 1948, 1391, 1390, 1946, 1384, 1853, 1387, 1815, 1388, 1608, 1389, 1617, 1385, 1823, 1386, 1451, 1405]

# ---- 프리우스 승강장(값 같음) ----
t = P0.read_text()
first = t.split("\n", 1)[0]
t = "# 승강장 파일 v3.3d 1판(2026-11-04, v3.3c 1판과 값 같고 지도만 v3.3d: 후문 앞 섬 둘레 유턴)\n# 이전 머리말: " + first[2:] + "\n" + t.split("\n", 1)[1]
assert t.count(f"lanelet2_sha256_16: {old16}") == 1
t = t.replace(f"lanelet2_sha256_16: {old16}", f"lanelet2_sha256_16: {new16}").replace("map/maps/v33c/lanelet2_map.osm", "map/maps/v33d/lanelet2_map.osm")
P1.write_text(t)

# ---- 스프린터 승강장 ----
D = yaml.safe_load(P1.read_text()); S = json.loads((C / "integration/logs/s77/platforms_sprinter.json").read_text())
DS = copy.deepcopy(D)
DS["vehicle"] = dict(name="vehicle.mercedes.sprinter(CARLA 0.9.15)", wheel_base=3.661, front_overhang=1.015, rear_overhang=1.239, wheel_tread=1.676,
                     side_overhang=0.156, length=5.915, width=1.988, base_link="뒷바퀴 축 가운데",
                     door=dict(type="오른쪽 슬라이딩 문", x_from_base_link=S["door_x"], y_from_base_link=-0.994, width_assumed=1.3, panel_m=1.55,
                               note="문 가운데 = 뒷축 앞 1.52m. CARLA 옆 사진(문짝 이음매 두 줄)에서 잼"))
DS["params"]["corridor_width"] = round(1.988 + 0.6, 3)
for p in DS["platforms"]:
    b = S["platforms"][p["id"]]["best"]
    for k in ("marker", "door_pose", "base_link_goal", "base_link_ref", "stop_zone", "door_zone", "front_corridor"): p[k] = b[k]
    p["door_zone_area_m2"] = b["check"]["door_zone_area_m2"]; p["stop_zone_length_m"] = round(5.915 + 2.0, 2)
    p["check"] = dict(ok=True, **b["check"]); p["marker_shift_m"] = b["shift_m"]; p["goal_moved_from_prius_m"] = b["goal_moved_m"]
    p.pop("alt_stop", None); p.pop("door_along_error_m", None)
head = ("# 승강장 파일 v3.3d 1판 스프린터 판(2026-11-04). 프리우스 판(shuttle_platforms_v33d_r1.yaml)과 표지 점·문 구역 검사 식 같음, 차 치수·문 자리만 스프린터\n"
        "# 만든 것 map/scripts/s77_platforms_sprinter.py -> s77_write.py. 표지 점에서 안 되면 같은 길 앞뒤 1m 씩 옮김(marker_shift_m). 대체 정차점(alt_stop)은 없음\n")
P1S.write_text(head + yaml.safe_dump(DS, allow_unicode=True, sort_keys=False, width=200))

# ---- 순환 노선 ----
r = R0.read_text()
r = r.replace(r.split("\n", 1)[0], "# 순환 노선 v3.3d 1판(2026-11-04). v3.3c 1판에서 후문 안쪽 -> 남문 구간만 바뀜(후문 앞 유턴을 섬 둘레로, 반경 6.9m). 만든 것 map/scripts/s77_write.py", 1)
R = yaml.safe_load(r)
R["id"] = "circulation_v33d"; R["platforms_file"] = "map/data/shuttle_platforms_v33d_r1.yaml"; R["platforms_sha256_16"] = h16(P1)
R["lanelet2"] = "map/maps/v33d/lanelet2_map.osm"; R["lanelet2_sha256_16"] = new16; R["length_m"] = a.loop_len
R["note"] = R["note"].replace("후문 안쪽 다음 후문 앞 유턴 레인렛으로 돌아 나옴(11/03: 공간\n  있음, 맞은편 차가 안 올 때 돌림 = 판단 쪽 몫)", "")
R["note"] = R["note"].split("후문 안쪽 다음")[0] + "후문 안쪽 다음 후문 앞 섬을 한 바퀴 돌아 나옴(11/04: 공간 있음, 스프린터 반경. 맞은편 차가 안 올 때 돌림 = 판단 쪽 몫)"
sg = next(s for s in R["segments"] if s["frm"] == "back_gate_inner_se"); sg["roads"] = SEG3; sg["length_m"] = a.seg3_len; sg["uturn_back"] = "back_gate_island"   # via_uturn 키는 모서리 유턴(lib_north) 표시라 따로 씀
hdr = "\n".join(l for l in r.split("\n") if l.startswith("#")) + "\n"
R1.write_text(hdr + yaml.safe_dump(R, allow_unicode=True, sort_keys=False, width=200))
RS = copy.deepcopy(R); RS["id"] = "circulation_v33d_sprinter"; RS["platforms_file"] = "map/data/shuttle_platforms_v33d_r1_sprinter.yaml"; RS["platforms_sha256_16"] = h16(P1S)
RS["vehicle"] = "vehicle.mercedes.sprinter"
PS = {p["id"]: p for p in DS["platforms"]}
for st in RS["stops"]:
    p = PS[st["platform"]]; g = p["base_link_goal"]; m = p["marker"]
    st["via"] = dict(x=m["x"], y=m["y"], yaw_deg=m["yaw_deg"]); st["goal"] = dict(x=g["x"], y=g["y"], z=0.0, yaw_deg=g["yaw_deg"], qz=g["qz"], qw=g["qw"])
R1S.write_text(hdr.replace("v3.3d 1판(", "v3.3d 1판 스프린터 판(목표·경유점만 슬라이딩 문 기준, ") + yaml.safe_dump(RS, allow_unicode=True, sort_keys=False, width=200))
for f in (P1, P1S, R1, R1S): print(f.relative_to(C), h16(f))
