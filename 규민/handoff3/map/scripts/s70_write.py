#!/usr/bin/env python3
# 10/25 3단계: 대체 정차점을 넣은 새 판 두 개를 씀(원래 판은 안 건드림)
#   map/data/shuttle_platforms_v31_r2.yaml = r1 본문 그대로 + 승강장마다 alt_stop 블록(s70_alt_stops.json·s70_alt_route_check.json)
#   map/data/routes/pilot_route_r3.yaml = r2 본문 그대로 + platforms_file 을 r2 로, 정류장마다 alt_goal
# 이미 있으면 덮지 않음(먼저 옮김). 사용: python3 map/scripts/s70_write.py
import hashlib
import json
import sys
from pathlib import Path

C = Path(__file__).resolve().parents[2]
P1, P2 = C / "map/data/shuttle_platforms_v31_r1.yaml", C / "map/data/shuttle_platforms_v31_r2.yaml"
R2, R3 = C / "map/data/routes/pilot_route_r2.yaml", C / "map/data/routes/pilot_route_r3.yaml"
ALT = json.loads((C / "map/docs/logs/s70_alt_stops.json").read_text())
RC = json.loads((C / "map/docs/logs/s70_alt_route_check.json").read_text())
h16 = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()[:16]
j = lambda v: json.dumps(v, ensure_ascii=False)
for p in (P2, R3):
    if p.exists():
        sys.exit(f"{p} 가 이미 있음. 옮긴 뒤 다시")

alt = {p["id"]: p for p in ALT["platforms"]}


def block(pid):
    a = alt[pid]
    if a["status"] != "있음":
        return [f"    alt_stop: {{\"status\": \"없음\", \"reason\": {j(a['reason'])}}}"]
    x, r31, r32 = a["alt"], RC["v31"][pid], RC["v32a"][pid]
    inc = r31["incoming"]
    ok = sum(1 for v in inc.values() if v["found"] and v["prefix_of_original"])
    out = ["    alt_stop:  # 세션70 대체 정차점('정류장 막힘' 장면용). 같은 차로, 승강장 뒤쪽(들어오는 쪽)으로 문 점을 shift_back_m 만큼 옮김",
           f"      shift_back_m: {x['shift_back_m']}  # 정차 구역 길이 6.89m 부터 0.25m 씩 늘려 검사를 모두 통과한 가장 가까운 값",
           f"      door_pose: {j(x['door_pose'])}",
           f"      base_link_goal: {j(x['base_link_goal'])}",
           f"      base_link_ref: {j(x['base_link_ref'])}  # 참고용",
           f"      stop_zone: {j(x['stop_zone'])}",
           f"      door_zone: {j(x['door_zone'])}",
           f"      front_corridor: {j(x['front_corridor'])}  # 앞 8m 는 원래 정차 구역 쪽(막은 물체가 있으면 출발 조건에 걸림)",
           f"      check: {j(dict(body_out_of_lanelets_m2=x['body_out_of_lanelets_m2'], gap_to_original_stop_zone_m=x['gap_to_original_stop_zone_m'], door_zone=x['door_zone_info'], stop_zone_roads=x['stop_zone_roads'], junction_roads=[], crosswalk_v32a=[], trees_in_door_zone=x['trees_in_door_zone'], skipped_before=a['fails_before']))}",
           f"      route_28a57711: {j(dict(incoming_pairs_prefix_ok=f'{ok}/{len(inc)}', orig_to_alt=r31['orig_to_alt'], alt_to_orig=r31['alt_to_orig']))}",
           f"      route_v32a_same: {j(r31 == r32)}"]
    return out


lines, cur = [], None
for ln in P1.read_text().splitlines():
    if ln.startswith("  - id: "):
        if cur:
            lines += block(cur)
        cur = ln.split(": ", 1)[1].strip()
    lines.append(ln)
lines += block(cur)
head = [f"# 승강장 파일 2판(세션70, 2026-10-25): 1판(shuttle_platforms_v31_r1.yaml {h16(P1)}) 본문 그대로에 승강장마다 alt_stop(대체 정차점)만 더함",
        "# 대체 정차점 만든 것 map/scripts/s70_alt_stops.py -> s70_alt_route_check.py -> s70_write.py. 설명 map/docs/shuttle_route_file.md '대체 정차점'"]
P2.write_text("\n".join(head + lines) + "\n")

rl = R2.read_text().splitlines()
out = [f"# 노선 파일 3판(세션70, 2026-10-25): 2판(pilot_route_r2.yaml {h16(R2)})에서 platforms 참조를 승강장 2판으로, 정류장마다 alt_goal(대체 정차점 뒷축 목표)을 더함. 노선·목표·구간 값은 2판과 같음"]
for ln in rl:
    if ln.startswith("platforms_file:"):
        ln = "platforms_file: map/data/shuttle_platforms_v31_r2.yaml  # 3판: 대체 정차점(alt_stop)이 든 승강장 2판"
    elif ln.startswith("platforms_sha256_16:"):
        ln = f"platforms_sha256_16: {h16(P2)}  # 위 파일. 1판 본문(정차점·구역)은 바이트 같음"
    elif ln.startswith("  - {platform: "):
        pid = ln.split("platform: ", 1)[1].split(",", 1)[0]
        a = alt[pid]
        g = a["alt"]["base_link_goal"] if a["status"] == "있음" else None
        ln = ln.rstrip()
        tail = ""
        if "  #" in ln:
            ln, tail = ln.split("  #", 1)
            tail = "  #" + tail
        ln = ln[:-1] + f", alt_goal: {j(g)}}}" + tail
    out.append(ln)
out.append("# alt_goal: 원래 정차 구역이 막혔을 때 줄 목표(승강장 2판 alt_stop.base_link_goal). 들어오는 길로 와서 원래보다 먼저 섬. null 이면 대체 정차점 없음")
R3.write_text("\n".join(out) + "\n")
print(P2, h16(P2)); print(R3, h16(R3))
