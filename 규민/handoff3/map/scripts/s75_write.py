#!/usr/bin/env python3
# 11/03: v3.3c 승강장·순환 노선 파일(v3.3b 1판에서 글자 바꿈으로). 서버 없음
#   승강장 v33c_r1: v33b_r1 과 값 같음. 사범대 앞(edu_front)은 레인렛이 나뉘어 37801 -> 52801(모서리 뒤 조각), 순환 밖(선택)으로 메모
#   순환 v33c_r1: 사범대 앞을 빼고, 도서관 북문 -> (가 모서리 유턴 52803) -> 개성재 로 이음. road 열 = 1415(앞 조각), 1409(뒤 조각), 그 뒤 v33b 와 같음
#   길이는 s75_check_v33c.py 가 Lanelet2 로 다시 잼(그 값을 --len 로 넣음)
# 사용: python3 map/scripts/s75_write.py [--seg-len 0]
import argparse, hashlib, json, re
from pathlib import Path

C = Path(__file__).resolve().parents[2]
ap = argparse.ArgumentParser(); ap.add_argument("--seg-len", type=float, default=0.0); ap.add_argument("--loop-len", type=float, default=0.0); a = ap.parse_args()
h16 = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()[:16]
LL = C / "map/maps/v33c/lanelet2_map.osm"; CH = json.loads((C / "map/maps/v33c/v33c_changes.json").read_text())
P0 = C / "map/data/shuttle_platforms_v33b_r1.yaml"; P1 = C / "map/data/shuttle_platforms_v33c_r1.yaml"
R0 = C / "map/data/routes/circulation_v33b_r1.yaml"; R1 = C / "map/data/routes/circulation_v33c_r1.yaml"

t = P0.read_text()
t = t.replace("# 승강장 파일 v3.3b 1판(세션74, v3.3a 1판과 값 같고 지도만 v3.3b, 2026-11-02)",
              "# 승강장 파일 v3.3c 1판(세션75, 2026-11-03, v3.3b 1판과 값 같고 지도만 v3.3c, 사범대 앞 레인렛 id 만 바뀜)\n# 이전 머리말: 승강장 파일 v3.3b 1판(세션74, v3.3a 1판과 값 같고 지도만 v3.3b, 2026-11-02)", 1)
t = t.replace("lanelet2_sha256_16: bf0d421ef989efe8", f"lanelet2_sha256_16: {h16(LL)}", 1)
t = t.replace("lanelet2: map/maps/v33b/lanelet2_map.osm", "lanelet2: map/maps/v33c/lanelet2_map.osm")
i = t.index("- id: edu_front"); j = t.index("note: '세션73 새 승강장(선택): 유턴 바로 앞", i)
blk = t[i:j]; assert blk.count("- 37801") == 1
blk = blk.replace("- 37801", f"- {CH['split']['r1415']['back']}").replace("side: 북문 길 끝 쪽", "side: 도서관 길 모서리 지나 북동쪽")
t = t[:i] + blk + t[j:].replace("note: '세션73 새 승강장(선택): 유턴 바로 앞, 교대 학생 하차(사용자 11/2)'",
                                    "note: '세션73 새 승강장(선택): 교대 학생 하차(사용자 11/2). 세션75 에서 유턴이 도서관 길 모서리(가)로 옮겨 순환 밖. 쓰려면 모서리 유턴 앞에서 북동으로 들어갔다 나오는 길이 따로 있어야 함(수요일 재점검)'", 1)
P1.write_text(t)

r = R0.read_text()
r = r.replace("# 순환 노선 v3.3b 1판(세션74, 2026-11-02). 만든 것 map/scripts/s73_loop.py -> s73_write.py",
              "# 순환 노선 v3.3c 1판(세션75, 2026-11-03). v3.3b 1판에서 map/scripts/s75_write.py 로 고침(유턴 자리 '가', 사범대 앞 뺌)", 1)
r = r.replace("id: circulation_v33b", "id: circulation_v33c").replace("platforms_file: map/data/shuttle_platforms_v33b_r1.yaml", "platforms_file: map/data/shuttle_platforms_v33c_r1.yaml")
r = re.sub(r"platforms_sha256_16: \w+", f"platforms_sha256_16: {h16(P1)}", r, 1)
r = r.replace("lanelet2: map/maps/v33b/lanelet2_map.osm", "lanelet2: map/maps/v33c/lanelet2_map.osm")
r = r.replace("lanelet2_sha256_16: bf0d421ef989efe8", f"lanelet2_sha256_16: {h16(LL)}", 1)
r = r.replace("북문 다음 사범대 앞에서 북문 길 끝 유턴 레인렛으로 돌아 나옴.",
              "북문 다음 도서관 길이 꺾이는 모서리(개신문화관 정면)에서 유턴 레인렛으로 돌아 나옴(사용자 11/3 오후: 좌회전하는 척하다 차 없으면 유턴). 사범대 앞은 순환 밖.")
# 정류장 목록에서 edu_front 빼기
i = r.index("- platform: edu_front"); j = r.index("- platform: middle_gate_s"); r = r[:i] + r[j:]
# 구간 둘을 하나로
i = r.index("- frm: lib_north\n  to: edu_front"); j = r.index("- frm: middle_gate_s\n  to: main_gate_inner_nw")
old = r[i:j]; tail = old[old.index("- frm: edu_front"):]
roads = re.findall(r"  - (\d+)", tail[:tail.index("length_m")])
assert roads[:2] == ["1415", "1409"], roads
seg = "- frm: lib_north\n  to: middle_gate_s\n  via_uturn: library_corner\n  roads:\n" + "".join(f"  - {x}\n" for x in roads) + f"  length_m: {a.seg_len}\n"
r = r[:i] + seg + r[j:]
r = r.replace(f"- lanelet: 52799\n  where: 북문 길 r1415 끝(개신문화관 지나 사범대 앞)",
              f"- lanelet: {CH['uturn']['relation']}\n  where: 도서관 길 꺾이는 모서리(r1415 -> r1409, 개신문화관 정면, 사용자 11/3 오후)")
if a.loop_len: r = re.sub(r"^length_m: [\d.]+", f"length_m: {a.loop_len}", r, count=1, flags=re.M)
r = r.replace("  - lib_north\n  - edu_front\n  - middle_gate_s", "  - lib_north\n  - middle_gate_s")
R1.write_text(r)
print(P1, h16(P1)); print(R1, h16(R1))
