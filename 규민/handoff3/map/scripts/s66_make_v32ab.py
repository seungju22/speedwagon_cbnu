#!/usr/bin/env python3
# 10/23: v3.2 마무리 준비. X30·X44(10/22 보류)를 로드뷰로 정할 수 있게 두 판을 만듦(서버 없음)
#   v32a = v3.2 후보(4719e5e5) 그대로 42곳. 바이트 복사
#   v32b = v32a 에서 X30(횡단보도)·X44(방지턱)만 뺀 40곳. 두 요소가 만든 node·way·relation 과
#          도로 레인렛의 regulatory_element member 줄만 지움. 나머지 줄·id 는 v32a 와 같음
# 지울 대상은 v32_elements.csv 의 element 열과 osm 안 s63_src 태그로 찾고 서로 맞는지 확인함
# 사용: .venv-carla/bin/python map/scripts/s66_make_v32ab.py
import csv
import hashlib
import re
import shutil
from pathlib import Path

C = Path(__file__).resolve().parents[2]
SRC = C / "map/maps/v32_candidate"
SRC_SHA = "4719e5e5a4f4588bc0e8a48c53d9bc71beaade7c446389e0f5a4ee3c47cfc789"
DROP = ["X30", "X44"]
OUT_A, OUT_B = C / "map/maps/v32a", C / "map/maps/v32b"


def block(text, kind, i):
    m = re.search(rf'  <{kind} id="{i}" [^\n]*>\n.*?  </{kind}>\n', text, re.S)
    assert m, (kind, i)
    return m


def main():
    raw = (SRC / "lanelet2_map.osm").read_bytes()
    assert hashlib.sha256(raw).hexdigest() == SRC_SHA, "v3.2 후보 해시 다름"
    text = raw.decode("utf-8")
    E = {r["id"]: r for r in csv.DictReader((SRC / "v32_elements.csv").open(encoding="utf-8"))}

    # relation(s63_src 태그) -> 그 relation 이 가리키는 way·relation -> way 의 node
    rels = []
    for m in re.finditer(r'  <relation id="(\d+)" [^\n]*>\n(.*?)  </relation>\n', text, re.S):
        src = re.search(r'k="s63_src" v="(\w+)"', m.group(2))
        if src and src.group(1) in DROP:
            rels.append((src.group(1), int(m.group(1)), m.group(2)))
    rel_ids, way_ids = set(), set()
    for x, rid, body in rels:
        rel_ids.add(rid)
        for t, r in re.findall(r'<member type="(\w+)" ref="(\d+)"', body):
            (way_ids if t == "way" else rel_ids).add(int(r))
    # crosswalk 레인렛의 경계 way 도(레인렛 relation 이 rels 안에 있음)
    node_ids = set()
    for w in way_ids:
        node_ids |= {int(n) for n in re.findall(r'<nd ref="(\d+)"', block(text, "way", w).group(0))}
    # csv element 열과 맞는지
    for x in DROP:
        want = {int(n) for n in re.findall(r"\d+", E[x]["element"])}
        got = {rid for s, rid, _ in rels if s == x}
        assert want <= rel_ids | way_ids, (x, want, got)
    assert all(i >= 900001 for i in rel_ids | way_ids | node_ids), "원본 id 를 지우려 함"

    out = text
    n_mem = 0
    for g in sorted(rel_ids):
        pat = f'    <member type="relation" ref="{g}" role="regulatory_element" />\n'
        n_mem += out.count(pat)
        out = out.replace(pat, "")
    for i in sorted(rel_ids):
        out = out.replace(block(out, "relation", i).group(0), "")
    for i in sorted(way_ids):
        out = out.replace(block(out, "way", i).group(0), "")
    for i in sorted(node_ids):
        out = out.replace(block(out, "node", i).group(0), "")
    for i in node_ids:  # 지운 node 를 다른 way 가 쓰지 않음
        assert f'<nd ref="{i}"' not in out, i

    OUT_A.mkdir(parents=True, exist_ok=True)
    OUT_B.mkdir(parents=True, exist_ok=True)
    for f in ("lanelet2_map.osm", "map_projector_info.yaml"):
        shutil.copy2(SRC / f, OUT_A / f)
    (OUT_B / "lanelet2_map.osm").write_text(out, encoding="utf-8")
    shutil.copy2(SRC / "map_projector_info.yaml", OUT_B / "map_projector_info.yaml")
    with (OUT_B / "v32b_dropped.txt").open("w", encoding="utf-8") as f:
        f.write(f"v32b = v32a({SRC_SHA[:8]}) 에서 {', '.join(DROP)} 뺌\n")
        f.write(f"relation {sorted(rel_ids)}\nway {sorted(way_ids)}\nnode {len(node_ids)}개 {sorted(node_ids)}\n")
        f.write(f"도로 레인렛 member 줄 {n_mem}\n")
    for d in (OUT_A, OUT_B):
        h = hashlib.sha256((d / "lanelet2_map.osm").read_bytes()).hexdigest()
        print(d.relative_to(C), "lanelet2_map.osm", h)
    print("v32b 지움: relation", len(rel_ids), "way", len(way_ids), "node", len(node_ids), "member 줄", n_mem)


if __name__ == "__main__":
    main()
