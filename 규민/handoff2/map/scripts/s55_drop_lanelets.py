#!/usr/bin/env python3
# 맵세션55 2번(안 나): Lanelet2 지도 사본에서 지정한 xodr road 의 lanelet 만 뺀다(xodr 은 그대로, 되돌리려면 원본 폴더를 쓰면 된다)
# - l1_build_lanelet2.py 출력 폴더를 읽어 새 폴더에 쓴다. 원본 폴더는 안 건드림
# - 빼는 것: xodr_road 가 목록에 든 lanelet relation, 그 relation 의 left/right way(이 지도는 lanelet 마다 way 2개를 따로 씀),
#   그 way 들에만 쓰인 node. 다른 lanelet 이 같이 쓰는 끝점 node 는 남긴다
# - 남은 줄은 바이트 그대로(줄 단위로 빼기만 함). map_projector_info.yaml 은 그대로 복사
# - l1_meta.json 사본: dropped 에 뺀 lane 을 더하고 lanelet/lanelet_last/pieces 에서 지움(l1_eval.py 가 읽게)
# 빌더에서 회차를 빼는 것(--keep-uturn 없음)과 달리 끝점 공유 평균을 다시 하지 않으므로 이웃 lanelet 모양이 안 바뀐다
# 사용: python3 map/scripts/s55_drop_lanelets.py <원본 폴더> <새 폴더> <road> [<road> ...]
import json
import re
import shutil
import sys
from pathlib import Path

src, dst = Path(sys.argv[1]), Path(sys.argv[2])
roads = {int(x) for x in sys.argv[3:]}
lines = (src / "lanelet2_map.osm").read_text().splitlines(keepends=True)

# 블록 나누기: node / way / relation 은 여는 줄 ~ 닫는 줄
blocks = []          # (kind, id, start, end)
i = 0
while i < len(lines):
    m = re.match(r'\s*<(node|way|relation) id="(\d+)"', lines[i])
    if m:
        kind, oid = m.group(1), m.group(2)
        if lines[i].rstrip().endswith("/>"):
            blocks.append((kind, oid, i, i))
        else:
            j = i
            while f"</{kind}>" not in lines[j]:
                j += 1
            blocks.append((kind, oid, i, j))
            i = j
    i += 1
body = {(k, o): "".join(lines[a:b + 1]) for k, o, a, b in blocks}
rel_drop, way_drop = set(), set()
for k, o, a, b in blocks:
    if k != "relation":
        continue
    t = body[(k, o)]
    m = re.search(r'k="xodr_road" v="(\d+)"', t)
    if m and int(m.group(1)) in roads:
        rel_drop.add(o)
        way_drop.update(re.findall(r'<member type="way" ref="(\d+)"', t))
# 남는 relation 이 쓰는 way, 남는 way 가 쓰는 node
way_keep_ref = set()
for k, o, a, b in blocks:
    if k == "relation" and o not in rel_drop:
        way_keep_ref.update(re.findall(r'<member type="way" ref="(\d+)"', body[(k, o)]))
assert not (way_drop & way_keep_ref), "뺄 way 를 다른 lanelet 이 같이 씀"
node_keep_ref = set()
node_drop_ref = set()
for k, o, a, b in blocks:
    if k == "way":
        refs = re.findall(r'<nd ref="(\d+)"', body[(k, o)])
        (node_drop_ref if o in way_drop else node_keep_ref).update(refs)
node_drop = node_drop_ref - node_keep_ref
skip = set()
for k, o, a, b in blocks:
    if (k == "relation" and o in rel_drop) or (k == "way" and o in way_drop) or (k == "node" and o in node_drop):
        skip.update(range(a, b + 1))
dst.mkdir(parents=True, exist_ok=True)
(dst / "lanelet2_map.osm").write_text("".join(l for n, l in enumerate(lines) if n not in skip))
shutil.copy2(src / "map_projector_info.yaml", dst / "map_projector_info.yaml")
meta = json.loads((src / "l1_meta.json").read_text())
gone = sorted(k for k in meta["lanelet"] if int(k.split(",")[0]) in roads)
for k in gone:
    meta["lanelet"].pop(k, None)
    meta.get("lanelet_last", {}).pop(k, None)
    meta.get("pieces", {}).pop(k, None)
    meta["dropped"].append([int(x) for x in k.split(",")])
meta["s55_dropped_roads"] = sorted(roads)
meta["s55_source"] = str(src)
(dst / "l1_meta.json").write_text(json.dumps(meta, ensure_ascii=False))
print(f"뺀 road {sorted(roads)}: lanelet {len(rel_drop)}, way {len(way_drop)}, node {len(node_drop)}(공유 끝점 남김), 남은 줄 {len(lines) - len(skip)}/{len(lines)}")
