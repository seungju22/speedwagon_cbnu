#!/usr/bin/env python3
# I6-3(2026-10-13): Autoware 경로 레인렛 id(i63_route_get.py 출력)를 지도 xodr_road 태그로 road 열로 바꿔 pairs30 xodr 노선과 비교
# 출력 마지막 줄: ROUTE_SAME 또는 ROUTE_DIFF. 사용: i63_routechk.py <레인렛 id 파일> <쌍>
import json, os, re, sys
C = os.path.expanduser('~/campus_mobility_sim')
osm = os.path.expanduser('~/autoware_data/maps/OpenDriveMap/lanelet2_map.osm')
road = {}; cur = None
for line in open(osm):
    m = re.match(r'\s*<relation id="(\d+)"', line)
    if m: cur = int(m.group(1)); continue
    m = re.search(r'k="xodr_road" v="(\d+)"', line)
    if m and cur is not None: road[cur] = int(m.group(1))
ids = [int(x) for x in open(sys.argv[1]).read().split()]
rs = []
for i in ids:
    r = road.get(i, 'L%d?' % i)
    if not rs or rs[-1] != r: rs.append(r)
ref = json.load(open(C + '/map/build_v31_s54/pairs30.json'))[sys.argv[2]]['roads']
print('lanelets', len(ids)); print('autoware', rs); print('xodr    ', ref)
print('ROUTE_SAME' if rs == ref and ids else 'ROUTE_DIFF')
