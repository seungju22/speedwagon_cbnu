import sys, numpy as np
from classify_rem import *
xod,osmp=sys.argv[1],sys.argv[2]
nodes,ways=load_osm(osmp); segs,S=build_segs(nodes,ways)
R,s,t,med=align(xod,nodes,S)
roads,sites=scan_xodr(xod)
Ri=np.linalg.inv(R)
def to_xodr(e,n): 
    v=Ri@((np.array([e,n])-t)/s); return v
for label,nid in (("s=80 커브 노드 3957495733","3957495733"),("s=95 커브 노드 3957495732","3957495732")):
    if nid in nodes:
        p=nodes[nid]; c=to_xodr(*p)
    else:
        c=None
    # 원본 노드가 흡수/삭제됐을 수 있으므로 원본 좌표로 조회
    import xml.etree.ElementTree as ET
    orig=ET.parse("/home/gyumin/campus_mobility_sim/map/data/processed/cbnu_internal_boundary_tags73.osm").getroot()
    for n in orig.findall("node"):
        if n.get("id")==nid: p=to_en(float(n.get("lat")),float(n.get("lon"))); break
    c=to_xodr(*p)
    near=[(f,rid,s_,R_) for rid,s_,f,R_,x,y in sites if (x-c[0])**2+(y-c[1])**2<=5.0**2]
    if near:
        f,rid,s_,R_=min(near); print(f"{label}: 반경5m 내 기준선 샘플 {len(near)}개, 최악 f={f:+.2f} (R2={R_:.1f}m) road{rid} s={s_:.1f}")
    else: print(label,": 주변 인도 road 샘플 없음")
