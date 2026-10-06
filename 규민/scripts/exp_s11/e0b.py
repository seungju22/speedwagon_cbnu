from exp_lib import *
import xml.etree.ElementTree as ET
chain=["2261340221","3957495734","3957495733","4748296079","3957495732","4748296078","3957495731","4748296080"]
root=ET.parse("/home/gyumin/campus_mobility_sim/map/data/processed/cbnu_internal_boundary_tags73.osm").getroot()
nodes={n.get("id"):to_en(float(n.get("lat")),float(n.get("lon"))) for n in root.findall("node")}
pl=[nodes[c] for c in chain]
print("원본 체인(격리 변환) R2=2m창 최소반경 f2=2m창 기준 안쪽인도 최소f")
for m in measure2(convert(make_osm([pl]))): print({k:(round(v,3) if isinstance(v,float) else v) for k,v in m.items()})
print("\n합성 단일 꺾임 28.5 a=b=12 + 필렛")
base=[(-12,0),(0,0),(12*math.cos(math.radians(28.5)),12*math.sin(math.radians(28.5)))]
for R in (0,8,12,16,20):
    pl2,_=fillet_polyline(base,{1:R},2.0) if R else (base,[])
    ms=[m for m in measure2(convert(make_osm([pl2]))) if m["has_sw"]]
    print(R, "R2=%.2f f2=%.2f Ran=%.2f"%(min(m["R2"] for m in ms),min(m["f2"] for m in ms),min(m["Ran"] for m in ms)))
