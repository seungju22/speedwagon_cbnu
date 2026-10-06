from exp_lib import *
import xml.etree.ElementTree as ET
chain=["2261340221","3957495734","3957495733","4748296079","3957495732","4748296078","3957495731","4748296080"]
root=ET.parse("/home/gyumin/campus_mobility_sim/map/data/processed/cbnu_internal_boundary_tags73.osm").getroot()
nodes={n.get("id"):to_en(float(n.get("lat")),float(n.get("lon"))) for n in root.findall("node")}
pl=[nodes[c] for c in chain]
osm=make_osm([pl])
x=convert(osm)
for m in measure(x): print(m)
