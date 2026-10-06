import sys, pickle, copy, math, collections
import xml.etree.ElementTree as ET
from exp_lib import *
import os
R_T=sys.argv[1] if len(sys.argv)>1 else "9.0"
TAG=os.environ.get("TAG","")
R_T=R_T+TAG
OSM="/home/gyumin/campus_mobility_sim/map/data/processed/cbnu_internal_boundary_tags73.osm"
d=pickle.load(open(f"trial_{R_T}.pkl","rb"))
tree=ET.parse(OSM); root=tree.getroot()
for w in root.findall("way"):
    wid=w.get("id")
    if wid in d["new_ways"] and [n.get("ref") for n in w.findall("nd")]!=d["new_ways"][wid]:
        for nd in w.findall("nd"): w.remove(nd)
        tags=w.findall("tag")
        for t in tags: w.remove(t)
        for ref in d["new_ways"][wid]: ET.SubElement(w,"nd",ref=ref)
        for t in tags: w.append(t)
for nid,(e,n) in d["new_nodes"].items():
    lat,lon=to_ll(e,n)
    ET.SubElement(root,"node",id=nid,lat=f"{lat:.9f}",lon=f"{lon:.9f}",version="1")
used={nd.get("ref") for w in root.findall("way") for nd in w.findall("nd")}
for n in list(root.findall("node")):
    if n.get("id") not in used and not n.findall("tag"): root.remove(n)
xml=ET.tostring(root,encoding="unicode")
open(f"trial_{R_T}.osm","w").write(xml)
x=convert(xml)
open(f"trial_{R_T}.xodr","w").write(x)
xr=ET.fromstring(x)
nroad=sum(1 for _ in xr.iter("road")); nj=sum(1 for _ in xr.iter("junction"))
print("road/junction:",nroad,nj,"(기준 878/95)")
