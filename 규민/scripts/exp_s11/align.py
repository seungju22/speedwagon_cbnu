# xodr(기준선) <-> OSM 폴리라인 ICP 정렬. 스크래치패드 실험(프로젝트 파일 아님)
import math, sys, pickle
import numpy as np
import xml.etree.ElementTree as ET
from exp_lib import *
import geo_calibrate as G

OSM="/home/gyumin/campus_mobility_sim/map/data/processed/cbnu_internal_boundary_tags73.osm"
XODR="/home/gyumin/campus_mobility_sim/map/maps/cbnu_internal_only_localtm_tags73.xodr"
root=ET.parse(OSM).getroot()
nodes={n.get("id"):to_en(float(n.get("lat")),float(n.get("lon"))) for n in root.findall("node")}
ways={}
for w in root.findall("way"):
    ways[w.get("id")]=[n.get("ref") for n in w.findall("nd")]
segs=[]   # (way, i, ax, ay, bx, by)
for wid,nds in ways.items():
    for i in range(len(nds)-1):
        a=nodes[nds[i]]; b=nodes[nds[i+1]]
        segs.append((wid,i,a[0],a[1],b[0],b[1]))
S=np.array([[s[2],s[3],s[4],s[5]] for s in segs])
def nearest(P):
    """점 P(N,2) -> (거리, seg index, 세그먼트 위 매개변수 u)"""
    A=S[:,0:2]; B=S[:,2:4]; D=B-A; L2=(D**2).sum(1)+1e-12
    best=np.full(len(P),1e18); bi=np.zeros(len(P),int); bu=np.zeros(len(P))
    for k0 in range(0,len(P),2000):
        p=P[k0:k0+2000][:,None,:]
        u=np.clip(((p-A)*D).sum(2)/L2,0,1)
        q=A+u[...,None]*D
        d=np.linalg.norm(p-q,axis=2)
        j=d.argmin(1); best[k0:k0+2000]=d[np.arange(len(j)),j]; bi[k0:k0+2000]=j; bu[k0:k0+2000]=u[np.arange(len(j)),j]
    return best,bi,bu

xroot=ET.parse(XODR).getroot()
roads=parse_roads(xroot); junctions={}
from analyze_geometry import parse_junctions
junctions=parse_junctions(xroot)
# 초기: 세션9 방식 junction 적합
jxy=G.junction_xodr_locations(roads,junctions)
src=[];dst=[]
for jid,j in junctions.items():
    if j["name"] in nodes and jid in jxy:
        src.append(jxy[jid]); dst.append(nodes[j["name"]])
R,s,t,ang,res=G.fit_similarity(src,dst)
print("초기 junction 적합: n=%d s=%.6f rot=%.3f RMS=%.2f"%(len(src),s,ang,math.sqrt((res**2).mean())))
# plain road 기준선 점(2m)
pts=[];meta=[]
for rid,r in roads.items():
    if r["junction"]!="-1": continue
    smp=W.sample_road(r,0.25)
    for i in range(0,len(smp),8):
        pts.append((smp[i]["x"],smp[i]["y"])); meta.append((rid,smp[i]["s"]))
X=np.array(pts)
for it in range(12):
    P=(s*(R@X.T)).T+t
    d,bi,bu=nearest(P)
    keep=d<np.percentile(d,80)
    Q=S[bi][:,0:2]+bu[:,None]*(S[bi][:,2:4]-S[bi][:,0:2])
    R,s,t,ang,_=G.fit_similarity(X[keep],Q[keep])
    print(f"it{it} 점 {len(X)} median={np.median(d):.3f} mean80={d[keep].mean():.3f} s={s:.6f} rot={ang:.3f}")
pickle.dump({"R":R,"s":s,"t":t},open("align.pkl","wb"))
