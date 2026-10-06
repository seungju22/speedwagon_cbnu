import sys, math, pickle, collections
import numpy as np
import xml.etree.ElementTree as ET
from exp_lib import *
R_T=sys.argv[1] if len(sys.argv)>1 else "10.0"
d=pickle.load(open(f"trial_{R_T}.pkl","rb"))
# 원본(교내 485way 파일 기준 노드 좌표)
osm=ET.parse("/home/gyumin/campus_mobility_sim/map/data/processed/cbnu_internal_boundary_tags73.osm").getroot()
nodes={n.get("id"):to_en(float(n.get("lat")),float(n.get("lon"))) for n in osm.findall("node")}
ways={w.get("id"):[n.get("ref") for n in w.findall("nd")] for w in osm.findall("way")}
nn=d["new_nodes"]
def pos(n): return nodes[n] if n in nodes else nn[n]
# 건물: 원본 raw OSM(건물 way 399)
raw=ET.parse("/home/gyumin/campus_mobility_sim/map/data/raw/cbnu_campus.osm").getroot()
rn={n.get("id"):to_en(float(n.get("lat")),float(n.get("lon"))) for n in raw.findall("node")}
polys=[]
for w in raw.findall("way"):
    if any(t.get("k")=="building" for t in w.findall("tag")):
        pts=[rn[n.get("ref")] for n in w.findall("nd") if n.get("ref") in rn]
        if len(pts)>=4: polys.append(np.array(pts))
print("건물 폴리곤",len(polys))
E=[]  # 건물 변 (ax,ay,bx,by)
for pg in polys:
    for i in range(len(pg)-1): E.append((*pg[i],*pg[i+1]))
E=np.array(E)
def dist_edges(P):
    A=E[:,0:2];B=E[:,2:4];D=B-A;L2=(D**2).sum(1)+1e-12
    out=[]
    for p in P:
        u=np.clip(((p-A)*D).sum(1)/L2,0,1); q=A+u[:,None]*D
        out.append(np.linalg.norm(p-q,axis=1).min())
    return np.array(out)
def inside(p):
    x,y=p
    for pg in polys:
        if pg[:,0].min()<=x<=pg[:,0].max() and pg[:,1].min()<=y<=pg[:,1].max():
            c=False
            for i in range(len(pg)-1):
                x1,y1=pg[i];x2,y2=pg[i+1]
                if (y1>y)!=(y2>y) and x<(x2-x1)*(y-y1)/(y2-y1+1e-12)+x1: c=not c
            if c: return True
    return False
def polyline_dist(p,pl):
    best=1e9
    for a,b in zip(pl,pl[1:]):
        ax,ay=a;bx,by=b;dx,dy=bx-ax,by-ay;L2=dx*dx+dy*dy+1e-12
        u=max(0,min(1,((p[0]-ax)*dx+(p[1]-ay)*dy)/L2)); best=min(best,math.hypot(p[0]-ax-u*dx,p[1]-ay-u*dy))
    return best
def dense(pl,step=0.5):
    out=[]
    for a,b in zip(pl,pl[1:]):
        L=math.dist(a,b);n=max(int(L/step),1)
        for k in range(n): out.append((a[0]+(b[0]-a[0])*k/n,a[1]+(b[1]-a[1])*k/n))
    out.append(pl[-1]); return out
HALF=6.15
devs=[];newin=0;newnear=0;oldnear=0;res=[]
for wid,new in d["new_ways"].items():
    old=ways[wid]
    if new==old: continue
    opl=[nodes[n] for n in old]; npl=[pos(n) for n in new]
    # 변경 구간: 처음/마지막으로 달라지는 위치
    i0=next(i for i,(a,b) in enumerate(zip(old,new)) if a!=b)
    j0=next(i for i,(a,b) in enumerate(zip(reversed(old),reversed(new))) if a!=b)
    seg_old=opl[max(i0-1,0):len(opl)-j0+1]; seg_new=npl[max(i0-1,0):len(npl)-j0+1]
    do=dense(seg_old); dn=dense(seg_new)
    h1=max(polyline_dist(p,seg_old) for p in dn)      # 새 도로 -> 원본 최대 거리
    h2=max(polyline_dist(p,seg_new) for p in do)      # 원본 -> 새 도로
    hd=max(h1,h2); devs.append((hd,wid))
    # 건물
    dnew=dist_edges(np.array(dn)); dold=dist_edges(np.array(do))
    ins=any(inside(p) for p in dn); ins_old=any(inside(p) for p in do)
    nm=dnew.min(); om=dold.min()
    res.append((wid,hd,nm,om,ins,ins_old))
devs.sort(reverse=True)
print(f"변경 way {len(res)}개. 원본 대비 최대 이탈(Hausdorff, m): 중앙 {np.median([r[1] for r in res]):.2f} 최대 {max(r[1] for r in res):.2f}; >1m {sum(1 for r in res if r[1]>1)}개, >2m {sum(1 for r in res if r[1]>2)}개, >3m {sum(1 for r in res if r[1]>3)}개")
print("건물 중심선 내부 침범: 새 도로 %d곳 (원본이 이미 내부 %d곳)"%(sum(1 for r in res if r[4]),sum(1 for r in res if r[5])))
print("건물과 중심선 거리 <6.15m(인도 포함 폭 절반): 새 %d곳 / 원본 %d곳"%(sum(1 for r in res if r[2]<HALF),sum(1 for r in res if r[3]<HALF)))
worse=[r for r in res if r[2]<HALF and r[2]<r[3]-0.05]
print("새로 악화(새 거리<6.15 이고 원본보다 0.05m 이상 가까움): %d곳"%len(worse))
for r in sorted(worse,key=lambda r:r[2])[:12]: print(f"  way{r[0]} 이탈{r[1]:.2f} 건물거리 새{r[2]:.2f} 원본{r[3]:.2f}")
newinside=[r for r in res if r[4] and not r[5]]
print("새로 건물 내부로 들어간 way:",[(r[0],round(r[1],2)) for r in newinside])
