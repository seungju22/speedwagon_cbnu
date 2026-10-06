import sys, math, collections, pickle
import numpy as np
import xml.etree.ElementTree as ET
from scan_generic import *
import geo_calibrate as G
from analyze_geometry import parse_junctions
def load_osm(path):
    root=ET.parse(path).getroot()
    nodes={n.get("id"):to_en(float(n.get("lat")),float(n.get("lon"))) for n in root.findall("node")}
    ways={w.get("id"):[n.get("ref") for n in w.findall("nd")] for w in root.findall("way")}
    return nodes,ways
def build_segs(nodes,ways):
    segs=[]
    for wid,nds in ways.items():
        for i in range(len(nds)-1):
            a=nodes[nds[i]];b=nodes[nds[i+1]]; segs.append((wid,i,a[0],a[1],b[0],b[1]))
    return segs,np.array([[s[2],s[3],s[4],s[5]] for s in segs])
def nearest(S,P):
    A=S[:,0:2];B=S[:,2:4];D=B-A;L2=(D**2).sum(1)+1e-12
    bd=np.full(len(P),1e18);bi=np.zeros(len(P),int);bu=np.zeros(len(P))
    for k0 in range(0,len(P),1000):
        p=P[k0:k0+1000][:,None,:]; u=np.clip(((p-A)*D).sum(2)/L2,0,1); q=A+u[...,None]*D
        d=np.linalg.norm(p-q,axis=2); j=d.argmin(1); ar=np.arange(len(j))
        bd[k0:k0+1000]=d[ar,j];bi[k0:k0+1000]=j;bu[k0:k0+1000]=u[ar,j]
    return bd,bi,bu
def align(xodr_path,nodes,S):
    xr=ET.parse(xodr_path).getroot(); roads=parse_roads(xr); js=parse_junctions(xr)
    jxy=G.junction_xodr_locations(roads,js); src=[];dst=[]
    for jid,j in js.items():
        if j["name"] in nodes and jid in jxy: src.append(jxy[jid]); dst.append(nodes[j["name"]])
    R,s,t,ang,res=G.fit_similarity(src,dst)
    pts=[]
    for rid,r in roads.items():
        if r["junction"]!="-1": continue
        smp=W.sample_road(r,0.25)
        for i in range(0,len(smp),8): pts.append((smp[i]["x"],smp[i]["y"]))
    X=np.array(pts)
    for it in range(10):
        P=(s*(R@X.T)).T+t; d,bi,bu=nearest(S,P); keep=d<np.percentile(d,80)
        Q=S[bi][:,0:2]+bu[:,None]*(S[bi][:,2:4]-S[bi][:,0:2]); R,s,t,ang,_=G.fit_similarity(X[keep],Q[keep])
    return R,s,t,float(np.median(d))
if __name__=="__main__":
    xod=sys.argv[1]; osmp=sys.argv[2]
    nodes,ways=load_osm(osmp); segs,S=build_segs(nodes,ways)
    R,s,t,med=align(xod,nodes,S); print("정렬 median %.2f m"%med)
    occ=collections.Counter(n for nds in ways.values() for n in nds)
    roads,sites=scan_xodr(xod); g=group(sites,0.3)
    # 사이트 좌표
    xy={ (rid,round(s_,2)):(x,y) for rid,s_,f,R_,x,y in sites}
    P=np.array([xy[(x["road"],round(x["smin"],2))] for x in g]); P=(s*(R@P.T)).T+t
    d,bi,bu=nearest(S,P)
    cat=collections.Counter(); detail=[]
    for x,di,si,ui in zip(g,d,bi,bu):
        wid,i,*_=segs[si]; nds=ways[wid]
        # 반경 12m 안 노드들: 새 노드(호), 고정, 원본 꺾임
        cum=[0.0]
        for k in range(1,len(nds)): cum.append(cum[-1]+math.dist(nodes[nds[k]],nodes[nds[k-1]]))
        pos=cum[i]+ui*(cum[i+1]-cum[i])
        near=[k for k in range(len(nds)) if abs(cum[k]-pos)<=8]
        has_new=any(int(nds[k])>=9000000000000 for k in near)
        has_fixed=any((k in (0,len(nds)-1)) or occ[nds[k]]>1 for k in near)
        kind=("호(신규노드)근처 " if has_new else "")+("고정노드근처" if has_fixed else "")
        kind=kind or "원본 내부노드만"
        cat[kind]+=1
        detail.append((x["f"],x["road"],wid,kind,di))
    print(dict(cat))
    for f,rid,wid,kind,di in sorted(detail)[:40]: print(f"f={f:+.2f} road{rid} way{wid} {kind} 거리{di:.2f}")
