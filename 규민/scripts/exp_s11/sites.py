import math, pickle, collections
import numpy as np
from align import *
from exp_lib import _len,_sub
elems={r.get("id"):r for r in xroot.iter("road")}
ROUTE={str(x) for x in [1247,1914,1356,1917,1355,1446,1172,1428,1240,1695,1239,1426,1238,1564,1278]}
# 노드 차수(way 출현 횟수 + 끝점 여부)
occ=collections.Counter()
for wid,nds in ways.items():
    for k,n in enumerate(nds):
        occ[n]+=1+(1 if k in (0,len(nds)-1) else 0)*0
def node_deg(n):
    # 다른 way 와 공유되거나 way 끝점이면 "고정 노드"
    return occ[n]
def turn_at(wid,i):
    nds=ways[wid]
    if i<=0 or i>=len(nds)-1: return None
    p0,p1,p2=nodes[nds[i-1]],nodes[nds[i]],nodes[nds[i+1]]
    return turn_deg(p0,p1,p2), _len(_sub(p1,p0)), _len(_sub(p2,p1))
def is_fixed(wid,i):
    nds=ways[wid]
    return i<=0 or i>=len(nds)-1 or occ[nds[i]]>1

def scan_sites(win=2.0):
    m=int(round(win/2/0.25)); sites=[]
    for rid,r in roads.items():
        if r["junction"]!="-1": continue
        offs,secs=W.parse_lanes(elems[rid])
        if not any(e["type"]=="sidewalk" for e in W.lane_edges(offs,secs,0.0)): continue
        S_=W.sample_road(r,0.25)
        cur=None
        for i in range(m,len(S_)-m):
            ds=S_[i+m]["s"]-S_[i-m]["s"]; kw=(S_[i+m]["hdg"]-S_[i-m]["hdg"])/ds
            e=[e for e in W.lane_edges(offs,secs,S_[i]["s"]) if e["type"]=="sidewalk"]
            if not e: continue
            tf=e[0]["t_out"]; f=1-kw*tf if kw*tf>0 else 1.0
            rec=(S_[i]["s"],f,(1/abs(kw) if abs(kw)>1e-9 else 1e9),S_[i]["x"],S_[i]["y"])
            sites.append((rid,)+rec)
    return sites
sites=scan_sites()
pickle.dump(sites,open("sites.pkl","wb"))
def group(sites,fmax):
    out=[];cur=None
    for rid,s_,f,R_,x,y in sites:
        if f<fmax:
            if cur and cur["road"]==rid and s_-cur["s1"]<=0.6:
                cur["s1"]=s_
                if f<cur["f"]: cur.update(f=f,R=R_,x=x,y=y,smin=s_)
            else:
                if cur: out.append(cur)
                cur={"road":rid,"s0":s_,"s1":s_,"f":f,"R":R_,"x":x,"y":y,"smin":s_}
        else:
            if cur: out.append(cur); cur=None
    if cur: out.append(cur)
    return out
print("전 망 plain 인도 road:",len({s_[0] for s_ in sites}),"메시스케일 f 임계별 문제 구간(road 반쪽 단위)")
for fmax in (0.0,0.25,0.3,0.4):
    g=group(sites,fmax)
    onr=[x for x in g if x["road"] in ROUTE]
    # 쌍둥이 환산 경로 위(1247<->1120 만 알려진 경우) 별도
    print(f"f<{fmax}: R2<{6.15/(1-fmax):.2f}m  전체 {len(g)}곳 / 경로 자기road {len(onr)}곳")
pickle.dump(group(sites,0.3),open("g03.pkl","wb"))
