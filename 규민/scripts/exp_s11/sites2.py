import pickle, math, collections
import numpy as np
from align import *
from exp_lib import _len,_sub
import sites as ST
g=pickle.load(open("g03.pkl","rb"))
twins=W.find_twins(roads)
ROUTE=ST.ROUTE
route_phys=set(ROUTE)|{twins[r] for r in ROUTE if r in twins}
print("경로 road 의 쌍둥이:",{r:twins.get(r) for r in sorted(ROUTE) if r in twins})
onroute=[x for x in g if x["road"] in route_phys]
print("f<0.3 문제 구간: 전체",len(g),"/ 경로 물리(자기+쌍둥이)",len(onroute))
# OSM 대응
P=(s*(R@np.array([[x["x"],x["y"]] for x in g]).T)).T+t
d,bi,bu=nearest(P)
rows=[]
for x,di,si,ui in zip(g,d,bi,bu):
    wid,i,ax,ay,bx,by=segs[si]
    nds=ways[wid]
    j=i if ui<0.5 else i+1          # 가장 가까운 노드 인덱스
    # 주변 ±8m 내 노드들의 꺾임 (클러스터 확인)
    cum=[0.0]
    for k in range(1,len(nds)): cum.append(cum[-1]+_len(_sub(nodes[nds[k]],nodes[nds[k-1]])))
    near=[(k,ST.turn_at(wid,k)) for k in range(len(nds)) if abs(cum[k]-cum[j])<=8 and ST.turn_at(wid,k)]
    tt=ST.turn_at(wid,j)
    big=max(near,key=lambda z:abs(z[1][0])) if near else None
    rows.append({"road":x["road"],"s":x["smin"],"f":x["f"],"R2":x["R"],"dist":di,"way":wid,"j":j,"node":nds[j],
                 "turn":tt,"fixed":ST.is_fixed(wid,j),"near":near,"big":big,"onroute":x["road"] in route_phys})
pickle.dump(rows,open("rows.pkl","wb"))
print("\n문제 구간별 (way, 최근접노드, 꺾임°, 인접세그먼트길이, 고정노드여부) — 경로 위 먼저")
for r in sorted(rows,key=lambda r:(not r["onroute"],r["f"]))[:25]:
    tt=r["turn"]; b=r["big"]
    print(f"{'경로' if r['onroute'] else '    '} road{r['road']} s={r['s']:.1f} f={r['f']:.2f} R2={r['R2']:.1f} way{r['way']} node{r['node']} "
          f"θ={tt[0]:+.1f} a={tt[1]:.1f} b={tt[2]:.1f} 고정={r['fixed']} | 주변최대꺾임 {b[1][0]:+.1f}°(idx{b[0]}) 거리오차{r['dist']:.2f}" if tt else f"road{r['road']} 끝점노드")
# 통계
fixed=sum(1 for r in rows if r["fixed"]); print("\n최근접 노드가 고정노드(공유/끝점):",fixed,"/",len(rows))
ths=[abs(r["big"][1][0]) for r in rows if r["big"]]
print("주변 최대 꺾임각 분포(°): min %.1f 중앙 %.1f max %.1f"%(min(ths),float(np.median(ths)),max(ths)))
mins=[min(r["big"][1][1],r["big"][1][2]) for r in rows if r["big"]]
print("그 노드의 짧은쪽 인접세그먼트(m): min %.1f 중앙 %.1f max %.1f"%(min(mins),float(np.median(mins)),max(mins)))
