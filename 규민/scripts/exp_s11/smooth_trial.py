# 평활화 설계 프로토타입(스크래치패드). 정식 스크립트는 승인 후 map/scripts/ 에 작성.
import math, pickle, collections, csv, sys
import numpy as np
import xml.etree.ElementTree as ET
from align import *                 # nodes, ways, occ 계산은 sites 에서
from exp_lib import _len,_sub
import sites as ST

R_T=float(sys.argv[1]) if len(sys.argv)>1 else 9.0
import os
D_MAX=float(os.environ.get("D_MAX","1e9"))
R_ACC=float(os.environ.get("R_ACC","9.0"))
M_J=2.0
TAG=os.environ.get("TAG","")
H=2.0            # 호 위 노드 간격(m) = CARLA vertex_distance
TH_CAUSE=8.0     # 원인 후보 최소 꺾임각
TH_ABSORB=10.0   # 이 미만의 비고정·비목표 노드는 필렛 접선영역 안에서 흡수(삭제)
rows=pickle.load(open("rows.pkl","rb"))

def path_cum(nds):
    c=[0.0]
    for k in range(1,len(nds)): c.append(c[-1]+_len(_sub(nodes[nds[k]],nodes[nds[k-1]])))
    return c
targets=collections.defaultdict(set)      # way -> node index set
for r in rows:
    wid=r["way"]; nds=ways[wid]; cum=path_cum(nds); j=r["j"]
    for k in range(1,len(nds)-1):
        if abs(cum[k]-cum[j])<=8 and not ST.is_fixed(wid,k):
            tt=ST.turn_at(wid,k)
            if abs(tt[0])>=TH_CAUSE and R_T*(1/math.cos(math.radians(abs(tt[0]))/2)-1)<=D_MAX: targets[wid].add(k)
print(f"R_target={R_T}  대상 way {len(targets)}개, 대상 노드 {sum(len(v) for v in targets.values())}개")

new_ways={}; log=[]; unresolved=[]
newid=[9000000000000]
new_nodes={}
for wid,nds in ways.items():
    if wid not in targets:
        new_ways[wid]=list(nds); continue
    cum=path_cum(nds); T_idx=sorted(targets[wid])
    th={k:math.radians(ST.turn_at(wid,k)[0]) for k in T_idx}
    Rk={k:R_T for k in T_idx}
    # 접선 길이 제약: 이웃 blocking 노드(고정, 목표, |θ|>=TH_ABSORB 비목표)까지
    def blocker(k,direction):
        j=k+direction
        while 0<=j<len(nds):
            if j in (0,len(nds)-1) or ST.is_fixed(wid,j): return j,"fixed"
            if j in targets[wid]: return j,"target"
            if abs(ST.turn_at(wid,j)[0])>=TH_ABSORB: return j,"kink"
            j+=direction
        return None,None
    for _ in range(50):
        changed=False
        for k in T_idx:
            for d in (-1,1):
                j,kind=blocker(k,d)
                if j is None: continue
                dist=abs(cum[j]-cum[k])-(M_J if kind=='fixed' else 0.0)
                Tk=Rk[k]*math.tan(abs(th[k])/2)
                Tj=(Rk[j]*math.tan(abs(th[j])/2)) if kind=="target" else 0.0
                if Tk+Tj>dist+1e-9:
                    f=dist/(Tk+Tj)*0.999
                    Rk[k]*=f
                    if kind=="target": Rk[j]*=f
                    changed=True
        if not changed: break
    # 실제 필렛 적용
    out=[nds[0]]; absorbed=set()
    zones=[]
    for k in T_idx:
        Tk=Rk[k]*math.tan(abs(th[k])/2)
        zones.append((cum[k]-Tk,cum[k]+Tk,k))
    for k in range(1,len(nds)-1):
        if any(lo<cum[k]<hi and k!=kk for lo,hi,kk in zones) and k not in targets[wid]:
            absorbed.add(k)
    for k in range(1,len(nds)-1):
        if k in absorbed: continue
        if k in targets[wid] and Rk[k]<R_ACC:
            unresolved.append((wid,k,Rk[k],math.degrees(th[k]))); out.append(nds[k]); continue
        if k in targets[wid]:
            p0=nodes[nds[k-1]] if (k-1) not in absorbed else None
            # 인접 실제 노드(흡수 안 된 이전/다음)
            a=k-1
            while a in absorbed: a-=1
            b=k+1
            while b in absorbed: b+=1
            pts,T,dev=fillet_vertex(nodes[nds[a]],nodes[nds[k]],nodes[nds[b]],Rk[k],H) if False else (None,None,None)
            # fillet 은 꼭짓점 기준 방향이 흡수된 노드 쪽 직선이므로 원 세그먼트 방향(인접 원본노드) 사용
            pts,T,dev=fillet_vertex(nodes[nds[k-1]],nodes[nds[k]],nodes[nds[k+1]],Rk[k],H)
            ids=[]
            for p in pts:
                newid[0]+=1; nid=str(newid[0]); new_nodes[nid]=p; ids.append(nid)
            out.extend(ids)
            log.append({"way":wid,"idx":k,"node":nds[k],"theta":math.degrees(th[k]),"R":Rk[k],"T":T,"dev":dev,
                        "n_new":len(ids),"absorbed":[nds[x] for x in absorbed if abs(cum[x]-cum[k])<=T]})
            if Rk[k]<R_T-0.01: unresolved.append((wid,k,Rk[k],math.degrees(th[k])))
        else:
            out.append(nds[k])
    out.append(nds[-1])
    new_ways[wid]=out
pickle.dump({"log":log,"new_ways":new_ways,"new_nodes":new_nodes,"unresolved":unresolved},open(f"trial_{R_T}{TAG}.pkl","wb"))
devs=[l["dev"] for l in log]; Ts=[l["T"] for l in log]
print(f"필렛 {len(log)}개, 흡수 노드 {sum(len(l['absorbed']) for l in log)}개, 신규 노드 {sum(l['n_new'] for l in log)}개")
print(f"꼭짓점-호 이탈(m): 중앙 {np.median(devs):.2f} 최대 {max(devs):.2f} / 접선길이 최대 {max(Ts):.1f}m")
print(f"반경 축소(목표 미달) {len(unresolved)}개: 최소 R={min([u[2] for u in unresolved] or [R_T]):.2f}")
