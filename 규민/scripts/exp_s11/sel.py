import pickle, math, collections
import numpy as np
from align import *
from exp_lib import _len,_sub
import sites as ST
rows=pickle.load(open("rows.pkl","rb"))
# 문제 지점에 해당하는 (way, 노드 인덱스) 집합: 주변 ±8m 내 꺾임 노드 중 |θ|>=10° 인 것 전부를 원인 후보로
cause=set()
for r in rows:
    for k,tt in r["near"]:
        if abs(tt[0])>=10: cause.add((r["way"],k))
allc=[]   # 모든 내부 비고정 노드
tagged=set()
for n in root.findall("node"):
    if n.findall("tag"): tagged.add(n.get("id"))
print("태그 있는 노드:",len(tagged),"(내 way 에서 쓰이는 것:",sum(1 for w in ways.values() for n in w if n in tagged),")")
for wid,nds in ways.items():
    for k in range(1,len(nds)-1):
        if ST.is_fixed(wid,k): continue
        tt=ST.turn_at(wid,k)
        allc.append((wid,k,tt[0],tt[1],tt[2],(wid,k) in cause))
print("내부 비고정 노드",len(allc),"/ 원인 후보로 지목된 것",sum(1 for a in allc if a[5]))
print("\nθ 구간별 (전체 노드 수, 문제 원인 노드 수)")
bins=[0,3,6,10,15,20,30,45,60,80,100,180]
for lo,hi in zip(bins,bins[1:]):
    tot=[a for a in allc if lo<=abs(a[2])<hi]; pr=[a for a in tot if a[5]]
    print(f"{lo:>3}~{hi:<3}° 전체 {len(tot):4d} 문제 {len(pr):3d}")
print("\n문제 원인 노드 중 θ<30° 목록(짧은쪽 세그먼트 포함)")
for a in sorted([a for a in allc if a[5] and abs(a[2])<30],key=lambda a:abs(a[2])):
    print(f"way{a[0]} idx{a[1]} θ={a[2]:+.1f} a={a[3]:.1f} b={a[4]:.1f}")
