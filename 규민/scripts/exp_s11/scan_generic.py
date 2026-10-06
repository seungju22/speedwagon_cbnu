import sys, math, xml.etree.ElementTree as ET
from exp_lib import *
def scan_xodr(path, win=2.0):
    xr=ET.parse(path).getroot(); roads=parse_roads(xr); elems={r.get("id"):r for r in xr.iter("road")}
    m=int(round(win/2/0.25)); sites=[]
    for rid,r in roads.items():
        if r["junction"]!="-1": continue
        offs,secs=W.parse_lanes(elems[rid])
        if not any(e["type"]=="sidewalk" for e in W.lane_edges(offs,secs,0.0)): continue
        S_=W.sample_road(r,0.25)
        for i in range(m,len(S_)-m):
            ds=S_[i+m]["s"]-S_[i-m]["s"]; kw=(S_[i+m]["hdg"]-S_[i-m]["hdg"])/ds
            e=[e for e in W.lane_edges(offs,secs,S_[i]["s"]) if e["type"]=="sidewalk"]
            tf=e[0]["t_out"]; f=1-kw*tf if kw*tf>0 else 1.0
            sites.append((rid,S_[i]["s"],f,(1/abs(kw) if abs(kw)>1e-9 else 1e9),S_[i]["x"],S_[i]["y"]))
    return roads,sites
def group(sites,fmax):
    out=[];cur=None
    for rid,s_,f,R_,x,y in sites:
        if f<fmax:
            if cur and cur["road"]==rid and s_-cur["s1"]<=0.6:
                cur["s1"]=s_
                if f<cur["f"]: cur.update(f=f,R=R_,smin=s_)
            else:
                if cur: out.append(cur)
                cur={"road":rid,"s0":s_,"s1":s_,"f":f,"R":R_,"smin":s_}
        else:
            if cur: out.append(cur); cur=None
    if cur: out.append(cur)
    return out
if __name__=="__main__":
    roads,sites=scan_xodr(sys.argv[1])
    for fmax in (0.0,0.3):
        g=group(sites,fmax); print(f"f<{fmax}: {len(g)}곳; 최악 f={min([x['f'] for x in g] or [1]):.2f}")
