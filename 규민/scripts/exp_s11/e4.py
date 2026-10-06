from exp_lib import *
import xml.etree.ElementTree as ET
root=ET.parse("/home/gyumin/campus_mobility_sim/map/maps/cbnu_internal_only_localtm_tags73.xodr").getroot()
roads=parse_roads(root); elems={r.get("id"):r for r in root.iter("road")}
def prof(rid, lo, hi, win=2.0):
    r=roads[rid]; offs,secs=W.parse_lanes(elems[rid]); S=W.sample_road(r,0.25)
    h=S[1]["s"]-S[0]["s"]; m=int(round(win/2/h)); best=None
    for i in range(m,len(S)-m):
        s=S[i]["s"]
        if not lo<=s<=hi: continue
        kw=(S[i+m]["hdg"]-S[i-m]["hdg"])/(S[i+m]["s"]-S[i-m]["s"])
        for e in W.lane_edges(offs,secs,s):
            if e["type"]=="sidewalk":
                f=1-kw*e["t_out"]; fn=1-kw*e["t_in"]
                if best is None or f<best[0]: best=(f,fn,s,(1/abs(kw) if abs(kw)>1e-9 else 1e9),kw)
    return best
L=roads["1247"]["length"]
print("road1120(쌍둥이, 내 왼쪽=안쪽 인도) 2m창 지표")
for name,(a,b) in {"통과 s=80":(L-84.5,L-78.5),"정지 s=95":(L-97.5,L-92.5)}.items():
    f,fn,s,R,k=prof("1120",a,b)
    print(f"{name}: 쌍둥이 s'={s:.2f}(내 s={L-s:.2f}) R2={R:.2f} f_far(6.15)={f:.3f} f_near(3.35)={fn:.3f} kappa={k:+.3f}")
print("road1247 자기 인도(바깥쪽) 참고")
for name,(a,b) in {"통과 s=80":(78.5,84.5),"정지 s=95":(92.5,97.5)}.items():
    f,fn,s,R,k=prof("1247",a,b)
    print(f"{name}: s={s:.2f} R2={R:.2f} f_far={f:.3f}")
