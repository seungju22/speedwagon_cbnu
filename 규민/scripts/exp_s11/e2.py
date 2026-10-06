from exp_lib import *
import math
print("단일 꺾임: a=진입세그먼트, b=진출세그먼트, theta=꺾임각. Rmin=변환 후 기준선 최소반경")
print("a    b    theta  Rmin   Rmin*theta/min(a,b)  min_f_inside")
for a,b in ((6,6),(12,12),(25,25),(50,50),(5,50),(50,5)):
    for th in (10,17,28.5,45,60,90):
        t=math.radians(th)
        pl=[(-a,0),(0,0),(b*math.cos(t),b*math.sin(t))]
        # 양끝이 막다른 끝이라 junction 이 생기지만 plain road 만 본다
        try:
            ms=measure(convert(make_osm([pl])))
        except Exception as e:
            print(a,b,th,"ERR",e); continue
        ms=[m for m in ms if m["has_sw"]]
        if not ms: print(a,b,th,"no plain sidewalk road"); continue
        Rm=min(m["Rmin"] for m in ms); mf=min(m["min_f_inside"] for m in ms)
        print(f"{a:<4} {b:<4} {th:<6} {Rm:6.2f}  {Rm*t/min(a,b):6.3f}   {mf:7.2f}   roads={[round(m['len'],1) for m in ms]}")
