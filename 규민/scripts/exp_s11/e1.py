from exp_lib import *
import math
print("== 합성 단일 꺾임 + 필렛 (a=b=12, theta 28.5) ==")
print("R_poly h    Rmin   min_f  이탈(꼭짓점-호)")
base=[(-12,0),(0,0),(12*math.cos(math.radians(28.5)),12*math.sin(math.radians(28.5)))]
for R in (0,8,12,16,20):
    for h in ((2.0,) if R==0 else (1.0,2.0,3.0)):
        try:
            pl,devs=fillet_polyline(base,{1:R},h) if R else (base,[])
            ms=[m for m in measure(convert(make_osm([pl]))) if m["has_sw"]]
            print(f"{R:<6} {h:<4} {min(m['Rmin'] for m in ms):6.2f} {min(m['min_f_inside'] for m in ms):6.2f}  {devs[0][2] if devs else 0:.2f}")
        except Exception as e: print(R,h,"ERR",e)
