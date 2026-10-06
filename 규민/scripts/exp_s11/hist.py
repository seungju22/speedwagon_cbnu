import sys
from scan_generic import *
def hist(path):
    roads,sites=scan_xodr(path); g=group(sites,0.3)
    b=[("<0",lambda f:f<0),("0~0.1",lambda f:0<=f<0.1),("0.1~0.2",lambda f:0.1<=f<0.2),("0.2~0.3",lambda f:0.2<=f<0.3)]
    return len(g),[(n,sum(1 for x in g if fn(x["f"]))) for n,fn in b]
for p in sys.argv[1:]: print(p, hist(p))
