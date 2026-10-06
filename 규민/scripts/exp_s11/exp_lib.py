# 스크래치패드 실험용(프로젝트 파일 아님). 격리 OSM -> Osm2Odr 변환 -> 곡률 측정
import math, sys, xml.etree.ElementTree as ET
sys.path.insert(0, "/home/gyumin/campus_mobility_sim/map/scripts")
import osm_to_xodr as o2x
from analyze_geometry import parse_roads
import analyze_wedge_s11 as W

LAT0, LON0 = 36.6318, 127.4532
MLAT = 110574.0
MLON = 111320.0 * math.cos(math.radians(LAT0))
def to_ll(e, n): return LAT0 + n / MLAT, LON0 + e / MLON
def to_en(lat, lon): return (lon - LON0) * MLON, (lat - LAT0) * MLAT

_nid = [900000000000]
def make_osm(polylines, tags=None):
    tags = tags or {"highway": "unclassified", "service": "parking_aisle"}
    root = ET.Element("osm", version="0.6", generator="exp")
    wid = 800000000000
    for pl in polylines:
        refs = []
        for (e, n) in pl:
            _nid[0] += 1
            lat, lon = to_ll(e, n)
            ET.SubElement(root, "node", id=str(_nid[0]), lat=f"{lat:.9f}", lon=f"{lon:.9f}", version="1")
            refs.append(_nid[0])
        wid += 1
        w = ET.SubElement(root, "way", id=str(wid), version="1")
        for r in refs: ET.SubElement(w, "nd", ref=str(r))
        for k, v in tags.items(): ET.SubElement(w, "tag", k=k, v=v)
    return ET.tostring(root, encoding="unicode")

def convert(osm_str):
    return o2x.convert(osm_str, o2x.build_settings())

def measure(xodr_str):
    """plain road 별: 길이, 기준선 최소 반경(해석적 곡률), 안쪽 인도 접힘 최소 f."""
    root = ET.fromstring(xodr_str)
    roads = parse_roads(root)
    elems = {r.get("id"): r for r in root.iter("road")}
    out = []
    for rid, r in roads.items():
        if r["junction"] != "-1": continue
        offs, secs = W.parse_lanes(elems[rid])
        samples = W.sample_road(r, 0.25)
        kmax = max(abs(x["k"]) for x in samples)
        recs, _ = W.scan_road(rid, r, offs, secs)
        minf = min([x["f_far"] for x in recs if x["inside"]] or [1.0])
        out.append({"id": rid, "len": r["length"], "Rmin": (1/kmax if kmax > 1e-9 else float("inf")),
                    "min_f_inside": minf, "has_sw": bool(recs)})
    return out

def _sub(a,b): return (a[0]-b[0], a[1]-b[1])
def _len(v): return math.hypot(v[0], v[1])
def turn_deg(p0,p1,p2):
    a=_sub(p1,p0); b=_sub(p2,p1)
    return math.degrees(math.atan2(a[0]*b[1]-a[1]*b[0], a[0]*b[0]+a[1]*b[1]))

def fillet_vertex(p0, p1, p2, R, h):
    """p1 꼭짓점을 반경 R 원호로 모따기. 반환: (호 위 점 리스트, T, 최대이탈(꼭짓점~호중점))"""
    a=_sub(p1,p0); b=_sub(p2,p1); la,lb=_len(a),_len(b)
    u1=(a[0]/la,a[1]/la); u2=(b[0]/lb,b[1]/lb)
    th=math.atan2(u1[0]*u2[1]-u1[1]*u2[0], u1[0]*u2[0]+u1[1]*u2[1])   # 부호있는 꺾임
    ath=abs(th); T=R*math.tan(ath/2)
    A=(p1[0]-u1[0]*T, p1[1]-u1[1]*T)
    sgn=1 if th>0 else -1
    nrm=(-u1[1]*sgn, u1[0]*sgn)               # 회전 중심 방향(꺾이는 쪽)
    C=(A[0]+nrm[0]*R, A[1]+nrm[1]*R)
    arc=R*ath; n=max(int(math.ceil(arc/h)),1)
    a0=math.atan2(A[1]-C[1], A[0]-C[0])
    pts=[]
    for k in range(n+1):
        ang=a0+sgn*ath*k/n
        pts.append((C[0]+R*math.cos(ang), C[1]+R*math.sin(ang)))
    dev=R*(1/math.cos(ath/2)-1)
    return pts, T, dev

def fillet_polyline(pl, targets, h):
    """targets: {index: R}. 인접 필렛 접선길이 합이 세그먼트 길이 초과하면 예외."""
    out=[pl[0]]; devs=[]
    Ts={i:R*math.tan(abs(math.radians(turn_deg(pl[i-1],pl[i],pl[i+1])))/2) for i,R in targets.items()}
    for i in range(1,len(pl)-1):
        if i in targets:
            R=targets[i]
            for j in (i-1,i+1):
                seg=_len(_sub(pl[i],pl[j]))
                other=Ts.get(j,0.0) if 0<j<len(pl)-1 else 0.0
                if Ts[i]+other>seg+1e-9: raise ValueError(f"필렛 접선 겹침 node{i} R={R}")
            pts,T,dev=fillet_vertex(pl[i-1],pl[i],pl[i+1],R,h)
            out.extend(pts); devs.append((i,T,dev))
        else:
            out.append(pl[i])
    out.append(pl[-1])
    return out, devs

def measure2(xodr_str, win=2.0, phases=8):
    """메시 기준(vertex_distance 2m) 지표. 기준선 곡률을 win(2m) 창 평균으로 계산:
    kappa_w(s)=(theta(s+win/2)-theta(s-win/2))/win. 창 위상은 연속 슬라이딩(최악 위상 포함).
    반환: plain road(인도 lane 있는) 별 R_win_min, 안쪽 인도(t_far)에서의 f_win_min, 해석적 Rmin."""
    root = ET.fromstring(xodr_str)
    roads = parse_roads(root)
    elems = {r.get("id"): r for r in root.iter("road")}
    out = []
    for rid, r in roads.items():
        if r["junction"] != "-1": continue
        offs, secs = W.parse_lanes(elems[rid])
        S = W.sample_road(r, 0.25)
        h = S[1]["s"] - S[0]["s"] if len(S) > 1 else 0.25
        m = max(int(round(win / 2 / h)), 1)
        best_R = float("inf"); best_f = 1.0; kan = max(abs(x["k"]) for x in S)
        for i in range(m, len(S) - m):
            ds = S[i + m]["s"] - S[i - m]["s"]
            kw = (S[i + m]["hdg"] - S[i - m]["hdg"]) / ds
            if abs(kw) > 1e-9: best_R = min(best_R, 1 / abs(kw))
            for e in W.lane_edges(offs, secs, S[i]["s"]):
                if e["type"] == "sidewalk" and kw * e["t_out"] > 0:
                    best_f = min(best_f, 1 - kw * e["t_out"])
        has_sw = any(e["type"] == "sidewalk" for e in W.lane_edges(offs, secs, 0.0))
        out.append({"id": rid, "len": r["length"], "R2": best_R, "f2": best_f,
                    "Ran": (1 / kan if kan > 1e-9 else float("inf")), "has_sw": has_sw})
    return out
