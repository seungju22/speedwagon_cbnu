"""11/02(계획, 서버 없음) 공통: 동결 xodr(214507af)을 carla.Map 으로 오프라인 읽기, road -> OSM way·원래 태그, OSM 건물·길, 위경도, 로드뷰 링크
지도 파일은 읽기만 함. 좌표는 지도 좌표(xodr 평면 = CARLA (x, -y))
road -> way: map/docs/logs/s42_road_way.csv(10/09 대응표, road 번호는 v3.1 에도 그대로) + 10/09 북문 고리 r3001~r3009 는 '대응 없음'
원래 태그: map/data/raw/cbnu_campus.osm(받은 그대로). 변환 입력은 태그 보정본이라 모두 unclassified 였음(s42_diag.py 주석)
"""
import csv, math, sys, xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent; BASE = HERE.parent; C = BASE.parent
sys.path.insert(0, str(HERE))
from s22_buildings import Frame, load_osm  # noqa: E402

XODR = BASE / "maps/cbnu_campus_frozen_v31.xodr"
OSM_RAW = BASE / "data/raw/cbnu_campus.osm"
LL31 = C / "autoware_map/cbnu_campus_v31/lanelet2_map.osm"
TEXT = XODR.read_text()
FR = Frame(TEXT)
ROOT = ET.fromstring(TEXT)
RLEN = {int(r.get("id")): float(r.get("length")) for r in ROOT.iter("road")}
RJUNC = {int(r.get("id")): int(r.get("junction", "-1")) for r in ROOT.iter("road")}
R2W = {}
for r in csv.DictReader(open(BASE / "docs/logs/s42_road_way.csv")):
    if r["way"]:
        R2W[int(r["road"])] = r["way"]
NODES, WAYS = load_osm(OSM_RAW)   # nodes: id -> (lat, lon), ways: id -> (nd list, tags)


def way_tag(w):
    return WAYS.get(w, (None, {}))[1]


def road_tag(rid):
    w = R2W.get(rid)
    if w is None:
        return None, {}
    return w, way_tag(w)


_cmap = None


def cmap():
    global _cmap
    if _cmap is None:
        import carla
        _cmap = carla.Map("OpenDriveMap", TEXT)
    return _cmap


def road_line(rid, lane=-1, step=1.0):
    """road 의 lane 중심선(지도 좌표). lane -1 이 없으면 1"""
    m = cmap(); L = RLEN[rid]; out = []
    for ln in (lane, -lane):
        out = []
        s = 0.0
        while s <= L + 1e-6:
            w = m.get_waypoint_xodr(rid, ln, min(s, L - 0.01))
            if w is None: break
            t = w.transform.location; out.append((t.x, -t.y)); s += step
        if out: return out
    return out


def latlon(x, y):
    la, lo = 36.63, 127.455
    for _ in range(30):
        fx, fy = FR.xodr(la, lo); ex, ey = x - fx, y - fy
        if math.hypot(ex, ey) < 1e-4: break
        d = 1e-6
        a1 = (FR.xodr(la + d, lo)[0] - fx) / d; a2 = (FR.xodr(la, lo + d)[0] - fx) / d
        b1 = (FR.xodr(la + d, lo)[1] - fy) / d; b2 = (FR.xodr(la, lo + d)[1] - fy) / d
        det = a1 * b2 - a2 * b1
        la += (ex * b2 - a2 * ey) / det; lo += (a1 * ey - ex * b1) / det
    return la, lo


def kakao(x, y):
    la, lo = latlon(x, y)
    return f"https://map.kakao.com/link/roadview/{la:.7f},{lo:.7f}"


def osm_xy(nid):
    la, lo = NODES[nid]
    return FR.xodr(la, lo)


def osm_ways(pred):
    """조건에 맞는 OSM way 의 (id, 지도 좌표 점 열, 태그)"""
    for wid, (nd, t) in WAYS.items():
        if pred(t) and all(i in NODES for i in nd):
            yield wid, [osm_xy(i) for i in nd], t
