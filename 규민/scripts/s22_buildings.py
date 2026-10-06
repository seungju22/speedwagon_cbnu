#!/usr/bin/env python3
# 맵세션22 Phase 3-2: OSM 건물 폴리곤 -> 중심 위경도 -> CARLA 좌표 (읽기전용, 서버 없음).
# 좌표를 하드코딩하지 않는다. 투영 중심(lat_0, lon_0)과 offset 은 대상 xodr 머리말에서 읽는다.
# 그래서 재변환(center_map 이동량이 바뀜) 뒤에도 --xodr 만 새 파일로 주면 그대로 쓸 수 있다.
# 좌표 사슬: 위경도 -> 지역 TM(동 x, 북 y) -> xodr = TM + offset -> CARLA = (x, -y)  (s19_scope_common 과 같은 규약)
# 자기 검사: junction 이름 = OSM 노드 id 인 성질로, 노드 투영 위치와 xodr 안 그 junction 커넥터 시작점 중심의 거리 중앙값을 낸다.
# 도로 여유: 오프라인 carla.Map 으로 중심에서 가장 가까운 주행 차선·인도 차선 중심까지 거리.
# 실행: .venv-carla/bin/python map/scripts/s22_buildings.py [--xodr PATH] [--all]
#   출력 map/data/processed/s22_buildings.csv, 로그는 stdout
import argparse
import csv
import hashlib
import math
import re
import statistics
import xml.etree.ElementTree as ET
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
RAW_OSM = BASE / "data/raw/cbnu_campus.osm"
XODR_DEFAULT = BASE / "maps/cbnu_internal_only_localtm_tags73_smooth.xodr"
OUT_CSV = BASE / "data/processed/s22_buildings.csv"
# 발표·배치 대상 주요 건물(이름 일부 일치). --all 이면 이름 있는 건물 전부
KEY_NAMES = ["중앙도서관 구관", "중앙도서관 신관", "대학본부 (N10)", "경영대학 본관", "자연과학대학 본관",
             "양성재", "(S17-",  # 양성재 이름 폴리곤은 13m 소형 1동뿐, 기숙사동은 S17-x 번호(지선관·명덕관 등)
             "인문사회관", "사회과학대학 본관", "법학관", "개신문화관", "신학생회관", "구학생회관"]


def header_proj(xodr_text):
    m = re.search(r"<geoReference>.*?(\+proj=[^\]<]*)", xodr_text, re.S)
    proj = m.group(1).strip()
    p = dict(kv.lstrip("+").split("=", 1) for kv in proj.split() if "=" in kv)
    o = re.search(r'<offset x="([-\d.]+)" y="([-\d.]+)"', xodr_text)
    if "lat_0" not in p or "lon_0" not in p:
        raise SystemExit(f"투영식에 lat_0/lon_0 없음: {proj} (세션3 문제와 같은 상태, 중단)")
    return proj, float(p["lat_0"]), float(p["lon_0"]), float(o.group(1)), float(o.group(2))


class TM:
    """WGS84 횡메르카토르 k0=1 (Snyder 8-9~8-10). 중심 1~2km 범위에서 mm 수준."""
    A, F = 6378137.0, 1 / 298.257223563

    def __init__(self, lat0, lon0):
        self.e2 = self.F * (2 - self.F)
        self.ep2 = self.e2 / (1 - self.e2)
        self.lat0, self.lon0 = lat0, lon0
        self.m0 = self._mer(math.radians(lat0))

    def _mer(self, phi):
        e2, e4, e6 = self.e2, self.e2 ** 2, self.e2 ** 3
        return self.A * ((1 - e2 / 4 - 3 * e4 / 64 - 5 * e6 / 256) * phi
                         - (3 * e2 / 8 + 3 * e4 / 32 + 45 * e6 / 1024) * math.sin(2 * phi)
                         + (15 * e4 / 256 + 45 * e6 / 1024) * math.sin(4 * phi)
                         - (35 * e6 / 3072) * math.sin(6 * phi))

    def fwd(self, lat, lon):
        phi, lam = math.radians(lat), math.radians(lon - self.lon0)
        n = self.A / math.sqrt(1 - self.e2 * math.sin(phi) ** 2)
        t, c = math.tan(phi) ** 2, self.ep2 * math.cos(phi) ** 2
        a = lam * math.cos(phi)
        x = n * (a + (1 - t + c) * a ** 3 / 6 + (5 - 18 * t + t * t + 72 * c - 58 * self.ep2) * a ** 5 / 120)
        y = (self._mer(phi) - self.m0 + n * math.tan(phi) * (a * a / 2 + (5 - t + 9 * c + 4 * c * c) * a ** 4 / 24
                                                              + (61 - 58 * t + t * t + 600 * c - 330 * self.ep2) * a ** 6 / 720))
        return x, y


class Frame:
    def __init__(self, xodr_text):
        self.proj, lat0, lon0, self.ox, self.oy = header_proj(xodr_text)
        self.tm = TM(lat0, lon0)

    def xodr(self, lat, lon):
        x, y = self.tm.fwd(lat, lon)
        return x + self.ox, y + self.oy

    def carla(self, lat, lon):
        x, y = self.xodr(lat, lon)
        return x, -y


def load_osm(path):
    nodes, ways = {}, {}
    for _, el in ET.iterparse(str(path)):
        if el.tag == "node":
            nodes[el.get("id")] = (float(el.get("lat")), float(el.get("lon")))
        elif el.tag == "way":
            ways[el.get("id")] = ([n.get("ref") for n in el.findall("nd")],
                                  {t.get("k"): t.get("v") for t in el.findall("tag")})
            el.clear()
    return nodes, ways


def poly_stats(pts):
    """평면 다각형(m) -> 면적 중심, 면적, 최소 넓이 외접 사각형(길이·폭·방향)."""
    if pts[0] != pts[-1]:
        pts = pts + [pts[0]]
    a = cx = cy = 0.0
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        cr = x0 * y1 - x1 * y0
        a += cr
        cx += (x0 + x1) * cr
        cy += (y0 + y1) * cr
    a /= 2
    if abs(a) < 1e-9:
        cx, cy = sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)
    else:
        cx, cy = cx / (6 * a), cy / (6 * a)
    best = None
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        h = math.atan2(y1 - y0, x1 - x0)
        c, s = math.cos(h), math.sin(h)
        u = [(x - cx) * c + (y - cy) * s for x, y in pts]
        v = [-(x - cx) * s + (y - cy) * c for x, y in pts]
        L, W = max(u) - min(u), max(v) - min(v)
        if best is None or L * W < best[0]:
            best = (L * W, max(L, W), min(L, W), math.degrees(h if L >= W else h + math.pi / 2))
    return (cx, cy), abs(a), best[1], best[2], (best[3] + 180) % 180


def self_check(frame, xodr_text, nodes):
    """junction 이름(=OSM 노드 id) 투영 위치 vs 그 junction 커넥터 시작점 평균. 반환 (표본 수, 중앙값 m)."""
    root = ET.fromstring(xodr_text.encode())
    starts = {}
    for rd in root.iter("road"):
        j = rd.get("junction")
        if j == "-1":
            continue
        g = rd.find("planView/geometry")
        starts.setdefault(j, []).append((float(g.get("x")), float(g.get("y"))))
    names = {jn.get("id"): jn.get("name") for jn in root.iter("junction")}
    d = []
    for jid, pts in starts.items():
        nid = names.get(jid)
        if nid not in nodes:
            continue
        px, py = frame.xodr(*nodes[nid])
        mx, my = sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)
        d.append(math.hypot(px - mx, py - my))
    return len(d), (statistics.median(d) if d else float("nan"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xodr", type=Path, default=XODR_DEFAULT)
    ap.add_argument("--osm", type=Path, default=RAW_OSM)
    ap.add_argument("--out", type=Path, default=OUT_CSV)
    ap.add_argument("--all", action="store_true", help="이름 있는 건물 전부")
    args = ap.parse_args()

    text = args.xodr.read_text()
    sha = hashlib.sha256(text.encode()).hexdigest()[:16]
    frame = Frame(text)
    print(f"xodr {args.xodr.name} sha256 {sha}")
    print(f"proj {frame.proj} / offset {frame.ox} {frame.oy}")
    nodes, ways = load_osm(args.osm)
    n, med = self_check(frame, text, nodes)
    print(f"자기검사: junction {n}개, 노드 투영 vs 커넥터 시작점 평균 거리 중앙값 {med:.2f}m")

    try:
        import carla
        cmap = carla.Map(args.xodr.stem, text)
    except Exception as e:  # carla 없으면 도로 여유만 생략
        print(f"carla.Map 생략: {e}")
        cmap = None

    rows = []
    for wid, (nds, tags) in ways.items():
        if "building" not in tags or not tags.get("name"):
            continue
        name = tags["name"]
        if not args.all and not any(k in name for k in KEY_NAMES):
            continue
        ll = [nodes[i] for i in nds if i in nodes]
        if len(ll) < 3:
            continue
        pts = [frame.tm.fwd(*p) for p in ll]
        _, area, L, W, yaw_tm = poly_stats(pts)
        # 면적 중심은 아핀 변환에 불변 -> 건물 크기(100m)에서는 위경도 평면에서 바로 구해도 같다.
        # 이 위경도가 원천 값이고, CARLA 좌표는 매번 여기서 변환한다(재변환 후 재사용)
        (lon, lat), *_ = poly_stats([(p[1], p[0]) for p in ll])
        x, y = frame.carla(lat, lon)
        yaw_carla = (-yaw_tm) % 180          # CARLA 는 y 반전이라 각도 부호가 바뀐다
        d_drive = d_side = float("nan")
        if cmap is not None:
            loc = carla.Location(x=x, y=y, z=0.0)
            wp = cmap.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
            if wp:
                d_drive = wp.transform.location.distance(loc) - wp.lane_width / 2
            wp = cmap.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Sidewalk)
            if wp:
                d_side = wp.transform.location.distance(loc) - wp.lane_width / 2
        rows.append(dict(name=name, way_id=wid, building=tags.get("building"), lat=f"{lat:.6f}", lon=f"{lon:.6f}",
                         carla_x=f"{x:.2f}", carla_y=f"{y:.2f}", area_m2=f"{area:.0f}", len_m=f"{L:.1f}",
                         wid_m=f"{W:.1f}", yaw_deg=f"{yaw_carla:.1f}", clear_drive_m=f"{d_drive:.1f}",
                         clear_sidewalk_m=f"{d_side:.1f}", map_sha=sha))
    rows.sort(key=lambda r: r["name"])
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with open(args.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"건물 {len(rows)}개 -> {args.out}")
    for r in rows:
        print(f"{r['name']} way{r['way_id']} ({r['lat']},{r['lon']}) carla({r['carla_x']},{r['carla_y']}) "
              f"{r['len_m']}x{r['wid_m']}m yaw{r['yaw_deg']} 차선끝까지{r['clear_drive_m']}m 인도끝까지{r['clear_sidewalk_m']}m")


if __name__ == "__main__":
    main()
