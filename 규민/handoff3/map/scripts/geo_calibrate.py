#!/usr/bin/env python3
# xodr 좌표 -> 위경도 역변환. 읽기전용, xodr/osm 파일 미수정.
#
# xodr 헤더 geoReference는 "+proj=tmerc"뿐이고 lon_0/lat_0/ellps/단위가
# 없어 수학적으로 역변환 불가(offset 값이 수백만 단위로 나오는 것 자체가
# 불완전한 proj 파라미터의 증거). 대신 junction 이름=원본 OSM 노드ID라는
# 사실을 이용해, 원본 OSM(data/raw/cbnu_campus.osm)의 실측 위경도와
# xodr 좌표를 대응시켜 로컬 유사변환(회전+등방스케일+평행이동)을
# 최소자승으로 직접 추정한다(Umeyama 방법).
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_geometry import XODR_PATH, parse_roads, parse_junctions, road_end_state

OSM_RAW_PATH = Path(__file__).resolve().parent.parent / "data" / "raw" / "cbnu_campus.osm"
METERS_PER_DEG_LAT = 111320  # compute_bbox.py와 동일 상수(WGS84 근사)


def load_osm_nodes(path):
    nodes = {}
    for _, elem in ET.iterparse(path, events=("end",)):
        if elem.tag == "node":
            nodes[elem.get("id")] = (float(elem.get("lat")), float(elem.get("lon")))
        elem.clear()
    return nodes


def junction_xodr_locations(roads, junctions):
    locs = {}
    for jid, junction in junctions.items():
        pts = []
        for conn in junction["connections"]:
            connecting = roads.get(conn["connectingRoad"])
            if connecting:
                pts.append(road_end_state(connecting, "start")[:2])
        if pts:
            locs[jid] = (
                sum(p[0] for p in pts) / len(pts),
                sum(p[1] for p in pts) / len(pts),
            )
    return locs


def latlon_to_enu(lat, lon, ref_lat, ref_lon):
    east = (lon - ref_lon) * METERS_PER_DEG_LAT * math.cos(math.radians(ref_lat))
    north = (lat - ref_lat) * METERS_PER_DEG_LAT
    return east, north


def enu_to_latlon(east, north, ref_lat, ref_lon):
    lat = ref_lat + north / METERS_PER_DEG_LAT
    lon = ref_lon + east / (METERS_PER_DEG_LAT * math.cos(math.radians(ref_lat)))
    return lat, lon


def fit_similarity(src_xy, dst_en):
    # Umeyama 2D, no reflection: dst ≈ s*R*src + t
    X = np.asarray(src_xy, dtype=float)
    Y = np.asarray(dst_en, dtype=float)
    mu_x, mu_y = X.mean(axis=0), Y.mean(axis=0)
    Xc, Yc = X - mu_x, Y - mu_y
    n = len(X)
    cov = (Yc.T @ Xc) / n
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(2)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[-1, -1] = -1
    R = U @ S @ Vt
    var_x = (Xc ** 2).sum() / n
    s = (D * np.diag(S)).sum() / var_x
    t = mu_y - s * R @ mu_x

    pred = (s * (R @ X.T).T) + t
    residuals = np.linalg.norm(pred - Y, axis=1)
    angle_deg = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    return R, s, t, angle_deg, residuals


def apply_transform(R, s, t, x, y):
    v = s * (R @ np.array([x, y])) + t
    return v[0], v[1]


def main():
    root = ET.parse(XODR_PATH).getroot()
    roads = parse_roads(root)
    junctions = parse_junctions(root)
    osm_nodes = load_osm_nodes(OSM_RAW_PATH)
    j_xy = junction_xodr_locations(roads, junctions)

    pairs = []  # (name, xodr_x, xodr_y, lat, lon)
    for jid, junction in junctions.items():
        name = junction["name"]
        if name in osm_nodes and jid in j_xy:
            lat, lon = osm_nodes[name]
            x, y = j_xy[jid]
            pairs.append((name, x, y, lat, lon))

    print(f"보정점(junction) 매칭 수: {len(pairs)} / {len(junctions)}")
    ref_lat = sum(p[3] for p in pairs) / len(pairs)
    ref_lon = sum(p[4] for p in pairs) / len(pairs)

    src_xy = [(p[1], p[2]) for p in pairs]
    dst_en = [latlon_to_enu(p[3], p[4], ref_lat, ref_lon) for p in pairs]

    R, s, t, angle_deg, residuals = fit_similarity(src_xy, dst_en)
    rms = math.sqrt(sum(r ** 2 for r in residuals) / len(residuals))
    print(f"적합 회전각(도, xodr Y+ 축이 실제 어느 방향인지): {angle_deg:.4f}")
    print(f"적합 스케일(1.0이면 미터 단위 일치): {s:.6f}")
    print(f"잔차(m) RMS/max: {rms:.3f}/{max(residuals):.3f}")
    ranked = sorted(zip(pairs, residuals), key=lambda pr: -pr[1])
    print("잔차 상위 5개 보정점:")
    for (name, x, y, lat, lon), res in ranked[:5]:
        print(f"  junction name={name} 잔차={res:.3f}m")
    print()

    # 1. 강조구간 두 끝점 = junction1(4748296080), junction11(4402717742)
    #    이 둘은 OSM 노드ID 그 자체이므로 변환 없이 원본 위경도 그대로 사용(무오차)
    for jid, jname in [("1", "4748296080"), ("11", "4402717742")]:
        if jname in osm_nodes:
            lat, lon = osm_nodes[jname]
            print(f"강조구간 끝점 junction{jid}({jname}, 원본OSM 직접조회): "
                  f"lat={lat:.6f} lon={lon:.6f}")

    # 2. 북쪽 고립 덩어리 중심(연결요소 분석으로 확인한 xodr 좌표, 별도 계산)
    isolated_center_xodr = (318.22, 1630.36)
    e, n = isolated_center_xodr
    inv = np.linalg.inv(R)
    # apply_transform은 순방향(xodr->enu)이므로 중심점은 이미 xodr좌표 그대로 순변환
    lat, lon = enu_to_latlon(*apply_transform(R, s, t, *isolated_center_xodr), ref_lat, ref_lon)
    print(f"북쪽 고립덩어리 중심(xodr {isolated_center_xodr}, 변환추정): "
          f"lat={lat:.6f} lon={lon:.6f}")

    # 3. 도로망 전체 중심(bbox 중심, 이전 분석값 재사용)
    network_center_xodr = ((3.34 + 1202.77) / 2, (-3.08 + 1699.23) / 2)
    lat, lon = enu_to_latlon(*apply_transform(R, s, t, *network_center_xodr), ref_lat, ref_lon)
    print(f"도로망 전체 중심(xodr {network_center_xodr}, 변환추정): "
          f"lat={lat:.6f} lon={lon:.6f}")


if __name__ == "__main__":
    main()
