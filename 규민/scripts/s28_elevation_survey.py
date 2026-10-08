#!/usr/bin/env python3
# 맵세션28 Phase 5: 고도 지원 오프라인 확인 (읽기전용, 서버 없음, xodr 수정 없음).
# CARLA 0.9.15 설치 디렉터리의 기본 맵 xodr 과 frozen_v1 에서
#   road 수 / elevationProfile 요소 유무 / elevation 개수·계수(a,b,c,d) 0 아닌 수·범위
#   lateralProfile 유무 / superelevation·shape 개수·0 아닌 수
#   s 를 따라 계산한 고도(a + b ds + c ds^2 + d ds^3) 범위(geometry 끝까지 다음 레코드 직전 표본)
# 사용: python s28_elevation_survey.py [xodr ...]  (인자 없으면 CARLA 기본 맵 전부 + frozen_v1)
import glob
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

CARLA_DIRS = [Path.home() / "carla/CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive",
              Path.home() / "carla/CARLA_0.9.15/PythonAPI/util/opendrive"]
FROZEN = Path("/home/gyumin/campus_mobility_sim/map/maps/cbnu_internal_only_localtm_tags73_smooth.xodr")
EPS = 1e-12


def poly_range(recs, length):
    """recs: [(s,a,b,c,d)] s 오름차순. 각 구간을 1m 간격으로 표본해 최소·최대."""
    lo, hi = float("inf"), float("-inf")
    for i, (s0, a, b, c, d) in enumerate(recs):
        s1 = recs[i + 1][0] if i + 1 < len(recs) else length
        n = max(1, int((s1 - s0) // 1.0))
        for k in range(n + 1):
            ds = (s1 - s0) * k / n
            z = a + b * ds + c * ds * ds + d * ds ** 3
            lo, hi = min(lo, z), max(hi, z)
    return lo, hi


def grade_stats(recs, length, min_seg=1.0):
    """구간 길이 1m 이상인 elevation 레코드에서 기울기 b + 2c ds + 3d ds^2 의 최대 |값|.
    |c| 가 가장 큰 레코드의 구간 길이도 돌려준다(짧은 구간의 큰 계수인지 보려고)."""
    gmax, cbig = 0.0, (0.0, 0.0)
    for i, (s0, a, b, c, d) in enumerate(recs):
        s1 = recs[i + 1][0] if i + 1 < len(recs) else length
        seg = s1 - s0
        if abs(c) > cbig[0]:
            cbig = (abs(c), seg)
        if seg < min_seg:
            continue
        n = max(1, int(seg // 1.0))
        for k in range(n + 1):
            ds = seg * k / n
            gmax = max(gmax, abs(b + 2 * c * ds + 3 * d * ds * ds))
    return gmax, cbig


def survey(path):
    root = ET.parse(str(path)).getroot()
    roads = list(root.iter("road"))
    st = dict(roads=len(roads), ep_elem=0, elev=0, elev_nz=0, roads_nz=0, lp_elem=0, sup=0, sup_nz=0,
              shape=0, shape_nz=0, gmax=0.0, gmax10=0.0, cbig=(0.0, 0.0), ep_txt="", lp_txt="", zmin=float("inf"), zmax=float("-inf"), bmax=0.0, cmax=0.0, dmax=0.0, supmax=0.0)
    for r in roads:
        ep = r.find("elevationProfile")
        if ep is not None:
            st["ep_elem"] += 1
            recs = []
            nz_road = False
            for e in ep.findall("elevation"):
                a, b, c, d = (float(e.get(k, 0)) for k in "abcd")
                recs.append((float(e.get("s", 0)), a, b, c, d))
                st["elev"] += 1
                if max(abs(a), abs(b), abs(c), abs(d)) > EPS:
                    st["elev_nz"] += 1
                    nz_road = True
                st["bmax"] = max(st["bmax"], abs(b))
                st["cmax"] = max(st["cmax"], abs(c))
                st["dmax"] = max(st["dmax"], abs(d))
            if recs:
                recs.sort()
                lo, hi = poly_range(recs, float(r.get("length")))
                st["zmin"], st["zmax"] = min(st["zmin"], lo), max(st["zmax"], hi)
                g, cb = grade_stats(recs, float(r.get("length")))
                st["gmax"] = max(st["gmax"], g)
                st["gmax10"] = max(st["gmax10"], grade_stats(recs, float(r.get("length")), 10.0)[0])
                if cb[0] > st["cbig"][0]:
                    st["cbig"] = cb
            if not st["ep_txt"]:
                st["ep_txt"] = " ".join(ET.tostring(ep, encoding="unicode").split())[:160]
            st["roads_nz"] += nz_road
        lp = r.find("lateralProfile")
        if lp is not None:
            st["lp_elem"] += 1
            if not st["lp_txt"]:
                st["lp_txt"] = " ".join(ET.tostring(lp, encoding="unicode").split())[:120]
            for e in lp.findall("superelevation"):
                st["sup"] += 1
                v = max(abs(float(e.get(k, 0))) for k in "abcd")
                st["sup_nz"] += v > EPS
                st["supmax"] = max(st["supmax"], abs(float(e.get("a", 0))))
            for e in lp.findall("shape"):
                st["shape"] += 1
                st["shape_nz"] += max(abs(float(e.get(k, 0))) for k in "abcd") > EPS
    return st


def main():
    files = [Path(p) for p in sys.argv[1:]]
    if not files:
        for d in CARLA_DIRS:
            # OpenDriveMap.xodr 는 기본 맵이 아니다. 서버가 generate_opendrive_world 로 받은 우리 맵을 쓴 캐시(road 878)
            files += sorted(Path(p) for p in glob.glob(str(d / "*.xodr")) if Path(p).name != "OpenDriveMap.xodr")
        files.append(FROZEN)
    for f in files:
        s = survey(f)
        z = f"{s['zmin']:.2f}~{s['zmax']:.2f}m" if s["elev"] else "-"
        print(f"{f.name}: road {s['roads']}, elevationProfile 요소 {s['ep_elem']}, elevation {s['elev']}"
              f" (0 아님 {s['elev_nz']}, 그런 road {s['roads_nz']}), 고도 {z},"
              f" |b|max {s['bmax']:.4f} |c|max {s['cmax']:.2e} |d|max {s['dmax']:.2e};"
              f" lateralProfile 요소 {s['lp_elem']}, superelevation {s['sup']} (0 아님 {s['sup_nz']}, |a|max {s['supmax']:.4f} rad),"
              f" shape {s['shape']} (0 아님 {s['shape_nz']})")
        print(f"  기울기 |dz/ds| 최대(구간 1m 이상) {s['gmax'] * 100:.1f}% (구간 10m 이상 {s['gmax10'] * 100:.1f}%), |c| 최대 레코드의 구간 길이 {s['cbig'][1]:.3f}m")
        print(f"  첫 road 원문: {s['ep_txt']} / {s['lp_txt']}")


if __name__ == "__main__":
    main()
