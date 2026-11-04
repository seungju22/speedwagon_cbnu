#!/usr/bin/env python3
# 10/14(M20): 차체 크기별 노선 적합성(서버 없이, 읽기만). 결과는 새 파일로만 낸다(노선 파일 1·2판, s53·s56 결과 안 건드림)
# 1) 차체 검사: s53_footprint.py 의 check() 와 같은 방법(경로 lanelet 합 안에 네 모서리, 옆 이동 -1.0~+1.0m). 차 치수만 바꾼다.
#    더한 것: 못 들어가는 점에서 '가장 덜 나간 옆 이동'일 때 모서리가 lanelet 밖으로 나간 최대 거리(m)
# 2) 곡률: 같은 중심선(0.5m 간격)에서 앞뒤 2.0m 점으로 외접원 반경. 그 차의 뒷축 최소 회전 반경보다 작은 곳을 road 별로 묶음
# 3) 승강장 10곳: s56_shuttle.py 를 모듈로 불러 머리 값(WB, FO, RO, 폭, 문 자리, 통로 폭)만 바꿔 main() 을 돌리고,
#    출력 경로(OUT, 로그)는 이 세션 파일로 돌린다. 통과 기준은 s56_write.py 62~63행과 같다
# 입력: autoware_map/cbnu_campus_v31/lanelet2_map.osm(28a57711, 동결), map/build_v31_s54/pairs35.json(30쌍 p_*),
#       map/docs/logs/s56b_footprint_pairs.json(시범 노선 1안), s56c_footprint_pairs.json(최종 확장안)
# 출력: map/docs/logs/s59_m20.json, s59_m20.log, s59_m20_platforms_<차>.json
# 사용: .venv-carla/bin/python map/scripts/s59_m20.py [차 ...]
import importlib.util
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

from shapely.geometry import Point, Polygon
from shapely.ops import unary_union
from shapely.prepared import prep

HERE = Path(__file__).resolve().parent
BASE = HERE.parent
ROOT = BASE.parent
LOGS = BASE / "docs/logs"
OSM = ROOT / "autoware_map/cbnu_campus_v31/lanelet2_map.osm"
sys.path.insert(0, str(HERE))
import s53_footprint as fp  # noqa: E402  (load·resample·MIN_START·OFFS 만 씀, 파일 안 고침)

# 차 치수(m). R = 뒷축 가운데가 그리는 최소 회전 반경. 근거와 가정은 map/docs/m20_vehicle_fit.md
# 제조사 '회전 원'(바깥 앞바퀴 지름)은 R = sqrt((D/2)^2 - WB^2) - tread/2 로 바꿨다
VEH = {
    "sample": dict(name="지금 차(sample_vehicle)", WB=2.79, FO=1.0, RO=1.1, W=1.896, TREAD=1.64, R=2.79 / math.tan(0.70)),
    "ez10": dict(name="소형 셔틀(EZ10급)", WB=2.80, FO=0.625, RO=0.625, W=1.892, TREAD=1.70, R=4.0),
    "rosa_s": dict(name="Rosa 짧은 형(6.99m)", WB=3.995, FO=0.94, RO=2.04, W=2.07, TREAD=1.655,
                   R=math.sqrt(6.3 ** 2 - 3.995 ** 2) - 1.655 / 2),
    "rosa_l": dict(name="Rosa 긴 형(7.73m)", WB=4.55, FO=0.94, RO=2.225, W=2.07, TREAD=1.655,
                   R=math.sqrt(7.1 ** 2 - 4.55 ** 2) - 1.655 / 2),
    "fusorosa": dict(name="CARLA fusorosa", WB=5.63, FO=None, RO=None, W=None, TREAD=None, R=None),  # 서버에서 잰 값으로 채움(아래)
}
CARLA_FUSO = LOGS / "s59_fusorosa_carla.json"   # 10/14 서버에서 잰 bbox·바퀴 자리·최대 조향
CURV_HALF = 2.0   # 곡률 잴 때 앞뒤 거리(m)


def fill_fusorosa():
    v = VEH["fusorosa"]
    if not CARLA_FUSO.exists():
        return False
    d = json.loads(CARLA_FUSO.read_text())
    # bbox 폭 3.944m 는 거울까지 든 것으로 보이고(Rosa 실차 2.07m), 최대 조향 70도(R 2.05m)는 물리 모델 값이라 쓰지 않는다.
    # 폭 = 앞바퀴 바깥(윤거 2.617 + 타이어 0.3, 가정), R = 축거 / tan(45도)(가정). 길이·축거·오버행은 CARLA 값 그대로
    v.update(WB=d["wheelbase"], FO=d["front_overhang"], RO=d["rear_overhang"], W=round(d["tread"] + 0.3, 3), TREAD=d["tread"],
             R=d["wheelbase"] / math.tan(math.radians(45.0)))
    return True


def corners(v):
    h = v["W"] / 2
    f = v["WB"] + v["FO"]
    return [(f, h), (f, -h), (-v["RO"], h), (-v["RO"], -h)]


def centerline(lls, pair):
    """s53_footprint.check() 의 중심선 만드는 부분과 같다 -> (점, road, 주행 가능 영역)"""
    polys, center, owner = [], [], []
    rs = pair["roads"]
    for k, rid in enumerate(rs):
        for piece, L, R in lls.get(rid, []):
            polys.append(Polygon(L + R[::-1]).buffer(0))
        cl = []
        for piece, L, R in lls.get(rid, []):
            lenL = sum(math.dist(a, b) for a, b in zip(L, L[1:]))
            n = max(3, int(lenL / 0.5) + 1)
            a, _ = fp.resample(L, n)
            b, _ = fp.resample(R, n)
            cl += [((p[0] + q[0]) / 2, (p[1] + q[1]) / 2) for p, q in zip(a, b)]
        if not cl:
            continue
        tot = sum(math.dist(a, b) for a, b in zip(cl, cl[1:])) or 1
        acc = [0.0]
        for a, b in zip(cl, cl[1:]):
            acc.append(acc[-1] + math.dist(a, b))
        lo = max(pair["start_s"], fp.MIN_START) if k == 0 else -1
        hi = pair["end_s"] if k == len(rs) - 1 else math.inf
        xl = pair.get("_len", {}).get(rid)
        sc = tot / xl if xl else 1.0
        for p, s in zip(cl, acc):
            if (k == 0 and s < lo * sc) or (k == len(rs) - 1 and s > hi * sc):
                continue
            if center and math.dist(center[-1], p) < 0.25:
                continue
            center.append(p)
            owner.append(rid)
    area = unary_union([p.buffer(0.02) for p in polys]).buffer(-0.02)
    return center, owner, area


def body_check(center, owner, area, cs):
    pa = prep(area)
    ext = area.boundary
    bad, emax, out_max = [], 0.0, 0.0
    for i, p in enumerate(center):
        a = center[max(0, i - 2)]
        b = center[min(len(center) - 1, i + 2)]
        h = math.atan2(b[1] - a[1], b[0] - a[0])
        c, s = math.cos(h), math.sin(h)
        ok, best = None, math.inf
        for e in fp.OFFS:
            bx, by = p[0] - e * s, p[1] + e * c
            pts = [Point(bx + x * c - y * s, by + x * s + y * c) for x, y in cs]
            if all(pa.contains(q) for q in pts):
                ok = e
                break
            best = min(best, max(0.0 if pa.contains(q) else ext.distance(q) for q in pts))
        if ok is None:
            bad.append((owner[i], round(p[0], 2), round(p[1], 2), round(best, 2)))
            out_max = max(out_max, best)
        else:
            emax = max(emax, abs(ok))
    return bad, emax, out_max


def curvature(center, owner, rmin):
    """앞뒤 CURV_HALF m 점으로 외접원 반경. rmin 보다 작은 점을 road 별로"""
    acc = [0.0]
    for a, b in zip(center, center[1:]):
        acc.append(acc[-1] + math.dist(a, b))
    j0 = j1 = 0
    tight = defaultdict(list)
    rlow = math.inf
    for i, p in enumerate(center):
        while acc[j0] < acc[i] - CURV_HALF:
            j0 += 1
        while j1 < len(center) - 1 and acc[j1] < acc[i] + CURV_HALF:
            j1 += 1
        if acc[i] - acc[j0] < CURV_HALF * 0.9 or acc[j1] - acc[i] < CURV_HALF * 0.9:
            continue
        a, b = center[j0], center[j1]
        la, lb, lc = math.dist(a, p), math.dist(p, b), math.dist(a, b)
        cross = abs((p[0] - a[0]) * (b[1] - a[1]) - (p[1] - a[1]) * (b[0] - a[0]))
        r = math.inf if cross < 1e-9 else la * lb * lc / (2 * cross)
        rlow = min(rlow, r)
        if r < rmin:
            tight[owner[i]].append((round(p[0], 1), round(p[1], 1), round(r, 2)))
    return rlow, tight


def route_sets():
    p35 = json.loads((BASE / "build_v31_s54/pairs35.json").read_text())
    pairs = {k: v for k, v in p35.items() if k.startswith("p_")}
    pil = json.loads((LOGS / "s56b_footprint_pairs.json").read_text())["c_main_gate_in_middle_gate_s_lib_north_yangseong_se"]
    fin = json.loads((LOGS / "s56c_footprint_pairs.json").read_text())
    fin = fin["c_main_gate_in_middle_gate_s_lib_north_yangseong_se_back_gate_e"]
    return pairs, {"pilot_1an": pil, "final_ext": fin}


def platforms(key, v):
    """s56_shuttle 머리 값만 바꿔 승강장 10곳(+lib_south_se) 다시 계산. 결과 파일은 이 세션 이름으로"""
    spec = importlib.util.spec_from_file_location("s56s_" + key, HERE / "s56_shuttle.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    m.WB, m.FO, m.RO = v["WB"], v["FO"], v["RO"]
    m.TREAD = v["TREAD"]
    m.LO = m.RGO = (v["W"] - v["TREAD"]) / 2
    m.FRONT, m.REAR = m.WB + m.FO, -m.RO
    m.HALF_W = v["W"] / 2
    m.DOOR_X = (m.FRONT + m.REAR) / 2          # 문은 차 길이 가운데(가정 그대로)
    m.DOOR_Y = -m.HALF_W
    m.CORRIDOR_W = v["W"] + 0.6                # 차 폭 + 양쪽 0.3(가정 그대로)
    m.OUT = LOGS / f"s59_m20_platforms_{key}.json"
    tmp = LOGS / f"s59_m20_tmp_{key}"
    (tmp / "docs/logs").mkdir(parents=True, exist_ok=True)
    m.BASE = tmp                               # main() 끝의 s56_shuttle.log 를 여기로(원래 로그 안 덮음)
    import contextlib
    import io
    with contextlib.redirect_stdout(io.StringIO()):
        m.main()
    d = json.loads(m.OUT.read_text())
    res = []
    for p in d["platforms"]:
        dz, cr = p["door_zone"], p["corridor"]
        ok = dz["area_m2"] > 0.5 and dz["final_overlap"]["lanes_m2"] < 0.01 and cr["out_of_own_lane_m2"] < 0.01 \
            and not cr["other_lanes_m2"] and p["body_check"]["out_of_lanelets_m2"] < 0.01
        why = []
        if dz["area_m2"] <= 0.5: why.append(f"문구역 {dz['area_m2']}m2")
        if dz["final_overlap"]["lanes_m2"] >= 0.01: why.append("문구역 차로 겹침")
        if cr["out_of_own_lane_m2"] >= 0.01: why.append(f"통로 차로밖 {cr['out_of_own_lane_m2']}")
        if cr["other_lanes_m2"]: why.append(f"통로 다른차로 {cr['other_lanes_m2']}")
        if p["body_check"]["out_of_lanelets_m2"] >= 0.01:
            why.append(f"차체 lanelet밖 {p['body_check']['out_of_lanelets_m2']}m2 모서리 {p['body_check']['corner_out_m']}m")
        res.append(dict(label=p["label"], ok=ok, why=why, stop_zone_m=p["stop_zone"]["length_m"]))
    return res


def main():
    want = sys.argv[1:] or list(VEH)
    have_fuso = fill_fusorosa()
    lls = fp.load(OSM)
    pairs, routes = route_sets()
    log, out = [], {"osm": str(OSM.relative_to(ROOT)), "osm_sha16": fp_sha(OSM), "curv_half_m": CURV_HALF, "vehicles": {}}

    def P(s):
        print(s, flush=True)
        log.append(s)

    cache = {n: centerline(lls, p) for n, p in {**pairs, **routes}.items()}
    for key in want:
        v = VEH[key]
        if v["W"] is None:
            P(f"{key}: 치수 없음(서버 값 {CARLA_FUSO.name} 없음), 건너뜀")
            continue
        cs = corners(v)
        P(f"== {key} {v['name']}: 길이 {v['WB'] + v['FO'] + v['RO']:.3f} 폭 {v['W']:.3f} 축거 {v['WB']} 앞 {v['FO']} 뒤 {v['RO']} 최소 R(뒷축) {v['R']:.2f}")
        rv = {"spec": {k: (round(x, 3) if isinstance(x, float) else x) for k, x in v.items()}, "pairs": {}, "routes": {}}
        for grp, items in (("pairs", pairs), ("routes", routes)):
            for n in items:
                c, o, a = cache[n]
                bad, emax, omax = body_check(c, o, a, cs)
                rlow, tight = curvature(c, o, v["R"])
                bad_roads = sorted({b[0] for b in bad})
                rv[grp][n] = dict(points=len(c), bad=len(bad), bad_roads=bad_roads, shift_max=round(emax, 2), out_max=round(omax, 2),
                                  r_min=round(rlow, 2), tight_roads={str(k): dict(n=len(t), r_min=min(x[2] for x in t), at=t[0][:2])
                                                                      for k, t in tight.items()},
                                  bad_pts=[b[1:] for b in bad], tight_pts=[x for t in tight.values() for x in t])
                P(f"{n}: 점 {len(c)} 못들어감 {len(bad)} {bad_roads} 옆이동최대 {emax:.2f} 밖최대 {omax:.2f} / 곡률 최소 R {rlow:.2f}"
                  f" 좁은 곳 {dict((k, (len(t), min(x[2] for x in t))) for k, t in tight.items())}")
        pr = platforms(key, v)
        rv["platforms"] = pr
        P("승강장: " + " / ".join(f"{p['label']} {'OK' if p['ok'] else 'X ' + ';'.join(p['why'])}" for p in pr))
        out["vehicles"][key] = rv
    (LOGS / "s59_m20.json").write_text(json.dumps(out, ensure_ascii=False, indent=1))
    (LOGS / "s59_m20.log").write_text("\n".join(log) + "\n")


def fp_sha(p):
    import hashlib
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()[:16]


if __name__ == "__main__":
    main()
