#!/usr/bin/env python3
# 세션6 Phase A: 878 road 판(tags73) junction 커넥터 결함 분류. 읽기전용, CARLA 불필요.
# analyze_geometry.py 의 파서·기하 함수를 재사용한다(원본 무수정).
# 출력: map/docs/logs/session6_phaseA.txt (상세) + 화면에는 HARD STOP A 요약만
import math
import statistics
import sys
from collections import Counter, defaultdict
from pathlib import Path
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parent))
import analyze_geometry as ag  # noqa: E402

MAP_DIR = Path(__file__).resolve().parent.parent
XODR = MAP_DIR / "maps" / "cbnu_internal_only_localtm_tags73.xodr"
OUT = MAP_DIR / "docs" / "logs" / "session6_phaseA.txt"

GAP_ANOMALY_M = 0.01        # 세션4 "gap이상" 기준(131건의 정의)
SHORT_M = 5.0               # 극단적으로 짧은 road
BIG_HDIFF_DEG = 10.0        # 큰 방향차 기준(가정, 분포를 함께 출력)
BIG_TURN_DEG = 45.0         # 커넥터 자체 회전각 기준(가정)
TEMPLATE_LEN = 9.892401     # 세션2에서 확인된 netconvert 고정 템플릿 길이

ROUTE = [1247, 1914, 1356, 1917, 1355, 1446, 1172,
         1428, 1240, 1695, 1239, 1426, 1238, 1278]
ROUTE_JUNCTIONS = {"1", "2", "10", "35", "36", "42", "60", "84", "107"}


def norm180(a):
    return (a + 180.0) % 360.0 - 180.0


def signature(road):
    # 길이 + 모든 geometry 의 계수(소수 4자리)가 같으면 같은 템플릿 집단
    parts = [round(road["length"], 4)]
    for g in road["geoms"]:
        parts.append(g["type"])
        if g["type"] == "paramPoly3":
            parts.append(tuple(round(v, 4) for v in g["coeffs"].values()))
    return tuple(parts)


def turn_deg(road):
    _, _, h0 = ag.road_end_state(road, "start")
    _, _, h1 = ag.road_end_state(road, "end")
    return norm180(math.degrees(h1 - h0))


def touch(a, b, roads, junctions):
    """a 의 어느 끝이 b 의 어느 끝과 닿는지. (endA, endB) 또는 None"""
    for x, y, swap in ((a, b, False), (b, a, True)):
        for end, key in (("start", "pred"), ("end", "succ")):
            lk = x[key]
            if lk and lk.get("elementType") == "road" \
                    and lk.get("elementId") == y["id"]:
                other = "start" if lk.get("contactPoint") == "start" else "end"
                return (other, end) if swap else (end, other)
    # 한쪽이 plain road, 다른 쪽이 그 junction 의 커넥터
    for inc, con, swap in ((a, b, False), (b, a, True)):
        j = junctions.get(con["junction"])
        if not j:
            continue
        for c in j["connections"]:
            if c["incomingRoad"] == inc["id"] and c["connectingRoad"] == con["id"]:
                e_in = ag.attach_end_of_incoming(inc, j["id"])
                e_con = c["contactPoint"]
                if e_in:
                    return (e_con, e_in) if swap else (e_in, e_con)
    return None


FAIL_PTS = [(275.33, 957.23), (280.80, 960.21)]   # 세션5 2차 실패(xodr 좌표)
ROUTE_FIXED = ROUTE[:-1] + [1564, 1278]           # r1564: junction36 커넥터
ROUTES_CHECK = {
    "S5 출입구1/2 원본(14)": ROUTE,
    "S5 출입구1/2 정정(15)": ROUTE_FIXED,
    "S5 출입구3": ROUTE[:-1] + [1368],
    "S4 A안(15)": [1247, 1914, 1356, 1917, 1355, 1447, 1373, 1673, 1374,
                   1591, 1375, 1689, 1376, 1625, 1377],
}


def followup(root, roads, junctions, L):
    """세션6 Phase A 후속: junction 구성, 실패점 주변, 경로 누락/방향 검증.
    (인라인 실행분을 재현 가능하게 저장. 세션3 스크립트 유실 방지)"""
    xr = {r.get("id"): r for r in root.iter("road")}

    L.append("")
    L.append("## 추가6 junction 구성(1: 실패점 / 36: 커넥터 누락)")
    for jid in ("1", "36"):
        j = junctions[jid]
        L.append(f"  junction{jid} name={j['name']} connection={len(j['connections'])}")
        for c in j["connections"]:
            k = roads[c["connectingRoad"]]
            L.append(f"    in r{c['incomingRoad']} -> conn r{k['id']} "
                     f"len={k['length']:.2f} turn={turn_deg(k):.1f} "
                     f"-> r{k['succ'].get('elementId')}")

    L.append("")
    L.append("## 추가7 실패점 주변 lane/고도 + 최근접 커넥터")
    for rid in ("1247", "1914", "1356"):
        e = xr[rid]
        el = [dict(x.attrib) for x in e.find("elevationProfile")]
        lanes = [(ln.get("type"), ln.find("width").get("a"))
                 for ln in e.find("lanes").find("laneSection").find("right")
                 .findall("lane")]
        L.append(f"  r{rid} elevation={[(x['a'], x['b']) for x in el]} "
                 f"laneOffset={len(e.find('lanes').findall('laneOffset'))} "
                 f"lanes(right)={lanes}")
    for rid in ("1247", "1914", "1915", "1120"):
        pl = ag.road_polyline(roads[rid], curve_samples=12)
        ds = [min(math.hypot(p[0] - f[0], p[1] - f[1]) for p in pl)
              for f in FAIL_PTS]
        hs = math.degrees(ag.road_end_state(roads[rid], "start")[2])
        he = math.degrees(ag.road_end_state(roads[rid], "end")[2])
        L.append(f"  r{rid} len={roads[rid]['length']:.2f} hdg {hs:.1f}->{he:.1f} "
                 f"실패점1/2 최근접거리 {ds[0]:.2f}/{ds[1]:.2f}m")

    L.append("")
    L.append("## 추가8 경로 검증(정방향 여부 / 사이 커넥터 누락)")
    n_dir = Counter()
    for r in xr.values():
        sec = r.find("lanes").find("laneSection")
        for side in ("left", "right"):
            sd = sec.find(side)
            if sd is not None and any(l.get("type") == "driving"
                                      for l in sd.findall("lane")):
                n_dir[side] += 1
    L.append(f"  driving 차선 보유 road: {dict(n_dir)} (전체 {len(xr)}) "
             "-> 전 road 단방향(s 증가 방향만 주행)")

    def fwd(a, b):
        sa, pb = roads[a]["succ"], roads[b]["pred"]
        return bool(
            (sa and sa.get("elementType") == "road" and sa.get("elementId") == b
             and sa.get("contactPoint") == "start")
            or (pb and pb.get("elementType") == "road"
                and pb.get("elementId") == a and pb.get("contactPoint") == "end"))

    def linked(a, b):
        return any(roads[x][k] and roads[x][k].get("elementType") == "road"
                   and roads[x][k].get("elementId") == y
                   for x, y in ((a, b), (b, a)) for k in ("pred", "succ"))

    for name, path in ROUTES_CHECK.items():
        ids = [str(p) for p in path]
        miss, wrong = [], []
        for a, b in zip(ids, ids[1:]):
            if not linked(a, b):
                cand = [c["connectingRoad"] for j in junctions.values()
                        for c in j["connections"]
                        if linked(c["connectingRoad"], a)
                        and linked(c["connectingRoad"], b)]
                miss.append((a, b, cand))
            elif not fwd(a, b):
                wrong.append((a, b))
        total = sum(roads[i]["length"] for i in ids[:-1])
        L.append(f"  {name}: road {len(ids)} 커넥터누락 {miss} 역방향 {wrong}")
    L.append("  ※ 출입구3: r1238·r1368 은 모두 junction36 의 진입도로라 "
             "정방향 연결 자체가 없음(경로 무효)")


def main():
    root = ET.parse(XODR).getroot()
    roads = ag.parse_roads(root)
    junctions = ag.parse_junctions(root)
    L = []

    # ---- 공통: connection 별 접합 지표 ----
    conn_rows = []
    for jid, j in junctions.items():
        for r in ag.analyze_junction(j, roads):
            r["junction"] = jid
            conn_rows.append(r)
    anomalies = [r for r in conn_rows if r["gap_m"] >= GAP_ANOMALY_M]

    # ---- A-1 ----
    L.append("## A-1 gap 이상 분류")
    L.append(f"road {len(roads)} / junction {len(junctions)} / "
             f"connection(접합 계산됨) {len(conn_rows)}")
    L.append(f"gap>={GAP_ANOMALY_M}m 이상: {len(anomalies)}건")
    sig_of = {r["connectingRoad"]: signature(roads[r["connectingRoad"]])
              for r in anomalies}
    cnt = Counter(sig_of.values())
    known = [s for s in cnt if abs(s[0] - round(TEMPLATE_LEN, 4)) < 1e-4]
    L.append(f"서로 다른 (길이+계수) 집단: {len(cnt)}개")
    tmpl_n = sum(cnt[s] for s in known)
    rep_n = sum(c for s, c in cnt.items() if s not in known and c >= 2)
    uniq = [r for r in anomalies
            if sig_of[r["connectingRoad"]] not in known
            and cnt[sig_of[r["connectingRoad"]]] == 1]
    L.append(f"알려진 템플릿(len={TEMPLATE_LEN}) 일치: {tmpl_n}건")
    L.append(f"그 외 반복 집단(2건 이상 동일): {rep_n}건")
    L.append(f"형상이 유일(실형상): {len(uniq)}건")
    L.append("반복 집단 상위:")
    for s, c in cnt.most_common(8):
        tag = "TEMPLATE" if s in known else "repeat"
        L.append(f"  n={c} len={s[0]} {tag}")
    L.append("실형상 목록(길이·gap·방향차·위치):")
    L.append("  junction road   len_m  gap_m hdiff  along lateral x,y")
    real_list = []
    for r in sorted(uniq, key=lambda r: -r["heading_diff_deg"]):
        rd = roads[r["connectingRoad"]]
        x, y, _ = ag.road_end_state(rd, "start")
        real_list.append(r)
        L.append(f"  j{r['junction']:<4} r{r['connectingRoad']:<5} "
                 f"{rd['length']:6.2f} {r['gap_m']:6.2f} "
                 f"{r['heading_diff_deg']:5.1f} {r['along_m']:6.2f} "
                 f"{r['lateral_m']:6.2f} {x:.0f},{y:.0f}")
    hd_anom = [r for r in conn_rows if r["heading_diff_deg"] >= BIG_HDIFF_DEG]
    L.append(f"참고: heading_diff>={BIG_HDIFF_DEG}° 전체 {len(hd_anom)}건")

    # ---- A-2 ----
    L.append("")
    L.append("## A-2 짧은 road")
    lens = [r["length"] for r in roads.values()]
    bins = [0, 3, 5, 10, 20, 50, 100, 1e9]
    for i in range(len(bins) - 1):
        n = sum(1 for v in lens if bins[i] <= v < bins[i + 1])
        nj = sum(1 for r in roads.values()
                 if bins[i] <= r["length"] < bins[i + 1] and r["junction"] != "-1")
        hi = "inf" if bins[i + 1] > 1e8 else f"{bins[i+1]:g}"
        L.append(f"  {bins[i]:g}~{hi}m: {n}개 (junction내부 {nj})")
    L.append(f"min/median/max(m): {min(lens):.2f}/"
             f"{statistics.median(lens):.2f}/{max(lens):.2f}")
    short = [r for r in roads.values() if r["length"] < SHORT_M]
    short_j = [r for r in short if r["junction"] != "-1"]
    L.append(f"{SHORT_M}m 미만: {len(short)}개 / junction내부 {len(short_j)}개")
    hd_by_conn = defaultdict(list)
    for r in conn_rows:
        hd_by_conn[r["connectingRoad"]].append(r["heading_diff_deg"])
    both = []
    L.append("짧은 커넥터의 방향차(진입도로 대비) 분포:")
    hb = Counter()
    for r in short_j:
        h = max(hd_by_conn.get(r["id"], [0.0]))
        b = "0~1" if h < 1 else "1~10" if h < 10 else \
            "10~30" if h < 30 else "30+"
        hb[b] += 1
        t = abs(turn_deg(r))
        if h >= BIG_HDIFF_DEG or t >= BIG_TURN_DEG:
            both.append((r, h, t))
    for b in ("0~1", "1~10", "10~30", "30+"):
        L.append(f"  {b}°: {hb[b]}개")
    L.append(f"짧음(<{SHORT_M}m) & (방향차>={BIG_HDIFF_DEG}° 또는 "
             f"자체회전>={BIG_TURN_DEG}°): {len(both)}건")
    for r, h, t in sorted(both, key=lambda z: z[0]["length"]):
        L.append(f"  j{r['junction']:<4} r{r['id']:<5} len={r['length']:5.2f} "
                 f"hdiff={h:5.1f} turn={t:6.1f}")
    both_ids = {r["id"] for r, _, _ in both}
    L.append("자체회전만 본 짧은 커넥터 수(>=45°): "
             f"{sum(1 for r in short_j if abs(turn_deg(r)) >= BIG_TURN_DEG)}")

    # ---- A-3 ----
    L.append("")
    L.append("## A-3 확정경로 14 road")
    anom_road = {r["connectingRoad"] for r in anomalies}
    real_road = {r["connectingRoad"] for r in real_list}
    L.append("id  kind  jn  len_m  turn  joint  flags")
    risky = []
    prev = None
    prev_exit_h = None
    seen_j = set()
    for rid in ROUTE:
        rd = roads[str(rid)]
        is_j = rd["junction"] != "-1"
        if is_j:
            seen_j.add(rd["junction"])
        endA = endB = None
        if prev is not None:
            t = touch(prev[0], rd, roads, junctions)
            if t:
                endA, endB = t
        # 진행 방향: 이전 road 와 닿는 끝이 start 면 정방향
        if endB is None:
            forward = True
        else:
            forward = (endB == "start")
        hs = ag.road_end_state(rd, "start")[2]
        he = ag.road_end_state(rd, "end")[2]
        entry_h = math.degrees(hs if forward else he + math.pi)
        exit_h = math.degrees(he if forward else hs + math.pi)
        tn = norm180(exit_h - entry_h)
        joint = norm180(entry_h - prev_exit_h) if prev_exit_h is not None else 0.0
        flags = []
        if rd["length"] < SHORT_M:
            flags.append("짧음")
        if abs(tn) >= BIG_TURN_DEG:
            flags.append("급회전")
        if abs(joint) >= BIG_HDIFF_DEG:
            flags.append("접합각")
        if rd["id"] in real_road:
            flags.append("실형상이상")
        elif rd["id"] in anom_road:
            flags.append("템플릿이상")
        if rd["id"] in both_ids:
            flags.append("짧음+큰방향차")
        if prev is not None and endB is None:
            flags.append("접합미확인")
        # 위험 = 짧음+큰방향차 / 실형상이상 / (짧음 & 급회전)
        if "짧음+큰방향차" in flags or "실형상이상" in flags \
                or ("짧음" in flags and "급회전" in flags):
            risky.append(rid)
        L.append(f"{rid:<5}{'conn' if is_j else 'road':<5} "
                 f"{rd['junction']:<4}{rd['length']:6.2f} {tn:6.1f} "
                 f"{joint:6.1f} {','.join(flags) or '-'}")
        prev = (rd, forward)
        prev_exit_h = exit_h
    L.append(f"경로상 junction: {sorted(seen_j, key=int)} "
             f"(지시서 9개와 일치: {seen_j == ROUTE_JUNCTIONS})")
    r1917 = roads["1917"]
    L.append(f"road1917: junction={r1917['junction']} len={r1917['length']:.2f} "
             f"in_route={1917 in ROUTE}")
    j1 = [j for j in junctions.values() if j["name"] == "4748296080"]
    L.append(f"junction name 4748296080 -> id "
             f"{[j['id'] for j in j1]} / 경로상 여부: "
             f"{[j['id'] in seen_j for j in j1]}")
    L.append(f"위험구간(정의: 실형상이상 | 짧음+큰방향차 | 짧음&급회전): {risky}")

    # ---- 추가(사용자 요청 1~3 + 경로 접합 확인) ----
    L.append("")
    L.append("## 추가1 길이 세분(0~12m, 1m 간격 / 4~8m 0.5m 간격)")
    edges = [0, 1, 2, 3, 4, 4.5, 5, 5.5, 6, 7, 8, 10, 12]
    for i in range(len(edges) - 1):
        sel = [r for r in roads.values() if edges[i] <= r["length"] < edges[i+1]]
        nj = sum(1 for r in sel if r["junction"] != "-1")
        L.append(f"  {edges[i]:g}~{edges[i+1]:g}m: {len(sel)}개 (junction내부 {nj})")
    L.append("5m 미만 27개 개별(id,junction,len):")
    for r in sorted(short, key=lambda r: r["length"]):
        L.append(f"  r{r['id']} jn={r['junction']} len={r['length']:.3f}")

    L.append("")
    L.append("## 추가2 위험구간 판정 실측값(임계: 5m/10°/45°)")
    for r, h, t in sorted(both, key=lambda z: z[0]["length"]):
        L.append(f"  r{r['id']} len={r['length']:.3f}(임계5) "
                 f"hdiff={h:.2f}(임계10) turn={t:.2f}(임계45)")
    L.append("  ※ 1917 은 turn 0°, 1742 는 hdiff 42° — 각각 한 기준으로만 걸림")

    L.append("")
    L.append("## 추가3 템플릿 서명별 상세(gap이상 131건)")
    by_sig = defaultdict(list)
    for r in anomalies:
        by_sig[sig_of[r["connectingRoad"]]].append(r)
    for s, rows in sorted(by_sig.items(), key=lambda kv: -len(kv[1])):
        rd = roads[rows[0]["connectingRoad"]]
        g = rd["geoms"][0]
        gaps = [x["gap_m"] for x in rows]
        hds = [x["heading_diff_deg"] for x in rows]
        L.append(f"  n={len(rows)} len={rd['length']:.6f} type={g['type']}")
        if g["type"] == "paramPoly3":
            L.append("    coeffs=" + ",".join(f"{k}={v:.5f}" for k, v
                                             in g["coeffs"].items()))
        L.append(f"    gap {min(gaps):.3f}~{max(gaps):.3f} "
                 f"hdiff {min(hds):.3f}~{max(hds):.3f}")

    L.append("")
    L.append("## 추가4 heading_diff>=10° 이면서 gap<0.01 (gap 기준이 놓친 것)")
    for r in sorted(hd_anom, key=lambda r: -r["heading_diff_deg"]):
        if r["gap_m"] < GAP_ANOMALY_M:
            rd = roads[r["connectingRoad"]]
            L.append(f"  j{r['junction']} r{r['connectingRoad']} "
                     f"len={rd['length']:.2f} hdiff={r['heading_diff_deg']:.1f} "
                     f"turn={turn_deg(rd):.1f} in_route="
                     f"{int(r['connectingRoad']) in ROUTE}")
    L.append(f"  heading>=10° 중 템플릿 서명: "
             f"{sum(1 for r in hd_anom if signature(roads[r['connectingRoad']]) in known or r['connectingRoad'] in anom_road)}건")

    L.append("")
    L.append("## 추가5 경로 road 의 link (junction 2/35/36 미검출 원인 확인)")
    for rid in ROUTE:
        rd = roads[str(rid)]
        L.append(f"  r{rid} jn={rd['junction']} pred={rd['pred']} succ={rd['succ']}")

    followup(root, roads, junctions, L)

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text("\n".join(L) + "\n", encoding="utf-8")
    print(f"상세 로그: {OUT}")
    print(f"gap이상 {len(anomalies)} = 템플릿 {tmpl_n} / 반복 {rep_n}"
          f" / 실형상 {len(uniq)}")
    print(f"5m미만 {len(short)}개 (junction내부 {len(short_j)}개)")
    print(f"짧음+큰방향차 {len(both)}건")
    print(f"경로 위험구간 {len(risky)}개 {risky}")
    print(f"road1917 경로상: {1917 in ROUTE}")


if __name__ == "__main__":
    main()
