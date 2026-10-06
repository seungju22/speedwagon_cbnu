#!/usr/bin/env python3
# 맵세션9 Phase A 추가진단. 읽기전용, CARLA 불필요.
# 세션7 CSV(drive_log_20260919_231354.csv)로 물리접촉 vs TM제동 가설 1차 스크리닝.
# 1. z 변화(연석을 탔다면 상승)
# 2. 차선(lane -1) 중심선 대비 차량 횡방향 편차(xodr 기하로 중심선 계산)
# 3. 제어값(throttle/brake/steer) 기록 여부
# 4. 차량 진행방향 yaw vs 도로 yaw 차이(모서리 자르기 여부)
# 5. 통과구간(s=80~85) vs 정지구간(s=88~92.3) 비교
# + xodr 기하: 인도(lane -2) 안쪽 경계까지 거리(양쪽 구간)
import csv
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_geometry import parse_roads, geom_point, geom_heading

BASE = Path(__file__).resolve().parent.parent
XODR_PATH = BASE / "maps" / "cbnu_internal_only_localtm_tags73.xodr"
CSV_PATH = BASE / "docs" / "logs" / "drive_log_20260919_231354.csv"
OUT_PATH = BASE / "docs" / "map_session09_curb_analysis.md"

LANE1_T = -1.675       # lane -1 중심 (reference line 기준 t, 폭 3.35 절반)
SIDEWALK_INNER_T = -3.35  # lane -1/-2 경계(인도 안쪽 경계)
VEHICLE_HALF_WIDTH = 0.85  # audi.a2 약 1.7m 폭의 절반(추정치, 근거: 사용자 제공)


def road_point_and_heading(road, s):
    geom = None
    for g in road["geoms"]:
        if g["s"] - 1e-6 <= s <= g["s"] + g["length"] + 1e-6:
            geom = g
    if geom is None:
        geom = road["geoms"][-1]
    p = (s - geom["s"]) / geom["length"] if geom["length"] > 0 else 0.0
    p = min(max(p, 0.0), 1.0)
    x, y = geom_point(geom, p)
    yaw = geom_heading(geom, p)
    return x, y, yaw


def offset_point(road, s, t):
    x, y, yaw = road_point_and_heading(road, s)
    # OpenDRIVE 관례: t 양수=진행방향 왼쪽. normal=(-sin(hdg), cos(hdg))
    # xodr 내부 좌표계 기준(자기일관적). CARLA 좌표와 비교할 땐 to_carla()로 변환.
    ox = x + t * (-math.sin(yaw))
    oy = y + t * math.cos(yaw)
    return ox, oy, yaw


def to_carla(x, y, yaw):
    # test_drive.py 확인 사항: CARLA world 좌표는 xodr y 부호 반전.
    return x, -y, -yaw


def load_csv_rows():
    rows = []
    with open(CSV_PATH, newline="") as f:
        for row in csv.DictReader(f):
            rows.append(row)
    return rows


def dedupe_by_s(rows, road_id, bin_size=0.1):
    seen = {}
    for r in rows:
        if r["road_id"] != str(road_id):
            continue
        s = float(r["s"])
        key = round(s / bin_size)
        if key not in seen:
            seen[key] = r
    return [seen[k] for k in sorted(seen)]


def analyze_range(road, csv_rows, s_lo, s_hi, label, lines):
    pts = [r for r in csv_rows if s_lo <= float(r["s"]) <= s_hi]
    if not pts:
        lines.append(f"- {label}: CSV 표본 없음(s={s_lo}~{s_hi})")
        return

    zs = [float(r["z"]) for r in pts]
    z0, z1, zmax = zs[0], zs[-1], max(zs)
    lines.append(f"### {label} (s={s_lo}~{s_hi}, 표본 {len(pts)}개)")
    lines.append(f"- z: 시작={z0:.4f} 끝={z1:.4f} 최대={zmax:.4f} "
                 f"변화(끝-시작)={z1 - z0:+.4f}m 최대상승={zmax - z0:+.4f}m")

    # 횡방향 편차: 각 표본의 s에서 lane-1 중심선(x,y) 계산 후 차량과의 거리
    lat_devs = []
    for r in pts:
        s = float(r["s"])
        lx, ly, lyaw = offset_point(road, s, LANE1_T)
        lx, ly, lyaw = to_carla(lx, ly, lyaw)
        vx, vy = float(r["x"]), float(r["y"])
        dist = math.hypot(vx - lx, vy - ly)
        # 부호: 진행방향(lyaw) 기준 왼쪽normal=(-sin(lyaw),cos(lyaw))에 투영.
        # 음수=진행방향 오른쪽(인도 쪽)으로 치우침, 양수=왼쪽(도로중앙 쪽)
        nx, ny = -math.sin(lyaw), math.cos(lyaw)
        signed = (vx - lx) * nx + (vy - ly) * ny
        lat_devs.append((s, dist, signed))
    max_dev = max(lat_devs, key=lambda d: d[1])
    toward_sw = [d for d in lat_devs if d[2] < 0]
    max_toward_sw = max(toward_sw, key=lambda d: d[1]) if toward_sw else None
    lines.append(f"- 차선중심 대비 횡편차(절대값): 최대={max_dev[1]:.3f}m (s={max_dev[0]:.2f}) "
                 f"부호={max_dev[2]:+.3f}m 평균={sum(d[1] for d in lat_devs) / len(lat_devs):.3f}m")
    if max_toward_sw:
        lines.append(f"- 인도쪽(오른쪽) 치우침 최대: {max_toward_sw[1]:.3f}m (s={max_toward_sw[0]:.2f})")
    else:
        lines.append("- 인도쪽(오른쪽) 치우침: 없음(전 구간 왼쪽/중앙 쪽으로만 편차)")

    # 차량 yaw(진행방향, 연속 표본 간 dx,dy) vs 도로 yaw
    yaw_diffs = []
    for i in range(1, len(pts)):
        a, b = pts[i - 1], pts[i]
        dx = float(b["x"]) - float(a["x"])
        dy = float(b["y"]) - float(a["y"])
        if math.hypot(dx, dy) < 0.05:
            continue
        veh_yaw = math.atan2(dy, dx)
        s_mid = (float(a["s"]) + float(b["s"])) / 2
        _, _, road_yaw_xodr = road_point_and_heading(road, s_mid)
        _, _, road_yaw = to_carla(0.0, 0.0, road_yaw_xodr)
        diff = math.degrees(((veh_yaw - road_yaw + math.pi) % (2 * math.pi)) - math.pi)
        yaw_diffs.append((s_mid, diff))
    if yaw_diffs:
        max_yd = max(yaw_diffs, key=lambda d: abs(d[1]))
        lines.append(f"- 차량진행yaw - 도로yaw: 최대편차={max_yd[1]:+.2f}deg (s={max_yd[0]:.2f}) "
                     f"평균절대값={sum(abs(d[1]) for d in yaw_diffs) / len(yaw_diffs):.2f}deg")
    else:
        lines.append("- 차량진행yaw 계산 불가(표본 간 이동거리 부족, 정지 상태)")

    # 제어값
    ctrl_keys = [k for k in pts[0].keys() if k in ("throttle", "brake", "steer")]
    lines.append(f"- 제어값 컬럼: {ctrl_keys if ctrl_keys else '없음'}")

    return {"z0": z0, "z1": z1, "zmax": zmax, "max_dev": max_dev, "yaw_diffs": yaw_diffs}


def sidewalk_clearance(road, s_lo, s_hi, step, lines, label):
    lines.append(f"### {label} 인도 안쪽 경계 거리(s={s_lo}~{s_hi})")
    s = s_lo
    min_clear = None
    while s <= s_hi + 1e-6:
        lx, ly, _ = offset_point(road, s, LANE1_T)
        swx, swy, _ = offset_point(road, s, SIDEWALK_INNER_T)
        clear = math.hypot(swx - lx, swy - ly)
        if min_clear is None or clear < min_clear[1]:
            min_clear = (s, clear)
        s += step
    lines.append(f"- 차선중심~인도안쪽경계 최소거리: {min_clear[1]:.3f}m (s={min_clear[0]:.2f})")
    lines.append(f"- 차량 반폭({VEHICLE_HALF_WIDTH}m) 대비 여유: "
                 f"{min_clear[1] - VEHICLE_HALF_WIDTH:.3f}m")
    return min_clear


def main():
    root = ET.parse(XODR_PATH).getroot()
    roads = parse_roads(root)
    road1247 = roads["1247"]

    all_rows = load_csv_rows()
    csv_rows_1247 = dedupe_by_s(all_rows, 1247, bin_size=0.1)

    lines = []
    lines.append("# 맵세션9 물리접촉 vs TM제동 스크리닝 (읽기전용, 자동생성)")
    lines.append("")
    lines.append(f"- CSV: {CSV_PATH.name}, road1247 중복제거 표본 {len(csv_rows_1247)}개(0.1m bin)")
    lines.append(f"- CSV 헤더: {list(all_rows[0].keys())}")
    lines.append("")

    pass_res = analyze_range(road1247, csv_rows_1247, 80.0, 85.0, "통과구간", lines)
    lines.append("")
    stop_res = analyze_range(road1247, csv_rows_1247, 88.0, 92.3, "정지구간(감속시작~정지)", lines)
    lines.append("")

    pass_clear = sidewalk_clearance(road1247, 80.0, 85.0, 0.2, lines, "통과구간")
    lines.append("")
    stop_clear = sidewalk_clearance(road1247, 93.0, 96.0, 0.2, lines, "정지구간")
    lines.append("")

    OUT_PATH.write_text("\n".join(lines), encoding="utf-8")
    print(f"완료: {OUT_PATH}")
    if pass_res:
        print(f"통과 z변화={pass_res['z1']-pass_res['z0']:+.4f}m 최대횡편차={pass_res['max_dev'][1]:.3f}m")
    if stop_res:
        print(f"정지 z변화={stop_res['z1']-stop_res['z0']:+.4f}m 최대횡편차={stop_res['max_dev'][1]:.3f}m")
    print(f"통과구간 인도경계 최소거리={pass_clear[1]:.3f}m, 정지구간={stop_clear[1]:.3f}m")


if __name__ == "__main__":
    main()
