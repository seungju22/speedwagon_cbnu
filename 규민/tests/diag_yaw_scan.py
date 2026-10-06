"""진단(세션8): 급굴곡 전수 조회 + 세션7 속도-s 대조. 읽기전용.

서버에 878 road 맵이 로드돼 있어야 한다. 액터 생성 없음.
- generate_waypoints(1.0)로 얻은 (road, lane) 별 waypoint 를 s 순으로 정렬
- 5m 창에서 yaw 변화 절대값 >= THRESH 인 지점을 급굴곡으로 센다
- 확정경로 15 road 위 급굴곡과, 세션7 CSV 의 같은 s 구간 속도를 함께 출력
"""
import csv
from collections import defaultdict
from pathlib import Path

import carla

HOST, PORT, TIMEOUT = "localhost", 2000, 60.0
WINDOW = 5      # waypoint 개수(1m 간격이므로 5m)
THRESH = 25.0   # 도, 5m 창 안 yaw 변화
LOGS = Path(__file__).resolve().parents[1] / "docs/logs"
PLAN_CSV = LOGS / "drive_plan_20260919_231354.csv"
DRIVE_CSV = LOGS / "drive_log_20260919_231354.csv"


def dyaw(a, b):
    return (b - a + 180.0) % 360.0 - 180.0


def main():
    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)
    cmap = client.get_world().get_map()
    print("맵:", cmap.name)

    plan_roads = {int(p["road_id"]) for p in csv.DictReader(open(PLAN_CSV))}
    drive = list(csv.DictReader(open(DRIVE_CSV)))

    lanes = defaultdict(list)
    for wp in cmap.generate_waypoints(1.0):
        lanes[(wp.road_id, wp.lane_id)].append(wp)
    print(f"(road,lane) {len(lanes)}개")

    hits = []  # (road, lane, s_start, s_end, total_dyaw, is_junction)
    for (rid, lid), wps in lanes.items():
        wps.sort(key=lambda w: w.s)
        if lid > 0:  # 반대방향 차선은 s 증가가 진행방향과 반대라 부호만 다르다
            pass
        i = 0
        while i + WINDOW < len(wps):
            d = dyaw(wps[i].transform.rotation.yaw, wps[i + WINDOW].transform.rotation.yaw)
            if abs(d) >= THRESH:
                j = i + WINDOW
                hits.append((rid, lid, wps[i].s, wps[j].s, d, wps[i].is_junction))
                i = j  # 같은 굴곡 중복 방지
            else:
                i += 1

    plain = [h for h in hits if not h[5]]
    print(f"\n급굴곡(>= {THRESH:.0f}도/{WINDOW}m) 전체 {len(hits)}개, 커넥터 밖 {len(plain)}개")

    print("\n=== 확정경로 15 road 위 (차량 진행 차선 lane<0, 커넥터 밖) ===")
    on_plan = sorted((h for h in plain if h[0] in plan_roads and h[1] < 0),
                     key=lambda h: (h[0], h[2]))
    for rid, lid, s0, s1, d, _ in on_plan:
        print(f"road{rid} lane{lid} s={s0:.1f}~{s1:.1f} yaw변화 {d:+.1f}도")
    print(f"경로 위 커넥터 밖 급굴곡: {len(on_plan)}개")

    print("\n=== 세션7 road1247 s 구간별 속도(m/s) ===")
    bins = defaultdict(list)
    for r in drive:
        if r["road_id"] == "1247":
            bins[int(float(r["s"]) // 2 * 2)].append(float(r["speed_mps"]))
    for b in sorted(bins):
        if 60 <= b <= 94:
            v = bins[b]
            print(f"s={b}~{b + 2}: 평균 {sum(v) / len(v):.2f} 최소 {min(v):.2f} 최대 {max(v):.2f}")


if __name__ == "__main__":
    main()
