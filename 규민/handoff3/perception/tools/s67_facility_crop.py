"""10/23 5단계: 정류장 시설 점 지움을 주행 중 노드로(10/15 오프라인 s60_cloud_bag.py --facility 와 같은 방식). 컨테이너 python3 로 돈다
- 입력: /perception/obstacle_segmentation/pointcloud(지면 뺀 점군, base_link), /localization/kinematic_state(map 의 base_link 자세)
- 정류장 시설 다각형(perception/config/s67_facility_polygons.json, 노선 파일 1판 facility 10개, 지도 좌표)을 사방 --pad m 넓히고,
  점을 지도 좌표로 옮겨 다각형 안 점을 지운 점군을 --output 으로 냄. 필드·배치·header 는 그대로(행만 고름)
- 자차에서 --near m 밖 다각형은 안 봄. 처리 시간·지운 점 수를 --log CSV 로 남김
- 클러스터링 노드 입력을 이 출력으로 바꾸는 것은 s67_fac_swap.sh(실행 중 컴포넌트 내리고 다시 올림)
- 10/25: --buildings <json> 이면 건물 바닥 사각형(s69_building_polys.py, 레이어 v1·v2 건물 정육면체)도 지움. 변마다 바깥으로 --bpad m(기본 0.3)
  안 주면 10/23 과 같음(시설만). 로그 polys 칸에 지운 건물 수를 'bld<n>' 으로 더함
- 10/25: --rain-rate R(mm/h, 기본 0 = 끔)이면 지우기 전에 비 열화를 먼저 함(Goodin et al. 2019, Electronics 8(1) 89, 식 8·9 와 2.2절 순서)
  1) 거리 잡음: LiDAR 에서 잰 거리 z 를 z + N(0, 0.02 z (1 - e^-R)^2) 로, 빔 방향을 따라 점을 옮김(식 9)
  2) 점 버림: 상대 세기 Pn = rho / z'^2 * e^(-0.02 R^0.6 z') 가 Pmin = 0.9 / (pi zmax^2) 보다 작으면 버림(식 8·4)
     rho = --rain-rho(기본 0.07, 논문 그림 4 의 어두운 표적), zmax = --rain-zmax(기본 100, 센서 파일 range)
  가까운 거짓 점(빗방울 반사)은 안 더함: 같은 논문 2절 '자동차 LiDAR 에서는 매우 드묾'. 세기 필드는 안 바꿈(클러스터링이 안 씀)
  난수는 (--rain-seed, header stamp) 로 정해 같은 bag 이면 몇 번을 돌려도 같은 점군. LiDAR 위치 --rain-lidar(base_link 기준, carla_sensor_kit 값)
  끄면(기본) 10/25 와 바이트까지 같은 출력. 켜면 로그 끝에 n_rain_drop 칸
사용: python3 s67_facility_crop.py [--pad 0.3] [--output /s67/fac_crop/pointcloud] [--log csv] [--buildings json --bpad 0.3] [--rain-rate 0 --rain-seed 0]
"""
import argparse, json, math, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

ap = argparse.ArgumentParser()
ap.add_argument("--polygons", default="/home/aw/gt_perception/config/s67_facility_polygons.json")
ap.add_argument("--pad", type=float, default=0.3)
ap.add_argument("--near", type=float, default=80.0)
ap.add_argument("--input", default="/perception/obstacle_segmentation/pointcloud")
ap.add_argument("--output", default="/s67/fac_crop/pointcloud")
ap.add_argument("--log", default="")
ap.add_argument("--buildings", default="")
ap.add_argument("--bpad", type=float, default=0.3)
ap.add_argument("--rain-rate", type=float, default=0.0)
ap.add_argument("--rain-seed", type=int, default=0)
ap.add_argument("--rain-rho", type=float, default=0.07)
ap.add_argument("--rain-zmax", type=float, default=100.0)
ap.add_argument("--rain-lidar", default="1.035,0.0,1.840")
a = ap.parse_args()
RL = np.array([float(v) for v in a.rain_lidar.split(",")])
PMIN = 0.9 / (math.pi * a.rain_zmax ** 2)


def rain(m):
    """비 열화(위 설명 1·2). 반환 (새 점군 메시지, 버린 점 수)"""
    n = m.width * m.height
    if n == 0:
        return m, 0
    off = {f.name: f.offset for f in m.fields}
    buf = np.frombuffer(m.data, np.uint8).reshape(n, m.point_step).copy()
    xyz = np.stack([buf[:, off[k]:off[k] + 4].copy().view(np.float32)[:, 0].astype(np.float64) for k in ("x", "y", "z")], 1)
    v = xyz - RL; z = np.linalg.norm(v, axis=1); z[z < 1e-6] = 1e-6
    rng = np.random.default_rng([a.rain_seed, m.header.stamp.sec, m.header.stamp.nanosec])
    z2 = z + rng.standard_normal(n) * 0.02 * z * (1 - math.exp(-a.rain_rate)) ** 2
    z2 = np.maximum(z2, 1e-3)
    keep = a.rain_rho / z2 ** 2 * np.exp(-0.02 * a.rain_rate ** 0.6 * z2) >= PMIN
    p = RL + v * (z2 / z)[:, None]
    for i, k in enumerate(("x", "y", "z")):
        buf[:, off[k]:off[k] + 4] = p[:, i].astype(np.float32).view(np.uint8).reshape(n, 4)
    kb = buf[keep]
    return PointCloud2(header=m.header, height=1, width=int(kb.shape[0]), fields=m.fields, is_bigendian=m.is_bigendian,
                       point_step=m.point_step, row_step=m.point_step * int(kb.shape[0]), data=kb.tobytes(), is_dense=m.is_dense), int(n - keep.sum())


def grow(poly, d):   # s60_cloud_bag.py grow 와 같은 방식(중심에서 꼭짓점 쪽으로 d 만큼)
    cx = sum(p[0] for p in poly) / len(poly); cy = sum(p[1] for p in poly) / len(poly)
    out = []
    for x, y in poly:
        r = math.hypot(x - cx, y - cy); out.append((cx + (x - cx) * (1 + d / r), cy + (y - cy) * (1 + d / r)))
    return out


POLYS = {k: np.array(grow(v, a.pad)) for k, v in json.load(open(a.polygons))["polygons"].items()}
# 건물: (cx, cy, cos, sin, 반 변 + bpad) 배열. 정사각형이라 바깥 원 반경 = 반 변 x 루트2 로 먼저 거름
BLD = np.array([(r["cx"], r["cy"], math.cos(math.radians(r["yaw_deg"])), math.sin(math.radians(r["yaw_deg"])), r["side"] / 2 + a.bpad)
                for r in json.load(open(a.buildings))["rects"]]) if a.buildings else np.zeros((0, 5))


def inside(px, py, poly):
    ins = np.zeros(px.shape, bool); n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]; x2, y2 = poly[(i + 1) % n]
        c = (y1 > py) != (y2 > py)
        xi = x1 + (py - y1) * (x2 - x1) / ((y2 - y1) if y2 != y1 else 1e-12)
        ins ^= c & (px < xi)
    return ins


class Crop(Node):
    def __init__(s):
        super().__init__("s67_facility_crop")
        s.ego = None
        s.create_subscription(Odometry, "/localization/kinematic_state", s.on_odom, 10)
        s.create_subscription(PointCloud2, a.input, s.on_cloud, qos_profile_sensor_data)
        s.pub = s.create_publisher(PointCloud2, a.output, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST))
        s.f = open(a.log, "w") if a.log else None
        if s.f: s.f.write("stamp,n_in,n_removed,polys,proc_ms%s\n" % (",n_rain_drop" if a.rain_rate > 0 else ""))

    def on_odom(s, m):
        p = m.pose.pose; q = p.orientation
        s.ego = (p.position.x, p.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def on_cloud(s, m):
        t0 = time.perf_counter(); rd = 0
        if a.rain_rate > 0:
            m, rd = rain(m)
        n = m.width * m.height; rm = 0; used = []
        out = m
        if s.ego is not None and n > 0:
            ex, ey, eh = s.ego
            near = [k for k, P in POLYS.items() if math.hypot(P[:, 0].mean() - ex, P[:, 1].mean() - ey) < a.near]
            bnear = BLD[np.hypot(BLD[:, 0] - ex, BLD[:, 1] - ey) < a.near + BLD[:, 4] * 1.415] if len(BLD) else BLD
            if near or len(bnear):
                off = {f.name: f.offset for f in m.fields}
                buf = np.frombuffer(m.data, np.uint8).reshape(n, m.point_step)
                x = buf[:, off["x"]:off["x"] + 4].copy().view(np.float32)[:, 0].astype(np.float64)
                y = buf[:, off["y"]:off["y"] + 4].copy().view(np.float32)[:, 0].astype(np.float64)
                c, si = math.cos(eh), math.sin(eh)
                mx = ex + x * c - y * si; my = ey + x * si + y * c
                kill = np.zeros(n, bool)
                for k in near:
                    P = POLYS[k]
                    bb = (mx >= P[:, 0].min()) & (mx <= P[:, 0].max()) & (my >= P[:, 1].min()) & (my <= P[:, 1].max())
                    if bb.any():
                        kill[bb] |= inside(mx[bb], my[bb], P); used.append(k)
                nb = 0
                for bx, by, bc, bs, h in bnear:
                    cand = (np.abs(mx - bx) <= h * 1.415) & (np.abs(my - by) <= h * 1.415)
                    if not cand.any(): continue
                    dx, dy = mx[cand] - bx, my[cand] - by
                    hit = (np.abs(dx * bc + dy * bs) <= h) & (np.abs(-dx * bs + dy * bc) <= h)
                    if hit.any():
                        kill[np.flatnonzero(cand)[hit]] = True; nb += 1
                if nb: used.append("bld%d" % nb)
                rm = int(kill.sum())
                if rm:
                    keep = buf[~kill]
                    out = PointCloud2(header=m.header, height=1, width=int(keep.shape[0]), fields=m.fields, is_bigendian=m.is_bigendian,
                                      point_step=m.point_step, row_step=m.point_step * int(keep.shape[0]), data=keep.tobytes(), is_dense=m.is_dense)
        s.pub.publish(out)
        if s.f:
            st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            s.f.write("%.3f,%d,%d,%s,%.2f%s\n" % (st, n, rm, "|".join(used), (time.perf_counter() - t0) * 1000,
                                                   (",%d" % rd) if a.rain_rate > 0 else "")); s.f.flush()


rclpy.init(); node = Crop()
try:
    rclpy.spin(node)
except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    pass
