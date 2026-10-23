"""차 시점 판정(P1b, 세션58). gt_objects2.py 와 대조 도구(perception/tools/p1b_lidar_bench.py)가 같이 쓴다.
보이는가 = 거리 안 + 표본점 중 LiDAR 상하 시야 안이고 cast_ray 가 안 막힌 것이 하나 이상.
막힘으로 안 치는 것: 그 물체 자기 상자(0.2m 키움) 안, 라벨 NONE 이고 어느 보행자든 그 보행자의 보이지 않는 상자
  (보행자 좌표 x -2.34~+2.78, y ±0.85) 안인 점. 이 상자가 cast_ray 에 걸린다(세션58 실측. 일반 LiDAR 점은 보행자 몸에 맞는다).
  레이어 건물 상자도 라벨 NONE 이라 NONE 을 통째로 뺄 수는 없다. 그래서 보행자 상자 안 건물 벽은 가림으로 못 본다(한계), 자차 상자(0.1m 키움) 안, 지면(라벨 또는 z < 0.15m)
"""
import math
import carla

GROUND_TAGS = {carla.CityObjectLabel.Ground, carla.CityObjectLabel.Roads, carla.CityObjectLabel.Sidewalks,
               carla.CityObjectLabel.RoadLines, carla.CityObjectLabel.Terrain}
# 브리지 0.52.0: velodyne_top_base_link (x 1.035, z 1.840, base_link) - 축거 2.850/2 -> CARLA 차 원점 기준
LIDAR_CAR = (-0.390, 0.0, 1.840)
# 보행자 둘레 보이지 않는 상자(cast_ray 라벨 NONE). 세션58 실측: 보행자 좌표 x -2.34~+2.78, y ±0.85, 청사진 4종·방향 무관.
# 일반·시맨틱 LiDAR 점은 이 상자가 아니라 보행자 몸에 맞는다. 0.1m 여유
WALKER_SHELL_LOC = carla.Location(0.22, 0.0, 0.0)
WALKER_SHELL_EXT = carla.Vector3D(2.66, 0.95, 1.2)


def inside(p, tf, bloc, ext, pad, pad_xy=None):
    """세계 점 p 가 actor(tf) 의 bbox(bloc, ext) 를 pad(수평은 pad_xy) 만큼 키운 상자 안인가"""
    pxy = pad if pad_xy is None else pad_xy
    M = tf.get_inverse_matrix()     # 0.9.15 Transform 에는 inverse_transform 이 없다
    qx, qy, qz = (M[i][0] * p.x + M[i][1] * p.y + M[i][2] * p.z + M[i][3] for i in range(3))
    return abs(qx - bloc.x) <= ext.x + pxy and abs(qy - bloc.y) <= ext.y + pxy and abs(qz - bloc.z) <= ext.z + pad


def samples(tf, bloc, ext):
    """가운데, 위 가운데, 가운데 높이 모서리 4, 위 모서리 4 (모서리는 0.9 배 안쪽)"""
    pts = [(0, 0, 0), (0, 0, ext.z * 0.9)]
    for sx in (-1, 1):
        for sy in (-1, 1):
            pts.append((sx * ext.x * 0.9, sy * ext.y * 0.9, 0))
            pts.append((sx * ext.x * 0.9, sy * ext.y * 0.9, ext.z * 0.9))
    return [tf.transform(carla.Location(bloc.x + x, bloc.y + y, bloc.z + z)) for x, y, z in pts]


def lidar_location(ego_tf, lidar=LIDAR_CAR):
    return ego_tf.transform(carla.Location(*lidar))


def judge(world, lid, tf, bloc, ext, rng, lower, upper, ego_tf=None, ego_bb=None, all_samples=False, self_pad_xy=0.2, walkers=()):
    """walkers: 보이지 않는 상자를 무시할 보행자 transform 목록. 대상 보행자도 넣는다"""
    """-> (dist, in_range, n_fov, n_clear). all_samples 면 안 막힌 표본을 끝까지 센다(대조용)"""
    cen = tf.transform(carla.Location(bloc.x, bloc.y, bloc.z))
    d = cen.distance(lid)
    if d > rng:
        return d, False, 0, 0
    n_fov = n_clear = 0
    for p in samples(tf, bloc, ext):
        h = math.hypot(p.x - lid.x, p.y - lid.y)
        el = math.degrees(math.atan2(p.z - lid.z, h))
        if el < lower or el > upper:
            continue
        n_fov += 1
        seg = p.distance(lid)
        blocked = False
        for hit in sorted(world.cast_ray(lid, p), key=lambda h: h.location.distance(lid)):
            hl = hit.location
            if hl.distance(lid) >= seg - 0.05:
                break
            if hit.label in GROUND_TAGS or hl.z < 0.15:
                continue
            if inside(hl, tf, bloc, ext, 0.2, self_pad_xy):
                continue
            if hit.label == carla.CityObjectLabel.NONE and any(
                    inside(hl, wt, WALKER_SHELL_LOC, WALKER_SHELL_EXT, 0.0) for wt in walkers):
                continue
            if ego_bb is not None and inside(hl, ego_tf, ego_bb.location, ego_bb.extent, 0.1):
                continue
            blocked = True
            break
        if not blocked:
            n_clear += 1
            if not all_samples:
                break
    return d, True, n_fov, n_clear
