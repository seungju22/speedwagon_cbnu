"""10/23 4단계: 거리별 bag 에서 처리 시간 토픽과 자차 앞 점을 뽑는다. 컨테이너 python3(rosbag2_py) 로 돈다
- processing_time_ms(Float64Stamped) -> <접두어>_ptime.csv(stamp, ms)
- 점군 두 개를 base_link 좌표로 옮겨 자차 앞 구역(x -5~50, y -10~10)만 남김 -> <접두어>_pts.npz
  raw  = /sensing/lidar/top/pointcloud_before_sync(브리지 LiDAR, 센서 좌표 -> tf_static 으로 base_link)
  seg  = /perception/obstacle_segmentation/pointcloud(지면 뺀 점군, base_link 인지 header 로 확인)
  npz 키: <이름>_stamp(메시지 stamp), <이름>_idx(메시지별 시작 위치), <이름>_xyz(float32)
사용: python3 s68_bag_points.py <bag> <출력 접두어>
"""
import csv, json, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2

bag, pre = sys.argv[1], sys.argv[2]
TOP = {"/sensing/lidar/top/pointcloud_before_sync": "raw", "/perception/obstacle_segmentation/pointcloud": "seg"}
PT = "/perception/object_recognition/detection/clustering/voxel_grid_based_euclidean_cluster/debug/processing_time_ms"


def reader():
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    return r, types


def quat_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# tf_static: child -> parent 를 모아 base_link 까지 이음
r, types = reader(); tf = {}
while r.has_next():
    t, d, _ = r.read_next()
    if t == "/tf_static":
        for tr in deserialize_message(d, get_message(types[t])).transforms:
            R = quat_mat(tr.transform.rotation); p = tr.transform.translation
            tf[tr.child_frame_id] = (tr.header.frame_id, R, np.array([p.x, p.y, p.z]))


def to_base(frame):
    R, p = np.eye(3), np.zeros(3); f = frame; chain = [f]
    while f != "base_link":
        if f not in tf: return None, None, chain
        par, Rp, pp = tf[f]; R, p = Rp @ R, Rp @ p + pp; f = par; chain.append(f)
    return R, p, chain


r, types = reader(); out = {k: {"stamp": [], "idx": [], "xyz": []} for k in TOP.values()}; pos = {k: 0 for k in TOP.values()}
pt = []; frames = {}
while r.has_next():
    t, d, _ = r.read_next()
    if t == PT:
        m = deserialize_message(d, get_message(types[t])); pt.append((m.stamp.sec + m.stamp.nanosec * 1e-9, m.data))
    elif t in TOP:
        k = TOP[t]; m = deserialize_message(d, get_message(types[t]))
        if k not in frames:
            R, p, chain = to_base(m.header.frame_id); frames[k] = {"frame": m.header.frame_id, "chain": chain, "R": R, "p": p}
        R, p = frames[k]["R"], frames[k]["p"]
        s = point_cloud2.read_points(m, field_names=("x", "y", "z"), skip_nans=True)   # 필드 형식이 섞여 있어 구조 배열로 읽음
        a = np.stack([s["x"], s["y"], s["z"]], axis=1).astype(np.float64)
        if R is not None: a = a @ R.T + p
        a = a[(a[:, 0] > -5) & (a[:, 0] < 50) & (np.abs(a[:, 1]) < 10)].astype(np.float32)
        o = out[k]; o["stamp"].append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9); o["idx"].append(pos[k]); o["xyz"].append(a); pos[k] += len(a)
np.savez_compressed(pre + "_pts.npz", **{f"{k}_{n}": (np.concatenate(v[n]) if n == "xyz" else np.array(v[n])) for k, v in out.items() for n in v})
with open(pre + "_ptime.csv", "w", newline="") as f:
    w = csv.writer(f); w.writerow(["stamp", "ms"]); w.writerows(pt)
print(json.dumps({"ptime_msgs": len(pt), **{k: {"msgs": len(v["stamp"]), "pts": pos[k], "frame": frames.get(k, {}).get("frame"),
                  "chain": frames.get(k, {}).get("chain")} for k, v in out.items()}}, ensure_ascii=False))
