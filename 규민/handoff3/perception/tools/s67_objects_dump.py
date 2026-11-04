"""10/23: 클러스터링 객체 bag 을 CSV 로 풀고 처리 시간을 센다. 컨테이너 python3(rosbag2_py) 로 돈다.
- 객체 토픽(DetectedObjects 계열)마다 메시지 하나 = 틱 하나. 객체 중심을 지도 좌표로 옮겨 한 줄씩 쓴다
  frame 이 map 이 아니면(클러스터링 출력은 base_link) 자차 자세(/localization/kinematic_state, 가장 가까운 stamp)로 옮긴다
- 처리 시간: 입력 점군(--input)과 같은 header.stamp 를 가진 출력 메시지의 받은 시각(bag 기록 시각) 차. 기록기가 받은 시각이라 DDS 전달이 들어간다
사용: python3 s67_objects_dump.py <객체 bag> <자세 bag> <출력 접두어> --input <입력 점군 토픽> --topics <토픽,...>
출력: <접두어>_objects.csv(topic,stamp,recv_s,idx,label,x,y,z,yaw,dx,dy,dz,frame), <접두어>_ticks.csv(topic,stamp,recv_s,n,lat_ms), <접두어>_dump.json
"""
import argparse, bisect, csv, json, math
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ap = argparse.ArgumentParser()
ap.add_argument("objbag"); ap.add_argument("posebag"); ap.add_argument("out")
ap.add_argument("--input", default="/perception/obstacle_segmentation/pointcloud")
ap.add_argument("--topics", required=True)
ap.add_argument("--pose-topic", default="/localization/kinematic_state")
a = ap.parse_args()
TOPICS = a.topics.split(",")


def read(uri, topics, decode=True):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    while r.has_next():
        t, d, rt = r.read_next()
        yield t, rt, deserialize_message(d, get_message(types[t]))


def st(h): return round(h.stamp.sec + h.stamp.nanosec * 1e-9, 4)


def yaw_of(q): return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


ego = []
for _, _, m in read(a.posebag, [a.pose_topic]):
    p = m.pose.pose
    ego.append((st(m.header), p.position.x, p.position.y, yaw_of(p.orientation)))
ego.sort(); et = [e[0] for e in ego]


def ego_at(s):
    i = bisect.bisect_left(et, s)
    c = [j for j in (i - 1, i) if 0 <= j < len(et)]
    return ego[min(c, key=lambda j: abs(et[j] - s))]


inp = {}   # stamp -> 받은 시각(첫 번째)
ticks = []; rows = []
for t, rt, m in read(a.objbag, [a.input] + TOPICS):
    s = st(m.header)
    if t == a.input:
        inp.setdefault(s, rt); continue
    objs = m.objects if hasattr(m, "objects") else [f.object for f in m.feature_objects]   # 클러스터(WithFeature)는 feature_objects
    fr = m.header.frame_id; n = len(objs)
    ticks.append([t, s, rt, n])
    if fr != "map":
        _, ex, ey, eh = ego_at(s); c, si = math.cos(eh), math.sin(eh)
    for k, o in enumerate(objs):
        k_ = o.kinematics
        pose = k_.pose_with_covariance.pose if hasattr(k_, "pose_with_covariance") else k_.initial_pose_with_covariance.pose
        x, y, z = pose.position.x, pose.position.y, pose.position.z; yw = yaw_of(pose.orientation)
        if fr != "map":
            x, y, yw = ex + x * c - y * si, ey + x * si + y * c, yw + eh
        lab = max(o.classification, key=lambda q: q.probability).label if o.classification else -1
        d = o.shape.dimensions
        rows.append([t, s, round(rt * 1e-9, 4), k, lab, round(x, 3), round(y, 3), round(z, 3), round(yw, 4),
                     round(d.x, 3), round(d.y, 3), round(d.z, 3), fr])
with open(a.out + "_objects.csv", "w", newline="") as f:
    w = csv.writer(f); w.writerow(["topic", "stamp", "recv_s", "idx", "label", "x", "y", "z", "yaw", "dx", "dy", "dz", "frame"]); w.writerows(rows)
summ = {"input_msgs": len(inp), "pose_msgs": len(ego), "topics": {}}
with open(a.out + "_ticks.csv", "w", newline="") as f:
    w = csv.writer(f); w.writerow(["topic", "stamp", "recv_s", "n", "lat_ms"])
    for t in TOPICS:
        tk = [x for x in ticks if x[0] == t]; lat = []; ns = []
        for _, s, rt, n in tk:
            li = (rt - inp[s]) * 1e-6 if s in inp else None
            if li is not None: lat.append(li)
            ns.append(n); w.writerow([t, s, round(rt * 1e-9, 4), n, round(li, 2) if li is not None else ""])
        lat.sort(); ns.sort()
        q = lambda v, p: v[min(len(v) - 1, int(p * len(v)))] if v else None
        summ["topics"][t] = {"msgs": len(tk), "matched": len(lat), "lat_ms_med": q(lat, 0.5), "lat_ms_p95": q(lat, 0.95), "lat_ms_max": lat[-1] if lat else None,
                             "n_med": q(ns, 0.5), "n_p95": q(ns, 0.95), "n_max": ns[-1] if ns else None, "frames": sorted({r[12] for r in rows if r[0] == t})}
json.dump(summ, open(a.out + "_dump.json", "w"), ensure_ascii=False, indent=1, default=float)
print(json.dumps(summ, ensure_ascii=False, default=float))
