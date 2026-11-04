"""10/25: 쓸기 결과 bag(s69_sweep_offline.sh)을 CSV 로 풂. 컨테이너 python3(rosbag2_py) 로 돈다
- /s69/<조합>/objects(DetectedObjectsWithFeature, base_link)마다 객체 중심을 지도 좌표로 옮김(자세 = 원래 bag 의 /localization/kinematic_state, 가장 가까운 stamp)
- 객체마다 클러스터 점 수(n_pts)와 클러스터 점 가운데 자차 앞 방향(base_link x)으로 가장 가까운 점의 x(minx_base, '가장 가까운 면' 오차용)
- 조합마다 클러스터 debug/processing_time_ms
출력: <접두어>_objects.csv(topic,stamp,label,x,y,n_pts,minx_base,miny_base), <접두어>_ticks.csv(topic,stamp,n), <접두어>_ptime.csv(topic,stamp,ms), <접두어>_dump.json
사용: python3 s69_sweep_dump.py <쓸기 bag> <자세 bag> <출력 접두어>
"""
import bisect, collections, csv, json, math, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2

swbag, posebag, pre = sys.argv[1:4]


def read(uri, want=None):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    if want is not None: r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in types if want(t)]))
    while r.has_next():
        t, d, rt = r.read_next()
        yield t, deserialize_message(d, get_message(types[t]))


st = lambda h: round(h.stamp.sec + h.stamp.nanosec * 1e-9, 4)
yaw_of = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
ego = sorted((st(m.header), m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
             for _, m in read(posebag, lambda t: t == "/localization/kinematic_state"))
et = [e[0] for e in ego]


def ego_at(s):
    i = bisect.bisect_left(et, s); c = [j for j in (i - 1, i) if 0 <= j < len(et)]
    return ego[min(c, key=lambda j: abs(et[j] - s))]


rows, ticks, pts = [], [], []
for t, m in read(swbag, lambda t: t.endswith("/objects") or t.endswith("processing_time_ms")):
    if t.endswith("processing_time_ms"):
        pts.append([t.split("/")[2], round(m.stamp.sec + m.stamp.nanosec * 1e-9, 4), round(m.data, 4)]); continue
    s = st(m.header); c = t.split("/")[2]; _, ex, ey, eh = ego_at(s); co, si = math.cos(eh), math.sin(eh)
    ticks.append([c, s, len(m.feature_objects)])
    for fo in m.feature_objects:
        o = fo.object; p = o.kinematics.pose_with_covariance.pose.position
        lab = max(o.classification, key=lambda q: q.probability).label if o.classification else -1
        a = point_cloud2.read_points(fo.feature.cluster, field_names=("x", "y"), skip_nans=True)
        n = len(a); mx = my = ""
        if n:
            k = int(np.argmin(a["x"])); mx, my = round(float(a["x"][k]), 3), round(float(a["y"][k]), 3)
        rows.append([c, s, lab, round(ex + p.x * co - p.y * si, 3), round(ey + p.x * si + p.y * co, 3), n, mx, my])
for suf, head, data in (("objects", ["topic", "stamp", "label", "x", "y", "n_pts", "minx_base", "miny_base"], rows),
                        ("ticks", ["topic", "stamp", "n"], ticks), ("ptime", ["topic", "stamp", "ms"], pts)):
    with open(f"{pre}_{suf}.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(head); w.writerows(data)
summ = {c: {"msgs": sum(1 for x in ticks if x[0] == c), "objs": sum(1 for x in rows if x[0] == c), "ptime": sum(1 for x in pts if x[0] == c)}
        for c in sorted({x[0] for x in ticks})}
json.dump({"pose_msgs": len(ego), "configs": summ}, open(pre + "_dump.json", "w"), ensure_ascii=False, indent=1)
print(json.dumps({"pose_msgs": len(ego), "configs": summ}, ensure_ascii=False))
