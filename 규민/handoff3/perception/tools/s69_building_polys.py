"""10/25 2단계: 레이어 배치 데이터(건물 1순위 buildings·2순위 buildings2)에서 건물 바닥 사각형을 뽑아 점 지움 노드용 JSON 으로 씀
- 입력: map/docs/logs/campus_layers_dry_s66_all_actors.csv(place_campus_layers_s66.py dry-run, v2 978 배치물. CARLA 좌표)
- 건물은 표현 나(정육면체 GroundCube, 한 변 = scale m, place_campus_layers.py CUBE side 1.0). 층으로 쌓인 같은 바닥은 하나로 묶음
- 지도 좌표(ROS) = CARLA (x, -y), yaw = -yaw. 사각형은 중심·yaw·한 변으로 남기고, 여유(pad)는 지움 노드가 변마다 바깥으로 더함
사용: python3 s69_building_polys.py [--actors csv] [--out perception/config/s69_building_rects.json]
"""
import argparse, collections, csv, hashlib, json, math
from pathlib import Path

C = Path.home() / "campus_mobility_sim"
ap = argparse.ArgumentParser()
ap.add_argument("--actors", default=str(C / "map/docs/logs/campus_layers_dry_s66_all_actors.csv"))
ap.add_argument("--out", default=str(C / "perception/config/s69_building_rects.json"))
a = ap.parse_args()

seen = {}; cnt = collections.Counter()
for r in csv.DictReader(open(a.actors)):
    if r["layer"] not in ("buildings", "buildings2") or r["kind"] != "cube": continue
    cnt[r["layer"]] += 1
    key = (r["layer"], round(float(r["x"]), 3), round(float(r["y"]), 3), round(float(r["yaw"]), 2), float(r["scale"]))
    seen.setdefault(key, r["owner"])
rects = [{"layer": k[0], "owner": o, "cx": k[1], "cy": -k[2], "yaw_deg": round(-k[3], 2), "side": k[4]} for k, o in seen.items()]
src = hashlib.sha256(open(a.actors, "rb").read()).hexdigest()[:16]
json.dump({"source": str(Path(a.actors).relative_to(C)), "source_sha256_16": src, "frame": "map", "note": "정육면체 바닥 정사각형(중심·yaw·한 변). 여유는 노드 --bpad",
           "rects": rects}, open(a.out, "w"), ensure_ascii=False, indent=0)
print(json.dumps({"cubes": dict(cnt), "rects": len(rects), "by_layer": dict(collections.Counter(x["layer"] for x in rects)),
                  "side_min": min(x["side"] for x in rects), "side_max": max(x["side"] for x in rects), "src": src}, ensure_ascii=False))
