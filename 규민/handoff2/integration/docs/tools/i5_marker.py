"""I5 3단계: 브리지가 월드를 다시 불러오는지 확인. place = world.id 기록 + 경로에 없는 도로 위에 표지 소품 1개, check = 같은지 비교
사용: python i5_marker.py place|check <state.json> [road_id]
"""
import json, sys, carla
c = carla.Client("localhost", 2000); c.set_timeout(30)
w = c.get_world(); cmd, st = sys.argv[1], sys.argv[2]
if cmd == "place":
    road = int(sys.argv[3]) if len(sys.argv) > 3 else 1366
    wp = w.get_map().get_waypoint_xodr(road, -1, 20.0)
    tf = wp.transform; tf.location.z += 0.1
    a = w.spawn_actor(w.get_blueprint_library().find("static.prop.trafficcone01"), tf)
    w.wait_for_tick(10)
    d = dict(world_id=w.id, map=w.get_map().name, prop_id=a.id, road=road, x=tf.location.x, y=tf.location.y)
    json.dump(d, open(st, "w")); print("place", d)
else:
    d = json.load(open(st)); a = w.get_actor(d["prop_id"])
    loc = a.get_location() if a else None
    ok = w.id == d["world_id"] and a is not None
    print("check world_id", w.id, "기록", d["world_id"], "map", w.get_map().name, "prop", None if loc is None else (round(loc.x, 2), round(loc.y, 2)), "통과", ok)
