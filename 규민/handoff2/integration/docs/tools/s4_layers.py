"""S4: 장식 레이어 소환물이 브리지 기동 뒤에도 그대로인지. place_campus_layers.py 상태 파일(actors id·type)을 읽는다
mark  = 지금 world.id 와 레이어별 살아 있는 수를 <out.json> 에 기록
check = 다시 세서 mark 와 비교(world.id 같음, 레이어별 수 같음이면 통과)
사용: python s4_layers.py mark|check <state.json> <out.json>
"""
import collections, json, sys, carla
cmd, st, out = sys.argv[1], sys.argv[2], sys.argv[3]
c = carla.Client("localhost", 2000); c.set_timeout(30); w = c.get_world()
w.wait_for_tick(10)   # 새 클라이언트는 tick 을 받아야 액터 목록이 찬다(동기면 브리지 tick 을 기다림)
S = json.load(open(st)); alive = collections.Counter(); gone = 0
ids = {a.id: a.type_id for a in w.get_actors()}
for r in S["actors"]:
    if ids.get(r["id"]) == r["type_id"]: alive[r["layer"]] += 1
    else: gone += 1
d = dict(world_id=w.id, map=w.get_map().name, alive=dict(alive), total=sum(alive.values()), state_total=len(S["actors"]), gone=gone,
         all_actors=len(ids))
if cmd == "mark":
    json.dump(d, open(out, "w"), ensure_ascii=False); print("mark", d)
else:
    m = json.load(open(out)); ok = d["world_id"] == m["world_id"] and d["alive"] == m["alive"]
    print("check", d, "mark", m["world_id"], m["alive"], "통과", ok)
