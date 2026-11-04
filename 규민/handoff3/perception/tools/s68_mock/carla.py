"""10/23 오프라인 시험용 가짜 carla 모듈(gt_objects2.py 가 쓰는 것만). CARLA 서버 없이 '소환 틱 -> 처음 낸 틱'과 틱당 처리 시간을 본다
- 액터: 정적 소품 MOCK_STATIC 개(레이어 v2 978 과 비슷하게, role_name 없음) + 자차(role ego_vehicle) + 일정에 따라 보행자 소환
- 틱: 실제 시간 0.05s 마다 frame 하나(동기 CARLA 0.998배 근사). get_actors 는 RPC 를 흉내 내 MOCK_RPC_MS 만큼 쉼(목록 전체는 액터 수에 비례)
- 소환 일정 MOCK_SPAWN="frame,frame,..."(시작 frame 기준 상대). 소환 기록 MOCK_SPAWN_LOG 에 'spawn <id> <frame>'
"""
import math, os, threading, time

N_STATIC = int(os.environ.get("MOCK_STATIC", "980")); RPC_MS = float(os.environ.get("MOCK_RPC_MS", "2"))
SPAWN = [int(x) for x in os.environ.get("MOCK_SPAWN", "").split(",") if x]; SLOG = os.environ.get("MOCK_SPAWN_LOG", "")


class Vector3D:
    def __init__(s, x=0.0, y=0.0, z=0.0): s.x, s.y, s.z = x, y, z


class Location(Vector3D):
    def distance(s, o): return math.sqrt((s.x - o.x) ** 2 + (s.y - o.y) ** 2 + (s.z - o.z) ** 2)


class Rotation:
    def __init__(s, pitch=0.0, yaw=0.0, roll=0.0): s.pitch, s.yaw, s.roll = pitch, yaw, roll


class Transform:
    def __init__(s, location=None, rotation=None): s.location = location or Location(); s.rotation = rotation or Rotation()

    def transform(s, p):
        c, si = math.cos(math.radians(s.rotation.yaw)), math.sin(math.radians(s.rotation.yaw))
        return Location(s.location.x + p.x * c - p.y * si, s.location.y + p.x * si + p.y * c, s.location.z + p.z)


class BoundingBox:
    def __init__(s, loc, ext): s.location, s.extent = loc, ext


class _Labels:
    def __getattr__(s, k): return k


CityObjectLabel = _Labels()


class Actor:
    def __init__(s, aid, tid, attrs, ext, loc):
        s.id, s.type_id, s.attributes = aid, tid, attrs; s.bounding_box = BoundingBox(Location(0, 0, ext.z), ext); s.loc = loc


class ActorSnapshot:
    def __init__(s, a): s.id = a.id; s._a = a

    def get_transform(s): return Transform(s._a.loc, Rotation())

    def get_velocity(s): return Vector3D()

    def get_angular_velocity(s): return Vector3D()


class _TS:
    def __init__(s, e): s.elapsed_seconds = e


class WorldSnapshot:
    def __init__(s, frame, el, actors):
        s.frame = frame; s.timestamp = _TS(el); s._m = {a.id: ActorSnapshot(a) for a in actors}

    def __iter__(s): return iter(s._m.values())

    def find(s, aid): return s._m.get(aid)


class World:
    def __init__(s):
        s.lock = threading.Lock(); s.actors = {}; s.cbs = {}; s.frame = 1000; s.el = 0.0; s.nid = 100; s.snap = None
        for i in range(N_STATIC):
            s._new("static.prop.mesh", {}, Vector3D(2, 2, 2), Location(i * 3.0, 50.0, 0))
        s._new("vehicle.toyota.prius", {"role_name": "ego_vehicle", "base_type": "car"}, Vector3D(2.4, 1.0, 0.8), Location(0, 0, 0))
        s.f0 = s.frame; s.slog = open(SLOG, "w") if SLOG else None
        threading.Thread(target=s._ticker, daemon=True).start()

    def _new(s, tid, attrs, ext, loc):
        s.nid += 1; a = Actor(s.nid, tid, attrs, ext, loc)
        with s.lock: s.actors[a.id] = a
        return a

    def _ticker(s):
        nxt = time.time()
        while True:
            nxt += 0.05; time.sleep(max(0.0, nxt - time.time()))
            s.frame += 1; s.el += 0.05
            for k, rel in enumerate(SPAWN):
                if s.frame - s.f0 == rel:
                    a = s._new("walker.pedestrian.0001", {}, Vector3D(0.2, 0.2, 0.9), Location(5 + k, 2, 0))
                    if s.slog: s.slog.write("spawn %d %d\n" % (a.id, s.frame)); s.slog.flush()
            with s.lock: acts = list(s.actors.values())
            s.snap = WorldSnapshot(s.frame, s.el, acts)
            for cb in list(s.cbs.values()): cb(s.snap)

    def get_actors(s, ids=None):
        with s.lock: acts = list(s.actors.values()) if ids is None else [s.actors[i] for i in ids if i in s.actors]
        time.sleep(RPC_MS / 1000 * (1 + len(acts) / 100.0))   # 목록 크기에 비례한 RPC 흉내
        return acts

    def get_snapshot(s):
        while s.snap is None: time.sleep(0.01)
        return s.snap

    def wait_for_tick(s, t=10):
        f = s.frame
        while s.frame == f: time.sleep(0.005)
        return s.snap

    def on_tick(s, cb):
        k = len(s.cbs) + 1; s.cbs[k] = cb; return k

    def remove_on_tick(s, k): s.cbs.pop(k, None)

    def cast_ray(s, a, b): return []


class Client:
    def __init__(s, host="localhost", port=2000): pass

    def set_timeout(s, t): pass

    def get_world(s): return World()
