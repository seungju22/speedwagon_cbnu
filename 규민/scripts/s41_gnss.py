#!/usr/bin/env python3
# 맵세션41 5: GNSS 기준 실측. 정문·후문 정류장(stops_v2.yaml) 위에 a2 를 세우고(physics 끔) GNSS 센서(노이즈 0 명시)로 위경도를 읽는다
# 비교: 센서 위경도 vs stops_v2.yaml 위경도(프로젝트 좌표 사슬) / 센서 vs map.transform_to_geolocation(센서 위치) / 세션40 C(offset 718.07m)
# 주행 없음. 끝나면 띄운 액터 전부 지우고 동기 설정 복원
# 사용: .venv-carla/bin/python map/scripts/s41_gnss.py
import math
import queue
import re
from pathlib import Path

import carla

BASE = Path(__file__).resolve().parent.parent
STOPS = BASE / "data/stops_v2.yaml"
NOISE0 = ["noise_alt_bias", "noise_alt_stddev", "noise_lat_bias", "noise_lat_stddev", "noise_lon_bias", "noise_lon_stddev"]


def dist(a, b):
    return math.hypot((a[0] - b[0]) * 111320, (a[1] - b[1]) * 111320 * math.cos(math.radians(a[0])))


def stops():
    y = STOPS.read_text().split("depot_candidates")[0]
    out = {}
    for m in re.finditer(r"id: (\w+).*?road_id: (\d+)\n    s: ([\d.]+)\n    lane_id: (-?\d+).*?lat: ([\d.]+)\n    lon: ([\d.]+)", y, re.S):
        out[m.group(1)] = (int(m.group(2)), float(m.group(3)), int(m.group(4)), float(m.group(5)), float(m.group(6)))
    return out


def main():
    st = stops()
    c = carla.Client("localhost", 2000)
    c.set_timeout(30.0)
    w = c.get_world()
    cmap = w.get_map()
    orig = w.get_settings()
    lib = w.get_blueprint_library()
    gbp = lib.find("sensor.other.gnss")
    for k in NOISE0:
        gbp.set_attribute(k, "0.0")
    gbp.set_attribute("noise_seed", "0")
    print("GNSS 속성: " + ", ".join(f"{k}={gbp.get_attribute(k).as_float()}" for k in NOISE0)
          + f", noise_seed={gbp.get_attribute('noise_seed').as_int()}")
    actors = []
    try:
        s = w.get_settings()
        s.synchronous_mode = True
        s.fixed_delta_seconds = 0.05
        w.apply_settings(s)
        w.tick()
        for sid in ("main_gate", "back_gate"):
            rid, ss, lane, lat, lon = st[sid]
            wp = cmap.get_waypoint_xodr(rid, lane, ss)
            tf = carla.Transform(wp.transform.location + carla.Location(z=0.5), wp.transform.rotation)
            v = w.spawn_actor(lib.find("vehicle.audi.a2"), tf)
            v.set_simulate_physics(False)
            g = w.spawn_actor(gbp, carla.Transform(), attach_to=v)
            actors += [g, v]
            q = queue.Queue()
            g.listen(q.put)
            for _ in range(3):
                w.tick()
                m = q.get(timeout=10)
            g.stop()
            loc = m.transform.location
            api = cmap.transform_to_geolocation(loc)
            sens = (m.latitude, m.longitude)
            print(f"{sid}: r{rid} s={ss} lane {lane}, 센서 위치 CARLA ({loc.x:.3f},{loc.y:.3f},{loc.z:.3f}), waypoint 와 수평 차 "
                  f"{math.hypot(loc.x - wp.transform.location.x, loc.y - wp.transform.location.y):.3f}m")
            print(f"  센서 위경도 ({m.latitude:.7f}, {m.longitude:.7f}) 고도 {m.altitude:.3f}")
            print(f"  stops_v2.yaml ({lat:.7f}, {lon:.7f}) -> 센서와 {dist(sens, (lat, lon)):.2f}m")
            print(f"  transform_to_geolocation ({api.latitude:.7f}, {api.longitude:.7f}) -> 센서와 {dist(sens, (api.latitude, api.longitude)):.3f}m")
    finally:
        for a in actors:
            a.destroy()
        s = w.get_settings()
        s.synchronous_mode = False
        s.fixed_delta_seconds = None
        w.apply_settings(s)
        left = [(a.id, a.type_id) for a in w.get_actors() if a.type_id.startswith(("vehicle.", "sensor.", "static.prop."))]
        print(f"정리: 남은 차량·센서·소품 {left}, 설정 synchronous_mode={w.get_settings().synchronous_mode} (원래 {orig.synchronous_mode})")


if __name__ == "__main__":
    main()
