"""추적 카메라(세션58): 자차(role ego_vehicle) 뒤에 RGB 카메라를 달아 주행 중에 바로 jpg 로 남긴다.
CARLA 녹화 재생이 아니라 주행 회차 안에서 찍는다(레이어 732 녹화는 재생 때 자차가 안 생겼다, S4).
- 카메라: 자차 원점 기준 뒤 --dist, 위 --height, 아래로 --pitch, Rigid 부착. sensor_tick = 1/--fps(시뮬레이터 시간)
- 끝: 자차가 없어지거나 SIGINT/SIGTERM. 그다음 글자 넣기 integration/docs/tools/s4_label.py, 인코딩 i2_encode.py(컨테이너)
사용: python chase_cam.py <frame_dir> [--w 640] [--h 360] [--fps 10] [--dist 7] [--height 3.2] [--pitch -14] [--fov 90]
"""
import argparse, os, signal, time
import carla

ap = argparse.ArgumentParser(); ap.add_argument("out")
ap.add_argument("--w", type=int, default=640); ap.add_argument("--h", type=int, default=360); ap.add_argument("--fps", type=float, default=10.0)
ap.add_argument("--dist", type=float, default=7.0); ap.add_argument("--height", type=float, default=3.2)
ap.add_argument("--pitch", type=float, default=-14.0); ap.add_argument("--fov", type=float, default=90.0)
ap.add_argument("--wait", type=float, default=600.0, help="자차 기다리는 실제 시간 초")
a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)
c = carla.Client("localhost", 2000); c.set_timeout(20); w = c.get_world()
stop = {"v": False}
signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("v", True)); signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("v", True))
t0 = time.time(); ego = None
while ego is None and not stop["v"] and time.time() - t0 < a.wait:
    ego = next((v for v in w.get_actors().filter("vehicle.*") if v.attributes.get("role_name") == "ego_vehicle"), None)
    if ego is None: time.sleep(1)
if ego is None: raise SystemExit("자차 없음")
b = w.get_blueprint_library().find("sensor.camera.rgb")
for k, v in (("image_size_x", a.w), ("image_size_y", a.h), ("fov", a.fov), ("sensor_tick", 1.0 / a.fps)): b.set_attribute(k, str(v))
cam = w.spawn_actor(b, carla.Transform(carla.Location(x=-a.dist, z=a.height), carla.Rotation(pitch=a.pitch)), attach_to=ego,
                    attachment_type=carla.AttachmentType.Rigid)
n = {"k": 0, "first": None, "last": None}


def on_img(img):
    img.save_to_disk(os.path.join(a.out, "%06d.jpg" % img.frame))
    n["k"] += 1; n["last"] = img.timestamp
    if n["first"] is None: n["first"] = img.timestamp


cam.listen(on_img)
print("cam %d on ego %d %dx%d fps %.1f" % (cam.id, ego.id, a.w, a.h, a.fps), flush=True)
try:
    while not stop["v"]:
        time.sleep(1.0)
        if not any(x.id == ego.id for x in w.get_actors().filter("vehicle.*")): break
finally:
    try: cam.stop(); cam.destroy()
    except Exception as e: print("cam destroy", e)
    print("frames %d sim %.2f~%.2f" % (n["k"], n["first"] or 0, n["last"] or 0), flush=True)
