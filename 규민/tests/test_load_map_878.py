#!/usr/bin/env python3
# 맵세션5 Phase C-2. cbnu_internal_only_localtm_tags73.xodr(878
# road/95 junction판)을 CARLA 서버에 로드하는 테스트. test_load_map.py
# (세션3, 134road판)의 사본 — XODR_PATH만 교체, 나머지 로직 동일.
# 액터 생성 없음, 맵 이름/spawn point 개수만 확인.
# 서버는 별도 터미널에서 이미 실행 중이어야 함
# (~/carla/CARLA_0.9.15/CarlaUE4.sh -quality-level=Low -windowed
# -ResX=800 -ResY=600).
#
# 맵세션11: 로드할 xodr 를 인자로 선택. 기본값은 평활화판, --control 은 옛(평활화 전) xodr,
# --xodr PATH 는 임의 파일. 대조 실험용. 인자 없이 실행하면 평활화판을 올린다.
#
# 맵세션12: OpendriveGenerationParameters 7개 항목을 전부 인자로 받는다.
# 지정하지 않은 항목은 CARLA 기본값(생성자 기본값) 그대로 두고, 실제로 적용된 7개 값을
# 로드 전에 전부 출력한다(실행별 설정 추적용).
# CARLA 는 가장 바깥 차선 가장자리에 1m 벽을 세운다(안전 장치, 메시 밖 낙하 방지).
# 이 지도는 양방향 도로를 단방향 half-road 둘로 쪼갠 구조라 바깥 가장자리가
# 중앙선이 된다. 결과적으로 중앙선에 벽이 서고 차량이 걸린다(세션12 확인).
# 벽은 xodr 에 없고 서버 로드 시 OpendriveGenerationParameters 로 생성되므로,
# 같은 xodr 로 벽 유무만 바꿔 대조할 수 있다. (--wall-height 0)
# 근거: https://carla.readthedocs.io/en/0.9.15/python_api/ (carla.OpendriveGenerationParameters)
#       https://github.com/carla-simulator/carla/blob/0.9.15/LibCarla/source/carla/road/MeshFactory.cpp
import argparse
import hashlib
import carla
from pathlib import Path

MAPS_DIR = Path(__file__).resolve().parent.parent / "maps"
SMOOTH_XODR = MAPS_DIR / "cbnu_internal_only_localtm_tags73_smooth.xodr"
CONTROL_XODR = MAPS_DIR / "cbnu_internal_only_localtm_tags73.xodr"
HOST = "localhost"
PORT = 2000
TIMEOUT = 30.0

# (인자 이름, 속성 이름, 타입). 순서는 CARLA 생성자 인자 순서.
GEN_PARAMS = [
    ("--vertex-distance", "vertex_distance", float),
    ("--max-road-length", "max_road_length", float),
    ("--wall-height", "wall_height", float),
    ("--additional-width", "additional_width", float),
    ("--smooth-junctions", "smooth_junctions", "bool"),
    ("--enable-mesh-visibility", "enable_mesh_visibility", "bool"),
    ("--enable-pedestrian-navigation", "enable_pedestrian_navigation", "bool"),
]


def str2bool(v):
    if v.lower() in ("true", "1", "yes"):
        return True
    if v.lower() in ("false", "0", "no"):
        return False
    raise argparse.ArgumentTypeError(f"true/false 로 지정: {v}")


def main():
    ap = argparse.ArgumentParser(description="xodr 를 CARLA 서버에 로드(기본: 평활화판)")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--control", action="store_true", help="옛(평활화 전) xodr 로드")
    g.add_argument("--xodr", type=Path, help="임의 xodr 경로")
    defaults = carla.OpendriveGenerationParameters()
    for flag, attr, typ in GEN_PARAMS:
        ap.add_argument(flag, dest=attr, default=None,
                        type=str2bool if typ == "bool" else typ,
                        help=f"OpendriveGenerationParameters.{attr} "
                             f"(CARLA 기본 {getattr(defaults, attr)})")
    args = ap.parse_args()
    XODR_PATH = args.xodr if args.xodr else (CONTROL_XODR if args.control else SMOOTH_XODR)
    if not XODR_PATH.exists():
        raise SystemExit(f"xodr 없음: {XODR_PATH}")
    # 세션19: 두 맵 병행 대비. 로그 첫 줄에 로드할 맵 sha256 앞 16자
    print(f"map_sha256={hashlib.sha256(XODR_PATH.read_bytes()).hexdigest()[:16]} ({XODR_PATH.name})")
    print(f"로드 대상: {XODR_PATH}")
    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)
    print(f"서버 버전: {client.get_server_version()}")

    xodr_data = XODR_PATH.read_text()
    print(f"xodr 파일 크기: {len(xodr_data)} bytes ({XODR_PATH.name})")

    params = carla.OpendriveGenerationParameters()  # CARLA 기본값에서 시작
    for _, attr, _ in GEN_PARAMS:
        v = getattr(args, attr)
        if v is not None:
            setattr(params, attr, v)
    print("적용 OpendriveGenerationParameters (* = 인자로 변경):")
    for _, attr, _ in GEN_PARAMS:
        mark = "*" if getattr(args, attr) is not None else " "
        print(f"{mark} {attr}={getattr(params, attr)}")
    world = client.generate_opendrive_world(xodr_data, params, reset_settings=True)

    carla_map = world.get_map()
    print(f"맵 이름: {carla_map.name}")
    spawn_points = carla_map.get_spawn_points()
    print(f"spawn point 개수: {len(spawn_points)}")
    # 세션20: 서버가 실제로 가진 맵과 보낸 파일 대조. 맵 이름은 xodr 에 상관없이 같으므로 식별자가 못 된다.
    # 서버가 돌려주는 OpenDRIVE 원문(to_opendrive)의 해시를 보낸 문자열 해시와 비교하고, 다르면 road/junction 수로 대조
    srv = carla_map.to_opendrive()
    sent16 = hashlib.sha256(xodr_data.encode("utf-8")).hexdigest()[:16]
    srv16 = hashlib.sha256(srv.encode("utf-8")).hexdigest()[:16]
    print(f"보낸 문자열 sha256={sent16}, 서버 to_opendrive sha256={srv16} "
          f"({'일치' if sent16 == srv16 else '불일치'}), 길이 {len(xodr_data)} / {len(srv)}")
    import xml.etree.ElementTree as ET
    r = ET.fromstring(srv)
    print(f"서버 맵 road {len(r.findall('road'))} / junction {len(r.findall('junction'))}")


if __name__ == "__main__":
    main()
