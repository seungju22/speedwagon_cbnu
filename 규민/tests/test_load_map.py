#!/usr/bin/env python3
# cbnu_internal_only_localtm.xodr(재변환본)을 CARLA 서버에 로드하는
# 테스트. 액터 생성 없음, 맵 이름/spawn point 개수만 확인.
# 서버는 별도 터미널에서 이미 실행 중이어야 함
# (~/carla/CARLA_0.9.15/CarlaUE4.sh -quality-level=Low -windowed
# -ResX=800 -ResY=600).
import carla
from pathlib import Path

XODR_PATH = Path(__file__).resolve().parent.parent / "maps" / "cbnu_internal_only_localtm.xodr"
HOST = "localhost"
PORT = 2000
TIMEOUT = 20.0


def main():
    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)
    print(f"서버 버전: {client.get_server_version()}")

    xodr_data = XODR_PATH.read_text()
    print(f"xodr 파일 크기: {len(xodr_data)} bytes ({XODR_PATH.name})")

    params = carla.OpendriveGenerationParameters()  # 전부 기본값
    world = client.generate_opendrive_world(xodr_data, params, reset_settings=True)

    carla_map = world.get_map()
    print(f"맵 이름: {carla_map.name}")
    spawn_points = carla_map.get_spawn_points()
    print(f"spawn point 개수: {len(spawn_points)}")


if __name__ == "__main__":
    main()
