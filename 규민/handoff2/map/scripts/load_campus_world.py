#!/usr/bin/env python3
# 맵세션40: 충북대 캠퍼스 월드 로더(인계 1). 서버가 켜져 있어야 한다(cd ~/carla/CARLA_0.9.15 && ./CarlaUE4.sh -quality-level=Low -win)
# 생성 인자 출처: tests/test_load_map_878.py (사용자 결정, 세션40). 그 파일이 쓰는 값 = CARLA 기본값 + --wall-height 0
#   세션38 로드 로그(map/docs/logs/s38_v2_back_run1_load.log "적용 OpendriveGenerationParameters")와 7개 모두 같음을 확인(세션40)
#   기본값으로 쓰던 것도 여기서는 전부 값을 적는다. CARLA 버전이 바뀌어 기본값이 달라져도 이 로더 결과는 같게
# wall_height 0.0 인 이유: 이 지도는 양방향 도로를 단방향 반쪽 도로 둘로 나눈 구조라 바깥 가장자리가 중앙선이다.
#   기본 1m 벽이면 중앙선 위에 벽이 선다(map_frozen_v1.md "로드 파라미터")
# 주의: 월드를 새로 만들면 그 전에 띄운 차·소품은 사라진다
# 근거: https://carla.readthedocs.io/en/0.9.15/python_api/ (carla.Client.generate_opendrive_world, carla.OpendriveGenerationParameters)
# 사용: python scripts/load_campus_world.py [--map v3.1|v3|v2] [--xodr PATH] [--host HOST] [--port PORT]
# 세션48: 기본 동결본을 v3 로 바꿈. --map v2 로 v2 동결본, --xodr 를 주면 그 파일(--map 보다 우선)
# 세션57: 기본 동결본을 v3.1 로 바꿈. --map v3 로 v3 동결본
import argparse
import hashlib
import xml.etree.ElementTree as ET
from pathlib import Path

import carla

MAPS_DIR = Path(__file__).resolve().parent.parent / "maps"
FROZEN = {                                 # 이름: (파일, sha256 앞 16자, 기록 문서)
    "v3.1": (MAPS_DIR / "cbnu_campus_frozen_v31.xodr", "214507afc87c2523", "map_frozen_v31.md"),
    "v3": (MAPS_DIR / "cbnu_campus_frozen_v3.xodr", "a17ab012e04c664f", "map_frozen_v3.md"),
    "v2": (MAPS_DIR / "cbnu_campus_frozen_v2.xodr", "5240ca8b883e42c0", "map_frozen_v2.md"),
}
DEFAULT_MAP = "v3.1"
TIMEOUT = 30.0                             # test_load_map_878.py 와 같음
GEN_PARAMS = dict(                         # 세션38 로드 로그 적용값과 같음(세션40 대조)
    vertex_distance=2.0,
    max_road_length=50.0,
    wall_height=0.0,
    additional_width=0.6,
    smooth_junctions=True,
    enable_mesh_visibility=True,
    enable_pedestrian_navigation=True,
)


def main():
    ap = argparse.ArgumentParser(description="충북대 캠퍼스 xodr 를 CARLA 서버에 월드로 올린다")
    ap.add_argument("--map", choices=sorted(FROZEN), default=DEFAULT_MAP, help=f"동결본 이름(기본 {DEFAULT_MAP})")
    ap.add_argument("--xodr", type=Path, default=None, help="xodr 경로(주면 --map 대신 이 파일)")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    a = ap.parse_args()
    if a.xodr is None:
        a.xodr = FROZEN[a.map][0]
    if not a.xodr.exists():
        raise SystemExit(f"xodr 없음: {a.xodr}")
    raw = a.xodr.read_bytes()
    sha = hashlib.sha256(raw).hexdigest()
    print(f"map_sha256={sha[:16]} ({a.xodr.name})")
    known = [k for k, (_, h, _) in FROZEN.items() if sha.startswith(h)]
    if known:
        print(f"동결본 {known[0]} ({FROZEN[known[0]][2]})")
    else:
        print("경고: 동결본(" + ", ".join(f"{k} {h}" for k, (_, h, _) in FROZEN.items())
              + ")과 다른 파일이다. 노선·정류장 파일은 동결본 기준이다")

    params = carla.OpendriveGenerationParameters()
    for k, v in GEN_PARAMS.items():
        setattr(params, k, v)
    print("생성 인자: " + ", ".join(f"{k}={getattr(params, k)}" for k in GEN_PARAMS))

    client = carla.Client(a.host, a.port)
    client.set_timeout(TIMEOUT)
    print(f"서버 {a.host}:{a.port} 버전 {client.get_server_version()}")
    xodr = raw.decode("utf-8")
    world = client.generate_opendrive_world(xodr, params, reset_settings=True)

    cmap = world.get_map()
    srv = cmap.to_opendrive()
    srv16 = hashlib.sha256(srv.encode("utf-8")).hexdigest()[:16]
    root = ET.fromstring(srv)
    print(f"맵 이름: {cmap.name}")
    print(f"서버 to_opendrive sha256={srv16} ({'파일과 일치' if srv16 == sha[:16] else '파일과 불일치'})")
    print(f"도로 {len(root.findall('road'))} / 교차로 {len(root.findall('junction'))}")
    print(f"스폰 지점 {len(cmap.get_spawn_points())}")


if __name__ == "__main__":
    main()
