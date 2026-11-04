#!/usr/bin/env python3
# 충북대 캠퍼스 OSM relation 6705106(Nominatim jsonv2 조회,
# 2026-09-15)의 boundingbox에 300m 여유를 더한 사각형을 계산한다.
# 위도/경도 1도의 실거리가 다르므로 각각 따로 환산한다.
import math

# Nominatim search API 조회 결과 (읽기전용, 저장 좌표)
# https://nominatim.openstreetmap.org/search?q=충북대학교&format=jsonv2
# osm_type=relation, osm_id=6705106, category=amenity, type=university
SOUTH = 36.6225972
NORTH = 36.6337319
WEST = 127.4499818
EAST = 127.4634806

BUFFER_M = 300
METERS_PER_DEG_LAT = 111320  # WGS84 근사, 위도 방향은 거의 일정


def compute_buffered_bbox(south, north, west, east, buffer_m):
    lat_center = (south + north) / 2
    lat_rad = math.radians(lat_center)

    d_lat = buffer_m / METERS_PER_DEG_LAT
    d_lon = buffer_m / (METERS_PER_DEG_LAT * math.cos(lat_rad))

    return {
        "south": south - d_lat,
        "north": north + d_lat,
        "west": west - d_lon,
        "east": east + d_lon,
    }


if __name__ == "__main__":
    bbox = compute_buffered_bbox(SOUTH, NORTH, WEST, EAST, BUFFER_M)

    lat_center = (SOUTH + NORTH) / 2
    lat_rad = math.radians(lat_center)
    width_m = (bbox["east"] - bbox["west"]) * METERS_PER_DEG_LAT * math.cos(lat_rad)
    height_m = (bbox["north"] - bbox["south"]) * METERS_PER_DEG_LAT

    print(f"원본 SW: {SOUTH:.6f}, {WEST:.6f}")
    print(f"원본 NE: {NORTH:.6f}, {EAST:.6f}")
    print(f"여유적용 SW: {bbox['south']:.6f}, {bbox['west']:.6f}")
    print(f"여유적용 NE: {bbox['north']:.6f}, {bbox['east']:.6f}")
    print(f"가로(동서) 대략: {width_m:.0f} m")
    print(f"세로(남북) 대략: {height_m:.0f} m")
