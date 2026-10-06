#!/usr/bin/env python3
# cbnu_campus_fixed.osm을 CARLA Osm2Odr로 변환해 cbnu.xodr을
# 만든다. CARLA 서버는 필요 없음(순수 라이브러리 변환).
# 설정값 근거는 map/docs/osm_survey.md, map/docs/odd.md 참고.
import contextlib
import datetime
import os
import tempfile
from pathlib import Path

import carla


@contextlib.contextmanager
def capture_stderr(result_box):
    # carla.Osm2Odr.convert()는 파이썬이 아닌 C++ 확장이라
    # 경고를 fd 2(OS 레벨 stderr)에 직접 쓴다. sys.stderr 리다이렉트로는
    # 안 잡히므로 fd 자체를 임시파일로 바꿔치기한 뒤 복원한다.
    # result_box: 캡처한 텍스트를 담을 1칸짜리 list(호출부에서 읽어감)
    stderr_fd = 2
    saved_fd = os.dup(stderr_fd)
    with tempfile.TemporaryFile(mode="w+b") as tmp:
        os.dup2(tmp.fileno(), stderr_fd)
        try:
            yield
        finally:
            os.dup2(saved_fd, stderr_fd)
            os.close(saved_fd)
            tmp.seek(0)
            result_box.append(tmp.read().decode(errors="replace"))

BASE_DIR = Path(__file__).resolve().parent.parent
# 맵세션4 Phase C-1/C-4(2026-09-18): 태그보정 73건판 파이프라인.
# 기존 cbnu_internal_only_localtm.xodr(19건판, road134/junction25)은
# 대조군으로 보존 — 덮어쓰지 않고 새 파일명으로 출력(C-4 지시).
INPUT_PATH = BASE_DIR / "data" / "processed" / "cbnu_internal_boundary_tags73.osm"
OUTPUT_PATH = BASE_DIR / "maps" / "cbnu_internal_only_localtm_tags73.xodr"
LOG_DIR = BASE_DIR / "logs"

# 맵세션3 Phase B에서 채택(2026-09-17). 기존 geoReference가
# "+proj=tmerc"뿐이라 PROJ 기본값(lon_0=0, 그리니치)으로 투영돼
# 한국(동경127.45도)에서 스케일 왜곡(비율 약1.30배, 실측 map_session03_report.md
# B절 참고)이 발생했던 문제의 수정. 캠퍼스 자체를 중앙자오선으로 삼아
# 왜곡을 이론상 무시가능한 수준으로 낮춤. 중심좌표=geo_calibrate.py의
# 25개 보정점 평균(ref_lat/ref_lon).
PROJ_STRING = "+proj=tmerc +lat_0=36.627298 +lon_0=127.456394 +ellps=WGS84"

# 3.35m = carla 0.9.15 런타임 실측 기본값. 공식 문서(python_api.md)는
# 4.0으로 기재돼 있으나 설치판 인스턴스 실측값은 3.35였음
# (2026-09-15 확인, 소스 코드 미확인). OSM에 차도 width 태그가
# 없어 데이터 기반 추정 불가하여 도구 기본값 채택. CARLA 로드 후
# 실제 도로 형상과 대조해 조정 대상.
DEFAULT_LANE_WIDTH = 3.35

# 교내 신호등 자동생성 끔. 근거: map/docs/odd.md — 사용자가 직접
# 확인한 실제 ODD는 교내 신호등이 아예 없고 비신호 횡단이 일반적.
# 자동생성을 켜면 실제보다 더 통제된 운행 환경을 가정하게 됨.
GENERATE_TRAFFIC_LIGHTS = False


def load_osm(input_path):
    with open(input_path, "r") as f:
        return f.read()


def build_settings():
    settings = carla.Osm2OdrSettings()
    settings.default_lane_width = DEFAULT_LANE_WIDTH
    settings.generate_traffic_lights = GENERATE_TRAFFIC_LIGHTS
    settings.proj_string = PROJ_STRING
    # use_offsets/offset_x/offset_y/elevation_layer_height/
    # center_map/all_junctions_with_traffic_lights는 설치판 실측
    # 기본값을 그대로 둠(osm_survey.md 표 참고).
    return settings


def convert(osm_data, settings):
    return carla.Osm2Odr.convert(osm_data, settings)


def write(xodr_data, output_path):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w") as f:
        f.write(xodr_data)


def write_log(settings, output_path, log_dir, stderr_text=""):
    log_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = log_dir / f"osm_to_xodr_{timestamp}.log"
    size_bytes = output_path.stat().st_size
    with open(log_path, "w") as f:
        f.write(f"실행시각: {timestamp}\n")
        f.write(f"입력: {INPUT_PATH}\n")
        f.write(f"출력: {output_path}\n")
        f.write(f"출력크기: {size_bytes} bytes\n")
        f.write("설정값:\n")
        f.write(f"  use_offsets={settings.use_offsets}\n")
        f.write(f"  offset_x={settings.offset_x}\n")
        f.write(f"  offset_y={settings.offset_y}\n")
        f.write(f"  default_lane_width={settings.default_lane_width}\n")
        f.write(f"  elevation_layer_height={settings.elevation_layer_height}\n")
        f.write(f"  center_map={settings.center_map}\n")
        f.write(f"  proj_string={settings.proj_string}\n")
        f.write(f"  generate_traffic_lights={settings.generate_traffic_lights}\n")
        f.write(
            "  all_junctions_with_traffic_lights="
            f"{settings.all_junctions_with_traffic_lights}\n"
        )
        f.write("stderr:\n")
        f.write(stderr_text if stderr_text else "(없음)\n")
    return log_path


if __name__ == "__main__":
    # 맵세션11: 평활화 단계 산출물을 변환할 수 있도록 경로 인자 추가. 기본값은 기존과 동일.
    import argparse
    _ap = argparse.ArgumentParser()
    _ap.add_argument("--input", type=Path, default=INPUT_PATH)
    _ap.add_argument("--output", type=Path, default=OUTPUT_PATH)
    _args = _ap.parse_args()
    INPUT_PATH, OUTPUT_PATH = _args.input, _args.output
    osm_data = load_osm(INPUT_PATH)
    settings = build_settings()
    stderr_box = []
    with capture_stderr(stderr_box):
        xodr_data = convert(osm_data, settings)
    write(xodr_data, OUTPUT_PATH)
    log_path = write_log(settings, OUTPUT_PATH, LOG_DIR, stderr_box[0])

    print(f"변환 완료: {OUTPUT_PATH}")
    print(f"출력크기: {OUTPUT_PATH.stat().st_size} bytes")
    print(f"로그: {log_path}")
