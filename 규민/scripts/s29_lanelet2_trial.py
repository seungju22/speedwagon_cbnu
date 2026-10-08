#!/usr/bin/env python3
# 맵세션29: CommonRoad Scenario Designer(crdesigner 0.8.5)로 xodr -> Lanelet2 변환 시험 (서버 없음, 재변환 없음).
# 반드시 시험 전용 venv 로 실행: ~/lanelet2_trial_venv/bin/python (프로젝트 .venv-carla 아님)
# 입력은 frozen_v1 의 복사본(작업 디렉터리 안). 원본은 해시만 읽는다
# 흐름: xodr --(odr2cr: parse_opendrive -> Network -> CommonRoad Scenario)--> --(CR2LaneletConverter)--> Lanelet2 OSM
#   공개 함수 opendrive_to_lanelet() 과 같은 두 단계를 나눠 불러 단계별 시간·경고 수를 잰다
#   CLI(crdesigner 명령)는 typer 0.9.4 + click 8.5 조합에서 시작 시 TypeError 로 죽는다(세션29 실측). 그래서 파이썬 API 를 쓴다
# 사용: python s29_lanelet2_trial.py <작업 디렉터리> <입력 xodr 복사본> <꼬리> [--autoware] [--local]
import argparse
import hashlib
import logging
import resource
import sys
import time
import warnings
from pathlib import Path

from lxml import etree

FROZEN = Path("/home/gyumin/campus_mobility_sim/map/maps/cbnu_internal_only_localtm_tags73_smooth.xodr")


class Count(logging.Handler):
    def __init__(self):
        super().__init__(logging.WARNING)
        self.n = {}
        self.first = []

    def emit(self, rec):
        self.n[rec.levelname] = self.n.get(rec.levelname, 0) + 1
        if len(self.first) < 15:
            self.first.append(f"{rec.levelname} {rec.name}: {rec.getMessage()[:160]}")


def sha16(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()[:16]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("work")
    ap.add_argument("xodr")
    ap.add_argument("tag")
    ap.add_argument("--autoware", action="store_true")
    ap.add_argument("--local", action="store_true")
    a = ap.parse_args()
    work, xodr = Path(a.work), Path(a.xodr)
    if xodr.resolve() == FROZEN.resolve():
        sys.exit("원본을 직접 입력으로 쓰지 않는다. 복사본을 주라")
    print(f"frozen_v1 sha {sha16(FROZEN)} / 입력 {xodr.name} sha {sha16(xodr)} {xodr.stat().st_size}B")
    h = Count()
    logging.getLogger().addHandler(h)
    caught = []
    warnings.simplefilter("always")
    warnings.showwarning = lambda m, c, f, l, *r: caught.append(f"{c.__name__}: {str(m)[:160]} ({Path(f).name}:{l})")

    from crdesigner.common.config.general_config import general_config
    from crdesigner.common.config.lanelet2_config import lanelet2_config
    from crdesigner.common.config.opendrive_config import open_drive_config
    from crdesigner.map_conversion.map_conversion_interface import opendrive_to_commonroad
    from crdesigner.map_conversion.lanelet2.cr2lanelet import CR2LaneletConverter
    from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
    from commonroad.planning.planning_problem import PlanningProblemSet

    lanelet2_config.autoware = a.autoware
    lanelet2_config.use_local_coordinates = a.local
    print(f"설정 autoware={lanelet2_config.autoware} use_local_coordinates={lanelet2_config.use_local_coordinates}"
          f" proj_string_cr={general_config.proj_string_cr}")
    t0 = time.time()
    try:
        sc = opendrive_to_commonroad(xodr, general_conf=general_config, odr_conf=open_drive_config)
    except Exception as e:  # noqa: BLE001
        print(f"[1단계 odr->CR] 실패 {time.time() - t0:.1f}s: {type(e).__name__}: {e}")
        raise
    t1 = time.time()
    ln = sc.lanelet_network
    print(f"[1단계 odr->CR] {t1 - t0:.1f}s, lanelet {len(ln.lanelets)}, intersection {len(ln.intersections)},"
          f" traffic_sign {len(ln.traffic_signs)}, traffic_light {len(ln.traffic_lights)}")
    crf = work / f"s29_{a.tag}.cr.xml"
    CommonRoadFileWriter(sc, PlanningProblemSet(), author="s29", affiliation="", source="frozen_v1 copy",
                         tags=set()).write_to_file(str(crf), OverwriteExistingFile.ALWAYS)
    try:
        osm = CR2LaneletConverter(config=lanelet2_config)(sc)
    except Exception as e:  # noqa: BLE001
        print(f"[2단계 CR->Lanelet2] 실패 {time.time() - t1:.1f}s: {type(e).__name__}: {e}")
        raise
    t2 = time.time()
    out = work / f"s29_{a.tag}.osm"
    out.write_bytes(etree.tostring(osm, xml_declaration=True, encoding="UTF-8", pretty_print=True))
    print(f"[2단계 CR->Lanelet2] {t2 - t1:.1f}s")
    print(f"합계 {t2 - t0:.1f}s, 출력 {out.name} {out.stat().st_size}B, CommonRoad 중간 {crf.stat().st_size}B")
    print(f"최대 RSS {resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024:.0f}MiB")
    print(f"logging 경고·오류 건수 {h.n or 0}")
    for s in h.first:
        print(f"  {s}")
    print(f"python warnings {len(caught)}")
    for s in caught[:10]:
        print(f"  {s}")
    print(f"frozen_v1 sha {sha16(FROZEN)}")


if __name__ == "__main__":
    main()
