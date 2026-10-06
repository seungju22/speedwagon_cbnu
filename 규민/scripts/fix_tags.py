#!/usr/bin/env python3
# 맵세션4 Phase C-1(2026-09-18)에서 기존 19건 + 신규 54건 =
# 총 73건으로 대상을 갱신, highway=unclassified로 고쳐 사본을 만든다.
# 대상 id·판별기준·근거는 map/docs/osm_survey.md(기존19건),
# map/docs/road_candidates.md(신규54건) 참고.
# 원본(data/raw/cbnu_campus.osm)은 열기만 하고 절대 덮어쓰지 않는다.
import datetime
import xml.etree.ElementTree as ET
from pathlib import Path

# 세 갈래 근거:
# (a) 기존19건 중 16건 — 길이>=200m AND 양끝 다른 way 2개 이상과
#     연결(간선후보 기준) 충족, 분류=internal 또는 boundary
#     (분류=external 8건 제외). osm_survey.md 참고.
TARGET_WAY_IDS_LENGTH_BASED = [
    "481950067", "774924319", "648686770", "392632036",
    "392632035", "481943504", "452421706", "446440340",
    "442595037", "442708395", "452870648", "452870641",
    "442595034", "442595030", "473926892", "442708402",
]
# (b) 기존19건 중 3건 — 길이기준 미달(147.5m/48.7m/64.5m)로 (a)에서
#     걸러졌으나, 정문-도서관 최단경로(그래프 BFS 실측) 구성에
#     필수인 구간. 길이만으로 거르면 경로가 끊기는 사례로
#     osm_survey.md에 기록.
TARGET_WAY_IDS_ROUTE_BASED = [
    "392532207", "481950063", "481950062",
]
# (c) 신규 54건 — Phase B 1차필터(steps 제외/width<2.5m 제외/기존
#     19way와 직접 노드공유/길이>=50m) 통과 후보 64건 중 사용자가
#     채택한 54건(제외10건은 map/docs/road_candidates.md 제외사유
#     참고). 번호-way id 대응은 road_candidates.md 표와 동일
#     (map/scripts/road_candidates.py get_candidates() 산출물).
#     맵세션4 검산 기록(setup_log.md 2026-09-18): 지시서 원문의
#     "52건" 표기는 오기, 실제 54건이 맞음(사용자 확인).
TARGET_WAY_IDS_NEW_CANDIDATES = [
    "442065041", "442065042", "442065043", "442065044", "442065049",
    "442065057", "442065059", "442065060", "442065062", "442065828",
    "442065829", "442066132", "442066134", "442361807", "442361821",
    "442361822", "442361829", "442595017", "442595029", "442595031",
    "442595040", "442595041", "442595852", "442595857", "442596415",
    "442596417", "442708394", "442708397", "442708398", "442708400",
    "442708403", "442708404", "442708405", "442709216", "442709221",
    "442709224", "442709226", "442760318", "442761502", "446440351",
    "452421704", "468616604", "468616606", "468616610", "471400786",
    "471400793", "471400834", "471400900", "472252988", "472252989",
    "472252990", "473926894", "648686772", "815864978",
]
TARGET_WAY_IDS = (
    TARGET_WAY_IDS_LENGTH_BASED
    + TARGET_WAY_IDS_ROUTE_BASED
    + TARGET_WAY_IDS_NEW_CANDIDATES
)

BASE_DIR = Path(__file__).resolve().parent.parent
INPUT_PATH = BASE_DIR / "data" / "raw" / "cbnu_campus.osm"
# 맵세션4 파일명: 기존 cbnu_campus_fixed.osm(19건 버전)을 대조군으로
# 보존하기 위해 별도 이름 사용(사본을 새로 만든다, C-1 지시).
OUTPUT_PATH = BASE_DIR / "data" / "processed" / "cbnu_campus_fixed_tags73.osm"
LOG_DIR = BASE_DIR / "logs"
NEW_HIGHWAY_VALUE = "unclassified"


def load(input_path):
    tree = ET.parse(input_path)
    return tree


def apply_fixes(tree, target_ids):
    changes = []
    root = tree.getroot()
    remaining = set(target_ids)
    for way in root.findall("way"):
        wid = way.get("id")
        if wid not in remaining:
            continue
        highway_tag = None
        for tag in way.findall("tag"):
            if tag.get("k") == "highway":
                highway_tag = tag
                break
        if highway_tag is None:
            continue
        old_value = highway_tag.get("v")
        highway_tag.set("v", NEW_HIGHWAY_VALUE)
        changes.append((wid, old_value, NEW_HIGHWAY_VALUE))
        remaining.discard(wid)
    return changes, remaining


def write(tree, output_path):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_path, encoding="UTF-8", xml_declaration=True)


def write_log(changes, not_found, log_dir):
    log_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = log_dir / f"fix_tags_{timestamp}.log"
    with open(log_path, "w") as f:
        f.write(f"실행시각: {timestamp}\n")
        f.write(f"입력: {INPUT_PATH}\n")
        f.write(f"출력: {OUTPUT_PATH}\n")
        f.write(f"보정 건수: {len(changes)}\n")
        for wid, old, new in changes:
            f.write(f"way {wid}: highway={old} -> highway={new}\n")
        if not_found:
            f.write(f"찾지 못한 id({len(not_found)}건): {sorted(not_found)}\n")

    ids_path = log_dir / "fixed_way_ids.txt"
    with open(ids_path, "w") as f:
        for wid, _, _ in changes:
            f.write(f"{wid}\n")

    return log_path, ids_path


if __name__ == "__main__":
    tree = load(INPUT_PATH)
    changes, not_found = apply_fixes(tree, TARGET_WAY_IDS)
    write(tree, OUTPUT_PATH)
    log_path, ids_path = write_log(changes, not_found, LOG_DIR)

    print(f"보정 완료: {len(changes)}건")
    if not_found:
        print(f"경고: 찾지 못한 id {sorted(not_found)}")
    print(f"출력: {OUTPUT_PATH}")
    print(f"로그: {log_path}")
    print(f"대상id목록: {ids_path}")
