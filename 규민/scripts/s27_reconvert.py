#!/usr/bin/env python3
# 맵세션27 작성: 10/9 재변환 래퍼. 세션18 D 의 trial_convert_s18.py(scratchpad, 소멸)와 같은 방식을 프로젝트 안에 남긴다.
# 왜 래퍼인가: fix_tags.py / classify_internal.py 는 출력 경로가 코드에 고정돼 그대로 실행하면 tags73 OSM 을 덮어쓰고,
#   fix_tags.write_log 는 map/logs/fixed_way_ids.txt 를 덮어쓴다(s18_D_reconvert.md). 그래서 함수만 import 해 새 경로로 쓴다.
#   smooth_osm_curves.py 는 --log-prefix 를 안 주면 session11_smoothing_*.csv 를 덮어쓴다 -> 반드시 새 접두어.
# 세션28: 392632034 추가(9개), --variant A|B 추가. 세션27 check 로그(s27_reconvert_check.log)는 8개 시절 값
# 모드
#   check            읽기전용. 입력 해시, 추가 way 존재·태그, 기존 73 과 중복, 분류(메모리 안) 결과, 출력 파일 미존재 확인.
#                    세션27 에 실행한 것은 이 모드뿐
#   run --tag T      실제 재변환(새 OSM 3개 + 새 xodr 1개). --base 면 추가 way 없이 73건으로(재현성 기준선)
#                    세션27 에서는 실행 금지(지시문). 10/9 부터
# 흐름: raw OSM -> 태그 보정(73 + 추가) -> 폴리곤 분류(internal+boundary) -> 곡선 평활화(세션11 기본 인자) -> Osm2Odr
import argparse
import hashlib
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fix_tags                    # noqa: E402
import classify_internal as ci     # noqa: E402

BASE = Path(__file__).resolve().parent.parent
PY = BASE.parent / ".venv-carla/bin/python"
RAW = BASE / "data/raw/cbnu_campus.osm"
POLY = BASE / "data/processed/cbnu_relation_polygon.json"
FROZEN = BASE / "maps/cbnu_internal_only_localtm_tags73_smooth.xodr"
# 세션18 D 에서 기록한 값(s18_D_reconvert.md 안전 확인) + frozen_v1
EXPECT = {RAW: "c383870eb4d1e196", FROZEN: "bf835cdfad0cea65",
          BASE / "data/processed/cbnu_campus_fixed_tags73.osm": "dfcc5ff7b1a41532",
          BASE / "data/processed/cbnu_internal_boundary_tags73.osm": "ce457630b2de089f",
          BASE / "data/processed/cbnu_internal_boundary_tags73_smooth.osm": "caa2c69b026ccc75"}
# 세션27 확정 재변환 대상(map/docs/logs/s24_reconv_list.md "2026-10-06 갱신" 절). 바꾸면 그 절도 고친다
ADD_WAYS = ["481950060",                                   # 정문 출구
            "442595850",                                   # 박물관 서쪽 차고지 고리(A7)
            "481945510", "481943505", "481475019",          # 후문 연결 6
            "452870644", "481945509", "481943503",
            "392632034"]                                   # 세션28 추가: 후문B 게이트 진입로(s28_gate_review.md)
# 세션28 A/B 비교(reconversion_plan.md 7절): A안 = ADD_WAYS 전부, B안 = 정문 출구만 제외
VARIANT_EXCLUDE = {"A": [], "B": ["481950060"]}
EXPECT_CLASS = (447, 38, 664)   # 태그 보정은 분류를 안 바꾼다(highway 태그 있는 way 집합 불변, s18 확인)


def sha16(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()[:16]


def paths(tag):
    p = BASE / "data/processed"
    return (p / f"cbnu_campus_fixed_{tag}.osm", p / f"cbnu_internal_boundary_{tag}.osm",
            p / f"cbnu_internal_boundary_{tag}_smooth.osm",
            BASE / f"maps/cbnu_internal_only_localtm_{tag}_smooth.xodr")


def check_inputs():
    bad = 0
    for p, h in EXPECT.items():
        got = sha16(p)
        print(f"  {'일치' if got == h else '불일치'} {got} (기대 {h}) {p.relative_to(BASE)}")
        bad += got != h
    return bad


def variant_ways(variant):
    return [w for w in ADD_WAYS if w not in VARIANT_EXCLUDE[variant]]


def check(tag, variant="A"):
    add = variant_ways(variant)
    print(f"[0] 안 {variant}: 추가 way {len(add)}개 (제외 {VARIANT_EXCLUDE[variant] or '없음'})")
    print("[1] 입력 해시")
    bad = check_inputs()
    print("[2] 추가 way")
    tree = fix_tags.load(RAW)
    ways = {w.get("id"): w for w in tree.getroot().findall("way")}
    dup = set(add) & set(fix_tags.TARGET_WAY_IDS)
    print(f"  기존 보정 {len(fix_tags.TARGET_WAY_IDS)}건, 추가 {len(add)}건, 겹침 {sorted(dup) or 0}")
    bad += bool(dup)
    for wid in add:
        w = ways.get(wid)
        if w is None:
            print(f"  없음 way{wid}")
            bad += 1
            continue
        t = {x.get("k"): x.get("v") for x in w.findall("tag")}
        print(f"  way{wid} highway={t.get('highway')} service={t.get('service')} oneway={t.get('oneway', '-')}")
        bad += t.get("highway") != "service"
    print("[3] 분류(메모리 안, 파일 안 씀)")
    nodes, wl = ci.load_osm(RAW)
    cls = ci.classify(nodes, wl, ci.load_polygon_rings(POLY))
    cnt = tuple(sum(1 for v in cls.values() if v == k) for k in ("internal", "boundary", "external"))
    print(f"  internal/boundary/external {cnt} (기대 {EXPECT_CLASS})")
    bad += cnt != EXPECT_CLASS
    for wid in add:
        print(f"  way{wid}: {cls.get(wid)}")
        bad += cls.get(wid) not in ("internal", "boundary")
    print("[4] 출력 경로(없어야 함)")
    for p in paths(tag):
        print(f"  {'이미 있음' if p.exists() else '없음'} {p.relative_to(BASE)}")
        bad += p.exists()
    print(f"점검 결과: 문제 {bad}건")
    return bad


def run(tag, base, variant="A"):
    if check(tag, variant):
        sys.exit("점검 실패: 실행하지 않음")
    fixed, internal, smooth, xodr = paths(tag)
    targets = list(fix_tags.TARGET_WAY_IDS) + ([] if base else variant_ways(variant))
    tree = fix_tags.load(RAW)
    changes, nf = fix_tags.apply_fixes(tree, targets)
    if nf:
        sys.exit(f"못 찾은 id {sorted(nf)}")
    fix_tags.write(tree, fixed)            # write_log 는 부르지 않는다(fixed_way_ids.txt 덮어씀)
    print(f"[run] 태그 보정 {len(changes)}건 -> {fixed.name}")
    nodes, wl = ci.load_osm(fixed)
    cls = ci.classify(nodes, wl, ci.load_polygon_rings(POLY))
    n_w, n_n = ci.write_subset_osm(nodes, wl, cls, internal)
    print(f"[run] 분류 사본 way {n_w} node {n_n} -> {internal.name}")
    subprocess.run([str(PY), str(BASE / "scripts/smooth_osm_curves.py"), "--input", str(internal),
                    "--output", str(smooth), "--log-prefix", f"s28_smoothing_{tag}"], check=True)
    subprocess.run([str(PY), str(BASE / "scripts/osm_to_xodr.py"), "--input", str(smooth),
                    "--output", str(xodr)], check=True)
    print(f"[run] 출력 {xodr} sha {sha16(xodr)}")
    print(f"[run] frozen_v1 재확인 {sha16(FROZEN)} (기대 bf835cdfad0cea65)")
    if sha16(FROZEN) != "bf835cdfad0cea65":
        sys.exit("frozen_v1 해시 변경: 즉시 중단")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["check", "run"])
    ap.add_argument("--variant", choices=["A", "B"], default="A",
                    help="세션28 A/B 안. A = 추가 9 전부, B = 정문 출구 481950060 제외 8")
    ap.add_argument("--tag", default=None,
                    help="출력 파일명 꼬리. 기본: A -> tags82_A, B -> tags81_B (73 + 추가 수)")
    ap.add_argument("--base", action="store_true", help="추가 way 없이 73건 그대로(재현성 기준선)")
    a = ap.parse_args()
    tag = a.tag or f"tags{73 + len(variant_ways(a.variant))}_{a.variant}"
    if a.mode == "check":
        sys.exit(1 if check(tag, a.variant) else 0)
    run(tag, a.base, a.variant)
