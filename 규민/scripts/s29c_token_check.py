#!/usr/bin/env python3
# 맵세션29c: 문서 병합 검증. 숫자·좌표·ID 토큰을 뽑아 "원본에 있던 값이 병합본에 전부 있는가" 를 본다 (읽기전용)
# 토큰: 숫자를 포함한 연속 문자열(영문·숫자·. , - _). 예 36.627730, 4,704m2, way293211019, E7-3, G5, r1333, 2026-10-06
# 끝의 마침표·쉼표는 문장부호로 보고 뗀다. 비교는 집합(있는가 없는가). 횟수 차이는 참고로만 출력
# 사용: python s29c_token_check.py <병합본> <원본1> [<원본2> ...]
import collections
import re
import sys

TOK = re.compile(r"[A-Za-z_]*\d[0-9A-Za-z_.,\-~]*")      # ASCII 만. "3곳이다" -> "3" (한글 조사·단위는 떼고 본다)
# 문서 구조 번호(절 번호)는 값이 아니다. 정리본은 절 번호를 없앴으므로 따로 빼서 보여 준다
STRUCT = re.compile(r"^#+\s*\d+\.|\d+(?:~\d+)?절", re.M)


def toks(path):
    text = open(path, encoding="utf-8").read()
    st = collections.Counter(m.group(0) for m in STRUCT.finditer(text))
    text = STRUCT.sub(" ", text)
    out = collections.Counter()
    for m in TOK.findall(text):
        out[m.rstrip(".,-~")] += 1
    return out, st


merged, mst = toks(sys.argv[1])
bad = 0
for src in sys.argv[2:]:
    t, st = toks(src)
    print(f"{src}: (구조 번호, 비교 제외) {dict(st)}")
    miss = sorted(set(t) - set(merged))
    fewer = sorted((k, t[k], merged[k]) for k in t if k in merged and merged[k] < t[k])
    print(f"{src}: 토큰 종류 {len(t)}, 병합본에 없는 것 {len(miss)} {miss}")
    print(f"  (참고) 병합본에서 횟수가 줄어든 것 {len(fewer)} {fewer[:20]}")
    bad += len(miss)
print(f"결과: 빠진 토큰 {bad}")
sys.exit(1 if bad else 0)
