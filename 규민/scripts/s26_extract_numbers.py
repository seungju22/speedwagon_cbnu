#!/usr/bin/env python3
# 맵세션26: 발표 자료 5개 파일에서 측정·계산·집계 숫자 추출 (읽기전용).
# 제외: 날짜, 세션 번호, road/junction/way/node ID, 좌표, 해시, 버전, 줄 번호, 그림 번호.
# 출력 CSV: file,line,value,unit,priority,context
import csv
import re
import sys
from pathlib import Path

D = Path("/home/gyumin/campus_mobility_sim/map/docs")
FILES = ["presentation_2026-10-08.md", "ppt_outline_2026-10-08.md", "stops_status_2026-09-29.md",
         "map_frozen_v1.md", "how_the_map_was_made.md"]
UNIT = r"(m/s\^2|m/s²|m/s|km/h|GiB|Gi|MiB|Mi|MB|KB|bytes|tick|Hz|ms|mm|cm|km|m|s|초|%|도|°|배|개|회|곳|건|대|번|명|점|줄|road|way)"
NUM = re.compile(r"(?<![\w.#/-])([+-]?\d+(?:,\d{3})*(?:\.\d+)?)(?:\s*(?:~|-)\s*([+-]?\d+(?:\.\d+)?))?\s*" + UNIT + r"?(?![\w])")
FRAC = re.compile(r"\b(\d+)\s*/\s*(\d+)\b")
SKIP_PREV = re.compile(r"(세션|session|road|r|j|junction|way|노드|node|Phase|phase|단계|번호|s=|lane|id=|ID|sha|v|버전|L|k|CARLA_|0\.9\.|Ubuntu|Humble|carla==|번째|백업|본|슬라이드|그림|행|절|부록|오전|오후)\s*$")
P1 = re.compile(r"정지|감속|TTC|제동|거리|궤적|소요|완주|초|도달|왕복|복귀|비율|%|충돌|벽|road|junction|스폰|spawn|878|고원식|1243|1245|2139|1604|94\.1|88\.4|0\.5")
P2 = re.compile(r"반경|R2|평균R|R |정차 오차|오차|속도|km/h|m/s|avail|swap|Gi|메모리|RSS")
rows = []
for f in FILES:
    p = D / f
    if not p.exists():
        print("없음", f)
        continue
    for i, line in enumerate(p.read_text().splitlines(), 1):
        if re.search(r"sha256|^\s*```", line):
            pass
        for m in NUM.finditer(line):
            v, v2, u = m.group(1), m.group(2), m.group(3) or ""
            pre = line[:m.start()]
            if SKIP_PREV.search(pre):
                continue
            if re.match(r"20\d\d", v) and (u == "" or "-" in line[m.start():m.start() + 8]):
                continue   # 연도·날짜
            if re.search(r"\d{2}\.\d{6}", v):
                continue   # 좌표
            if re.fullmatch(r"\d{6,}", v.replace(",", "")) and u == "":
                continue   # 긴 ID
            if u == "" and re.fullmatch(r"\d", v) and not re.search(r"[x×/]", line[m.end():m.end() + 2]):
                continue   # 목록 번호 등 한 자리 단독
            if u in ("번",) :
                continue
            ctx = line.strip()
            pr = 1 if P1.search(ctx) else (2 if P2.search(ctx) else 3)
            rows.append((f, i, v + (("~" + v2) if v2 else ""), u, pr, ctx[:200]))
out = Path(sys.argv[1])
with open(out, "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["file", "line", "value", "unit", "priority", "context"])
    w.writerows(rows)
from collections import Counter
print("총", len(rows))
print(Counter(r[0] for r in rows))
print(Counter(r[4] for r in rows))
