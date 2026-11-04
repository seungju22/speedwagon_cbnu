"""10/23 3단계: 소환 지연 셈. 장면 csv(s68_spawn_lat.py: actor, spawn_frame)와 정답 노드 기록 csv(why=pub 첫 frame)를 맞춤
지연 = (처음 낸 frame - 소환 frame) x 0.05s. 서버 쪽 frame 끼리 비교라 노드 자기 기록의 'new id' 시각에 기대지 않음
사용: s68_spawn_lat_eval.py <장면 csv> <정답 기록 csv> [...]
"""
import csv, json, sys
sp = {int(r["actor"]): int(r["spawn_frame"]) for r in csv.DictReader(open(sys.argv[1]))}
out = {"spawned": len(sp)}
for g in sys.argv[2:]:
    first = {}
    for r in csv.DictReader(open(g)):
        if r["why"] == "pub": first.setdefault(int(r["aid"]), int(r["frame"]))
    d = sorted(round((first[i] - f) * 0.05, 2) for i, f in sp.items() if i in first)
    out[g.split("/")[-1]] = {"published": len(d), "missing": len(sp) - len(d), "delay_s": d,
                              "mean": round(sum(d) / len(d), 3) if d else None, "max": d[-1] if d else None}
print(json.dumps(out, ensure_ascii=False))
