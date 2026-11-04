#!/usr/bin/env python3
# 11/03: 헤맴 막기 감시. s4_track.py 가 매 틱 쓰는 track csv 를 따라 읽다가 바퀴 이탈(wheels_off > 0)이 시뮬 시간 --sec 초 넘게 이어지면
#   <marker> 파일에 까닭을 쓰고 구간 스크립트(--pid)를 TERM 으로 끔(그 회차를 끊음). 구간 시간 2배 넘김은 s73_ps_goal.sh 의 TO 로 끊음
# 사용: python3 s75_offroad_watch.py <track.csv> <marker> --pid <구간 스크립트 PID> [--sec 10]
import argparse, os, signal, time

ap = argparse.ArgumentParser(); ap.add_argument("csv"); ap.add_argument("marker"); ap.add_argument("--pid", type=int, required=True); ap.add_argument("--sec", type=float, default=10.0)
a = ap.parse_args()
for _ in range(120):
    if os.path.exists(a.csv): break
    time.sleep(1)
f = open(a.csv); hdr = f.readline().strip().split(","); f.seek(0, 2)   # 지금 끝부터(앞 구간은 안 봄)
iw, isim, ix, iy = hdr.index("wheels_off"), hdr.index("sim"), hdr.index("x"), hdr.index("y")
t_off = None; buf = ""
while True:
    try: os.kill(a.pid, 0)
    except OSError: break
    line = f.readline()
    if not line: time.sleep(0.2); continue
    buf += line
    if not buf.endswith("\n"): continue
    r = buf.strip().split(","); buf = ""
    if len(r) < len(hdr): continue
    sim, wo = float(r[isim]), int(r[iw])
    if wo > 0:
        if t_off is None: t_off = sim
        if sim - t_off >= a.sec:
            msg = f"cut offroad {sim - t_off:.1f}s wheels {wo} at {r[ix]},{r[iy]} sim {sim:.1f}"
            open(a.marker, "w").write(msg + "\n"); print(time.strftime("%T"), msg, flush=True)
            try: os.kill(a.pid, signal.SIGTERM)
            except OSError: pass
            break
    else:
        t_off = None
