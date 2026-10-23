#!/bin/bash
# I6 정지 감시(2026-10-11): 자율 전환 뒤 차가 <hold_s> 초 동안 0.05m/s 밑이고 목표(ROS map 좌표)에서 3m 넘게 떨어져 있으면
# 컨테이너 안 i6_diag.py 로 세운 모듈·사유를 남기고 i2_watch.py 를 끝내 회차를 마감시킨다(I5·S4 는 손으로 끊음)
# 사용: i6_stall.sh <tag> <gx> <gy> <hold_s>
TAG=$1; GX=$2; GY=$3; H=${4:-90}; L=~/campus_mobility_sim/integration/logs; S='source /opt/autoware/setup.bash;'
until grep -q engage_wall $L/i5_drive_$TAG.log 2>/dev/null; do sleep 5; done
while sleep 10; do
  r=$(python3 - "$L/i5_track_$TAG.csv" $GX $GY $H <<'PY'
import csv, math, sys, time
rows = list(csv.reader(open(sys.argv[1])))[1:]
gx, gy, h = float(sys.argv[2]), -float(sys.argv[3]), float(sys.argv[4])
if not rows: print("wait"); sys.exit()
now = float(rows[-1][0]); x, y = float(rows[-1][2]), float(rows[-1][3])
win = [r for r in rows if float(r[0]) >= now - h]
moving = any(float(r[5]) > 0.05 for r in win)
d = math.hypot(x - gx, y - gy)
print("stall" if (not moving and float(win[0][0]) <= now - h + 5 and d > 3) else "ok", f"{d:.1f}")
PY
)
  case "$r" in stall*)
    echo "$(date +%T) stall $r" >> $L/i6_stall_$TAG.log
    docker exec -u 1000:1000 i1_aw bash -c "$S timeout 60 python3 /home/aw/i3_tools/i6_diag.py" > $L/i6_stop_diag_$TAG.txt 2>&1
    echo "$(date +%T) diag done, i2_watch 끝냄" >> $L/i6_stall_$TAG.log
    docker exec i1_aw pkill -INT -f i2_watch.py; exit 0;;
  esac
done
