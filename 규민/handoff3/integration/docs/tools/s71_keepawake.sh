#!/bin/bash
# 10/26(s70_keepawake.sh 사본, 이름만 s71) 절전 막기(세션 전체). 시스템 설정은 안 바꾸고 막는 쪽(inhibit)만 씀
#   1) systemd-inhibit --what=sleep:idle:handle-lid-switch --mode=block : logind 의 절전·유휴·뚜껑 처리를 막음
#   2) gnome-session-inhibit --inhibit suspend:idle : GNOME 쪽 절전·유휴를 막음(I6 에서 systemd-inhibit 만으로는 배터리 20분 절전을 못 막았음)
#   3) 60초마다 전원(ADP1·배터리 %)·두 inhibit 살아 있는지·GNOME 절전 설정을 한 줄로 남김. inhibit 가 죽으면 다시 띄우고 적음
#      배터리로 돌면 WARN, 20% 밑이면 <log>.lowbat 파일을 만듦(서버 회차는 이 파일이 있으면 시작하지 않음)
# 끄기: 이 스크립트 PID 에 kill(TERM). 자식 inhibit 둘도 PID 로 끄고 끝남
# 사용: nohup bash s70_keepawake.sh <log> > /dev/null 2>&1 &
LOG=$1; PF=$LOG.pids; S=sleep:idle:handle-lid-switch
start_sys() { systemd-inhibit --what=$S --mode=block --who=s71 --why="s71 keep awake" sleep infinity & SYS=$!; }
start_gn() { gnome-session-inhibit --inhibit suspend:idle --reason "s71 keep awake" sleep infinity > /dev/null 2>&1 & GN=$!; }
stop_all() { kill $SYS $GN 2>/dev/null; echo "$(date +%T) stop sys $SYS gn $GN" >> $LOG; rm -f $PF; exit 0; }
trap stop_all TERM INT
start_sys; start_gn; echo "SELF=$$ SYS=$SYS GN=$GN" > $PF
echo "$(date +%T) start self $$ sys $SYS gn $GN" >> $LOG
echo "time adp1 bat_status bat_pct sys gn ac_type bat_type" >> $LOG
while true; do
  kill -0 $SYS 2>/dev/null || { start_sys; echo "$(date +%T) RESTART sys $SYS" >> $LOG; echo "SELF=$$ SYS=$SYS GN=$GN" > $PF; }
  kill -0 $GN 2>/dev/null || { start_gn; echo "$(date +%T) RESTART gn $GN" >> $LOG; echo "SELF=$$ SYS=$SYS GN=$GN" > $PF; }
  ac=$(cat /sys/class/power_supply/ADP1/online); st=$(cat /sys/class/power_supply/BAT*/status | head -1); pct=$(cat /sys/class/power_supply/BAT*/capacity | head -1)
  at=$(gsettings get org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type); bt=$(gsettings get org.gnome.settings-daemon.plugins.power sleep-inactive-battery-type)
  echo "$(date +%T) $ac $st $pct $SYS $GN $at $bt" >> $LOG
  [ "$ac" = 0 ] && echo "$(date +%T) WARN 배터리로 돎 $pct%" >> $LOG
  if [ "$ac" = 0 ] && [ "$pct" -lt 20 ]; then touch $LOG.lowbat; else rm -f $LOG.lowbat; fi
  sleep 60 & wait $!
done
