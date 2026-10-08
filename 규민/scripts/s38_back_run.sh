#!/usr/bin/env bash
# 맵세션38: 사용자가 켠 서버에서 후문(back) 1회 = (첫 회만) v2 로드 -> 2분 메모리 -> 주행 -> 주행 후 메모리
# s33_one_run.sh 에서 서버 기동·종료를 뺀 것. 서버 종료는 사용자가 한다
# 사용: bash map/scripts/s38_back_run.sh <회차 번호> [load]
# 메모리: avail < 1024Mi 이면 즉시 kill(세션32 기준). 2분 후 avail < 2048Mi 또는 swap > 4096Mi 이면 주행 안 함, 종료 3
set -u
N=$1; LOAD=${2:-}
ROOT=/home/gyumin/campus_mobility_sim
PY=$ROOT/.venv-carla/bin/python
LOGS=$ROOT/map/docs/logs
V2=$ROOT/map/maps/cbnu_internal_only_localtm_tags81_B_smooth_turnfix.xodr
TAG=s38_v2_back_run${N}
avail() { awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo; }
swapu() { awk '/SwapTotal/{t=$2}/SwapFree/{f=$2}END{print int((t-f)/1024)}' /proc/meminfo; }

PID=$(pgrep -x CarlaUE4-Linux- | head -1)
[ -z "$PID" ] && { echo "[$TAG] 서버 없음"; exit 2; }
echo "[$TAG] $(date +%T) 시작 server PID=$PID avail=$(avail)Mi swap=$(swapu)Mi"

( while kill -0 "$PID" 2>/dev/null; do a=$(avail); if [ "$a" -lt 1024 ]; then kill "$PID"; echo "[$TAG] $(date +%T) KILL avail=${a}Mi" >> "$LOGS/s38_memguard.log"; break; fi; sleep 1; done ) &
GUARD=$!
trap 'kill $GUARD 2>/dev/null' EXIT

cd "$ROOT" || exit 2
if [ "$LOAD" = load ]; then
  $PY map/tests/test_load_map_878.py --wall-height 0 --xodr "$V2" > "$LOGS/${TAG}_load.log" 2>&1
  grep -E "spawn point|일치|road .* junction" "$LOGS/${TAG}_load.log" | sed "s/^/[$TAG] /"
  echo "[$TAG] $(date +%T) 로드 직후 avail=$(avail)Mi swap=$(swapu)Mi"
  sleep 120
fi
A=$(avail); S=$(swapu)
echo "[$TAG] $(date +%T) 주행 전 avail=${A}Mi swap=${S}Mi"
grep -s "\[$TAG\]" "$LOGS/s38_memguard.log" && { echo "[$TAG] 감시 kill 기록 있음, 중단"; exit 3; }
kill -0 "$PID" 2>/dev/null || { echo "[$TAG] 서버 없음"; exit 3; }
if [ "$A" -lt 2048 ] || [ "$S" -gt 4096 ]; then echo "[$TAG] 중단 기준 도달, 주행 안 함"; exit 3; fi

$PY map/tests/test_drive.py --route back --xodr "$V2" > "$LOGS/${TAG}.log" 2>&1
echo "[$TAG] $(date +%T) 주행 rc=$? 주행 후 avail=$(avail)Mi swap=$(swapu)Mi"
