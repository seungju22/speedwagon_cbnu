#!/usr/bin/env bash
# 맵세션41: 사용자가 켠 서버에서 (load) load_campus_world.py 기본값 실행 또는 (노선) test_drive.py --route <노선> --xodr 동결본 1회
# 서버 기동·종료 없음(사용자). 메모리 감시: avail < 1024Mi 즉시 kill(세션32 기준). 주행 전 avail < 2048Mi 또는 swap > 4096Mi 면 주행 안 함
# 사용: bash map/scripts/s41_run.sh load | north | south | middle | yangseong
set -u
WHAT=$1
ROOT=/home/gyumin/campus_mobility_sim
PY=$ROOT/.venv-carla/bin/python
LOGS=$ROOT/map/docs/logs
FROZEN=$ROOT/map/maps/cbnu_campus_frozen_v2.xodr
TAG=s41_${WHAT}
avail() { awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo; }
swapu() { awk '/SwapTotal/{t=$2}/SwapFree/{f=$2}END{print int((t-f)/1024)}' /proc/meminfo; }
PID=$(pgrep -x CarlaUE4-Linux- | head -1)
[ -z "$PID" ] && { echo "[$TAG] 서버 없음"; exit 2; }
echo "[$TAG] $(date +%T) 시작 server PID=$PID avail=$(avail)Mi swap=$(swapu)Mi"
( while kill -0 "$PID" 2>/dev/null; do a=$(avail); if [ "$a" -lt 1024 ]; then kill "$PID"; echo "[$TAG] $(date +%T) KILL avail=${a}Mi" >> "$LOGS/s41_memguard.log"; break; fi; sleep 1; done ) &
GUARD=$!
trap 'kill $GUARD 2>/dev/null' EXIT
cd "$ROOT" || exit 2
if [ "$WHAT" = load ]; then
  $PY map/scripts/load_campus_world.py > "$LOGS/${TAG}.log" 2>&1
  echo "[$TAG] $(date +%T) 로더 rc=$? 직후 avail=$(avail)Mi swap=$(swapu)Mi"
  sed "s/^/[$TAG] /" "$LOGS/${TAG}.log"
  exit 0
fi
A=$(avail); S=$(swapu)
grep -s "\[s41_" "$LOGS/s41_memguard.log" && { echo "[$TAG] 감시 kill 기록 있음, 중단"; exit 3; }
if [ "$A" -lt 2048 ] || [ "$S" -gt 4096 ]; then echo "[$TAG] 중단 기준 도달 avail=${A}Mi swap=${S}Mi, 주행 안 함"; exit 3; fi
$PY map/tests/test_drive.py --route "$WHAT" --xodr "$FROZEN" > "$LOGS/${TAG}.log" 2>&1
echo "[$TAG] $(date +%T) 주행 rc=$? 주행 후 avail=$(avail)Mi swap=$(swapu)Mi"
