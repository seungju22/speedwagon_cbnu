#!/usr/bin/env bash
# 맵세션33: 양성재 정차 재현성 1회차 = 서버 기동 -> 맵 로드 -> 2분 메모리 -> 양성재 주행 -> PID kill -> 서버 로그 집계
# 사용: bash map/scripts/s33_one_run.sh <v1|v2> <회차 번호> [노선, 기본 yangseong]
# 서버 플래그는 세션32와 같게 고정(-quality-level=Low -win). kill 은 기록한 Shipping PID 만.
# 메모리: 측정 시점 무관 avail < 1024Mi 이면 즉시 kill(세션32 추가 기준). 2분 후 avail < 2048Mi 또는 swap > 4096Mi 이면 kill 후 종료 3.
set -u
MAPK=$1; N=$2; ROUTE=${3:-yangseong}
ROOT=/home/gyumin/campus_mobility_sim
PY=$ROOT/.venv-carla/bin/python
LOGS=$ROOT/map/docs/logs
V2=$ROOT/map/maps/cbnu_internal_only_localtm_tags81_B_smooth_turnfix.xodr
P=${PFX:-s33}; TAG=${P}_${MAPK}_run${N}; [ "$ROUTE" != yangseong ] && TAG=${P}_${MAPK}_${ROUTE}_run${N}
SLOG=/tmp/carla_${TAG}.log
avail() { awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo; }
swapu() { awk '/SwapTotal/{t=$2}/SwapFree/{f=$2}END{print int((t-f)/1024)}' /proc/meminfo; }

ff=$(ps -eo comm | grep -c '^firefox'); cu=$(ps -eo comm | grep -ic carla)
echo "[$TAG] $(date +%T) 시작 firefox=$ff carla=$cu avail=$(avail)Mi swap=$(swapu)Mi"
if [ "$ff" -ne 0 ] || [ "$cu" -ne 0 ]; then echo "[$TAG] 전제 불충족, 중단"; exit 2; fi
# 세션33 추가: GPU 드라이버(TTM) 풀이 재기동마다 쌓여 avail 을 깎는다(drop_caches 로 5.9 -> 11.6GiB 확인). 시작 avail 9GiB 미만이면 중단
if [ "$(avail)" -lt 9000 ]; then echo "[$TAG] 시작 avail $(avail)Mi < 9000Mi, 중단(drop_caches 필요)"; exit 5; fi

cd ~/carla/CARLA_0.9.15 || exit 2
nohup ./CarlaUE4.sh -quality-level=Low -win > "$SLOG" 2>&1 &
SH=$!
for i in $(seq 1 300); do
  if (echo > /dev/tcp/127.0.0.1/2000) 2>/dev/null; then break; fi
  sleep 1
done
PID=$(pgrep -x CarlaUE4-Linux- | head -1)   # comm 은 15자로 잘림. 실행 파일 CarlaUE4-Linux-Shipping
echo "[$TAG] nohup sh PID=$SH"
if ! (echo > /dev/tcp/127.0.0.1/2000) 2>/dev/null; then echo "[$TAG] 포트 2000 5분 내 안 열림"; tail -5 "$SLOG"; [ -n "$PID" ] && kill "$PID"; exit 4; fi
echo "[$TAG] $(date +%T) 포트 열림 ${i}s, server PID=$PID"
# 세션33 수정: 포트가 열려도 월드가 아직 준비 안 됐을 수 있다(1회차 실패). get_world() 가 응답할 때까지 폴링(최대 5분)
for k in $(seq 1 60); do
  if $PY -c "import carla; c=carla.Client('localhost',2000); c.set_timeout(5.0); c.get_world()" 2>/dev/null; then break; fi
  kill -0 "$PID" 2>/dev/null || { echo "[$TAG] 서버 사라짐"; exit 4; }
done
echo "[$TAG] $(date +%T) get_world 응답(시도 ${k}회) avail=$(avail)Mi swap=$(swapu)Mi"

# 메모리 감시(1Gi 미만 즉시 kill)
( while kill -0 "$PID" 2>/dev/null; do a=$(avail); if [ "$a" -lt 1024 ]; then kill "$PID"; echo "[$TAG] $(date +%T) KILL avail=${a}Mi" >> "$LOGS/s33_memguard.log"; sleep 15; kill -0 "$PID" 2>/dev/null && kill -9 "$PID" && echo "[$TAG] $(date +%T) KILL -9" >> "$LOGS/s33_memguard.log"; break; fi; sleep 1; done ) &
GUARD=$!

cd "$ROOT" || exit 2
if [ "$MAPK" = v2 ]; then $PY map/tests/test_load_map_878.py --wall-height 0 --xodr "$V2" > "$LOGS/${TAG}_load.log" 2>&1
else $PY map/tests/test_load_map_878.py --wall-height 0 > "$LOGS/${TAG}_load.log" 2>&1; fi
grep -E "spawn point|일치|road .* junction" "$LOGS/${TAG}_load.log" | sed "s/^/[$TAG] /"
echo "[$TAG] $(date +%T) 로드 직후 avail=$(avail)Mi swap=$(swapu)Mi"
grep -s "\[$TAG\]" "$LOGS/s33_memguard.log" && { echo "[$TAG] 감시 kill 기록 있음, 중단"; sleep 16; kill -0 "$PID" 2>/dev/null && kill -9 "$PID"; exit 3; }
sleep 120
A=$(avail); S=$(swapu)
echo "[$TAG] $(date +%T) 2분 후 avail=${A}Mi swap=${S}Mi"
grep -s "\[$TAG\]" "$LOGS/s33_memguard.log" && { echo "[$TAG] 감시 kill 기록 있음, 중단"; sleep 16; kill -0 "$PID" 2>/dev/null && kill -9 "$PID"; exit 3; }
if ! kill -0 "$PID" 2>/dev/null; then echo "[$TAG] 서버 없음"; exit 3; fi
if [ "$A" -lt 2048 ] || [ "$S" -gt 4096 ]; then kill "$PID"; echo "[$TAG] 중단 기준 도달, kill $PID"; exit 3; fi

if [ "$MAPK" = v2 ]; then $PY map/tests/test_drive.py --route "$ROUTE" --xodr "$V2" > "$LOGS/${TAG}.log" 2>&1
else $PY map/tests/test_drive.py --route "$ROUTE" > "$LOGS/${TAG}.log" 2>&1; fi
echo "[$TAG] $(date +%T) 주행 rc=$? 주행 후 avail=$(avail)Mi swap=$(swapu)Mi"
ERR_RUN=$(grep -a -ciE "error" "$SLOG")

kill "$PID"
for j in $(seq 1 30); do kill -0 "$PID" 2>/dev/null || break; sleep 1; done
kill -0 "$PID" 2>/dev/null && { echo "[$TAG] PID $PID SIGTERM 30s 뒤 생존 -> SIGKILL"; kill -9 "$PID"; sleep 2; }
wait $GUARD 2>/dev/null
echo "[$TAG] $(date +%T) 종료 carla 프로세스=$(ps -eo comm | grep -ic carla) 주행 중 서버 error 줄=$ERR_RUN Signal11=$(grep -a -c 'Signal 11 caught' "$SLOG")"
