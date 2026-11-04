#!/bin/bash
# 10/23: 가짜 carla(이 폴더 carla.py)로 gt_objects2.py(지금 판 또는 .bak_s68)를 Autoware 1.9.0 컨테이너에서 돌려 '소환 틱 -> 처음 낸 틱'을 셈
# 사용: run_mock.sh <tag> <gt 파일 이름(gt_objects2.py|gt_objects2.py.bak_s68)> <mode car|all> [초 40]
TAG=$1; GT=$2; MODE=$3; DUR=${4:-40}; C=~/campus_mobility_sim; O=$C/perception/logs/s68_mock; mkdir -p $O
DDS='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General></Domain></CycloneDDS>'
# 보행자 12명: 시작 뒤 4초부터 2.35초 간격(틱 47, 1초 refresh 와 위상이 계속 바뀌게)
SP=$(python3 -c "print(','.join(str(80 + 47 * k) for k in range(12)))")
docker run --rm --network none --cap-add NET_ADMIN -u 1000:1000 -v $C/perception:/p -e CYCLONEDDS_URI="$DDS" \
  -e MOCK_SPAWN=$SP -e MOCK_SPAWN_LOG=/p/logs/s68_mock/spawn_$TAG.txt -e PYTHONPATH=/p/tools/s68_mock:/p/gt_objects \
  --entrypoint bash ghcr.io/autowarefoundation/autoware:universe-humble-1.9.0 -c "
source /opt/ros/humble/setup.bash; source /opt/autoware/setup.bash
python3 -c '
import rclpy, time
from rosgraph_msgs.msg import Clock
rclpy.init(); n = rclpy.create_node(\"mock_clock\"); p = n.create_publisher(Clock, \"/clock\", 10); t0 = time.time()
while True:
    t = time.time() - t0; m = Clock(); m.clock.sec = int(t); m.clock.nanosec = int((t - int(t)) * 1e9); p.publish(m); time.sleep(0.05)
' > /dev/null 2>&1 &
CP=\$!
cp /p/gt_objects/$GT /tmp/gt_run.py
timeout $DUR python3 -u /tmp/gt_run.py --mode $MODE --log /p/logs/s68_mock/obj_$TAG.csv > /p/logs/s68_mock/gt_$TAG.log 2>&1
kill \$CP; echo done"
python3 - $O/spawn_$TAG.txt $O/obj_$TAG.csv <<'PY'
# 처음 낸 틱 = 기록 csv 에서 그 id 의 why 가 pub 인 첫 frame(옛 판 'new id' 줄에는 frame 이 없어서 기록으로 셈)
import csv, sys
sp = {int(l.split()[1]): int(l.split()[2]) for l in open(sys.argv[1])}
first = {}
for r in csv.DictReader(open(sys.argv[2])):
    if r["why"] == "pub": first.setdefault(int(r["aid"]), int(r["frame"]))
d = [(first[i] - f) * 0.05 if i in first else None for i, f in sorted(sp.items())]
ok = [x for x in d if x is not None]
print("spawned", len(sp), "published", len(ok), "delay_s", [round(x, 2) for x in ok], "max", round(max(ok), 2) if ok else None, "mean", round(sum(ok) / len(ok), 3) if ok else None)
PY
grep '^stat' $O/gt_$TAG.log | tail -1 | cut -c1-300
