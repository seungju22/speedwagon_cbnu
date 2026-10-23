#!/usr/bin/env bash
# 맵세션57: v3.1 Autoware 지도(28a57711)를 동결 xodr 에서 다시 만들고 해시를 맞춰 본다
# 1) l1_build_lanelet2.py: 동결 xodr + 보정 목록 14개 -> 빌더 출력(83edb775 이어야 함)
# 2) s55_drop_lanelets.py: 빌더 출력에서 r1318·r1188 레인렛만 지움 -> 28a57711 이어야 함
# 빌더에 제외 규칙을 넣지 않는 까닭: 빌더 안에서 빼면 끝점 공유 평균을 다시 해 이웃 레인렛 끝점이 바뀐다(맵세션55)
# 사용: bash map/scripts/s57_make_v31_lanelet2.sh [출력 폴더, 기본 map/build_s57_regen]  (이미 있으면 멈춤)
set -euo pipefail
cd "$(dirname "$0")/../.."
OUT=${1:-map/build_s57_regen}
XODR=map/maps/cbnu_campus_frozen_v31.xodr
CHORD=map/build_v31_s54/s54_chord_list_no1317_1825.json
WANT_XODR=214507afc87c2523f191e1c19154fb44b781eb16930b15bbc965fa2ab37ceb84
WANT_BUILD=83edb7755588184fa5a561fa508746ec166ef5171698b517123eae06e73d2c88
WANT_V31=28a57711094effaf3e0523cfd103e94a010c2bac01d37feebd0de90438dd02f9
WANT_PROJ=a6fb6af33e164e61a075f9a79eeffa5eca3ead4435dc3de49e811525175b8307

[ -e "$OUT" ] && { echo "있음: $OUT (지우거나 다른 이름)"; exit 1; }
h() { sha256sum "$1" | cut -d' ' -f1; }
chk() { [ "$(h "$1")" = "$2" ] && echo "같음 $1 ${2:0:16}" || { echo "다름 $1 $(h "$1" | cut -c1-16) (기대 ${2:0:16})"; exit 1; }; }

chk "$XODR" "$WANT_XODR"
.venv-carla/bin/python autoware_map/scripts/l1_build_lanelet2.py "$XODR" "$OUT/build" --chord-fix "$CHORD"
chk "$OUT/build/lanelet2_map.osm" "$WANT_BUILD"
python3 map/scripts/s55_drop_lanelets.py "$OUT/build" "$OUT/v31" 1318 1188
chk "$OUT/v31/lanelet2_map.osm" "$WANT_V31"
chk "$OUT/v31/map_projector_info.yaml" "$WANT_PROJ"
