#!/usr/bin/env bash
# 10/26: OSM -> xodr -> Lanelet2 -> v3.2 -> 정류장·레이어 자료를 빈 폴더에서 다시 만들고 동결본 해시와 견줌(서버 없음)
# 원본 폴더(이 스크립트가 든 campus_mobility_sim, 작업 폴더나 인계 3 사본)는 건드리지 않음
#   <임시 HOME>/campus_mobility_sim 에 스크립트와 입력만 복사해 그 안에서 돌림. 스크립트들이 자기 자리(map/scripts) 기준 상대 경로로 읽고 씀
#   .venv-carla 는 원본 폴더 것을 심볼릭 링크로 씀(carla 0.9.15 의 Osm2Odr, shapely, numpy, matplotlib)
# 기대 해시는 아래 줄마다 적은 sha256 전체. 중간 파일이 없어도 돌게 파일 대신 해시로 견줌
# Osm2Odr 출력 xodr 은 생성 시각이 두 곳(3행 '<!-- generated on <시각> by' 주석, <header> 의 date 속성) 들어감.
#   두 곳을 뺀 내용의 해시를 먼저 보고, 두 곳을 그때 값으로 바꿔 넣은 뒤 파일 해시를 봄. 다음 단계는 바꿔 넣은 파일을 씀
# 사용: bash map/scripts/s71_reproduce.sh <임시 HOME>   (그 아래 campus_mobility_sim 이 이미 있으면 멈춤)
set -uo pipefail
SRC=$(cd "$(dirname "$0")/../.." && pwd)
H=${1:?임시 HOME}
T=$H/campus_mobility_sim
[ -e "$T" ] && { echo "있음: $T"; exit 1; }
export HOME=$H
mkdir -p "$T"
NS=0; ND=0
sha() { sha256sum "$1" 2>/dev/null | cut -d' ' -f1; }
cmpf() {  # cmpf <새 파일(T 기준)> <기대 sha256> <이름>
  local g; g=$(sha "$1")
  if [ "$g" = "$2" ]; then echo "같음 $1 ${g:0:16}"; NS=$((NS+1)); else echo "다름 $1 ${g:0:16} (기대 ${2:0:16} $3)"; ND=$((ND+1)); fi; }
nodate() { sed -E -e 's/^<!-- generated on [0-9: -]+ by/<!-- generated on X by/' -e 's/ +date="[^"]*"/ date=""/' "$1"; }
cmpx() {  # cmpx <새 xodr> <시각 뺀 sha256> <그때 'generated on' 시각> <그때 date 값> <기대 sha256> <이름>
  local g; g=$(nodate "$1" | sha256sum | cut -d' ' -f1)
  if [ "$g" = "$2" ]; then echo "같음(시각 뺌) $1"; NS=$((NS+1)); else echo "다름(시각 뺌) $1 ${g:0:16} (기대 ${2:0:16})"; ND=$((ND+1)); fi
  sed -i -E -e "s/^<!-- generated on [0-9: -]+ by/<!-- generated on $3 by/" -e "0,/ +date=\"[^\"]*\"/s// date=\"$4\"/" "$1"
  cmpf "$1" "$5" "$6"; }
cpin() { for f in "$@"; do mkdir -p "$T/$(dirname "$f")"; cp -p "$SRC/$f" "$T/$f"; echo "입력 $f $(sha "$T/$f" | cut -c1-16)"; done; }

echo "== 0 준비 $(date +%T)"
mkdir -p "$T/map" "$T/autoware_map" "$T/map/logs" "$T/map/docs/logs" "$T/map/docs/figures" "$T/map/maps" "$T/map/data/processed"
cp -rp "$SRC/map/scripts" "$T/map/scripts"; rm -rf "$T/map/scripts/__pycache__"
cp -rp "$SRC/autoware_map/scripts" "$T/autoware_map/scripts"; rm -rf "$T/autoware_map/scripts/__pycache__"
ln -s "$SRC/.venv-carla" "$T/.venv-carla"
PY=$T/.venv-carla/bin/python
# 그때 받은 원본: OSM(2026-09-15), 캠퍼스 경계 폴리곤(Nominatim relation 6705106)
cpin map/data/raw/cbnu_campus.osm map/data/processed/cbnu_relation_polygon.json
# s27_reconvert.py check 가 해시로 확인하는 기준 파일(frozen_v1 과 그 중간 OSM). 1-0 에서 다시 만들어 같은지도 봄
cpin map/maps/cbnu_internal_only_localtm_tags73_smooth.xodr map/data/processed/cbnu_campus_fixed_tags73.osm \
     map/data/processed/cbnu_internal_boundary_tags73.osm map/data/processed/cbnu_internal_boundary_tags73_smooth.osm
# 사람이 고른 목록(손으로 정한 입력)
cpin map/build_v31_s54/s54_chord_list_no1317_1825.json map/data/crossings_v3_candidates.csv \
     map/data/stops_v3_draft_s46.yaml map/docs/logs/s25_buildings.csv map/data/processed/s40_b2_cut.csv
# 스크립트가 만든 중간 결과지만 이 재현에서는 다시 안 만드는 것(s42_diag.py 가 v2 에서 만든 way 방향표, v2 정류장)
cpin map/docs/logs/s42_way_dir.csv map/data/stops_v2.yaml
# 노선표(road 열). s53_geom·place_campus_layers 가 불러 씀
cpin map/tests/test_drive.py
cd "$T"
MP=map/maps; DP=map/data/processed

echo "== 1-0 frozen_v1 기준선(태그 보정 73) $(date +%T)"
$PY map/scripts/s27_reconvert.py run --base --tag r71base > map/logs/s71_r71base.log 2>&1; echo "exit $?"
cmpf $DP/cbnu_campus_fixed_r71base.osm dfcc5ff7b1a41532424e5c3360bd23b313d1e905c60840ef2f4f4a570808a43b cbnu_campus_fixed_tags73.osm
cmpf $DP/cbnu_internal_boundary_r71base.osm ce457630b2de089f25fa20bfec9f5144ecde7f08960433017c8a1f4fedf5fb05 cbnu_internal_boundary_tags73.osm
cmpf $DP/cbnu_internal_boundary_r71base_smooth.osm caa2c69b026ccc75949ae2ebbd9f809893193a381c39ee1cdf30feb00dda4d1e cbnu_internal_boundary_tags73_smooth.osm
cmpx $MP/cbnu_internal_only_localtm_r71base_smooth.xodr 9f4febaad432d60c70427c50d4cf52f1b9f2fb6aee72c032aaaa1b80bb41f1e4 \
     "2026-09-24 01:43:06" "Thu Sep 24 01:43:06 2026" bf835cdfad0cea65b813e59bda2fd268edfbbfde1069dd5d217ae119d0b1cc9a frozen_v1

echo "== 1-1 v2 변환(B안 태그 보정 81) $(date +%T)"
$PY map/scripts/s27_reconvert.py run --variant B > map/logs/s71_tags81B.log 2>&1; echo "exit $?"
cmpf $DP/cbnu_campus_fixed_tags81_B.osm a60f88de3c9679c861e705e65b571574537766629c96e7924acd60ebfde8ec62 cbnu_campus_fixed_tags81_B.osm
cmpf $DP/cbnu_internal_boundary_tags81_B.osm a4ed787243952adf6505e22814c568bde85894866d4e5f54fc17dcc196bbfb1c cbnu_internal_boundary_tags81_B.osm
cmpf $DP/cbnu_internal_boundary_tags81_B_smooth.osm f682fd34eb05fa2ad1c523bab1a9b9ecbce72aff3036c1ef568b594914dd9335 cbnu_internal_boundary_tags81_B_smooth.osm
cmpx $MP/cbnu_internal_only_localtm_tags81_B_smooth.xodr 70751f1bdb3b7f44fca150de43089d927840dc30dc93064fac5ec293dd621578 \
     "2026-10-07 14:11:25" "Wed Oct  7 14:11:25 2026" d01e5f86767bc9d777012c86bd2e89cf7499d7b121b7b07fc81304dbc2307a60 tags81_B_smooth.xodr

echo "== 1-2 A안(태그 보정 82, s43 이 정문 출구 형상만 읽음) $(date +%T)"
$PY map/scripts/s27_reconvert.py run --variant A > map/logs/s71_tags82A.log 2>&1; echo "exit $?"
cmpx $MP/cbnu_internal_only_localtm_tags82_A_smooth.xodr a0804df2e799385d2bb28a74c931ef646388a650b5fc8d69a4635554149c7e5a \
     "2026-10-07 14:10:51" "Wed Oct  7 14:10:51 2026" a0b2d75b8d24d20cd31da0b690a198b080e2aaae5ed307e13159503be0536df1 tags82_A_smooth.xodr

echo "== 1-3 회차 후처리 -> v2 $(date +%T)"
$PY map/scripts/s27_turnfix.py write $MP/cbnu_internal_only_localtm_tags81_B_smooth.xodr \
    $MP/cbnu_internal_only_localtm_tags81_B_smooth_turnfix.xodr > map/logs/s71_turnfix.log 2>&1; echo "exit $?"
cmpf $MP/cbnu_internal_only_localtm_tags81_B_smooth_turnfix.xodr 5240ca8b883e42c02840ae863fc2df47e0f5b7371f714444c8b0b122522ed922 frozen_v2
cp -p $MP/cbnu_internal_only_localtm_tags81_B_smooth_turnfix.xodr $MP/cbnu_campus_frozen_v2.xodr

echo "== 2 v2 -> v3(s43 -> s45 -> s46) $(date +%T)"
$PY map/scripts/s43_build_v3.py > map/logs/s71_s43.log 2>&1; echo "exit $?"
cmpf $MP/cbnu_campus_v3_candidate_s43.xodr dae3f058ed86f907390856236cd653fedf6203a78c1117698f41896d846ff165 v3_candidate_s43
$PY map/scripts/s45_build_v3.py > map/logs/s71_s45.log 2>&1; echo "exit $?"
cmpf $MP/cbnu_campus_v3_candidate_s45.xodr d9b9c504e93e2255687e6a9facc2f53b615a049f1287c62057b11354fad946e7 v3_candidate_s45
$PY map/scripts/s46_build_v3.py > map/logs/s71_s46.log 2>&1; echo "exit $?"
cmpf $MP/cbnu_campus_v3_candidate_s46.xodr a17ab012e04c664fb2fb558deff6a227a2e105b77cee19be851f2816914d06d8 frozen_v3
cp -p $MP/cbnu_campus_v3_candidate_s46.xodr $MP/cbnu_campus_frozen_v3.xodr

echo "== 3 v3 -> v3.1(s53 -> s54) $(date +%T)"
$PY map/scripts/s53_build_v31.py > map/logs/s71_s53.log 2>&1; echo "exit $?"
cmpf $MP/cbnu_campus_v31_candidate_s53.xodr b9cc23692441de3ae968032afb88b9bc1c9951619ca158b223a51fe7de9f8391 v31_candidate_s53
$PY map/scripts/s54_build.py > map/logs/s71_s54.log 2>&1; echo "exit $?"
cmpf $MP/cbnu_campus_v31_candidate_s54.xodr 214507afc87c2523f191e1c19154fb44b781eb16930b15bbc965fa2ab37ceb84 frozen_v31
cp -p $MP/cbnu_campus_v31_candidate_s54.xodr $MP/cbnu_campus_frozen_v31.xodr

echo "== 4 v3.1 Lanelet2(s57_make_v31_lanelet2.sh) $(date +%T)"
# 기본 출력 폴더(map/build_s57_regen)로 만듦. 검사 스크립트(s63·s66·s70 route_check)가 그 안 l1_meta.json 을 읽음
bash map/scripts/s57_make_v31_lanelet2.sh > map/logs/s71_s57.log 2>&1; echo "exit $?"; grep -E "^(같음|다름)" map/logs/s71_s57.log
cmpf map/build_s57_regen/v31/lanelet2_map.osm 28a57711094effaf3e0523cfd103e94a010c2bac01d37feebd0de90438dd02f9 v31_lanelet2
cmpf map/build_s57_regen/v31/map_projector_info.yaml a6fb6af33e164e61a075f9a79eeffa5eca3ead4435dc3de49e811525175b8307 map_projector_info
mkdir -p autoware_map/cbnu_campus_v31; cp -p map/build_s57_regen/v31/lanelet2_map.osm map/build_s57_regen/v31/map_projector_info.yaml autoware_map/cbnu_campus_v31/

echo "== 5 v3.2(s63 -> s66 v32a·v32b) $(date +%T)"
$PY map/scripts/s63_v32_candidate.py > map/logs/s71_s63.log 2>&1; echo "exit $?"
cmpf $MP/v32_candidate/lanelet2_map.osm 4719e5e5a4f4588bc0e8a48c53d9bc71beaade7c446389e0f5a4ee3c47cfc789 v32_candidate
$PY map/scripts/s66_make_v32ab.py > map/logs/s71_s66v32.log 2>&1; echo "exit $?"
cmpf $MP/v32a/lanelet2_map.osm 4719e5e5a4f4588bc0e8a48c53d9bc71beaade7c446389e0f5a4ee3c47cfc789 v32a
cmpf $MP/v32b/lanelet2_map.osm 485429206e9861547e8ef5a2fa5858637abea94f41fcfae38e9044d28ded4305 v32b
cmpf $MP/v32a/map_projector_info.yaml a6fb6af33e164e61a075f9a79eeffa5eca3ead4435dc3de49e811525175b8307 v32a_projector

echo "== 6 정류장·레이어 자료 $(date +%T)"
$PY map/scripts/s48_stops.py > map/logs/s71_s48.log 2>&1; echo "exit $?"
cmpf map/data/stops_v3.yaml befb9fddffe112add253f3a1ff65e00ba7973b91bb5b97fba0c11272dd14576e stops_v3
$PY map/scripts/s49_b2_v3.py > map/logs/s71_s49.log 2>&1; echo "exit $?"
cmpf map/data/buildings_layout_v3.csv ac9bd41d4ef7561f9d3f601a04dd5919729f06cb7f3f3203e9fb1829d5cf3158 buildings_layout_v3
$PY map/scripts/s66_b2_tier2.py > map/logs/s71_s66t2.log 2>&1; echo "exit $?"
cmpf map/data/buildings_layout_tier2_s66.csv 4cb019764ada3fe4047df3919327c9713e80cea016821482b980b9bfda9884af buildings_layout_tier2_s66
$PY map/scripts/s66_trees.py > map/logs/s71_s66tr.log 2>&1; echo "exit $?"
cmpf map/data/trees_layout_s66.csv 0c60976b39f60111ed2e0484183178f181b607f90e57c0a1ba45c43271f8c140 trees_layout_s66
echo "== 끝 $(date +%T) 같음 $NS 다름 $ND"
