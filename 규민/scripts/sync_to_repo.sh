#!/usr/bin/env bash
# 작업 폴더(campus_mobility_sim/map) -> 팀 저장소(speedwagon_cbnu/규민) 동기화. 세션28 마감 뒤 작성(2026-10-07)
# 동기화 시점: 점 병합 시험 끝 / 재변환 + 노선 재검증 끝 / Lanelet2 단계 끝 / 제출 전. 매 세션 하지 않는다
# 기본은 --dry-run(보기만). 실제 복사는 --apply 를 줘야 한다
# git add·commit·push 는 하지 않는다. 복사 뒤 사용자가 저장소에서 직접 확인·커밋
# 지우지 않는다(--delete 없음). 저장소에만 있는 파일이 생겨도 그대로 남는다
#   (README·ATTRIBUTION·NOTICE 는 세션29c 부터 작업 폴더에 원본이 있다)
#
# 대상
#   map/docs/    -> 규민/docs/      (docs/figures/ATTRIBUTION.md 포함)
#   map/scripts/ -> 규민/scripts/
#   map/tests/   -> 규민/tests/
#   map/maps/    -> 규민/maps/      (MAPS 에 적은 파일만: 동결본 + NOTICE.md)
#   map/data/    -> 규민/data/      (DATA 에 적은 파일만. 세션41 추가: stops_v2.yaml)
#   docs/(작업 폴더 최상위) -> 규민/docs/ (TOPDOCS 에 적은 파일만. 세션41 추가: handoff_1_README.md)
#   map/README.md -> 규민/README.md
# 제외: __pycache__, *.pyc, *.bak*, *.tar.gz, *.webm, 100MB 초과 파일
# 규칙(2026-10-07): 문서는 campus_mobility_sim 에서만 고친다. 규민/·GitHub 웹에서 직접 고치지 않는다.
#   그래서 보호 목록이 없다. 세션29c 에서 field_survey 2개를 역병합해 단일 원본으로 만든 뒤 보호 목록을 지웠다
# --apply 와 git 명령은 사용자가 지시할 때만 실행한다
#
# 사용: map/scripts/sync_to_repo.sh            # 미리보기
#       map/scripts/sync_to_repo.sh --apply    # 실제 복사
set -euo pipefail

SRC=/home/gyumin/campus_mobility_sim/map
DST=/home/gyumin/speedwagon_cbnu/규민
# maps 에서 보낼 파일. 동결본이 바뀌면 여기만 고친다(예: 재변환 v2 확정 후). NOTICE.md 는 지도 데이터 고지
MAPS=(cbnu_internal_only_localtm_tags73_smooth.xodr cbnu_campus_frozen_v2.xodr NOTICE.md)
# 세션41(사용자 승인, 세션40 B안): data/ 와 작업 폴더 최상위 docs/ 는 목록에 적은 파일만 보낸다
#   data/ 전체는 OSM 원본·중간 파일이 있어 보내지 않는다. 최상위 docs/ 에는 개인 환경 기록(setup_log 등)이 있어 목록만
ROOT=/home/gyumin/campus_mobility_sim
DATA=(stops_v2.yaml)
TOPDOCS=(handoff_1_README.md)

MODE=--dry-run
case "${1:-}" in
  "") ;;
  --apply) MODE= ;;
  *) echo "사용: $0 [--apply]" >&2; exit 2 ;;
esac

[ -d "$SRC" ] || { echo "작업 폴더 없음: $SRC" >&2; exit 1; }
[ -d "$DST" ] || { echo "저장소 폴더 없음: $DST" >&2; exit 1; }

# 토큰이 문서에 섞여 들어가지 않았는지 먼저 본다(세션 중 PAT 붙여넣기 전례). 걸리면 멈춘다
# 세션41: 검사 범위 = 보내는 모든 경로(MAPS·DATA·TOPDOCS 포함)
SCAN=("$SRC/docs" "$SRC/scripts" "$SRC/tests" "$SRC/README.md")
for m in "${MAPS[@]}"; do SCAN+=("$SRC/maps/$m"); done
for d in "${DATA[@]}"; do SCAN+=("$SRC/data/$d"); done
for h in "${TOPDOCS[@]}"; do SCAN+=("$ROOT/docs/$h"); done
if grep -rIlE 'ghp_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}' "${SCAN[@]}" 2>/dev/null; then
  echo "위 파일에 GitHub 토큰 형태 문자열이 있다. 동기화 중단" >&2
  exit 1
fi

EXCL=(--exclude=__pycache__/ --exclude='*.pyc' --exclude='*.bak*' --exclude='*.tar.gz' --exclude='*.webm' --max-size=100m)

echo "== 모드: ${MODE:-실제 복사}"
echo "== docs"
rsync -a $MODE --itemize-changes "${EXCL[@]}" "$SRC/docs/" "$DST/docs/"
echo "== scripts"
rsync -a $MODE --itemize-changes "${EXCL[@]}" "$SRC/scripts/" "$DST/scripts/"
echo "== tests"
rsync -a $MODE --itemize-changes "${EXCL[@]}" "$SRC/tests/" "$DST/tests/"
echo "== maps (MAPS 목록만)"
for m in "${MAPS[@]}"; do
  [ -f "$SRC/maps/$m" ] || { echo "maps 파일 없음: $m" >&2; exit 1; }
  rsync -a $MODE --itemize-changes --max-size=100m "$SRC/maps/$m" "$DST/maps/"
done
echo "== data (DATA 목록만)"
for d in "${DATA[@]}"; do
  [ -f "$SRC/data/$d" ] || { echo "data 파일 없음: $d" >&2; exit 1; }
  rsync -a $MODE --itemize-changes --max-size=100m "$SRC/data/$d" "$DST/data/"
done
echo "== 최상위 docs (TOPDOCS 목록만)"
for h in "${TOPDOCS[@]}"; do
  [ -f "$ROOT/docs/$h" ] || { echo "최상위 docs 파일 없음: $h" >&2; exit 1; }
  rsync -a $MODE --itemize-changes --max-size=100m "$ROOT/docs/$h" "$DST/docs/"
done
echo "== README"
[ -f "$SRC/README.md" ] || { echo "README 없음: $SRC/README.md" >&2; exit 1; }
rsync -a $MODE --itemize-changes "$SRC/README.md" "$DST/README.md"
[ -z "$MODE" ] && echo "복사 끝. git add·commit·push 는 하지 않았다" || echo "미리보기 끝. 실제 복사는 --apply"
