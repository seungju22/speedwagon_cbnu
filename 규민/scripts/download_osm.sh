#!/usr/bin/env bash
# 충북대 캠퍼스+300m 여유 사각형을 OSM 기본 API(/api/0.6/map)로
# 내려받는다. 좌표 출처: map/scripts/compute_bbox.py 출력
# (1-2 산출, Nominatim relation 6705106 기준, 2026-09-15).
# 이 범위 실측(Overpass count, 2026-09-15): node 9,997 / way 2,067 /
# relation 38, 면적 약 0.00033평방도 — 기본 API 제한
# (0.25평방도, 50,000노드)에 여유있게 부합해 기본 API를 사용한다.
set -euo pipefail

# bbox=left(서),bottom(남),right(동),top(북) — WGS84 십진도
WEST=127.446624
SOUTH=36.619902
EAST=127.466839
NORTH=36.636427

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT="${SCRIPT_DIR}/../data/raw/cbnu_campus.osm"
URL="https://api.openstreetmap.org/api/0.6/map?bbox=${WEST},${SOUTH},${EAST},${NORTH}"

# OSM API 사용정책은 연락 가능한 User-Agent를 요구한다
# (wiki.openstreetmap.org/wiki/API_usage_policy). 개인 연락처를
# 저장소에 평문으로 남기지 않기 위해 OSM_CONTACT 환경변수로
# 받는다. 미설정 시 프로젝트명만 사용(정책 최소 요건은 충족하되
# 연락처는 빠짐 — 필요시 실행 전 export OSM_CONTACT="..." 권장).
USER_AGENT="campus_mobility_sim-research/0.1"
if [ -n "${OSM_CONTACT:-}" ]; then
  USER_AGENT="${USER_AGENT} (${OSM_CONTACT})"
fi

echo "요청: ${URL}"

HTTP_CODE=$(curl -sS \
  -H "User-Agent: ${USER_AGENT}" \
  -w "%{http_code}" \
  -o "${OUTPUT}" \
  "${URL}") || {
    echo "실패: curl 요청 자체가 실패함(네트워크 오류 등)" >&2
    exit 1
  }

if [ "${HTTP_CODE}" != "200" ]; then
  echo "실패: HTTP ${HTTP_CODE} 응답" >&2
  echo "--- 응답 본문(에러 메시지일 수 있음) ---" >&2
  cat "${OUTPUT}" >&2
  exit 1
fi

if [ ! -s "${OUTPUT}" ]; then
  echo "실패: 응답 본문이 비어 있음" >&2
  exit 1
fi

if ! head -c 100 "${OUTPUT}" | grep -q "<?xml"; then
  echo "실패: XML로 시작하지 않음(유효한 OSM 응답 아님)" >&2
  echo "--- 응답 앞부분 ---" >&2
  head -c 300 "${OUTPUT}" >&2
  exit 1
fi

echo "성공: ${OUTPUT} 저장됨"
ls -lh "${OUTPUT}"
