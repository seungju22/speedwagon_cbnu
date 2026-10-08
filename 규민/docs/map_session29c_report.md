# 맵세션29c 보고서 (2026-10-07) — 팀 저장소 분기 역병합

## 왜
- 같은 파일이 두 곳에서 따로 고쳐졌다
  - 저장소(~/speedwagon_cbnu/규민): 236132a "docs: 답사 기록 서술 정리"(field_survey 2개), README 는 GitHub 웹 편집 2회(a1df70d, c6bd498) + 로컬 커밋 2회(6bc6312, 869cfc8)
  - 작업(~/campus_mobility_sim/map): 세션28 이 field_survey_2026-10-06.md 에 6·7절 append
- rsync 는 한 방향이라 이번 한 번만 저장소 -> 작업으로 당겨 합쳤다

## 차이 목록 (세션29b 에서 보고, 사용자 결정)
- 양쪽이 다 고친 것: field_survey_2026-10-06.md -> 정리본 위에 세션28 두 절을 다시 얹음
- 저장소만 고친 것: field_survey_2026-10-04.md -> 저장소본 그대로 가져옴(작업본은 공개 이후 바뀐 적 없음)
- 작업만 고친 것: logs/s24_reconv_list.md, reconversion_plan.md(둘 다 append), scripts/s27_reconvert.py(세션28 수정) -> 손대지 않음. 다음 동기화 때 정방향
- 저장소에만 있던 것: README.md, docs/figures/ATTRIBUTION.md, maps/NOTICE.md -> 작업 쪽 같은 상대 경로로 가져옴
- 저장소 HEAD 869cfc8 = origin/main(ls-remote 로 확인). 저장소 작업 트리 변경 없음

## 한 일
- 백업: 바꾸기 전 작업본 field_survey 2개, sync_to_repo.sh, CLAUDE.md 를 scratchpad/s29c_backup 에 복사(세션 끝나면 사라짐)
- field_survey_2026-10-04.md: 저장소본으로 교체. 바이트 동일 확인
- field_survey_2026-10-06.md: 저장소본 + 두 절. 저장소본 바이트가 앞부분과 그대로 같음을 확인
  - 얹은 절(정리본 문체: 평서체, 1인칭, 분류어 없는 소제목)
    - "고가차도 근거 다시 보기 (세션28, 2026-10-07)" = 옛 6절
    - "후문은 B (세션28 마감, 2026-10-07)" = 옛 7절
  - 절 번호 참조("5절", "6절")는 정리본에 절 번호가 없어서 "위 OSM 대조 절", "위 절" 로 바꿨다
- 가져온 파일: map/README.md, map/docs/figures/ATTRIBUTION.md, map/maps/NOTICE.md(저장소와 바이트 동일)
  - map/maps 는 9개 -> 10개(NOTICE.md 추가). xodr 는 그대로

## 검증 (scripts/s29c_token_check.py, logs/s29c_token_check.log)
- 방법: 숫자가 들어간 ASCII 토큰(좌표, way id, m2, G5, E7-3, 날짜 등)을 뽑아 병합본에 전부 있는지 집합으로 비교
- 10-06: 작업 원본 79종 중 빠짐 0, 저장소 정리본 63종 중 빠짐 0. 횟수가 줄어든 것도 0
- 10-04: 작업 원본 281종 중 빠짐 0, 저장소 정리본 281종 중 빠짐 0
- 비교에서 뺀 것: 절 번호(제목의 "## 1." ~ "## 7.", 본문의 "5절" "1~4절" 같은 참조). 정리본이 절 번호를 없앤 구조라 값이 아니라고 봤다
  - 처음 비교(한글 조사가 붙은 토큰, 절 번호 포함)에서는 10-06 11건, 10-04 3건이 빠짐으로 나왔다.
    전부 절 번호이거나 "3곳이다" -> "3곳이" 같은 조사 차이였다. 판단 근거를 남기려고 로그에 절 번호 목록을 따로 찍었다
- 불일치 0 이라 되돌리지 않았다

## sync_to_repo.sh 수정 내역
- 보호 목록(PROTECT)과 관련 exclude·경고 출력을 지움. field_survey 2개가 단일 원본이 됐다
- 대상 추가: map/README.md -> 규민/README.md, maps 목록에 NOTICE.md(MAPS=(동결본 xodr, NOTICE.md)). ATTRIBUTION.md 는 docs/ 동기화에 들어감
- 토큰 검사 대상에 README.md 추가
- 머리 주석에 규칙(작업 폴더에서만 편집, --apply·git 은 사용자 지시 때만) 기록
- 기본 dry-run, --delete 없음, git 명령 없음은 그대로. bash -n 문법 검사만 했고 실행하지 않음

## 규칙 기록
- CLAUDE.md 에 "팀 저장소 공유" 절 추가, docs/setup_log.md 1줄
  1. 문서 편집은 campus_mobility_sim 에서만. 규민/·GitHub 웹 직접 편집 금지
  2. sync_to_repo.sh --apply 는 사용자 지시 때만. 기본 dry-run
  3. git add, commit, push 는 사용자 지시 때만. 세션 마감에 자동으로 넣지 않는다

## 하지 않은 것
- sync --apply, git add·commit·push, 저장소 파일 수정
- 숫자·좌표·ID 변경

## 마감
- frozen_v1 sha256 bf835cdfad0cea65 (불변)
- 새 파일: map_session29c_report.md, scripts/s29c_token_check.py, logs/s29c_token_check.log, map/README.md,
  docs/figures/ATTRIBUTION.md, maps/NOTICE.md
- 수정: docs/field_survey_2026-10-04.md·10-06.md(위 병합), scripts/sync_to_repo.sh, CLAUDE.md(절 추가), docs/setup_log.md(append)
- 다음 동기화 때 정방향으로 갈 것: field_survey 10-06(두 절), s24_reconv_list, reconversion_plan, s27_reconvert.py, 세션28·29 새 파일
