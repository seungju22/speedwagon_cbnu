# s28 Phase 4. 회차 채움(s27_turnfix write) 왕복 검증 (2026-10-07, 재변환 없음, 서버 없음)

스크립트 map/scripts/s28_turnfix_roundtrip.py, 로그 s28_turnaround_test.log
제거한 원문·기하·비교 결과 map/docs/logs/s28_turnaround_removed.json
테스트 파일: scratchpad 안 frozen_v1_test_copy / _test_removed_0·1 / _test_restored_0·1. 스크립트 끝에서 삭제, 남은 _test_ 파일 0
frozen_v1 sha256 시작 bf835cdfad0cea65, 끝 bf835cdfad0cea65 (불변)

## 방법
- 대상: frozen_v1 의 기존 회차 연결로 24개(connection 1개 junction, paramPoly3, 길이 9.89240086. s27 scan 정의)
  - j1 r1915·j107 r1916 도 회차 모양이지만 connection 2개 junction 안에 있어 대상이 아니다(write 도 이것을 만들지 않는다)
- write 는 같은 파일 안의 틀을 복제한다. 24개를 한 번에 지우면 틀이 0 이라 동작하지 않는다(코드: "틀 커넥터 0개: 채우지 않음")
  - 그래서 두 번 나눴다. 1회차 짝수 번째 12개 제거(남은 12가 틀), 2회차 홀수 번째 12개 제거. 합쳐서 24개 전부 시험
- 제거: 복사본 원문에서 해당 connectingRoad 의 road 요소와 junction 요소만 잘라냈다(서식 보존). 일반 road 의 link 는 그대로
  -> 그 junction 은 "정의 없는 junction" 이 된다. 재변환 맵에서 변환기가 회차를 못 만든 끝과 같은 모양
- 각 회차: write -> verify -> 원래 것과 비교

## 결과
개수
- 1회차·2회차 모두 제거 12 -> 복원 12. 복원 안 된 것 0. 합 24/24
- 두 회차 모두 원래 정의 없던 14개(j4 15 17 26 55 56 75 83 89 92 97 98 99 104)도 함께 채웠다 -> 회차마다 커넥터 26 추가
  - 이 14개는 비교할 원본이 없다. verify(next() 이음)만 통과
- 제거본 road 866 / junction 83 -> 복원본 road 892 / junction 109 (866 + 26, 83 + 26)

연결 관계
- 24/24 같음: predecessor·successor road, incomingRoad, contactPoint, laneLink(-1>-1, 인도 있으면 -2>-2), 차선 구성, laneOffset
- verify 두 회차 모두 실패 0 (들어가는 반쪽 -> 커넥터 -> 나가는 반쪽 next() 이음)

기하 (24개 최대값)
- 시작점 x,y 차: 0.000096m (평균 0.000033m)
- 방위 차: 최대 3.3e-07rad (j80)
- 길이 차: 0 (9.89240086 그대로 복제)
- paramPoly3 계수 차: 0 (원래 24개 계수가 서로 같다)
- 곡률 최대값: 원래·복원 모두 0.59701 1/m(반지름 약 1.68m, 계산값), 차 0
- 차선 중심 표본(커넥터 lane -1, 21점, carla.Map 로 계산) 차: 최대 0.000126m (평균 0.000044m)
- 허용 오차를 미리 정하지 않았다. 차이는 mm 아래이고 xodr 기록 자릿수(소수 8자리)와 틀 상대 위치 평균값 반올림으로 설명된다(추정)

파싱
- 복원본 두 개 모두 ElementTree 파싱 가능, carla.Map 오프라인 로드 가능
- 서버 로드는 하지 않았다(지시문 금지)

## 판정
- write/verify 는 frozen_v1 기준 왕복에서 원래 회차를 개수·연결·기하 모두 재현한다. 세션27 의 "미검증" 을 "frozen_v1 왕복 검증됨" 으로 바꾼다
- 완전 일치가 아닌 항목: 시작점 0.0001m, 방위 3.3e-07rad, 표본 0.0001m. road id 는 다르다(새 id = 최대 id + 1 부터)

## 이 시험이 보장하지 않는 것
- 재변환 맵에 틀이 하나도 없으면 write 는 멈춘다(설계상). 재변환 맵 scan 의 "틀 N개" 를 먼저 본다
- 틀 상대 위치 폭이 0.01 을 넘으면 멈춘다(설계상). 이번 두 회차 폭은 0.0002m 이하
- 짝 road 가 1:1 이 아닌 끝, 차선 간격 3.35m·방향차 180도가 아닌 끝은 "채움가능=False" 로 건너뛴다. 이번 시험에는 그런 끝이 없어
  건너뛰기 경로는 시험되지 않았다
- 회차 연결로 자체의 결함(이음매 0.77m, 26.6도, s23)은 원래 것과 같게 복제된다. 고치는 도구가 아니다
- road id 가 바뀌므로, 회차 id 를 쓰는 노선(test_drive.py uturn 의 r1565)은 재변환 뒤 다시 번역해야 한다
