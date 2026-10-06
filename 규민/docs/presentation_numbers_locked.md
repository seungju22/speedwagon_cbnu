아래 값은 실측 및 검증을 마친 수치입니다.
발표 자료 작성 시 이 값을 그대로 사용하고,
반올림하거나 다시 계산하지 마십시오.
표기 자릿수도 그대로 유지하십시오.

# 발표 확정 숫자 카드 (2026-10-04, 세션26 감사 + 26-B 결정 반영)

형식: 항목 / 값 / 출처 파일. 출처 경로는 map/docs/ 기준. 정의가 붙은 값은 정의와 함께만 쓴다
맵 파일: map/maps/cbnu_internal_only_localtm_tags73_smooth.xodr, sha256 앞 16자 bf835cdfad0cea65
주행 조건(모든 주행 값 공통): 빈 도로·맑음·평지(z=0), CARLA 0.9.15, BasicAgent, 동기 20Hz(0.05s/tick), 목표 20km/h

## 1. 지도 규모
- road 수 / 878 / logs/session18_load.log
- junction 수 / 95 / logs/session18_load.log
- 차를 놓을 수 있는 위치(spawn point) / 1243 / logs/session18_load.log
- 실제 위치와의 축척 / 1.0007 / map_session09_curve_analysis.md 61행
  - 정의: OSM 보정점 90개와 맵 좌표의 유사변환 스케일(1.000709). 잔차 RMS 3.7m. "1.0007배 크다" 가 아니라 "축척 오차 0.07%"

## 2. 정류장 노선 (주행 궤적)
- 북문 거리 / 622.5m / map_session18_report.md 192행
- 북문 시간 / 123.1s / map_session18_report.md 192행
- 남문 거리 / 726.8m / map_session18_report.md 206행
- 남문 시간 / 143.8s / map_session18_report.md 206행
- 중문 거리 / 690.2m / map_session20_report.md 56행
- 중문 시간 / 136.8s / map_session20_report.md 56행
- 양성재 거리 / 633.9m / map_session20_report.md 69행
- 양성재 시간 / 125.6s / map_session20_report.md 69행
  - 거리 정의: 출발점(정문 밖 공도 교차점, road1247 시작)부터 실제 주행 궤적 누적. 캠퍼스 경계 밖 24m 포함
  - 시간 정의: 시뮬레이션 시간(실제 시계 시간 아님)
- 경로 합(설계값) 북문 / 남문 / 중문 / 양성재 / 622.9m / 730.0m / 693.7m / 637.4m / logs/s26_route_sums.log
  - 정의: 계획 road 길이 합 + 종점 s. 궤적과 차이 최대 3.5m(중문·양성재, 0.5~0.6%)
- 캠퍼스 경계 밖 구간 / 24m / logs/s26_gate_dist.log
  - 정의: 도로 중심선 기준(23.9m). 주행 차선 중심 기준이면 25.4m
- 출발점에서 OSM "정문" 노드까지 / 약 41m / logs/s26_gate_dist.log (40.8m)
- 평균 속도(4노선 모두) / 18.2km/h / logs/s26_route_sums.log
  - 정의: 궤적 거리 / 시뮬레이션 시간
- 최고 속도 / 23.1~24.0km/h / map_session18_report.md 192·206행, map_session20_report.md 56·69행
- 충돌(4노선) / 0 / map_session18_report.md, map_session20_report.md (collision CSV 헤더만)
- 완주 / 4/4 노선(조건 1종) = 15 조합 기준 4/15 / map_session20_report.md
- 정차 판정 거리 북문 / 남문 / 중문 / 양성재 / 0.93m / 0.92m / 0.90m / 0.96m / map_session18·20 보고서
  - 정의: 완주 판정 순간 정차점까지 거리. 판정을 정차점 1.0m 앞에 둬서 1m 를 넘을 수 없다. 정지 오차가 아니다
- 실제 정지 위치 오차 / 0.33~1.06m (3회 중 2회만 1.0m 이내) / map_session20_report.md 108~120행
  - 정의: 차가 완전히 선 위치와 정차점 거리. 3회 모두 정차점 앞에 섬
- 후문까지 지금 맵에서 갈 수 있는 가장 가까운 곳 / 217m / logs/session18_phase0b.log (217.2m)

## 3. 제동과 TTC (세 값을 섞지 말 것)
- 정지거리 / 3.78~4.00m (5회 평균 3.93m) / logs/session20_brake_hold3_cross_stdout.log
- 정지 시간 / 0.95~1.00s / logs/session20_brake_hold3_cross_stdout.log
  - 정의: 제동 명령(brake 1.0)부터 정지까지 실측 시간. 5회 중 1회는 같은 자리 반복이라 독립 위치 4곳
  - 제동 순간 속도 5.64~5.90m/s(20km/h 목표 주행 중). 명령 뒤 약 0.10s 동안 속도가 더 올라 6.23~6.41m/s
- 정상 구간 감속도 / 6.42m/s^2 / logs/session20_brake_hold3_cross_stdout.log (a_fit 5회 평균)
  - 정의: 제동 중 속도가 고르게 줄어드는 구간의 회귀 기울기
- TTC 기준 / 0.70s / map_session20_report.md 99행
  - 정의: 정지거리 / 제동 순간 속도. 거리 기준 지표. 제동 시간 아님. 회차별 비율의 최댓값(3.94 / 5.635)
  - 실제 정지에는 0.95~1.00s 가 걸린다
- TTC 기준을 5.56m/s 로 환산한 정지거리 / 3.89m / map_session21_report.md 31행
  - 정의: 0.70 x 5.56. 실측 평균이 아니다(실측 평균은 3.93m)
- 목표 TTC / 1.2s / map_session21_report.md 30~31행
- 목표 TTC 의 간격 / 6.67m / map_session21_report.md 31행 (1.2 x 5.56)
- 거리 여유 / 2.78m / map_session21_report.md 31행 (6.67 - 3.89)
- 허용 인지·판단 지연 / 0.50s / map_session21_report.md 31행 (2.78 / 5.56)
  - 정의: 제동 시작이 이만큼 늦어져도 정지거리 안에 선다는 거리 기준 값
- 목표 TTC / TTC 기준 / 1.7배 / map_session21_report.md 31행 (1.2 / 0.70 = 1.71)
- 시간 기준 여유 / 0.20~0.25s / presentation_2026-10-08.md 5절 11번 (1.2 - 1.00, 1.2 - 0.95)
  - 정의: 1.2s 거리의 정지물 앞에서 바로 제동할 때, 정지가 충돌 예상 시각보다 먼저 끝나는 시간. 허용 지연(0.50s)이 아니다

## 4. 연결성 (현재 맵, 재변환 전)
- 정문에서 도달 가능한 road / 826/878 = 94.1% / logs/s23_connectivity.md
  - 정의: 차선(방향) 기준. 못 가는 52 중 51은 이미 가는 길의 반대 방향 차선
- 주 순환망으로 돌아올 수 있는 road / 776/878 = 88.4% / logs/s23_connectivity.md
- 정문까지 왕복 가능한 road / 4/878 = 0.5% / logs/s23_connectivity.md
  - 원인: 정문 나가는 방향 way 가 변환에서 빠진 데이터 문제(2026-10-04 현장: 정문 차단기 없음)

## 5. 1m 벽 문제
- 세션10 충돌 이벤트 / 1,604건 / logs/collision_events_20260922_213844.csv (평활화 전 파일, 92m 정지)
- 세션12 충돌 이벤트 / 2,139건 / logs/collision_events_20260924_113337.csv (벽 1.0, 548m 정지)
- 세션13 이후 충돌 / 0건 / 같은 파일에서 벽만 끈 대조(wall_height 0), 이후 모든 주행 collision CSV 헤더만
  - 정의: 벽 대조 실험의 "전 -> 후" 는 2,139 -> 0. 1,604 는 다른 파일(평활화 전) 값
- 세션12 충돌 중 차를 왼쪽으로 민 것 / 2,138건 / map_session12_report.md 46행
- 스크린샷으로 본 벽 높이 / 0.94~1.08m / map_session12_report.md 168행
- 차 왼쪽 모서리가 중앙선을 넘은 양(92m 정지) / 0.19m / map_session11_report.md 51행

## 6. 진행 서사 거리 (각 단계 주행 궤적)
- 92m 정지 지점 / road1247 s≈92m / map_session10_report.md
- 세션12 정지 / 548m (547.7m) / map_session12_report.md 67행
- 세션13 / 554m (554.4m) / map_session13_report.md 45행
- 세션14 / 258.7m / map_session14_report.md 6행
- 세션15 1차 / 387.7m / map_session15_report.md 25행
- 세션15 도서관 경로 완주 / 궤적 774.4m, 153.2s / map_session15_report.md 95·112행
- 세션17 재현 / 궤적 774.4m, 153.4s / map_session17_report.md 47~48행
- 세션18 중문 실패 지점 / 437.9m / map_session18_report.md 224행

## 7. 차선 이탈 (차체 기준, 중앙선 쪽)
- 4노선 합 / 8회 (북문 1, 남문 2, 중문 3, 양성재 2) / logs/s19_replay_judge.log, map_session20_report.md 62·73행
  - 정의: 차체 옆면이 차선 왼쪽 경계(= 중앙선)를 넘은 구간 수. 차 중심이 넘은 것이 아니다
- 정문 커브 r1247 초과량 / 0.10~0.17m / logs/s19_replay_judge.log, map_session20_report.md 73행
- 중문 r1592 / 1.19m / map_session20_report.md 58행
- 중문 r1838 / 0.84m / map_session20_report.md 61행 (같은 로그 커브 요약은 0.81m. 보수적으로 0.84)
- 남문 r1428 / 0.61m / logs/s19_replay_judge.log

## 8. 그 밖
- 고원식 횡단보도 1곳당 추가 시간 / 0.9~2.0s / logs/s24_speedbump.md (세션25 정정 절)
  - 정의: 가정 포함 계산값. 감속도 6.42m/s^2(재가속 같은 값 가정), 통과 10km/h 가정, 고원식 길이 4~10m 가정. 실측 아님
- 속도 진동(정속 구간) / 4.1~6.3m/s (목표 5.56m/s 대비 -26%~+13%) / map_session20_report.md 86행, map_session21_report.md 45행
- 투영 오류 배율(고치기 전) / 1.2987 / map_session03_report.md 231행 (300쌍 평균. 이론값 1.297, 고친 뒤 0.9997)
- 태그 보정 way / 73 (16 + 3 + 54) / map_session04_report.md 286·295행
- 보정 전후 road / 134 -> 878 / map_session04_verification.md 4·18행

---

[발표에서 사용하지 말 것]
- 781m (도서관 경로 길이): 세션17 에서 폐기. 같은 경로는 궤적 774.4m 로 말한다
- 재변환 way 개수 (10, 9, 7, 2, 12 등 모든 개수): 현장 조사 반영해 확정 중. "후문 구간은 추가 조사 후 결정" 으로만 말한다
- 3.97m/s^2, 1.40s: 3.89m 를 등감속으로 역산한 값. 실측과 다르다
- 0.27s "반응 지연": 등가 환산값. 측정된 지연이 아니다
- 0.71s TTC: 출처 정의와 다른 계산(평균 / 명목 속도)
- "평균 3.89m": 3.89 는 환산값. 실측 평균은 3.93m
- "물리 최소 TTC 0.70s 에 정지한다": 0.70 은 제동 시간이 아니다
- 1.1~2.2s (고원식): 폐기된 3.97 기준. 0.9~2.0s 를 쓴다
- 1245 (spawn): 평활화 전 맵 값
- "벽 대조 1,604 -> 0": 같은 파일 대조는 2,139 -> 0
- 0.14~0.17m (정문 커브): 양성재 추가 전 범위. 0.10~0.17m 를 쓴다
- 후문 약 1,436m: 채택 안 된 시험 맵의 예측값이고 후문 way 는 보류 중. 미주행
- 커브 진입 속도 19.4km/h(r1592), 16.4km/h(r1838): 같은 로그에 다른 정의 값(21.4, 22.0km/h)이 있다. 쓰려면 "커넥터 진입 순간 속도" 라는 정의를 같이 말할 것
- "TM 이 U턴 커넥터 4곳을 모두 밟음": 확실한 것은 r1697 1곳. 나머지는 오탐일 수 있다
- "평균 반경 9m 가 통과 경계": 옛 판정 기준값. 세션20 에서 7.82m·5.73m 도 통과
- "유턴 연결로는 전부 같은 틀": 확인 범위는 junction 30여 개
- 세션18 중문 실패 지속 시간 0.30s / 0.35s: 정의 차이. 말할 필요가 있으면 "7 tick" 으로
