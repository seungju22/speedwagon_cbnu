# 맵세션9 Phase A 급커브 가설 검증 (읽기전용, 자동생성)

## A-1. 통과 커브(s=80~85) vs 정지 커브(s=93~96)
- CSV: drive_log_20260919_231354.csv, road1247 행수=5815

### 통과 구간 s=80~85
- s=80.0~80.5 dyaw=+6.58deg rate=+13.15deg/m radius=4.36m speed~=5.29m/s(t=16.38s)
- s=80.5~81.0 dyaw=+3.90deg rate=+7.81deg/m radius=7.34m speed~=5.29m/s(t=16.50s)
- s=81.0~81.5 dyaw=+2.37deg rate=+4.73deg/m radius=12.10m speed~=5.29m/s(t=16.59s)
- s=81.5~82.0 dyaw=+1.34deg rate=+2.67deg/m radius=21.44m speed~=5.30m/s(t=16.73s)
- s=82.0~82.5 dyaw=+0.50deg rate=+1.00deg/m radius=57.55m speed~=5.29m/s(t=16.83s)
- s=82.5~83.0 dyaw=-0.38deg rate=-0.75deg/m radius=76.05m speed~=5.29m/s(t=16.94s)
- s=83.0~83.5 dyaw=-1.60deg rate=-3.20deg/m radius=17.91m speed~=5.29m/s(t=17.03s)
- s=83.5~84.0 dyaw=-2.78deg rate=-5.55deg/m radius=10.32m speed~=5.30m/s(t=17.13s)
- s=84.0~84.5 dyaw=-1.74deg rate=-3.48deg/m radius=16.44m speed~=5.30m/s(t=17.20s)
- s=84.5~85.0 dyaw=-1.28deg rate=-2.55deg/m radius=22.44m speed~=5.29m/s(t=17.29s)
- **최소 곡률반경(통과)**: 4.36m (s=80.0~80.5, rate=13.15deg/m)

### 정지 구간 s=93~96
- s=93.0~93.5 dyaw=+2.79deg rate=+5.58deg/m radius=10.26m speed~=0.06m/s(t=25.16s)
- s=93.5~94.0 dyaw=+3.36deg rate=+6.73deg/m radius=8.52m speed~=0.06m/s(t=25.16s)
- s=94.0~94.5 dyaw=+4.07deg rate=+8.14deg/m radius=7.04m speed~=0.06m/s(t=25.16s)
- s=94.5~95.0 dyaw=+4.93deg rate=+9.86deg/m radius=5.81m speed~=0.06m/s(t=25.16s)
- s=95.0~95.5 dyaw=+5.96deg rate=+11.92deg/m radius=4.81m speed~=0.06m/s(t=25.16s)
- s=95.5~96.0 dyaw=+2.10deg rate=+4.20deg/m radius=13.64m speed~=0.06m/s(t=25.16s)
- **최소 곡률반경(정지)**: 4.81m (s=95.0~95.5, rate=11.92deg/m)

- 비교: 정지구간 반경/통과구간 반경 = 1.103
- 참고용 rate 값: 통과 13.15deg/m, 정지 11.92deg/m

## A-2. 확정경로 15 road 급커브 전수
- 기준: rate>=6.0deg/m (30deg/5m 환산과 동일)
- 급커브 구간 수: 6 (plain road 3 / connector 3)
- road1247 [plain] s=79.5~81.0 총꺾임=+16.4deg 구간길이=1.50m 최소반경=4.36m
- road1247 [plain] s=93.5~95.5 총꺾임=+18.3deg 구간길이=2.00m 최소반경=4.81m
- road1356 [plain] s=11.5~12.0 총꺾임=+4.2deg 구간길이=0.50m 최소반경=6.76m
- road1564 [connector(junction36)] s=1.0~9.0 총꺾임=+58.0deg 구간길이=8.00m 최소반경=7.22m
- road1428 [connector(junction10)] s=0.0~5.0 총꺾임=+33.8deg 구간길이=5.00m 최소반경=8.16m
- road1446 [connector(junction42)] s=2.0~8.5 총꺾임=-43.0deg 구간길이=6.50m 최소반경=8.25m

## A-3. road1247 원본 OSM 대응
- road1247 predecessor=junction2(OSM노드 2261340221), successor=junction1(OSM노드 4748296080)
- OSM 노드 체인 길이: 8개 노드
  - 노드2261340221 lat=36.632650 lon=127.453018 간격=0.00m 누적=0.00m
  - 노드3957495734 lat=36.632164 lon=127.453047 간격=54.18m 누적=54.18m
  - 노드3957495733 lat=36.631933 lon=127.453078 간격=25.79m 누적=79.97m
  - 노드4748296079 lat=36.631900 lon=127.453096 간격=4.07m 누적=84.04m
  - 노드3957495732 lat=36.631802 lon=127.453148 간격=11.78m 누적=95.82m
  - 노드4748296078 lat=36.631652 lon=127.453382 간격=26.77m 누적=122.59m
  - 노드3957495731 lat=36.631553 lon=127.453535 간격=17.56m 누적=140.16m
  - 노드4748296080 lat=36.631513 lon=127.453602 간격=7.48m 누적=147.64m

### 노드간 방위각 변화
  - 노드3957495734(누적54.2m): 진입방위=-87.2deg 진출방위=-83.7deg 꺾임=+3.5deg
  - 노드3957495733(누적80.0m): 진입방위=-83.7deg 진출방위=-67.0deg 꺾임=+16.7deg
  - 노드4748296079(누적84.0m): 진입방위=-67.0deg 진출방위=-67.1deg 꺾임=-0.1deg
  - 노드3957495732(누적95.8m): 진입방위=-67.1deg 진출방위=-38.7deg 꺾임=+28.5deg
  - 노드4748296078(누적122.6m): 진입방위=-38.7deg 진출방위=-38.7deg 꺾임=-0.0deg
  - 노드3957495731(누적140.2m): 진입방위=-38.7deg 진출방위=-36.9deg 꺾임=+1.8deg

### 유사변환 적합(참고): 스케일=1.000709 회전=0.065deg 잔차RMS=3.732m (보정점 90개)
- xodr s=93.0 -> 변환위경도 lat=36.631806 lon=127.453116 -> 최근접 OSM노드 3957495732(누적95.8m) 거리2.86m
- xodr s=96.0 -> 변환위경도 lat=36.631787 lon=127.453134 -> 최근접 OSM노드 3957495732(누적95.8m) 거리2.01m