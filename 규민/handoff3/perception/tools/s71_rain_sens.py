"""10/26 비 민감도. Goodin 2019 식 8·4 의 모델 최대 거리를 확산 반사율 r 별로 다시 셈.
식 3 의 rho 는 후방 산란 계수라 람베르트 표면이면 rho = r / pi. 10/25 은 rho = 0.07 을 그대로 넣었으니 r = 0.07 pi = 22%.
최대 거리 z: rho / z^2 e^(-0.02 R^0.6 z) = Pmin, Pmin = 0.9 / (pi zmax^2), zmax 100m. 이분법으로 풂.
정지 거리 = v * 1s + v^2 / (2 * 1.0 m/s2)(perception_demo.md S1 기준과 같은 식)"""
import math
PMIN = 0.9 / (math.pi * 100.0 ** 2)
def zmax(rho, R):
    f = lambda z: rho / z ** 2 * math.exp(-0.02 * R ** 0.6 * z) - PMIN
    lo, hi = 0.1, 1000.0
    for _ in range(200):
        m = (lo + hi) / 2
        lo, hi = (m, hi) if f(m) > 0 else (lo, m)
    return lo
def stop(kmh): v = kmh / 3.6; return v * 1.0 + v * v / 2.0
print("rho 0.07 그대로(세션70):", " ".join("%.2f" % zmax(0.07, R) for R in (0, 9, 25)))
for r in (0.22, 0.10, 0.07):
    rho = r / math.pi
    print("r %.2f rho %.4f: 맑음 %.1f 9mm/h %.1f 25mm/h %.1f" % ((r, rho) + tuple(zmax(rho, R) for R in (0, 9, 25))))
print("r = 0.07 pi = %.4f" % (0.07 * math.pi))
for k in (15, 12, 10): print("정지 거리 %d km/h %.2f m" % (k, stop(k)))
