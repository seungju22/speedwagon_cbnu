#!/usr/bin/env python3
# 10/11: 바뀐 road 의 전후 그림과 이웃 차선 간격 확인(서버 없음)
# - 그림: 바뀐 road 주변 반경 R m, 옛(회색 점선)·새(빨강) 차선 중심과 가장자리, 다른 road 는 파랑(일반)·회색(연결로)
# - 간격: 바뀐 road 의 새 차선 중심에서 '이어지지 않은' 다른 road 차선 중심까지 최소 거리(전후). 3.35m(차선 폭) 미만이면 차선이 겹친다
#   같은 junction 의 연결로·앞뒤로 이어진 road 는 뺀다(교차로 안에서는 원래 겹침)
# 사용: .venv-carla/bin/python map/scripts/s53_check.py <old.xodr> <new.xodr> <tag> [road ...]
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
BASE = HERE.parent
sys.path.insert(0, str(HERE))
import s53_geom as G  # noqa: E402


def linked(roads, juncs, a, b):
    """a·b 가 실제로 이어졌나: 직접 road 링크, 연결로와 그 들어오는/나가는 road, 같은 junction 의 연결로끼리.
    같은 junction 에 들어오는 일반 road 둘(예: 양방향 길의 짝 반쪽)은 이어진 것으로 보지 않는다"""
    ra, rb = roads[a], roads[b]
    if ra.junction != -1 and ra.junction == rb.junction:
        return True
    for r, o in ((ra, b), (rb, a)):
        for lk in (r.pred, r.succ):
            if lk and lk[0] == "road" and lk[1] == o:
                return True
    for c, o in ((ra, b), (rb, a)):
        if c.junction != -1:
            for lk in (c.pred, c.succ):
                if lk and lk[0] == "road" and lk[1] == o:
                    return True
    return False


def min_gap(roads, juncs, rid, step=0.5):
    from shapely.geometry import LineString, Point
    me = [(x, y) for s, x, y, *_ in roads[rid].samples(step)]
    box = LineString(me).buffer(12)
    best = (math.inf, None)
    for oid, o in roads.items():
        if oid == rid or linked(roads, juncs, rid, oid):
            continue
        pts = [(x, y) for s, x, y, *_ in o.samples(1.0)]
        if not any(box.contains(Point(p)) for p in pts):
            continue
        ol = LineString([(x, y) for s, x, y, *_ in o.samples(step)])
        d = min(ol.distance(Point(p)) for p in me)
        if d < best[0]:
            best = (d, oid)
    return best


def main():
    old_x, new_x, tag = sys.argv[1], sys.argv[2], sys.argv[3]
    _, ro, jo = G.load(old_x)
    _, rn, jn = G.load(new_x)
    rids = [int(a.lstrip("r")) for a in sys.argv[4:]] or [r for r in rn if r in ro and rn[r].geos != ro[r].geos]
    out = []
    for rid in rids:
        go, gn = min_gap(ro, jo, rid), min_gap(rn, jn, rid)
        out.append(f"r{rid}: 이어지지 않은 이웃 차선 중심까지 최소 {go[0]:.2f}m(r{go[1]}) -> {gn[0]:.2f}m(r{gn[1]})"
                   + ("  [3.35m 미만: 차선 겹침]" if gn[0] < 3.35 and gn[0] < go[0] - 0.05 else ""))
    print("\n".join(out))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.font_manager
    import matplotlib.pyplot as plt
    f = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
    matplotlib.font_manager.fontManager.addfont(f)
    plt.rcParams["font.family"] = ["Noto Sans CJK JP"]
    for rid in rids:
        cx, cy = rn[rid].lane_at(rn[rid].length / 2)[:2]
        R = max(20.0, min(60.0, rn[rid].length * 0.7))
        fig, ax = plt.subplots(figsize=(8, 8))
        for roads, st, col_fn in ((ro, ":", lambda r: "#888"), (rn, "-", lambda r: "#d1495b" if r == rid else None)):
            for oid, o in roads.items():
                sm = o.samples(0.25)
                if not any(abs(x - cx) < R and abs(y - cy) < R for s, x, y, *_ in sm):
                    continue
                if roads is ro and oid != rid:
                    continue
                col = col_fn(oid) or ("#2a78d6" if o.junction == -1 else "#9a9a9a")
                ax.plot([p[1] for p in sm], [p[2] for p in sm], st, color=col, lw=1.3 if oid == rid else 0.9)
                for sg in (1, -1):
                    e = [(x - sg * o.width / 2 * math.sin(h), y + sg * o.width / 2 * math.cos(h)) for s, x, y, h, k, fo in sm]
                    ax.plot([p[0] for p in e], [p[1] for p in e], st, color=col, lw=0.4, alpha=0.7)
                if roads is rn:
                    ins = [q for q in sm if abs(q[1] - cx) < R * 0.95 and abs(q[2] - cy) < R * 0.95]
                    if ins:
                        m = ins[len(ins) // 2]
                        ax.text(m[1], m[2], str(oid), fontsize=7, color=col, clip_on=True)
        ax.set_xlim(cx - R, cx + R)
        ax.set_ylim(cy - R, cy + R)
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        ax.set_title(f"r{rid} 전(회색 점선) 후(빨강), xodr 좌표(CARLA y = -y)")
        fig.tight_layout()
        p = BASE / f"docs/figures/s53_{tag}_r{rid}.png"
        fig.savefig(p, dpi=100)
        plt.close(fig)
        print("그림", p.relative_to(BASE))


if __name__ == "__main__":
    main()
