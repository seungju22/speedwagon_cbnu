#!/usr/bin/env python3
# 맵세션28 Phase 4: s27_turnfix.py write 왕복 검증 (재변환 없음, 서버 없음).
# 1 frozen_v1 을 scratchpad 로 복사(파일명에 _test_). frozen_v1 은 읽기만
# 2 복사본에서 기존 회차 연결로(틀 24개 = connection 1개 junction 의 paramPoly3·9.89m 커넥터)를 제거
#    write 는 같은 파일 안의 틀을 복제하므로 24개를 한 번에 다 지우면 틀이 0 이 되어 동작하지 않는다.
#    그래서 두 번 나눈다: 1회차 짝수 번째 12개 제거(홀수 12개가 틀), 2회차 홀수 번째 12개 제거
#    제거한 road·junction 원문과 기하는 JSON 으로 저장(map/docs/logs/s28_turnaround_removed.json)
# 3 s27_turnfix.write 를 제거본에 실행 -> 복원본, s27_turnfix.verify
# 4 복원된 것과 원래 것 비교: 개수, 연결(들어옴·나감·laneLink·contactPoint·인도 차선), 기하(시작점·방위·길이·
#    paramPoly3 계수·곡률·차선 중심 표본점), carla.Map 로드 가능 여부
# 5 테스트 파일 삭제는 이 스크립트 끝에서 한다(_test_ 파일만, scratchpad 안만)
# 사용: python s28_turnfix_roundtrip.py <scratchpad 디렉터리>
import hashlib
import json
import math
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import carla

sys.path.insert(0, str(Path(__file__).resolve().parent))
import s27_turnfix as tf  # noqa: E402

FROZEN = tf.FROZEN
LOGS = Path(__file__).resolve().parents[1] / "docs/logs"
SAMPLE_N = 21                      # 커넥터 lane -1 중심 표본 수(0~L 균등)


def sha16(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()[:16]


def templates(root):
    roads = {int(r.get("id")): r for r in root.iter("road")}
    out = []
    for j in root.iter("junction"):
        cs = j.findall("connection")
        if len(cs) != 1:
            continue
        cr = roads[int(cs[0].get("connectingRoad"))]
        g = cr.find("planView/geometry")
        if g.find("paramPoly3") is None or abs(float(cr.get("length")) - tf.TPL_LEN) > tf.TPL_TOL:
            continue
        out.append((int(j.get("id")), int(cr.get("id"))))
    return sorted(out)


def conn_info(root, jid):
    """junction jid 의 커넥터 1개 정보(비교용)."""
    roads = {int(r.get("id")): r for r in root.iter("road")}
    j = [x for x in root.iter("junction") if int(x.get("id")) == jid][0]
    c = j.find("connection")
    r = roads[int(c.get("connectingRoad"))]
    g = r.find("planView/geometry")
    p = g.find("paramPoly3")
    lk = tf.links(r)
    return dict(junction=jid, road=int(r.get("id")), name=r.get("name"), length=float(r.get("length")),
                x=float(g.get("x")), y=float(g.get("y")), hdg=float(g.get("hdg")),
                poly={k: float(p.get(k)) for k in ("aU", "bU", "cU", "dU", "aV", "bV", "cV", "dV")},
                pred=lk["predecessor"], succ=lk["successor"], incoming=int(c.get("incomingRoad")),
                contact=c.get("contactPoint"),
                lanelinks=sorted((ll.get("from"), ll.get("to")) for ll in c.findall("laneLink")),
                lanes=sorted(int(ln.get("id")) for ln in r.iter("lane")),
                laneoffset=r.find("lanes/laneOffset").get("a"))


def curvature_max(poly, L):
    """paramPoly3(normalized) 곡률 최대 |k| (1/m). u(p), v(p), p in [0,1] 표본 201."""
    best = 0.0
    for i in range(201):
        t = i / 200
        du = poly["bU"] + 2 * poly["cU"] * t + 3 * poly["dU"] * t * t
        dv = poly["bV"] + 2 * poly["cV"] * t + 3 * poly["dV"] * t * t
        ddu = 2 * poly["cU"] + 6 * poly["dU"] * t
        ddv = 2 * poly["cV"] + 6 * poly["dV"] * t
        k = abs(du * ddv - dv * ddu) / max((du * du + dv * dv) ** 1.5, 1e-12)
        best = max(best, k)
    return best


def samples(cmap, rid, L):
    pts = []
    for i in range(SAMPLE_N):
        wp = cmap.get_waypoint_xodr(rid, -1, min(max(L * i / (SAMPLE_N - 1), 0.001), L - 0.001))
        pts.append(None if wp is None else (wp.transform.location.x, wp.transform.location.y))
    return pts


def strip(text, jids, rids):
    """원문 텍스트에서 road·junction 요소만 잘라낸다(서식 보존). 잘라낸 원문을 돌려준다."""
    cut = {}
    for rid in rids:
        m = re.search(r'<road [^>]*\sid="%d"[^>]*>.*?</road>\n?' % rid, text, re.S)
        cut[f"road{rid}"] = m.group(0)
        text = text[:m.start()] + text[m.end():]
    for jid in jids:
        m = re.search(r'[ \t]*<junction [^>]*\sid="%d"[^>]*>.*?</junction>\n?' % jid, text, re.S)
        cut[f"junction{jid}"] = m.group(0)
        text = text[:m.start()] + text[m.end():]
    return text, cut


def main():
    sp = Path(sys.argv[1])
    print(f"frozen_v1 시작 sha {sha16(FROZEN)}")
    copy = sp / "frozen_v1_test_copy.xodr"
    shutil.copyfile(FROZEN, copy)
    print(f"복사 {copy.name} sha {sha16(copy)} (frozen_v1 과 같아야 함)")
    orig_text = copy.read_text()
    orig_root = ET.fromstring(orig_text.encode())
    tpls = templates(orig_root)
    print(f"원래 틀 회차 {len(tpls)}개: " + " ".join(f"j{j}/r{r}" for j, r in tpls))
    orig = {j: conn_info(orig_root, j) for j, _ in tpls}
    orig_map = carla.Map("orig", orig_text)
    removed_all, rows, test_files = {}, [], [copy]
    for half in (0, 1):
        pick = [t for i, t in enumerate(tpls) if i % 2 == half]
        text, cut = strip(orig_text, [j for j, _ in pick], [r for _, r in pick])
        rm = sp / f"frozen_v1_test_removed_{half}.xodr"
        out = sp / f"frozen_v1_test_restored_{half}.xodr"
        for f in (rm, out):
            if f.exists():
                f.unlink()
        rm.write_text(text)
        test_files += [rm, out]
        for k, v in cut.items():
            removed_all[k] = v
        print(f"\n## {half + 1}회차: 제거 {len(pick)}개 " + " ".join(f"j{j}/r{r}" for j, r in pick))
        rroot = ET.fromstring(text.encode())
        print(f"제거본 road {len(list(rroot.iter('road')))} / junction {len(list(rroot.iter('junction')))}, 남은 틀 {len(templates(rroot))}")
        print("[scan + write]")
        tf.write(rm, out)
        print("[verify]")
        ok = tf.verify(out)
        print(f"verify {'통과' if ok else '실패'}")
        out_text = out.read_text()
        oroot = ET.fromstring(out_text.encode())
        try:
            omap = carla.Map("restored", out_text)
            loaded = True
        except Exception as e:  # noqa: BLE001
            loaded = False
            print(f"carla.Map 로드 실패: {e}")
        print(f"복원본 road {len(list(oroot.iter('road')))} / junction {len(list(oroot.iter('junction')))}, carla.Map 로드 {'가능' if loaded else '불가'}")
        added = [r for r in oroot.iter("road") if (r.get("name") or "").startswith(tf.NAME_PREFIX)]
        added_j = sorted(int(r.get("junction")) for r in added)
        extra = [j for j in added_j if j not in {p[0] for p in pick}]
        print(f"추가된 커넥터 {len(added)}: 제거했던 junction {len(added_j) - len(extra)} + 원래 정의 없던 junction {len(extra)} {extra}")
        missing = [j for j, _ in pick if j not in added_j]
        print(f"제거했는데 복원 안 된 junction {len(missing)}: {missing}")
        for j, r in pick:
            if j in missing:
                continue
            a, b = orig[j], conn_info(oroot, j)
            d_xy = math.hypot(a["x"] - b["x"], a["y"] - b["y"])
            d_h = abs((a["hdg"] - b["hdg"] + math.pi) % (2 * math.pi) - math.pi)
            d_poly = max(abs(a["poly"][k] - b["poly"][k]) for k in a["poly"])
            ka, kb = curvature_max(a["poly"], a["length"]), curvature_max(b["poly"], b["length"])
            same_link = (a["pred"] == b["pred"] and a["succ"] == b["succ"] and a["incoming"] == b["incoming"]
                         and a["contact"] == b["contact"] and a["lanelinks"] == b["lanelinks"] and a["lanes"] == b["lanes"]
                         and a["laneoffset"] == b["laneoffset"])
            pa = samples(orig_map, a["road"], a["length"])
            pb = samples(omap, b["road"], b["length"]) if loaded else [None] * SAMPLE_N
            dp = [math.hypot(u[0] - v[0], u[1] - v[1]) for u, v in zip(pa, pb) if u and v]
            row = dict(half=half + 1, junction=j, orig_road=r, new_road=b["road"], link_same=same_link,
                       d_xy=d_xy, d_hdg=d_h, d_len=abs(a["length"] - b["length"]), d_poly=d_poly,
                       kmax_orig=ka, kmax_new=kb, d_sample_max=max(dp) if dp else None, n_sample=len(dp),
                       lanes_orig=a["lanes"], lanes_new=b["lanes"], pred=a["pred"], succ=a["succ"])
            rows.append(row)
            print(f"  j{j}: r{r} -> r{b['road']} 연결 {'같음' if same_link else '다름'} 시작점 {d_xy:.4f}m 방위 {d_h:.2e}rad"
                  f" 길이 {row['d_len']:.2e}m 계수 {d_poly:.2e} 곡률max {ka:.5f}/{kb:.5f} 표본최대 "
                  + (f"{row['d_sample_max']:.4f}m({len(dp)}점)" if dp else "없음"))
            if not same_link:
                print(f"    원래 pred {a['pred']} succ {a['succ']} in {a['incoming']} {a['lanelinks']} lanes {a['lanes']} off {a['laneoffset']}")
                print(f"    복원 pred {b['pred']} succ {b['succ']} in {b['incoming']} {b['lanelinks']} lanes {b['lanes']} off {b['laneoffset']}")
    print("\n## 합계")
    print(f"비교 {len(rows)} / 원래 {len(tpls)}")
    print(f"연결 같음 {sum(r['link_same'] for r in rows)}")
    for k, nm in (("d_xy", "시작점(m)"), ("d_hdg", "방위(rad)"), ("d_len", "길이(m)"), ("d_poly", "계수"),
                  ("d_sample_max", "차선 중심 표본(m)")):
        v = [r[k] for r in rows if r[k] is not None]
        print(f"차이 {nm}: 최대 {max(v):.6f} 평균 {sum(v) / len(v):.6f}")
    kk = [abs(r["kmax_orig"] - r["kmax_new"]) for r in rows]
    print(f"곡률 최대값 차: 최대 {max(kk):.2e} 1/m (원래 곡률max 범위 {min(r['kmax_orig'] for r in rows):.5f}~{max(r['kmax_orig'] for r in rows):.5f})")
    (LOGS / "s28_turnaround_removed.json").write_text(json.dumps(
        dict(source="frozen_v1 bf835cdfad0cea65", note="Phase 4 에서 테스트 복사본에서 제거한 회차 연결로 원문과 비교 결과",
             templates=[dict(junction=j, road=r, **{k: v for k, v in orig[j].items() if k not in ("junction", "road")})
                        for j, r in tpls],
             removed_text=removed_all, compare=rows), ensure_ascii=False, indent=1))
    print(f"저장 {LOGS / 's28_turnaround_removed.json'}")
    for f in test_files:
        if f.exists() and "_test_" in f.name and f.parent == sp:
            f.unlink()
    print(f"테스트 파일 삭제: 남은 _test_ 파일 {len(list(sp.glob('*_test_*')))}")
    print(f"frozen_v1 끝 sha {sha16(FROZEN)}")


if __name__ == "__main__":
    main()
