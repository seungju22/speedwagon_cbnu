"""10/15 5단계: 장면마다 정지 화면 두 장(처음 검출, 정지 순간)을 글자 넣은 프레임(logs/s59/lbl_cam_<tag>)에서 뽑는다.
- 처음 검출: 정답 객체 기록(perception/logs/s5x_obj_<tag>.csv)에서 대상 라벨이 처음 why=pub 인 CARLA frame
- 정지 순간: 가(ped)는 장면 기록 scn_<tag>.csv(매 틱 ego_v)에서 검출 뒤 처음 0.05m/s 밑. 나(door)는 검출과 상관없이 도착해 선 때) i5_compare_<tag>.csv 의 자차 속도 v 가 처음 0.05m/s 밑인 carla_frame. 문 구역(door)은 북문 목표 5m 안에서 처음 선 frame
- 프레임은 10fps 라 가장 가까운 앞쪽 프레임을 쓴다
사용: python3 docs/midterm_material/make_stills.py"""
import csv, glob, math, os, shutil
C = os.path.expanduser("~/campus_mobility_sim"); OUT = C + "/docs/midterm_material/stills"; os.makedirs(OUT, exist_ok=True)
GOAL = (656.162, 684.456)
SC = [("s58a_all", "s58", "PEDESTRIAN", "ped"), ("s58a_car", "s58", "PEDESTRIAN", "ped"), ("s58b_car", "s58", "PEDESTRIAN", "door"),
      ("s59a_car", "s59", "PEDESTRIAN", "ped"), ("s59b_car", "s59", "PEDESTRIAN", "door")]
rows = []
for tag, ses, lab, kind in SC:
    fr = sorted(int(os.path.basename(f)[:-4]) for f in glob.glob(f"{C}/integration/logs/s59/lbl_cam_{tag}/*.jpg"))
    det = next((int(r["frame"]) for r in csv.DictReader(open(f"{C}/perception/logs/{ses}_obj_{tag}.csv")) if r["label"] == lab and r["why"] == "pub"), None)
    cmp_ = [r for r in csv.DictReader(open(f"{C}/integration/logs/i5_compare_{tag}.csv")) if r["carla_frame"]]
    stop = None
    for r in cmp_:
        f = int(r["carla_frame"]); v = float(r["v"])
        if v >= 0.05 or (kind != "door" and (det is None or f < det)): continue
        if kind == "door" and math.dist((float(r["aw_x"]), float(r["aw_y"])), GOAL) > 5: continue
        stop = f; break
    if kind == "ped":   # 보행자 앞 정지는 짧아 1Hz 비교 기록에서 빠진다. 장면 기록(매 틱 ego_v)으로 찾고 wall 시각을 비교 기록으로 frame 에 옮긴다
        W = [(float(r["wall"]), int(r["carla_frame"])) for r in cmp_]
        def w2f(w):
            for (w0, f0), (w1, f1) in zip(W, W[1:]):
                if w0 <= w <= w1: return int(round(f0 + (w - w0) * (f1 - f0) / (w1 - w0)))
        sc = list(csv.DictReader(open(f"{C}/integration/logs/{ses}/scn_{tag}.csv")))
        stop = None
        for r in sc:
            f = int(r["frame"]) if "frame" in r else w2f(float(r["wall"]))
            if f is None or det is None or f < det or r["state"] == "wait": continue
            if float(r["ego_v"]) < 0.05: stop = f; break
    for ev, f in (("1_처음검출", det), ("2_정지", stop)):
        if f is None: rows.append((tag, ev, None, None)); continue
        pick = max([x for x in fr if x <= f] or fr[:1])
        dst = f"{OUT}/{tag}_{ev}.jpg"; shutil.copy(f"{C}/integration/logs/s59/lbl_cam_{tag}/{pick}.jpg", dst)
        rows.append((tag, ev, f, pick))
with open(OUT + "/stills.tsv", "w") as o:
    o.write("tag\tevent\tframe\tpicked_frame\n")
    for r in rows: o.write("\t".join(map(str, r)) + "\n")
for r in rows: print(*r)
