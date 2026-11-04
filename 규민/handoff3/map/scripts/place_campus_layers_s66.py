#!/usr/bin/env python3
# 10/23 새 판: place_campus_layers.py(동결 v3.1 묶음, 8b067484)는 안 고치고, 그 함수를 불러 레이어 둘을 더함
#   buildings2: 건물 2순위 11동(data/buildings_layout_tier2_s66.csv, s66_b2_tier2.py). 표현 나(정육면체 덩어리)
#   trees:      가로수·풀숲(data/trees_layout_s66.csv, s66_trees.py). static.prop.mesh 나무 메시, 바닥 = 수관 원
# 레이어는 --layers 로 하나씩 켜고 끔. 기본값(buildings,busstop)·다른 인자는 원래 스크립트와 같고, 기본 출력도 같음(10/23 dry-run 비교)
# 정류장 자리 고를 때 피하는 상자는 원래와 같이 1순위만 씀(새 레이어를 켜도 정류장 자리가 안 바뀌게).
#   대신 새 레이어 배치물이 정류장 시설 바닥과 겹치면 그 배치물을 뺌
# 사용 예
#   --dry-run : .venv-carla/bin/python map/scripts/place_campus_layers_s66.py --layers buildings,busstop,buildings2,trees --repr 나 --xodr map/maps/cbnu_campus_frozen_v31.xodr --dry-run --tag s66_all
#   서버      : 위에서 --dry-run 대신 --state <파일> [--measure]. 지우기 --remove --state <파일>
#   레이어 판(10/23): 인자 없이 = v2(978), --set v1 = 732. 예 .venv-carla/bin/python map/scripts/place_campus_layers_s66.py --state <파일>
import argparse
import csv
import hashlib
import json
import random
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import place_campus_layers as pcl  # noqa: E402

LAYOUT2 = pcl.BASE / "data/buildings_layout_tier2_s66.csv"
TREES = pcl.BASE / "data/trees_layout_s66.csv"
LAYERS = pcl.LAYERS + ["buildings2"]
# 10/23: 레이어 판. --layers 를 안 주면 --set 판(기본 v2)의 레이어·지면 메시·메시 크기·xodr 를 씀(따로 준 인자가 먼저).
#   v1 = 10/11~10/23 회차에서 쓴 732(나 + 정류장 + 지면 판 Plane), v2 = v1 + 건물 2순위 + 나무(978, 10/23 판정으로 기본 켬)
#   --layers 를 주면 10/23 판과 똑같이 돎(판 이름은 '직접'). 판 이름은 상태 파일·dry-run JSON 의 layer_set 에 남김
SETS = {"v1": ["buildings", "busstop", "ground"], "v2": ["buildings", "busstop", "ground", "buildings2", "trees"]}
SET_ARGS = dict(ground_mesh="Plane", mesh_sizes=pcl.BASE / "docs/logs/s51_bench/mesh_sizes.json",
                xodr=pcl.BASE / "maps/cbnu_campus_frozen_v31.xodr")


def building2_actors(rows, frame):
    acts = []
    for r in rows:
        for a in pcl.building_actors(r, "나", frame):
            a["layer"] = "buildings2"
            acts.append(a)
    return acts


def tree_actors(path, frame):
    from shapely.geometry import Point
    acts = []
    for r in csv.DictReader(open(path)):
        x, y = frame.carla(float(r["lat"]), float(r["lon"]))
        acts.append(dict(layer="trees", owner=r["id"], kind="tree", bp="static.prop.mesh", mesh_path=r["mesh_path"],
                         scale=float(r["scale"]), x=x, y=y, z=0.0, yaw=float(r["yaw_deg"]),
                         foot=Point(x, y).buffer(float(r["crown_r_m"]), 32), foot_note=f"수관 원 반경 {r['crown_r_m']}m ({r['mesh']})"))
    return acts


def main():
    ap = argparse.ArgumentParser(description="캠퍼스 장식 레이어 배치(세션66 판: buildings2·trees 추가)")
    ap.add_argument("--set", choices=sorted(SETS), default="v2", help="레이어 판(--layers 를 안 줄 때만). v1 732, v2 978")
    ap.add_argument("--layers", default=None, help=f"쉼표 목록 {LAYERS}. 주면 --set 대신 이것(세션66 판 기본은 buildings,busstop)")
    ap.add_argument("--repr", choices=["가", "나", "다"], default="나",
                    help="건물 표현(기본 나, 세션50 결정). 다 는 메시 실제 크기를 잰 뒤 다시 비교")
    ap.add_argument("--mesh-sizes", type=Path, default=None, help="layers_bench.py 가 잰 메시 크기 JSON")
    ap.add_argument("--tag", default=None, help="--dry-run 출력 이름(기본: 레이어·표현에서 만듦. 세션49 파일을 덮지 않게 줄 것)")
    ap.add_argument("--ground-mesh", choices=sorted(pcl.GROUND_MESHES), default=None, help=f"ground 레이어 메시(기본 {pcl.GROUND})")
    ap.add_argument("--seed", type=int, default=pcl.SEED, help="무작위 배치 레이어용(이번 구현 레이어는 쓰지 않음)")
    ap.add_argument("--xodr", type=Path, default=None, help="기본: --set 판이면 v3.1 동결본, --layers 를 주면 v3(세션66 판과 같음)")
    ap.add_argument("--layout", type=Path, default=pcl.LAYOUT)
    ap.add_argument("--layout2", type=Path, default=LAYOUT2, help="buildings2 레이어 layout(2순위)")
    ap.add_argument("--trees-layout", type=Path, default=TREES, help="trees 레이어 목록")
    ap.add_argument("--stops", type=Path, default=pcl.STOPS)
    ap.add_argument("--state", type=Path, default=pcl.STATE)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--remove", action="store_true")
    ap.add_argument("--measure", action="store_true", help="서버: 소환 전과 레이어마다 100틱 ms/틱(렌더 켬·끔)·avail 기록")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    args = ap.parse_args()
    if args.remove:
        return pcl.remove(args)
    if args.layers is None:
        layer_set = args.set
        args.layers = ",".join(SETS[args.set])
        for k, v in SET_ARGS.items():
            if getattr(args, k) is None:
                setattr(args, k, v)
    else:
        layer_set = "직접"
        if args.xodr is None:
            args.xodr = pcl.XODR_V3
    args.layers = [s.strip() for s in args.layers.split(",") if s.strip()]
    if layer_set != "직접":
        print(f"레이어 판 {layer_set}: {args.layers} / ground {args.ground_mesh} / mesh-sizes {args.mesh_sizes}")
    bad = [s for s in args.layers if s not in LAYERS]
    if bad:
        raise SystemExit(f"모르는 레이어 {bad}. 가능: {LAYERS}")
    if "buildings" in args.layers and not args.repr:
        raise SystemExit("buildings 레이어는 --repr 가|나|다 가 필요")
    random.seed(args.seed)
    if args.mesh_sizes:
        pcl.load_mesh_sizes(args.mesh_sizes)

    import carla
    text = args.xodr.read_text()
    sha = hashlib.sha256(text.encode()).hexdigest()[:16]
    print(f"xodr {args.xodr.name} sha256 {sha} / layers {args.layers} / repr {args.repr} / seed {args.seed} / 여유 {pcl.MARGIN}m")
    frame = pcl.Frame(text)
    cmap = carla.Map(args.xodr.stem, text)
    root = ET.fromstring(text)
    platforms = pcl.parse_platforms(args.stops)
    rows = pcl.read_layout(args.layout)
    quads = pcl.lane_quads(cmap, root)
    rroads, nroutes = pcl.route_roads()
    acts, notes = [], []
    for layer in args.layers:
        if layer == "buildings":
            to_na = []
            for r in rows:
                rp = r["repr_c"] if args.repr == "다" else args.repr
                btag = r["name"] or f"way{r['way_id']}"
                if rp == "가" and r["way_id"] in pcl.GA_TO_NA:
                    rp = "나"
                    to_na.append((f"{btag} part{r['part']}", pcl.GA_TO_NA[r["way_id"]]))
                elif rp == "가":
                    hit = [v for v in pcl.check(pcl.building_actors(r, "가", frame), quads, rroads) if v["route"]]
                    if hit:
                        rp = "나"
                        to_na.append((f"{btag} part{r['part']}", "자동: 가 메시 경로 침범 r" + "·r".join(str(v["road"]) for v in hit)))
                acts += pcl.building_actors(r, rp, frame)
            if args.repr in ("가", "다"):
                print(f"[buildings] --repr {args.repr}: 가 -> 나 로 바꾼 동 {len(to_na)}")
                for n, why in to_na:
                    print(f"  나로 {n}: {why}")
    blocks = [(a["owner"], a["foot"]) for a in acts] + \
             [(r["name"] or r["way_id"], pcl.rect(*frame.carla(float(r["center_lat"]), float(r["center_lon"])), float(r["yaw_deg"]),
                                                 float(r["length_m"]), float(r["width_m"]))) for r in rows]
    for layer in args.layers:
        if layer == "busstop":
            a, notes = pcl.busstop_actors(platforms, cmap, root, quads, rroads, blocks)
            acts += a
        elif layer == "ground":
            g = pcl.ground_actors(quads, args.ground_mesh)
            print(f"[ground] {g[0]['owner']} scale {g[0]['scale']:g} 중심 ({g[0]['x']:.1f}, {g[0]['y']:.1f}, {g[0]['z']:.2f}) "
                  f"{g[0]['foot_note']}")
            acts += g
        elif layer == "buildings2":
            rows2 = pcl.read_layout(args.layout2)
            b2 = building2_actors(rows2, frame)
            print(f"[buildings2] {args.layout2.name}: {len({r['way_id'] for r in rows2})}동 상자 {len(rows2)} -> 정육면체 {len(b2)}")
            acts += b2
        elif layer == "trees":
            t = tree_actors(args.trees_layout, frame)
            print(f"[trees] {args.trees_layout.name}: {len(t)}그루")
            acts += t
        elif layer != "buildings":
            print(f"[{layer}] 빈 레이어(이번 초안 미구현) 0개")
    # 새 레이어가 정류장 시설과 겹치면 뺌(정류장 자리는 원래대로)
    bus = [a for a in acts if a["layer"] == "busstop"]
    new_clash = [a for a in acts if a["layer"] in ("buildings2", "trees")
                 and any(a["foot"].intersection(s["foot"]).area > 1e-3 for s in bus)]
    if new_clash:
        print(f"새 레이어 vs 정류장 시설 겹침 {len(new_clash)} -> 뺌 {[a['owner'] for a in new_clash]}")
        acts = [a for a in acts if a not in new_clash]
    viol = pcl.check(acts, quads, rroads)
    n_route = sum(1 for v in viol if v["route"])
    print(f"차선 조각 {len(quads)}, 경로 {nroutes}개 road {len(rroads)}")
    for layer in args.layers:
        n = sum(1 for a in acts if a["layer"] == layer)
        vr = len({v["idx"] for v in viol if v["layer"] == layer and v["route"]})
        vo = len({v["idx"] for v in viol if v["layer"] == layer and not v["route"]})
        print(f"[{layer}] 배치물 {n}, 경로 차로 침범 배치물 {vr}, 그 밖 차로만 침범 {len({v['idx'] for v in viol if v['layer'] == layer}) - vr} (그 밖 포함 {vo})")
    print(f"경로 차로 침범 건수 {n_route} -> {'기준 통과(0)' if n_route == 0 else '기준 미달'}")
    alt = [dict(a, foot=a["foot_box"]) for a in acts if "foot_box" in a]
    if alt:
        va = pcl.check(alt, quads, rroads)
        print(f"[가 참고] 메시 폭 = 상자 폭으로 가정하면: 경로 차로 침범 배치물 {len({v['idx'] for v in va if v['route']})}, "
              f"건수 {sum(1 for v in va if v['route'])}, 그 밖 포함 배치물 {len({v['idx'] for v in va})}")
    for v in viol:
        print(f"  위반 {'경로' if v['route'] else '그밖'} {v['layer']} {v['owner']} {v['kind']} r{v['road']} "
              f"거리 {v['clearance_m']}m 겹침 {v['overlap_m2']}m2")
    bus = [a for a in acts if a["layer"] == "busstop"]
    blds = [a for a in acts if a["layer"] == "buildings"]
    clash = [(b["owner"], s["owner"], round(b["foot"].intersection(s["foot"]).area, 2)) for s in bus for b in blds
             if b["foot"].intersects(s["foot"]) and b["foot"].intersection(s["foot"]).area > 1e-3]
    print(f"정류장 시설 vs 건물 바닥 겹침 {len(clash)} {clash[:10]}")
    for n in notes:
        print(f"  busstop {n}")
    if any(l in args.layers for l in ("buildings2", "trees")):
        old = [a for a in acts if a["layer"] in ("buildings", "buildings2")]
        tr = [a for a in acts if a["layer"] == "trees"]
        tb = sum(1 for t in tr if any(t["foot"].intersection(b["foot"]).area > 1e-3 for b in old))
        print(f"[새 레이어] 나무 수관 vs 건물 바닥 겹침 {tb}")

    if args.dry_run:
        tag = args.tag or (args.repr if args.layers == ["buildings", "busstop"] else
                           "-".join(([args.repr] if "buildings" in args.layers else []) + [l for l in args.layers if l != "buildings"]))
        out = pcl.LOGS / f"campus_layers_dry_{tag}.json"
        out.write_text(json.dumps(dict(
            map_sha16=sha, layers=args.layers, repr=args.repr, seed=args.seed, margin_m=pcl.MARGIN, routes=nroutes,
            route_roads=len(rroads), lane_pieces=len(quads),
            actors=[{k: (round(v, 3) if isinstance(v, float) else v) for k, v in a.items() if k not in ("foot", "foot_box")}
                    for a in acts],
            violations=viol, busstop_notes=notes, busstop_building_clash=clash,
            **({} if layer_set == "직접" else dict(layer_set=layer_set))), ensure_ascii=False, indent=1))
        with open(pcl.LOGS / f"campus_layers_dry_{tag}_actors.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["layer", "owner", "kind", "blueprint", "mesh_path", "scale", "x", "y", "z", "yaw"])
            for a in acts:
                w.writerow([a["layer"], a["owner"], a["kind"], a["bp"], a["mesh_path"] or "", a["scale"] if a["scale"] is not None else "",
                            f"{a['x']:.3f}", f"{a['y']:.3f}", f"{a['z']:.3f}", f"{a['yaw']:.2f}"])
        pcl.draw(acts, viol, quads, rroads, platforms, pcl.FIGS / f"campus_layers_dry_{tag}.png",
                 f"place_campus_layers --dry-run --repr {tag}: 배치물 {len(acts)}, 경로 차로 침범 {n_route} (여유 {pcl.MARGIN}m)",
                 rows if "buildings" in args.layers else None)
        if "busstop" in args.layers:
            pcl.draw_busstops(acts, viol, quads, rroads, platforms,
                              pcl.FIGS / (f"campus_layers_dry_busstop_{args.tag}.png" if args.tag else "campus_layers_dry_busstop.png"))
        print(f"-> {out.name}, campus_layers_dry_{tag}_actors.csv, figures/campus_layers_dry_{tag}.png")
        return
    pcl.spawn(args, acts, viol, sha)
    st = json.loads(args.state.read_text())     # 판 이름을 상태 파일에 더함(s59_run.sh 가 실행 기록에 찍음)
    st["layer_set"] = layer_set
    args.state.write_text(json.dumps(st, ensure_ascii=False, indent=1))
    print(f"레이어 판 {layer_set} -> {args.state.name}")


if __name__ == "__main__":
    main()
