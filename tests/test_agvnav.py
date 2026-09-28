#!/usr/bin/env python3
"""libagvnav (planning/native/agvnav.c) 与 Python 规划算法逐项对比

  bash planning/native/build.sh && python3 tests/test_agvnav.py [每场景用例数]
随机: 全部内置场景 × 车体外形/过弯半径/净空/过弯模式 × 起终点 × 动态障碍物；
另有 maneuver.clearance / plan_corner 的随机对比。
"""
import math
import os
import random
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from planning import maneuver, native  # noqa: E402
from planning.dijkstra_planner import SCENARIO_DEFINITIONS, DijkstraPlanner  # noqa: E402

LIB = native.lib


def with_native(on, fn, *a, **k):
    native.lib = LIB if on else None
    try:
        return fn(*a, **k)
    finally:
        native.lib = LIB


def same_route(a, b, nodes=None):
    if len(a["points"]) != len(b["points"]) or abs(a["length"] - b["length"]) > 0.011:
        return False
    if not all(math.hypot(p[0] - q[0], p[1] - q[1]) < 1e-9 for p, q in zip(a["points"], b["points"])):
        return False
    for p, la, lb in zip(a["points"], a["labels"], b["labels"]):
        if la == lb:
            continue
        # 起点恰在节点上、两条等长路线 (经节点 / 同边直达) 并列: CPython math.hypot 与 C hypot 末位舍入不同，
        # 选中的一条可能不同 —— 点列相同，只差该点是否标上节点名
        n = la or lb
        if (la is None or lb is None) and nodes and n in nodes and math.hypot(nodes[n][0] - p[0], nodes[n][1] - p[1]) < 1e-9:
            continue
        return False
    return True


def bounds(sc):
    xs = [p[0] for p in sc["nodes"].values()]
    ys = [p[1] for p in sc["nodes"].values()]
    return min(xs) - 1, max(xs) + 1, min(ys) - 1, max(ys) + 1


def test_routes(n_per):
    rng = random.Random(5)
    total = bad = ties = 0
    for sid in list(SCENARIO_DEFINITIONS):
        sc = SCENARIO_DEFINITIONS[sid]
        x0, x1, y0, y1 = bounds(sc)
        for trial in range(max(1, n_per // 10)):
            fp = (round(rng.uniform(0.5, 1.4), 3), round(rng.uniform(0.3, 0.7), 3), round(rng.uniform(0.3, 0.55), 3),
                  round(rng.uniform(0.6, 1.4), 3), 0.05, rng.choice(["auto", "arc", "rotate"]))
            hw, circ = fp[2], math.hypot(fp[0], fp[2])
            py, cc = DijkstraPlanner(sid), DijkstraPlanner(sid)
            for p in (py, cc):
                p.robot_half_width, p.robot_circum_radius = hw, circ
                if rng.random() < 0.8 or True:
                    p.set_footprint(fp[0], fp[1], fp[2], fp[3], fp[4], fp[5])
            for _ in range(10):
                if rng.random() < 0.5 and sc.get("stations"):
                    st = rng.choice(sc["stations"])
                    s = (st["x"], st["y"])
                else:
                    s = (rng.uniform(x0, x1), rng.uniform(y0, y1))
                st = rng.choice(sc["stations"]) if sc.get("stations") else None
                g = (st["x"], st["y"]) if st and rng.random() < 0.7 else (rng.uniform(x0, x1), rng.uniform(y0, y1))
                obs = [{"x": rng.uniform(x0, x1), "y": rng.uniform(y0, y1), "w": rng.uniform(0.3, 1.2), "h": rng.uniform(0.3, 1.2)}
                       for _ in range(rng.choice([0, 0, 1, 2, 3]))]
                a = with_native(False, py.plan_route, s, g, obs)
                b = with_native(True, cc.plan_route, s, g, obs)
                total += 1
                if not same_route(a, b, sc["nodes"]) and a["points"] and b["points"] and abs(a["length"] - b["length"]) < 1e-6 \
                        and a["points"][0] == b["points"][0] and a["points"][-1] == b["points"][-1]:
                    ties += 1       # 等长并列路线 (如对称场景两条镜像路线)，浮点末位决定选哪条: 两个都对
                    continue
                if not same_route(a, b, sc["nodes"]):
                    bad += 1
                    if bad <= 5:
                        print(f"不一致 {sid} fp={fp} s={s} g={g} obs={len(obs)}\n  Py {a}\n  C  {b}")
    print(f"{'PASS' if bad == 0 else 'FAIL'} 拓扑贴合规划: {total - bad - ties}/{total} 逐点一致，{ties} 个等长并列路线 (选了镜像的另一条)")
    return bad == 0


def test_maneuver(n):
    rng = random.Random(9)
    bad_c = bad_p = 0
    for _ in range(n):
        segs = [(rng.uniform(-4, 4), rng.uniform(-4, 4), rng.uniform(-4, 4), rng.uniform(-4, 4)) for _ in range(rng.randint(0, 12))]
        poses = [(rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-3.2, 3.2)) for _ in range(rng.randint(1, 20))]
        head, tail, hw = rng.uniform(0.4, 1.4), rng.uniform(0.3, 0.7), rng.uniform(0.3, 0.55)
        a = with_native(False, maneuver.clearance, segs, poses, head, tail, hw)
        b = with_native(True, maneuver.clearance, segs, poses, head, tail, hw)
        if abs(a - b) > 1e-9:
            bad_c += 1
        node = (rng.uniform(-1, 1), rng.uniform(-1, 1))
        h1, h2 = rng.uniform(-3.2, 3.2), rng.uniform(-3.2, 3.2)
        args = (segs, node, h1, h2, rng.uniform(0.5, 5), rng.uniform(0.5, 5), head, tail, hw, rng.uniform(0.6, 1.4), 0.05)
        mode = rng.choice(["auto", "arc", "rotate"])
        pa = with_native(False, maneuver.plan_corner, *args, mode=mode)
        pb = with_native(True, maneuver.plan_corner, *args, mode=mode)
        ok = abs(pa[1] - pb[1]) < 1e-9 and set(pa[0]) == set(pb[0]) and all(abs(pa[0][k] - pb[0][k]) < 1e-9 for k in pa[0])
        if not ok and abs(pa[1] - pb[1]) < 1e-9 and "rotate" in pa[0] and "rotate" in pb[0]:
            ok = True       # 两个转向方向净空相同 (差 1e-17 级)：浮点舍入决定取哪个，两种都对
        if not ok:
            bad_p += 1
            if bad_p <= 3:
                print("plan_corner 不一致", mode, "\n  Py", pa, "\n  C ", pb)
    print(f"{'PASS' if bad_c == 0 else 'FAIL'} 车体净空 clearance: {n - bad_c}/{n}")
    print(f"{'PASS' if bad_p == 0 else 'FAIL'} 拐点过弯 plan_corner: {n - bad_p}/{n}")
    return bad_c == 0 and bad_p == 0


if __name__ == "__main__":
    if LIB is None:
        print("SKIP libagvnav 未编译:", native.status)
        sys.exit(0)
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 200
    ok = test_maneuver(n * 5)
    ok = test_routes(n) and ok
    sys.exit(0 if ok else 1)
