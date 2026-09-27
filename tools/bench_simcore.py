#!/usr/bin/env python3
"""
仿真内核热点基准: C 内核 (sim_core/native) vs 纯 Python，逐项计时 (µs/次)

  python3 tools/bench_simcore.py [--steps 3000] [--backend mujoco|kinematic]
  SIM_NATIVE=0 python3 tools/bench_simcore.py      # 纯 Python 基线 (同一进程内无法切换，分别运行)

输出 JSON 一行，便于写入 docs/PERFORMANCE.md。
"""
import argparse
import json
import math
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import numpy as np  # noqa: E402

from planning.dijkstra_planner import SCENARIO_DEFINITIONS  # noqa: E402
from sim_core import native  # noqa: E402
from sim_core.engine import SimCore  # noqa: E402


def _spec():
    p = os.path.join(ROOT, "tests", "data", "测试车模型.cmodel")
    if os.path.exists(p):
        from cmodel_parser import parse_cmodel_file
        return parse_cmodel_file(p)
    with open(os.path.join(ROOT, "robot_config.json"), encoding="utf-8") as f:
        return json.load(f)


def timeit(fn, n):
    t0 = time.perf_counter()
    for _ in range(n):
        fn()
    return (time.perf_counter() - t0) / n * 1e6


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=3000)
    ap.add_argument("--backend", default="mujoco")
    a = ap.parse_args()
    core = SimCore(_spec(), SCENARIO_DEFINITIONS, "grid_9_square", backend=a.backend, noise=True)
    core.set_obstacles([{"x": 1.0, "y": 0.5, "w": 0.6, "h": 0.4}, {"x": 2.5, "y": -1.5, "w": 0.5, "h": 0.5, "type": "person"}])
    core.set_cmd(0.6, 0.0, 0.15)
    for _ in range(200):                      # 预热 (舵角到位)
        core.set_cmd(0.6, 0.0, 0.15)
        core.step()
    n = a.steps
    kin, odom, w = core.kin, core.odom, core.world

    def full_step():
        core.cmd_time = core.t
        core.step()

    res = {
        "native": native.info()["status"], "backend": core.backend_name, "chassis": core.chassis_type,
        "engine_step_us": timeit(full_step, n),
        "kin_step_us": timeit(lambda: kin.step(0.6, 0.0, 0.15, 0.01), n),
        "odom_update_us": timeit(lambda: odom.update(0.01), n),
        "forward_us": timeit(lambda: kin.forward(), n),
        "update_discrete_us": timeit(lambda: core.update_discrete(force=True), n),
        "photo_update_us": timeit(lambda: [p.update(w, core.x, core.y, core.th) for p in core.photos], n) / max(1, len(core.photos)),
        "bumper_update_us": timeit(lambda: [b.update(w, core.x, core.y, core.th, core.t) for b in core.bumpers], n) / max(1, len(core.bumpers)),
        "collides_footprint_us": timeit(lambda: w.collides(core.footprint, core.x, core.y, core.th), n),
        "imu_us": timeit(lambda: core.imu.sample(0.5, 0.0, 0.1, 0.0, 0.01), n),
        "slip_us": timeit(lambda: core.slip.apply(0.5, 0.0, 0.1, 0.01), n),
        "scan_all_ms": timeit(core.scan_all, max(50, n // 20)) / 1000.0,
        "n_photos": len(core.photos), "n_bumpers": len(core.bumpers), "n_segments": int(len(w.segments)),
    }
    print(json.dumps({k: (round(v, 2) if isinstance(v, float) else v) for k, v in res.items()}, ensure_ascii=False))


if __name__ == "__main__":
    main()
