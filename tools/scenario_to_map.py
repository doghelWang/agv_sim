#!/usr/bin/env python3
"""
场景定义 (planning/dijkstra_planner.SCENARIO_DEFINITIONS) → Nav2 map_server 栅格地图 (PGM + YAML)

与仿真世界使用同一份几何数据，保证 Nav2 全局地图与仿真器、Web 画面完全一致。
  外墙/货架边界线段 → 0.10 m 厚占据；货架/设备岛矩形 → 实心占据 (不可穿越，也不会被规划进内部)
用法: python3 tools/scenario_to_map.py [out_dir=maps] [--res 0.05]
"""

import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

from planning.dijkstra_planner import SCENARIO_DEFINITIONS  # noqa: E402

FREE, OCC = 254, 0


def rasterize(sc: dict, res: float = 0.05, margin: float = 1.0, wall_thick: float = 0.10):
    walls = sc["walls"]
    xs = [c for w in walls for c in (w[0], w[2])]
    ys = [c for w in walls for c in (w[1], w[3])]
    x0, y0 = min(xs) - margin, min(ys) - margin
    x1, y1 = max(xs) + margin, max(ys) + margin
    W = int(math.ceil((x1 - x0) / res))
    H = int(math.ceil((y1 - y0) / res))
    img = bytearray([205] * (W * H))  # unknown

    def setp(ix, iy, v):
        if 0 <= ix < W and 0 <= iy < H:
            img[(H - 1 - iy) * W + ix] = v

    # 外墙以内设为 free
    bx0, bx1 = min(c for w in walls[:4] for c in (w[0], w[2])), max(c for w in walls[:4] for c in (w[0], w[2]))
    by0, by1 = min(c for w in walls[:4] for c in (w[1], w[3])), max(c for w in walls[:4] for c in (w[1], w[3]))
    for iy in range(int((by0 - y0) / res), int((by1 - y0) / res) + 1):
        for ix in range(int((bx0 - x0) / res), int((bx1 - x0) / res) + 1):
            setp(ix, iy, FREE)
    # 货架实心
    for sh in sc.get("shelves", []):
        a, b = sorted((sh["x1"], sh["x2"])), sorted((sh["y1"], sh["y2"]))
        for iy in range(int((b[0] - y0) / res), int(math.ceil((b[1] - y0) / res)) + 1):
            for ix in range(int((a[0] - x0) / res), int(math.ceil((a[1] - x0) / res)) + 1):
                setp(ix, iy, OCC)
    # 线段加粗
    half = wall_thick / 2
    for (ax, ay, bx, by) in walls:
        L = math.hypot(bx - ax, by - ay)
        n = max(1, int(L / (res * 0.5)))
        for k in range(n + 1):
            px = ax + (bx - ax) * k / n
            py = ay + (by - ay) * k / n
            r = int(math.ceil(half / res))
            cx, cy = int((px - x0) / res), int((py - y0) / res)
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    setp(cx + dx, cy + dy, OCC)
    return img, W, H, (x0, y0)


def write_map(sid: str, out_dir: str, res: float = 0.05) -> str:
    sc = SCENARIO_DEFINITIONS[sid]
    img, W, H, (ox, oy) = rasterize(sc, res)
    os.makedirs(out_dir, exist_ok=True)
    pgm = os.path.join(out_dir, f"{sid}.pgm")
    with open(pgm, "wb") as f:
        f.write(f"P5\n# scenario {sid}\n{W} {H}\n255\n".encode())
        f.write(bytes(img))
    yml = os.path.join(out_dir, f"{sid}.yaml")
    with open(yml, "w", encoding="utf-8") as f:
        f.write(f"image: {sid}.pgm\nmode: trinary\nresolution: {res}\norigin: [{ox:.3f}, {oy:.3f}, 0.0]\n"
                f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    return yml


def write_all(out_dir: str, res: float = 0.05):
    return [write_map(s, out_dir, res) for s in SCENARIO_DEFINITIONS]


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else os.path.join(os.path.dirname(_HERE), "maps")
    res = float(sys.argv[sys.argv.index("--res") + 1]) if "--res" in sys.argv else 0.05
    for p in write_all(out, res):
        print("map:", p)
