#!/usr/bin/env python3
"""C++ 安全层 (ros2/agv_ros_bridge/src/safety.hpp) 与 Python navigator.safety_filter / _on_merged 逐项对比

  python3 tests/test_cpp_safety.py [用例数]      需要 g++；不依赖 ROS
随机: 外形、防护区分档、融合扫描点、指令速度、急停、限速、进站剩余行程、光电触发与检测距离。
原地转向防护的已知差异 (C++ 对"起始已在外扩区内、转动后进入车体"的点判受阻，Python 忽略) 单独测。
"""
import json, math, os, random, subprocess, sys, tempfile, types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
import numpy as np  # noqa: E402
from planning import protection  # noqa: E402
from nav_runtime.navigator import Navigator  # noqa: E402


def build():
    exe = os.path.join(tempfile.gettempdir(), "agv_safety_harness")
    src = os.path.join(ROOT, "ros2", "agv_ros_bridge", "test", "safety_harness.cpp")
    inc = os.path.join(ROOT, "ros2", "agv_nav2_plugins", "include")
    subprocess.check_call(["g++", "-std=c++17", "-O2", "-I", inc, "-o", exe, src])
    return exe


class FakeNav:
    """Navigator.safety_filter 需要的最小状态"""
    safety_filter = Navigator.safety_filter
    _allowed_speed = Navigator._allowed_speed
    _rotation_blocked = Navigator._rotation_blocked
    _photo_sides = Navigator._photo_sides
    _on_merged = Navigator._on_merged
    _slam_uses_merged = lambda self: False  # noqa: E731

    def __init__(self, case, body):
        self.prot = case["prot"]
        self.body = body
        self.loaded = False
        self.active_planner = "nav2"
        self.slam = types.SimpleNamespace(ros_active=lambda: False, ext=None)
        self.speed_cap = case["speed_cap"] or None
        self.approach_left = case["left"]
        self.in_arc = case["in_arc"]
        self.cfg = {"chassis": {"max_decel_mps2": case["max_decel"] * 1.0}}
        self.obs = {"zone": "clear", "layer": None}
        self.lock = __import__("threading").Lock()
        self.telemetry = {"vx": case["v_meas"]}
        di = {p["di"]: (p["name"] in [h[0] for h in case["hits"]]) for p in case["photos"]}
        di["di_estop"] = case["estop"]
        self.link = types.SimpleNamespace(
            io={"inputs": di},
            photos={h[0]: {"distance_m": h[1]} for h in case["hits"]},
            sensors={"photoelectric": [{"name": p["name"], "di": p["di"], "mount": {"x": p["x"], "y": p["y"], "yaw": p["yaw"]}}
                                       for p in case["photos"]]})
        self.zone_log = []

    def outline(self):
        return protection.outline(self.body, self.prot, self.loaded)

    def _zone(self, zone, layer=None, d=None, need=None):
        self.obs["zone"], self.obs["layer"] = zone, layer if zone in ("slow", "stop") else None
        self.zone_log.append((zone, layer))


def rand_case(rng):
    body = {"head_offset_m": rng.uniform(0.4, 1.4), "tail_offset_m": rng.uniform(0.3, 0.8),
            "left_offset_m": rng.uniform(0.3, 0.55), "right_offset_m": rng.uniform(0.3, 0.55)}
    P = protection.effective({"chassis": dict(body, max_decel_mps2=0.6, max_speed_mps=1.2),
                              "protection": {"photo": {"mode": rng.choice(["field", "custom", "always"])}}})
    h, t, l, r = protection.outline(body, P, False)
    pts = []
    m = P["rotate_margin"]
    for _ in range(rng.randint(0, 120)):
        a = rng.uniform(-math.pi, math.pi)
        d = rng.uniform(0.05, 4.0)
        x, y = d * math.cos(a), d * math.sin(a)
        inside_m = x < h + m + 0.01 and x > -t - m - 0.01 and y < l + m + 0.01 and y > -r - m - 0.01
        if inside_m:           # 避开外扩区/车体内 (两边约定不同的情形单独测)
            continue
        pts.append([round(x, 4), round(y, 4)])
    photos = [{"name": f"pe{i}", "di": f"di_pe{i}", "x": rng.uniform(-t, h), "y": rng.uniform(-r, l),
               "yaw": rng.choice([0.0, math.pi, math.pi / 2, -math.pi / 2, 0.8, -2.4])} for i in range(rng.randint(0, 4))]
    hits = [[p["name"], rng.choice([None, round(rng.uniform(0.05, 1.5), 3)])] for p in photos if rng.random() < 0.4]
    mode = rng.random()
    if mode < 0.4:
        cmd = [rng.uniform(-0.6, 1.2), 0.0, rng.uniform(-0.3, 0.3)]
    elif mode < 0.7:
        cmd = [rng.uniform(-0.04, 0.04), 0.0, rng.choice([-1, 1]) * rng.uniform(0.06, 0.6)]
    else:
        cmd = [rng.uniform(-0.5, 0.8), rng.uniform(-0.3, 0.3), rng.uniform(-0.5, 0.5)]
    return {"prot": P, "outline": [h, t, l, r], "max_decel": 0.6, "photos": photos, "hits": hits, "pts": pts,
            "estop": rng.random() < 0.05, "speed_cap": rng.choice([0.0, 0.0, 0.3, 0.6]),
            "left": rng.choice([None, None, round(rng.uniform(0.0, 1.5), 3)]), "in_arc": rng.random() < 0.1,
            "v_meas": rng.uniform(-0.5, 1.2), "cmd": cmd}, body


def run_python(case, body):
    nav = FakeNav(case, body)
    rs = [math.hypot(x, y) for x, y in case["pts"]]
    an = [math.atan2(y, x) for x, y in case["pts"]]
    # _on_merged 期望等角度扫描: 按点逐个构造 (inc=0 时 a = a0)，逐点调用等价于分别计算后取最小 → 这里直接构造点集
    nav._pts = (np.array([p[0] for p in case["pts"]]), np.array([p[1] for p in case["pts"]]))
    h, t, lo, ro = nav.outline()
    px, py = nav._pts
    bands = []
    for f in nav.prot["fields"]:
        mk = (py <= lo + f["side"]) & (py >= -(ro + f["side"]))
        fr = px[mk & (px > 0)] - h
        rr = -px[mk & (px <= 0)] - t
        fr, rr = fr[fr > -0.05], rr[rr > -0.05]
        bands.append((round(max(0.0, float(fr.min())), 2) if fr.size else None, round(max(0.0, float(rr.min())), 2) if rr.size else None))
    nav.obs["bands"] = bands
    i, _ = protection.field_for_speed(nav.prot, case["v_meas"])
    nav.obs["band"] = i
    vx, vy, wz = nav.safety_filter(*case["cmd"])
    return {"vx": vx, "vy": vy, "wz": wz, "bands": bands, "zone": nav.zone_log[-1][0] if nav.zone_log else ""}


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 3000
    exe = build()
    rng = random.Random(11)
    cases = [rand_case(rng) for _ in range(n)]
    inp = "\n".join(json.dumps(c, separators=(",", ":")) for c, _ in cases) + "\n"
    out = subprocess.run([exe], input=inp, capture_output=True, text=True, check=True).stdout.strip().split("\n")
    bad = 0
    for (c, body), line in zip(cases, out):
        cc = json.loads(line)
        py = run_python(c, body)
        pb = [[-1 if a is None else a, -1 if b is None else b] for a, b in py["bands"]]
        ok = all(abs(cc[k] - py[k]) < 1e-6 for k in ("vx", "vy", "wz")) and \
            all(abs(a - b) < 1e-6 for x, y in zip(cc["bands"], pb) for a, b in zip(x, y))
        if not ok:
            bad += 1
            if bad <= 5:
                print("不一致:", json.dumps({"cmd": c["cmd"], "left": c["left"], "cap": c["speed_cap"], "estop": c["estop"],
                                           "arc": c["in_arc"], "hits": c["hits"]}, ensure_ascii=False), "\n  C++", cc, "\n  Py ",
                      {k: py[k] for k in ("vx", "vy", "wz", "bands")})
    print(f"{'PASS' if bad == 0 else 'FAIL'} 安全层一致性: {n - bad}/{n}")
    # 已知差异: 贴墙 1 cm (外扩区内) 的墙，向墙转 → C++ 受阻，Python 不受阻
    body = {"head_offset_m": 0.6, "tail_offset_m": 0.4, "left_offset_m": 0.35, "right_offset_m": 0.35}
    P = protection.effective({"chassis": dict(body, max_decel_mps2=0.6)})
    wall = [[round(x, 3), 0.36] for x in np.arange(-0.3, 0.55, 0.02)]
    case = {"prot": P, "outline": [0.6, 0.4, 0.35, 0.35], "max_decel": 0.6, "photos": [], "hits": [], "pts": wall, "estop": False,
            "speed_cap": 0.0, "left": None, "in_arc": False, "v_meas": 0.0, "cmd": [0.0, 0.0, 0.4]}
    cc = json.loads(subprocess.run([exe], input=json.dumps(case) + "\n", capture_output=True, text=True).stdout)
    py = run_python(case, body)
    print(f"{'PASS' if cc['wz'] == 0.0 else 'FAIL'} 贴墙 1 cm 向墙转: C++ wz={cc['wz']} (受阻)，Python wz={py['wz']} (旧约定不受阻)")
    return 0 if bad == 0 and cc["wz"] == 0.0 else 1


if __name__ == "__main__":
    sys.exit(main())
