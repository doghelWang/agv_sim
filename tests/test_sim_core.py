#!/usr/bin/env python3
"""
离线单元测试 (无需 ROS): python3 -m pytest tests/test_sim_core.py -q   或   python3 tests/test_sim_core.py
"""
import json
import math
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import numpy as np  # noqa: E402

from cmodel_parser import generate_urdf, parse_cmodel_file  # noqa: E402
from planning.dijkstra_planner import SCENARIO_DEFINITIONS  # noqa: E402
from sim_core.engine import SimCore  # noqa: E402
from sim_core.kinematics import build_kinematics, se2_integrate  # noqa: E402

DATA = os.path.join(ROOT, "tests", "data")
CMODEL = os.path.join(DATA, "测试车模型.cmodel")


def _spec():
    if os.path.exists(CMODEL):
        return parse_cmodel_file(CMODEL)
    with open(os.path.join(ROOT, "robot_config.json"), encoding="utf-8") as f:
        return json.load(f)


def _run(kin, cmd, T, dt=0.01):
    x = y = th = 0.0
    for _ in range(int(T / dt)):
        vx, vy, wz = kin.step(*cmd, dt)
        x, y, th = se2_integrate(x, y, th, vx, vy, wz, dt)
    return x, y, th


def test_parser_restores_real_robot():
    s = _spec()
    ch = s["chassis"]
    assert ch["type"] == "single_steer"
    assert abs(ch["head_offset_m"] - 1.308) < 1e-6 and abs(ch["tail_offset_m"] - 0.48) < 1e-6
    sw = [w for w in s["wheels"] if w["kind"] == "steer"][0]
    assert abs(sw["x"] - 1.09) < 1e-6 and abs(sw["radius_m"] - 0.115) < 1e-6
    assert abs(math.degrees(sw["steer_max_rad"]) - 120) < 1e-3
    assert abs(sw["max_speed_mps"] - 1.397) < 0.01          # 2560rpm / 22.07 × 2πr
    assert abs(math.degrees(sw["steer_rate_radps"]) - 67.0) < 0.5
    lids = {l["name"]: l for l in s["lidars"]}
    assert set(lids) == {"laser", "laser0", "laser1"}
    assert lids["laser0"]["inverted"] and not lids["laser"]["inverted"]
    assert abs(lids["laser"]["z"] - 2.03) < 1e-6


def test_urdf_valid():
    import xml.dom.minidom
    u = generate_urdf(_spec())
    xml.dom.minidom.parseString(u)
    assert "Steerwheel_steer_joint" in u and "laser0_joint" in u


def test_single_steer_reverse_and_spin():
    kin, _ = build_kinematics(_spec(), "single_steer")
    x, y, th = _run(kin, (-0.5, 0, 0), 5.0)
    assert x < -2.0 and abs(y) < 0.01 and abs(th) < 0.01      # 旧模型此处会转 -105°
    kin, _ = build_kinematics(_spec(), "single_steer")
    x, y, th = _run(kin, (0, 0, 0.5), 5.0)
    assert math.hypot(x, y) < 0.08 and th > 1.5                # 绕后桥原地转向


def test_wheel_speed_saturation():
    kin, _ = build_kinematics(_spec(), "single_steer")
    _run(kin, (3.0, 0, 0), 6.0)
    assert abs(kin.vx - 1.397) < 0.02 and kin.saturation < 1.0


def test_dual_steer_lateral():
    kin, _ = build_kinematics(_spec(), "dual_steer")
    x, y, th = _run(kin, (0, 0.4, 0), 5.0)
    assert y > 1.5 and abs(x) < 0.05 and abs(th) < 0.01


def test_collision_and_bumper():
    sim = SimCore(_spec(), SCENARIO_DEFINITIONS, "grid_9_square", noise=False)
    sim.reset_pose(4.3, 0.0, math.pi / 2)
    for _ in range(600):
        sim.set_cmd(0.8, 0, 0)
        sim.step()
    assert sim.collisions >= 1 and sim.y < 1.35                # 车头 1.308m 撞到 y=2.6 的货架
    assert sim.io.di["di_bumper_front"]


def test_low_obstacle_invisible_to_top_lidar():
    sim = SimCore(_spec(), SCENARIO_DEFINITIONS, "fms_workshop", noise=False)
    sim.set_obstacles([{"x": 2.0, "y": 4.5, "w": 0.8, "h": 0.8, "z": 1.0}])
    sim.reset_pose(0.0, 4.5, 0.0)
    rs, merged = sim.scan_all()
    by = {l.name: r for l, r in zip(sim.lidars, rs)}
    assert np.nanmin(np.where(np.isfinite(by["laser1"]), by["laser1"], np.nan)) < 0.4
    assert np.nanmin(np.where(np.isfinite(by["laser"]), by["laser"], np.nan)) > 1.5
    mid = len(merged) // 2
    assert abs(np.nanmin(merged[mid - 3: mid + 4]) - 1.6) < 0.05


def test_realtime_accumulator():
    sim = SimCore(_spec(), SCENARIO_DEFINITIONS, "grid_9_square", noise=False)
    n = sim.advance(0.105)
    assert n == 10 and abs(sim.t - 0.10) < 1e-9
    sim.advance(10.0)                                           # 严重落后 → 丢弃积压而不是卡死
    assert sim.overruns == 1


def test_mid360s_top_lidar():
    """顶部激光按 sensor_overrides.json 替换为 Livox Mid-360S 3D 激光"""
    ov = json.load(open(os.path.join(ROOT, "sensor_overrides.json"), encoding="utf-8"))
    ov.pop("_comment", None)
    spec = parse_cmodel_file(CMODEL, overrides=ov)
    top = [l for l in spec["lidars"] if l["name"] == "laser"][0]
    assert top["type"] == "3d" and top["vendor_model"] == "Livox Mid-360S"
    assert top["vfov_min_deg"] == -7.0 and top["vfov_max_deg"] == 52.0 and top["min_range"] == 0.1
    sim = SimCore(spec, SCENARIO_DEFINITIONS, "grid_9_square", noise=False)
    sim.reset_pose(0.0, 5.0, 0.0)
    sim.scan_all()
    l3, cl, sl = sim.last_clouds[0]
    P = cl["points"]
    el = np.degrees(np.arctan2(P[:, 2], np.hypot(P[:, 0], P[:, 1])))
    assert el.min() >= -7.5 and el.max() <= 52.5
    assert np.linalg.norm(P, axis=1).min() >= 0.1
    assert (l3.points_base(cl)[:, 2] > 5.9).sum() > 100       # 屋顶回波
    assert set(np.unique(cl["line"])) == {0, 1, 2, 3}


def test_nav2_params_and_maps():
    try:
        import yaml
    except ImportError:          # 仿真镜像/手机环境不带 PyYAML: 跳过 (pip install pyyaml 后可测)
        print("  (跳过: 未安装 PyYAML)")
        return
    from tools.gen_nav2_params import write_all
    from tools.scenario_to_map import write_all as write_maps
    with tempfile.TemporaryDirectory() as d:
        for p in write_all(_spec(), d):
            y = yaml.safe_load(open(p))
            assert "controller_server" in y and "local_costmap" in y
        for p in write_maps(d):
            assert os.path.exists(p.replace(".yaml", ".pgm"))


def test_bumper_strip_and_photoelectric():
    """触边: 接触即压下 → 前向封锁、允许后退脱困；光电: 角部障碍触发"""
    c = SimCore(_spec(), SCENARIO_DEFINITIONS, "grid_9_square", noise=False)
    x0 = c.x
    c.set_obstacles([{"x": x0 + 2.0, "y": c.y, "w": 0.4, "h": 0.4}])
    for _ in range(1500):
        c.set_cmd(0.4, 0, 0)
        c.step()
    front = next(b for b in c.bumpers if b.side == "front")
    assert front.pressed and c.io.di["di_bumper_front"]
    assert c.x < x0 + 2.0 - 0.2 - c.spec["chassis"]["head_offset_m"] + 0.001   # 未穿透
    assert c.io.do["do_brake_release"] and c.io.do["do_tower_red"]            # 触边不断抱闸，但报警
    for _ in range(150):
        c.set_cmd(-0.3, 0, 0)
        c.step()
    assert not front.pressed
    # 光电: 在左前角 30° 方向 0.2 m 放障碍
    c.set_obstacles([])
    p = next(p for p in c.photos if p.name == "front_left")
    import math as _m
    ox = c.x + _m.cos(c.th) * p.x - _m.sin(c.th) * p.y
    oy = c.y + _m.sin(c.th) * p.x + _m.cos(c.th) * p.y
    a = c.th + p.yaw
    c.set_obstacles([{"x": ox + _m.cos(a) * 0.35, "y": oy + _m.sin(a) * 0.35, "w": 0.2, "h": 0.2}])
    c.update_discrete(force=True)
    assert p.detected and c.io.di[p.di], p.view()


def test_mujoco_engine_and_cameras():
    """MuJoCo 后端: 撞墙被接触约束阻挡；单目/双目/ToF 成像尺寸、深度与几何一致"""
    from sim_core.mujoco_backend import MUJOCO_AVAILABLE
    if not MUJOCO_AVAILABLE:
        return
    sp = _spec()
    sp["cameras"] = [{"name": "c", "type": "camera", "x": 1.3, "y": 0, "z": 0.5, "width": 160, "height": 120},
                     {"name": "s", "type": "stereo", "x": 1.3, "y": 0, "z": 0.5, "width": 160, "height": 90},
                     {"name": "t", "type": "tof", "x": 1.3, "y": 0, "z": 0.5}]
    sim = SimCore(sp, SCENARIO_DEFINITIONS, "grid_9_square", noise=False)
    assert sim.backend_name.startswith("mujoco")
    sim.set_obstacles([{"x": 3.0, "y": 0.0, "w": 0.4, "h": 1.2, "z": 1.5}])
    front = 3.0 - 0.2                                              # 障碍物前表面
    outs = {c.name: sim.capture_camera(c) for c in sim.cameras}
    assert outs["c"]["rgb"].shape == (120, 160, 3)
    z = outs["t"]["depth"]
    cz = z[z.shape[0] // 2, z.shape[1] // 2]
    assert abs(cz - (front - 1.3)) < 0.02, cz                      # ToF 中心像素 = 相机到障碍物面距离
    zs = outs["s"]["depth"]
    assert abs(zs[45, 80] - (front - 1.3)) < 0.03
    assert outs["t"]["points"].shape[1] == 3
    for _ in range(800):                                          # 顶着障碍物开 → 被挡住、触边压下
        sim.set_cmd(0.5, 0, 0)
        sim.step()
    assert sim.x + sim.spec["chassis"]["head_offset_m"] < front + 0.005
    assert any(b.pressed for b in sim.bumpers)


def test_model_overrides_roundtrip():
    """人工补全: 新增/修改/删除传感器、顶升 → spec / 审计 / URDF"""
    import xml.dom.minidom
    from model_overrides import apply_overrides, audit
    base = _spec()
    ov = {"chassis": {"mass_kg": 300}, "lift": {"enabled": True, "stroke_m": 0.06},
          "sensors": {"cam_a": {"type": "camera", "_added": True, "x": 1.2, "z": 0.8},
                      "tof_a": {"type": "tof", "_added": True, "x": 1.2, "z": 0.3},
                      "laser0": {"_removed": True}, "laser1": {"max_range": 12.0}}}
    sp = apply_overrides(base, ov)
    assert sp["chassis"]["mass_kg"] == 300
    assert {c["name"] for c in sp["cameras"]} >= {"cam_a", "tof_a"}
    assert "laser0" not in {l["name"] for l in sp["lidars"]}
    assert next(l for l in sp["lidars"] if l["name"] == "laser1")["max_range"] == 12.0
    a = audit(sp, ov)
    assert any(i["source"] == "manual" and i["key"] == "chassis.mass_kg" for i in a["items"])
    u = generate_urdf(sp)
    xml.dom.minidom.parseString(u)
    assert "cam_a_optical_frame" in u and "lift_joint" in u


# ---------------------------------------------------------------------- C 内核 (sim_core/native) 与 Python 实现一致性
class _Skip(Exception):
    pass


def _skip(msg):
    try:
        import pytest
    except ImportError:
        raise _Skip(msg)
    pytest.skip(msg)


def _need_native():
    from sim_core import native
    if native.lib is None:
        _skip(f"libsimcore 不可用: {native.status}")
    return native


def _world(sid="grid_9_square", obstacles=True):
    from sim_core.world import World
    w = World()
    w.load_scenario(SCENARIO_DEFINITIONS[sid])
    if obstacles:
        w.set_obstacles([{"x": 1.0, "y": 0.5, "w": 0.6, "h": 0.4, "yaw": 0.3},
                         {"x": -1.2, "y": 2.0, "w": 0.1, "h": 0.1, "z": 0.2},
                         {"x": 2.5, "y": -1.5, "w": 0.5, "h": 0.5, "type": "person", "z": 1.7}])
    return w


def test_native_collides_matches_python():
    native = _need_native()
    w = _world()
    rng = np.random.default_rng(1)
    b = w.bounds
    fps = [np.asarray(_spec()["chassis"]["footprint"], float), np.array([[0.3, 0.1], [0.33, 0.1], [0.33, -0.1], [0.3, -0.1]])]
    n_hit = 0
    for k in range(3000):
        fp = fps[k % 2]
        x, y, th = rng.uniform(b[0] - 0.5, b[2] + 0.5), rng.uniform(b[1] - 0.5, b[3] + 0.5), rng.uniform(-math.pi, math.pi)
        h1, p1 = native.collides(native.f64(fp), x, y, th, w.segments)
        h2, p2 = w._collides_py(fp, x, y, th)
        assert h1 == h2, (x, y, th)
        if h1:
            n_hit += 1
            assert np.allclose(p1, p2, atol=1e-9), (p1, p2)
    assert n_hit > 50


def test_native_raycast_matches_python():
    native = _need_native()
    w = _world()
    rng = np.random.default_rng(2)
    b = w.bounds
    ang = np.linspace(-math.pi, math.pi, 721)
    for _ in range(200):
        ox, oy = rng.uniform(b[0], b[2]), rng.uniform(b[1], b[3])
        zmin = float(rng.choice([0.0, 0.15, 0.5, 2.03]))
        r1 = native.raycast2d(w.segments, None, zmin, ox, oy, ang, 12.0)
        r2 = w._raycast_py(ox, oy, ang, 12.0, zmin)
        assert np.array_equal(np.isinf(r1), np.isinf(r2))
        f = np.isfinite(r1)
        assert np.allclose(r1[f], r2[f], atol=1e-9)


def test_native_forward_matches_lstsq():
    native = _need_native()
    from sim_core.kinematics import ChassisKinematics, ChassisLimits, Wheel
    rng = np.random.default_rng(3)
    for k in range(400):
        kinds = rng.choice(["drive", "steer", "fixed", "caster"], size=int(rng.integers(1, 6)))
        ws = [Wheel(f"w{i}", str(kd), float(rng.uniform(-1, 1)), float(rng.uniform(-0.6, 0.6)), 0.1) for i, kd in enumerate(kinds)]
        if k % 7 == 0:          # 退化: 同一点的轮子 (秩亏)
            for x in ws:
                x.x, x.y = 0.3, 0.0
        for x in ws:
            x.steer, x.speed = float(rng.uniform(-2, 2)), float(rng.uniform(-1.5, 1.5))
        kin = ChassisKinematics("t", ws, ChassisLimits())
        assert kin.native
        st = {x.name: float(rng.uniform(-1, 1)) for x in ws if rng.random() < 0.5}
        sp = {x.name: float(rng.uniform(-1, 1)) for x in ws if rng.random() < 0.5}
        a = kin._forward_native(st, sp)
        b = kin._forward_py(st, sp)
        assert np.allclose(a, b, atol=1e-9), (kinds, a, b)


def test_native_kin_step_matches_python():
    native = _need_native()
    spec = _spec()
    rng = np.random.default_rng(4)
    for ct in ("single_steer", "diff_drive", "dual_steer"):
        ka, _ = build_kinematics(spec, ct)
        kb, _ = build_kinematics(spec, ct)
        assert ka.native
        kb.native = False
        cmd = (0.0, 0.0, 0.0)
        for i in range(3000):
            if i % 150 == 0:
                cmd = (float(rng.uniform(-1.5, 1.5)), float(rng.uniform(-0.5, 0.5)), float(rng.uniform(-1.2, 1.2)))
                if i % 600 == 0:
                    cmd = (0.0, 0.0, cmd[2])      # 原地旋转
            brake = 900 <= i < 960
            if i == 2000:
                ka.stop_now(); kb.stop_now()
            va = ka.step(*cmd, 0.01, brake=brake)
            vb = kb.step(*cmd, 0.01, brake=brake)
            assert np.allclose(va, vb, atol=1e-9), (ct, i, va, vb)
            for wa, wb in zip(ka.wheels, kb.wheels):
                for f in ("steer", "steer_target", "speed", "speed_target", "cmd_speed", "angle", "motor_rpm", "current_a", "torque_nm"):
                    assert abs(getattr(wa, f) - getattr(wb, f)) < 1e-6, (ct, i, wa.name, f, getattr(wa, f), getattr(wb, f))
            assert abs(ka.slip_residual - kb.slip_residual) < 1e-9 and abs(ka.saturation - kb.saturation) < 1e-12


def test_native_se2_integrate():
    native = _need_native()
    import ctypes
    out = (ctypes.c_double * 3)()
    rng = np.random.default_rng(5)
    for _ in range(500):
        a = [float(v) for v in rng.uniform(-3, 3, 7)]
        a[6] = abs(a[6]) * 0.01
        if rng.random() < 0.2:
            a[5] = 0.0
        native.lib.sc_se2_integrate(*a, ctypes.addressof(out))
        assert np.allclose(list(out), se2_integrate(*a), atol=1e-12)


def test_native_photo_rays_match_mujoco():
    """光电: C 线段射线 (MuJoCo 同款盒体/圆柱几何) vs mj_multiRay，偏差应在毫米内"""
    native = _need_native()
    from sim_core.mujoco_backend import MUJOCO_AVAILABLE
    if not MUJOCO_AVAILABLE:
        _skip("未安装 mujoco")
    core = SimCore(_spec(), SCENARIO_DEFINITIONS, "grid_9_square", backend="mujoco", noise=False)
    core.set_obstacles([{"x": 1.0, "y": 0.5, "w": 0.6, "h": 0.4, "yaw": 0.3},
                        {"x": 2.5, "y": -1.5, "w": 0.5, "h": 0.5, "type": "person", "z": 1.7}])
    w = core.world
    rng = np.random.default_rng(6)
    b = w.bounds
    ang = np.linspace(-math.pi, math.pi, 90)
    worst, n = 0.0, 0
    for _ in range(300):
        ox, oy = rng.uniform(b[0], b[2]), rng.uniform(b[1], b[3])
        z = float(rng.choice([0.12, 0.3, 1.2]))
        r1 = native.raycast2d(w.mj_segments, w.mj_circles, z, ox, oy, ang, 3.0)
        r2 = core.mj.raycast2d(ox, oy, z, ang, 3.0)
        both = np.isfinite(r1) & np.isfinite(r2)
        n += int(both.sum())
        if both.any():
            worst = max(worst, float(np.abs(r1[both] - r2[both]).max()))
        # 命中/未命中只允许出现在量程边缘或盒体拐角 (掠射)
        assert int((np.isfinite(r1) != np.isfinite(r2)).sum()) <= 2
    assert n > 1000 and worst < 2e-3, worst


if __name__ == "__main__":
    fails = 0
    for k, f in list(globals().items()):
        if k.startswith("test_"):
            try:
                f()
                print("PASS", k)
            except _Skip as e:
                print("SKIP", k, e)
            except Exception as e:  # pragma: no cover
                fails += 1
                print("FAIL", k, repr(e))
    sys.exit(1 if fails else 0)
