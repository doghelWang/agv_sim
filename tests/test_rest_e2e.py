#!/usr/bin/env python3
"""三进程 REST 端到端测试 (无 ROS)：sim_server:8090 + nav_runtime:8091 + web_gateway:8088 需已启动"""
import json, sys, time, urllib.request

GW, SIM, NAV = "http://127.0.0.1:8088", "http://127.0.0.1:8090", "http://127.0.0.1:8091"
ok = True


def req(url, method="GET", body=None, accept="application/json"):
    data = json.dumps(body).encode() if body is not None else None
    r = urllib.request.Request(url, data=data, method=method, headers={"Content-Type": "application/json", "Accept": accept})
    with urllib.request.urlopen(r, timeout=5) as f:
        raw = f.read()
        return json.loads(raw) if "json" in f.headers.get("Content-Type", "") else (f.headers, raw)


def check(name, cond, info=""):
    global ok
    ok &= bool(cond)
    print(("  PASS " if cond else "  FAIL ") + name + (f"  {info}" if info else ""))


print("[1] 接口自描述")
check("sim /api/v1 routes", len(req(SIM + "/api/v1")["routes"]) > 20)
check("nav /api/v1 routes", len(req(NAV + "/api/v1")["routes"]) > 8)

print("[2] 仿真数据")
st = req(SIM + "/api/v1/state"); check("state", "truth" in st and "odom" in st)
sens = req(SIM + "/api/v1/sensors"); names = [l["name"] for l in sens["lidars"]]; check("sensors", names, names)
for n in names:
    h, raw = req(f"{SIM}/api/v1/sensors/lidars/{n}?after_seq=-1&wait=1", accept="application/octet-stream")
    meta = json.loads(h["X-Meta"]); check(f"lidar {n} 二进制", len(raw) > 0, f"type={meta['type']} bytes={len(raw)} seq={h['X-Seq']}")
io = req(SIM + "/api/v1/io"); check("io", "inputs" in io and "outputs" in io, f"DI={len(io['inputs'])} DO={len(io['outputs'])}")
req(SIM + "/api/v1/io/do/do_tower_yellow", "PUT", {"value": True})
check("写 DO", req(SIM + "/api/v1/io")["outputs"].get("do_tower_yellow") is True)
req(SIM + "/api/v1/io/do/do_tower_yellow", "PUT", {"value": False})
bp = req(SIM + "/api/v1/sensors/bumpers"); check("碰撞条", len(bp["strips"]) >= 2, [b["name"] for b in bp["strips"]])
pe = req(SIM + "/api/v1/sensors/photoelectric"); check("光电", len(pe["sensors"]) >= 1, [p["name"] for p in pe["sensors"]])
check("imu", "wz" in json.dumps(req(SIM + "/api/v1/sensors/imu")))

print("[3] 网关聚合")
t = req(GW + "/api/telemetry?full=1&scans=1"); time.sleep(0.3); t = req(GW + "/api/telemetry?full=1&scans=1")
check("arch 双进程在线", t["arch"]["sim_online"] and t["arch"]["nav_online"], json.dumps(t["arch"].get("link", {}), ensure_ascii=False)[:120])
check("robot_spec", t.get("robot_spec", {}).get("active_chassis"))
check("lidar_scans", t.get("lidar_scans"), list((t.get("lidar_scans") or {}).keys()))

print("[4] 任务闭环 (Web → 执行进程 → 仿真进程 → 回馈)")
req(SIM + "/api/v1/sim/reset", "POST", {})
req(GW + "/api/planner_type", "POST", {"planner_type": "dijkstra"})
tgt = next(s for s in t["scenario_metadata"]["stations"] if abs(s["x"]) + abs(s["y"]) > 3)
req(GW + "/api/navigate_to_pose", "POST", {"x": tgt["x"], "y": tgt["y"], "yaw": tgt.get("dock_yaw", 0.0)})
t0, status = time.time(), ""
while time.time() - t0 < 90:
    t = req(GW + "/api/telemetry"); status = t["nav_status"]
    if status in ("ARRIVED", "FAILED", "NO_PATH"):
        break
    time.sleep(0.5)
err = ((t["x"] - tgt["x"]) ** 2 + (t["y"] - tgt["y"]) ** 2) ** 0.5
check(f"到达 {tgt['id']}", status == "ARRIVED" and err < 0.1, f"status={status} err={err:.3f}m t={time.time()-t0:.1f}s")
ctl = req(SIM + "/api/v1/control"); check("仿真进程收到执行进程指令", ctl["source"].startswith("nav"), ctl["source"])
check("导航回馈已写入仿真进程", req(SIM + "/api/v1/nav/feedback").get("online"))
m = req(NAV + "/api/v1/missions"); check("任务历史", m["missions"] and m["missions"][0]["status"] == "ARRIVED")

print("[5] 车型/场景切换传播")
req(GW + "/api/chassis_type", "POST", {"chassis_type": "diff_drive"}); time.sleep(1.5)
check("仿真进程车型", req(SIM + "/api/v1/state")["chassis"] == "diff_drive")
check("执行进程已刷新模型", req(GW + "/api/telemetry?full=1")["robot_spec"]["active_chassis"] == "diff_drive")
req(GW + "/api/chassis_type", "POST", {"chassis_type": "single_steer"}); time.sleep(1.0)

print("[6] 障碍物/暂停 (原 UI 流程: 暂停 → 布置 → 恢复生效)")
req(GW + "/api/sim_pause", "POST", {}); time.sleep(0.3); check("暂停", req(SIM + "/api/v1/sim")["paused"])
req(GW + "/api/obstacles/add", "POST", {"x": 2.0, "y": 2.0}); time.sleep(0.3)
req(GW + "/api/sim_resume", "POST", {}); time.sleep(0.5); check("恢复", not req(SIM + "/api/v1/sim")["paused"])
check("障碍物下发到仿真进程", len(req(SIM + "/api/v1/world/obstacles")) >= 1, req(SIM + "/api/v1/world/obstacles"))
req(GW + "/api/obstacles/clear", "POST", {}); time.sleep(0.3)
check("障碍物清除", len(req(SIM + "/api/v1/world/obstacles")) == 0)

print("[7] 安全链: 光电/触边/急停 → 执行进程")
req(SIM + "/api/v1/sim/reset", "POST", {})
time.sleep(0.5)
st = req(SIM + "/api/v1/state")["truth"]
import math as _m
fl = next(p for p in pe["sensors"] if p["name"] == "front_left")
mx, my, a = fl["mount"]["x"], fl["mount"]["y"], st["yaw"] + fl["mount"]["yaw"]
ox = st["x"] + _m.cos(st["yaw"]) * mx - _m.sin(st["yaw"]) * my + _m.cos(a) * 0.3
oy = st["y"] + _m.sin(st["yaw"]) * mx + _m.cos(st["yaw"]) * my + _m.sin(a) * 0.3
req(SIM + "/api/v1/world/obstacles", "PUT", [{"x": ox, "y": oy, "w": 0.2, "h": 0.2}]); time.sleep(0.4)
check("光电触发 DI", req(SIM + "/api/v1/io")["inputs"].get("di_pe_front_left"))
sf = req(NAV + "/api/v1/nav")["safety"]
check("执行进程感知光电", "front_left" in sf["photo_raw"]["front"], sf["photo_raw"])
# 检测点在车体斜前方 0.3 m (低速档停车区外侧) → 光电保护包络外，不响应
check("光电保护包络: 侧前方检测点不响应", "front_left" in sf.get("photo_ignored", []) and "front_left" not in sf["photo"]["front"],
      {"photo": sf["photo"], "ignored": sf.get("photo_ignored")})
req(NAV + "/api/v1/teleop", "POST", {"vx": 0.3}); time.sleep(0.1)
req(SIM + "/api/v1/world/obstacles", "PUT", []); time.sleep(0.3)
req(SIM + "/api/v1/io/di/di_estop", "PUT", {"value": True}); time.sleep(0.3)
check("急停 → 抱闸", req(SIM + "/api/v1/io")["is_emergency_stop"] and req(NAV + "/api/v1/nav")["safety"]["estop"])
req(SIM + "/api/v1/io/di/di_estop", "PUT", {"value": False}); time.sleep(0.3)
check("急停复位", not req(SIM + "/api/v1/io")["is_emergency_stop"])
# 触边: 任务途中前方放障碍, 光电在角部看不到正前方矮箱 → 撞上触边 → BUMPER_STOP
req(SIM + "/api/v1/sim/reset", "POST", {}); time.sleep(0.3)
st = req(SIM + "/api/v1/state")["truth"]
req(NAV + "/api/v1/nav/planner", "PUT", {"type": "direct"})
gx, gy = st["x"] + 3.0 * _m.cos(st["yaw"]), st["y"] + 3.0 * _m.sin(st["yaw"])
req(SIM + "/api/v1/world/obstacles", "PUT", [{"x": st["x"] + 2.2 * _m.cos(st["yaw"]), "y": st["y"] + 2.2 * _m.sin(st["yaw"]), "w": 0.3, "h": 0.3, "z": 0.08}])
req(NAV + "/api/v1/missions", "POST", {"x": gx, "y": gy, "yaw": st["yaw"]})
t0 = time.time(); s = ""
while time.time() - t0 < 20:
    s = req(NAV + "/api/v1/nav")["status"]
    if s in ("BUMPER_STOP", "ARRIVED", "FAILED"):
        break
    time.sleep(0.2)
check("低矮障碍 (激光盲区) → 触边 → BUMPER_STOP", s == "BUMPER_STOP", s)
req(SIM + "/api/v1/world/obstacles", "PUT", []); req(SIM + "/api/v1/sim/reset", "POST", {})
req(NAV + "/api/v1/nav/planner", "PUT", {"type": "dijkstra"})

print("[8] 模型补全 + 相机类传感器 (单目/双目/ToF)")
ed = req(SIM + "/api/v1/model/editor")
check("完整度审计", ed["audit"]["summary"]["total"] > 10, ed["audit"]["summary"])
orig = ed["overrides"]
orig["sensors"] = {k: v for k, v in (orig.get("sensors") or {}).items() if not k.startswith("e2e_")}   # 清理上次中断残留
ov = json.loads(json.dumps(orig))
ov.setdefault("sensors", {}).update({
    "e2e_cam": {"type": "camera", "stype": "camera", "_added": True, "x": 1.3, "y": 0, "z": 0.9, "pitch": 0.1},
    "e2e_stereo": {"type": "stereo", "stype": "stereo", "_added": True, "x": 1.3, "y": 0, "z": 0.5},
    "e2e_tof": {"type": "tof", "stype": "tof", "_added": True, "x": 1.3, "y": 0, "z": 0.3, "pitch": 0.5}})
pv = req(SIM + "/api/v1/model/preview", "POST", ov)
check("试算 URDF 含光学坐标系", "e2e_stereo_right_optical_frame" in pv["urdf"] and "e2e_tof_optical_frame" in pv["urdf"])
req(SIM + "/api/v1/model/overrides", "PUT", ov)
cams = {c["name"]: c for c in req(SIM + "/api/v1/sensors/cameras")["cameras"]}
check("相机已安装", {"e2e_cam", "e2e_stereo", "e2e_tof"} <= set(cams), list(cams))
for n, st, fm, ct in (("e2e_cam", "rgb", "jpeg", "image/"), ("e2e_stereo", "right", "png", "image/png"), ("e2e_stereo", "depth", "raw", "octet"),
                      ("e2e_tof", "points", "raw", "octet")):
    h, raw = req(f"{SIM}/api/v1/sensors/cameras/{n}?stream={st}&format={fm}&after_seq=-1&wait=3", accept="*/*")
    meta = json.loads(h["X-Meta"])
    check(f"{n}/{st} ({fm})", ct in h["Content-Type"] and len(raw) > 1000, f"{len(raw)} B {meta.get('encoding', '')} {meta['width']}x{meta['height']}")
h, raw = req(f"{SIM}/api/v1/sensors/cameras/e2e_tof?stream=depth&format=raw", accept="*/*")
import struct
vals = [v for v in struct.unpack(f"<{len(raw) // 4}f", raw) if v == v]
check("ToF 深度在量程内", vals and 0.1 <= min(vals) and max(vals) <= 4.0 + 0.1, f"{min(vals):.2f}~{max(vals):.2f} m, 有效 {len(vals)}")
urdf = urllib.request.urlopen(SIM + "/api/v1/model/urdf").read().decode()
check("URDF 已更新", "e2e_cam_optical_frame" in urdf)
req(SIM + "/api/v1/model/overrides", "PUT", orig)
check("恢复原模型", "e2e_cam" not in {c["name"] for c in req(SIM + "/api/v1/sensors/cameras")["cameras"]})

print("\n结果:", "全部通过" if ok else "存在失败")
sys.exit(0 if ok else 1)
