#!/usr/bin/env python3
"""
仿真进程 REST API v1  (默认 http://0.0.0.0:8090)

资源一览 (详见 docs/API.md)
  GET  /api/v1/health                         进程健康/仿真时钟
  GET  /api/v1/model                          机器人模型 (cmodel 解析结果 + 当前车型轮组)
  GET  /api/v1/model/urdf                     当前车型 URDF (application/xml)
  PUT  /api/v1/model/chassis                  {"type": "single_steer|diff_drive|dual_steer|cmodel"}
  POST /api/v1/model/reload                   {"cmodel_path": "...", "load": "full|idle"} 重新解析并重建
  GET  /api/v1/world                          场景 (墙/货架/工位/拓扑/障碍物)
  PUT  /api/v1/world/scenario                 {"id": "grid_9_square"}
  GET  /api/v1/world/map?part=pgm|yaml        Nav2 栅格地图 (执行进程下载后供 map_server 使用)
  GET  /api/v1/world/obstacles | PUT          [{"x","y","w","h","z","yaw","type"}]
  GET  /api/v1/sim | PUT                      {"paused": bool, "rtf": float}
  POST /api/v1/sim/reset                      {"x","y","yaw"} (缺省 = 场景原点)
  GET  /api/v1/state                          真值/里程计/map→odom/关节/碰撞
  GET  /api/v1/sensors                        传感器清单 (安装位姿/型号/频率/最新帧序号)
  GET  /api/v1/sensors/lidars/{name}          激光帧 ?after_seq=N&wait=0.2 长轮询；Accept: application/octet-stream 取二进制
  GET  /api/v1/sensors/scan                   融合 360° 扫描 (机体系)
  PUT  /api/v1/sensors/lidar_config           {"beams","range_max","freq_hz","sensor_resolution_deg"}
  GET  /api/v1/sensors/imu                    IMU
  GET  /api/v1/sensors/codes                  读码相机/工位标签
  GET  /api/v1/sensors/bumpers                防撞触边 (碰撞条) 状态/多边形/计数
  GET  /api/v1/sensors/photoelectric          光电传感器 (检测距离/触发/DI 映射)
  GET  /api/v1/sensors/cameras                相机清单 (单目/双目/ToF，内参 K、流、帧序号)
  GET  /api/v1/sensors/cameras/{name}         相机帧 ?stream=rgb|left|right|depth|amplitude|points&format=jpeg|png|raw (长轮询)
  GET  /api/v1/model/editor                   模型补全编辑器数据 (完整度审计/传感器清单/覆盖)
  PUT  /api/v1/model/overrides                保存并应用人工补全 (重建 URDF 与仿真)
  POST /api/v1/model/preview                  试算补全 (不保存)
  GET  /api/v1/model/sensor_template?type=    新增传感器默认参数
  GET  /api/v1/io | PUT {"di":{},"do":{}}     工业 IO (急停/触边/光电/抱闸/塔灯/顶升)
  PUT  /api/v1/io/{kind}/{name}               {"value": true}
  GET  /api/v1/control                        当前执行指令/看门狗
  PUT  /api/v1/control/cmd_vel                {"vx","vy","wz","source"}  ← 执行进程回馈控制量
  GET  /api/v1/nav/feedback | PUT             执行进程回馈导航状态 (任务/路径/Nav2)
  GET  /api/v1/events?since=N                 仿真事件
  GET  /api/v1/snapshot?scans=1               Web 汇总快照
  GET  /api/v1/stream?hz=50&io_hz=20&scans=1  推送流: 状态二进制帧 + IO/光电 + 2D 激光/融合 (执行进程用，替代轮询)
"""

import json
import os
import struct
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from common.rest import ApiError, BinaryBody, RawBody, RestServer  # noqa: E402
from sim_server.service import API_VERSION, SimService, ranges_json  # noqa: E402


def _jsonable_scan(d: dict) -> dict:
    out = {}
    for k, v in d.items():
        if isinstance(v, np.ndarray):
            if k in ("ranges", "slice"):
                out[k] = ranges_json(v, 4)
            elif k == "points":
                out[k] = np.round(v.astype(np.float64), 4).ravel().tolist()
                out["point_fields"] = ["x", "y", "z"]
            elif k == "offset_time":
                continue
            else:
                out[k] = v.tolist()
        else:
            out[k] = v
    return out


def _binary_scan(d: dict) -> BinaryBody:
    """二进制帧: 2D = float32 ranges (inf 表示无回波)；3D = 每点 [x,y,z,intensity] float32 + line uint8 数组"""
    meta = {k: v for k, v in d.items() if not isinstance(v, np.ndarray)}
    if d.get("type", "2d") == "2d":         # 融合扫描 (merged) 没有 type 字段
        body = np.asarray(d["ranges"], dtype="<f4").tobytes()
        meta["layout"] = "float32[ranges]"
    else:
        P = d["points"].astype("<f4")
        xyzi = np.concatenate([P, d["intensity"].astype("<f4")[:, None]], axis=1)
        body = xyzi.tobytes() + d["line"].astype("u1").tobytes()
        meta["layout"] = "float32[N,4](x,y,z,intensity) + uint8[N](line)"
        meta["count"] = int(len(P))
    return BinaryBody(body, {"X-Seq": str(d["seq"]), "X-Stamp": str(d["t"]), "X-Meta": json.dumps(meta, ensure_ascii=True)})


def _camera_body(d: dict, stream, fmt):
    """相机帧编码。rgb/left/right: jpeg(默认)/png/raw(uint8 HxWx3)；depth: png(16bit mm, 默认)/jpeg(伪彩)/raw(float32 m, NaN 无效)；
    amplitude: png16/raw uint16；points: raw float32 [N,3] (光学坐标系)"""
    from sim_core.imgcodec import colorize_depth, image_mime, jpeg, png
    frames, info = d["frames"], d["info"]
    stream = stream or next(iter(frames))
    if stream not in frames:
        raise ApiError(400, f"流 {stream} 不存在，可用: {list(frames)}")
    a = frames[stream]
    meta = {"camera": info["name"], "type": info["type"], "stream": stream, "frame_id": info["frame_id"],
            "width": info["width"], "height": info["height"], "K": info["K"], "pose": d["pose"]}
    if stream == "right":
        meta["frame_id"] = info["name"] + "_right_optical_frame"
    if stream in ("rgb", "left", "right"):
        fmt = fmt or "jpeg"
        if fmt == "raw":
            body, ctype, meta["encoding"] = a.tobytes(), "application/octet-stream", "rgb8"
        elif fmt == "png":
            body, ctype = png(a), "image/png"
        else:
            body, ctype = jpeg(a), image_mime("jpeg")
    elif stream == "depth":
        fmt = fmt or "png"
        rng = info.get("range") or [0.1, 10.0]
        if fmt == "raw":
            body, ctype, meta["encoding"] = a.astype("<f4").tobytes(), "application/octet-stream", "32FC1"
        elif fmt == "jpeg":
            body, ctype = jpeg(colorize_depth(a, rng[0], rng[1])), image_mime("jpeg")
        else:
            mm = np.nan_to_num(a * 1000.0, nan=0.0).clip(0, 65535).astype(np.uint16)
            body, ctype, meta["encoding"] = png(mm), "image/png", "16UC1 (mm)"
    elif stream == "amplitude":
        if fmt == "raw":
            body, ctype, meta["encoding"] = a.astype("<u2").tobytes(), "application/octet-stream", "16UC1"
        elif fmt == "jpeg":
            g = (np.clip(a.astype(np.float32) / max(1.0, float(np.percentile(a, 99))), 0, 1) * 255).astype(np.uint8)
            body, ctype = jpeg(np.stack([g, g, g], -1)), image_mime("jpeg")
        else:
            body, ctype = png(a.astype(np.uint16)), "image/png"
    else:   # points
        body, ctype = a.astype("<f4").tobytes(), "application/octet-stream"
        meta.update({"encoding": "float32[N,3] xyz", "count": int(len(a))})
    return BinaryBody(body, {"X-Seq": str(d["seq"]), "X-Stamp": str(d["t"]), "X-Meta": json.dumps(meta, ensure_ascii=True)},
                      content_type=ctype)


def build_api(svc: SimService, port: int = 8090) -> RestServer:
    api = RestServer("sim", port=port, title="仿真进程 sim_server",
                     description="MuJoCo 物理仿真: 车型、场景、障碍物、传感器 (激光/相机/IMU/光电/触边/读码)、IO、速度指令、推送流")
    R = api.route

    R("GET", "/api/v1/health", lambda q: {"service": "sim", "api_version": API_VERSION, "ok": True,
                                          "sim_time": svc.core.t, "rtf": svc.core.rtf, "paused": svc.core.paused}, "健康检查")
    # ---- 模型
    R("GET", "/api/v1/model", lambda q: svc.model(), "机器人模型")
    R("GET", "/api/v1/model/urdf", lambda q: RawBody(svc.urdf().encode("utf-8"), "application/xml; charset=utf-8"), "URDF")

    # ---- 模型补全 (人工)
    R("GET", "/api/v1/model/editor", lambda q: svc.model_editor(), "补全编辑器数据: 完整度审计/传感器清单/当前覆盖")
    R("GET", "/api/v1/model/audit", lambda q: svc.model_editor()["audit"], "完整度审计 (参数来源/缺失)")
    R("GET", "/api/v1/model/overrides", lambda q: svc.overrides, "人工补全覆盖 (model_overrides.json)")

    def put_overrides(q):
        ov = q.json
        if not isinstance(ov, dict):
            raise ApiError(400, "需要 JSON 对象")
        try:
            return svc.apply_overrides(ov, save=True)
        except (ValueError, KeyError, TypeError) as e:
            raise ApiError(400, f"覆盖参数无效: {e}")
    R("PUT", "/api/v1/model/overrides", put_overrides, "保存并应用人工补全 (重建模型/URDF/仿真)")

    def preview(q):
        try:
            return svc.preview_overrides(q.json)
        except (ValueError, KeyError, TypeError) as e:
            raise ApiError(400, f"覆盖参数无效: {e}")
    R("POST", "/api/v1/model/preview", preview, "试算补全结果 (不保存): spec/审计/URDF")

    def template(q):
        from model_overrides import sensor_template
        try:
            return sensor_template(q.q("type", "camera"))
        except ValueError as e:
            raise ApiError(400, str(e))
    R("GET", "/api/v1/model/sensor_template", template, "新增传感器默认参数 ?type=lidar2d|lidar3d|camera|stereo|tof|photoelectric|codeReader")

    def put_chassis(q):
        t = q.json.get("type")
        if t not in (None, "", "cmodel", "diff_drive", "single_steer", "dual_steer", "multi_steer"):
            raise ApiError(400, f"未知车型 {t}")
        return {"chassis": svc.set_chassis(t)}
    R("PUT", "/api/v1/model/chassis", put_chassis, "切换车型")

    def reload(q):
        b = q.json
        try:
            return svc.reload_model(b.get("cmodel_path"), b.get("load", "full"), bool(b.get("save", True)))
        except FileNotFoundError as e:
            raise ApiError(404, str(e))
    R("POST", "/api/v1/model/reload", reload, "重新解析 cmodel 并重建仿真")

    # ---- 世界
    R("GET", "/api/v1/world", lambda q: svc.world(), "场景")

    def put_scenario(q):
        sid = q.json.get("id") or q.json.get("scenario")
        try:
            svc.set_scenario(sid)
        except KeyError:
            raise ApiError(404, f"未知场景 {sid}")
        return {"scenario": sid}
    R("PUT", "/api/v1/world/scenario", put_scenario, "切换场景")
    R("GET", "/api/v1/world/obstacles", lambda q: svc.world()["obstacles"], "动态障碍物")

    def scene_load(q):
        b = q.json
        if b.get("scene"):
            sid = svc.load_scene(b["scene"])
        elif b.get("hub_scene_id"):
            from sim_server.bootstrap import pull_scene
            hub = b.get("hub") or os.environ.get("HUB_API", "")
            if not hub:
                raise ApiError(400, "未配置平台地址 HUB_API")
            p = pull_scene(hub.rstrip("/"), os.path.dirname(os.path.abspath(svc.config_path)), b["hub_scene_id"])
            with open(p, encoding="utf-8") as f:
                sid = svc.load_scene(json.load(f))
        else:
            raise ApiError(400, "需要 scene (场景定义) 或 hub_scene_id")
        return {"scenario": sid}
    R("POST", "/api/v1/scene/load", scene_load, "加载场景定义 {scene} 或平台场景 {hub_scene_id}")

    def model_load(q):
        b = q.json
        hub = b.get("hub") or os.environ.get("HUB_API", "")
        if not hub or not b.get("model_id"):
            raise ApiError(400, "需要 model_id 且配置了平台地址 HUB_API")
        return svc.load_model_from_hub(hub.rstrip("/"), b["model_id"], b.get("version", ""))
    R("POST", "/api/v1/model/load", model_load, "从平台加载模型 {model_id, version}")
    R("GET", "/api/v1/instance", lambda q: svc.instance(), "实例信息 (平台模型/场景)")

    def get_map(q):
        m = svc.occupancy_map(q.q("res", 0.05, float))
        if q.q("part", "pgm") == "yaml":
            return RawBody(m["yaml"].encode(), "text/yaml; charset=utf-8")
        return BinaryBody(m["pgm"], {"X-Meta": json.dumps({k: m[k] for k in ("id", "width", "height", "resolution", "origin")})},
                          content_type="image/x-portable-graymap")
    R("GET", "/api/v1/world/map", get_map, "Nav2 栅格地图 ?part=pgm|yaml&res=0.05")

    def put_obs(q):
        obs = q.json
        if not isinstance(obs, list):
            raise ApiError(400, "需要障碍物数组")
        svc.set_obstacles(obs)
        return {"count": len(obs)}
    R("PUT", "/api/v1/world/obstacles", put_obs, "设置动态障碍物 (请求体为数组，元素 {x, y, w, h, z, yaw, motion})")

    # ---- 仿真控制
    R("GET", "/api/v1/sim", lambda q: svc.sim_status(), "仿真状态/性能")

    def put_sim(q):
        b = q.json
        svc.set_sim(b.get("paused"), b.get("rtf"))
        return svc.sim_status()
    R("PUT", "/api/v1/sim", put_sim, "暂停/实时因子")

    def reset(q):
        b = q.json
        svc.reset(b.get("x"), b.get("y"), b.get("yaw"))
        return svc.state()["truth"]
    R("POST", "/api/v1/sim/reset", reset, "位姿复位")

    # ---- 状态 & 传感器
    R("GET", "/api/v1/state", lambda q: svc.state(), "真值/里程计/关节/碰撞")
    R("GET", "/api/v1/sensors", lambda q: svc.sensors(), "传感器清单")

    def get_lidar(q, name=None):
        name = name or q.params["name"]
        try:
            d = svc.wait_lidar(name, q.q("after_seq", -1, int), q.q("wait", 0.0, float))
        except KeyError:
            raise ApiError(404, f"无此激光 {name}")
        if d is None:
            raise ApiError(503, "尚无数据", "no_data")
        if "octet-stream" in (q.headers.get("Accept") or "") or q.q("format") == "bin":
            return _binary_scan(d)
        return _jsonable_scan(d)
    R("GET", "/api/v1/sensors/lidars/{name}", get_lidar, "激光帧 (长轮询/二进制)")
    R("GET", "/api/v1/sensors/scan", lambda q: get_lidar(q, "merged"), "融合扫描")
    R("PUT", "/api/v1/sensors/lidar_config", lambda q: svc.set_lidar_config(q.json), "激光配置 {beams, range_max, freq_hz, sensor_resolution_deg}")

    # ---- 相机类 (单目 / 双目 / ToF)
    R("GET", "/api/v1/sensors/cameras", lambda q: {"cameras": svc.cameras()}, "相机清单 (内参/流/帧序号)")

    def get_camera(q):
        name = q.params["name"]
        try:
            d = svc.wait_camera(name, q.q("after_seq", -1, int), q.q("wait", 0.0, float))
        except KeyError:
            raise ApiError(404, f"无此相机 {name}")
        if d is None:
            raise ApiError(503, "尚无图像", "no_data")
        return _camera_body(d, q.q("stream", None), q.q("format", None))
    R("GET", "/api/v1/sensors/cameras/{name}", get_camera, "相机帧 ?stream=rgb|left|right|depth|amplitude|points&format=jpeg|png|raw")
    R("GET", "/api/v1/sensors/imu", lambda q: svc.imu(), "IMU")
    R("GET", "/api/v1/sensors/codes", lambda q: svc.codes(), "读码/工位标签")
    R("GET", "/api/v1/sensors/bumpers", lambda q: svc.bumpers(), "防撞触边 (碰撞条)")
    R("GET", "/api/v1/sensors/photoelectric", lambda q: svc.photoelectric(), "光电传感器")

    # ---- IO
    R("GET", "/api/v1/io", lambda q: svc.io(), "IO 状态")
    R("PUT", "/api/v1/io", lambda q: svc.set_io(q.json.get("di"), q.json.get("do")), "批量写 IO")

    def put_io_one(q):
        kind, name = q.params["kind"], q.params["name"]
        v = bool(q.json.get("value"))
        if kind not in ("di", "do"):
            raise ApiError(400, "kind 必须为 di 或 do")
        return svc.set_io(**{kind: {name: v}})
    R("PUT", "/api/v1/io/{kind}/{name}", put_io_one, "写单个 IO")

    # ---- 控制 (执行进程回馈)
    R("GET", "/api/v1/control", lambda q: svc.control(), "当前指令/看门狗")

    def put_cmd(q):
        b = q.json
        try:
            return svc.set_cmd(float(b.get("vx", 0.0)), float(b.get("vy", 0.0)), float(b.get("wz", 0.0)), str(b.get("source", "nav")))
        except (TypeError, ValueError):
            raise ApiError(400, "vx/vy/wz 必须为数值")
    R("PUT", "/api/v1/control/cmd_vel", put_cmd, "速度指令 (执行进程 → 仿真)")

    R("GET", "/api/v1/nav/feedback", lambda q: svc.get_nav_feedback(), "导航回馈")
    R("PUT", "/api/v1/nav/feedback", lambda q: svc.set_nav_feedback(q.json) or {"ok": True}, "执行进程回馈导航状态 (请求体为导航回馈对象，字段同 GET /api/v1/nav/feedback)")

    R("GET", "/api/v1/events", lambda q: {"events": svc.events_since(q.q("since", 0, int))}, "仿真事件")
    R("GET", "/api/v1/snapshot", lambda q: svc.snapshot(q.q("scans", False, bool)), "Web 汇总快照")

    def stream(h):
        """GET /api/v1/stream?hz=50&io_hz=20&scans=1: 长连接推送 (状态二进制帧 + IO/光电 + 2D 激光/融合扫描)，
        替代执行进程的状态/IO/光电轮询与激光长轮询"""
        import time as _t
        import urllib.parse as _u
        qs = _u.parse_qs(_u.urlparse(h.path).query)
        hz = max(1.0, min(100.0, float((qs.get("hz") or ["50"])[0])))
        io_hz = max(0.0, min(50.0, float((qs.get("io_hz") or ["20"])[0])))
        scans = (qs.get("scans") or ["0"])[0] == "1"
        sent = {}
        h.send_response(200)
        h.send_header("Content-Type", "application/x-agv-stream")
        h.send_header("Cache-Control", "no-cache, no-store")
        h.send_header("Connection", "close")
        h.end_headers()
        h.close_connection = True
        meta_key, next_io, t_next = None, 0.0, _t.perf_counter()
        try:
            while not svc._stop.is_set():
                frame, meta, meta_key = svc.state_frames(meta_key)
                out = (meta or b"") + frame
                now = _t.perf_counter()
                if io_hz > 0 and now >= next_io:
                    out += svc.io_frame()
                    next_io = now + 1.0 / io_hz
                if scans:
                    out += svc.scan_frames(sent)
                h.wfile.write(out)
                t_next += 1.0 / hz
                d = t_next - _t.perf_counter()
                if d > 0:
                    _t.sleep(d)
                else:
                    t_next = _t.perf_counter()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        return True
    api.mount("/api/v1/stream", stream)
    return api


def main():
    import argparse
    ap = argparse.ArgumentParser(description="AGV 仿真进程 (REST API)")
    ap.add_argument("--config", default=os.environ.get("ROBOT_CONFIG", os.path.join(ROOT, "robot_config.json")))
    ap.add_argument("--port", type=int, default=int(os.environ.get("SIM_API_PORT", "8090")))
    ap.add_argument("--scenario", default=os.environ.get("SIM_SCENARIO", "grid_9_square"))
    a = ap.parse_args()
    scenario = a.scenario
    sf = os.environ.get("SIM_SCENE_FILE")
    if sf and os.path.exists(sf):
        from planning.dijkstra_planner import register_scenario
        with open(sf, encoding="utf-8") as f:
            scenario = register_scenario(json.load(f))
    svc = SimService(a.config, scenario)
    svc.start()
    svc.start_cmd_udp(a.port)
    api = build_api(svc, a.port)
    print(f"[sim_server] REST API http://0.0.0.0:{a.port}/api/v1  车型={svc.core.chassis_type} 场景={svc.core.scenario_id} "
          f"激光={[l.name for l in svc.core.lidars + svc.core.lidars3d]}", flush=True)
    try:
        api.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
