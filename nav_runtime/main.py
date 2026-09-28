#!/usr/bin/env python3
"""
执行进程 (导航运行时)  —— ROS 2 / Nav2 + 任务执行，默认 REST 端口 8091

  与仿真进程之间: 仅 REST (SimLink)
  与 Web 之间   : 仅 REST (本文件 API)
  进程内部      : ROS 2 话题只用于驱动 Nav2 (ros_bridge)，无 ROS 环境时自动降级为纯导引模式

REST API v1 (详见 docs/API.md)
  GET    /api/v1/health
  GET    /api/v1/nav                         执行状态 (规划器/当前任务/Nav2/ROS 图/链路统计)
  PUT    /api/v1/nav/planner                 {"type": "dijkstra|astar|direct|nav2"}
  GET    /api/v1/missions                    最近任务
  POST   /api/v1/missions                    {"x","y","yaw"} 或 {"station": "S1"}，可带 "planner"
  GET    /api/v1/missions/current
  GET    /api/v1/missions/{id}
  DELETE /api/v1/missions/current | /{id}    取消任务
  POST   /api/v1/teleop                      {"vx","vy","wz"}  手动遥控 (转发仿真进程)
  GET    /api/v1/events?since=N
  GET    /api/v1/ros                         ROS 2 节点/话题 (执行进程内部)
"""

import os
import signal
import sys

# ROS 2 只在本机通信：同一局域网里多台设备 (树莓派/手机) 用相同 ROS_DOMAIN_ID 时 DDS 会互相发现，
# 出现两个 follow_path 动作服务器、TF 串话 (map→odom 来自另一台设备)。需要跨机 ROS 通信时设 ROS_LOCALHOST_ONLY=0
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from common.rest import ApiError, BinaryBody, RawBody, RestClient, RestServer, wait_for  # noqa: E402
from nav_runtime.navigator import Navigator  # noqa: E402
from nav_runtime.sim_link import SimLink  # noqa: E402


def log(msg):
    print(msg, flush=True)


def build_api(nav: Navigator, link: SimLink, ros=None, port: int = 8091) -> RestServer:
    api = RestServer("nav", port=port)
    R = api.route
    R("GET", "/api/v1/health", lambda q: {"service": "nav", "ok": True, "sim_link": link.online,
                                          "ros": ros is not None, "nav2_ready": nav.nav2.status().get("server_ready", False)}, "健康检查")

    def status(q):
        fb = nav.feedback()
        fb["link"] = {"sim_url": link.url, "online": link.online, **link.stats}
        fb["ros"] = ros.graph() if ros else None
        fb["ros_bridge"] = ros.bridge_status() if ros else None
        return fb
    R("GET", "/api/v1/nav", status, "执行状态")

    def put_planner(q):
        t = q.json.get("type")
        if not nav.set_planner_type(t):
            raise ApiError(400, f"规划器不可用: {t}")
        return {"planner": nav.active_planner}
    R("PUT", "/api/v1/nav/planner", put_planner, "切换规划器")

    R("GET", "/api/v1/missions", lambda q: {"missions": [nav.mission_view(m) for m in list(nav.missions)][::-1]}, "最近任务")

    def post_mission(q):
        b = q.json
        if "station" in b:
            st = next((s for s in link.world.get("stations", []) if s["id"] == b["station"]), None)
            if not st:
                raise ApiError(404, f"未知工位 {b['station']}")
            x, y, yaw = st["x"], st["y"], st.get("dock_yaw", 0.0)
        else:
            try:
                x, y, yaw = float(b["x"]), float(b["y"]), float(b.get("yaw", 0.0))
            except (KeyError, TypeError, ValueError):
                raise ApiError(400, "需要 x, y (可选 yaw) 或 station")
        try:
            return nav.submit(x, y, yaw, b.get("planner"))
        except ValueError as e:
            raise ApiError(400, str(e))
    R("POST", "/api/v1/missions", post_mission, "下发任务")

    def cur(q):
        v = nav.mission_view()
        if not v:
            raise ApiError(404, "无任务")
        return v
    R("GET", "/api/v1/missions/current", cur, "当前任务")

    def get_m(q):
        mid = int(q.params["id"]) if q.params["id"].isdigit() else -1
        m = next((m for m in nav.missions if m["id"] == mid), None)
        if not m:
            raise ApiError(404, f"无任务 {q.params['id']}")
        return nav.mission_view(m)
    R("GET", "/api/v1/missions/{id}", get_m, "任务详情")

    def cancel(q):
        nav.cancel_nav()
        return {"canceled": True}
    R("DELETE", "/api/v1/missions/current", cancel, "取消当前任务")
    R("DELETE", "/api/v1/missions/{id}", cancel, "取消任务")

    def teleop(q):
        b = q.json
        ok = link.send_cmd(float(b.get("vx", 0)), float(b.get("vy", 0)), float(b.get("wz", 0)), source="teleop")
        return {"forwarded": ok}
    R("POST", "/api/v1/teleop", teleop, "手动遥控")
    # ---- 任务流 (工步序列)
    def tf_run(q):
        flow = q.json.get("flow") or q.json
        try:
            return nav.taskflow.run(flow)
        except ValueError as e:
            raise ApiError(400, str(e))
    R("POST", "/api/v1/taskflows/run", tf_run, "执行任务流 {flow:{id,name,tid,loop,steps:[{type,target,speed,seconds}]}}")
    R("POST", "/api/v1/taskflows/stop", lambda q: nav.taskflow.stop(reason=q.json.get("reason", "操作员终止")), "终止任务流")
    R("GET", "/api/v1/taskflows/current", lambda q: nav.taskflow.view(), "任务流进度")

    def plan_preview(q):
        """路线预览 (不下发): {points:[{x,y}...]} 依次串联规划，与实际执行同一规划器 (含车体过弯可行性)"""
        pts = [(float(p["x"]), float(p["y"])) for p in (q.json.get("points") or [])]
        with nav.lock:
            obstacles = list(nav.dynamic_obstacles) if q.json.get("obstacles") else []
        legs = []
        for a, b in zip(pts, pts[1:]):
            r = nav.dijkstra_planner.plan_route(a, b, obstacles)
            legs.append({"points": [{"x": round(x, 3), "y": round(y, 3)} for x, y in r["points"]], "labels": r["labels"], "length": r["length"]})
        return {"legs": legs, "length": round(sum(l["length"] for l in legs), 2)}
    R("POST", "/api/v1/plan", plan_preview, "路线预览 (拓扑规划，不下发)")

    def put_safety(q):
        """保护空间运行时调整: {protection: {...}} 或旧字段 {stop_dist, slow_dist, slow_speed, corridor_margin, corner_radius, enabled}"""
        b = q.json or {}
        patch = dict(b.get("protection") or {})
        if "enabled" in b:
            patch["enabled"] = bool(b["enabled"])
        for k in ("slow_speed", "corner_radius"):
            if k in b:
                patch[k] = max(0.0, float(b[k]))
        if any(k in b for k in ("stop_dist", "corridor_margin", "slow_dist")):
            # 旧接口: 统一改各档 (停车距离不小于原值时按比例放大)
            fs = [dict(f) for f in nav.prot["fields"]]
            for f in fs:
                if "stop_dist" in b:
                    f["front"] = max(f["front"], float(b["stop_dist"])) if f is not fs[0] else float(b["stop_dist"])
                if "corridor_margin" in b:
                    f["side"] = float(b["corridor_margin"])
            patch["fields"] = fs
            if "slow_dist" in b and "stop_dist" in b and float(b["stop_dist"]) > 0:
                patch["slow_ratio"] = max(1.0, float(b["slow_dist"]) / float(b["stop_dist"]))
        view = nav.set_protection(patch)
        f = nav.prot["fields"]
        nav.event_hub.emit("safety", "SAFETY_PARAMS", "info", "保护空间参数更新 (本次运行)",
                           "防护区 " + " / ".join(f"{x['name']}≤{x['v_max']}: 前{x['front']} 后{x['rear']} 侧{x['side']}" for x in f) +
                           f"；车体净空 {nav.prot['body_margin']} m", {})
        return dict(nav.safety_params, protection_view=view)
    R("GET", "/api/v1/safety/params", lambda q: dict(nav.safety_params, protection_view=nav.protection_view()), "保护空间 / 避障参数")
    R("PUT", "/api/v1/safety/params", put_safety, "运行时调整保护空间 {protection:{...}} (兼容旧字段)")
    # ---- 定位 (激光 SLAM + 里程计融合)
    slam = nav.slam
    R("GET", "/api/v1/slam", lambda q: slam.status(), "定位状态: 模式/位姿/协方差/匹配得分/相对真值误差")

    def slam_map(q):
        part = q.q("part", "grid")
        if part in ("pgm", "yaml"):
            pgm, yaml = slam.pgm_yaml()
            if pgm is None:
                raise ApiError(404, "地图为空")
            if part == "pgm":
                return BinaryBody(pgm, {"Content-Disposition": f'attachment; filename="{slam.scenario or "slam"}.pgm"'}, "image/x-portable-graymap")
            return RawBody(yaml.encode(), "text/plain; charset=utf-8", {"Content-Disposition": f'attachment; filename="{slam.scenario or "slam"}.yaml"'})
        return slam.map_payload(step=max(1, q.q("step", 1, int)))
    R("GET", "/api/v1/slam/map", slam_map, "SLAM 地图 ?part=grid(默认, base64 0未知/1空闲/2占据)|pgm|yaml &step=N 下采样")

    def slam_mode(q):
        try:
            note = slam.set_mode(str(q.json.get("mode", "")))
        except ValueError as e:
            raise ApiError(400, str(e))
        return dict(slam.status(), note=note)
    R("POST", "/api/v1/slam/mode", slam_mode, "切换定位模式 {mode: slam|localization|odom|ground_truth}")

    def slam_save(q):
        try:
            return {"saved": slam.save(), "status": slam.status()}
        except ValueError as e:
            raise ApiError(400, str(e))
    R("POST", "/api/v1/slam/save", slam_save, "保存当前场景的 SLAM 地图 (之后该场景自动进入定位模式)")

    def slam_reset(q):
        if q.json.get("delete_saved"):
            slam.delete_saved()
        slam.reset_map()
        return slam.status()
    R("POST", "/api/v1/slam/reset", slam_reset, "清空 SLAM 地图重新建图 {delete_saved: bool}")

    def slam_init(q):
        b = q.json
        try:
            slam.set_initial_pose(float(b["x"]), float(b["y"]), float(b.get("yaw", 0.0)))
        except (KeyError, TypeError, ValueError):
            raise ApiError(400, "需要 x, y (可选 yaw)")
        return slam.status()
    R("POST", "/api/v1/slam/initialpose", slam_init, "设置初始位姿 {x, y, yaw}")
    R("GET", "/api/v1/events", lambda q: nav.event_hub.get_events(since_id=q.q("since", 0, int), limit=q.q("limit", 200, int)), "执行事件")
    R("GET", "/api/v1/ros", lambda q: ros.graph() if ros else {"nodes": [], "topics": [], "note": "ROS 未启用"}, "ROS 图")
    return api


def main():
    sim_url = os.environ.get("SIM_API", "http://127.0.0.1:8090")
    port = int(os.environ.get("NAV_API_PORT", "8091"))
    link = SimLink(sim_url, log=log)
    log(f"[nav_runtime] 连接仿真进程 {sim_url} ...")
    wait_for(RestClient(sim_url), timeout=120, log=log)
    nav = Navigator(link, log=log)
    from nav_runtime.taskflow import TaskFlowRunner
    nav.taskflow = TaskFlowRunner(nav, link)
    link.start()
    time.sleep(1.0)

    ros = None
    use_ros = os.environ.get("NAV_USE_ROS", "1") == "1"
    if use_ros:
        try:
            from nav2_bridge import Nav2Bridge
            from nav_runtime import cpp_ros
            rclpy_free = cpp_ros.usable(link)
            if rclpy_free:        # 核心模式: ROS 通信全在 C++ 桥接，本进程不加载 rclpy
                ros = cpp_ros.CppRos(link, nav, log=log)
            else:
                import rclpy
                from nav_runtime.ros_bridge import RosBridge
                rclpy.init()
                ros = RosBridge(link, nav)
            nav.nav2 = Nav2Bridge(ros)
            if nav.slam.ext is not None:             # slam_toolbox 提供 /map，Nav2 不再加载场景地图
                nav.nav2.supervisor.map_source = "topic"
            nav.map_dir = os.path.join(ROOT, "maps")
            link.fetch_map(nav.map_dir)              # 当前场景地图 (之后随场景切换自动更新)

            def regen_nav2(model):
                # Nav2 参数由仿真进程提供的模型实时生成 (车型切换 → 重启 Nav2)
                try:
                    from tools.gen_nav2_params import write_all
                    write_all(model, os.path.join(ROOT, "nav2"), os.environ.get("SIM_USE_SIM_TIME", "0") == "1")
                except Exception as e:
                    log(f"[nav_runtime] 生成 Nav2 参数失败: {e}")
                sup = nav.nav2.supervisor
                ct = model.get("active_chassis", "single_steer")
                if sup.enabled and (not sup.running() or sup.chassis != ct):
                    if not link.world:
                        link.refresh_world()
                    w = link.world or {}
                    link.fetch_map(nav.map_dir, w.get("id"))
                    o = w.get("origin", {"x": 0, "y": 0, "yaw": 0})
                    sup.start(ct, w.get("id", "grid_9_square"), (o.get("x", 0), o.get("y", 0), o.get("yaw", 0)))
                    nav.event_hub.emit("navigation", "NAV2_START", "info", "Nav2 启动", f"车型参数 nav2_params_{ct}.yaml", {})
            link.on_model_change.append(regen_nav2)
            if link.model:
                regen_nav2(link.model)
            # NAV_ROS_EXECUTOR=single: 单线程执行器 (wait set 只建一次/轮，回调串行)；默认 multi (4 线程)
            if not rclpy_free:
                from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
                ex = SingleThreadedExecutor() if os.environ.get("NAV_ROS_EXECUTOR", "multi").strip().lower() == "single" \
                    else MultiThreadedExecutor(num_threads=4)
                ex.add_node(ros)
                threading.Thread(target=ex.spin, daemon=True, name="ros-spin").start()

            def auto_nav2():
                while True:
                    time.sleep(2.0)
                    if nav.nav2.status().get("server_ready") and os.environ.get("NAV2_DEFAULT", "1") == "1":
                        if nav.active_planner != "nav2":
                            nav.set_planner_type("nav2")
                        return
            threading.Thread(target=auto_nav2, daemon=True).start()
            log("[nav_runtime] ROS 2 桥已启动 (Nav2 由本进程监管)")
        except Exception as e:
            log(f"[nav_runtime] ROS 2 不可用，降级为纯导引模式: {e}")
            ros = None

    api = build_api(nav, link, ros, port)
    log(f"[nav_runtime] REST API http://0.0.0.0:{port}/api/v1")

    def _on_term(*_):
        # 节点代理停止实例时发 SIGTERM: 走下面的 finally 停掉 Nav2 / slam_toolbox / robot_state_publisher
        # (它们各自在独立会话里，默认的 SIGTERM 直接退出会把整套 ROS 进程留成孤儿，下次启动出现同名节点)
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _on_term)
    try:
        api.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        nav.running = False
        if ros:
            nav.nav2.supervisor.stop()
            ros.shutdown()


if __name__ == "__main__":
    main()
