#!/usr/bin/env python3
"""
核心模式下执行进程的 ROS 门面 (替代 ros_bridge.RosBridge；执行进程不加载 rclpy)

  全部 ROS 通信都在 C++ 桥接 (ros2/agv_ros_bridge --core) 里:
    仿真推送流 → ROS 话题 / TF；Nav2 指令 → 安全层 → 仿真；路线导航 (NavigateToPose)；自研导引；
    Nav2 生命周期查询、ROS 图、/map、/set_pose、位姿图保存、map_server 换图
  本类只负责: 启动 C++ 桥接、TF 发布标志 (MODE)、定位栈进程 (ros_slam.RosLocalization)、robot_state_publisher 进程、
  把 C++ 的回馈转给 Nav2Bridge / RosLocalization / 界面。
  回退模式 (NAV2_ROUTE_MODE=follow_path 等) 要用 Python Action 客户端时，rclpy_node() 按需创建一个 rclpy 节点。

  NAV_RCLPY_FREE=0 关闭 (走 ros_bridge.RosBridge)。3D 激光点云与相机图像也由 C++ 从仿真拉取并发布
"""
import itertools
import os
import threading
import time

from nav_runtime.cpp_bridge import CppBridge
from nav_runtime.rsp import RspSupervisor


class _Log:
    def __init__(self, log):
        self._log = log

    def info(self, m):
        self._log(m)

    def warn(self, m):
        self._log("[警告] " + m)

    warning = warn

    def error(self, m):
        self._log("[错误] " + m)


def usable(link) -> bool:
    """核心模式 + 路线模式 (回退模式的 Python Action 客户端按需再建 rclpy 节点)"""
    from nav_runtime.cpp_bridge import wanted_mode
    if os.environ.get("NAV_RCLPY_FREE", "1") == "0" or os.environ.get("NAV_CPP_CORE", "1") == "0":
        return False
    return os.environ.get("NAV2_ROUTE_MODE", "agv") == "agv" and wanted_mode() == "cpp"


class CppRos:
    rclpy_free = True
    core = True

    def __init__(self, link, navigator, log=print):
        self.link, self.nav, self._log = link, navigator, log
        self.localization = os.environ.get("SIM_LOCALIZATION", "ground_truth")
        self.logger = _Log(log)
        self.cpp = CppBridge(log=log)
        if not self.cpp.core:
            raise RuntimeError("C++ 桥接不在核心模式 (NAV_CPP_CORE=0)")
        self.cpp.on_relay = link.feed
        link.media_external = True          # 3D 激光点云由 C++ 拉取发布，本进程不再长轮询
        self.cpp.on_ros = self._on_ros
        self.nav2_active = False
        self.mode_flags = {}
        self._graph = {"nodes": [], "topics": []}
        self._ids = itertools.count(1)
        self._calls = {}
        self.on_map = None
        self._rnode = None
        self._rlock = threading.Lock()
        self.rsp = RspSupervisor(log)
        self.loc = None
        if os.environ.get("LOC_ENGINE", "auto") != "builtin" and self.localization != "amcl":
            from nav_runtime.ros_slam import RosLocalization, stack_installed
            if stack_installed():
                self.loc = RosLocalization(self, navigator.slam, log=log)
                navigator.slam.attach_external(self.loc)
            else:
                log("[cpp_ros] 未安装 slam_toolbox / robot_localization，定位使用内置 SLAM (nav_runtime/slam.py)")
        link.on_state.append(self._on_state)
        link.on_model_change.append(lambda m: self.rsp.apply(self.link.urdf))
        if link.urdf:
            self.rsp.apply(link.urdf)
        log("[cpp_ros] 执行进程不加载 rclpy: ROS 通信全部在 C++ 桥接")

    # ------------------------------------------------------------------ 与 RosBridge 相同的接口
    def get_logger(self):
        return self.logger

    def _on_state(self, st):
        """TF 发布标志 (与 RosBridge._on_state_cpp 同一套判断) → MODE (Navigator 周期下发)"""
        slam = getattr(self.nav, "slam", None)
        ext_tf = slam is not None and slam.ros_active()
        own_odom = not ext_tf or (self.loc is not None and not getattr(self.loc, "ekf", True))
        map_odom = self.localization != "amcl" and not ext_tf
        m2o = (0.0, 0.0, 0.0)
        if map_odom:
            m = st["map_to_odom"]
            m2o = slam.M if slam is not None else (m["x"], m["y"], m["yaw"])
        self.mode_flags = {"own_odom": own_odom, "map_odom": map_odom, "m2o": [round(float(v), 5) for v in m2o],
                           "loc_ext": bool(ext_tf), "want_raw_lidar": not ext_tf}

    def graph(self):
        return self._graph

    def bridge_status(self) -> dict:
        return self.cpp.status()

    def shutdown(self):
        self.rsp.stop()
        self.cpp.stop()
        if self.loc is not None:
            self.loc.stop()
        if self._rnode is not None:
            try:
                import rclpy
                rclpy.shutdown()
            except Exception:
                pass

    # ------------------------------------------------------------------ C++ 回馈
    def _on_ros(self, m: dict):
        k = m.get("k")
        if k == "ros":
            self.nav2_active = bool(m.get("nav2_active"))
            self._graph = {"nodes": m.get("nodes", []), "topics": m.get("topics", [])}
        elif k == "map":
            cb = self.on_map
            if cb:
                cb(m.get("path"), m.get("rev"))
        elif k == "roscall":
            ev = self._calls.get(m.get("id"))
            if ev is not None:
                ev[1] = m
                ev[0].set()

    def roscall(self, op, wait=False, timeout=5.0, **kw):
        """C++ 执行 ROS 操作 (load_map / set_pose / save_map)；wait=True 时等回复 (dict)"""
        cid = next(self._ids)
        ev = [threading.Event(), None]
        if wait:
            self._calls[cid] = ev
        import json
        self.cpp._send(b"AGV1" + bytes([9]) + json.dumps(dict(kw, op=op, id=cid), separators=(",", ":")).encode("utf-8"))
        if not wait:
            return True
        ev[0].wait(timeout)
        self._calls.pop(cid, None)
        return ev[1]

    # ------------------------------------------------------------------ 回退模式用的 rclpy 节点
    def rclpy_node(self):
        with self._rlock:
            if self._rnode is None:
                import rclpy
                from rclpy.executors import MultiThreadedExecutor
                from rclpy.node import Node
                if not rclpy.ok():
                    rclpy.init()
                self._rnode = Node("nav_runtime_fallback")
                ex = MultiThreadedExecutor(num_threads=2)
                ex.add_node(self._rnode)
                threading.Thread(target=ex.spin, daemon=True, name="ros-fallback-spin").start()
                self._log("[cpp_ros] 回退模式需要 Python Action 客户端，已按需创建 rclpy 节点")
                time.sleep(0.2)
            return self._rnode
