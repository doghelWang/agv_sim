#!/usr/bin/env python3
"""robot_state_publisher 子进程监管 (URDF 来自仿真进程 GET /api/v1/model/urdf)。不依赖 rclpy (核心模式下执行进程不加载 rclpy)"""
import os
import shutil
import signal
import tempfile

from common import spawn

# 轮子关节 TF 频率 (导航不依赖)；Android proot 下降到 10 Hz 减少 /tf 扇出
RSP_HZ = float(os.environ.get("RSP_HZ", "10.0" if os.path.exists("/system/build.prop") else "30.0"))


class RspSupervisor:
    """robot_state_publisher 子进程 (URDF 来自仿真进程 GET /api/v1/model/urdf)"""

    def __init__(self, log):
        self.log, self.proc, self.urdf = log, None, None

    def apply(self, urdf: str):
        if not urdf or urdf == self.urdf or shutil.which("ros2") is None:
            return
        self.urdf = urdf
        self.stop()
        body = "\n".join("      " + ln for ln in urdf.rstrip("\n").split("\n"))
        fd, path = tempfile.mkstemp(suffix=".yaml", prefix="rsp_")
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(f"robot_state_publisher:\n  ros__parameters:\n    publish_frequency: {RSP_HZ}\n    robot_description: |\n" + body + "\n")
        self.proc = spawn.popen("rsp", ["ros2", "run", "robot_state_publisher", "robot_state_publisher", "--ros-args",
                                                "--params-file", path])
        self.log(f"[ros_bridge] robot_state_publisher 已启动 (URDF {len(urdf)} 字节)")

    def stop(self):
        if self.proc and self.proc.poll() is None:
            try:
                spawn.killpg(self.proc, signal.SIGINT)
                self.proc.wait(timeout=5)
            except Exception:
                pass
        self.proc = None
