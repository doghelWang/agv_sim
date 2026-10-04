"""GPU 射线求交客户端 (配合 gpucastd，见 gpucastd.c)

仿真进程在 proot/容器里加载不了手机厂商的 OpenCL 驱动，所以 GPU 求交放在一个独立的小服务里，这里通过本机 TCP 调用。
  SIM_GPU_CAST=auto (默认)   启动时探测 127.0.0.1:8068，服务在就用，不在就完全不启用
  SIM_GPU_CAST=host:port     指定地址
  SIM_GPU_CAST=0             关闭
  SIM_GPU_CAST_MIN=20000     射线数不少于这个值才走 GPU (少量射线 CPU 更快，往返开销约 1 ms)
两种用法: cast() 只做求交 (深度/ToF/3D 激光，返回距离/几何体/法向)；render() 整帧 RGB 相机，像素方向、求交、着色、
噪声都在 GPU 上，只回传 8 位图像 (单目相机默认走这条)。
任何错误都退回 CPU 路径，5 秒后再尝试重连。GPU 用 float 计算，距离与 CPU 版相差在 0.1 mm 量级。
"""
import os
import socket
import struct
import threading
import time

import numpy as np

MAGIC = 0x31435047
DEFAULT_ADDR = ("127.0.0.1", 8068)
MIN_RAYS = int(os.environ.get("SIM_GPU_CAST_MIN", "20000"))


def _addr():
    v = os.environ.get("SIM_GPU_CAST", "auto").strip().lower()
    if v in ("0", "off", "false", "no", ""):
        return None
    if v in ("auto", "1", "on", "true", "yes"):
        return DEFAULT_ADDR
    h, _, p = v.rpartition(":")
    return (h or "127.0.0.1", int(p))


class GpuCast:
    """一个连接 = 一个场景。线程安全 (内部加锁，求交串行)。"""

    def __init__(self, addr=None, timeout=2.0):
        self.addr = addr if addr is not None else _addr()
        self.timeout = timeout
        self.sock = None
        self.device = ""
        self.lock = threading.Lock()
        self._scene_key = None
        self._shade_key = None
        self._shade_ref = None
        self._retry_at = 0.0
        self.renders = 0
        self.render_ms = 0.0
        self.render_gpu_ms = 0.0
        self.calls = 0
        self.ms = 0.0
        self.gpu_ms = 0.0
        self.errors = 0

    # ---- 连接
    def _recv(self, n):
        buf = bytearray(n)
        mv, got = memoryview(buf), 0
        while got < n:
            k = self.sock.recv_into(mv[got:], n - got)
            if k <= 0:
                raise ConnectionError("gpucastd 断开")
            got += k
        return buf

    def _hdr(self):
        magic, status, n, us = struct.unpack("<IiII", self._recv(16))
        if magic != MAGIC:
            raise ConnectionError("gpucastd 应答错误")
        if status != 0:
            raise RuntimeError(f"gpucastd 状态 {status}")
        return n, us

    def _connect(self):
        s = socket.create_connection(self.addr, timeout=0.5)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.settimeout(self.timeout)
        self.sock = s
        s.sendall(struct.pack("<IIII4f", MAGIC, 3, 0, 0, 0, 0, 0, 0))
        self._hdr()
        self.device = bytes(self._recv(64)).split(b"\0")[0].decode("utf-8", "replace")
        self._scene_key = None
        self._shade_key = None

    def _drop(self):
        try:
            if self.sock:
                self.sock.close()
        except OSError:
            pass
        self.sock = None
        self._scene_key = None
        self._shade_key = None
        self._shade_ref = None
        self._retry_at = time.monotonic() + 5.0

    def available(self) -> bool:
        if self.addr is None:
            return False
        if self.sock is not None:
            return True
        if time.monotonic() < self._retry_at:
            return False
        with self.lock:
            if self.sock is None:
                try:
                    self._connect()
                except (OSError, RuntimeError, ConnectionError):
                    self._drop()
        return self.sock is not None

    def _send_scene(self, scene_key, prims, ceil_h, floor_gid, ceil_gid):
        if scene_key == self._scene_key:
            return
        p = np.ascontiguousarray(prims, np.float64)
        self.sock.sendall(struct.pack("<IIII4f", MAGIC, 1, len(p), 0, 0, 0, 0, 0)
                          + struct.pack("<iid", int(floor_gid), int(ceil_gid), float(ceil_h)) + p.tobytes())
        self._hdr()
        self._scene_key = scene_key

    # ---- 整帧相机 (方向 + 求交 + 着色 + 噪声全在 GPU)
    def render(self, scene_key, prims, ceil_h, floor_gid, ceil_gid, geom_rgb, tex, tex_org, origin, Rw, fx, fy, cx, cy,
               W, H, max_range, sigma, seed):
        """→ uint8 [H,W,3]；失败返回 None (调用方退回 CPU)。geom_rgb [G,3] float32；tex [th,tw,3] float32 (0~1) 地面贴图，
        tex_org = (x0, y0, res)。贴图/颜色只在对象变化时重发 (贴图以 8 位上传，与 CPU 版至多差 1 个灰度级)"""
        t0 = time.perf_counter()
        with self.lock:
            if self.sock is None:
                return None
            try:
                self._send_scene(scene_key, prims, ceil_h, floor_gid, ceil_gid)
                if self._shade_key != (scene_key, id(tex), id(geom_rgb)):
                    rgb = np.ascontiguousarray(geom_rgb, np.float32)
                    t8 = np.clip(np.asarray(tex) * 255.0 + 0.5, 0, 255).astype(np.uint8)
                    th, tw = t8.shape[:2]
                    self.sock.sendall(struct.pack("<IIII4f", MAGIC, 5, 0, 0, 0, 0, 0, 0)
                                      + struct.pack("<4i4f", len(rgb), th, tw, 0, float(tex_org[0]), float(tex_org[1]), float(tex_org[2]), 0.0)
                                      + rgb.tobytes())
                    self.sock.sendall(memoryview(np.ascontiguousarray(t8)).cast("B"))
                    self._hdr()
                    self._shade_key = (scene_key, id(tex), id(geom_rgb))
                    self._shade_ref = (tex, geom_rgb)             # 持有引用，id 不会被新对象复用
                o = np.asarray(origin, np.float64)
                R = np.asarray(Rw, np.float64).reshape(9)
                self.sock.sendall(struct.pack("<IIII4f", MAGIC, 4, W * H, 0, 0, 0, 0, 0)
                                  + struct.pack("<18d4I", *o[:3], *R, fx, fy, cx, cy, float(max_range), float(sigma),
                                                int(W), int(H), int(seed) & 0xFFFFFFFF, 0))
                _, us = self._hdr()
                img = np.frombuffer(self._recv(W * H * 3), np.uint8).reshape(H, W, 3)
            except (OSError, RuntimeError, ConnectionError, struct.error):
                self.errors += 1
                self._drop()
                return None
        self.renders += 1
        self.render_gpu_ms += us / 1000.0
        self.render_ms += (time.perf_counter() - t0) * 1000.0
        return img

    # ---- 求交
    def cast(self, scene_key, prims, ceil_h, floor_gid, ceil_gid, origin, dirs, max_range, want_normal):
        """prims: [N,18] float64 (同 sc_cast_prims)；scene_key 变化时才重发场景。
        返回 (dist float64[inf=无回波], gid int32, nrm float64[N,3]|None)；失败返回 None (调用方退回 CPU)"""
        n = len(dirs)
        t0 = time.perf_counter()
        with self.lock:
            if self.sock is None:
                return None
            try:
                self._send_scene(scene_key, prims, ceil_h, floor_gid, ceil_gid)
                d = np.ascontiguousarray(dirs, np.float32)
                o = np.asarray(origin, np.float64)
                self.sock.sendall(struct.pack("<IIII4f", MAGIC, 2, n, 1 if want_normal else 0, 0, 0, 0, 0)
                                  + struct.pack("<4d", o[0], o[1], o[2], float(max_range)))
                self.sock.sendall(memoryview(d).cast("B"))
                _, us = self._hdr()
                dist = np.frombuffer(self._recv(n * 4), np.float32).astype(np.float64)
                gid = np.frombuffer(self._recv(n * 4), np.int32)
                nrm = np.frombuffer(self._recv(n * 12), np.float32).astype(np.float64).reshape(n, 3) if want_normal else None
            except (OSError, RuntimeError, ConnectionError, struct.error):
                self.errors += 1
                self._drop()
                return None
        self.calls += 1
        self.gpu_ms += us / 1000.0
        self.ms += (time.perf_counter() - t0) * 1000.0
        return dist, gid, nrm

    def info(self) -> dict:
        return {"addr": f"{self.addr[0]}:{self.addr[1]}" if self.addr else None, "connected": self.sock is not None,
                "device": self.device, "calls": self.calls, "errors": self.errors, "min_rays": MIN_RAYS,
                "avg_ms": round(self.ms / self.calls, 3) if self.calls else None,
                "avg_gpu_ms": round(self.gpu_ms / self.calls, 3) if self.calls else None,
                "renders": self.renders, "render_avg_ms": round(self.render_ms / self.renders, 3) if self.renders else None,
                "render_avg_gpu_ms": round(self.render_gpu_ms / self.renders, 3) if self.renders else None}
