"""
libagvnav 加载器 (ctypes)：规划算法的 C 实现 (planning/native/agvnav.c)。加载失败时 lib = None，调用方回退 Python。

  AGV_NATIVE_PLAN=0        强制关闭 (纯 Python)
  AGV_NATIVE_AUTOBUILD=1   找不到库时尝试就地编译 (需要 cc/gcc)
"""
import ctypes
import math
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ABI = 1
_D, _I, _P = ctypes.c_double, ctypes.c_int, ctypes.c_void_p
MODES = {"rotate": 0, "arc": 1, "auto": 2}


def _load():
    if os.environ.get("AGV_NATIVE_PLAN", "1").strip() in ("0", "false", "off", "no"):
        return None, "disabled (AGV_NATIVE_PLAN=0)"
    path = os.path.join(HERE, "libagvnav." + ("dylib" if sys.platform == "darwin" else "so"))
    if not os.path.exists(path) and os.environ.get("AGV_NATIVE_AUTOBUILD", "0") == "1":
        try:
            subprocess.run(["bash", os.path.join(HERE, "build.sh")], check=True, capture_output=True, timeout=120)
        except Exception as e:  # pragma: no cover
            return None, f"build failed: {e}"
    if not os.path.exists(path):
        return None, f"not built ({os.path.basename(path)} missing; bash planning/native/build.sh)"
    try:
        lib = ctypes.CDLL(path)
        if lib.an_abi() != ABI:
            return None, f"ABI mismatch ({lib.an_abi()} != {ABI})"
    except Exception as e:
        return None, f"load failed: {e}"
    lib.an_clearance.argtypes = [_P, _I, _P, _I, _D, _D, _D]
    lib.an_clearance.restype = _D
    lib.an_plan_corner.argtypes = [_P, _I, _D, _D, _D, _D, _D, _D, _D, _D, _D, _D, _D, _I, _P]
    lib.an_plan_corner.restype = _D
    lib.an_router_new.restype = _P
    lib.an_router_free.argtypes = [_P]
    lib.an_router_set_graph.argtypes = [_P, _P, _I, _P, _P, _I, _P, _I]
    lib.an_router_set_robot.argtypes = [_P, _D, _D]
    lib.an_router_set_footprint.argtypes = [_P, _I, _D, _D, _D, _D, _D, _I]
    lib.an_router_plan.argtypes = [_P, _D, _D, _D, _D, _P, _I, _P, _P, _I, _P]
    lib.an_router_plan.restype = _I
    return lib, "ok"


lib, status = _load()


def _arr(a, dt="<f8"):
    return np.ascontiguousarray(np.asarray(a, dtype=dt))


def _ptr(a):
    return a.ctypes.data_as(_P)


def clearance(segs, poses, head, tail, hw) -> float:
    S = _arr([s[:4] for s in segs] if len(segs) else np.zeros((0, 4)))
    Q = _arr([p[:3] for p in poses] if len(poses) else np.zeros((0, 3)))
    return float(lib.an_clearance(_ptr(S), len(S), _ptr(Q), len(Q), float(head), float(tail), float(hw)))


def plan_corner(segs, node, h1, h2, len_in, len_out, head, tail, hw, r_pref, clear_min, mode):
    S = _arr([s[:4] for s in segs] if len(segs) else np.zeros((0, 4)))
    out = np.zeros(8)
    clr = lib.an_plan_corner(_ptr(S), len(S), float(node[0]), float(node[1]), float(h1), float(h2), float(len_in), float(len_out),
                             float(head), float(tail), float(hw), float(r_pref), float(clear_min), MODES.get(mode, 0), _ptr(out))
    if out[0] == 1:
        c = {"heading": float(out[4]), "turn": float(out[5]), "R": float(out[2]), "d": float(out[3]), "v": 0.35,
             "cx": float(out[6]), "cy": float(out[7])}
    else:
        c = {"rotate": float(out[1])}
    return c, float(clr)


class Router:
    """拓扑贴合规划 (DijkstraPlanner.plan_route 的 C 实现)"""

    def __init__(self):
        self.h = lib.an_router_new()
        self.names = []

    def __del__(self):
        try:
            if lib is not None and self.h:
                lib.an_router_free(self.h)
        except Exception:
            pass

    def set_graph(self, nodes: dict, connections, segs):
        """nodes: {名: (x, y)} (插入顺序)；connections: [(u, v), ...] 与 DijkstraPlanner 建邻接表的顺序相同"""
        self.names = list(nodes)
        idx = {n: i for i, n in enumerate(self.names)}
        order = sorted(self.names + ["__S__", "__G__"])            # heapq 并列按节点名字典序
        rk = {n: i for i, n in enumerate(order)}
        self._rank = _arr([rk[n] for n in self.names] + [rk["__S__"], rk["__G__"]], "<i4")
        self._nodes = _arr([nodes[n] for n in self.names]) if self.names else np.zeros((0, 2))
        conns = [(idx[u], idx[v]) for u, v in connections if u in idx and v in idx]
        self._edges = _arr(conns, "<i4") if conns else np.zeros((0, 2), "<i4")
        self._segs = _arr([s[:4] for s in segs]) if len(segs) else np.zeros((0, 4))
        lib.an_router_set_graph(self.h, _ptr(self._nodes), len(self.names), _ptr(self._rank), _ptr(self._edges), len(conns),
                                _ptr(self._segs), len(self._segs))

    def set_footprint(self, fp):
        if fp:
            head, tail, hw, r, cmin, mode = fp
            lib.an_router_set_footprint(self.h, 1, head, tail, hw, r, cmin, MODES.get(mode, 0))
        else:
            lib.an_router_set_footprint(self.h, 0, 0, 0, 0, 0, 0, 0)

    def plan(self, start, goal, obstacles, half_width, circum):
        circles = []
        for o in obstacles or []:
            if isinstance(o, dict):
                circles.append((float(o.get("x", 0.0)), float(o.get("y", 0.0)), math.hypot(float(o.get("w", 0.8)), float(o.get("h", 0.8))) / 2.0))
            else:
                circles.append(((o[0] + o[2]) / 2.0, (o[1] + o[3]) / 2.0, 0.3))
        C = _arr(circles) if circles else np.zeros((0, 3))
        lib.an_router_set_robot(self.h, float(half_width), float(circum))
        cap = 4 * len(self.names) + 16
        P = np.zeros(2 * cap)
        L = np.zeros(cap, "<i4")
        length = ctypes.c_double(0.0)
        n = lib.an_router_plan(self.h, float(start[0]), float(start[1]), float(goal[0]), float(goal[1]), _ptr(C), len(circles),
                               _ptr(P), _ptr(L), cap, ctypes.byref(length))
        if n <= 0:
            return {"points": [], "labels": [], "length": 0.0}
        pts = [(float(P[2 * i]), float(P[2 * i + 1])) for i in range(n)]
        labels = [self.names[L[i]] if L[i] >= 0 else None for i in range(n)]
        return {"points": pts, "labels": labels, "length": float(length.value)}
