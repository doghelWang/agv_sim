"""
内置 SLAM 计算内核的 C 实现 (planning/native/slamcore.c，编进 libagvnav) 的 ctypes 封装。
nav_runtime/slam.py 的 GridMap.insert / build_fields / match 在 lib 可用时调用这里，否则走 numpy 版。

  AGV_NATIVE_SLAM=0   强制关闭 (纯 numpy)；与规划的 AGV_NATIVE_PLAN 互相独立
一致性: tests/test_slamcore.py (栅格/匹配场 float32 逐格对比，匹配结果 1e-9 级)
"""
import ctypes
import math
import os
import sys

import numpy as np

_D, _I, _F, _P = ctypes.c_double, ctypes.c_int, ctypes.c_float, ctypes.c_void_p


def _load():
    if os.environ.get("AGV_NATIVE_SLAM", "1").strip() in ("0", "false", "off", "no"):
        return None, "disabled (AGV_NATIVE_SLAM=0)"
    here = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "planning", "native")
    path = os.path.join(here, "libagvnav." + ("dylib" if sys.platform == "darwin" else "so"))
    if not os.path.exists(path):
        return None, "not built (bash planning/native/build.sh)"
    try:
        lib = ctypes.CDLL(path)
        lib.sl_match
    except Exception as e:
        return None, f"load failed: {e}"
    lib.sl_bounds.argtypes = [_D, _D, _D, _P, _P, _P, _P, _P, _I, _D, _P]
    lib.sl_insert.argtypes = [_P, _P, _I, _I, _D, _D, _D, _D, _D, _D, _P, _P, _P, _P, _P, _I, _D, _F, _F, _F, _F, _F]
    lib.sl_insert.restype = ctypes.c_long
    lib.sl_field_bbox.argtypes = [_P, _I, _I, _I, _P, _P]
    lib.sl_field_bbox.restype = _I
    lib.sl_field_level.argtypes = [_P, _P, _I, _P, _I, _D, _I, _P, _P, _P]
    lib.sl_match.argtypes = [_P, _I, _I, _D, _D, _D, _P, _I, _I, _D, _D, _D, _P, _P, _I, _P, _P, _D, _I, _I, _P, _P, _P]
    lib.sl_match.restype = _I
    return lib, "ok"


lib, status = _load()


def _f8(a):
    return np.ascontiguousarray(a, dtype=np.float64)


def _ptr(a):
    return a.ctypes.data_as(_P)


def insert(gm, pose, px, py, hit, sx, sy, free_cap):
    """GridMap.insert 的 C 实现 (包围盒 → gm.ensure 扩图仍在 numpy)"""
    x, y, th = (float(v) for v in pose)
    px, py = _f8(px), _f8(py)
    n = len(px)
    hit = np.ascontiguousarray(np.broadcast_to(hit, px.shape), dtype=np.uint8)
    sx = np.zeros(n) if sx is None else _f8(np.broadcast_to(sx, px.shape))
    sy = np.zeros(n) if sy is None else _f8(np.broadcast_to(sy, px.shape))
    bb = np.zeros(4)
    lib.sl_bounds(x, y, th, _ptr(px), _ptr(py), _ptr(hit), _ptr(sx), _ptr(sy), n, float(free_cap), _ptr(bb))
    gm.ensure(bb[0], bb[1], bb[2], bb[3])
    h, w = gm.L.shape
    lib.sl_insert(_ptr(gm.L), _ptr(gm.Hd), h, w, gm.ox, gm.oy, gm.res, x, y, th, _ptr(px), _ptr(py), _ptr(hit), _ptr(sx), _ptr(sy), n,
                  float(free_cap), gm.L_FREE, gm.L_OCC, gm.L_MIN, gm.L_MAX, gm.HIT_CAP)


def build_fields(gm, levels, Field):
    h, w = gm.L.shape
    L = np.ascontiguousarray(gm.L, dtype=np.float32)
    Hd = np.ascontiguousarray(gm.Hd, dtype=np.float32)
    mask = np.zeros((h, w), np.uint8)
    bbox = np.zeros(4, np.int32)
    if not lib.sl_field_bbox(_ptr(L), h, w, int(math.ceil(0.8 / gm.res)), _ptr(mask), _ptr(bbox)):
        return None
    y0, x0 = int(bbox[0]), int(bbox[2])
    ox, oy = gm.ox + x0 * gm.res, gm.oy + y0 * gm.res
    out = {"rev": gm.rev}
    for name, k, sig, rad in levels:
        r = gm.res * k if k > 1 else gm.res
        hh, ww = int(bbox[1]) - y0, int(bbox[3]) - x0
        if k > 1:
            hh, ww = -(-hh // k), -(-ww // k)
        F = np.zeros((hh, ww), np.float32)
        oh, ow = ctypes.c_int(), ctypes.c_int()
        lib.sl_field_level(_ptr(Hd), _ptr(mask), w, _ptr(bbox), int(k), sig / r, max(1, int(round(rad / r))), _ptr(F),
                           ctypes.byref(oh), ctypes.byref(ow))
        out[name] = Field(F, ox, oy, r)
    return out


def match(fields, px, py, init, P0, sigma, iters):
    fi, co = fields["fine"], fields["coarse"]
    Ff = np.ascontiguousarray(fi.F, dtype=np.float32)
    Fc = np.ascontiguousarray(co.F, dtype=np.float32)
    px, py = _f8(px), _f8(py)
    ini = _f8(init)
    P = _f8(P0) if P0 is not None else None
    pose, cov, info = np.zeros(3), np.zeros(9), np.zeros(4)
    lib.sl_match(_ptr(Ff), Ff.shape[0], Ff.shape[1], fi.ox, fi.oy, fi.res, _ptr(Fc), Fc.shape[0], Fc.shape[1], co.ox, co.oy, co.res,
                 _ptr(px), _ptr(py), len(px), _ptr(ini), _ptr(P) if P is not None else None, float(sigma), int(iters[0]), int(iters[1]),
                 _ptr(pose), _ptr(cov), _ptr(info))
    inf = {"inliers": float(info[0]), "score": float(info[1]), "n": int(len(px)), "eig_min": float(info[2]), "eig_max": float(info[3])}
    return (float(pose[0]), float(pose[1]), float(pose[2])), cov.reshape(3, 3), inf
