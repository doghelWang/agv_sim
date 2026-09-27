"""
libsimcore 加载器 (ctypes)。加载失败时 lib = None，调用方自动回退原 Python/numpy 实现。

  SIM_NATIVE=0            强制关闭 (纯 Python)
  SIM_NATIVE=1 / 未设置    有库就用
  SIM_NATIVE_AUTOBUILD=1  找不到库时尝试就地编译 (需要 cc/gcc；默认 0，镜像构建时已编译)
"""
import ctypes
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ABI = 1

_D = ctypes.c_double
_I = ctypes.c_int
_P = ctypes.c_void_p

WHEEL_PARAMS = ("x", "y", "radius", "steer_min", "steer_max", "steer_rate", "max_speed", "drive_tau", "gear_ratio")
WHEEL_STATE = ("steer", "steer_target", "speed", "speed_target", "cmd_speed", "angle", "motor_rpm", "current_a",
               "torque_nm", "steer_current_a")
KIND = {"drive": 0, "steer": 1, "fixed": 2, "caster": 3}


class CWheel(ctypes.Structure):
    _fields_ = [("kind", _I), ("_pad", _I)] + [(n, _D) for n in WHEEL_PARAMS + WHEEL_STATE]


class CChassis(ctypes.Structure):
    _fields_ = ([(n, _D) for n in ("max_speed", "max_accel", "max_decel", "max_ang_speed", "max_ang_accel", "max_ang_decel",
                                   "mass", "axle_x", "kt", "rolling_coeff", "efficiency")]
                + [("holonomic", _I), ("_pad", _I), ("shaped", _D * 3)]
                + [(n, _D) for n in ("vx", "vy", "wz", "slip_residual", "saturation")])


def _lib_path():
    ext = "dylib" if sys.platform == "darwin" else "so"
    return os.path.join(HERE, f"libsimcore.{ext}")


def _load():
    if os.environ.get("SIM_NATIVE", "1").strip() in ("0", "false", "off", "no"):
        return None, "disabled (SIM_NATIVE=0)"
    path = _lib_path()
    if not os.path.exists(path) and os.environ.get("SIM_NATIVE_AUTOBUILD", "0") == "1":
        try:
            subprocess.run(["bash", os.path.join(HERE, "build.sh")], check=True, capture_output=True, timeout=120)
        except Exception as e:  # pragma: no cover
            return None, f"build failed: {e}"
    if not os.path.exists(path):
        return None, f"not built ({os.path.basename(path)} missing; bash sim_core/native/build.sh)"
    try:
        lib = ctypes.CDLL(path)
        if lib.sc_abi() != ABI:
            return None, f"ABI mismatch ({lib.sc_abi()} != {ABI})"
        if lib.sc_sizeof_wheel() != ctypes.sizeof(CWheel) or lib.sc_sizeof_chassis() != ctypes.sizeof(CChassis):
            return None, "struct layout mismatch"
    except Exception as e:
        return None, f"load failed: {e}"
    lib.sc_collides.argtypes = [_P, _I, _D, _D, _D, _P, _I, _P]
    lib.sc_collides.restype = _I
    lib.sc_raycast2d.argtypes = [_P, _I, _P, _I, _D, _D, _D, _P, _I, _D, _P]
    lib.sc_raycast2d.restype = None
    lib.sc_forward.argtypes = [_P, _I, _I, _D, _P, _P, _P]
    lib.sc_forward.restype = _I
    lib.sc_se2_integrate.argtypes = [_D, _D, _D, _D, _D, _D, _D, _P]
    lib.sc_se2_integrate.restype = None
    lib.sc_kin_step.argtypes = [_P, _P, _I, _D, _D, _D, _D, _I, _P]
    lib.sc_kin_step.restype = None
    lib.sc_wrap.argtypes = [_D]
    lib.sc_wrap.restype = _D
    return lib, "ok"


lib, status = _load()
NAN = float("nan")


def available() -> bool:
    return lib is not None


def info() -> dict:
    return {"enabled": lib is not None, "status": status, "path": _lib_path() if lib is not None else None}


# ---------------------------------------------------------------------- 几何
def f64(a) -> np.ndarray:
    return np.ascontiguousarray(a, dtype=np.float64)


_pt = (_D * 2)()


def collides(fp: np.ndarray, x: float, y: float, th: float, segs: np.ndarray):
    """fp: 连续 float64 [N,2]；segs: 连续 float64 [M,5]"""
    if lib.sc_collides(fp.ctypes.data, len(fp), x, y, th, segs.ctypes.data, len(segs), ctypes.addressof(_pt)):
        return True, np.array([_pt[0], _pt[1]])
    return False, None


def raycast2d(segs: np.ndarray, circles, zmin: float, ox: float, oy: float, angles: np.ndarray, max_range: float) -> np.ndarray:
    angles = f64(angles)
    out = np.empty(len(angles))
    lib.sc_raycast2d(segs.ctypes.data, len(segs), circles.ctypes.data if circles is not None and len(circles) else None,
                     len(circles) if circles is not None else 0, zmin, ox, oy, angles.ctypes.data, len(angles), max_range,
                     out.ctypes.data)
    return out
