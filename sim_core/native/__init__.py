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
ABI = 3

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
    lib.sc_rng_seed.argtypes = [_P, ctypes.c_uint64]
    lib.sc_rng_seed.restype = None
    lib.sc_rng_gauss.argtypes = [_P]
    lib.sc_rng_gauss.restype = _D
    lib.sc_photo_batch.argtypes = [_P, _I, _P, _I, _D, _D, _D, _P]
    lib.sc_photo_batch.restype = None
    lib.sc_collides_batch.argtypes = [_P, _P, _I, _D, _D, _D, _P, _I, _P]
    lib.sc_collides_batch.restype = None
    lib.sc_odom_update.argtypes = [_P, _I, _I, _D, _P, _P, _D, _D, _D, _P, _P]
    lib.sc_odom_update.restype = None
    lib.sc_imu_sample.argtypes = [_P, _P, _D, _D, _D, _D, _P, _P]
    lib.sc_imu_sample.restype = None
    lib.sc_slip.argtypes = [_P, _P, _D, _P]
    lib.sc_slip.restype = None
    lib.sc_lidar_scan.argtypes = [_P, _I, _D, _D, _D, _D, _D, _D, _D, _I, _D, _D, _I, _D, _D, _D, _P, _P]
    lib.sc_lidar_scan.restype = None
    lib.sc_lidar_post.argtypes = [_P, _I, _D, _I, _D, _D, _D, _P]
    lib.sc_lidar_post.restype = None
    lib.sc_mj_bind.argtypes = [ctypes.c_char_p]
    lib.sc_mj_bind.restype = _I
    lib.sc_mj_header_version.restype = _I
    lib.sc_mj_raycast2d.argtypes = [_P, _P, _P, _I, _D, _D, _D, _P, _I, _D, _P]
    lib.sc_mj_raycast2d.restype = _I
    for fn, args, res in (("sc_rt_create", [ctypes.c_uint64], _P), ("sc_rt_destroy", [_P], None), ("sc_rt_start", [_P], _I),
                          ("sc_rt_stop", [_P], None), ("sc_rt_lock", [_P], None), ("sc_rt_unlock", [_P], None),
                          ("sc_rt_set_config", [_P, _P], None), ("sc_rt_get_state", [_P, _P], None), ("sc_rt_set_state", [_P, _P], None),
                          ("sc_rt_set_kin", [_P, _P, _P, _I, _P, _P], None), ("sc_rt_set_geom", [_P, _P, _I, _P, _I], None),
                          ("sc_rt_set_mj", [_P, _P, _P, _P, _I], None), ("sc_rt_set_photos", [_P, _P, _I], None),
                          ("sc_rt_set_bumpers", [_P, _P, _P, _P, _P, _I], None), ("sc_rt_set_lidars", [_P, _P, _I, _I], None),
                          ("sc_rt_read_lidar", [_P, _I, ctypes.c_uint32, _P, _I, _P, _P, _P], _I),
                          ("sc_rt_set_cmd", [_P, _D, _D, _D], None), ("sc_rt_step_n", [_P, _I], None),
                          ("sc_rt_udp_start", [_P, ctypes.c_char_p, _I], _I), ("sc_rt_udp_retarget", [_P], None),
                          ("sc_rt_udp_meta", [_P, _P], None), ("sc_rt_sizeof_state", [], _I), ("sc_rt_sizeof_config", [], _I),
                          ("sc_rt_sizeof_lidar", [], _I)):
        f = getattr(lib, fn)
        f.argtypes, f.restype = args, res
    lib.sc_merge_add.argtypes = [_P, _I, _P, _I, _D, _D, _D, _D, _D, _D]
    lib.sc_merge_add.restype = None
    lib.sc_merge_finish.argtypes = [_P, _I, _D]
    lib.sc_merge_finish.restype = None
    return lib, "ok"


lib, status = _load()
NAN = float("nan")


def available() -> bool:
    return lib is not None


def info() -> dict:
    return {"enabled": lib is not None, "status": status, "path": _lib_path() if lib is not None else None}


def summary() -> str:
    """/api/v1/sim 的 native 字段"""
    if lib is None:
        return status
    try:
        import mujoco  # noqa: F401
        return f"ok (mujoco C API: {mj_status()})"
    except ImportError:
        return "ok"


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


class Rng:
    """C 侧随机数状态 (xoshiro256**)。种子取自 Python random，便于整体用 random.seed 复现"""

    def __init__(self, seed=None):
        import random
        self.s = (ctypes.c_uint64 * 6)()
        self.addr = ctypes.addressof(self.s)
        lib.sc_rng_seed(self.addr, random.getrandbits(64) if seed is None else int(seed) & (2 ** 64 - 1))

    def gauss(self) -> float:
        return lib.sc_rng_gauss(self.addr)


def ptr(a: np.ndarray):
    return a.ctypes.data if a is not None and len(a) else None


# ---------------------------------------------------------------------- MuJoCo C API (经 C 直接调用 mj_multiRay / mj_step)
_mj = {"status": None}


def mj_bind() -> bool:
    """dlopen pip 包自带的 libmujoco (与 Python 绑定同一个库)；头文件版本与运行库一致才启用"""
    if _mj["status"] is None:
        if lib is None:
            _mj["status"] = "no libsimcore"
        else:
            try:
                import glob
                import mujoco
                cands = sorted(glob.glob(os.path.join(os.path.dirname(mujoco.__file__), "libmujoco*")))
                r = lib.sc_mj_bind(cands[0].encode()) if cands else 2
                _mj["status"] = {0: "ok", 1: "built without MuJoCo headers", 2: "dlopen failed",
                                 3: f"version mismatch (header {lib.sc_mj_header_version()} / runtime {mujoco.mj_version()})"}.get(r, str(r))
            except Exception as e:
                _mj["status"] = f"unavailable: {e}"
    return _mj["status"] == "ok"


def mj_status() -> str:
    mj_bind()
    return _mj["status"]
