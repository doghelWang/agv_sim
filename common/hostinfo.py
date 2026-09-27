"""
主机信息 (树莓派 / x86 / Android 手机 Termux+proot 通用)

Android 上 (Termux, proot-distro Ubuntu) 的差异:
  * /proc/stat 被 proot 伪造或受限 → psutil.cpu_percent 恒为 0：改为统计本机可见进程的 CPU 时间估算
  * 没有 /proc/device-tree/model → 从 /system/build.prop 或 AGV_DEVICE_MODEL 取机型
  * thermal_zone0 常不可读 → 依次尝试各 thermal_zone 与电池温度
  * big.LITTLE：按 cpuinfo_max_freq 给出大/小核分组，供进程绑核 (taskset) 参考
"""
import os
import platform
import time
from typing import Dict, List, Optional


def is_android() -> bool:
    return bool(os.environ.get("ANDROID_ROOT") or os.environ.get("TERMUX_VERSION") or os.path.exists("/system/build.prop")
                or os.environ.get("AGV_PLATFORM") == "android")


def _read(p) -> Optional[str]:
    try:
        with open(p) as f:
            return f.read()
    except Exception:
        return None


def _prop(name) -> Optional[str]:
    for p in ("/system/build.prop", "/vendor/build.prop", "/system/vendor/build.prop", "/odm/etc/build.prop"):
        t = _read(p)
        if t:
            for line in t.splitlines():
                if line.startswith(name + "="):
                    return line.split("=", 1)[1].strip()
    return None


def model() -> str:
    if os.environ.get("AGV_DEVICE_MODEL"):
        return os.environ["AGV_DEVICE_MODEL"]
    t = _read("/proc/device-tree/model")
    if t:
        return t.strip("\x00\n ")
    if is_android():
        m = _prop("ro.product.model") or _prop("ro.product.system.model") or _prop("ro.product.vendor.model")
        soc = _prop("ro.soc.model") or _prop("ro.board.platform") or ""
        rel = _prop("ro.build.version.release") or ""
        return " ".join(x for x in (m or "Android", f"({soc})" if soc else "", f"Android {rel}" if rel else "") if x)
    t = _read("/proc/cpuinfo") or ""
    for line in t.splitlines():
        if line.startswith("model name") or line.startswith("Hardware"):
            return line.split(":", 1)[1].strip()
    return platform.processor() or platform.machine()


def core_groups() -> Dict[str, List[int]]:
    """按最高频率分组: {"2841600": [7], "2419200": [4,5,6], "1785600": [0..3]} → big/little"""
    g: Dict[int, List[int]] = {}
    n = os.cpu_count() or 1
    for i in range(n):
        f = _read(f"/sys/devices/system/cpu/cpu{i}/cpufreq/cpuinfo_max_freq")
        if f and f.strip().isdigit():
            g.setdefault(int(f), []).append(i)
    if not g:
        return {}
    ks = sorted(g, reverse=True)
    out = {"big": [c for k in ks[:-1] for c in g[k]] if len(ks) > 1 else g[ks[0]], "little": g[ks[-1]] if len(ks) > 1 else []}
    out["max_mhz"] = {str(c): k // 1000 for k in ks for c in g[k]}
    return out


def temp_c() -> Optional[float]:
    best = None
    for i in range(0, 80):
        typ = (_read(f"/sys/class/thermal/thermal_zone{i}/type") or "").strip().lower()
        v = _read(f"/sys/class/thermal/thermal_zone{i}/temp")
        if v is None:
            if i > 10 and best is None and typ == "":
                break
            continue
        try:
            t = int(v.strip()) / (1000.0 if abs(int(v.strip())) > 1000 else 1.0)
        except Exception:
            continue
        if i == 0 and not is_android():
            return round(t, 1)
        if any(k in typ for k in ("cpu", "soc", "tsens", "skin", "xo-therm", "cluster")) and 0 < t < 130:
            best = max(best or 0.0, t)
    if best is not None:
        return round(best, 1)
    b = _read("/sys/class/power_supply/battery/temp")
    if b and b.strip().lstrip("-").isdigit():
        return round(int(b.strip()) / 10.0, 1)          # 电池温度 (0.1 °C)
    return None


class CpuEstimator:
    """/proc/stat 不可用时 (Android proot)：统计可见进程的 utime+stime 增量 / 墙钟 / 核数"""

    def __init__(self):
        self._prev = None
        self._hz = os.sysconf("SC_CLK_TCK") if hasattr(os, "sysconf") else 100

    def total_ticks(self) -> int:
        s = 0
        for pid in os.listdir("/proc"):
            if not pid.isdigit():
                continue
            t = _read(f"/proc/{pid}/stat")
            if not t:
                continue
            try:
                f = t.rsplit(")", 1)[1].split()
                s += int(f[11]) + int(f[12])
            except Exception:
                pass
        return s

    def percent(self) -> float:
        now, ticks = time.time(), self.total_ticks()
        prev, self._prev = self._prev, (now, ticks)
        if not prev or now - prev[0] <= 0:
            return 0.0
        return round(min(100.0, 100.0 * (ticks - prev[1]) / self._hz / (now - prev[0]) / (os.cpu_count() or 1)), 1)


def proc_stat_usable() -> bool:
    t = _read("/proc/stat") or ""
    first = t.splitlines()[0].split()[1:] if t else []
    try:
        return sum(int(x) for x in first) > 0
    except Exception:
        return False
