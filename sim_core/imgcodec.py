#!/usr/bin/env python3
"""图像编码: JPEG (有 Pillow 时) / PNG (纯 zlib 实现，支持 8bit 灰度 / 16bit 灰度 / RGB) + 深度伪彩色"""

import struct
import zlib
from io import BytesIO

import numpy as np

try:
    from PIL import Image
    PIL_OK = True
except Exception:  # pragma: no cover
    PIL_OK = False


def png(arr: np.ndarray) -> bytes:
    a = np.ascontiguousarray(arr)
    if a.ndim == 3 and a.shape[2] == 3 and a.dtype == np.uint8:
        ctype, depth, rows = 2, 8, a.reshape(a.shape[0], -1)
    elif a.ndim == 2 and a.dtype == np.uint8:
        ctype, depth, rows = 0, 8, a
    elif a.ndim == 2 and a.dtype == np.uint16:
        ctype, depth, rows = 0, 16, a.astype(">u2").view(np.uint8).reshape(a.shape[0], -1)
    else:
        raise ValueError(f"不支持的 PNG 数据 {a.dtype} {a.shape}")
    h, w = a.shape[:2]
    raw = np.hstack([np.zeros((h, 1), np.uint8), rows]).tobytes()

    def chunk(t, d):
        return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, depth, ctype, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 3)) + chunk(b"IEND", b""))


def jpeg(rgb: np.ndarray, quality: int = 80) -> bytes:
    if not PIL_OK:
        return png(rgb)
    buf = BytesIO()
    Image.fromarray(rgb).save(buf, "JPEG", quality=quality)
    return buf.getvalue()


def image_mime(fmt: str) -> str:
    return "image/jpeg" if fmt == "jpeg" and PIL_OK else "image/png"


def colorize_depth(z: np.ndarray, zmin: float, zmax: float) -> np.ndarray:
    """深度 (m, NaN=无效) → turbo 近似伪彩色 RGB，无效像素黑色"""
    t = np.clip((np.nan_to_num(z, nan=zmax) - zmin) / max(zmax - zmin, 1e-6), 0, 1)
    r = np.clip(1.5 - np.abs(4 * t - 3), 0, 1)
    g = np.clip(1.5 - np.abs(4 * t - 2), 0, 1)
    b = np.clip(1.5 - np.abs(4 * t - 1), 0, 1)
    out = (np.stack([r, g, b], -1) * 255).astype(np.uint8)
    out[~np.isfinite(z)] = 0
    return out
