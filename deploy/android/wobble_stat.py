#!/usr/bin/env python3
"""汇总 wobble.py 的 CSV (直行段的车身横摆与定位误差): python3 ~/wobble_stat.py ~/wobble_*.csv"""
import csv, math, os, sys
rms = lambda v: math.sqrt(sum(x * x for x in v) / len(v))
for f in sys.argv[1:]:
    r = list(csv.DictReader(open(f)))
    if len(r) < 10:
        print(f, "样本太少", len(r)); continue
    lat = [float(x["lat_mm"]) for x in r]; hd = [float(x["head_deg"]) for x in r]; wz = [float(x["wz_dps"]) for x in r]
    ll = [float(x["loc_lat_mm"]) for x in r]; ly = [float(x["loc_yaw_deg"]) for x in r]
    t = [float(x["t"]) for x in r]; tk = [x["task"] for x in r]; v = [float(x["v"]) for x in r]
    dts = sorted(t[i] - t[i - 1] for i in range(1, len(r)) if tk[i] == tk[i - 1]); per = dts[len(dts) // 2]
    dist = sum(v) * per
    fl = sum(1 for i in range(1, len(r)) if tk[i] == tk[i - 1] and wz[i] * wz[i - 1] < 0 and abs(wz[i] - wz[i - 1]) > 2)
    dj = [abs(ll[i] - ll[i - 1]) for i in range(1, len(r)) if tk[i] == tk[i - 1] and t[i] - t[i - 1] < per * 2.5]
    st = [i for i in range(len(r)) if abs(hd[i]) < 3.0]      # 稳态直行 (不算转弯出口)
    print(f"{os.path.basename(f)[7:-4]:14s} 点 {len(r):4d} ≈{dist:3.0f} m | 车身横向 RMS {rms(lat):4.0f} 最大 {max(map(abs, lat)):4.0f} mm | "
          f"车头偏角 RMS {rms(hd):4.2f}° | 稳态段横向 RMS {rms([lat[i] for i in st]):4.0f} mm | 换向 {fl / max(dist, 1):.2f} 次/m | "
          f"定位横向误差 RMS {rms(ll):4.0f} 最大 {max(map(abs, ll)):4.0f} mm 朝向 RMS {rms(ly):.2f}° | 定位跳变 均值 {sum(dj) / len(dj):.1f} 最大 {max(dj):.0f} mm")
