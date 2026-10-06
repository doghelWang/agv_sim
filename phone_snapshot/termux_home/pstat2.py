#!/usr/bin/env python3
"""汇总 nav_profile 的 CSV: python3 pstat2.py prof_a.csv prof_b.csv ..."""
import csv, sys, math
NAV2 = ("controller", "planner", "bt_navigator", "behavior", "velocity_smoother", "smoother", "lifecycle", "map_server")
def corr(a, b):
    n = len(a)
    if n < 3: return 0
    ma, mb = sum(a) / n, sum(b) / n
    va, vb = sum((x - ma) ** 2 for x in a), sum((y - mb) ** 2 for y in b)
    return sum((x - ma) * (y - mb) for x, y in zip(a, b)) / math.sqrt(va * vb) if va > 0 and vb > 0 else 0
def pct(v, p):
    v = sorted(v); return v[min(len(v) - 1, int(p * len(v)))] if v else 0
for fn in sys.argv[1:]:
    rows = list(csv.DictReader(open(fn)))
    if not rows: continue
    cpu = [c for c in rows[0] if c.startswith("cpu_")]
    f = lambda r, c: float(r.get(c) or 0)
    nav2c = [c for c in cpu if any(k in c for k in NAV2)]
    tot = [sum(f(r, c) for c in cpu) for r in rows]
    n2 = [sum(f(r, c) for c in nav2c) for r in rows]
    wn2 = [sum(f(r, "wait_" + c[4:]) for c in nav2c) for r in rows]
    wall = [sum(f(r, "wait_" + c[4:]) for c in cpu) for r in rows]
    tf = [f(r, "tf_age") for r in rows]
    cap = 300 if "moderate" in rows[len(rows) // 2]["cpuset"] else 800
    print(f"== {fn}  n={len(rows)} cpuset={sorted(set(r['cpuset'] for r in rows))}")
    print(f"  总需求 mean {sum(tot)/len(tot):.0f} p95 {pct(tot,.95):.0f} max {max(tot):.0f}  Nav2 CPU mean {sum(n2)/len(n2):.1f} max {max(n2):.1f}  Nav2 等待 mean {sum(wn2)/len(wn2):.0f} p95 {pct(wn2,.95):.0f} max {max(wn2):.0f} ms/s")
    print(f"  相关: 总需求~Nav2等待 r={corr(tot,wn2):.2f}  总需求~全部等待 r={corr(tot,wall):.2f}  Nav2CPU~Nav2等待 r={corr(n2,wn2):.2f}  TF龄 mean {sum(tf)/len(tf):.2f} max {max(tf):.2f}")
    hi = [i for i, t in enumerate(tot) if t > 0.85 * cap]; lo = [i for i, t in enumerate(tot) if t <= 0.6 * cap]
    m = lambda idx, v: sum(v[i] for i in idx) / len(idx) if idx else float('nan')
    print(f"  需求>85%容量({cap}) 的时刻 {len(hi)}/{len(rows)}: Nav2 等待 {m(hi,wn2):.0f} ms/s;  需求<60% 的时刻 {len(lo)}: Nav2 等待 {m(lo,wn2):.0f} ms/s")
    for ph in ("直行", "转向", "进站", "静止"):
        idx = [i for i, r in enumerate(rows) if r["phase"] == ph]
        if idx:
            print(f"  {ph} n={len(idx):4d} 总 {m(idx,tot):4.0f}/{max(tot[i] for i in idx):4.0f}  Nav2 {m(idx,n2):5.1f}/{max(n2[i] for i in idx):5.1f}  Nav2等待 {m(idx,wn2):4.0f}/{max(wn2[i] for i in idx):5.0f}")
    top = sorted(cpu, key=lambda c: -sum(f(r, c) for r in rows))[:9]
    print("  分组 CPU mean/max: " + "  ".join(f"{c[4:]} {sum(f(r,c) for r in rows)/len(rows):.1f}/{max(f(r,c) for r in rows):.0f}" for c in top))
