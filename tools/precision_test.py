#!/usr/bin/env python3
"""
导航精度 / 定位精度 考核脚本 (经 Web 网关，无需 ROS)

  python3 tools/precision_test.py                                  # 默认: 当前场景的一组工位
  python3 tools/precision_test.py --gw http://127.0.0.1:8088 --scenario fms_workshop_xl \
          --goals '[[6.4,2.9],[6.4,-2.9],[-6.4,-2.9],[0,10.2]]' --planner nav2 --out /tmp/prec.json

逐个下发目标，10 Hz 记录车辆真值位姿、参考线 (plan_curve)、定位误差 (执行进程定位 vs 真值)，输出：
  行驶中最大横向偏差 (真值到参考线的距离，速度 > 0.03 m/s 的采样)、终点位置误差、定位误差最大值、耗时、碰撞
需求: 横向偏差与终点误差 ≤ 20 mm (--tol 可改)。
"""
import argparse
import json
import math
import sys
import time
import urllib.request


def _req(gw, path, body=None, method=None, timeout=10):
    data = None if body is None else json.dumps(body).encode()
    r = urllib.request.Request(gw + path, data=data, headers={"Content-Type": "application/json"},
                               method=method or ("POST" if body is not None else "GET"))
    return json.loads(urllib.request.urlopen(r, timeout=timeout).read() or b"{}")


def _dseg(px, py, a, b):
    dx, dy = b["x"] - a["x"], b["y"] - a["y"]
    L2 = dx * dx + dy * dy or 1e-12
    u = max(0.0, min(1.0, ((px - a["x"]) * dx + (py - a["y"]) * dy) / L2))
    return math.hypot(px - a["x"] - u * dx, py - a["y"] - u * dy)


def _xte(px, py, curve):
    return min((_dseg(px, py, a, b) for a, b in zip(curve, curve[1:])), default=float("nan"))


def run(gw, scenario, goals, planner, tmax):
    if scenario:
        _req(gw, "/api/map_scenario", {"scenario": scenario})
        time.sleep(5)
    if planner:
        try:
            _req(gw, "/api/planner_type", {"type": planner})
            time.sleep(1.0)
        except Exception as e:  # noqa
            print(f"切换规划器失败 ({e})，使用当前规划器")
    runs = []
    for g in goals:
        gx, gy = g[0], g[1]
        gyaw = g[2] if len(g) > 2 else 0.0
        _req(gw, "/api/navigate_to_pose", {"x": gx, "y": gy, "yaw": gyaw})
        t0 = time.time()
        time.sleep(0.4)
        curve, traj, st, t = [], [], None, {}
        while time.time() - t0 < tmax:
            t = _req(gw, "/api/telemetry")
            st = t.get("nav_status")
            c = t.get("plan_curve") or []
            if c:
                curve = c
            loc = t.get("localization") or {}
            traj.append({"t": round(time.time() - t0, 2), "x": t["x"], "y": t["y"], "yaw": t["yaw"], "v": math.hypot(t.get("vx", 0), t.get("vy", 0)),
                         "loc_err_mm": loc.get("err_mm"), "planner": t.get("planner_type")})
            if st in ("ARRIVED", "FAILED", "NO_PATH", "CANCELED", "BUMPER_STOP"):
                break
            time.sleep(0.1)
        xte = [_xte(p["x"], p["y"], curve) for p in traj if p["v"] > 0.03] if len(curve) >= 2 else []
        le = [p["loc_err_mm"] for p in traj if p["loc_err_mm"] is not None]
        end = traj[-1] if traj else None
        r = {"goal": [gx, gy], "status": st, "t": round(time.time() - t0, 1), "planner": (end or {}).get("planner"),
             "xte_max_mm": round(max(xte) * 1000, 1) if xte else None,
             "end_err_mm": round(math.hypot(end["x"] - gx, end["y"] - gy) * 1000, 1) if end else None,
             "loc_err_max_mm": round(max(le), 1) if le else None,
             "collisions": (t.get("bumpers") or {}).get("count"), "curve": curve, "traj": traj}
        runs.append(r)
        print(f"目标 ({gx:6.2f},{gy:6.2f}) {st:10s} {r['t']:6.1f}s  横向偏差max {r['xte_max_mm']} mm  终点误差 {r['end_err_mm']} mm  "
              f"定位误差max {r['loc_err_max_mm']} mm  碰撞 {r['collisions']}", flush=True)
    return runs


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gw", default="http://127.0.0.1:8088", help="Web 网关地址 (实例的 web 端口)")
    ap.add_argument("--scenario", default="", help="先切换到该场景 (空=当前场景)")
    ap.add_argument("--goals", default="", help="目标列表 JSON [[x,y(,yaw)],...]；空=当前场景所有工位")
    ap.add_argument("--planner", default="", help="nav2 | dijkstra (空=当前)")
    ap.add_argument("--tmax", type=float, default=180.0, help="单个目标超时 (s)")
    ap.add_argument("--tol", type=float, default=20.0, help="合格阈值 (mm)")
    ap.add_argument("--out", default="precision_result.json")
    a = ap.parse_args()
    gw = a.gw.rstrip("/")
    if a.goals:
        goals = json.loads(a.goals)
    else:
        t = _req(gw, "/api/telemetry?full=1")
        sts = ((t.get("scenario_metadata") or {}).get("stations")) or []
        goals = [[s["x"], s["y"], s.get("dock_yaw", 0.0)] for s in sts]
        if not goals:
            sys.exit("当前场景没有工位，请用 --goals 指定目标")
    runs = run(gw, a.scenario, goals, a.planner, a.tmax)
    ok = [r for r in runs if r["status"] == "ARRIVED"]
    worst = lambda k: max((r[k] for r in ok if r[k] is not None), default=None)  # noqa: E731
    summ = {"arrived": f"{len(ok)}/{len(runs)}", "xte_max_mm": worst("xte_max_mm"), "end_err_max_mm": worst("end_err_mm"),
            "loc_err_max_mm": worst("loc_err_max_mm"), "tol_mm": a.tol}
    summ["pass"] = len(ok) == len(runs) and all(v is not None and v <= a.tol for v in (summ["xte_max_mm"], summ["end_err_max_mm"]))
    print("汇总:", json.dumps(summ, ensure_ascii=False))
    with open(a.out, "w") as f:
        json.dump({"summary": summ, "runs": runs}, f, ensure_ascii=False)
    print(f"明细已保存 {a.out}")


if __name__ == "__main__":
    main()
