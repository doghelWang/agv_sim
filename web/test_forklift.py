#!/usr/bin/env python3
"""End-to-end forklift test against a running sim-server loaded with models/forklift_4steer.cmodel.
usage: test_forklift.py [base_url]   (default http://127.0.0.1:18100)"""
import json, math, sys, time, urllib.request

BASE = (sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:18100").rstrip("/") + "/api/v1"
FAILS = []
TIP = 2.25

def req(m, p, b=None):
    r = urllib.request.Request(BASE + p, method=m, data=None if b is None else json.dumps(b).encode(), headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(r, timeout=10) as f: return json.loads(f.read() or b"null")
    except urllib.error.HTTPError as e: return {"_http": e.code, "_body": e.read().decode()[:200]}

def check(name, ok, info=""):
    print(("PASS " if ok else "FAIL ") + name + (f"  [{info}]" if info else ""))
    if not ok: FAILS.append(name)

def pose(): return req("GET", "/state")["truth"]
def fork(): return req("GET", "/fork")
def cargo(): return {c["id"]: c for c in req("GET", "/world/cargo")}

def drive(vx, vy=0.0, wz=0.0, secs=0.0):
    t0 = time.time()
    while time.time() - t0 < secs:
        req("PUT", "/control/cmd_vel", {"vx": vx, "vy": vy, "wz": wz}); time.sleep(0.05)

def drive_to_x(xt, vmax=0.5, tol=0.02, timeout=25, y_hold=None):
    t0 = time.time()
    while time.time() - t0 < timeout:
        p = pose(); e = xt - p["x"]
        if abs(e) < tol: break
        v = max(-vmax, min(vmax, 1.2 * e))
        req("PUT", "/control/cmd_vel", {"vx": v, "vy": 0.0, "wz": 0.0}); time.sleep(0.05)
    req("PUT", "/control/cmd_vel", {"vx": 0, "vy": 0, "wz": 0}); time.sleep(0.4)
    return pose()

def wait_fork(h, timeout=20):
    t0 = time.time()
    while time.time() - t0 < timeout:
        f = fork()
        if abs(f["height_m"] - h) < 0.01 and abs(f["velocity_mps"]) < 0.01: return f
        time.sleep(0.1)
    return fork()

def reset(x, y, yaw=0.0):
    req("POST", "/sim/reset", {"x": x, "y": y, "yaw": yaw}); time.sleep(0.3)

def main():
    print("backend:", req("GET", "/sim")["backend"], " chassis:", req("GET", "/state")["chassis"])
    f = fork()
    global TIP
    TIP = f["tine"]["root_x"] + f["tine"]["length"]
    check("fork present", isinstance(f, dict) and f.get("mast_max_m") == 3.0, str(f)[:80])
    for c in list(cargo()): req("DELETE", f"/world/cargo/{c}")

    # --- cargo spawn + stacking
    r = req("POST", "/world/cargo", {"kind": "pallet", "id": "P1", "x": 5.0, "y": 0.0, "yaw": 0.0})
    r2 = req("POST", "/world/cargo", {"kind": "box", "id": "B1", "x": 5.0, "y": 0.0, "size": [0.6, 0.4, 0.5]})
    cg = cargo()
    check("pallet spawned on floor", cg["P1"]["z0"] == 0.0)
    check("box auto-stacked on pallet", abs(cg["B1"]["z0"] - 0.144) < 1e-6 and cg["B1"].get("parent") == "P1", f"z0={cg['B1']['z0']} parent={cg['B1'].get('parent')}")

    # --- misaligned approach must be blocked by pallet runners
    reset(0.0, 0.17, 0.0)
    p = drive_to_x(5.45 - TIP, timeout=15)
    ev = [e["kind"] for e in req("GET", "/events?limit=30")] if isinstance(req("GET", "/events?limit=30"), list) else []
    check("misaligned insertion stopped by runner", p["x"] < 4.55 - TIP, f"x={p['x']:.2f} (tine tip would be at {p['x']+TIP:.2f}, pallet edge 4.4)")
    check("pallet not moved by collision", abs(cargo()["P1"]["x"] - 5.0) < 0.01)

    # --- aligned approach, insert, lift
    reset(0.0, 0.0, 0.0)
    time.sleep(0.2)
    p = drive_to_x(5.35 - TIP, vmax=0.4, tol=0.03)
    check("aligned insertion reaches depth", abs(p["x"] - (5.35 - TIP)) < 0.08, f"x={p['x']:.3f} y={p['y']:.3f}")
    check("pallet-detect DI on", req("GET", "/io")["inputs"].get("di_pallet_detect") is True)
    req("PUT", "/fork", {"height": 0.5}); f = wait_fork(0.5)
    check("fork reached 0.5 m", abs(f["height_m"] - 0.5) < 0.01, f"h={f['height_m']}")
    check("pallet attached", f["loaded"] and f["load_id"] == "P1" and len(f["load"]) == 2, f"loaded={f['loaded']} n={len(f['load'])}")
    cg = cargo()
    check("pallet lifted with tines", abs(cg["P1"]["z0"] - (0.5 + 0.06 - 0.10)) < 0.01, f"z0={cg['P1']['z0']}")
    check("box rides on pallet", abs(cg["B1"]["z0"] - (cg["P1"]["z0"] + 0.144)) < 0.01)
    check("DI cargo present", req("GET", "/io")["inputs"].get("di_cargo_present") is True)

    # --- carry: back out and drive with load (speed derating), crab sideways with 4 steer wheels
    p = drive_to_x(0.5, vmax=0.8)
    c1 = cargo()["P1"]
    dx = c1["x"] - p["x"]
    check("pallet follows vehicle", 1.5 < dx < 3.0, f"rel_x={dx:.2f}")
    t0 = pose()
    drive(0.0, 0.5, 0.0, 2.0); req("PUT", "/control/cmd_vel", {"vx": 0, "vy": 0, "wz": 0}); time.sleep(0.5)
    t1 = pose()
    check("crab (sideways) motion with load", t1["y"] - t0["y"] > 0.3, f"dy={t1['y']-t0['y']:.2f}")
    c2 = cargo()["P1"]
    check("pallet keeps relative pose while carried", abs((c2["y"] - t1["y"]) - (c1["y"] - t0["y"])) < 0.05)

    # --- place on floor: lower, tines go free, back out
    req("PUT", "/fork", {"height": 0.0}); f = wait_fork(0.0)
    check("pallet released on floor", not f["loaded"] and abs(cargo()["P1"]["z0"]) < 1e-3, f"z0={cargo()['P1']['z0']} loaded={f['loaded']}")
    pl = cargo()["P1"]
    x_before = pose()["x"]
    drive(-0.4, 0, 0, 3.0); req("PUT", "/control/cmd_vel", {"vx": 0, "vy": 0, "wz": 0}); time.sleep(0.4)
    check("backed out leaving pallet", cargo()["P1"]["x"] == pl["x"] and pose()["x"] < x_before - 0.5)
    check("box stays on placed pallet", abs(cargo()["B1"]["z0"] - 0.144) < 0.01)

    # --- stack: pick the box-laden pallet again and put on top of second pallet
    req("POST", "/world/cargo", {"kind": "pallet", "id": "P2", "x": pl["x"] - 3.0, "y": pl["y"] - 2.5, "yaw": math.pi / 2})
    # re-pick P1
    pp = pose(); reset(pl["x"] - 4.0, pl["y"], 0.0)
    drive_to_x(pl["x"] - 1.9, tol=0.03)
    req("PUT", "/fork", {"height": 0.3}); f = wait_fork(0.3)
    check("re-pick works from placed state", f["loaded"] and f["load_id"] == "P1")
    # top height
    req("PUT", "/fork", {"action": "top"}); f = wait_fork(3.0, 30)
    check("mast reaches top limit", f["limits"]["top"] and f["height_m"] >= 2.99, f"h={f['height_m']}")
    check("DI lift_top", req("GET", "/io")["inputs"].get("di_lift_top") is True)
    req("PUT", "/fork", {"height": 0.2}); wait_fork(0.2, 30)
    # lowering to floor-level target should release
    req("PUT", "/fork", {"height": 0.0}); f = wait_fork(0.0, 10)
    check("final lower to floor releases", not f["loaded"])

    # --- raising tines into a wall-like obstacle / blocked reporting
    reset(-8.0, 0.0, 0.0)
    req("POST", "/world/cargo", {"kind": "box", "id": "WALLBOX", "x": -3.5, "y": 0.0, "size": [0.6, 1.2, 2.0]})
    p = drive_to_x(-4.0, vmax=0.6, timeout=14)
    check("tines stop before solid box", p["x"] < -3.75 - TIP and cargo()["WALLBOX"]["x"] == -3.5, f"x={p['x']:.2f}")

    # --- cleanup
    for c in list(cargo()): req("DELETE", f"/world/cargo/{c}")
    check("cargo cleared", cargo() == {})
    print("\nRESULT:", "ALL PASS" if not FAILS else f"{len(FAILS)} FAILED: {FAILS}")
    return 1 if FAILS else 0

if __name__ == "__main__": sys.exit(main())
