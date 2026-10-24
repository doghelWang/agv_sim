#!/usr/bin/env python3
"""Task-flow level forklift test: pick a pallet (with a box stacked on it), carry it across the workshop, set it down, pick it again and stack
it on a second pallet.  Needs sim-server (forklift cmodel loaded, scenario grid_9_square) + nav-runtime.
usage: test_forklift_flow.py [sim_url] [nav_url]"""
import json, math, sys, time, urllib.request
S = (sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:18100").rstrip("/") + "/api/v1"
N = (sys.argv[2] if len(sys.argv) > 2 else "http://127.0.0.1:18102").rstrip("/") + "/api/v1"
FAILS = []
def rq(base, m, p, b=None):
    r = urllib.request.Request(base + p, method=m, data=None if b is None else json.dumps(b).encode(), headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(r, timeout=15) as f: return json.loads(f.read() or b"null")
    except urllib.error.HTTPError as e: return {"_http": e.code, "_body": e.read().decode()[:300]}
def check(n, ok, i=""):
    print(("PASS " if ok else "FAIL ") + n + (f"  [{i}]" if i else "")); 
    if not ok: FAILS.append(n)
def cargo(): return {c["id"]: c for c in rq(S, "GET", "/world/cargo")}

def run_flow(steps, timeout=900):
    r = rq(N, "POST", "/taskflows/run", {"id": "fk", "name": "forklift", "steps": steps})
    if "_http" in r: print("flow rejected", r); return None
    t0 = time.time(); last = -1
    while time.time() - t0 < timeout:
        v = rq(N, "GET", "/taskflows/current")
        if v.get("step_index") != last: last = v.get("step_index"); print(f"  .. step {last+1}/{len(steps)} {steps[last]['type']} [{time.time()-t0:.0f}s]")
        if v["status"] in ("done", "failed"): return v
        time.sleep(1.0)
    return rq(N, "GET", "/taskflows/current")

def main():
    for c in list(cargo()): rq(S, "DELETE", f"/world/cargo/{c}")
    rq(S, "PUT", "/fork", {"height": 0.0}); time.sleep(0.5)
    rq(S, "POST", "/sim/reset", {"x": 0.0, "y": 0.0, "yaw": 0.0}); time.sleep(0.5)
    rq(S, "POST", "/world/cargo", {"kind": "pallet", "id": "P1", "x": 5.0, "y": 0.0, "yaw": 0.0})
    rq(S, "POST", "/world/cargo", {"kind": "box", "id": "B1", "x": 5.0, "y": 0.0, "size": [0.6, 0.4, 0.5]})
    rq(S, "POST", "/world/cargo", {"kind": "pallet", "id": "P2", "x": 0.0, "y": -5.0, "yaw": math.pi / 2})
    v = run_flow([{"type": "pick", "cargo": "P1"}, {"type": "place", "x": -5.0, "y": 0.0, "yaw": math.pi}])
    check("flow 1 (pick + place on floor) done", v and v["status"] == "done" and all(l["ok"] for l in v.get("log", [])), json.dumps(v.get("log", [])[-2:], ensure_ascii=False)[:300] if v else "")
    cg = cargo(); f = rq(S, "GET", "/fork"); tr = rq(S, "GET", "/state")["truth"]
    check("P1 delivered near (-5,0)", math.hypot(cg["P1"]["x"] + 5.0, cg["P1"]["y"]) < 0.15 and cg["P1"]["z0"] == 0.0, f"P1=({cg['P1']['x']:.2f},{cg['P1']['y']:.2f}) z0={cg['P1']['z0']}")
    check("box delivered with pallet", abs(cg["B1"]["x"] - cg["P1"]["x"]) < 0.02 and cg["B1"].get("parent") == "P1")
    check("fork empty and lowered", not f["loaded"] and f["height_m"] < 0.2, f"h={f['height_m']}")
    v = run_flow([{"type": "pick", "cargo": "P1"}, {"type": "place", "x": 0.0, "y": -5.0, "yaw": -math.pi / 2, "z": 0.144}])
    check("flow 2 (pick + stack on P2) done", v and v["status"] == "done" and all(l["ok"] for l in v.get("log", [])), json.dumps(v.get("log", [])[-2:], ensure_ascii=False)[:300] if v else "")
    cg = cargo()
    check("P1 stacked on P2", abs(cg["P1"]["z0"] - 0.144) < 0.01 and cg["P1"].get("parent") == "P2", f"z0={cg['P1']['z0']} parent={cg['P1'].get('parent')} xy=({cg['P1']['x']:.2f},{cg['P1']['y']:.2f})")
    check("stack aligned over P2", math.hypot(cg["P1"]["x"], cg["P1"]["y"] + 5.0) < 0.15)
    check("box now at z0=0.288", abs(cg["B1"]["z0"] - 0.288) < 0.01, f"{cg['B1']['z0']}")
    ev = rq(S, "GET", "/events?limit=400")["events"]
    kinds = {}
    for e in ev: kinds[e["type"]] = kinds.get(e["type"], 0) + 1
    print("events:", {k: v for k, v in kinds.items() if k.startswith("FORK") or k in ("COLLISION", "BUMPER")})
    check("no body collisions during flows", kinds.get("COLLISION", 0) == 0)
    print("\nRESULT:", "ALL PASS" if not FAILS else f"{len(FAILS)} FAILED: {FAILS}")
    return 1 if FAILS else 0
if __name__ == "__main__": sys.exit(main())
