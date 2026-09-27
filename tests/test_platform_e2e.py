#!/usr/bin/env python3
"""
资源管理平台端到端测试 (无 Docker / 无 ROS): agv-hub + 两个节点代理 (process 运行时) → 分离部署一个实例
  1. 节点注册与心跳、节点镜像自动登记为程序包
  2. 场景包 (PGM + 拓扑) 与 cmodel 上传解析、程序包 (docker save) 上传解析
  3. 部署校验 → 分离部署 (仿真 pi-A / 执行 pi-B) → 运行中
  4. 工作台 (经平台代理): 控制权锁、任务流 (移动/顶升/等待/下降)、暂停注入人员 → 激光走廊停车 → 移除后完成
  5. 仿真记录 sim_bundle (契约字段) 与平台归档、环境重置、终止实例
用法: python3 tests/test_platform_e2e.py   (约 3 分钟)
"""
import gzip
import io
import json
import os
import shutil
import signal
import subprocess
import sys
import tarfile
import time
import urllib.error
import urllib.request
import zipfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TMP = "/tmp/agv_platform_e2e"
HUB = "http://127.0.0.1:18480"
ok = True
procs = []


def req(url, method="GET", body=None, headers=None, raw=None, timeout=20):
    h = {"Content-Type": "application/json"}
    h.update(headers or {})
    data = raw if raw is not None else (json.dumps(body).encode() if body is not None else None)
    r = urllib.request.Request(url, data=data, method=method, headers=h)
    try:
        with urllib.request.urlopen(r, timeout=timeout) as f:
            b = f.read()
            return json.loads(b) if "json" in f.headers.get("Content-Type", "") else b
    except urllib.error.HTTPError as e:
        return {"HTTP": e.code, "error": e.read().decode("utf-8", "replace")[:300]}
    except (urllib.error.URLError, OSError) as e:
        return {"HTTP": 0, "error": str(e)}


def check(name, cond, info=""):
    global ok
    ok &= bool(cond)
    print(("  PASS " if cond else "  FAIL ") + name + (f"  {info}" if info else ""), flush=True)


def spawn(args, env, log):
    e = dict(os.environ)
    e.update(env)
    p = subprocess.Popen(args, cwd=ROOT, env=e, stdout=open(log, "wb"), stderr=subprocess.STDOUT, start_new_session=True)
    procs.append(p)
    return p


def wait_until(fn, timeout, step=1.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        v = fn()
        if v:
            return v
        time.sleep(step)
    return None


def main():
    shutil.rmtree(TMP, ignore_errors=True)
    os.makedirs(TMP)
    spawn([sys.executable, "-m", "hub.server"], {"HUB_PORT": "18480", "HUB_DATA": f"{TMP}/hub"}, f"{TMP}/hub.log")
    wait_until(lambda: isinstance(req(HUB + "/api/hub/health"), dict) and req(HUB + "/api/hub/health").get("ok"), 20, 0.5)
    tok = open(f"{TMP}/hub/cluster_token").read().strip()
    for n, port, rng, kind in (("pi-A", 18470, "18500-18549", "hybrid"), ("pi-B", 18471, "18550-18599", "controller")):
        spawn([sys.executable, "-m", "agent.server"], {"AGENT_NAME": n, "AGENT_PORT": str(port), "AGENT_PORT_RANGE": rng,
                                                        "AGENT_DATA": f"{TMP}/{n}", "AGENT_RUNTIME": "process", "AGENT_KIND": kind,
                                                        "HUB_API": HUB, "JOIN_TOKEN": tok}, f"{TMP}/{n}.log")
    print("[1] 节点注册")
    nodes = wait_until(lambda: (lambda d: d if len(d) == 2 and all(n["online"] for n in d) else None)(req(HUB + "/api/hub/nodes").get("nodes", [])), 30)
    check("两个节点在线", nodes, [(n["name"], n["kind"]) for n in nodes or []])
    ids = {n["name"]: n["id"] for n in nodes or []}
    pk = req(HUB + "/api/hub/packages")["packages"]
    check("节点镜像自动登记为程序包", {p["kind"] for p in pk} == {"sim", "nav"}, [p["image_ref"] for p in pk])
    check("错误令牌被拒绝", req(HUB + "/api/hub/nodes/register", "POST", {"name": "x", "token": "bad"}).get("HTTP") == 401)

    print("[2] 仓库上传")
    W, H = 300, 200
    px = bytearray([254] * (W * H))
    for y in range(H):
        for x in range(W):
            if x < 3 or x >= W - 3 or y < 3 or y >= H - 3 or (130 < x < 170 and 70 < y < 130):
                px[y * W + x] = 0
    zb = io.BytesIO()
    with zipfile.ZipFile(zb, "w") as z:
        z.writestr("m.pgm", f"P5\n{W} {H}\n255\n".encode() + bytes(px))
        z.writestr("m.yaml", "image: m.pgm\nresolution: 0.05\norigin: [-7.5, -5.0, 0]\nnegate: 0\noccupied_thresh: 0.65\n")
        z.writestr("topology.json", json.dumps({"nodes": {"A": [-5, 0], "B": [5, 0], "C": [0, 3.5]}, "connections": [["A", "C"], ["C", "B"]],
                                                "stations": [{"id": "P0", "name": "起点", "x": -5, "y": 0}, {"id": "P1", "name": "终点", "x": 5, "y": 0}]}))
    sc = req(HUB + "/api/hub/scenes/upload?filename=t.zip&name=grid_test", "POST", raw=zb.getvalue())
    check("PGM 场景包 → 墙体", sc.get("summary", {}).get("walls", 0) > 4 and sc["summary"]["map"] == "pgm", sc.get("summary"))
    cm = open(os.path.join(ROOT, "tests", "data", os.listdir(os.path.join(ROOT, "tests", "data"))[0]), "rb").read()
    m = req(HUB + "/api/hub/models/upload?filename=E2E.cmodel&material_no=10000001-01&project=e2e", "POST", raw=cm)
    check("cmodel 上传解析", m.get("summary", {}).get("wheels"), (m.get("id"), m.get("latest"), m.get("audit")))
    ed = req(f"{HUB}/api/hub/models/{m['id']}/versions/v1/api/v1/model/editor")
    check("离线补全编辑器接口", "audit" in ed and "sensors" in ed)
    tb = io.BytesIO()
    with tarfile.open(fileobj=tb, mode="w") as t:
        for name, data in (("manifest.json", json.dumps([{"Config": "c.json", "RepoTags": ["agv-nav:e2e"], "Layers": []}]).encode()),
                           ("c.json", json.dumps({"architecture": "arm64", "config": {"Labels": {"org.agv.kind": "nav"}}}).encode()),
                           ("pad", os.urandom(4000))):
            ti = tarfile.TarInfo(name)
            ti.size = len(data)
            t.addfile(ti, io.BytesIO(data))
    up = req(HUB + "/api/hub/packages/upload?note=e2e", "POST", raw=gzip.compress(tb.getvalue()), headers={"Content-Type": "application/x-tar"})
    check("程序包上传解析", up.get("kind") == "nav" and up.get("arch") == "arm64", up)

    print("[3] 部署")
    psim = next(p for p in req(HUB + "/api/hub/packages?kind=sim")["packages"] if p["source"] == "process")
    pnav = next(p for p in req(HUB + "/api/hub/packages?kind=nav")["packages"] if p["source"] == "process")
    body = {"sim_node": ids["pi-A"], "nav_node": ids["pi-B"], "model_id": m["id"], "scene_id": "grid_9_square", "sim_pkg": psim["id"], "nav_pkg": pnav["id"]}
    bad = req(HUB + "/api/hub/deployments/check", "POST", dict(body, sim_node=ids["pi-B"]))
    check("控制器节点不能跑仿真 (校验拦截)", not bad["ok"], bad["issues"])
    chk = req(HUB + "/api/hub/deployments/check", "POST", body)
    check("部署校验", chk["ok"] and chk["split"], chk)
    inst = req(HUB + "/api/hub/instances", "POST", body, headers={"X-User": "e2e"})
    iid = inst["id"]
    fin = wait_until(lambda: (lambda d: d if d["status"] in ("running", "error") else None)(req(f"{HUB}/api/hub/instances/{iid}")), 150, 2)
    check("实例运行中 (分离部署)", fin and fin["status"] == "running", fin and fin.get("error"))
    if not fin or fin["status"] != "running":
        return
    I = f"{HUB}/inst/{iid}"
    info = req(I + "/api/v2/info")
    check("平台模型/场景已加载到实例", info["model"].get("model_id") == m["id"] and info["scene"]["id"] == "grid_9_square", info["model"])

    print("[4] 工作台")
    lk = req(I + "/api/v2/lock", "POST", {"user": "e2e"})
    T = {"X-Lock-Token": lk["token"]}
    check("无令牌写操作 423", req(I + "/api/v2/pause", "POST", {"paused": True}).get("HTTP") == 423)
    check("他人抢锁 409", req(I + "/api/v2/lock", "POST", {"user": "other"}).get("HTTP") == 409)
    flow = {"id": "TF-E2E", "name": "e2e", "tid": "90001", "loop": "single",
            "steps": [{"type": "move", "target": "S4", "speed": 1.0}, {"type": "lift"}, {"type": "wait", "seconds": 1}, {"type": "drop"},
                      {"type": "move", "target": "P0", "speed": 0.8}]}
    check("下发任务流", req(I + "/api/v2/taskflow/run", "POST", {"flow": flow}, headers=T).get("status") == "running")
    b = wait_until(lambda: (lambda x: x if x["taskflow"]["step_index"] == 4 and x["nav_status"] == "NAVIGATING" else None)(req(I + "/api/v2/brief")), 60, 0.5)
    check("顶升/等待/下降工步完成，返回 P0", b, b and b["pose"])
    req(I + "/api/v2/pause", "POST", {"paused": True}, headers=T)
    el = req(I + "/api/v2/inject", "POST", {"type": "person", "placement": "ahead_2.5"}, headers=T)
    check("注入人员 (路径前方 2.5 m)", el.get("id"), (el.get("x"), el.get("y")) if el.get("id") else el)
    req(I + "/api/v2/pause", "POST", {"paused": False}, headers=T)
    z = wait_until(lambda: (lambda t: t if (t.get("safety") or {}).get("obstacle", {}).get("zone") == "stop" and abs(t.get("vx", 1)) < 0.02 else None)(
        req(I + "/api/telemetry")), 40, 0.5)
    check("激光走廊: 减速后停车等待", z, z and z["safety"]["obstacle"])
    evs = req(I + "/api/v2/events?after=0&tag=OBS")["events"]
    check("OBS 事件", any(("减速" in e["title"] or "预警" in e["title"]) for e in evs) and any("停车" in e["title"] for e in evs), [e["title"] for e in evs][-3:])
    for _ in range(3):
        req(I + "/api/v2/lock", "PUT", headers=T)
        time.sleep(1)
    req(I + f"/api/v2/inject/{el['id']}", "DELETE", headers=T)
    d = wait_until(lambda: (lambda x: x if x["taskflow"]["status"] in ("done", "failed") else None)(req(I + "/api/v2/brief")), 60, 1)
    check("移除后继续并完成任务流", d and d["taskflow"]["status"] == "done", d and d["taskflow"])

    print("[5] 记录/重置/终止")
    rec = wait_until(lambda: (lambda r: r if r["records"] else None)(req(I + "/api/v2/records")), 10)
    r0 = rec["records"][0] if rec else {}
    check("仿真记录", r0.get("result") in ("AVOIDED", "SUCCESS") and r0.get("injections") == 1, r0)
    bd = req(I + f"/api/v2/records/{r0.get('id')}/bundle")
    keys = {"metadata", "taskFlow", "environment", "vehicleModel", "injectedElements", "events", "trajectory"}
    check("sim_bundle 契约字段", keys <= set(bd) and {"time", "x", "y", "yaw", "vx", "w", "obsDist", "status"} <= set(bd["trajectory"][0]),
          f"{len(bd['trajectory'])} 帧 {len(bd['events'])} 事件")
    arch = wait_until(lambda: req(HUB + f"/api/hub/records?instance={iid}")["records"], 10)
    check("记录归档到平台", arch, arch and arch[0]["id"])
    req(I + "/api/v2/reset", "POST", {}, headers=T)
    time.sleep(1.5)
    p = req(I + "/api/v2/brief")["pose"]
    check("重置回 P0", abs(p["x"]) < 0.05 and abs(p["y"]) < 0.05, p)
    st = req(f"{HUB}/api/hub/instances/{iid}/stop", "POST", {})
    check("终止实例", st.get("status") == "stopped", st.get("error"))
    left = wait_until(lambda: not any(c.get("instance") == iid and c.get("state") == "running"
                                      for n in req(HUB + "/api/hub/nodes")["nodes"] for c in n["containers"]), 15)
    check("两个节点上的实例进程已清理", left)


if __name__ == "__main__":
    try:
        main()
    finally:
        try:
            for i in req(HUB + "/api/hub/instances?active=1").get("instances", []):
                req(f"{HUB}/api/hub/instances/{i['id']}/stop", "POST", {})
        except Exception:
            pass
        for p in procs:
            try:
                os.killpg(p.pid, signal.SIGTERM)
            except Exception:
                pass
        time.sleep(1)
        subprocess.run(["bash", "-c", "pgrep -f 'agv_platform_e2e' | xargs -r kill 2>/dev/null"], check=False)
    print("\n结果: " + ("全部通过" if ok else "存在失败"))
    sys.exit(0 if ok else 1)
