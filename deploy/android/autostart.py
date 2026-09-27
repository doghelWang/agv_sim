#!/usr/bin/env python3
"""
开机自动拉起实例 (Termux 原生 Python，只用标准库；start_agv.sh 在 AGV_AUTOSTART=1 时调用)

  python3 ~/autostart.py --hub http://127.0.0.1:8082 --node pixel4 [--scene grid_9_square] [--model m-model]

流程: 等平台与本机节点在线 → 本机已有实例在运行且网关能访问就直接用 →
      否则重启本机最近的一个实例 (保留它的 SLAM 地图) → 没有就新部署一个 (仿真 + 执行都在本机) →
      等到运行中，打印工作台地址 (并写入 ~/agv_url.txt，装了 Termux:API 时发一条通知)。
环境变量 (~/.agv.env): AGV_AUTOSTART_INSTANCE 指定实例号 (如 i03)；AGV_AUTOSTART_SCENE / AGV_AUTOSTART_MODEL 新部署时用。
"""
import argparse
import json
import os
import socket
import subprocess
import sys
import time
import urllib.request

HUB = ""


def log(*a):
    print(time.strftime("%H:%M:%S"), "[autostart]", *a, flush=True)


def api(method, path, body=None, timeout=30):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(HUB + path, data=data, method=method, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode() or "{}")


def api_url(url, body):
    req = urllib.request.Request(url, data=json.dumps(body).encode(), method="PUT", headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.loads(r.read().decode() or "{}").get("planner")


def reachable(url, timeout=3):
    try:
        urllib.request.urlopen(url, timeout=timeout)
        return True
    except Exception:
        return False


def wlan_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"


def wait(pred, timeout, step=3):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            v = pred()
            if v:
                return v
        except Exception:
            pass
        time.sleep(step)
    return None


def node_id(name):
    for n in api("GET", "/api/hub/nodes")["nodes"]:
        if n.get("name") == name and n.get("online"):
            return n["id"]
    return None


def local(i, nid):
    return i.get("sim_node") == nid and (i.get("nav_node") or i.get("sim_node")) == nid


def web_port(i):
    return (i.get("ports") or {}).get("web")


def main():
    global HUB
    ap = argparse.ArgumentParser()
    ap.add_argument("--hub", default="http://127.0.0.1:8082")
    ap.add_argument("--node", default=os.environ.get("AGENT_NAME", "phone"))
    ap.add_argument("--instance", default=os.environ.get("AGV_AUTOSTART_INSTANCE", ""))
    ap.add_argument("--scene", default=os.environ.get("AGV_AUTOSTART_SCENE", "grid_9_square"))
    ap.add_argument("--model", default=os.environ.get("AGV_AUTOSTART_MODEL", ""))
    ap.add_argument("--planner", default=os.environ.get("AGV_AUTOSTART_PLANNER", "dijkstra"),
                    help="实例起来后设置的规划器: dijkstra (自研，手机上精度更好) / nav2 / 空 = 不设置")
    ap.add_argument("--timeout", type=float, default=900)
    a = ap.parse_args()
    HUB = a.hub.rstrip("/")

    if not wait(lambda: reachable(HUB + "/api/hub/health"), 180):
        log("平台没有启动:", HUB)
        return 1
    nid = wait(lambda: node_id(a.node), 180)
    if not nid:
        log(f"节点 {a.node} 没有在平台上线")
        return 1
    insts = [i for i in api("GET", "/api/hub/instances")["instances"] if local(i, nid)]
    insts.sort(key=lambda i: i.get("created") or 0, reverse=True)

    target = None
    for i in insts:          # 已在运行且网关能访问就不动 (重启手机后平台记录可能还是「运行中」，但进程已经没了)
        if i.get("status") == "running" and web_port(i) and reachable(f"http://127.0.0.1:{web_port(i)}/", 5):
            target = i
            log("实例已在运行:", i["id"])
            break
    if target is None:
        pick = next((i for i in insts if i["id"] == a.instance), None) if a.instance else (insts[0] if insts else None)
        if pick:
            log("重启实例", pick["id"], pick.get("name", ""))
            target = api("POST", f"/api/hub/instances/{pick['id']}/restart", {}, timeout=300)
        else:
            pk = api("GET", "/api/hub/packages")["packages"]
            sim_pkg = next((p["id"] for p in pk if p.get("kind") == "sim" and p.get("source") == "process"), None)
            nav_pkg = next((p["id"] for p in pk if p.get("kind") == "nav" and p.get("source") == "process"), None)
            models = api("GET", "/api/hub/models")["models"]
            m = next((x for x in models if x["id"] == a.model), models[0] if models else None)
            if not (sim_pkg and nav_pkg and m):
                log("缺少程序包或车辆模型，无法部署", sim_pkg, nav_pkg, m and m["id"])
                return 1
            log("新部署实例: 模型", m["id"], "场景", a.scene)
            target = api("POST", "/api/hub/instances", {"model_id": m["id"], "model_ver": m.get("latest"), "scene_id": a.scene,
                                                        "sim_node": nid, "nav_node": nid, "sim_pkg": sim_pkg, "nav_pkg": nav_pkg,
                                                        "name": "开机自启"})
    iid = target["id"]
    i = wait(lambda: (lambda x: x if x["status"] in ("running", "error", "stopped") else None)(
        api("GET", f"/api/hub/instances/{iid}")), a.timeout, 5)
    if not i or i["status"] != "running":
        log("实例没有进入运行状态:", iid, i and i.get("status"), i and i.get("error"))
        return 1
    ip = wlan_ip()
    port = web_port(i)
    nav_port = (i.get("ports") or {}).get("nav_api")
    if a.planner and nav_port:   # 执行进程在手机上时 Nav2 跟线偏差 200~300 mm，自研导引约 50 mm (PERFORMANCE.md 第 4 节)
        ok = wait(lambda: api_url(f"http://127.0.0.1:{nav_port}/api/v1/nav/planner", {"type": a.planner}) == a.planner, 180, 5)
        log("规划器", a.planner, "已设置" if ok else "设置失败 (可在工作台里手动选)")
    hub_port = HUB.rsplit(":", 1)[-1]
    lines = [f"实例 {iid} 运行中",
             f"资源平台 (本机浏览器): http://127.0.0.1:{hub_port}",
             f"资源平台 (同网络其它设备): http://{ip}:{hub_port}",
             f"工作台: http://{ip}:{hub_port}/inst/{iid}/  (或在资源平台的实例列表里点「工作台」)"]
    for ln in lines:
        log(ln)
    with open(os.path.expanduser("~/agv_url.txt"), "w") as f:
        f.write(time.strftime("%F %T") + "\n" + "\n".join(lines) + "\n")
    try:
        subprocess.run(["termux-notification", "--id", "agv", "--title", f"AMR 仿真 {iid} 已启动",
                        "--content", f"资源平台 http://{ip}:{hub_port}",
                        "--button1", "打开工作台", "--button1-action", f"termux-open-url http://127.0.0.1:{hub_port}/inst/{iid}/"],
                       timeout=15, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
