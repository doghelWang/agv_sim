#!/usr/bin/env python3
"""
手机 / 板卡 (树莓派、RK3588 …) 运行性能对比测试 (在手机 Termux 原生 Python 里运行，只用标准库)

  python3 perf_matrix.py --hub http://<主平台>:8082 --board-ssh user@<板卡IP> \
          --board-node <板卡节点名> --phone-node <手机节点名> --out ~/perf_matrix
  python3 perf_matrix.py ... --combos B,D               # 只测部分
前提: 两台设备都已接入同一主平台 (板卡用 Docker 运行时、手机用 process 运行时)；
      手机 Termux 能免密 ssh 到板卡 (ssh-copy-id)；板卡上项目目录 --board-repo 里有 tools/perf_sample.py。

组合 (同一车型、同一场景、同一组目标):
  A  仿真 板卡  + 执行 板卡        B  仿真 手机 + 执行 手机
  C  仿真 手机  + 执行 板卡        D  仿真 板卡 + 执行 手机
每个组合: 经主平台部署实例 → 等 Nav2 激活 → 静置 → 分别用 nav2 / dijkstra 跑一遍精度测试，
测试期间两台设备同时运行 tools/perf_sample.py 采样 (CPU/内存/频率/温度/RTF/状态频率/TF 延迟)。
结束后恢复测试前在运行的实例。结果: <out>/<组合>/…json 与 <out>/summary.json
"""
import argparse
import json
import os
import subprocess
import sys
import time
import urllib.request

HUB = ""
PI = ""                    # 板卡 ssh 目标 user@host (内部仍用键名 "pi" 表示板卡)
PI_REPO = "~/agv_sim"
PHONE_ROOT = os.path.expandvars("$PREFIX/var/lib/proot-distro/containers/" + os.environ.get("AGV_DISTRO", "ubuntu") + "/rootfs")
PHONE_REPO = PHONE_ROOT + "/opt/agv"
NODES, HOSTS, PKGS = {}, {}, {}
COMBOS = {"A": ("pi", "pi"), "B": ("phone", "phone"), "C": ("phone", "pi"), "D": ("pi", "phone")}
GOALS = "[[0,5,1.5708],[5,0,0],[0,-5,-1.5708],[-5,0,3.1416],[0,0,0]]"


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def api(method, path, body=None, timeout=30):
    data = json.dumps(body).encode() if body is not None else None
    r = urllib.request.Request(HUB + path, data=data, method=method, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(r, timeout=timeout) as resp:
        return json.loads(resp.read() or b"{}")


def configure(a):
    """按节点名找节点 id / 局域网地址，按节点取最新的 sim/nav 程序包"""
    global HUB, PI, PI_REPO
    HUB, PI, PI_REPO = a.hub.rstrip("/"), a.board_ssh, a.board_repo
    nodes = api("GET", "/api/hub/nodes")["nodes"]
    for key, name in (("pi", a.board_node), ("phone", a.phone_node)):
        n = next((x for x in nodes if x["name"] == name or x["id"] == name), None)
        if not n:
            sys.exit(f"主平台上没有节点 {name}: 现有 {[x['name'] for x in nodes]}")
        NODES[key] = n["id"]
        ips = ((n.get("info") or {}).get("ips") or [])
        HOSTS[key] = n["host"] if not str(n.get("host", "")).startswith("127.") else (ips[0] if ips else a.hub.split("//")[1].split(":")[0])
    pk = api("GET", "/api/hub/packages")
    pk = pk.get("packages", pk)
    for key in ("pi", "phone"):
        for kind in ("sim", "nav"):
            c = [p for p in pk if p["kind"] == kind and NODES[key] in (p.get("nodes") or [])]
            c.sort(key=lambda p: p.get("image_created") or p.get("created") or 0, reverse=True)
            if not c:
                sys.exit(f"节点 {key} 上没有 {kind} 程序包")
            PKGS[(key, kind)] = c[0]["id"]
    log("节点", NODES, "地址", HOSTS, "程序包", PKGS)


def get(url, timeout=5):
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return json.loads(r.read() or b"{}")
    except Exception:
        return None


def ssh(cmd, timeout=60):
    return subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", PI, cmd],
                          capture_output=True, text=True, timeout=timeout)


def active_instances():
    return [i for i in api("GET", "/api/hub/instances")["instances"] if i["status"] in ("running", "deploying")]


def stop_all():
    for i in active_instances():
        log("停止实例", i["id"])
        try:
            api("POST", f"/api/hub/instances/{i['id']}/stop", {}, timeout=120)
        except Exception as e:
            log("  停止失败", e)
    time.sleep(8)


def deploy(sim, nav, scene, model, ver):
    body = {"name": f"性能对比 仿真{sim}+执行{nav}", "model_id": model, "model_ver": ver, "scene_id": scene,
            "sim_node": NODES[sim], "nav_node": NODES[nav], "sim_pkg": PKGS[(sim, "sim")], "nav_pkg": PKGS[(nav, "nav")]}
    i = api("POST", "/api/hub/instances", body)
    iid, t0 = i["id"], time.time()
    while time.time() - t0 < 420:
        time.sleep(5)
        i = api("GET", f"/api/hub/instances/{iid}")
        if i["status"] in ("running", "error"):
            break
    return i


def wait_nav2(nav_url, timeout=180):
    t0 = time.time()
    while time.time() - t0 < timeout:
        n = get(nav_url + "/api/v1/nav")
        if n and (n.get("nav2") or {}).get("active"):
            return round(time.time() - t0, 1)
        time.sleep(3)
    return None


def start_samplers(tag, sim_url, nav_url, secs, outdir):
    stop_local = f"{outdir}/.stop_{tag}"
    for f in (stop_local,):
        if os.path.exists(f):
            os.remove(f)
    args = ["--secs", str(secs), "--interval", "2", "--sim", sim_url, "--nav", nav_url, "--label", tag]
    lp = subprocess.Popen([sys.executable, f"{PHONE_REPO}/tools/perf_sample.py", *args, "--out", f"{outdir}/phone_{tag}.json",
                           "--stop-file", stop_local], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rp = f"/tmp/perf_{tag}.json"
    ssh(f"rm -f /tmp/.stop_{tag} {rp}; nohup python3 {PI_REPO}/tools/perf_sample.py {' '.join(args)} --out {rp} "
        f"--stop-file /tmp/.stop_{tag} > /dev/null 2>&1 < /dev/null &")
    return lp, stop_local, rp


def stop_samplers(tag, lp, stop_local, rp, outdir):
    open(stop_local, "w").close()
    ssh(f"touch /tmp/.stop_{tag}")
    try:
        lp.wait(timeout=20)
    except Exception:
        lp.kill()
    for _ in range(10):
        r = ssh(f"cat {rp} 2>/dev/null")
        if r.stdout.strip():
            with open(f"{outdir}/pi_{tag}.json", "w") as f:
                f.write(r.stdout)
            break
        time.sleep(2)


def nav2_warnings(nav, iid, since_ts):
    """Nav2 控制/规划频率告警次数 (执行进程日志)"""
    pat = "missed its desired rate"
    if nav == "pi":
        r = ssh(f"docker logs --since {int(since_ts)} agv-nav-{iid} 2>&1 | grep -c '{pat}'")
        txt = r.stdout.strip()
        rates = ssh(f"docker logs --since {int(since_ts)} agv-nav-{iid} 2>&1 | grep -o 'Current loop rate is [0-9.]*' | tail -50").stdout
    else:
        lp = f"{PHONE_ROOT}/root/.agv-agent/logs/agv-nav-{iid}.log"
        txt = subprocess.run(f"grep -a -c '{pat}' {lp}", shell=True, capture_output=True, text=True).stdout.strip()
        rates = subprocess.run(f"grep -a -o 'Current loop rate is [0-9.]*' {lp} | tail -50", shell=True, capture_output=True, text=True).stdout
    vals = [float(x.split()[-1]) for x in rates.split("\n") if x.strip()]
    return {"missed_rate_warnings": int(txt or 0), "reported_loop_rate_min": min(vals) if vals else None,
            "reported_loop_rate_avg": round(sum(vals) / len(vals), 2) if vals else None}


def bench(outdir):
    log("单机基准 bench_host.py (手机 proot / 板卡容器)")
    r = subprocess.run(["proot-distro", "login", os.environ.get("AGV_DISTRO", "ubuntu"), "--", "bash", "-c", "cd /opt/agv && timeout 300 python3 tools/bench_host.py"],
                       capture_output=True, text=True, timeout=330)
    open(f"{outdir}/bench_phone.json", "w").write(r.stdout)
    r = ssh(f"docker run --rm --entrypoint python3 -v {PI_REPO}:/opt/agv -w /opt/agv agv-sim:latest tools/bench_host.py", timeout=330)
    open(f"{outdir}/bench_pi.json", "w").write(r.stdout)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hub", required=True, help="主平台地址 http://<IP>:<端口>")
    ap.add_argument("--board-ssh", required=True, help="板卡 ssh 目标 user@host")
    ap.add_argument("--board-repo", default="~/agv_sim", help="板卡上的项目目录")
    ap.add_argument("--board-node", required=True, help="板卡在平台上的节点名或 id")
    ap.add_argument("--phone-node", required=True, help="手机在平台上的节点名或 id")
    ap.add_argument("--out", default=os.path.expanduser("~/perf_matrix"))
    ap.add_argument("--combos", default="A,B,C,D")
    ap.add_argument("--planners", default="nav2,dijkstra")
    ap.add_argument("--scene", default="grid_9_square")
    ap.add_argument("--model", default="m-model")
    ap.add_argument("--ver", default="v1")
    ap.add_argument("--settle", type=float, default=30)
    ap.add_argument("--no-bench", action="store_true")
    a = ap.parse_args()
    configure(a)
    os.makedirs(a.out, exist_ok=True)
    restore = [i["id"] for i in active_instances()]
    log("测试前运行中的实例:", restore)
    summary = {"started": time.strftime("%F %T"), "combos": {}}
    try:
        stop_all()
        if not a.no_bench:
            bench(a.out)
        for c in a.combos.split(","):
            sim, nav = COMBOS[c]
            d = f"{a.out}/{c}"
            os.makedirs(d, exist_ok=True)
            log(f"== 组合 {c}: 仿真 {sim} + 执行 {nav}")
            stop_all()
            t_dep = time.time()
            inst = deploy(sim, nav, a.scene, a.model, a.ver)
            res = {"sim": sim, "nav": nav, "instance": inst["id"], "status": inst["status"], "error": inst.get("error"),
                   "deploy_s": round(time.time() - t_dep, 1),
                   "steps_s": {s["key"]: round((s.get("t1") or 0) - (s.get("t0") or 0), 1) for s in inst.get("steps", [])}}
            summary["combos"][c] = res
            if inst["status"] != "running":
                log("  部署失败", inst.get("error"))
                continue
            p = inst.get("ports") or {}
            urls = {"sim_api": f"http://{HOSTS[sim]}:{p['sim_api']}", "web": f"http://{HOSTS[sim]}:{p['web']}",
                    "nav_api": f"http://{HOSTS[nav]}:{p['nav_api']}"}
            res["urls"] = urls
            res["nav2_active_wait_s"] = wait_nav2(urls["nav_api"])
            time.sleep(a.settle)
            for pl in a.planners.split(","):
                tag = f"{c}_{pl}"
                log(f"  {tag}: 采样 + 精度测试")
                t_run = time.time()
                lp, sl, rp = start_samplers(tag, urls["sim_api"], urls["nav_api"], 900, d)
                subprocess.run([sys.executable, f"{PHONE_REPO}/tools/precision_test.py", "--gw", urls["web"], "--goals", GOALS,
                                "--planner", pl, "--out", f"{d}/prec_{tag}.json"], timeout=1200,
                               stdout=open(f"{d}/prec_{tag}.log", "w"), stderr=subprocess.STDOUT)
                stop_samplers(tag, lp, sl, rp, d)
                res[pl] = {"run_s": round(time.time() - t_run, 1), "nav2_log": nav2_warnings(nav, inst["id"], t_run)}
                try:
                    res[pl]["precision"] = json.load(open(f"{d}/prec_{tag}.json"))["summary"]
                except Exception as e:
                    res[pl]["precision"] = {"error": str(e)}
                json.dump(summary, open(f"{a.out}/summary.json", "w"), ensure_ascii=False, indent=1)
            log(f"  组合 {c} 完成")
    finally:
        stop_all()
        for iid in restore:
            log("恢复实例", iid)
            try:
                api("POST", f"/api/hub/instances/{iid}/restart", {}, timeout=300)
            except Exception as e:
                log("  恢复失败", e)
        summary["ended"] = time.strftime("%F %T")
        json.dump(summary, open(f"{a.out}/summary.json", "w"), ensure_ascii=False, indent=1)
        log("全部完成", a.out)


if __name__ == "__main__":
    main()
