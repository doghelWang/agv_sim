#!/data/data/com.termux/files/usr/bin/python3
"""
proot 进程派生服务 (运行在 Termux 原生 Python，不在 proot 里)

为什么需要:
  proot 用 ptrace 翻译系统调用，一个 proot 会话 = 一个单线程追踪进程 (tracer)。
  以前节点代理在一个 proot 里拉起全部进程 (仿真 + 网关 + 执行 + slam_toolbox + EKF + Nav2 …)，
  这些进程的每次 stat/sendmsg 都要排队经过同一个 tracer —— 8 核手机实际只用上约 1 核，RTF 掉到 0.07。
  实测 (Pixel 4): 3 个进程共用 1 个 proot，每次 stat 200~256 µs；各用 1 个 proot，105 µs。

做法:
  proot 里的程序通过本服务 (127.0.0.1:8069) 请求 "在一个新的 proot 会话里运行命令"，
  每个重负载进程 (仿真、网关、执行、定位栈、Nav2) 各自拥有独立的 tracer，可并行跑在多个核上。

接口 (JSON):
  POST /spawn  {name, argv, cwd, env, log, cpus?}   → {name, pid}      同名已在运行则先停止
  POST /signal {name, sig}                         → 向该会话内的全部进程发信号 (SIGKILL 时连同 proot 一起结束)
  POST /stop   {name, timeout?}                    → SIGTERM → 等待 → SIGKILL；名字以 "name/" 开头的子会话一并停止
  GET  /status?name=…                              → {running, exit_code}
  GET  /list
所有者清理: 名为 "A/xxx" 的会话属于 "A"；A 退出后其子会话自动停止 (执行进程崩溃不会遗留 Nav2)。
"""
import json
import os
import shlex
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

PORT = int(os.environ.get("SPAWNER_PORT", "8069"))
DISTRO = os.environ.get("SPAWNER_DISTRO", "ubuntu")
HOME = os.path.expanduser("~")
RUN_DIR = os.path.join(HOME, ".agv-spawn")
os.makedirs(RUN_DIR, exist_ok=True)

procs = {}          # name -> {"p": Popen, "argv": [...], "started": t}
lock = threading.Lock()


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def children_map():
    m = {}
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            with open(f"/proc/{d}/stat") as f:
                ppid = int(f.read().rsplit(")", 1)[1].split()[1])
            m.setdefault(ppid, []).append(int(d))
        except Exception:
            pass
    return m


def descendants(pid):
    m, out, st = children_map(), [], [pid]
    while st:
        for c in m.get(st.pop(), []):
            out.append(c)
            st.append(c)
    return out


def _signal_entry(e, sig):
    if e["p"].poll() is not None:
        return False
    pid = e["p"].pid
    targets = descendants(pid)                  # proot 的被追踪进程 (真正的程序)
    if sig == signal.SIGKILL:
        targets.append(pid)                     # proot 带 --kill-on-exit，结束它会带走残余进程
    for t in targets:
        try:
            os.kill(t, sig)
        except Exception:
            pass
    return True


def signal_session(name, sig):
    e = procs.get(name)
    return bool(e) and _signal_entry(e, sig)


def stop_session(name, timeout=8.0):
    """停止会话及其全部子会话 ("name/…")。
    按进程对象 (而不是名字) 操作，并行发信号: SIGINT (ROS 节点的正常退出) → SIGTERM → SIGKILL。
    以前逐个子会话串行等待 (Nav2 十几个会话 × 8 s)，调用方超时后已经按同名起了新进程，
    迟到的 SIGKILL 会按名字打到新进程上 (重启实例时执行进程 exit -9)。"""
    with lock:
        ents = [e for n, e in procs.items() if n == name or n.startswith(name + "/")]
    ents = [e for e in ents if e["p"].poll() is None]
    if not ents:
        return
    for sig, frac in ((signal.SIGINT, 0.5), (signal.SIGTERM, 0.5)):
        for e in ents:
            _signal_entry(e, sig)
        t0 = time.time()
        while time.time() - t0 < timeout * frac and any(e["p"].poll() is None for e in ents):
            time.sleep(0.1)
    for e in ents:
        if _signal_entry(e, signal.SIGKILL):
            try:
                e["p"].wait(timeout=3)
            except Exception:
                pass


def parse_cpus(spec):
    """"4-7" / "4,5,6,7" / "0-1,4" → {4,5,6,7}；空或无效 → None"""
    out = set()
    for part in str(spec or "").replace(" ", "").split(","):
        if not part:
            continue
        try:
            a, _, b = part.partition("-")
            out.update(range(int(a), int(b or a) + 1))
        except ValueError:
            return None
    try:
        out &= os.sched_getaffinity(0)
    except (AttributeError, OSError):
        pass
    return out or None


def spawn(req):
    name = req["name"]
    argv = [str(a) for a in req["argv"]]
    stop_session(name, 5)
    safe = name.replace("/", "__")
    script = os.path.join(RUN_DIR, safe + ".sh")
    lines = ["#!/bin/bash"]
    for k, v in (req.get("env") or {}).items():
        if k.replace("_", "").isalnum():
            lines.append(f"export {k}={shlex.quote(str(v))}")
    lines.append(f"export AGV_SPAWN_NAME={shlex.quote(name)}")
    if req.get("cwd"):
        lines.append(f"cd {shlex.quote(req['cwd'])} || exit 1")
    if req.get("log"):
        lines.append(f"exec >> {shlex.quote(req['log'])} 2>&1")
    cmd = " ".join(shlex.quote(a) for a in argv)
    lines.append(f"exec {cmd}")
    with open(script, "w") as f:
        f.write("\n".join(lines) + "\n")
    # CPU 绑定在 proot 外面做 (sched_setaffinity 继承给 proot-distro → proot 追踪进程 → 被追踪的全部进程)，
    # 这样追踪进程也在同一组核上 (在 proot 里 taskset 只绑得住被追踪进程)
    cpus = parse_cpus(req.get("cpus"))
    pre = None
    if cpus:
        def pre():
            try:
                os.sched_setaffinity(0, cpus)
            except OSError:
                pass        # 例如 Termux 在后台 cgroup 里没有大核: 不绑定
    p = subprocess.Popen(["proot-distro", "login", DISTRO, "--", "bash", script],
                         stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         start_new_session=True, preexec_fn=pre)
    with lock:
        procs[name] = {"p": p, "argv": argv, "started": time.time()}
    aff = ""
    if cpus:
        try:
            aff = " cpus=" + ",".join(map(str, sorted(os.sched_getaffinity(p.pid))))
        except OSError:
            aff = " cpus=?"
    log("spawn", name, p.pid, cmd[:160] + aff)
    return {"name": name, "pid": p.pid}


def reaper():
    while True:
        time.sleep(1.0)
        with lock:
            items = list(procs.items())
        dead = {n for n, e in items if e["p"].poll() is not None}
        for n, e in items:
            if "/" in n and n not in dead:
                owner = n.split("/", 1)[0]
                if owner in dead or owner not in procs:
                    log("owner gone, stopping", n)
                    threading.Thread(target=stop_session, args=(n, 5), daemon=True).start()
        now = time.time()
        with lock:              # 已退出超过 10 分钟的记录清掉
            for n, e in list(procs.items()):
                if n in dead and now - e.get("dead_at", now) > 600:
                    procs.pop(n, None)
                elif n in dead:
                    e.setdefault("dead_at", now)


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, obj):
        b = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        u = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(u.query).items()}
        if u.path == "/status":
            e = procs.get(q.get("name", ""))
            if not e:
                return self._send(404, {"error": "unknown"})
            rc = e["p"].poll()
            return self._send(200, {"running": rc is None, "exit_code": rc, "pid": e["p"].pid})
        if u.path == "/list":
            return self._send(200, {"sessions": [{"name": n, "pid": e["p"].pid, "running": e["p"].poll() is None,
                                                  "tracees": len(descendants(e["p"].pid)) if e["p"].poll() is None else 0,
                                                  "argv": e["argv"][:6]} for n, e in list(procs.items())]})
        if u.path == "/health":
            return self._send(200, {"ok": True, "sessions": len(procs)})
        self._send(404, {"error": "not found"})

    def do_POST(self):
        try:
            n = int(self.headers.get("Content-Length") or 0)
            req = json.loads(self.rfile.read(n) or b"{}")
            if self.path == "/spawn":
                return self._send(200, spawn(req))
            if self.path == "/signal":
                return self._send(200, {"ok": signal_session(req["name"], int(req.get("sig", signal.SIGTERM)))})
            if self.path == "/stop":
                stop_session(req["name"], float(req.get("timeout", 8)))
                return self._send(200, {"ok": True})
            self._send(404, {"error": "not found"})
        except Exception as ex:  # noqa
            self._send(500, {"error": str(ex)})


def main():
    def bye(*_):
        for n in list(procs):
            if "/" not in n:
                stop_session(n, 3)
        sys.exit(0)
    signal.signal(signal.SIGTERM, bye)
    threading.Thread(target=reaper, daemon=True).start()
    log(f"proot spawner :{PORT}  distro={DISTRO}")
    ThreadingHTTPServer(("127.0.0.1", PORT), H).serve_forever()


if __name__ == "__main__":
    main()
