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


def sweep_orphans():
    """清理"没人追踪"的残留进程: exe 是 proot 的 loader、TracerPid 为 0。追踪进程被 SIGKILL (或被系统杀掉) 时，
    里面的程序不会跟着退出，之后每个被拦截的系统调用都返回 ENOSYS —— 不干活、不退出、还占着端口。返回清掉的个数"""
    n = 0
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            if "/proot/loader" not in os.readlink(f"/proc/{d}/exe"):
                continue
            with open(f"/proc/{d}/status") as f:
                tp = next((l.split()[1] for l in f if l.startswith("TracerPid")), "1")
            if tp == "0":
                os.kill(int(d), signal.SIGKILL)
                n += 1
        except Exception:
            pass
    return n


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
    n = sweep_orphans()
    if n:
        log(f"清理残留进程 {n} 个 (停止 {name})")


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


# ---------------------------------------------------------------- 精简 proot 参数
# proot-distro login 固定带 --kernel-release (伪造内核版本号) 和 --change-id=0:0 (伪装成 root)。它们各自加载一个 proot 扩展，
# 把一批本来不用管的系统调用也拦下来 (每次拦截 = 两次进程切换，手机小核上 0.2~1 ms):
#   --kernel-release → kompat 扩展: futex、epoll_pwait、pselect6、fcntl、eventfd2、pipe2 …  (事件循环、线程唤醒每次都被拦)
#   --change-id      → fake_id0 扩展: getuid 一族、fstat/stat 的返回、chown/chmod …
# 运行期的仿真/执行/ROS 进程用不到这两个伪装 (内核本身够新；不装软件，不需要 root 身份)，去掉后实测 (Flip 5，小核):
#   epoll_wait 192 → 7 µs，线程唤醒往返 1266 → 300 µs，getuid 231 → 1 µs，stat 1112 → 640 µs。
# AGV_PROOT_DROP 是要去掉的参数前缀 (逗号分隔)，设为空串即恢复 proot-distro 原样。装软件、交互登录仍用 proot-distro login。
_DROP = [x for x in os.environ.get("AGV_PROOT_DROP", "--kernel-release,--change-id").split(",") if x]
_tmpl = {"argv": None, "t": 0.0}
_MARK = "__AGV_SCRIPT__"


def lean_cmd(script):
    """返回去掉 _DROP 参数的 proot 命令 (list)；拿不到 proot-distro 的命令模板时返回 None (调用方退回 proot-distro login)"""
    if not _DROP:
        return None
    if _tmpl["argv"] is None and time.time() - _tmpl["t"] > 60:
        _tmpl["t"] = time.time()
        try:
            out = subprocess.run(["proot-distro", "login", DISTRO, "--get-proot-cmd", "--", "bash", _MARK],
                                 capture_output=True, text=True, timeout=30, stdin=subprocess.DEVNULL).stdout
            i = out.find("env \\\n")            # 前面有一行说明文字
            argv = shlex.split(out[i:].replace("\\\n", " ").replace("\\\r\n", " ")) if i >= 0 else []
            if "--rootfs" in " ".join(argv) and any(_MARK in a for a in argv) and any(a.endswith("/proot") for a in argv):
                _tmpl["argv"] = argv
            else:
                log(f"[警告] 无法解析 proot-distro --get-proot-cmd 的输出，沿用 proot-distro login")
        except Exception as e:  # noqa
            log(f"[警告] proot-distro --get-proot-cmd 失败 ({e})，沿用 proot-distro login")
    if _tmpl["argv"] is None:
        return None
    out = []
    for a in _tmpl["argv"]:
        if any(a == d or a.startswith(d + "=") for d in _DROP):
            continue
        out.append(a.replace(_MARK, shlex.quote(script)) if _MARK in a else a)
    return out


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
    p = subprocess.Popen(lean_cmd(script) or ["proot-distro", "login", DISTRO, "--", "bash", script],
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
