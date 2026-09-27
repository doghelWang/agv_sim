"""
子进程派生: 本机直接 Popen，或 (Android proot 下) 交给 Termux 侧的 proot_spawner 在独立 proot 会话里运行

  from common import spawn
  p = spawn.popen("nav2", ["ros2", "launch", ...])      # 与 subprocess.Popen(start_new_session=True) 同用法
  spawn.killpg(p, signal.SIGINT); p.wait(timeout=8)

AGV_SPAWNER=http://127.0.0.1:8069 时走派生服务 (见 deploy/android/proot_spawner.py)：
  会话名 = "<当前会话 AGV_SPAWN_NAME>/<name>"，所属会话退出时派生服务会自动停止它。
其它情况 (树莓派 Docker、x86) 与原来完全一样。

命令行 (在 shell 脚本里把一个命令放到独立 proot 里，前台等待并转发 SIGTERM/SIGINT):
  python3 -m common.spawn exec <name> -- <cmd> [args...]
"""
import json
import os
import signal
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

SPAWNER = os.environ.get("AGV_SPAWNER", "").rstrip("/")


def _call(method, path, body=None, timeout=5.0):
    data = json.dumps(body).encode() if body is not None else None
    r = urllib.request.Request(SPAWNER + path, data=data, method=method,
                               headers={"Content-Type": "application/json"} if data else {})
    with urllib.request.urlopen(r, timeout=timeout) as resp:
        return json.loads(resp.read() or b"{}")


def available() -> bool:
    if not SPAWNER:
        return False
    try:
        return bool(_call("GET", "/health", timeout=1.5).get("ok"))
    except Exception:
        return False


class RemoteProc:
    """Popen 的最小子集: pid / poll / wait / returncode"""

    def __init__(self, name, pid):
        self.name, self.pid, self.returncode = name, pid, None
        self._t = 0.0

    def poll(self):
        if self.returncode is not None:
            return self.returncode
        if time.time() - self._t < 0.5:      # 每次查询都是一次 HTTP，限频
            return None
        self._t = time.time()
        try:
            s = _call("GET", "/status?name=" + urllib.parse.quote(self.name), timeout=3)
        except urllib.error.HTTPError as e:
            if e.code == 404:
                self.returncode = -1
            return self.returncode
        except Exception:
            return None
        if not s.get("running"):
            self.returncode = s.get("exit_code") if s.get("exit_code") is not None else -1
        return self.returncode

    def wait(self, timeout=None):
        t0 = time.time()
        while True:
            self._t = 0.0
            if self.poll() is not None:
                return self.returncode
            if timeout is not None and time.time() - t0 > timeout:
                raise subprocess.TimeoutExpired(self.name, timeout)
            time.sleep(0.2)

    def send_signal(self, sig):
        try:
            _call("POST", "/signal", {"name": self.name, "sig": int(sig)})
        except Exception:
            pass

    def terminate(self):
        self.send_signal(signal.SIGTERM)

    def kill(self):
        self.send_signal(signal.SIGKILL)

    def stop(self, timeout=8.0):
        try:
            _call("POST", "/stop", {"name": self.name, "timeout": timeout}, timeout=timeout + 10)
        except Exception:
            pass


def session_name(name: str) -> str:
    owner = os.environ.get("AGV_SPAWN_NAME", "")
    return f"{owner}/{name}" if owner and not name.startswith(owner + "/") else name


def popen(name, argv, cwd=None, env=None, log=None, stdout=None, cpus=None, top_level=False):
    """argv 列表；env 为完整环境 (默认当前环境)；log 为输出文件路径 (派生模式下默认沿用本进程的 AGV_SPAWN_LOG)"""
    if available():
        full = name if top_level else session_name(name)
        e = dict(os.environ if env is None else env)
        r = _call("POST", "/spawn", {"name": full, "argv": list(argv), "cwd": cwd or os.getcwd(), "env": e,
                                     "log": log or e.get("AGV_SPAWN_LOG") or os.environ.get("AGV_SPAWN_LOG"),
                                     "cpus": cpus}, timeout=20)
        return RemoteProc(full, r.get("pid"))
    kw = {}
    if stdout is not None:
        kw.update(stdout=stdout, stderr=subprocess.STDOUT)
    elif log:
        kw.update(stdout=open(log, "ab", buffering=0), stderr=subprocess.STDOUT)
    return subprocess.Popen(list(argv), cwd=cwd, env=env, start_new_session=True, **kw)


def killpg(p, sig):
    """向进程组 (本地) 或整个 proot 会话 (派生) 发信号"""
    if isinstance(p, RemoteProc):
        p.send_signal(sig)
    else:
        os.killpg(p.pid, sig)


def _cli():
    a = sys.argv[1:]
    if len(a) < 4 or a[0] != "exec" or "--" not in a:
        print(__doc__)
        sys.exit(2)
    name, cmd = a[1], a[a.index("--") + 1:]
    if not available():
        os.execvp(cmd[0], cmd)
    p = popen(name, cmd)
    def fwd(sig, _f):
        p.stop(6)
        sys.exit(128 + sig)
    signal.signal(signal.SIGTERM, fwd)
    signal.signal(signal.SIGINT, fwd)
    while p.poll() is None:
        time.sleep(1.0)
    sys.exit(p.returncode or 0)


if __name__ == "__main__":
    _cli()
