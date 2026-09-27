#!/usr/bin/env python3
"""
agv-agent —— 计算节点代理 (每台树莓派/工控机/虚拟机一份，默认 REST :8070)

  * 向平台 (agv-hub) 注册并每 5 s 上报心跳: CPU/内存/温度/磁盘、本机仿真镜像、托管容器、占用端口
  * 接受平台调度 (请求头 X-Node-Key 鉴权): 分配端口、导入/导出镜像、启停实例容器、读取日志、探测连通
  * 运行时: docker (正式) / process (无 Docker 的开发测试环境，直接跑源码)

环境变量
  HUB_API          平台地址 http://<hub>:8080 (必填)
  JOIN_TOKEN       接入令牌 (首次注册用；注册成功后改用 node_key，保存在 AGENT_DATA/agent.json)
  AGENT_PORT       8070          AGENT_NAME  主机名         AGENT_HOST  对外地址 (缺省由平台按来源 IP 判定)
  AGENT_DATA       ~/.agv-agent  AGENT_RUNTIME auto|docker|process
  AGENT_PORT_RANGE 8100-8199     AGENT_KIND  hybrid|controller|sim (节点类型，缺省 hybrid=运行+仿真)
"""

import json
import os
import platform as _pf
import secrets
import socket
import sys
import threading
import time
import urllib.parse
import uuid
from http.client import HTTPConnection, HTTPSConnection
from typing import Dict, Optional

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from agent.runtime import free_disk, make_runtime  # noqa: E402
from common.rest import ApiError, RestServer  # noqa: E402

AGENT_VERSION = "1.0"


def log(m):
    print(f"[agent] {m}", flush=True)


# ====================================================================== 系统信息
class SysInfo:
    def __init__(self):
        self._prev = None
        try:
            import psutil  # noqa
            self.psutil = psutil
        except Exception:
            self.psutil = None

    def cpu_percent(self) -> float:
        if self.psutil:
            v = float(self.psutil.cpu_percent(interval=None))
            if v <= 0.0:                    # Android proot: /proc/stat 不可用 → 进程 CPU 时间估算
                if not hasattr(self, "_est"):
                    from common.hostinfo import CpuEstimator
                    self._est = CpuEstimator()
                v = self._est.percent() or v
            return v
        try:
            with open("/proc/stat") as f:
                v = [int(x) for x in f.readline().split()[1:]]
            idle, total = v[3] + v[4], sum(v)
            if self._prev:
                di, dt = idle - self._prev[0], total - self._prev[1]
                self._prev = (idle, total)
                return round(100.0 * (1 - di / dt), 1) if dt else 0.0
            self._prev = (idle, total)
        except Exception:
            pass
        return 0.0

    def mem(self) -> dict:
        if self.psutil:
            m = self.psutil.virtual_memory()
            return {"total": m.total, "used": m.total - m.available, "percent": m.percent}
        try:
            d = {}
            with open("/proc/meminfo") as f:
                for line in f:
                    k, v = line.split(":")
                    d[k] = int(v.split()[0]) * 1024
            t, a = d["MemTotal"], d.get("MemAvailable", d["MemFree"])
            return {"total": t, "used": t - a, "percent": round(100.0 * (t - a) / t, 1)}
        except Exception:
            return {}

    @staticmethod
    def temp() -> Optional[float]:
        try:
            from common.hostinfo import temp_c
            return temp_c()
        except Exception:
            pass
        for p in ("/sys/class/thermal/thermal_zone0/temp",):
            try:
                with open(p) as f:
                    return round(int(f.read().strip()) / 1000.0, 1)
            except Exception:
                pass
        return None

    @staticmethod
    def model() -> str:
        try:
            from common.hostinfo import model as _m
            return _m()
        except Exception:
            pass
        for p in ("/proc/device-tree/model",):
            try:
                with open(p) as f:
                    return f.read().strip("\x00\n ")
            except Exception:
                pass
        try:
            with open("/proc/cpuinfo") as f:
                for line in f:
                    if line.startswith("model name"):
                        return line.split(":", 1)[1].strip()
        except Exception:
            pass
        return _pf.processor() or _pf.machine()

    @staticmethod
    def os_name() -> str:
        try:
            with open("/etc/os-release") as f:
                for line in f:
                    if line.startswith("PRETTY_NAME="):
                        return line.split("=", 1)[1].strip().strip('"')
        except Exception:
            pass
        return _pf.platform()

    def ips(self):
        out = []
        if self.psutil:
            try:
                for ifn, addrs in self.psutil.net_if_addrs().items():
                    if ifn == "lo" or ifn.startswith(("docker", "veth", "br-", "virbr", "cni", "flannel")):
                        continue
                    for a in addrs:
                        if a.family == socket.AF_INET and not a.address.startswith("127."):
                            out.append(a.address)
            except Exception:
                pass
        if not out:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect(("10.255.255.255", 1))
                out.append(s.getsockname()[0])
                s.close()
            except Exception:
                pass
        return out

    def listening_ports(self, lo: int, hi: int):
        out = set()
        if self.psutil:
            try:
                for c in self.psutil.net_connections(kind="inet"):
                    if c.status == "LISTEN" and c.laddr and lo <= c.laddr.port <= hi:
                        out.add(c.laddr.port)
                return sorted(out)
            except Exception:
                pass
        for p in range(lo, hi + 1):
            if not port_free(p):
                out.add(p)
        return sorted(out)


def port_free(port: int) -> bool:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("0.0.0.0", port))
        return True
    except OSError:
        return False
    finally:
        s.close()


def _http(url: str, timeout: float = 10.0):
    u = urllib.parse.urlparse(url)
    C = HTTPSConnection if u.scheme == "https" else HTTPConnection
    c = C(u.hostname, u.port or (443 if u.scheme == "https" else 80), timeout=timeout)
    path = u.path + ("?" + u.query if u.query else "")
    return c, path


# ====================================================================== 代理
class Agent:
    def __init__(self):
        self.hub = os.environ.get("HUB_API", "").rstrip("/")
        self.port = int(os.environ.get("AGENT_PORT", "8070"))
        self.name = os.environ.get("AGENT_NAME") or socket.gethostname()
        self.data = os.path.expanduser(os.environ.get("AGENT_DATA", "~/.agv-agent"))
        os.makedirs(self.data, exist_ok=True)
        lo, hi = os.environ.get("AGENT_PORT_RANGE", "8100-8199").split("-")
        self.port_lo, self.port_hi = int(lo), int(hi)
        self.kind = os.environ.get("AGENT_KIND", "hybrid")
        self.advertise = os.environ.get("AGENT_HOST", "")
        self.rt = make_runtime(os.environ.get("AGENT_RUNTIME", "auto"), ROOT, self.data)
        self.sys = SysInfo()
        self.state_path = os.path.join(self.data, "agent.json")
        self.state = self._load_state()
        self.lock = threading.Lock()
        self.jobs: Dict[str, dict] = {}
        self.hub_online = False
        self.last_error = ""

    # ------------------------------------------------------------------ 状态文件
    def _load_state(self) -> dict:
        try:
            with open(self.state_path) as f:
                return json.load(f)
        except Exception:
            return {"reservations": {}}

    def _save_state(self):
        tmp = self.state_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.state, f, indent=2)
        os.replace(tmp, self.state_path)

    # ------------------------------------------------------------------ 鉴权
    def check(self, req):
        key = req.headers.get("X-Node-Key", "")
        if not self.state.get("node_key") or not secrets.compare_digest(key, self.state["node_key"]):
            raise ApiError(401, "节点密钥无效", "unauthorized")

    # ------------------------------------------------------------------ 信息
    def info(self, full: bool = True) -> dict:
        try:
            images = self.rt.images()
        except Exception as e:
            images, self.last_error = [], f"镜像列表: {e}"
        try:
            conts = self.rt.list()
        except Exception as e:
            conts = []
            self.last_error = f"容器列表: {e}"
        cpu_n = os.cpu_count() or 1
        d = {"name": self.name, "hostname": socket.gethostname(), "arch": _norm_arch(_pf.machine()), "cpu_count": cpu_n,
             "cpu_percent": self.sys.cpu_percent(), "mem": self.sys.mem(), "temp_c": self.sys.temp(),
             "load": list(os.getloadavg()) if hasattr(os, "getloadavg") else [], "disk": free_disk(self.data),
             "model": self.sys.model(), "os": self.sys.os_name(), "ips": self.sys.ips(), "kind": self.kind, "runtime": self.rt.describe(),
             "agent_version": AGENT_VERSION, "api_port": self.port, "advertise_host": self.advertise,
             "images": images, "containers": conts, "port_range": [self.port_lo, self.port_hi],
             "reservations": self.state.get("reservations", {}), "last_error": self.last_error, "time": time.time()}
        if full:
            d["ports_in_use"] = self.sys.listening_ports(self.port_lo, self.port_hi)
        return d

    # ------------------------------------------------------------------ 注册与心跳
    def _hub(self, method: str, path: str, body=None, timeout: float = 8.0):
        c, p = _http(self.hub + path, timeout)
        h = {"Content-Type": "application/json"}
        if self.state.get("node_key"):
            h["X-Node-Key"] = self.state["node_key"]
        c.request(method, p, body=json.dumps(body or {}).encode(), headers=h)
        r = c.getresponse()
        raw = r.read()
        c.close()
        obj = json.loads(raw.decode() or "{}")
        if r.status >= 400:
            raise ApiError(r.status, (obj.get("error") or {}).get("message", str(r.status)))
        return obj

    def register(self) -> bool:
        body = {"name": self.name, "api_port": self.port, "advertise_host": self.advertise, "kind": self.kind,
                "node_id": self.state.get("node_id"), "node_key": self.state.get("node_key"),
                "token": os.environ.get("JOIN_TOKEN", ""), "hub_url": self.hub, "info": self.info(full=False)}
        try:
            r = self._hub("POST", "/api/hub/nodes/register", body)
        except Exception as e:
            self.last_error = f"注册失败: {e}"
            log(self.last_error)
            return False
        self.state.update({"node_id": r["node_id"], "node_key": r["node_key"], "hub": self.hub})
        self._save_state()
        log(f"已注册到平台 {self.hub}: 节点 {r['node_id']} ({r.get('host')})")
        self.last_error = ""
        return True

    def heartbeat_loop(self):
        registered = bool(self.state.get("node_id")) and self.register()
        while True:
            if not registered:
                registered = self.register()
                if not registered:
                    self.hub_online = False
                    time.sleep(5)
                    continue
            try:
                self._hub("POST", f"/api/hub/nodes/{self.state['node_id']}/heartbeat", self.info())
                self.hub_online = True
            except ApiError as e:
                self.hub_online = e.status < 500
                if e.status in (401, 404):
                    log(f"平台拒绝心跳 ({e.message})，重新注册")
                    registered = False
            except Exception as e:
                self.hub_online = False
                self.last_error = f"心跳失败: {e}"
            time.sleep(5)

    # ------------------------------------------------------------------ 端口
    def allocate(self, instance: str, names) -> dict:
        with self.lock:
            res = self.state.setdefault("reservations", {})
            cur = res.get(instance, {})
            if cur and all(n in cur for n in names):
                return cur              # 同一实例重复申请 (重启/重新部署) → 沿用原端口
            taken = {p for iid, ps in res.items() if iid != instance for p in ps.values()}
            out = {}
            for n in names:
                p = self.port_lo
                while p <= self.port_hi and (p in taken or p in out.values() or not port_free(p)):
                    p += 1
                if p > self.port_hi:
                    raise ApiError(409, f"端口范围 {self.port_lo}-{self.port_hi} 已用尽")
                out[n] = p
            res[instance] = out
            self._save_state()
            return out

    def release(self, instance: str):
        with self.lock:
            self.state.setdefault("reservations", {}).pop(instance, None)
            self._save_state()

    # ------------------------------------------------------------------ 任务 (镜像导入/导出，耗时长)
    def _job(self, kind: str, ref: str) -> dict:
        j = {"id": uuid.uuid4().hex[:10], "kind": kind, "ref": ref, "status": "running", "bytes": 0, "total": 0,
             "message": "", "started": time.time(), "ended": None}
        self.jobs[j["id"]] = j
        if len(self.jobs) > 50:
            for k in sorted(self.jobs, key=lambda k: self.jobs[k]["started"])[:10]:
                self.jobs.pop(k, None)
        return j

    def ensure_image(self, ref: str, package_url: Optional[str]) -> dict:
        if self.rt.image(ref):
            j = self._job("ensure", ref)
            j.update(status="done", message="镜像已存在", ended=time.time())
            return j
        if not package_url:
            raise ApiError(404, f"本机没有镜像 {ref}，且未提供程序包下载地址")
        j = self._job("ensure", ref)

        def work():
            try:
                c, p = _http(package_url, 3600)
                c.request("GET", p, headers={"X-Node-Key": self.state.get("node_key", "")})
                r = c.getresponse()
                if r.status >= 400:
                    raise RuntimeError(f"下载程序包失败 HTTP {r.status}: {r.read()[:200]!r}")
                j["total"] = int(r.getheader("Content-Length") or 0)

                def chunks():
                    while True:
                        b = r.read(1 << 20)
                        if not b:
                            break
                        j["bytes"] += len(b)
                        yield b
                j["message"] = self.rt.load(chunks())
                c.close()
                if not self.rt.image(ref):
                    raise RuntimeError(f"导入完成但未找到 {ref} (程序包内镜像名不一致?)\n{j['message']}")
                j["status"] = "done"
            except Exception as e:
                j["status"], j["message"] = "error", str(e)
            j["ended"] = time.time()
        threading.Thread(target=work, daemon=True).start()
        return j

    def export_image(self, ref: str, upload_url: str) -> dict:
        im = self.rt.image(ref)
        if not im:
            raise ApiError(404, f"本机没有镜像 {ref}")
        j = self._job("export", ref)
        j["total"] = int(im.get("size") or 0)

        def work():
            try:
                c, p = _http(upload_url, 3600)

                def chunks():
                    for b in self.rt.save(ref):
                        j["bytes"] += len(b)
                        yield b
                c.request("POST", p, body=chunks(), encode_chunked=True,
                          headers={"X-Node-Key": self.state.get("node_key", ""), "Content-Type": "application/x-tar"})
                r = c.getresponse()
                raw = r.read()
                c.close()
                if r.status >= 400:
                    raise RuntimeError(f"上传失败 HTTP {r.status}: {raw[:300]!r}")
                j["result"] = json.loads(raw.decode() or "{}")
                j["status"] = "done"
            except Exception as e:
                j["status"], j["message"] = "error", str(e)
            j["ended"] = time.time()
        threading.Thread(target=work, daemon=True).start()
        return j

    # ------------------------------------------------------------------ 容器
    def run(self, spec: dict) -> dict:
        if not self.rt.image(spec["image"]):
            raise ApiError(404, f"本机没有镜像 {spec['image']}")
        iid = spec.get("instance", "x")
        spec = dict(spec)
        if spec.get("data") is not False:
            spec["data_dir"] = os.path.join(self.data, "instances", iid, spec.get("role", "sim"))
        return self.rt.run(spec)


def _norm_arch(m: str) -> str:
    return {"aarch64": "arm64", "x86_64": "amd64", "armv7l": "arm"}.get(m, m)


# ====================================================================== REST
def build_api(ag: Agent) -> RestServer:
    api = RestServer("agent", port=ag.port)
    R = api.route
    R("GET", "/api/v1/health", lambda q: {"service": "agent", "ok": True, "name": ag.name, "node_id": ag.state.get("node_id"),
                                          "hub": ag.hub, "hub_online": ag.hub_online, "runtime": ag.rt.name}, "健康检查")

    def auth(fn):
        def w(q):
            ag.check(q)
            return fn(q)
        return w

    R("GET", "/api/v1/info", auth(lambda q: ag.info()), "节点信息")
    R("POST", "/api/v1/ports/allocate", auth(lambda q: {"ports": ag.allocate(q.json["instance"], q.json.get("names") or ["port"])}),
      "分配端口 {instance, names[]}")
    R("POST", "/api/v1/ports/release", auth(lambda q: ag.release(q.json["instance"]) or {"ok": True}), "释放端口")
    R("POST", "/api/v1/images/ensure", auth(lambda q: ag.ensure_image(q.json["ref"], q.json.get("package_url"))),
      "确保镜像存在 (缺失时从平台下载并导入)")
    R("POST", "/api/v1/images/export", auth(lambda q: ag.export_image(q.json["ref"], q.json["upload_url"])),
      "导出本机镜像并上传到平台")
    R("GET", "/api/v1/jobs/{jid}", auth(lambda q: ag.jobs.get(q.params["jid"]) or (_ for _ in ()).throw(ApiError(404, "无此任务"))),
      "任务进度")
    R("GET", "/api/v1/containers", auth(lambda q: {"containers": ag.rt.list()}), "托管容器")
    R("POST", "/api/v1/containers/run", auth(lambda q: ag.run(q.json)), "启动容器 {name,image,role,instance,env}")

    def cstatus(q):
        s = ag.rt.status(q.params["name"])
        if not s:
            raise ApiError(404, "容器不存在")
        return s
    R("GET", "/api/v1/containers/{name}", auth(cstatus), "容器状态")
    R("POST", "/api/v1/containers/{name}/stop", auth(lambda q: ag.rt.stop(q.params["name"]) or {"ok": True}), "停止")
    R("DELETE", "/api/v1/containers/{name}", auth(lambda q: ag.rt.remove(q.params["name"]) or {"ok": True}), "删除")
    R("GET", "/api/v1/containers/{name}/logs", auth(lambda q: {"name": q.params["name"],
                                                               "logs": ag.rt.logs(q.params["name"], q.q("tail", 300, int))}), "日志")

    def probe(q):
        url = q.json["url"]
        t0 = time.time()
        try:
            c, p = _http(url, float(q.json.get("timeout", 3)))
            c.request("GET", p)
            r = c.getresponse()
            r.read()
            c.close()
            return {"ok": r.status < 500, "status": r.status, "ms": round((time.time() - t0) * 1000, 1)}
        except Exception as e:
            return {"ok": False, "error": str(e), "ms": round((time.time() - t0) * 1000, 1)}
    R("POST", "/api/v1/probe", auth(probe), "探测 URL 连通性 {url}")
    R("DELETE", "/api/v1/instances/{iid}", auth(lambda q: ag.release(q.params["iid"]) or {"ok": True}), "释放实例资源")
    return api


def main():
    ag = Agent()
    if not ag.hub:
        log("未设置 HUB_API，节点代理仅本地运行 (不会注册到平台)")
    else:
        threading.Thread(target=ag.heartbeat_loop, daemon=True, name="heartbeat").start()
    api = build_api(ag)
    log(f"节点代理 {ag.name} :{ag.port}  运行时 {ag.rt.name}  数据 {ag.data}  平台 {ag.hub or '-'}")
    api.serve_forever()


if __name__ == "__main__":
    main()
