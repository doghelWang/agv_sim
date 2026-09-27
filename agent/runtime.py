#!/usr/bin/env python3
"""
节点运行时: 在本机启动/停止仿真实例的两个服务 (agv-sim / agv-nav)

  DockerRuntime  —— 正式部署: 经 docker.sock 管理容器 (host 网络、/data 挂载、unless-stopped 重启)
  ProcessRuntime —— 无 Docker 的开发/测试环境: 直接以子进程运行源码 (镜像引用为 process:sim / process:nav)

两者对上层 (agent/server.py) 提供相同接口。
"""

import os
import shutil
import signal
import subprocess
import threading
import time
from typing import Dict, Iterable, Iterator, List, Optional

from agent.docker_api import DockerClient, DockerError

MANAGED = "org.agv.managed"
KIND = "org.agv.kind"          # sim | nav
VERSION = "org.agv.version"
API = "org.agv.api"


def _kind_of(ref: str, labels: Optional[dict]) -> Optional[str]:
    labels = labels or {}
    if labels.get(KIND) in ("sim", "nav"):
        return labels[KIND]
    r = ref.split("/")[-1]
    if r.startswith("agv-sim"):
        return "sim"
    if r.startswith("agv-nav"):
        return "nav"
    return None


class DockerRuntime:
    name = "docker"

    def __init__(self, sock: str = "/var/run/docker.sock"):
        self.d = DockerClient(sock)

    def available(self) -> bool:
        return self.d.available()

    def describe(self) -> dict:
        try:
            v = self.d.version()
            return {"runtime": "docker", "version": v.get("Version"), "api": v.get("ApiVersion"), "arch": v.get("Arch"),
                    "os": v.get("Os")}
        except Exception as e:
            return {"runtime": "docker", "error": str(e)}

    def images(self) -> List[dict]:
        out = []
        for im in self.d.images():
            labels = im.get("Labels") or {}
            for ref in im.get("RepoTags") or []:
                if ref == "<none>:<none>":
                    continue
                k = _kind_of(ref, labels)
                if not k:
                    continue
                out.append({"ref": ref, "id": im["Id"], "kind": k, "size": im.get("Size", 0), "created": im.get("Created", 0),
                            "version": labels.get(VERSION) or ref.split(":")[-1], "api": labels.get(API, "v1"), "labels": labels})
        return out

    def image(self, ref: str) -> Optional[dict]:
        im = self.d.image(ref)
        if not im:
            return None
        cfg = im.get("Config") or {}
        return {"ref": ref, "id": im["Id"], "arch": im.get("Architecture"), "size": im.get("Size", 0),
                "created": im.get("Created"), "labels": cfg.get("Labels") or {}}

    def load(self, chunks: Iterable[bytes]) -> str:
        return self.d.load(chunks)

    def save(self, ref: str) -> Iterator[bytes]:
        r, c = self.d.save(ref)
        try:
            while True:
                b = r.read(1 << 20)
                if not b:
                    break
                yield b
        finally:
            c.close()

    def run(self, spec: dict) -> dict:
        name = spec["name"]
        self.d.remove(name)
        binds = []
        if spec.get("data_dir"):
            os.makedirs(spec["data_dir"], exist_ok=True)
            binds.append(f"{spec['data_dir']}:/data")
        binds += spec.get("binds", [])
        labels = {MANAGED: "1", KIND: spec.get("role", ""), "org.agv.instance": spec.get("instance", "")}
        cid = self.d.create(name, spec["image"], spec.get("env", {}), binds, labels, spec.get("cmd"))
        self.d.start(name)
        return {"name": name, "id": cid}

    def stop(self, name: str):
        self.d.stop(name)

    def remove(self, name: str):
        self.d.remove(name)

    def status(self, name: str) -> Optional[dict]:
        c = self.d.container(name)
        if not c:
            return None
        st = c.get("State") or {}
        return {"name": name, "state": st.get("Status"), "running": bool(st.get("Running")), "exit_code": st.get("ExitCode"),
                "started_at": st.get("StartedAt"), "restarts": c.get("RestartCount", 0), "image": (c.get("Config") or {}).get("Image"),
                "health": (st.get("Health") or {}).get("Status")}

    def list(self) -> List[dict]:
        out = []
        for c in self.d.containers(label=MANAGED + "=1"):
            lb = c.get("Labels") or {}
            out.append({"name": (c.get("Names") or ["/?"])[0].lstrip("/"), "state": c.get("State"), "status": c.get("Status"),
                        "image": c.get("Image"), "role": lb.get(KIND), "instance": lb.get("org.agv.instance")})
        return out

    def logs(self, name: str, tail: int = 200) -> str:
        try:
            return self.d.logs(name, tail)
        except DockerError as e:
            return f"[日志不可用] {e}"


class ProcessRuntime:
    """开发/测试: 用源码目录直接起进程 (无需 Docker)。镜像引用固定为 process:sim / process:nav"""
    name = "process"

    def __init__(self, code_dir: str, log_dir: str):
        self.code_dir = code_dir
        self.log_dir = log_dir
        os.makedirs(log_dir, exist_ok=True)
        self.procs: Dict[str, dict] = {}
        self.lock = threading.Lock()

    def available(self) -> bool:
        return True

    def describe(self) -> dict:
        return {"runtime": "process", "code_dir": self.code_dir}

    def images(self) -> List[dict]:
        v = "src-" + time.strftime("%Y%m%d", time.localtime(os.path.getmtime(os.path.join(self.code_dir, "web_gateway.py"))))
        try:   # 手机 update_from_git.sh 写入的 git 版本 (如 "044c4dd android: …") → src-044c4dd
            with open(os.path.join(self.code_dir, ".agv_version")) as f:
                h = f.read().split()[0]
            if h:
                v = "src-" + h
        except (OSError, IndexError):
            pass
        return [{"ref": "process:sim", "id": "process:sim", "kind": "sim", "size": 0, "created": 0, "version": v, "api": "v1", "labels": {}},
                {"ref": "process:nav", "id": "process:nav", "kind": "nav", "size": 0, "created": 0, "version": v, "api": "v1", "labels": {}}]

    def image(self, ref: str) -> Optional[dict]:
        return next(({"ref": i["ref"], "id": i["id"], "arch": os.uname().machine, "size": 0, "labels": {}}
                     for i in self.images() if i["ref"] == ref), None)

    def load(self, chunks: Iterable[bytes]) -> str:
        for _ in chunks:
            pass
        raise RuntimeError("process 运行时不支持导入镜像")

    def save(self, ref: str) -> Iterator[bytes]:
        raise RuntimeError("process 运行时不支持导出镜像")

    def run(self, spec: dict) -> dict:
        name = spec["name"]
        self.remove(name)
        role = spec.get("role") or ("sim" if spec["image"].endswith("sim") else "nav")
        env = dict(os.environ)
        env.update({k: str(v) for k, v in spec.get("env", {}).items()})
        env["AGV_HOME"] = self.code_dir
        env["PYTHONUNBUFFERED"] = "1"
        if spec.get("data_dir"):
            os.makedirs(spec["data_dir"], exist_ok=True)
            env["AGV_DATA"] = spec["data_dir"]
        if role == "nav":
            env.setdefault("NAV_USE_ROS", "0" if not os.path.exists("/opt/ros") else env.get("NAV_USE_ROS", "1"))
        script = os.path.join(self.code_dir, "docker", "sim_entrypoint.sh" if role == "sim" else "nav_entrypoint.sh")
        logp = os.path.join(self.log_dir, f"{name}.log")
        logf = open(logp, "ab", buffering=0)
        from common import spawn
        if spawn.available():
            # Android proot: 每个实例进程放进独立的 proot 会话 (独立 ptrace 追踪进程，可并行用多核)
            env["AGV_SPAWN_LOG"] = logp
            cpus = env.get("AGV_CPUS_SIM" if role == "sim" else "AGV_CPUS_NAV")
            p = spawn.popen(name, ["bash", script], cwd=self.code_dir, env=env, log=logp, cpus=cpus, top_level=True)
        else:
            p = subprocess.Popen(["bash", script], cwd=self.code_dir, env=env, stdout=logf, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        with self.lock:
            self.procs[name] = {"p": p, "log": logf, "role": role, "image": spec["image"], "instance": spec.get("instance", ""),
                                "started": time.strftime("%Y-%m-%dT%H:%M:%S"), "spec": spec}
        return {"name": name, "id": str(p.pid)}

    def stop(self, name: str):
        with self.lock:
            e = self.procs.get(name)
        if not e or e["p"].poll() is not None:
            return
        if hasattr(e["p"], "stop"):          # proot 派生会话
            e["p"].stop(8)
            return
        try:
            os.killpg(e["p"].pid, signal.SIGTERM)
            e["p"].wait(timeout=8)
        except Exception:
            try:
                os.killpg(e["p"].pid, signal.SIGKILL)
            except Exception:
                pass

    def remove(self, name: str):
        self.stop(name)
        with self.lock:
            e = self.procs.pop(name, None)
        if e:
            try:
                e["log"].close()
            except Exception:
                pass

    def status(self, name: str) -> Optional[dict]:
        with self.lock:
            e = self.procs.get(name)
        if not e:
            return None
        rc = e["p"].poll()
        return {"name": name, "state": "running" if rc is None else "exited", "running": rc is None, "exit_code": rc,
                "started_at": e["started"], "restarts": 0, "image": e["image"], "health": None}

    def list(self) -> List[dict]:
        with self.lock:
            items = list(self.procs.items())
        return [{"name": n, "state": "running" if e["p"].poll() is None else "exited", "status": "", "image": e["image"],
                 "role": e["role"], "instance": e["instance"]} for n, e in items]

    def logs(self, name: str, tail: int = 200) -> str:
        path = os.path.join(self.log_dir, f"{name}.log")
        if not os.path.exists(path):
            return ""
        with open(path, "rb") as f:
            f.seek(0, 2)
            size = f.tell()
            f.seek(max(0, size - 256 * 1024))
            lines = f.read().decode("utf-8", "replace").splitlines()
        return "\n".join(lines[-tail:])


def make_runtime(kind: str, code_dir: str, data_dir: str):
    if kind == "process":
        return ProcessRuntime(code_dir, os.path.join(data_dir, "logs"))
    rt = DockerRuntime(os.environ.get("DOCKER_SOCK", "/var/run/docker.sock"))
    if kind == "auto" and not rt.available():
        print("[agent] 未检测到 Docker，改用 process 运行时", flush=True)
        return ProcessRuntime(code_dir, os.path.join(data_dir, "logs"))
    return rt


def free_disk(path: str) -> dict:
    try:
        u = shutil.disk_usage(path)
        return {"total": u.total, "free": u.free}
    except Exception:
        return {}
