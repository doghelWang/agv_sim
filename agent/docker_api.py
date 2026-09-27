#!/usr/bin/env python3
"""
Docker Engine API 客户端 (经 /var/run/docker.sock，只用标准库，镜像内无需 docker CLI)

用到的接口 (Engine API v1.41+):
  GET  /version  /info  /images/json  /containers/json?all=1
  GET  /images/{ref}/json            镜像详情 (Id/Architecture/Size/Labels)
  GET  /images/{ref}/get             docker save (tar 流)
  POST /images/load                  docker load (接受 tar / tar.gz 流)
  POST /containers/create?name=      创建 (host 网络、挂载、重启策略)
  POST /containers/{id}/start|stop   DELETE /containers/{id}?force=1
  GET  /containers/{id}/json         GET /containers/{id}/logs?stdout=1&stderr=1&tail=N
"""

import json
import socket
import struct
import urllib.parse
from http.client import HTTPConnection
from typing import Any, Dict, Iterable, List, Optional


class DockerError(Exception):
    def __init__(self, status: int, message: str):
        super().__init__(f"docker {status}: {message}")
        self.status, self.message = status, message


class _UnixConn(HTTPConnection):
    def __init__(self, path: str, timeout: float = 30.0):
        super().__init__("localhost", timeout=timeout)
        self.unix_path = path

    def connect(self):
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(self.timeout)
        s.connect(self.unix_path)
        self.sock = s


class DockerClient:
    def __init__(self, sock: str = "/var/run/docker.sock"):
        self.sock = sock

    # ------------------------------------------------------------------ 底层
    def _req(self, method: str, path: str, body: Any = None, timeout: float = 30.0, headers: Optional[dict] = None,
             stream: bool = False, chunked: bool = False):
        c = _UnixConn(self.sock, timeout)
        h = dict(headers or {})
        data = body
        if isinstance(body, (dict, list)):
            data = json.dumps(body).encode()
            h["Content-Type"] = "application/json"
        if chunked:
            c.request(method, path, body=data, headers=h, encode_chunked=True)
        else:
            c.request(method, path, body=data, headers=h)
        r = c.getresponse()
        if stream:
            if r.status >= 400:
                msg = r.read().decode("utf-8", "replace")
                c.close()
                raise DockerError(r.status, _errmsg(msg))
            return r, c
        raw = r.read()
        c.close()
        if r.status >= 400:
            raise DockerError(r.status, _errmsg(raw.decode("utf-8", "replace")))
        if not raw:
            return None
        ct = r.getheader("Content-Type", "")
        if "json" in ct:
            return json.loads(raw.decode("utf-8"))
        return raw

    def available(self) -> bool:
        try:
            self._req("GET", "/_ping", timeout=3)
            return True
        except Exception:
            return False

    # ------------------------------------------------------------------ 信息
    def version(self) -> dict:
        return self._req("GET", "/version", timeout=5) or {}

    def info(self) -> dict:
        return self._req("GET", "/info", timeout=5) or {}

    def images(self) -> List[dict]:
        return self._req("GET", "/images/json", timeout=10) or []

    def image(self, ref: str) -> Optional[dict]:
        try:
            return self._req("GET", f"/images/{urllib.parse.quote(ref, safe='')}/json", timeout=10)
        except DockerError as e:
            if e.status == 404:
                return None
            raise

    def containers(self, all_: bool = True, label: Optional[str] = None) -> List[dict]:
        q = {"all": "1" if all_ else "0"}
        if label:
            q["filters"] = json.dumps({"label": [label]})
        return self._req("GET", "/containers/json?" + urllib.parse.urlencode(q), timeout=10) or []

    def container(self, name: str) -> Optional[dict]:
        try:
            return self._req("GET", f"/containers/{urllib.parse.quote(name)}/json", timeout=10)
        except DockerError as e:
            if e.status == 404:
                return None
            raise

    # ------------------------------------------------------------------ 镜像导入/导出
    def load(self, chunks: Iterable[bytes], timeout: float = 1800.0) -> str:
        """docker load: chunks 为 tar/tar.gz 字节流；返回 load 输出 (含 Loaded image: xxx)"""
        r, c = self._req("POST", "/images/load?quiet=1", body=chunks, timeout=timeout,
                         headers={"Content-Type": "application/x-tar"}, stream=True, chunked=True)
        out = r.read().decode("utf-8", "replace")
        c.close()
        msgs = []
        for line in out.splitlines():
            try:
                o = json.loads(line)
            except ValueError:
                continue
            if o.get("error"):
                raise DockerError(500, o["error"])
            if o.get("stream"):
                msgs.append(o["stream"].strip())
        return "\n".join(msgs)

    def save(self, ref: str, timeout: float = 1800.0):
        """docker save: 返回 (response, conn)，调用方读取 response 流后关闭 conn"""
        return self._req("GET", f"/images/{urllib.parse.quote(ref, safe='')}/get", timeout=timeout, stream=True)

    def tag(self, src: str, repo: str, tag: str):
        self._req("POST", f"/images/{urllib.parse.quote(src, safe='')}/tag?" + urllib.parse.urlencode({"repo": repo, "tag": tag}))

    # ------------------------------------------------------------------ 容器
    def create(self, name: str, image: str, env: Dict[str, str], binds: List[str], labels: Dict[str, str],
               cmd: Optional[List[str]] = None, restart: str = "unless-stopped") -> str:
        body = {"Image": image, "Env": [f"{k}={v}" for k, v in env.items()], "Labels": labels,
                "HostConfig": {"NetworkMode": "host", "Binds": binds, "RestartPolicy": {"Name": restart},
                               "LogConfig": {"Type": "json-file", "Config": {"max-size": "20m", "max-file": "3"}}}}
        if cmd:
            body["Cmd"] = cmd
        r = self._req("POST", "/containers/create?" + urllib.parse.urlencode({"name": name}), body)
        return r["Id"]

    def start(self, name: str):
        self._req("POST", f"/containers/{urllib.parse.quote(name)}/start")

    def stop(self, name: str, t: int = 10):
        try:
            self._req("POST", f"/containers/{urllib.parse.quote(name)}/stop?t={t}", timeout=t + 20)
        except DockerError as e:
            if e.status not in (304, 404):
                raise

    def remove(self, name: str):
        try:
            self._req("DELETE", f"/containers/{urllib.parse.quote(name)}?force=1&v=0", timeout=30)
        except DockerError as e:
            if e.status != 404:
                raise

    def logs(self, name: str, tail: int = 200) -> str:
        raw = self._req("GET", f"/containers/{urllib.parse.quote(name)}/logs?stdout=1&stderr=1&timestamps=0&tail={int(tail)}",
                        timeout=15)
        return _demux(raw or b"")


def _errmsg(s: str) -> str:
    try:
        return json.loads(s).get("message", s)
    except ValueError:
        return s.strip()[:500]


def _demux(raw: bytes) -> str:
    """非 TTY 容器日志为 8 字节帧头的多路复用流"""
    out, i = [], 0
    if len(raw) >= 8 and raw[0] in (0, 1, 2) and raw[1:4] == b"\x00\x00\x00":
        while i + 8 <= len(raw):
            n = struct.unpack(">I", raw[i + 4:i + 8])[0]
            out.append(raw[i + 8:i + 8 + n])
            i += 8 + n
        return b"".join(out).decode("utf-8", "replace")
    return raw.decode("utf-8", "replace")
