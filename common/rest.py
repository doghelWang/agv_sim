#!/usr/bin/env python3
"""
轻量 RESTful 通信层 (仅依赖 Python 标准库) —— 三进程之间唯一的通信方式

服务端 RestServer
  * 路由: 方法 + 路径模板 (/api/v1/sensors/lidars/{name})，处理函数返回 dict/list → JSON
  * 二进制: 处理函数返回 BinaryBody(bytes, headers) → application/octet-stream (点云等大数据)
  * HTTP/1.1 keep-alive (客户端复用连接，10~50 Hz 轮询开销低)
  * 统一错误: {"error": {"code": ..., "message": ...}}；CORS 允许浏览器直接访问
  * GET /api/v1 自动列出全部路由 (自描述)

客户端 RestClient
  * 每线程一条持久连接；超时/断线自动重连；json()/binary() 两种取数
"""

import json
import re
import socket
import threading
import time
import traceback
import urllib.parse
from http.client import HTTPConnection
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable, Dict, List, Optional, Tuple


class ApiError(Exception):
    def __init__(self, status: int, message: str, code: str = "error"):
        super().__init__(message)
        self.status, self.message, self.code = status, message, code


class BinaryBody:
    def __init__(self, data: bytes, headers: Optional[Dict[str, str]] = None, content_type: str = "application/octet-stream"):
        self.data, self.headers, self.content_type = data, headers or {}, content_type


class RawBody:
    """原样返回 (HTML/JS/CSS 等静态内容)"""

    def __init__(self, data: bytes, content_type: str, headers: Optional[Dict[str, str]] = None, status: int = 200):
        self.data, self.content_type, self.headers, self.status = data, content_type, headers or {}, status


class FileBody:
    """流式发送本地文件 (大文件: 镜像包/场景包)，不整体读入内存"""

    def __init__(self, path: str, content_type: str = "application/octet-stream", filename: Optional[str] = None,
                 headers: Optional[Dict[str, str]] = None):
        self.path, self.content_type, self.filename, self.headers = path, content_type, filename, headers or {}


class Request:
    def __init__(self, method, path, params, query, body, headers, client=None):
        self.method, self.path, self.params, self.query, self.headers = method, path, params, query, headers
        self.client = client          # (ip, port) 请求来源
        self._body = body

    @property
    def json(self) -> Any:
        if not self._body:
            return {}
        try:
            return json.loads(self._body.decode("utf-8"))
        except Exception:
            raise ApiError(400, "请求体不是合法 JSON", "bad_json")

    @property
    def raw(self) -> bytes:
        return self._body or b""

    def q(self, key: str, default=None, cast=str):
        v = self.query.get(key)
        if v is None:
            return default
        try:
            return cast(v[0]) if cast is not bool else v[0].lower() in ("1", "true", "yes")
        except Exception:
            raise ApiError(400, f"查询参数 {key} 无效", "bad_param")


class RestServer:
    def __init__(self, name: str, host: str = "0.0.0.0", port: int = 8090):
        self.name, self.host, self.port = name, host, port
        self.routes: List[Tuple[str, re.Pattern, str, Callable, str]] = []
        self.fallback: Optional[Callable] = None
        self.mounts: List[Tuple[str, Callable]] = []     # (路径前缀, fn(handler)) 原始处理: 反向代理/流式上传/静态文件
        self.httpd = None
        self.route("GET", "/api/v1", self._index, "列出全部接口")

    def route(self, method: str, template: str, fn: Callable, doc: str = ""):
        rx = "^" + re.sub(r"\{(\w+)\}", r"(?P<\1>[^/]+)", template.rstrip("/")) + "/?$"
        self.routes.append((method.upper(), re.compile(rx), template, fn, doc))

    def mount(self, prefix: str, fn: Callable):
        """前缀挂载: fn(handler) 直接读写 BaseHTTPRequestHandler (不预读请求体)；返回 False 表示不处理"""
        self.mounts.append((prefix, fn))

    def _index(self, req):
        return {"service": self.name, "routes": [{"method": m, "path": t, "doc": d} for m, _, t, _, d in self.routes]}

    def dispatch(self, method: str, raw_path: str, body: bytes, headers, client=None) -> Tuple[int, Any]:
        u = urllib.parse.urlparse(raw_path)
        path = u.path.rstrip("/") or "/"
        query = urllib.parse.parse_qs(u.query)
        allowed = []
        for m, rx, _, fn, _ in self.routes:
            mt = rx.match(path)
            if not mt:
                continue
            if m != method:
                allowed.append(m)
                continue
            req = Request(method, path, {k: urllib.parse.unquote(v) for k, v in mt.groupdict().items()}, query, body, headers, client)
            return 200, fn(req)
        if allowed:
            raise ApiError(405, f"{method} 不支持，可用: {','.join(sorted(set(allowed)))}", "method_not_allowed")
        if self.fallback and method == "GET":
            return 200, self.fallback(Request(method, path, {}, query, body, headers, client))
        raise ApiError(404, f"未找到资源 {path}", "not_found")

    def serve_forever(self):
        server = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, *a):  # 静默
                pass

            def setup(self):
                super().setup()
                try:  # 关闭 Nagle，避免 keep-alive 下 40 ms 延迟确认
                    self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                except OSError:
                    pass

            def _send(self, status, payload):
                extra = {}
                if isinstance(payload, FileBody):
                    import os as _os
                    size = _os.path.getsize(payload.path)
                    self.send_response(status)
                    self.send_header("Content-Type", payload.content_type)
                    self.send_header("Content-Length", str(size))
                    self.send_header("Access-Control-Allow-Origin", "*")
                    if payload.filename:
                        fn = urllib.parse.quote(payload.filename)
                        self.send_header("Content-Disposition", f"attachment; filename*=UTF-8''{fn}")
                    for k, v in payload.headers.items():
                        self.send_header(k, v)
                    self.end_headers()
                    if self.command != "HEAD":
                        with open(payload.path, "rb") as f:
                            while True:
                                b = f.read(1 << 20)
                                if not b:
                                    break
                                self.wfile.write(b)
                    return
                if isinstance(payload, RawBody):
                    status = payload.status
                    data, ctype, extra = payload.data, payload.content_type, payload.headers
                elif isinstance(payload, BinaryBody):
                    data, ctype, extra = payload.data, payload.content_type, payload.headers
                else:
                    data = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
                    ctype = "application/json; charset=utf-8"
                self.send_response(status)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Expose-Headers", "*")
                if "Cache-Control" not in extra:
                    self.send_header("Cache-Control", "no-cache, no-store")
                for k, v in extra.items():
                    self.send_header(k, v)
                self.end_headers()
                if self.command != "HEAD":
                    self.wfile.write(data)

            def _handle(self, method):
                for prefix, fn in server.mounts:
                    if self.path.startswith(prefix):
                        try:
                            if fn(self) is not False:
                                return
                        except (BrokenPipeError, ConnectionResetError):
                            return
                        except ApiError as e:
                            return self._send(e.status, {"error": {"code": e.code, "message": e.message}})
                        except Exception as e:  # pragma: no cover
                            traceback.print_exc()
                            try:
                                return self._send(500, {"error": {"code": "internal", "message": str(e)}})
                            except Exception:
                                return
                n = int(self.headers.get("Content-Length") or 0)
                body = self.rfile.read(n) if n > 0 else b""
                try:
                    status, payload = server.dispatch("GET" if method == "HEAD" else method, self.path, body, self.headers,
                                                     self.client_address)
                    if payload is None:
                        payload = {"ok": True}
                except ApiError as e:
                    status, payload = e.status, {"error": {"code": e.code, "message": e.message}}
                except Exception as e:  # pragma: no cover
                    traceback.print_exc()
                    status, payload = 500, {"error": {"code": "internal", "message": str(e)}}
                try:
                    self._send(status, payload)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def send_json(self, status, payload):
                self._send(status, payload)

            def do_GET(self): self._handle("GET")
            def do_HEAD(self): self._handle("HEAD")
            def do_POST(self): self._handle("POST")
            def do_PUT(self): self._handle("PUT")
            def do_PATCH(self): self._handle("PATCH")
            def do_DELETE(self): self._handle("DELETE")

            def do_OPTIONS(self):
                self.send_response(204)
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Methods", "GET,POST,PUT,PATCH,DELETE,OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type, X-Lock-Token, X-Node-Key, X-User")
                self.send_header("Content-Length", "0")
                self.end_headers()

        ThreadingHTTPServer.daemon_threads = True
        ThreadingHTTPServer.allow_reuse_address = True
        self.httpd = ThreadingHTTPServer((self.host, self.port), Handler)
        self.httpd.serve_forever()

    def start(self) -> threading.Thread:
        th = threading.Thread(target=self.serve_forever, daemon=True, name=f"rest-{self.name}")
        th.start()
        return th


class RestClient:
    """线程安全: 每个线程独立的持久 HTTP/1.1 连接"""

    def __init__(self, base_url: str, timeout: float = 2.0):
        u = urllib.parse.urlparse(base_url)
        self.host, self.port = u.hostname, u.port or 80
        self.prefix = u.path.rstrip("/")
        self.timeout = timeout
        self._tls = threading.local()
        self.ok = False
        self.last_error = ""

    def _conn(self, timeout):
        c = getattr(self._tls, "conn", None)
        if c is None:
            c = HTTPConnection(self.host, self.port, timeout=timeout)
            c.connect()
            try:
                c.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError:
                pass
            self._tls.conn = c
        else:
            c.timeout = timeout
            if c.sock is not None:
                c.sock.settimeout(timeout)
        return c

    def _reset(self):
        c = getattr(self._tls, "conn", None)
        if c is not None:
            try:
                c.close()
            except Exception:
                pass
        self._tls.conn = None

    def request(self, method: str, path: str, body: Any = None, timeout: Optional[float] = None,
                accept: str = "application/json") -> Tuple[int, Dict[str, str], bytes]:
        t = timeout or self.timeout
        data = None
        headers = {"Accept": accept, "Connection": "keep-alive"}
        if body is not None:
            data = json.dumps(body, ensure_ascii=False).encode("utf-8")
            headers["Content-Type"] = "application/json"
        for attempt in (0, 1):
            try:
                c = self._conn(t)
                c.request(method, self.prefix + path, body=data, headers=headers)
                r = c.getresponse()
                payload = r.read()
                self.ok = True
                return r.status, {k.lower(): v for k, v in r.getheaders()}, payload
            except (ConnectionError, socket.timeout, OSError, Exception) as e:  # 断线 → 重连一次
                self._reset()
                self.ok = False
                self.last_error = f"{type(e).__name__}: {e}"
                if attempt == 1:
                    raise ConnectionError(f"{method} {path} 失败: {self.last_error}")

    def json(self, method: str, path: str, body: Any = None, timeout: Optional[float] = None) -> Any:
        status, _, payload = self.request(method, path, body, timeout)
        obj = json.loads(payload.decode("utf-8")) if payload else {}
        if status >= 400:
            msg = obj.get("error", {}).get("message", "") if isinstance(obj, dict) else ""
            raise ApiError(status, f"{method} {path}: {msg}", "remote")
        return obj

    def get(self, path, **kw): return self.json("GET", path, **kw)
    def put(self, path, body=None, **kw): return self.json("PUT", path, body, **kw)
    def post(self, path, body=None, **kw): return self.json("POST", path, body, **kw)
    def delete(self, path, **kw): return self.json("DELETE", path, **kw)

    def binary(self, path: str, timeout: Optional[float] = None) -> Tuple[int, Dict[str, str], bytes]:
        return self.request("GET", path, None, timeout, accept="application/octet-stream")

    def safe(self, method: str, path: str, body: Any = None, default=None, timeout: Optional[float] = None):
        try:
            return self.json(method, path, body, timeout)
        except Exception:
            return default


def wait_for(client: RestClient, path: str = "/api/v1/health", timeout: float = 60.0, log=print) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        if client.safe("GET", path) is not None:
            return True
        time.sleep(0.5)
    log(f"[rest] 等待 {client.host}:{client.port}{path} 超时")
    return False
