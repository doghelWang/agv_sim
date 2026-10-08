#!/usr/bin/env python3
"""
轻量 RESTful 通信层 (仅依赖 Python 标准库) —— 三进程之间唯一的通信方式

服务端 RestServer
  * 路由: 方法 + 路径模板 (/api/v1/sensors/lidars/{name})，处理函数返回 dict/list → JSON
  * 二进制: 处理函数返回 BinaryBody(bytes, headers) → application/octet-stream (点云等大数据)
  * HTTP/1.1 keep-alive (客户端复用连接，10~50 Hz 轮询开销低)
  * 错误: RFC 9457 Problem Details (application/problem+json)，兼容保留旧字段 {"error": {"code", "message"}}
  * 自描述: GET {base} 列出全部路由 (响应头按 RFC 8631 带 Link: rel="service-desc")；
           GET {base}/openapi.json 由路由表生成 OpenAPI 3.1 文档。base 默认 /api/v1 (路径版本号，见 docs/API.md)
  * CORS 允许浏览器直接访问

客户端 RestClient
  * 每线程一条持久连接；超时/断线自动重连；json()/binary() 两种取数
"""

import os
import json
import re
import socket
import threading
import time
import traceback
import urllib.parse
import http.client
from http.client import HTTPConnection
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable, Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------- 快速 HTTP 头解析
# 标准库 http.client.parse_headers 用 email.feedparser 解析头 (每个请求/响应约 0.3~0.5 ms，树莓派上是
# 仿真/执行进程最大的单项开销)。这里逐行解析后直接填进 HTTPMessage，结果对象与原来同类型、同用法。
# 服务端 (BaseHTTPRequestHandler.parse_request) 与客户端 (HTTPResponse.begin) 都经过这个函数。AGV_FAST_HEADERS=0 关闭。
_orig_parse_headers = http.client.parse_headers


def _fast_parse_headers(fp, _class=http.client.HTTPMessage):
    msg = _class()
    last = None
    n = 0
    while True:
        line = fp.readline(http.client._MAXLINE + 1)
        if len(line) > http.client._MAXLINE:
            raise http.client.LineTooLong("header line")
        if line in (b"\r\n", b"\n", b""):
            break
        n += 1
        if n > http.client._MAXHEADERS:
            raise http.client.HTTPException(f"got more than {http.client._MAXHEADERS} headers")
        s = line.decode("iso-8859-1")
        if s[0] in " \t" and last is not None:          # 折叠行 (已废弃，但保持兼容)
            v = msg[last] + " " + s.strip()
            del msg[last]
            msg[last] = v
            continue
        k, sep, v = s.partition(":")
        if not sep:
            continue
        last = k.strip()
        msg[last] = v.strip()
    return msg


if os.environ.get("AGV_FAST_HEADERS", "1") != "0":
    http.client.parse_headers = _fast_parse_headers


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


API_VERSION = "1.0.0"          # 接口契约版本 (语义化版本)；路径里的 v1 是它的主版本号
API_DOC = "https://github.com/doghelWang/agv_sim/blob/main/docs/API.md"
PROBLEM_TYPE = API_DOC + "#error"          # 各错误码的说明锚点: docs/API.md#error-<code>


def problem(status: int, code: str, detail: str, instance: str = "") -> "RawBody":
    """RFC 9457 Problem Details。扩展成员 code 为机器可读错误码；error 为兼容旧客户端保留"""
    import http as _http
    try:
        title = _http.HTTPStatus(status).phrase
    except ValueError:
        title = "Error"
    body = {"type": f"{PROBLEM_TYPE}-{code}" if code and code != "error" else "about:blank", "title": title, "status": status, "detail": detail,
            "code": code, "error": {"code": code, "message": detail}}
    if instance:
        body["instance"] = instance
    return RawBody(json.dumps(body, ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
                   "application/problem+json; charset=utf-8", status=status)


def _doc_parts(doc: str):
    """从路由说明里拆出: 摘要、请求体字段 ({a, b: x|y})、查询参数 (?k=v&k2)"""
    doc = doc or ""
    summary = re.split(r"\s[{?(]|[{?]", doc, 1)[0].strip() or doc.strip()
    body = []
    i = doc.find("{")
    if i >= 0:
        depth, j, cur = 0, i, ""
        for j in range(i, len(doc)):
            c = doc[j]
            if c == "{":
                depth += 1
                if depth == 1:
                    continue
            elif c == "}":
                depth -= 1
                if depth == 0:
                    break
            if depth == 1 and c == ",":
                body.append(cur); cur = ""
            else:
                cur += c
        body.append(cur)
        fields = []
        for f in body:
            f = f.strip()
            m = re.match(r"^([A-Za-z_][\w]*)\s*(?::\s*(.*))?$", f, re.S)
            if m:
                fields.append((m.group(1), (m.group(2) or "").strip()))
        body = fields
    query = []
    for m in re.finditer(r"[?&]([A-Za-z_]\w*)(?:=([^&\s)，,]*))?", doc):
        query.append((m.group(1), m.group(2) or ""))
    return summary, body, query


def _handler_keys(fn, depth: int = 0):
    """从处理函数源码里找出用到的请求体字段 (q.json["k"] / q.json.get("k")) 和查询参数 (q.q("k"))；
    包装函数 (鉴权等) 顺着闭包往里找一层"""
    import inspect
    body, query = [], []
    try:
        src = inspect.getsource(fn)
    except (OSError, TypeError):
        src = ""
    if src.lstrip().startswith(("R(", "self.route(", "api.route(")) or "lambda" in src.split("\n", 1)[0]:
        i = src.find("lambda")                       # 一行里的 lambda: 只看 lambda 之后
        src = src[i:] if i >= 0 else src
    for k in re.findall(r"""q\.json(?:\.get\(\s*|\[\s*)["'](\w+)["']""", src):
        if k not in body:
            body.append(k)
    for var in set(re.findall(r"(\w+)\s*=\s*q\.json\b(?!\s*\.|\s*\[)", src)):     # b = q.json; b["x"] / b.get("x")
        for k in re.findall(r"\b" + var + r"""(?:\.get\(\s*|\[\s*)["'](\w+)["']""", src):
            if k not in body:
                body.append(k)
    for k in re.findall(r"""q\.q\(\s*["'](\w+)["']""", src):
        if k not in query:
            query.append(k)
    if depth < 2 and getattr(fn, "__closure__", None):
        for c in fn.__closure__:
            try:
                v = c.cell_contents
            except ValueError:
                continue
            if callable(v) and getattr(v, "__code__", None) is not None and v is not fn:
                b2, q2 = _handler_keys(v, depth + 1)
                body += [k for k in b2 if k not in body]
                query += [k for k in q2 if k not in query]
    return body, query


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
    def __init__(self, name: str, host: str = "", port: int = 8090, base: str = "/api/v1", title: str = "", description: str = ""):
        host = host or os.environ.get("AGV_BIND", "0.0.0.0")   # 单手机模式实例进程只监听本机 (平台部署时设置)
        self.name, self.host, self.port, self.base = name, host, port, base.rstrip("/")
        self.title, self.description = title or name, description
        self.routes: List[Tuple[str, re.Pattern, str, Callable, str]] = []
        self.fallback: Optional[Callable] = None
        self.mounts: List[Tuple[str, Callable]] = []     # (路径前缀, fn(handler)) 原始处理: 反向代理/流式上传/静态文件
        self.httpd = None
        self.route("GET", self.base, self._index, "列出全部接口 (响应头 Link 指向 OpenAPI 文档，RFC 8631)")
        self.route("GET", self.base + "/openapi.json", lambda q: self.openapi(), "本服务的 OpenAPI 3.1 文档 (由路由表生成)")

    def route(self, method: str, template: str, fn: Callable, doc: str = ""):
        rx = "^" + re.sub(r"\{(\w+)\}", r"(?P<\1>[^/]+)", template.rstrip("/")) + "/?$"
        self.routes.append((method.upper(), re.compile(rx), template, fn, doc))

    def mount(self, prefix: str, fn: Callable):
        """前缀挂载: fn(handler) 直接读写 BaseHTTPRequestHandler (不预读请求体)；返回 False 表示不处理"""
        self.mounts.append((prefix, fn))

    def _index(self, req):
        body = {"service": self.name, "api_version": API_VERSION, "openapi": self.base + "/openapi.json",
                "routes": [{"method": m, "path": t, "doc": d} for m, _, t, _, d in self.routes]}
        return RawBody(json.dumps(body, ensure_ascii=False, separators=(",", ":")).encode("utf-8"), "application/json; charset=utf-8",
                       {"Link": f'<{self.base}/openapi.json>; rel="service-desc"; type="application/openapi+json", '
                                f'<{API_DOC}>; rel="service-doc"'})

    def openapi(self) -> dict:
        """OpenAPI 3.1: 路径/方法/路径参数来自路由模板；摘要、请求体字段、查询参数来自路由说明 (doc 字符串)"""
        build = ""
        try:
            with open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), ".agv_version"), encoding="utf-8") as f:
                build = f.read().split(" ", 1)[0].strip()
        except OSError:
            pass
        paths, tags = {}, []
        for m, _, tpl, fn, doc in self.routes:
            summary, fields, query = _doc_parts(doc)
            hb, hq = _handler_keys(fn)              # 说明里没写的字段，从处理函数源码补上
            fields += [(k, "") for k in hb if k not in [f for f, _ in fields]]
            query += [(k, "") for k in hq if k not in [n for n, _ in query] and k not in re.findall(r"\{(\w+)\}", tpl)]
            rest = tpl[len(self.base):] if tpl.startswith(self.base) else tpl
            seg = [x for x in rest.split("/") if x and not x.startswith("{") and x not in ("api", "hub", "v1", "v2")]
            tag = seg[0] if seg else "meta"
            if tag not in tags:
                tags.append(tag)
            op = {"operationId": (m.lower() + "_" + re.sub(r"[^A-Za-z0-9]+", "_", tpl).strip("_"))[:120], "summary": summary,
                  "tags": [tag], "responses": {
                      "200": {"description": "成功 (JSON；少数接口返回二进制/文本，见说明)",
                              "content": {"application/json": {"schema": {}}}},
                      "default": {"description": "错误 (RFC 9457)", "content": {
                          "application/problem+json": {"schema": {"$ref": "#/components/schemas/Problem"}}}}}}
            if doc and doc != summary:
                op["description"] = doc
            params = [{"name": n, "in": "path", "required": True, "schema": {"type": "string"}} for n in re.findall(r"\{(\w+)\}", tpl)]
            params += [{"name": n, "in": "query", "required": False, "schema": {"type": "string"},
                        **({"description": f"例如 {v}"} if v else {})} for n, v in query]
            if params:
                op["parameters"] = params
            if m in ("POST", "PUT", "PATCH") and (tpl.endswith("/upload") or "请求体为文件" in doc or "请求体为 .tgz" in doc):
                op["requestBody"] = {"required": True, "description": "文件内容",
                                     "content": {"application/octet-stream": {"schema": {"type": "string", "contentMediaType": "application/octet-stream"}}}}
            elif m in ("POST", "PUT", "PATCH"):
                sch = {"type": "array", "items": {"type": "object"}} if "请求体为数组" in doc else {"type": "object"}
                if fields and sch["type"] == "array":
                    sch["items"]["properties"] = {n: ({"description": d} if d else {}) for n, d in fields}
                elif fields:
                    sch["properties"] = {n: ({"description": d} if d else {}) for n, d in fields}
                op["requestBody"] = {"required": bool(fields), "content": {"application/json": {"schema": sch}}}
            paths.setdefault(tpl, {})[m.lower()] = op
        spec = {"openapi": "3.1.0",
                "info": {"title": self.title, "version": API_VERSION, "description": self.description or f"{self.name} 服务接口",
                         **({"x-build": build} if build else {})},
                "servers": [{"url": "/", "description": "相对当前服务地址"}],
                "tags": [{"name": t} for t in tags], "paths": paths,
                "components": {"schemas": {"Problem": {
                    "type": "object", "description": "RFC 9457 Problem Details；code / error 为本项目扩展",
                    "properties": {"type": {"type": "string", "format": "uri-reference"}, "title": {"type": "string"},
                                   "status": {"type": "integer"}, "detail": {"type": "string"},
                                   "instance": {"type": "string", "format": "uri-reference"},
                                   "code": {"type": "string", "description": "机器可读错误码"},
                                   "error": {"type": "object", "description": "兼容旧客户端: {code, message}"}}}}}}
        if self.mounts:
            spec["x-raw-mounts"] = [p for p, _ in self.mounts]
        return spec

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

            _date_cache = (0, "")

            def date_time_string(self, timestamp=None):   # Date 头按秒缓存 (标准实现每次走 email.utils 格式化)
                if timestamp is not None:
                    return super().date_time_string(timestamp)
                now = int(time.time())
                c = Handler._date_cache
                if c[0] != now:
                    c = Handler._date_cache = (now, super().date_time_string(now))
                return c[1]

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
                            return self._send(e.status, problem(e.status, e.code, e.message, self.path.split("?")[0]))
                        except Exception as e:  # pragma: no cover
                            traceback.print_exc()
                            try:
                                return self._send(500, problem(500, "internal", str(e), self.path.split("?")[0]))
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
                    status, payload = e.status, problem(e.status, e.code, e.message, self.path.split("?")[0])
                except Exception as e:  # pragma: no cover
                    traceback.print_exc()
                    status, payload = 500, problem(500, "internal", str(e), self.path.split("?")[0])
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
                self.send_header("Access-Control-Allow-Headers", "Content-Type, X-Lock-Token, X-Node-Key, X-User, X-Admin-Token")
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
