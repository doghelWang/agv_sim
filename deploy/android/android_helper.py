#!/data/data/com.termux/files/usr/bin/python3
"""
安卓助手 (Termux 原生 Python，不在 proot 里) —— 平台「运维管理」用它做 proot 里做不了的安卓操作

为什么需要: 平台跑在 proot 里，/system/bin/pm、am 等系统命令在 proot 下被拒绝执行 (Operation not permitted)，
           Termux 原生环境里则可以。本服务只开放下面几个固定操作，不执行任意命令。

  GET  /status                    {ok, adb: [...]}
  GET  /app/version?package=…     {package, versionCode, installed}
  POST /app/install {path, mode, serial?}
       mode=prompt  termux-open 打开 APK → 手机上弹出系统安装框，用户点一次「安装」(Termux 需要「安装未知应用」权限)
       mode=adb     adb install -r (手机先开「无线调试」并在运维页配对、连接过一次)
       path 必须在 ~/agv-apps/ 下 (平台把 APK 复制到那里)
  GET  /adb/devices
  POST /adb/pair {port, code}     adb pair 127.0.0.1:端口 配对码 (无线调试 →「使用配对码配对设备」里显示)
  POST /adb/connect {port}        adb connect 127.0.0.1:端口 (无线调试主页面显示的「IP 地址和端口」)
  POST /adb/disconnect {serial}
  POST /adb/mdns                  adb mdns services (自动发现无线调试端口)
  POST /panel {action: open}      打开外屏面板 (am broadcast … --ez force true)

只监听 127.0.0.1:8067；请求头 X-Helper-Key 必须等于 ~/.agv-helper.key (首次启动时生成，平台从 Termux 主目录读取)。
启动: start_agv.sh 自动启动；手动 nohup python ~/android_helper.py > ~/android_helper.log 2>&1 &
"""
import json
import os
import re
import secrets
import subprocess
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

PORT = int(os.environ.get("AGV_HELPER_PORT", "8067"))
HOME = os.path.expanduser("~")
KEY_FILE = os.path.join(HOME, ".agv-helper.key")
APPS = os.path.realpath(os.path.join(HOME, "agv-apps"))
PKG_RE = re.compile(r"^[A-Za-z0-9_.]{3,120}$")
SERIAL_RE = re.compile(r"^[A-Za-z0-9_.:\-]{1,80}$")


def key() -> str:
    if not os.path.exists(KEY_FILE):
        fd = os.open(KEY_FILE, os.O_WRONLY | os.O_CREAT, 0o600)
        with os.fdopen(fd, "w") as f:
            f.write(secrets.token_urlsafe(24))
    with open(KEY_FILE) as f:
        return f.read().strip()


KEY = key()


def run(argv, timeout=30):
    try:
        p = subprocess.run(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout)
        return p.returncode, p.stdout.decode("utf-8", "replace").strip()
    except FileNotFoundError:
        return 127, f"找不到命令 {argv[0]}"
    except subprocess.TimeoutExpired:
        return 124, f"{argv[0]} 超时 ({timeout} s)"


def port_of(v):
    try:
        p = int(v)
    except (TypeError, ValueError):
        raise ValueError("端口无效")
    if not 1 <= p <= 65535:
        raise ValueError("端口无效")
    return p


def adb_devices():
    rc, out = run(["adb", "devices"], 15)
    if rc != 0:
        return []
    devs = []
    for line in out.splitlines()[1:]:
        parts = line.split()
        if len(parts) >= 2:
            devs.append({"serial": parts[0], "state": parts[1]})
    return devs


def app_version(pkg):
    if not PKG_RE.match(pkg or ""):
        raise ValueError("包名无效")
    rc, out = run(["pm", "list", "packages", "--show-versioncode", pkg], 20)
    for line in out.splitlines():
        m = re.match(r"package:(\S+)\s+versionCode:(\d+)", line.strip())
        if m and m.group(1) == pkg:
            return {"package": pkg, "installed": True, "versionCode": int(m.group(2))}
    return {"package": pkg, "installed": False, "versionCode": None, "detail": out[-200:] if rc else ""}


def install(body):
    path = os.path.realpath(str(body.get("path") or ""))
    if not path.startswith(APPS + os.sep) or not path.endswith(".apk") or not os.path.isfile(path):
        raise ValueError("APK 路径无效 (必须在 ~/agv-apps/ 下)")
    mode = body.get("mode") or "prompt"
    if mode == "prompt":
        rc, out = run(["termux-open", "--view", "--content-type", "application/vnd.android.package-archive", path], 30)
        if rc != 0:
            raise RuntimeError(f"打开安装界面失败: {out}")
        return {"ok": True, "mode": mode, "msg": "已在手机上弹出安装界面，请在手机上点「安装」(第一次会先要求允许 Termux 安装未知应用)"}
    if mode == "adb":
        serial = body.get("serial") or ""
        devs = [d for d in adb_devices() if d["state"] == "device"]
        if not serial:
            if not devs:
                raise RuntimeError("没有已连接的 adb 设备: 先在手机上打开「无线调试」，在运维页完成配对和连接")
            serial = devs[0]["serial"]
        if not SERIAL_RE.match(serial):
            raise ValueError("设备序列号无效")
        rc, out = run(["adb", "-s", serial, "install", "-r", path], 170)
        if rc != 0 or "Success" not in out:
            raise RuntimeError(f"adb 安装失败: {out[-400:]}")
        return {"ok": True, "mode": mode, "serial": serial, "msg": "安装完成"}
    raise ValueError("mode 只能是 prompt 或 adb")


def adb_action(action, body):
    if action == "pair":
        code = str(body.get("code") or "").strip()
        if not re.match(r"^\d{6}$", code):
            raise ValueError("配对码应为 6 位数字")
        rc, out = run(["adb", "pair", f"127.0.0.1:{port_of(body.get('port'))}", code], 30)
    elif action == "connect":
        rc, out = run(["adb", "connect", f"127.0.0.1:{port_of(body.get('port'))}"], 20)
        if "connected" not in out or "cannot" in out or "failed" in out:
            rc = rc or 1
    elif action == "disconnect":
        s = str(body.get("serial") or "")
        if s and not SERIAL_RE.match(s):
            raise ValueError("设备序列号无效")
        rc, out = run(["adb", "disconnect"] + ([s] if s else []), 15)
    elif action == "mdns":
        rc, out = run(["adb", "mdns", "services"], 15)
    else:
        raise ValueError("未知操作")
    if rc != 0:
        raise RuntimeError(out[-400:] or f"adb 退出码 {rc}")
    return {"ok": True, "output": out[-1000:], "devices": adb_devices()}


def panel(body):
    if (body.get("action") or "open") != "open":
        raise ValueError("只支持 open")
    rc, out = run(["am", "broadcast", "-n", "com.agvsim.cover/.StartReceiver", "--ez", "force", "true"], 20)
    if rc != 0:
        raise RuntimeError(out[-300:])
    return {"ok": True, "msg": "已请求打开外屏面板"}


class H(BaseHTTPRequestHandler):
    def log_message(self, fmt, *a):
        sys.stdout.write("[helper] " + (fmt % a) + "\n"); sys.stdout.flush()

    def _send(self, code, obj):
        b = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def _handle(self, method):
        if not secrets.compare_digest(self.headers.get("X-Helper-Key", ""), KEY):
            return self._send(401, {"error": "密钥无效"})
        u = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(u.query).items()}
        body = {}
        if method == "POST":
            n = int(self.headers.get("Content-Length") or 0)
            if n > 65536:
                return self._send(413, {"error": "请求过大"})
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                return self._send(400, {"error": "请求体不是 JSON"})
        try:
            p = u.path
            if method == "GET" and p == "/status":
                r = {"ok": True, "pid": os.getpid(), "adb": adb_devices()}
            elif method == "GET" and p == "/app/version":
                r = app_version(q.get("package", ""))
            elif method == "POST" and p == "/app/install":
                r = install(body)
            elif method == "GET" and p == "/adb/devices":
                r = {"devices": adb_devices()}
            elif method == "POST" and p.startswith("/adb/"):
                r = adb_action(p[5:], body)
            elif method == "POST" and p == "/panel":
                r = panel(body)
            else:
                return self._send(404, {"error": "没有这个接口"})
        except ValueError as e:
            return self._send(400, {"error": str(e)})
        except Exception as e:
            return self._send(500, {"error": str(e)})
        self._send(200, r)

    def do_GET(self):
        self._handle("GET")

    def do_POST(self):
        self._handle("POST")


if __name__ == "__main__":
    os.makedirs(APPS, exist_ok=True)
    print(f"[helper] 安卓助手 127.0.0.1:{PORT}", flush=True)
    ThreadingHTTPServer(("127.0.0.1", PORT), H).serve_forever()
