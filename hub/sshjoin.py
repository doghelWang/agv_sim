#!/usr/bin/env python3
"""
计算节点的 SSH 接入 (hub)

  添加节点 (check_login): 只验证 IP / SSH 端口 / 用户名 / 密码能否登录，读出架构与系统，保存到节点记录；不在目标设备上装任何东西。
                         密码用平台本机密钥 (数据目录 secret.key，仅本机可读) 加密后保存，接口永不返回。
  首次部署 (install_agent): 部署仿真到这个节点时，平台再用保存的账号登录，把节点程序 (agent + common，纯 Python 标准库)
                         拷到 ~/.agv-agent/app 并后台启动；它用一次性令牌注册到这条节点记录上，之后照常心跳、部署。
  - 目标设备需要 python3 (>= 3.8)；部署仿真实例需要 Docker (镜像由平台分发)
  - 节点程序日志 ~/.agv-agent/agent.log，进程号 ~/.agv-agent/agent.pid
  - SSH 客户端: 有 paramiko 就用；没有则用系统 ssh (OpenSSH >= 8.4，通过 SSH_ASKPASS 传密码)
"""

import base64
import io
import os
import shlex
import shutil
import socket
import subprocess
import tarfile
import tempfile
from typing import Tuple

from common.rest import ApiError

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

REMOTE = r'''
D="$HOME/.agv-agent"; mkdir -p "$D/app" || exit 10
command -v python3 >/dev/null 2>&1 || { echo "AGVERR 目标设备没有 python3"; exit 11; }
python3 -c 'import sys; sys.exit(0 if sys.version_info >= (3, 8) else 1)' || { echo "AGVERR 目标设备的 python3 版本过低 (需要 3.8 以上)"; exit 12; }
cd "$D/app" && rm -rf agent common && base64 -d | tar xzf - || { echo "AGVERR 解包失败"; exit 13; }
# 停掉之前装的代理 (本方式启动的进程，或 deploy.sh 起的 agv-agent 容器)，避免端口冲突
if [ -f "$D/agent.pid" ] && kill -0 "$(cat "$D/agent.pid")" 2>/dev/null; then kill "$(cat "$D/agent.pid")"; sleep 1; fi
command -v docker >/dev/null 2>&1 && docker rm -f agv-agent >/dev/null 2>&1
export HUB_API=__HUB__ JOIN_TOKEN=__TOKEN__ AGENT_KIND=__KIND__ AGENT_HOST=__HOST__ AGENT_DATA="$D" AGENT_ROLE=full PYTHONDONTWRITEBYTECODE=1
__NAME__
nohup python3 -m agent.server > "$D/agent.log" 2>&1 < /dev/null &
echo $! > "$D/agent.pid"
sleep 4
if ! kill -0 "$(cat "$D/agent.pid")" 2>/dev/null; then echo "AGVERR 节点程序启动后退出:"; tail -n 15 "$D/agent.log"; exit 14; fi
DK="无 Docker"; if command -v docker >/dev/null 2>&1; then V=$(docker version --format '{{.Server.Version}}' 2>/dev/null | head -1); DK="Docker ${V:-(当前用户无权访问 Docker 服务)}"; fi
echo "AGVOK $(uname -m) · $(python3 -V 2>&1) · $DK"
'''


def _payload() -> str:
    """agent/ 与 common/ 打成 tar.gz (base64)，经 SSH 标准输入传过去"""
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w:gz") as tf:
        for pkg in ("agent", "common"):
            for dp, dns, fns in os.walk(os.path.join(ROOT, pkg)):
                dns[:] = [d for d in dns if d != "__pycache__"]
                for f in fns:
                    if f.endswith(".py"):
                        p = os.path.join(dp, f)
                        tf.add(p, arcname=os.path.relpath(p, ROOT))
    return base64.b64encode(buf.getvalue()).decode()


def local_ip_towards(ip: str, port: int) -> str:
    """平台这台机器去往目标设备时用的本机地址 (目标设备回连平台用)"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((ip, port or 22))
        return s.getsockname()[0]
    except OSError:
        return ""
    finally:
        s.close()


def _run_paramiko(ip, port, user, password, cmd, data, timeout) -> Tuple[int, str]:
    import paramiko  # noqa: 只在装了 paramiko 时走这里
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        c.connect(ip, port=port, username=user, password=password, timeout=10, auth_timeout=15,
                  allow_agent=False, look_for_keys=False)
    except paramiko.AuthenticationException:
        raise ApiError(401, "SSH 登录失败: 用户名或密码错误", "ssh_auth")
    except Exception as e:
        raise ApiError(502, f"SSH 连接 {ip}:{port} 失败: {e}", "ssh_connect")
    try:
        stdin, stdout, _ = c.exec_command(cmd, timeout=timeout, get_pty=False)
        stdin.write(data); stdin.channel.shutdown_write()
        out = stdout.read().decode("utf-8", "replace")
        return stdout.channel.recv_exit_status(), out
    finally:
        c.close()


def _run_openssh(ip, port, user, password, cmd, data, timeout) -> Tuple[int, str]:
    if not shutil.which("ssh"):
        raise ApiError(500, "平台所在设备没有 SSH 客户端 (请安装 openssh-client，或 pip install paramiko)", "no_ssh")
    d = tempfile.mkdtemp(prefix="agvssh-")
    try:
        pw = os.path.join(d, "pw"); ask = os.path.join(d, "ask.sh")
        with open(os.open(pw, os.O_WRONLY | os.O_CREAT, 0o600), "w") as f:
            f.write(password)
        with open(os.open(ask, os.O_WRONLY | os.O_CREAT, 0o700), "w") as f:
            f.write(f"#!/bin/sh\ncat {shlex.quote(pw)}\n")
        env = dict(os.environ, SSH_ASKPASS=ask, SSH_ASKPASS_REQUIRE="force", DISPLAY=os.environ.get("DISPLAY", ":0"))
        args = ["ssh", "-p", str(port), "-o", "StrictHostKeyChecking=no", "-o", f"UserKnownHostsFile={os.path.join(d, 'kh')}",
                "-o", "ConnectTimeout=10", "-o", "PreferredAuthentications=password,keyboard-interactive",
                "-o", "PubkeyAuthentication=no", "-o", "NumberOfPasswordPrompts=1", "-o", "LogLevel=ERROR",
                f"{user}@{ip}", cmd]
        try:
            p = subprocess.run(args, input=data.encode(), stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env,
                               timeout=timeout, start_new_session=True)
        except subprocess.TimeoutExpired:
            raise ApiError(504, f"SSH 操作超时 ({timeout} s)", "ssh_timeout")
        out, err = p.stdout.decode("utf-8", "replace"), p.stderr.decode("utf-8", "replace").strip()
        if p.returncode == 255:
            if "Permission denied" in err:
                raise ApiError(401, "SSH 登录失败: 用户名或密码错误", "ssh_auth")
            raise ApiError(502, f"SSH 连接 {ip}:{port} 失败: {err[-300:] or '无响应'}", "ssh_connect")
        return p.returncode, out + (("\n" + err) if err and p.returncode else "")
    finally:
        shutil.rmtree(d, ignore_errors=True)


def _ssh(ip, port, user, password, cmd, data, timeout):
    try:
        import paramiko  # noqa: F401
        return _run_paramiko(ip, port, user, password, cmd, data, timeout)
    except ImportError:
        return _run_openssh(ip, port, user, password, cmd, data, timeout)


def check_login(ip: str, port: int, user: str, password: str) -> dict:
    """添加节点: 只验证端口可达、账号能登录；返回 {arch, os, python, docker, brief}"""
    try:
        socket.create_connection((ip, port), timeout=6).close()
    except OSError as e:
        raise ApiError(502, f"{ip}:{port} 端口不通: {e}", "ssh_connect")
    script = r'''echo "AGVOK"; uname -m; (. /etc/os-release 2>/dev/null && echo "$PRETTY_NAME") || uname -sr
python3 -V 2>&1 | head -1 || echo "无 python3"
if command -v docker >/dev/null 2>&1; then V=$(docker version --format '{{.Server.Version}}' 2>/dev/null | head -1); echo "Docker ${V:-(当前用户无权访问 Docker 服务)}"; else echo "无 Docker"; fi'''
    rc, out = _ssh(ip, port, user, password, "sh -c " + shlex.quote(script), "", 30)
    lines = out.splitlines()
    if "AGVOK" not in lines:
        raise ApiError(502, f"登录 {ip} 成功但执行命令失败: {out.strip()[-200:]}", "ssh_exec")
    v = (lines[lines.index("AGVOK") + 1:] + ["", "", "", ""])[:4]
    arch = {"aarch64": "arm64", "x86_64": "amd64", "armv7l": "arm"}.get(v[0].strip(), v[0].strip())
    return {"arch": arch, "os": v[1].strip(), "python": v[2].strip(), "docker": v[3].strip(),
            "brief": " · ".join(x for x in (arch, v[1].strip(), v[2].strip(), v[3].strip()) if x)}


# ---------------------------------------------------------------- 密码加密保存 (HMAC-SHA256 流加密 + 校验，密钥只在平台本机)
def _key(store) -> bytes:
    p = store.path("secret.key")
    if not os.path.exists(p):
        fd = os.open(p, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, "wb") as f:
            f.write(os.urandom(32))
    with open(p, "rb") as f:
        return f.read()


def seal(store, text: str) -> str:
    import hmac, hashlib
    k, nonce, data = _key(store), os.urandom(16), text.encode()
    stream = b"".join(hmac.new(k, nonce + i.to_bytes(4, "big"), hashlib.sha256).digest() for i in range(len(data) // 32 + 1))
    ct = bytes(a ^ b for a, b in zip(data, stream))
    tag = hmac.new(k, b"mac" + nonce + ct, hashlib.sha256).digest()[:16]
    return base64.b64encode(nonce + tag + ct).decode()


def unseal(store, blob: str) -> str:
    import hmac, hashlib
    raw = base64.b64decode(blob)
    k, nonce, tag, ct = _key(store), raw[:16], raw[16:32], raw[32:]
    if not hmac.compare_digest(tag, hmac.new(k, b"mac" + nonce + ct, hashlib.sha256).digest()[:16]):
        raise ApiError(500, "保存的 SSH 密码无法解密 (平台密钥已变)，请重新添加该节点", "cred")
    stream = b"".join(hmac.new(k, nonce + i.to_bytes(4, "big"), hashlib.sha256).digest() for i in range(len(ct) // 32 + 1))
    return bytes(a ^ b for a, b in zip(ct, stream)).decode()


def install_agent(ip: str, port: int, user: str, password: str, hub_url: str, token: str, kind: str, name: str = "",
                  timeout: int = 90) -> str:
    """首次部署: 登录目标设备安装并启动节点程序；成功返回目标设备简况，失败抛 ApiError (带中文原因)"""
    script = (REMOTE.replace("__HUB__", shlex.quote(hub_url)).replace("__TOKEN__", shlex.quote(token))
              .replace("__KIND__", shlex.quote(kind)).replace("__HOST__", shlex.quote(ip))
              .replace("__NAME__", f"export AGENT_NAME={shlex.quote(name)}" if name else ""))
    rc, out = _ssh(ip, port, user, password, "bash -c " + shlex.quote(script), _payload(), timeout)
    if rc != 0 or "AGVOK" not in out:
        msg = "\n".join(l.replace("AGVERR ", "") for l in out.strip().splitlines()[-16:]) or f"退出码 {rc}"
        raise ApiError(502, f"在 {ip} 上安装运行环境失败: {msg}", "agent_start")
    return next(l for l in reversed(out.splitlines()) if l.startswith("AGVOK ")).replace("AGVOK ", "").strip()
