#!/usr/bin/env python3
"""
通过 SSH 接入计算节点 (安装基础环境): 平台用用户在「添加计算节点」里填的 IP / SSH 端口 / 用户名 / 密码登录目标设备，
检查基础环境 (python3、Docker)，把节点程序 (agent + common，纯 Python 标准库) 拷过去，以「基础监测」角色 (AGENT_ROLE=monitor)
用一次性令牌 (平台内部生成，不给用户看) 在后台启动: 只上报 CPU/内存/温度/磁盘等状态，不接受部署。
第一次在该节点部署仿真时，平台把它切换为完整运行环境 (POST /api/v1/admin/role)，之后照常部署。

  - 密码只用于这一次登录，不写入平台数据库
  - 目标设备需要 python3 (>= 3.8)；有 Docker 时代理用 Docker 运行仿真实例，没有时用进程方式
  - 代理装在目标设备的 ~/.agv-agent/app，日志 ~/.agv-agent/agent.log，进程号 ~/.agv-agent/agent.pid
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
export HUB_API=__HUB__ JOIN_TOKEN=__TOKEN__ AGENT_KIND=__KIND__ AGENT_HOST=__HOST__ AGENT_DATA="$D" AGENT_ROLE=monitor PYTHONDONTWRITEBYTECODE=1
__NAME__
nohup python3 -m agent.server > "$D/agent.log" 2>&1 < /dev/null &
echo $! > "$D/agent.pid"
sleep 4
if ! kill -0 "$(cat "$D/agent.pid")" 2>/dev/null; then echo "AGVERR 基础监测程序启动后退出:"; tail -n 15 "$D/agent.log"; exit 14; fi
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


def install_agent(ip: str, port: int, user: str, password: str, hub_url: str, token: str, kind: str, name: str = "",
                  timeout: int = 90) -> str:
    """登录目标设备安装基础环境 (基础监测)；成功返回目标设备简况，失败抛 ApiError (带中文原因)"""
    script = (REMOTE.replace("__HUB__", shlex.quote(hub_url)).replace("__TOKEN__", shlex.quote(token))
              .replace("__KIND__", shlex.quote(kind)).replace("__HOST__", shlex.quote(ip))
              .replace("__NAME__", f"export AGENT_NAME={shlex.quote(name)}" if name else ""))
    cmd = "bash -c " + shlex.quote(script)
    data = _payload()
    try:
        import paramiko  # noqa: F401
        rc, out = _run_paramiko(ip, port, user, password, cmd, data, timeout)
    except ImportError:
        rc, out = _run_openssh(ip, port, user, password, cmd, data, timeout)
    if rc != 0 or "AGVOK" not in out:
        msg = "\n".join(l.replace("AGVERR ", "") for l in out.strip().splitlines()[-16:]) or f"退出码 {rc}"
        raise ApiError(502, f"在 {ip} 上安装基础环境失败: {msg}", "agent_start")
    return next(l for l in reversed(out.splitlines()) if l.startswith("AGVOK ")).replace("AGVOK ", "").strip()
