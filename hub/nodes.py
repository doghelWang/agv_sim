#!/usr/bin/env python3
"""
计算节点注册表 + 节点代理客户端

节点 doc: {id, name, host, api_port, key, kind(controller|hybrid|sim), arch, info(心跳), status, last_seen,
          max_instances, note, hub_url, created}
接入: 平台生成一次性令牌 (tokens 表) 或集群令牌 (settings.cluster_token) → 节点代理带令牌注册 → 平台签发 node_key
"""

import json
import secrets
import socket
import time
import urllib.parse
from http.client import HTTPConnection
from typing import Optional

from common.rest import ApiError
from hub.store import Store, new_id

KIND_LABEL = {"controller": "实体运行设备 (控制器)", "hybrid": "虚拟机设备 (运行 + 仿真引擎)", "sim": "虚拟机设备 (仿真引擎)"}
CAPS = {"controller": {"sim": False, "nav": True}, "hybrid": {"sim": True, "nav": True}, "sim": {"sim": True, "nav": False}}
OFFLINE_AFTER = 20.0


class AgentError(Exception):
    pass


class AgentClient:
    def __init__(self, node: dict, timeout: float = 10.0):
        self.node, self.timeout = node, timeout
        self.base = f"http://{node['host']}:{node.get('api_port', 8070)}"

    def call(self, method: str, path: str, body=None, timeout: Optional[float] = None):
        u = urllib.parse.urlparse(self.base)
        c = HTTPConnection(u.hostname, u.port, timeout=timeout or self.timeout)
        try:
            c.request(method, path, body=json.dumps(body).encode() if body is not None else None,
                      headers={"Content-Type": "application/json", "X-Node-Key": self.node.get("key", "")})
            r = c.getresponse()
            raw = r.read()
        except (OSError, socket.timeout) as e:
            raise AgentError(f"节点 {self.node.get('name')} ({self.base}) 不可达: {e}")
        finally:
            c.close()
        obj = json.loads(raw.decode() or "{}") if raw else {}
        if r.status >= 400:
            raise AgentError(f"节点 {self.node.get('name')}: {(obj.get('error') or {}).get('message', r.status)}")
        return obj


class NodeRegistry:
    def __init__(self, store: Store, packages=None):
        self.s = store
        self.packages = packages
        if not self.s.setting("cluster_token"):
            self.s.set_setting("cluster_token", "agv-" + secrets.token_urlsafe(12))

    # ------------------------------------------------------------------ 查询
    def get(self, nid: str) -> dict:
        n = self.s.get("nodes", nid)
        if not n:
            raise ApiError(404, f"节点 {nid} 不存在")
        return self._live(n)

    def _live(self, n: dict) -> dict:
        n = dict(n)
        if time.time() - (n.get("last_seen") or 0) > OFFLINE_AFTER:
            n["status"] = "offline"
        return n

    def list(self) -> list:
        return [self._live(n) for n in sorted(self.s.list("nodes"), key=lambda n: n.get("created", 0))]

    def view(self, n: dict, instances: list = ()) -> dict:
        """前端展示 (不含密钥)"""
        info = n.get("info") or {}
        mine = [i for i in instances if n["id"] in (i.get("sim_node"), i.get("nav_node")) and i.get("status") in
                ("deploying", "running", "degraded", "starting")]
        busy = "offline" if n.get("status") == "offline" else ("running" if mine else "idle")
        mem = info.get("mem") or {}
        return {"id": n["id"], "name": n.get("name"), "host": n.get("host"), "lan_host": self.lan_host(n), "api_port": n.get("api_port"),
                "kind": n.get("kind", "hybrid"), "kind_label": KIND_LABEL.get(n.get("kind", "hybrid")), "caps": caps(n),
                "arch": n.get("arch"), "status": busy, "online": n.get("status") != "offline", "last_seen": n.get("last_seen"),
                "cpu_count": info.get("cpu_count"), "cpu_percent": info.get("cpu_percent"), "mem_total": mem.get("total"),
                "mem_percent": mem.get("percent"), "temp_c": info.get("temp_c"), "model": info.get("model"), "os": info.get("os"),
                "runtime": info.get("runtime"), "images": info.get("images", []), "containers": info.get("containers", []),
                "ports_in_use": info.get("ports_in_use", []), "disk": info.get("disk"), "ips": info.get("ips", []),
                "load": info.get("load"), "ssh": {k: v for k, v in (n.get("ssh") or {}).items() if k != "password"} or None, "max_instances": n.get("max_instances", 1), "note": n.get("note", ""),
                "instances": [{"id": i["id"], "name": i.get("name"), "operator": i.get("operator"), "role":
                               "sim+nav" if i.get("sim_node") == i.get("nav_node") else ("sim" if i.get("sim_node") == n["id"] else "nav")}
                              for i in mine],
                "agent_error": info.get("last_error"), "role": info.get("role") or "full", "history": n.get("history", [])[-60:], "created": n.get("created")}

    # ------------------------------------------------------------------ 地址
    @staticmethod
    def lan_host(n: dict) -> str:
        """跨节点访问用地址: 注册地址是回环 (与平台同机) 时取节点上报的局域网 IP"""
        h = n.get("host") or "127.0.0.1"
        if h.startswith("127.") or h == "localhost":
            ips = (n.get("info") or {}).get("ips") or []
            if ips:
                return ips[0]
        return h

    def addr_between(self, src: dict, dst: dict) -> str:
        """src 节点上的进程访问 dst 节点时使用的主机名"""
        if src["id"] == dst["id"]:
            return "127.0.0.1"
        return self.lan_host(dst)

    # ------------------------------------------------------------------ 接入
    def new_token(self, operator: str = "", note: str = "", meta: Optional[dict] = None, ttl: float = 24 * 3600) -> dict:
        """meta: 注册成功后写到节点上的字段 (SSH 接入时的类型/备注/SSH 地址)"""
        tok = "jt-" + secrets.token_urlsafe(10)
        return self.s.put("tokens", {"id": tok, "operator": operator, "note": note, "used": False, "expires": time.time() + ttl,
                                     "meta": meta or {}})

    def register(self, body: dict, client_ip: str) -> dict:
        nid, key, tok = body.get("node_id"), body.get("node_key"), body.get("token") or ""
        host = body.get("advertise_host") or client_ip
        n = self.s.get("nodes", nid) if nid else None
        used_tok = None
        if n and key and secrets.compare_digest(n.get("key", ""), key):
            pass
        else:
            ok = tok and tok == self.s.setting("cluster_token")
            t = self.s.get("tokens", tok) if tok and not ok else None
            meta = {}
            if t and not t.get("used") and t.get("expires", 0) > time.time():
                ok = True
                meta = t.get("meta") or {}
                used_tok = tok
            if not ok:
                raise ApiError(401, "接入令牌无效或已过期，请在平台「添加计算节点」重新生成", "bad_token")
            n = next((x for x in self.s.list("nodes") if x.get("name") == body.get("name") and x.get("host") == host), None)
            if not n:
                n = {"id": new_id("n-", {x["id"]: 1 for x in self.s.list("nodes")}), "created": time.time(),
                     "kind": body.get("kind") or "hybrid", "max_instances": 1}
            n["key"] = secrets.token_urlsafe(24)
            for k in ("kind", "note", "ssh"):
                if meta.get(k):
                    n[k] = meta[k]
        info = body.get("info") or {}
        n.update({"name": body.get("name") or n.get("name") or host, "host": host, "api_port": int(body.get("api_port") or 8070),
                  "hub_url": body.get("hub_url"), "arch": info.get("arch") or n.get("arch"), "status": "online",
                  "last_seen": time.time()})
        if info:
            n["info"] = info
        self.s.put("nodes", n)
        if used_tok:
            self.s.update("tokens", used_tok, used=True, used_by=body.get("name"), used_at=time.time(), node_id=n["id"])
        return {"node_id": n["id"], "node_key": n["key"], "host": host}

    def heartbeat(self, nid: str, key: str, info: dict) -> dict:
        n = self.s.get("nodes", nid)
        if not n:
            raise ApiError(404, "节点未注册")
        if not secrets.compare_digest(n.get("key", ""), key or ""):
            raise ApiError(401, "节点密钥无效")
        hist = n.get("history", [])
        hist.append([round(time.time()), info.get("cpu_percent"), (info.get("mem") or {}).get("percent"), info.get("temp_c")])
        n.update({"info": info, "last_seen": time.time(), "status": "online", "arch": info.get("arch") or n.get("arch"),
                  "history": hist[-120:]})
        self.s.put("nodes", n)
        if self.packages:
            try:
                self.packages.sync_node_images(n)
            except Exception as e:
                print(f"[hub] 同步节点镜像失败: {e}", flush=True)
        return {"ok": True}

    def update(self, nid: str, fields: dict) -> dict:
        n = self.s.get("nodes", nid)
        if not n:
            raise ApiError(404, "节点不存在")
        for k in ("name", "kind", "note", "max_instances"):
            if k in fields:
                n[k] = int(fields[k]) if k == "max_instances" else fields[k]
        return self.s.put("nodes", n)

    def delete(self, nid: str):
        self.s.delete("nodes", nid)

    def client(self, nid_or_node) -> AgentClient:
        n = nid_or_node if isinstance(nid_or_node, dict) else self.get(nid_or_node)
        return AgentClient(n)


def caps(n: dict) -> dict:
    return CAPS.get(n.get("kind", "hybrid"), CAPS["hybrid"])
