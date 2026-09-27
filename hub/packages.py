#!/usr/bin/env python3
"""
软件程序包 (Docker 镜像) 仓库

  kind = sim (仿真程序包 agv-sim) | nav (运行程序包 agv-nav)
  source = upload (上传的 docker save 包，存于 packages/<pid>.tar[.gz]，可分发到任意同架构节点)
         | node   (节点心跳上报的本机镜像，仅在拥有该镜像的节点上可直接部署；可"入库"导出为 upload)
         | process(开发用源码进程运行时)
"""

import io
import json
import os
import tarfile
import time
from typing import Optional

from common.rest import ApiError
from hub.store import Store, new_id

TAGS = {"baseline": "基准推荐", "test": "测试版", "current": "当前运行", "lite": "轻量版", "node": "节点镜像"}


def read_image_tar(path: str) -> dict:
    """解析 docker save 包 (tar / tar.gz): RepoTags、架构、镜像标签"""
    out = {"refs": [], "arch": None, "labels": {}}
    try:
        with tarfile.open(path, "r:*") as t:
            man = json.loads(t.extractfile("manifest.json").read().decode())
            if not man:
                return out
            out["refs"] = man[0].get("RepoTags") or []
            cfg_name = man[0].get("Config")
            if cfg_name:
                cfg = json.loads(t.extractfile(cfg_name).read().decode())
                out["arch"] = cfg.get("architecture")
                out["labels"] = (cfg.get("config") or {}).get("Labels") or {}
                out["created"] = cfg.get("created")
    except Exception as e:
        out["error"] = str(e)
    return out


class PackageRepo:
    def __init__(self, store: Store):
        self.s = store

    def get(self, pid: str) -> dict:
        p = self.s.get("packages", pid)
        if not p:
            raise ApiError(404, f"程序包 {pid} 不存在")
        return p

    def list(self, kind: Optional[str] = None) -> list:
        ps = self.s.list("packages")
        if kind:
            ps = [p for p in ps if p["kind"] == kind]
        return sorted(ps, key=lambda p: (p["kind"], -(p.get("created") or 0)))

    def file(self, p: dict) -> Optional[str]:
        if p.get("file"):
            f = self.s.path("packages", p["file"])
            if os.path.exists(f):
                return f
        return None

    # ------------------------------------------------------------------ 上传 (流式写盘)
    def receive(self, handler, meta: dict) -> dict:
        pid = new_id("pkg-", {p["id"]: 1 for p in self.s.list("packages")})
        tmp = self.s.path("tmp", pid + ".part")
        n = 0
        with open(tmp, "wb") as f:
            te = (handler.headers.get("Transfer-Encoding") or "").lower()
            if "chunked" in te:
                while True:
                    line = handler.rfile.readline().strip()
                    size = int(line.split(b";")[0], 16) if line else 0
                    if size == 0:
                        handler.rfile.readline()
                        break
                    remain = size
                    while remain:
                        b = handler.rfile.read(min(remain, 1 << 20))
                        if not b:
                            raise ApiError(400, "上传中断")
                        f.write(b)
                        remain -= len(b)
                        n += len(b)
                    handler.rfile.readline()
            else:
                remain = int(handler.headers.get("Content-Length") or 0)
                while remain:
                    b = handler.rfile.read(min(remain, 1 << 20))
                    if not b:
                        raise ApiError(400, "上传中断")
                    f.write(b)
                    remain -= len(b)
                    n += len(b)
        if n < 200:
            os.remove(tmp)
            raise ApiError(400, "程序包为空或过小")
        with open(tmp, "rb") as f:
            gz = f.read(2) == b"\x1f\x8b"
        info = read_image_tar(tmp)
        if info.get("error") or not info["refs"]:
            os.remove(tmp)
            raise ApiError(400, f"不是有效的 docker save 镜像包: {info.get('error') or '无 RepoTags'}")
        fname = pid + (".tar.gz" if gz else ".tar")
        os.replace(tmp, self.s.path("packages", fname))
        ref = meta.get("ref") or info["refs"][0]
        labels = info.get("labels") or {}
        kind = meta.get("kind") or labels.get("org.agv.kind") or ("nav" if "agv-nav" in ref else "sim")
        doc = {"id": pid, "kind": kind, "source": "upload", "file": fname, "size": n, "image_ref": ref, "refs": info["refs"],
               "arch": _arch(info.get("arch")), "version": meta.get("version") or labels.get("org.agv.version") or ref.split(":")[-1],
               "api": labels.get("org.agv.api", "v1"), "note": meta.get("note", ""), "tag": meta.get("tag", ""),
               "image_created": info.get("created"), "operator": meta.get("operator", ""), "from_node": meta.get("from_node")}
        return self.s.put("packages", doc)

    # ------------------------------------------------------------------ 节点镜像登记 (心跳)
    def sync_node_images(self, node: dict):
        info = node.get("info") or {}
        rt = (info.get("runtime") or {}).get("runtime")
        for im in info.get("images", []):
            key = f"{im['kind']}:{im['id']}"
            exists = next((p for p in self.s.list("packages") if p.get("image_key") == key), None)
            if exists:
                nodes = set(exists.get("nodes", []))
                if node["id"] not in nodes:
                    nodes.add(node["id"])
                    self.s.update("packages", exists["id"], nodes=sorted(nodes))
                continue
            pid = new_id("pkg-", {p["id"]: 1 for p in self.s.list("packages")})
            self.s.put("packages", {"id": pid, "kind": im["kind"], "source": "process" if rt == "process" else "node",
                                    "image_ref": im["ref"], "image_key": key, "image_id": im["id"], "size": im.get("size", 0),
                                    "arch": node.get("arch"), "version": im.get("version") or im["ref"].split(":")[-1],
                                    "api": im.get("api", "v1"), "nodes": [node["id"]], "tag": "node",
                                    "note": "开发源码运行时" if rt == "process" else f"节点 {node.get('name')} 本机镜像",
                                    "image_created": im.get("created")})

    def update(self, pid: str, fields: dict) -> dict:
        p = self.get(pid)
        for k in ("version", "note", "tag"):
            if k in fields:
                p[k] = fields[k]
        if fields.get("tag") == "baseline":        # 同类只保留一个基准推荐
            for o in self.s.list("packages", kind=p["kind"]):
                if o["id"] != pid and o.get("tag") == "baseline":
                    self.s.update("packages", o["id"], tag="")
        return self.s.put("packages", p)

    def delete(self, pid: str):
        p = self.get(pid)
        f = self.file(p)
        if f:
            os.remove(f)
        self.s.delete("packages", pid)

    def deployable_on(self, p: dict, node: dict) -> Optional[str]:
        """None=可部署；否则返回原因"""
        if p["source"] == "process":
            rt = ((node.get("info") or {}).get("runtime") or {}).get("runtime")
            return None if rt == "process" else "开发源码包只能部署到 process 运行时节点"
        if node["id"] in (p.get("nodes") or []) and node_has_image(node, p):
            return None
        if p.get("arch") and node.get("arch") and _arch(p["arch"]) != _arch(node["arch"]):
            return f"架构不符: 包 {p['arch']} / 节点 {node['arch']}"
        if p["source"] == "node" and not self.file(p):
            return f"镜像只在节点 {','.join(p.get('nodes', []))} 上，需先「入库」才能分发到其它节点"
        return None


def node_has_image(node: dict, p: dict) -> bool:
    for im in (node.get("info") or {}).get("images", []):
        if im["ref"] == p.get("image_ref") or (p.get("image_id") and im["id"] == p.get("image_id")):
            return True
    return False


def _arch(a: Optional[str]) -> Optional[str]:
    return {"aarch64": "arm64", "x86_64": "amd64"}.get(a, a)
