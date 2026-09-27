#!/usr/bin/env python3
"""
仿真实例编排: 部署 (仿真节点 agv-sim + 执行节点 agv-nav，可同机可分离) / 监控 / 重启 / 终止

部署时序 (任一步失败 → 回滚已启动的容器):
  校验 → 分配端口 (两节点) → 确保镜像 (缺失时从平台下发) → 启动 agv-sim (带 NAV_API/HUB_API/模型/场景)
  → 等待仿真就绪 → 启动 agv-nav (带 SIM_API) → 等待执行进程就绪 → (Nav2 就绪) → 运行中
端口先分配后启动，两个容器在启动参数中即获得对方地址。
"""

import json
import threading
import time
import traceback
import urllib.parse
from http.client import HTTPConnection
from typing import Optional

from common.rest import ApiError
from hub.nodes import AgentError, caps
from hub.store import Store

STEPS = [("check", "资源校验"), ("ports", "分配端口"), ("images", "准备镜像"), ("sim", "启动仿真程序"),
         ("sim_ready", "仿真引擎就绪"), ("nav", "启动运行程序"), ("nav_ready", "执行进程就绪"), ("nav2", "Nav2 就绪")]
ACTIVE = ("deploying", "running", "degraded", "stopping")


def http_json(url: str, timeout: float = 3.0, method: str = "GET", body=None):
    u = urllib.parse.urlparse(url)
    c = HTTPConnection(u.hostname, u.port or 80, timeout=timeout)
    try:
        c.request(method, u.path + ("?" + u.query if u.query else ""), body=json.dumps(body).encode() if body is not None else None,
                  headers={"Content-Type": "application/json"})
        r = c.getresponse()
        raw = r.read()
        if r.status >= 400:
            raise ApiError(r.status, raw[:200].decode("utf-8", "replace"))
        return json.loads(raw.decode() or "{}")
    finally:
        c.close()


class Deployer:
    def __init__(self, store: Store, nodes, packages, models, scenes, hub_port: int):
        self.s, self.nodes, self.pk, self.models, self.scenes = store, nodes, packages, models, scenes
        self.hub_port = hub_port
        self.lock = threading.RLock()
        threading.Thread(target=self._monitor, daemon=True, name="inst-monitor").start()

    # ------------------------------------------------------------------ 查询
    def get(self, iid: str) -> dict:
        i = self.s.get("instances", iid)
        if not i:
            raise ApiError(404, f"实例 {iid} 不存在")
        return i

    def list(self, active_only: bool = False) -> list:
        xs = sorted(self.s.list("instances"), key=lambda i: i.get("created", 0), reverse=True)
        if active_only:
            xs = [i for i in xs if i.get("status") in ACTIVE]
        return xs

    def _new_iid(self) -> str:
        n = int(self.s.setting("instance_seq", 0)) + 1
        self.s.set_setting("instance_seq", n)
        return f"i{n:02d}"

    # ------------------------------------------------------------------ 校验 (向导实时调用)
    def check(self, req: dict) -> dict:
        issues, warns = [], []
        sim_node = nav_node = None
        try:
            sim_node = self.nodes.get(req.get("sim_node", ""))
        except ApiError:
            issues.append("未选择仿真节点")
        nav_id = req.get("nav_node") or req.get("sim_node")
        try:
            nav_node = self.nodes.get(nav_id or "")
        except ApiError:
            issues.append("未选择执行节点")
        active = self.list(active_only=True)
        for role, n in (("sim", sim_node), ("nav", nav_node)):
            if not n:
                continue
            if n.get("status") == "offline":
                issues.append(f"节点 {n.get('name')} 离线")
            if not caps(n)[role]:
                issues.append(f"节点 {n.get('name')} 类型为「{n.get('kind')}」，不能运行{'仿真引擎' if role == 'sim' else '运行程序'}")
            used = [i for i in active if n["id"] in (i.get("sim_node"), i.get("nav_node"))]
            if len(used) >= int(n.get("max_instances", 1)) and not (role == "nav" and sim_node and nav_node and sim_node["id"] == nav_node["id"]):
                owners = ", ".join(f"{i['id']}({i.get('operator') or '-'})" for i in used)
                issues.append(f"节点 {n.get('name')} 已被实例 {owners} 占用 (上限 {n.get('max_instances', 1)} 个)")
            info = n.get("info") or {}
            if (info.get("cpu_percent") or 0) > 85:
                warns.append(f"节点 {n.get('name')} CPU 占用 {info['cpu_percent']}%")
        for key, kind, node in (("sim_pkg", "sim", sim_node), ("nav_pkg", "nav", nav_node)):
            pid = req.get(key)
            if not pid:
                issues.append("未选择" + ("仿真程序包" if kind == "sim" else "运行程序包"))
                continue
            try:
                p = self.pk.get(pid)
            except ApiError:
                issues.append(f"程序包 {pid} 不存在")
                continue
            if p["kind"] != kind:
                issues.append(f"程序包 {p.get('version')} 类型不符")
            if node:
                why = self.pk.deployable_on(p, node)
                if why:
                    issues.append(why)
                elif not self._has_image(node, p) and p.get("file"):
                    warns.append(f"节点 {node.get('name')} 首次使用 {p.get('version')}，需要先分发镜像 ({(p.get('size') or 0) / 1e6:.0f} MB)")
        sp, npk = req.get("sim_pkg"), req.get("nav_pkg")
        if sp and npk:
            try:
                a, b = self.pk.get(sp).get("api", "v1"), self.pk.get(npk).get("api", "v1")
                if a != b:
                    issues.append(f"两个程序包协议版本不兼容: 仿真 {a} / 运行 {b}")
            except ApiError:
                pass
        model = scene = None
        try:
            model = self.models.get(req.get("model_id", ""))
            a = self.models.view(model).get("audit", {})
            miss = (a.get("by_severity") or {}).get("missing", 0)
            if miss:
                warns.append(f"车辆模型有 {miss} 项参数缺失，可先到模型详情「补全与传感器安装」补齐")
        except ApiError:
            issues.append("未选择车辆模型")
        try:
            scene = self.scenes.get(req.get("scene_id", ""))
        except ApiError:
            issues.append("未选择仿真场景")
        if model and scene:
            w = (model.get("summary") or {}).get("width_m") or 0
            corridor = (scene.get("summary") or {}).get("min_corridor_m")
            if corridor and w and w + 0.2 > corridor:
                warns.append(f"车宽 {w:.2f} m 接近场景最窄通道 {corridor:.2f} m")
        return {"ok": not issues, "issues": issues, "warnings": warns,
                "split": bool(sim_node and nav_node and sim_node["id"] != nav_node["id"])}

    @staticmethod
    def _has_image(node: dict, p: dict) -> bool:
        from hub.packages import node_has_image
        return node_has_image(node, p) or p.get("source") == "process"

    # ------------------------------------------------------------------ 部署
    def deploy(self, req: dict, operator: str = "") -> dict:
        chk = self.check(req)
        if not chk["ok"]:
            raise ApiError(409, "；".join(chk["issues"]), "check_failed")
        iid = self._new_iid()
        m = self.models.get(req["model_id"])
        sc = self.scenes.get(req["scene_id"])
        inst = {"id": iid, "name": req.get("name") or f"{m.get('name')} @ {sc.get('name')}", "operator": operator or req.get("operator", ""),
                "sim_node": req["sim_node"], "nav_node": req.get("nav_node") or req["sim_node"],
                "model_id": m["id"], "model_ver": req.get("model_ver") or m.get("latest"), "model_name": m.get("name"),
                "scene_id": sc["id"], "scene_name": sc.get("name"), "sim_pkg": req["sim_pkg"], "nav_pkg": req["nav_pkg"],
                "options": req.get("options") or {}, "status": "deploying", "created": time.time(), "started": None,
                "steps": [{"key": k, "label": l, "status": "pending", "msg": ""} for k, l in STEPS], "warnings": chk["warnings"]}
        self.s.put("instances", inst)
        threading.Thread(target=self._run_deploy, args=(iid,), daemon=True, name=f"deploy-{iid}").start()
        return inst

    def _step(self, iid: str, key: str, status: str, msg: str = "", **kw):
        with self.lock:
            i = self.get(iid)
            for st in i["steps"]:
                if st["key"] == key:
                    if status == "running" and st["status"] != "running":
                        st["t0"] = time.time()
                    if status in ("done", "error", "skipped"):
                        st["t1"] = time.time()
                    st.update(status=status, msg=msg, **kw)
            self.s.put("instances", i)

    def _set(self, iid: str, **fields):
        with self.lock:
            return self.s.update("instances", iid, **fields)

    def _run_deploy(self, iid: str):
        i = self.get(iid)
        started = []
        try:
            self._step(iid, "check", "running")
            sim_n, nav_n = self.nodes.get(i["sim_node"]), self.nodes.get(i["nav_node"])
            ag_s, ag_n = self.nodes.client(sim_n), self.nodes.client(nav_n)
            sim_p, nav_p = self.pk.get(i["sim_pkg"]), self.pk.get(i["nav_pkg"])
            ag_s.call("GET", "/api/v1/health", timeout=5)
            ag_n.call("GET", "/api/v1/health", timeout=5)
            self._step(iid, "check", "done", f"仿真 {sim_n['name']} · 执行 {nav_n['name']}")

            # ---- 端口
            self._step(iid, "ports", "running")
            same = sim_n["id"] == nav_n["id"]
            if same:
                ports = ag_s.call("POST", "/api/v1/ports/allocate", {"instance": iid, "names": ["sim_api", "web", "nav_api"]})["ports"]
            else:
                ports = ag_s.call("POST", "/api/v1/ports/allocate", {"instance": iid, "names": ["sim_api", "web"]})["ports"]
                ports.update(ag_n.call("POST", "/api/v1/ports/allocate", {"instance": iid, "names": ["nav_api"]})["ports"])
            dom = self._ros_domain(nav_n["id"], iid)
            hs, hn = sim_n["host"], nav_n["host"]
            urls = {"sim_api": f"http://{hs}:{ports['sim_api']}", "web": f"http://{hs}:{ports['web']}",
                    "nav_api": f"http://{hn}:{ports['nav_api']}"}
            self._set(iid, ports=ports, urls=urls, ros_domain=dom)
            self._step(iid, "ports", "done", f"仿真 {ports['sim_api']} · Web {ports['web']} · 执行 {ports['nav_api']}")

            # ---- 镜像
            self._step(iid, "images", "running")
            for node, ag, p in ((sim_n, ag_s, sim_p), (nav_n, ag_n, nav_p)):
                self._ensure_image(iid, node, ag, p)
            self._step(iid, "images", "done", f"{sim_p.get('version')} / {nav_p.get('version')}")

            # ---- 仿真
            self._step(iid, "sim", "running")
            nav_host = self.nodes.addr_between(sim_n, nav_n)
            sim_host_for_nav = self.nodes.addr_between(nav_n, sim_n)
            hub_url = sim_n.get("hub_url") or f"http://127.0.0.1:{self.hub_port}"
            opts = i.get("options") or {}
            env = {"SIM_API_PORT": ports["sim_api"], "WEB_PORT": ports["web"], "NAV_API": f"http://{nav_host}:{ports['nav_api']}",
                   "HUB_API": hub_url, "INSTANCE_ID": iid, "MODEL_ID": i["model_id"], "MODEL_VER": i["model_ver"],
                   "SCENE_ID": i["scene_id"], "INSTANCE_NAME": i["name"], "SIM_CAMERA_RENDER": opts.get("camera_render", "ray")}
            cname_s, cname_n = f"agv-sim-{iid}", f"agv-nav-{iid}"
            ag_s.call("POST", "/api/v1/containers/run", {"name": cname_s, "image": sim_p["image_ref"], "role": "sim", "instance": iid,
                                                          "env": {k: str(v) for k, v in env.items()}}, timeout=60)
            started.append((ag_s, cname_s))
            self._set(iid, containers={"sim": cname_s, "nav": cname_n})
            self._step(iid, "sim", "done", cname_s)

            self._step(iid, "sim_ready", "running")
            self._wait(iid, "sim_ready", urls["sim_api"] + "/api/v1/health", 120, ag_s, cname_s)
            self._wait(iid, "sim_ready", urls["web"] + "/api/v2/info", 60, ag_s, cname_s)
            self._step(iid, "sim_ready", "done", "模型/场景已加载")

            # ---- 执行
            self._step(iid, "nav", "running")
            env_n = {"NAV_API_PORT": ports["nav_api"], "SIM_API": f"http://{sim_host_for_nav}:{ports['sim_api']}",
                     "ROS_DOMAIN_ID": dom, "INSTANCE_ID": iid, "NAV_USE_ROS": "0" if opts.get("no_ros") else "1",
                     "NAV_LOCALIZATION": opts.get("localization", "slam")}
            ag_n.call("POST", "/api/v1/containers/run", {"name": cname_n, "image": nav_p["image_ref"], "role": "nav", "instance": iid,
                                                          "env": {k: str(v) for k, v in env_n.items()}}, timeout=60)   # /data: SLAM 地图
            started.append((ag_n, cname_n))
            self._step(iid, "nav", "done", cname_n)
            self._step(iid, "nav_ready", "running")
            h = self._wait(iid, "nav_ready", urls["nav_api"] + "/api/v1/health", 180, ag_n, cname_n)
            self._step(iid, "nav_ready", "done", "已连接仿真进程" if h.get("sim_link") else "")

            self._step(iid, "nav2", "running")
            if not h.get("ros"):
                self._step(iid, "nav2", "skipped", "未启用 ROS 2 (内置导引)")
            else:
                t0 = time.time()
                ok = False
                while time.time() - t0 < 150:
                    try:
                        if http_json(urls["nav_api"] + "/api/v1/health").get("nav2_ready"):
                            ok = True
                            break
                    except Exception:
                        pass
                    self._step(iid, "nav2", "running", f"等待 Nav2 生命周期节点 {int(time.time() - t0)} s")
                    time.sleep(2)
                self._step(iid, "nav2", "done" if ok else "skipped", "" if ok else "Nav2 未在 150 s 内就绪，可先用内置规划器")
            self._set(iid, status="running", started=time.time(), error=None)
        except Exception as e:
            msg = str(e)
            if not isinstance(e, (ApiError, AgentError)):
                traceback.print_exc()
            with self.lock:
                i = self.get(iid)
                for st in i["steps"]:
                    if st["status"] == "running":
                        st.update(status="error", msg=msg, t1=time.time())
                i.update(status="error", error=msg, ended=time.time())
                self.s.put("instances", i)
            for ag, name in started:
                try:
                    ag.call("DELETE", f"/api/v1/containers/{name}", timeout=40)
                except Exception:
                    pass

    def _ros_domain(self, node_id: str, iid: str) -> int:
        used = {i.get("ros_domain") for i in self.list(active_only=True) if i.get("nav_node") == node_id and i["id"] != iid}
        d = 10
        while d in used:
            d += 1
        return d

    def _ensure_image(self, iid: str, node: dict, ag, p: dict):
        if self._has_image(node, p):
            return
        url = None
        if p.get("file"):
            hub = node.get("hub_url") or f"http://127.0.0.1:{self.hub_port}"
            url = f"{hub}/api/hub/packages/{p['id']}/image"
        job = ag.call("POST", "/api/v1/images/ensure", {"ref": p["image_ref"], "package_url": url})
        while job.get("status") == "running":
            time.sleep(1.0)
            job = ag.call("GET", f"/api/v1/jobs/{job['id']}")
            tot = job.get("total") or p.get("size") or 0
            pct = round(100.0 * job.get("bytes", 0) / tot) if tot else None
            self._step(iid, "images", "running", f"{node.get('name')}: 导入 {p.get('version')} "
                                                 f"{job.get('bytes', 0) / 1e6:.0f}/{tot / 1e6:.0f} MB", progress=pct)
        if job.get("status") != "done":
            raise AgentError(f"节点 {node.get('name')} 导入镜像失败: {job.get('message')}")

    def _wait(self, iid, key, url, timeout, ag, cname) -> dict:
        t0 = time.time()
        last = ""
        while time.time() - t0 < timeout:
            try:
                return http_json(url, timeout=3)
            except Exception as e:
                last = str(e)
            try:
                st = ag.call("GET", f"/api/v1/containers/{cname}", timeout=5)
                if not st.get("running") and (st.get("exit_code") not in (None, 0) or st.get("state") in ("exited", "dead")):
                    logs = ag.call("GET", f"/api/v1/containers/{cname}/logs?tail=15", timeout=8).get("logs", "")
                    raise AgentError(f"{cname} 已退出 (exit {st.get('exit_code')})\n{logs[-800:]}")
            except AgentError:
                raise
            except Exception:
                pass
            self._step(iid, key, "running", f"等待 {int(time.time() - t0)} s")
            time.sleep(1.5)
        raise AgentError(f"{url} 在 {timeout} s 内未就绪: {last}")

    # ------------------------------------------------------------------ 终止/重启
    def stop(self, iid: str, remove_record: bool = False) -> dict:
        i = self.get(iid)
        if i.get("external"):
            self._set(iid, status="stopped", ended=time.time())
            return self.get(iid)
        self._set(iid, status="stopping")
        self._archive_records(i)
        errs = []
        for role, nid in (("nav", i.get("nav_node")), ("sim", i.get("sim_node"))):
            name = (i.get("containers") or {}).get(role)
            if not name or not nid:
                continue
            try:
                ag = self.nodes.client(nid)
                ag.call("DELETE", f"/api/v1/containers/{name}", timeout=40)
            except Exception as e:
                errs.append(str(e))
        for nid in {i.get("sim_node"), i.get("nav_node")}:
            try:
                self.nodes.client(nid).call("DELETE", f"/api/v1/instances/{iid}", timeout=10)
            except Exception:
                pass
        self._set(iid, status="stopped", ended=time.time(), error="；".join(errs) or None)
        return self.get(iid)

    def restart(self, iid: str) -> dict:
        i = self.get(iid)
        if i.get("external"):
            raise ApiError(400, "外部接入的实例不能由平台重启")
        for role, nid in (("nav", i.get("nav_node")), ("sim", i.get("sim_node"))):
            name = (i.get("containers") or {}).get(role)
            if name:
                try:
                    self.nodes.client(nid).call("DELETE", f"/api/v1/containers/{name}", timeout=40)
                except Exception:
                    pass
        i.update(status="deploying", error=None, steps=[{"key": k, "label": l, "status": "pending", "msg": ""} for k, l in STEPS])
        self.s.put("instances", i)
        threading.Thread(target=self._run_deploy, args=(iid,), daemon=True).start()
        return i

    def delete(self, iid: str):
        i = self.get(iid)
        if i.get("status") in ACTIVE and not i.get("external"):
            raise ApiError(409, "请先终止仿真")
        self.s.delete("instances", iid)

    def attach(self, body: dict, operator: str) -> dict:
        """接入未经平台部署的现有实例 (例如 deploy.sh 直接启动的)"""
        web = (body.get("web_url") or "").rstrip("/")
        if not web.startswith("http"):
            raise ApiError(400, "需要 Web 网关地址，例如 http://192.168.1.10:8088")
        try:
            info = http_json(web + "/api/v2/info", timeout=4)
        except Exception:
            try:
                http_json(web + "/api/telemetry", timeout=4)
                info = {}
            except Exception as e:
                raise ApiError(400, f"无法访问 {web}: {e}")
        iid = self._new_iid()
        inst = {"id": iid, "name": body.get("name") or f"外部实例 {web}", "external": True, "operator": operator,
                "urls": {"web": web, "sim_api": info.get("sim_api"), "nav_api": info.get("nav_api")}, "status": "running",
                "created": time.time(), "started": time.time(), "model_name": (info.get("model") or {}).get("name"),
                "scene_name": (info.get("scene") or {}).get("name"), "steps": []}
        return self.s.put("instances", inst)

    def logs(self, iid: str, svc: str, tail: int = 300) -> dict:
        i = self.get(iid)
        name = (i.get("containers") or {}).get(svc)
        nid = i.get("sim_node") if svc == "sim" else i.get("nav_node")
        if not name or not nid:
            raise ApiError(404, "该实例没有此服务容器")
        return self.nodes.client(nid).call("GET", f"/api/v1/containers/{name}/logs?tail={int(tail)}", timeout=15)

    # ------------------------------------------------------------------ 记录归档
    def _archive_records(self, i: dict):
        web = (i.get("urls") or {}).get("web")
        if not web:
            return
        try:
            http_json(web + "/api/v2/records/archive", timeout=10, method="POST", body={})
        except Exception:
            pass

    # ------------------------------------------------------------------ 监控
    def _monitor(self):
        while True:
            time.sleep(4)
            for i in self.list(active_only=True):
                if i.get("status") not in ("running", "degraded"):
                    continue
                try:
                    self._probe(i)
                except Exception as e:
                    print(f"[hub] 监控 {i['id']} 失败: {e}", flush=True)

    def _probe(self, i: dict):
        urls = i.get("urls") or {}
        web_ok = sim_ok = nav_ok = False
        brief = None
        try:
            brief = http_json(urls["web"] + "/api/v2/brief", timeout=2.5)
            web_ok = True
            sim_ok = brief.get("sim_online", True)
            nav_ok = brief.get("nav_online", True)
        except Exception:
            try:
                http_json(urls["web"] + "/api/telemetry", timeout=2.5)
                web_ok = sim_ok = nav_ok = True
            except Exception:
                pass
        st = "running" if (web_ok and sim_ok and nav_ok) else "degraded"
        health = {"web": web_ok, "sim": sim_ok, "nav": nav_ok, "t": time.time()}
        with self.lock:
            cur = self.s.get("instances", i["id"])
            if cur and cur.get("status") in ("running", "degraded"):
                cur.update(status=st, health=health)
                if brief:
                    cur["brief"] = brief
                self.s.put("instances", cur)
