#!/usr/bin/env python3
"""
agv-hub —— AMR 仿真资源管理平台 (默认 :8080)

  资源池: 计算节点 (节点代理注册/心跳)、车辆模型 (.cmodel)、仿真场景 (场景包)、软件程序包 (Docker 镜像)
  部署:   新建仿真 (仿真节点 + 执行节点，可分离) → 实例监控/重启/终止/日志
  工作台: /inst/<id>/*  反向代理到实例的 Web 网关 (长轮询/二进制帧原样透传)
  前端:   /  (web/ 目录)，模型补全页 /model-editor?api=...

环境变量: HUB_PORT=8080  HUB_DATA=~/.agv-hub (容器内 /data/hub)
"""

import json
import mimetypes
import os
import sys
import threading
import time
import urllib.parse
from http.client import HTTPConnection

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from common.rest import ApiError, BinaryBody, FileBody, RawBody, RestServer  # noqa: E402
from hub.deployer import Deployer  # noqa: E402
from hub.models_repo import ModelRepo  # noqa: E402
from hub.nodes import KIND_LABEL, NodeRegistry  # noqa: E402
from hub.packages import TAGS, PackageRepo  # noqa: E402
from hub.scenes_repo import SceneRepo  # noqa: E402
from hub.store import Store, new_id  # noqa: E402

HUB_VERSION = "1.0"
WEB_DIR = os.path.join(ROOT, "web")
mimetypes.add_type("application/javascript", ".js")
mimetypes.add_type("text/css", ".css")


def log(m):
    print(f"[hub] {m}", flush=True)


class Hub:
    def __init__(self, data: str, port: int):
        self.port = port
        self.s = Store(data)
        self.packages = PackageRepo(self.s)
        self.nodes = NodeRegistry(self.s, self.packages)
        self.models = ModelRepo(self.s)
        self.scenes = SceneRepo(self.s)
        self.scenes.seed_builtins()
        self._seed_models()
        self.dep = Deployer(self.s, self.nodes, self.packages, self.models, self.scenes, port)
        from hub.admin import Admin
        self.admin = Admin(self)

    def _seed_models(self):
        if self.s.list("models"):
            return
        d = os.path.join(ROOT, "tests", "data")
        for dd in (d, os.path.join(ROOT, "defaults", "cmodel")):
            if not os.path.isdir(dd):
                continue
            for f in sorted(os.listdir(dd)):
                if f.lower().endswith(".cmodel"):
                    try:
                        with open(os.path.join(dd, f), "rb") as fh:
                            self.models.add_cmodel(f, fh.read(), {"project": "示例项目", "note": "平台内置示例模型"})
                        log(f"内置示例模型入库: {f}")
                    except Exception as e:
                        log(f"示例模型 {f} 入库失败: {e}")
            break

    # ------------------------------------------------------------------ 概览
    def overview(self) -> dict:
        insts = self.dep.list()
        nodes = [self.nodes.view(n, insts) for n in self.nodes.list()]
        models, scenes, pkgs = self.s.list("models"), self.s.list("scenes"), self.s.list("packages")
        by_kind = {}
        for n in nodes:
            by_kind[n["kind"]] = by_kind.get(n["kind"], 0) + 1
        vt = {}
        for m in models:
            k = (m.get("summary") or {}).get("chassis_label") or "AMR"
            vt[k] = vt.get(k, 0) + 1
        active = [i for i in insts if i.get("status") in ("deploying", "running", "degraded", "stopping")]
        return {"nodes": {"total": len(nodes), "idle": len([n for n in nodes if n["status"] == "idle"]),
                          "online": len([n for n in nodes if n["online"]]), "by_kind": by_kind},
                "models": {"total": len(models), "by_chassis": vt},
                "scenes": {"total": len(scenes), "names": [s.get("name") for s in scenes][:4]},
                "packages": {"total": len(pkgs), "nav": len([p for p in pkgs if p["kind"] == "nav"]),
                             "sim": len([p for p in pkgs if p["kind"] == "sim"])},
                "instances": {"active": len(active), "total": len(insts)}}

    def inst_view(self, i: dict) -> dict:
        v = {k: i.get(k) for k in ("id", "name", "operator", "status", "created", "started", "ended", "error", "steps", "warnings",
                                   "model_id", "model_ver", "model_name", "scene_id", "scene_name", "sim_node", "nav_node",
                                   "sim_pkg", "nav_pkg", "ports", "external", "health", "brief", "ros_domain", "containers")}
        for role in ("sim_node", "nav_node"):
            n = self.s.get("nodes", i.get(role) or "")
            v[role + "_view"] = {"id": n["id"], "name": n.get("name"), "host": NodeRegistry.lan_host(n), "kind": n.get("kind"),
                                 "kind_label": KIND_LABEL.get(n.get("kind"))} if n else None
        for role in ("sim_pkg", "nav_pkg"):
            p = self.s.get("packages", i.get(role) or "")
            v[role + "_view"] = {"id": p["id"], "version": p.get("version"), "image_ref": p.get("image_ref")} if p else None
        v["web_url"] = (i.get("urls") or {}).get("web")
        return v


# ====================================================================== 反向代理 (工作台 → 实例网关)
_tls = threading.local()
HOP = {"connection", "keep-alive", "proxy-authenticate", "proxy-authorization", "te", "trailers", "transfer-encoding", "upgrade",
       "host", "content-length"}


def _conn(host, port, timeout):
    pool = getattr(_tls, "pool", None)
    if pool is None:
        pool = _tls.pool = {}
    c = pool.get((host, port))
    if c is None:
        c = HTTPConnection(host, port, timeout=timeout)
        pool[(host, port)] = c
    else:
        c.timeout = timeout
        if c.sock:
            c.sock.settimeout(timeout)
    return c


def proxy(h, target_base: str, path: str):
    u = urllib.parse.urlparse(target_base)
    n = int(h.headers.get("Content-Length") or 0)
    body = h.rfile.read(n) if n else None
    headers = {k: v for k, v in h.headers.items() if k.lower() not in HOP}
    headers["X-Forwarded-Prefix"] = h.path[:len(h.path) - len(path)] if path else h.path
    wait = 0.0
    q = urllib.parse.urlparse(path).query
    if "wait=" in q:
        try:
            wait = float(urllib.parse.parse_qs(q).get("wait", ["0"])[0])
        except ValueError:
            pass
    for attempt in (0, 1):
        c = _conn(u.hostname, u.port or 80, 15 + wait)
        try:
            c.request(h.command, path or "/", body=body, headers=headers)
            r = c.getresponse()
            data = r.read()
            break
        except Exception as e:
            c.close()
            _tls.pool.pop((u.hostname, u.port or 80), None)
            if attempt == 1:
                raise ApiError(502, f"实例网关不可达 ({target_base}): {e}", "bad_gateway")
    h.send_response(r.status)
    for k, v in r.getheaders():
        if k.lower() not in HOP:
            h.send_header(k, v)
    h.send_header("Content-Length", str(len(data)))
    h.end_headers()
    if h.command != "HEAD":
        h.wfile.write(data)


# ====================================================================== REST
def build_api(hub: Hub) -> RestServer:
    api = RestServer("hub", port=hub.port)
    R = api.route
    P = "/api/hub"

    def user(q):
        return urllib.parse.unquote(q.headers.get("X-User", "") or "")

    R("GET", P + "/health", lambda q: {"service": "hub", "ok": True, "version": HUB_VERSION}, "健康检查")
    R("GET", P + "/overview", lambda q: hub.overview(), "资源总览")

    # ---- 节点
    R("GET", P + "/nodes", lambda q: {"nodes": [hub.nodes.view(n, hub.dep.list()) for n in hub.nodes.list()],
                                      "kinds": KIND_LABEL}, "计算节点")

    def node_get(q):
        return hub.nodes.view(hub.nodes.get(q.params["nid"]), hub.dep.list())
    R("GET", P + "/nodes/{nid}", node_get, "节点详情")
    R("PATCH", P + "/nodes/{nid}", lambda q: hub.nodes.view(hub.nodes.update(q.params["nid"], q.json), hub.dep.list()), "修改节点")
    R("DELETE", P + "/nodes/{nid}", lambda q: hub.nodes.delete(q.params["nid"]) or {"ok": True}, "移除节点")

    def enroll(q):
        t = hub.nodes.new_token(user(q), q.json.get("note", ""))
        host = (q.headers.get("Host") or f"127.0.0.1:{hub.port}").split(",")[0]
        hub_url = f"http://{host}"
        kind = q.json.get("kind", "hybrid")
        return {"token": t["id"], "expires": t["expires"], "hub_url": hub_url,
                "command": f"bash deploy.sh agent --hub {hub_url} --token {t['id']} --kind {kind}",
                "docker": (f"docker run -d --name agv-agent --restart unless-stopped --network host "
                           f"-v /var/run/docker.sock:/var/run/docker.sock -v $HOME/.agv-agent:$HOME/.agv-agent "
                           f"-e AGENT_DATA=$HOME/.agv-agent -e HUB_API={hub_url} -e JOIN_TOKEN={t['id']} -e AGENT_KIND={kind} "
                           f"agv-platform:latest agent")}
    R("POST", P + "/nodes/enroll", enroll, "生成节点接入令牌与命令")

    def add_node(q):
        """SSH 接入: {kind, note, ip, ssh_port, username, password} → 登录目标设备安装基础环境 (基础监测)，等它注册上来"""
        from hub.sshjoin import install_agent, local_ip_towards
        b = q.json
        ip, usr, pw = str(b.get("ip", "")).strip(), str(b.get("username", "")).strip(), str(b.get("password", ""))
        try:
            port = int(b.get("ssh_port") or 22)
        except (TypeError, ValueError):
            port = 0
        kind = b.get("kind") or "hybrid"
        if not ip or any(c in ip for c in " /@:;'\"$`"):
            raise ApiError(400, "请填写正确的节点 IP")
        if not 0 < port < 65536:
            raise ApiError(400, "SSH 端口应为 1–65535")
        if not usr or not pw:
            raise ApiError(400, "请填写 SSH 用户名和密码")
        if kind not in KIND_LABEL:
            raise ApiError(400, "计算资源类型无效")
        me = local_ip_towards(ip, port) or (q.headers.get("Host") or "127.0.0.1").split(":")[0]
        hub_url = f"http://{me}:{hub.port}"
        ssh = {"ip": ip, "port": port, "user": usr}
        t = hub.nodes.new_token(user(q), b.get("note", ""), meta={"kind": kind, "note": b.get("note", ""), "ssh": ssh}, ttl=600)
        brief = install_agent(ip, port, usr, pw, hub_url, t["id"], kind, str(b.get("name", "")).strip())
        for _ in range(40):                      # 代理启动后几秒内注册
            tk = hub.nodes.s.get("tokens", t["id"]) or {}
            if tk.get("node_id"):
                n = hub.nodes.get(tk["node_id"])
                return {"node": hub.nodes.view(n, hub.dep.list()), "agent": brief, "hub_url": hub_url}
            time.sleep(0.5)
        raise ApiError(504, f"基础监测已在 {ip} 启动 ({brief})，但 20 秒内没有注册到平台。请确认该设备能访问 {hub_url}"
                            f" (日志在目标设备 ~/.agv-agent/agent.log)", "agent_no_register")
    R("POST", P + "/nodes/add", add_node, "SSH 接入计算节点 {kind,note,ip,ssh_port,username,password} (密码不保存)")
    R("POST", P + "/nodes/register", lambda q: hub.nodes.register(q.json, (q.client or ("127.0.0.1",))[0]), "节点代理注册")
    R("POST", P + "/nodes/{nid}/heartbeat", lambda q: hub.nodes.heartbeat(q.params["nid"], q.headers.get("X-Node-Key", ""), q.json),
      "节点心跳")
    R("GET", P + "/nodes/{nid}/jobs/{jid}", lambda q: hub.nodes.client(q.params["nid"]).call("GET", f"/api/v1/jobs/{q.params['jid']}"),
      "节点任务进度")

    def probe_link(q):
        a, b = hub.nodes.get(q.json["from"]), hub.nodes.get(q.json["to"])
        url = f"http://{hub.nodes.addr_between(a, b)}:{b.get('api_port', 8070)}/api/v1/health"
        return hub.nodes.client(a).call("POST", "/api/v1/probe", {"url": url})
    R("POST", P + "/nodes/probe", probe_link, "探测两节点之间的连通性 {from,to}")

    # ---- 运维管理 (管理员密码；hub/admin.py)
    A = P + "/admin"
    adm = hub.admin

    def tok(q):
        return q.headers.get("X-Admin-Token") or q.q("token") or ""

    def need(fn):
        def h(q):
            adm.check(tok(q))
            return fn(q)
        return h
    R("GET", A + "/state", lambda q: adm.state(tok(q)), "运维登录状态")
    R("POST", A + "/setup", lambda q: adm.setup(q.json.get("password", "")), "首次设置管理员密码")
    R("POST", A + "/login", lambda q: adm.login(q.json.get("password", "")), "管理员登录")
    R("POST", A + "/logout", lambda q: adm.logout(tok(q)), "退出登录")
    R("POST", A + "/password", need(lambda q: adm.change_pw(q.json.get("old", ""), q.json.get("new", ""))), "修改管理员密码")
    R("GET", A + "/services", need(lambda q: adm.services()), "服务状态")
    R("POST", A + "/restart", need(lambda q: adm.restart(q.json.get("target", ""), user(q))), "重启 {target: hub|agent}")
    R("GET", A + "/logs", need(lambda q: {"logs": adm.logs()}), "日志文件列表")
    R("GET", A + "/logs/{name}", need(lambda q: BinaryBody(adm.log_file(q.params["name"]), {
        "Content-Disposition": f'attachment; filename="{q.params["name"]}"'}, "text/plain; charset=utf-8")), "下载日志")
    R("GET", A + "/updates", need(lambda q: {"updates": adm.updates(), "current": adm.version()}), "更新包列表")
    R("POST", A + "/updates/upload", need(lambda q: adm.upload(q.raw, user(q))), "上传更新包 (请求体为 .tgz)")
    R("GET", A + "/updates/{uid}", need(lambda q: adm.view(q.params["uid"])), "更新包详情与预览")
    R("POST", A + "/updates/{uid}/apply", need(lambda q: adm.apply(q.params["uid"], q.json, user(q))), "应用更新 {restart_instances}")
    R("POST", A + "/updates/{uid}/rollback", need(lambda q: adm.rollback(q.params["uid"], q.json, user(q))), "回滚最近一次更新")
    R("DELETE", A + "/updates/{uid}", need(lambda q: adm.delete_update(q.params["uid"])), "删除更新包")
    R("GET", A + "/jobs", need(lambda q: {"jobs": adm.jobs(), "current": adm.job and adm.job.get("id")}), "运维任务")
    R("GET", A + "/jobs/{jid}", need(lambda q: adm.job_view(q.params["jid"])), "运维任务详情")
    R("GET", A + "/apps", need(lambda q: adm.apps()), "App 列表与安装状态")
    R("POST", A + "/apps/upload", need(lambda q: adm.app_upload(q.raw, q.q("filename", ""), user(q))), "上传 APK (请求体为文件)")
    R("DELETE", A + "/apps/{aid}", need(lambda q: adm.app_delete(q.params["aid"])), "删除 APK")
    R("POST", A + "/apps/{aid}/install", need(lambda q: adm.app_install(q.params["aid"], q.json.get("mode", "prompt"), q.json.get("serial", ""))),
      "安装 APK {mode: prompt|adb, serial}")
    R("POST", A + "/adb/{action}", need(lambda q: adm.adb(q.params["action"], q.json)), "手机无线调试 adb: pair|connect|disconnect|mdns")
    R("POST", A + "/panel", need(lambda q: adm.panel(q.json.get("action", "open"))), "外屏面板 open|close")

    # ---- 车辆模型
    R("GET", P + "/models", lambda q: {"models": hub.models.list()}, "车辆模型")
    R("GET", P + "/models/{mid}", lambda q: dict(hub.models.view(hub.models.get(q.params["mid"])),
                                                 spec=hub.models.spec(q.params["mid"], q.q("ver"))), "模型详情 (含生效参数)")

    def model_upload(q):
        fn = urllib.parse.unquote(q.q("filename", "model.cmodel"))
        meta = {k: urllib.parse.unquote(q.q(k)) for k in ("name", "project", "vtype", "material_no", "note", "version", "model_id")
                if q.q(k) is not None}
        meta["operator"] = user(q)
        return hub.models.add_cmodel(fn, q.raw, meta)
    R("POST", P + "/models/upload", model_upload, "上传 .cmodel (请求体为文件内容，?filename=&name=&project=&vtype=&material_no=)")
    R("PATCH", P + "/models/{mid}", lambda q: hub.models.update_meta(q.params["mid"], q.json), "修改模型信息")
    R("DELETE", P + "/models/{mid}", lambda q: hub.models.delete(q.params["mid"]) or {"ok": True}, "删除模型")
    R("GET", P + "/models/{mid}/cmodel", lambda q: FileBody(hub.models.cmodel_file(q.params["mid"], q.q("ver")),
                                                              filename=hub.models.get(q.params["mid"]).get("file") or "model.cmodel"),
      "下载 cmodel")
    V = P + "/models/{mid}/versions/{ver}"
    R("GET", V + "/bundle", lambda q: hub.models.bundle(q.params["mid"], q.params["ver"]), "实例启动用模型包 (基线+补全)")
    R("GET", V + "/api/v1/model/editor", lambda q: hub.models.editor(q.params["mid"], q.params["ver"]), "补全编辑器数据")
    R("POST", V + "/api/v1/model/preview", lambda q: hub.models.preview(q.params["mid"], q.params["ver"], q.json), "试算补全")
    R("PUT", V + "/api/v1/model/overrides", lambda q: hub.models.save_overrides(q.params["mid"], q.params["ver"], q.json), "保存补全")

    def tmpl(q):
        from model_overrides import sensor_template
        return sensor_template(q.q("type", "lidar2d"))
    R("GET", V + "/api/v1/model/sensor_template", tmpl, "传感器模板")
    R("GET", V + "/api/v1/model/urdf", lambda q: RawBody(hub.models.urdf(q.params["mid"], q.params["ver"]).encode(), "application/xml"),
      "URDF")
    R("GET", V + "/api/v1/model", lambda q: hub.models.spec(q.params["mid"], q.params["ver"]), "生效参数")
    R("GET", V + "/api/v1/sensors/cameras", lambda q: {"cameras": [], "offline": True}, "离线编辑无相机画面")

    # ---- 场景
    R("GET", P + "/scenes", lambda q: {"scenes": hub.scenes.list()}, "仿真场景")
    R("GET", P + "/scenes/{sid}", lambda q: dict(hub.scenes.view(hub.scenes.get(q.params["sid"])),
                                                 scene=hub.scenes.scene_def(q.params["sid"])), "场景详情 (含定义)")

    def scene_upload(q):
        fn = urllib.parse.unquote(q.q("filename", "scene.zip"))
        meta = {k: urllib.parse.unquote(q.q(k)) for k in ("name", "description", "tags", "scene_id") if q.q(k) is not None}
        meta["operator"] = user(q)
        return hub.scenes.import_upload(fn, q.raw, meta)
    R("POST", P + "/scenes/upload", scene_upload, "上传场景包 (.zip / scene.json)")
    R("PATCH", P + "/scenes/{sid}", lambda q: hub.scenes.update_meta(q.params["sid"], q.json), "修改场景信息")
    R("DELETE", P + "/scenes/{sid}", lambda q: hub.scenes.delete(q.params["sid"]) or {"ok": True}, "删除场景")
    R("GET", P + "/scenes/{sid}/definition", lambda q: hub.scenes.scene_def(q.params["sid"]), "场景定义 JSON")

    def scene_map(q):
        pgm, yml = hub.scenes.map_files(q.params["sid"])
        return RawBody(pgm, "image/x-portable-graymap") if q.q("part", "pgm") == "pgm" else RawBody(yml.encode(), "text/yaml")
    R("GET", P + "/scenes/{sid}/map", scene_map, "栅格地图 ?part=pgm|yaml")
    R("GET", P + "/scenes/{sid}/package", lambda q: RawBody(hub.scenes.package_zip(q.params["sid"]), "application/zip", {
        "Content-Disposition": f"attachment; filename*=UTF-8''{urllib.parse.quote(q.params['sid'])}.zip"}), "下载场景包")
    R("GET", P + "/scenes/{sid}/taskflows", lambda q: {"taskflows": hub.scenes.taskflows(q.params["sid"])}, "场景任务流")
    R("PUT", P + "/scenes/{sid}/taskflows", lambda q: {"taskflows": hub.scenes.save_taskflows(q.params["sid"], q.json.get("taskflows"))},
      "保存任务流")

    # ---- 程序包
    R("GET", P + "/packages", lambda q: {"packages": hub.packages.list(q.q("kind")), "tags": TAGS}, "软件程序包")
    R("PATCH", P + "/packages/{pid}", lambda q: hub.packages.update(q.params["pid"], q.json), "修改程序包")
    R("DELETE", P + "/packages/{pid}", lambda q: hub.packages.delete(q.params["pid"]) or {"ok": True}, "删除程序包")

    def pkg_image(q):
        p = hub.packages.get(q.params["pid"])
        f = hub.packages.file(p)
        if not f:
            raise ApiError(404, "该程序包没有镜像文件 (节点镜像需先入库)")
        return FileBody(f, "application/x-tar", filename=f"{p.get('image_ref', p['id']).replace('/', '_').replace(':', '_')}.tar"
                        + (".gz" if f.endswith(".gz") else ""))
    R("GET", P + "/packages/{pid}/image", pkg_image, "下载镜像包 (节点代理导入用)")

    def from_node(q):
        b = q.json
        n = hub.nodes.get(b["node_id"])
        hub_url = n.get("hub_url") or f"http://127.0.0.1:{hub.port}"
        qs = urllib.parse.urlencode({k: b[k] for k in ("kind", "version", "note", "tag") if b.get(k)} |
                                    {"ref": b["ref"], "from_node": n["id"], "operator": user(q)})
        job = hub.nodes.client(n).call("POST", "/api/v1/images/export", {"ref": b["ref"], "upload_url": f"{hub_url}{P}/packages/upload?{qs}"})
        return {"node_id": n["id"], "job": job}
    R("POST", P + "/packages/from_node", from_node, "节点镜像入库 (导出上传到平台) {node_id, ref, kind, version, note}")

    def upload_mount(h):
        if h.command != "POST" or not h.path.startswith(P + "/packages/upload"):
            return False
        qs = urllib.parse.parse_qs(urllib.parse.urlparse(h.path).query)
        meta = {k: v[0] for k, v in qs.items()}
        meta.setdefault("operator", urllib.parse.unquote(h.headers.get("X-User", "") or ""))
        doc = hub.packages.receive(h, meta)
        h.send_json(200, doc)
    api.mount(P + "/packages/upload", upload_mount)

    # ---- 实例
    R("POST", P + "/deployments/check", lambda q: hub.dep.check(q.json), "部署前校验")
    R("GET", P + "/instances", lambda q: {"instances": [hub.inst_view(i) for i in hub.dep.list(q.q("active", False, bool))]}, "仿真实例")
    R("POST", P + "/instances", lambda q: hub.inst_view(hub.dep.deploy(q.json, user(q))), "部署新实例")
    R("POST", P + "/instances/attach", lambda q: hub.inst_view(hub.dep.attach(q.json, user(q))), "接入外部实例 {name, web_url}")
    R("GET", P + "/instances/{iid}", lambda q: hub.inst_view(hub.dep.get(q.params["iid"])), "实例详情")
    R("POST", P + "/instances/{iid}/stop", lambda q: hub.inst_view(hub.dep.stop(q.params["iid"])), "终止仿真")
    R("POST", P + "/instances/{iid}/restart", lambda q: hub.inst_view(hub.dep.restart(q.params["iid"])), "重启实例")
    R("DELETE", P + "/instances/{iid}", lambda q: hub.dep.delete(q.params["iid"]) or {"ok": True}, "删除实例记录")
    R("GET", P + "/instances/{iid}/logs", lambda q: hub.dep.logs(q.params["iid"], q.q("svc", "sim"), q.q("tail", 300, int)), "容器日志")

    def logs_file(q):
        """下载实例日志文件 (默认仿真引擎 agv-sim，最近 20000 行)"""
        iid, svc = q.params["iid"], q.q("svc", "sim")
        r = hub.dep.logs(iid, svc, q.q("tail", 20000, int))
        fn = f"{iid}_{'sim-engine' if svc == 'sim' else 'nav'}_{time.strftime('%Y%m%d-%H%M%S')}.log"
        return BinaryBody((r.get("logs") or "").encode("utf-8"), {"Content-Disposition": f'attachment; filename="{fn}"'},
                          "text/plain; charset=utf-8")
    R("GET", P + "/instances/{iid}/logs/file", logs_file, "下载实例日志文件 ?svc=sim|nav&tail=20000")

    # ---- 仿真记录 (实例归档)
    def rec_post(q):
        b = q.json
        meta = b.get("metadata") or {}
        rid = f"{meta.get('instanceId', 'x')}-{meta.get('recordId') or meta.get('taskId') or new_id('r')}"
        path = hub.s.path("records", f"{rid}.json")
        hub.s.write_json(path, b)
        doc = {"id": rid, "instance_id": meta.get("instanceId"), "task_id": meta.get("taskId"), "task_name": meta.get("taskName"),
               "recorded_at": meta.get("recordedAt"), "duration": meta.get("durationSeconds"), "result": meta.get("result"),
               "scene": (b.get("environment") or {}).get("sceneName"), "model": (b.get("vehicleModel") or {}).get("modelName"),
               "injections": len(b.get("injectedElements") or []), "operator": meta.get("operator")}
        return hub.s.put("records", doc)
    R("POST", P + "/records", rec_post, "归档仿真记录 (sim_bundle)")
    R("GET", P + "/records", lambda q: {"records": [r for r in hub.s.list("records")
                                                    if not q.q("instance") or r.get("instance_id") == q.q("instance")]}, "仿真记录")

    def rec_get(q):
        f = hub.s.path("records", f"{q.params['rid']}.json")
        if not os.path.exists(f):
            raise ApiError(404, "记录不存在")
        return FileBody(f, "application/json", filename=f"sim_bundle_{q.params['rid']}.json")
    R("GET", P + "/records/{rid}", rec_get, "下载记录 sim_bundle")
    R("DELETE", P + "/records/{rid}", lambda q: hub.s.delete("records", q.params["rid"]) or {"ok": True}, "删除记录")

    # ---- 工作台代理
    def inst_mount(h):
        rest = h.path[len("/inst/"):]
        iid, _, sub = rest.partition("/")
        i = hub.s.get("instances", iid)
        if not i or not (i.get("urls") or {}).get("web"):
            raise ApiError(404, f"实例 {iid} 不存在或尚未分配地址")
        proxy(h, i["urls"]["web"], "/" + sub)
    api.mount("/inst/", inst_mount)

    # ---- 静态前端
    def static(q):
        p = q.path
        if p in ("/", "/index.html"):
            f = os.path.join(WEB_DIR, "index.html")
        elif p.startswith("/vendor/"):
            f = os.path.join(ROOT, "vendor", os.path.normpath(p[len("/vendor/"):]).lstrip("./"))
        elif p in ("/model-editor", "/model-editor.html"):
            f = os.path.join(ROOT, "model_editor.html")
        else:
            f = os.path.join(WEB_DIR, os.path.normpath(p.lstrip("/")))
        f = os.path.abspath(f)
        if not (f.startswith(WEB_DIR) or f.startswith(os.path.join(ROOT, "vendor")) or f.endswith("model_editor.html")) \
                or not os.path.isfile(f):
            raise ApiError(404, f"未找到 {p}")
        ct = mimetypes.guess_type(f)[0] or "application/octet-stream"
        if ct.startswith("text/") or ct == "application/javascript":
            ct += "; charset=utf-8"
        with open(f, "rb") as fh:
            return RawBody(fh.read(), ct, {"Cache-Control": "no-cache"})
    api.fallback = static
    return api


def main():
    port = int(os.environ.get("HUB_PORT", "8080"))
    data = os.environ.get("HUB_DATA", "~/.agv-hub")
    hub = Hub(data, port)
    api = build_api(hub)
    try:   # 本机节点代理用集群令牌自动接入 (deploy.sh hub 读取)
        with open(hub.s.path("cluster_token"), "w") as f:
            f.write(hub.s.setting("cluster_token"))
    except OSError:
        pass
    log(f"AMR 仿真资源管理平台 :{port}  数据 {hub.s.root}  集群令牌 {hub.s.setting('cluster_token')}")
    api.serve_forever()


if __name__ == "__main__":
    main()
