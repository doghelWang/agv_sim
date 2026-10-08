#!/usr/bin/env python3
"""
运维管理 (平台 → 「运维管理」页，需要管理员密码)

  平台更新  上传更新包 (tools/make_update.py 生成) → 校验 → 预览改动与影响 → 应用:
            备份被替换的文件 → 写入 → 需要时编译 ROS 插件 → 重启受影响的服务 (实例 / 节点代理 / 平台自身)
            每次应用都留备份，可以回滚最近一次。设备本地的数据与配置文件 (PROTECTED) 永远不动。
  App 管理  上传 APK (解析包名/版本) → 通过「安卓助手」(Termux 原生小服务 deploy/android/android_helper.py) 安装:
            prompt = 手机上弹出系统安装框，点一次「安装」；adb = 手机自连无线调试后静默安装
  服务管理  平台 / 节点代理 / 实例 / 安卓助手 / 外屏面板的状态与重启；下载各进程日志

只适用于「源码部署」(平台与节点代理直接跑在本机代码目录 ROOT 上，如手机)。Docker 部署用「软件程序包」更新镜像。
"""

import base64
import hashlib
import hmac
import io
import json
import os
import secrets
import shutil
import socket
import subprocess
import sys
import tarfile
import threading
import time
import urllib.parse
import zipfile
from http.client import HTTPConnection
from typing import Optional

from common.rest import ApiError

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FORMAT = "agv-update/1"
MANIFEST = "agv_update.json"
VERSION_FILE = ".agv_version"
# 设备本地的数据与配置，更新包里即使带了也不覆盖 (与 deploy/android/update_from_git.sh 一致)
PROTECTED = ("data/", "records/", "model_overrides.json", "robot_config.json", "sensor_overrides.json", "robot.urdf",
             ".git/", VERSION_FILE)
# Termux 主目录里的脚本: 更新包里的 deploy/android/<名字> 同时复制一份到 Termux 主目录 (下次启动生效)
TERMUX_HOME = os.environ.get("AGV_TERMUX_HOME", "/data/data/com.termux/files/home")
TERMUX_SCRIPTS = ("start_agv.sh", "stop_agv.sh", "status_agv.sh", "agv_common.sh", "keep_front.sh", "proot_spawner.py",
                  "update_from_git.sh", "android_helper.py")
TERMUX_LOGS = ("hub.log", "agent.log", "agv_boot.log", "autostart.log", "spawner.log", "agv_front.log", "gpucastd.log",
               "android_helper.log")
HELPER_URL = os.environ.get("AGV_HELPER_URL", "http://127.0.0.1:8067")
NO_EFFECT = ("docs/", "tests/", "tools/", "phone_snapshot/", "README", ".github/", "deploy/android/cover_app/")
MAX_UPDATE = 200 << 20
MAX_APK = 200 << 20


def log(m):
    print(f"[hub][admin] {m}", flush=True)


def sha256(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()


def safe_rel(p: str) -> str:
    p = (p or "").replace("\\", "/").lstrip("/")
    parts = [x for x in p.split("/") if x not in ("", ".")]
    if not parts or ".." in parts:
        raise ApiError(400, f"更新包里有非法路径: {p!r}")
    return "/".join(parts)


def protected(rel: str) -> bool:
    return any(rel == x.rstrip("/") or rel.startswith(x) for x in PROTECTED)


def components(rel: str) -> set:
    """一个文件改动会影响哪些服务"""
    if rel.startswith(NO_EFFECT) or rel.endswith(".md"):
        return set()
    if rel.startswith("web/") or rel in ("index.html", "model_editor.html"):
        return {"web"}
    if rel.startswith("hub/"):
        return {"hub"}
    if rel.startswith("agent/"):
        return {"agent"}
    if rel.startswith("common/"):
        return {"hub", "agent", "instances"}
    if rel.startswith("ros2/"):
        parts = rel.split("/")
        return {"instances", "build:" + parts[1]} if len(parts) > 2 else {"instances"}
    if rel.startswith("deploy/android/"):
        return {"termux"} if rel.split("/")[-1] in TERMUX_SCRIPTS else set()
    if rel.startswith("deploy/") or rel.startswith("docker/"):
        return set()
    return {"instances"}


COMP_LABEL = {"web": "网页 (刷新浏览器即可)", "hub": "平台 (自动重启，约 10 秒不可用)", "agent": "节点代理 (运行中的实例会先停止，代理重启后重新部署)",
              "instances": "运行中的仿真实例 (重新部署)", "termux": "Termux 启动脚本 (下次启动平台时生效)"}


# ====================================================================== APK 信息 (解析二进制 AndroidManifest.xml)
def apk_info(data: bytes) -> dict:
    try:
        z = zipfile.ZipFile(io.BytesIO(data))
        axml = z.read("AndroidManifest.xml")
    except Exception:
        raise ApiError(400, "不是有效的 APK 文件")
    info = {"package": None, "versionCode": None, "versionName": None, "label": None}
    try:
        import struct
        pos, strings = 8, []
        while pos < len(axml) - 8:
            typ, hsz, sz = struct.unpack_from("<HHI", axml, pos)
            if sz <= 0:
                break
            if typ == 0x0001:                                        # 字符串池
                cnt, _sty, flags, sstart = struct.unpack_from("<IIII", axml, pos + 8)
                utf8 = flags & 0x100
                offs = struct.unpack_from(f"<{cnt}I", axml, pos + hsz)
                base = pos + sstart
                for o in offs:
                    p = base + o
                    if utf8:
                        n = axml[p]; p += 2 if n & 0x80 else 1
                        n = axml[p]; p += 1
                        if n & 0x80:
                            n = ((n & 0x7F) << 8) | axml[p]; p += 1
                        strings.append(axml[p:p + n].decode("utf-8", "replace"))
                    else:
                        n = struct.unpack_from("<H", axml, p)[0]; p += 2
                        if n & 0x8000:
                            n = ((n & 0x7FFF) << 16) | struct.unpack_from("<H", axml, p)[0]; p += 2
                        strings.append(axml[p:p + n * 2].decode("utf-16-le", "replace"))
            elif typ == 0x0102:                                      # 元素开始
                name_i = struct.unpack_from("<I", axml, pos + 20)[0]
                tag = strings[name_i] if name_i < len(strings) else ""
                acnt = struct.unpack_from("<H", axml, pos + 28)[0]
                for k in range(acnt):
                    a = pos + 36 + k * 20
                    _ns, an, raw, _s, _r, dtype, dval = struct.unpack_from("<IIIHBBI", axml, a)
                    aname = strings[an] if an < len(strings) else ""
                    val = strings[raw] if raw != 0xFFFFFFFF and raw < len(strings) else (dval if dtype in (0x10, 0x11) else None)
                    if tag == "manifest" and aname in ("package", "versionCode", "versionName"):
                        info[aname] = val
                    if tag == "application" and aname == "label" and isinstance(val, str):
                        info["label"] = val
                if tag == "application":
                    break
            pos += sz
    except Exception as e:
        log(f"APK 清单解析不完整: {e}")
    if not info["package"]:
        raise ApiError(400, "无法从 APK 里读出包名")
    return info


# ====================================================================== 管理员
class Admin:
    def __init__(self, hub):
        self.hub = hub
        self.s = hub.s
        self.dir = self.s.path("admin")
        for d in ("updates", "backups", "jobs", "apps"):
            os.makedirs(os.path.join(self.dir, d), exist_ok=True)
        self.tokens = self._load_tokens()  # sha256(token) -> 过期时间；存盘，平台重启 (更新后) 不用重新登录
        self.lock = threading.Lock()
        self.job: Optional[dict] = None  # 正在执行的任务
        self.started = time.time()

    # ------------------------------------------------------------------ 鉴权
    def _tok_path(self):
        return os.path.join(self.dir, "tokens.json")

    def _load_tokens(self) -> dict:
        try:
            with open(self._tok_path(), encoding="utf-8") as f:
                return {k: v for k, v in json.load(f).items() if v > time.time()}
        except (OSError, ValueError):
            return {}

    def _save_tokens(self):
        p = self._tok_path()
        fd = os.open(p + ".tmp", os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, "w") as f:
            json.dump({k: v for k, v in self.tokens.items() if v > time.time()}, f)
        os.replace(p + ".tmp", p)
    def _hash(self, pw: str, salt: bytes) -> str:
        return base64.b64encode(hashlib.pbkdf2_hmac("sha256", pw.encode(), salt, 120000)).decode()

    def state(self, token: str = "") -> dict:
        return {"configured": bool(self.s.setting("admin_pw")), "authed": self.check(token, raise_=False)}

    def setup(self, pw: str) -> dict:
        if self.s.setting("admin_pw"):
            raise ApiError(409, "管理员密码已设置，请直接登录")
        return self._set_pw(pw)

    def _set_pw(self, pw: str) -> dict:
        if len(pw or "") < 6:
            raise ApiError(400, "密码至少 6 位")
        salt = secrets.token_bytes(16)
        self.s.set_setting("admin_pw", json.dumps({"salt": base64.b64encode(salt).decode(), "hash": self._hash(pw, salt)}))
        self.tokens.clear()
        self._save_tokens()
        return self._issue()

    def login(self, pw: str) -> dict:
        rec = self.s.setting("admin_pw")
        if not rec:
            raise ApiError(409, "尚未设置管理员密码")
        rec = json.loads(rec)
        if not hmac.compare_digest(self._hash(pw or "", base64.b64decode(rec["salt"])), rec["hash"]):
            time.sleep(1.0)
            raise ApiError(401, "密码错误", "bad_password")
        return self._issue()

    def change_pw(self, old: str, new: str) -> dict:
        self.login(old)
        return self._set_pw(new)

    def _issue(self) -> dict:
        t = secrets.token_urlsafe(24)
        exp = time.time() + 12 * 3600
        self.tokens[sha256(t.encode())] = exp
        self._save_tokens()
        return {"token": t, "expires": exp}

    def check(self, token: str, raise_: bool = True) -> bool:
        ok = bool(token) and self.tokens.get(sha256(token.encode()), 0) > time.time()
        if not ok and raise_:
            raise ApiError(401, "请先登录运维管理", "admin_auth")
        return ok

    def logout(self, token: str):
        self.tokens.pop(sha256((token or "").encode()), None)
        self._save_tokens()
        return {"ok": True}

    # ------------------------------------------------------------------ 版本
    @staticmethod
    def version() -> dict:
        try:
            with open(os.path.join(ROOT, VERSION_FILE), encoding="utf-8") as f:
                line = f.readline().strip()
        except OSError:
            line = ""
        v, _, t = line.partition(" ")
        return {"version": v or "unknown", "title": t}

    # ------------------------------------------------------------------ 更新包
    def _meta_path(self, uid):
        return os.path.join(self.dir, "updates", uid, "meta.json")

    def _load(self, uid) -> dict:
        try:
            with open(self._meta_path(uid), encoding="utf-8") as f:
                return json.load(f)
        except OSError:
            raise ApiError(404, f"更新包 {uid} 不存在")

    def _save(self, m: dict):
        p = self._meta_path(m["id"])
        with open(p + ".tmp", "w", encoding="utf-8") as f:
            json.dump(m, f, ensure_ascii=False, indent=1)
        os.replace(p + ".tmp", p)

    def updates(self) -> list:
        out = []
        d = os.path.join(self.dir, "updates")
        for uid in os.listdir(d):
            try:
                m = self._load(uid)
            except ApiError:
                continue
            out.append({k: m.get(k) for k in ("id", "version", "title", "base", "created", "uploaded", "uploaded_by", "status",
                                               "applied_at", "rolled_back_at", "counts", "components", "size")})
        return sorted(out, key=lambda m: m.get("uploaded") or 0, reverse=True)

    def upload(self, data: bytes, by: str = "") -> dict:
        if not data:
            raise ApiError(400, "请求体为空")
        if len(data) > MAX_UPDATE:
            raise ApiError(413, "更新包过大")
        try:
            tf = tarfile.open(fileobj=io.BytesIO(data), mode="r:*")
        except Exception:
            raise ApiError(400, "不是有效的更新包 (应为 tools/make_update.py 生成的 .tgz)")
        with tf:
            try:
                man = json.load(tf.extractfile(MANIFEST))
            except Exception:
                raise ApiError(400, f"更新包缺少 {MANIFEST}")
            if man.get("format") != FORMAT:
                raise ApiError(400, f"更新包格式不支持: {man.get('format')}")
            files = []
            for f in man.get("files") or []:
                rel = safe_rel(f["path"])
                try:
                    m = tf.getmember("files/" + rel)
                except KeyError:
                    raise ApiError(400, f"更新包缺少文件 {rel}")
                if not m.isfile():
                    raise ApiError(400, f"更新包里 {rel} 不是普通文件")
                b = tf.extractfile(m).read()
                if sha256(b) != f.get("sha256"):
                    raise ApiError(400, f"文件校验失败: {rel} (更新包损坏)")
                files.append({"path": rel, "sha256": f["sha256"], "size": len(b), "mode": int(f.get("mode") or 0o644)})
            deletes = [safe_rel(p) for p in man.get("delete") or []]
        uid = time.strftime("u%Y%m%d-%H%M%S-") + secrets.token_hex(2)
        os.makedirs(os.path.join(self.dir, "updates", uid))
        with open(os.path.join(self.dir, "updates", uid, "pkg.tgz"), "wb") as f:
            f.write(data)
        m = {"id": uid, "version": str(man.get("version") or uid)[:40], "title": str(man.get("title") or "")[:200],
             "base": man.get("base"), "created": man.get("created"), "uploaded": time.time(), "uploaded_by": by,
             "files": files, "delete": deletes, "status": "uploaded", "size": len(data)}
        m.update(self._plan(m))
        self._save(m)
        log(f"收到更新包 {uid} 版本 {m['version']} ({len(files)} 个文件)")
        return self.view(uid)

    def _plan(self, m: dict) -> dict:
        """与当前代码比较: 新增/修改/相同/受保护跳过，以及影响的服务"""
        rows, comps = [], set()
        for f in m["files"]:
            p = os.path.join(ROOT, f["path"])
            if protected(f["path"]):
                st = "protected"
            elif not os.path.exists(p):
                st = "new"
            else:
                with open(p, "rb") as fh:
                    st = "same" if sha256(fh.read()) == f["sha256"] else "changed"
            rows.append({"path": f["path"], "status": st, "size": f["size"]})
            if st in ("new", "changed"):
                comps |= components(f["path"])
        for rel in m["delete"]:
            exists = os.path.exists(os.path.join(ROOT, rel))
            st = "protected" if protected(rel) else ("delete" if exists else "absent")
            rows.append({"path": rel, "status": st, "size": 0})
            if st == "delete":
                comps |= components(rel)
        counts = {}
        for r in rows:
            counts[r["status"]] = counts.get(r["status"], 0) + 1
        return {"plan": rows, "counts": counts, "components": sorted(comps)}

    def view(self, uid: str) -> dict:
        m = self._load(uid)
        if m.get("status") == "uploaded":         # 还没应用: 按当前代码重新比较
            m.update(self._plan(m))
        cur = self.version()["version"]
        builds = sorted(c[6:] for c in m["components"] if c.startswith("build:"))
        return {**{k: v for k, v in m.items() if k not in ("files",)},
            "current": cur, "base_mismatch": bool(m.get("base")) and m.get("base") != cur and m.get("status") == "uploaded",
            "builds": builds, "effects": [COMP_LABEL[c] for c in m["components"] if c in COMP_LABEL],
            "running_instances": [i["id"] for i in self.hub.dep.list() if i.get("status") in ("running", "degraded", "deploying")],
            "can_rollback": self._last_applied() == uid}

    def delete_update(self, uid: str):
        m = self._load(uid)
        if m.get("status") in ("applying", "rolling_back"):
            raise ApiError(409, "更新正在进行")
        if m.get("status") == "applied" and self._last_applied() == uid:
            raise ApiError(409, "这是当前生效的更新，删除后无法回滚；请先回滚或等下一次更新后再删")
        shutil.rmtree(os.path.join(self.dir, "updates", uid), ignore_errors=True)
        shutil.rmtree(os.path.join(self.dir, "backups", uid), ignore_errors=True)
        return {"ok": True}

    def _last_applied(self) -> Optional[str]:
        ap = [u for u in self.updates() if u.get("status") == "applied"]
        return max(ap, key=lambda u: u.get("applied_at") or 0)["id"] if ap else None

    # ------------------------------------------------------------------ 任务
    def _new_job(self, kind: str, title: str, by: str, ref: str = "") -> dict:
        with self.lock:
            if self.job and self.job["status"] == "running":
                raise ApiError(409, f"已有任务在执行: {self.job['title']}")
            self.job = {"id": time.strftime("j%Y%m%d-%H%M%S-") + secrets.token_hex(2), "kind": kind, "title": title, "by": by,
                        "ref": ref, "status": "running", "started": time.time(), "ended": None, "log": []}
            self._persist_job()
            return self.job

    def _persist_job(self):
        j = self.job
        p = os.path.join(self.dir, "jobs", j["id"] + ".json")
        with open(p + ".tmp", "w", encoding="utf-8") as f:
            json.dump(j, f, ensure_ascii=False)
        os.replace(p + ".tmp", p)

    def _jlog(self, msg: str, level: str = "info"):
        log(msg)
        self.job["log"].append({"t": time.time(), "level": level, "msg": msg})
        self.job["log"] = self.job["log"][-800:]
        self._persist_job()

    def _end_job(self, ok: bool, msg: str = ""):
        if msg:
            self._jlog(msg, "ok" if ok else "error")
        self.job.update(status="done" if ok else "failed", ended=time.time())
        self._persist_job()

    def jobs(self) -> list:
        out = []
        d = os.path.join(self.dir, "jobs")
        for f in sorted(os.listdir(d), reverse=True)[:30]:
            try:
                with open(os.path.join(d, f), encoding="utf-8") as fh:
                    j = json.load(fh)
                if j["status"] == "running" and (not self.job or self.job["id"] != j["id"]):
                    j["status"] = "interrupted"            # 平台重启前没跑完
                out.append({k: j.get(k) for k in ("id", "kind", "title", "by", "status", "started", "ended")})
            except Exception:
                continue
        return out

    def job_view(self, jid: str) -> dict:
        if self.job and self.job["id"] == jid:
            return self.job
        try:
            with open(os.path.join(self.dir, "jobs", safe_rel(jid) + ".json"), encoding="utf-8") as f:
                j = json.load(f)
        except OSError:
            raise ApiError(404, "任务不存在")
        if j["status"] == "running":
            j["status"] = "interrupted"
        return j

    # ------------------------------------------------------------------ 应用 / 回滚
    def apply(self, uid: str, opts: dict, by: str = "") -> dict:
        m = self._load(uid)
        if m.get("status") != "uploaded":
            raise ApiError(409, f"该更新包状态为 {m.get('status')}，不能再次应用 (可重新上传)")
        last = self._last_applied()
        j = self._new_job("apply", f"应用更新 {m['version']}", by, uid)
        threading.Thread(target=self._run_apply, args=(j, uid, last, opts), daemon=True, name="admin-apply").start()
        return j

    def rollback(self, uid: str, opts: dict, by: str = "") -> dict:
        m = self._load(uid)
        if self._last_applied() != uid:
            raise ApiError(409, "只能回滚最近一次应用的更新")
        j = self._new_job("rollback", f"回滚更新 {m['version']}", by, uid)
        threading.Thread(target=self._run_rollback, args=(j, uid, opts), daemon=True, name="admin-rollback").start()
        return j

    def _write(self, rel: str, data: bytes, mode: int = 0o644):
        p = os.path.join(ROOT, rel)
        os.makedirs(os.path.dirname(p), exist_ok=True)
        tmp = p + ".agvtmp"
        with open(tmp, "wb") as f:
            f.write(data)
        os.chmod(tmp, mode & 0o777 or 0o644)
        os.replace(tmp, p)

    def _run_apply(self, j, uid, prev, opts):
        m = self._load(uid)
        bdir = os.path.join(self.dir, "backups", uid)
        try:
            m["status"] = "applying"; self._save(m)
            plan = {r["path"]: r["status"] for r in self._plan(m)["plan"]}
            todo = [f for f in m["files"] if plan.get(f["path"]) in ("new", "changed")]
            dels = [p for p in m["delete"] if plan.get(p) == "delete"]
            comps = set()
            for rel in [f["path"] for f in todo] + dels:
                comps |= components(rel)
            self._jlog(f"更新 {m['version']} {m.get('title', '')}: 写入 {len(todo)} 个文件，删除 {len(dels)} 个")
            # 1) 备份
            shutil.rmtree(bdir, ignore_errors=True)
            os.makedirs(os.path.join(bdir, "files"))
            added = []
            for rel in [f["path"] for f in todo] + dels + [VERSION_FILE]:
                src = os.path.join(ROOT, rel)
                if os.path.isfile(src):
                    dst = os.path.join(bdir, "files", rel)
                    os.makedirs(os.path.dirname(dst), exist_ok=True)
                    shutil.copy2(src, dst)
                elif rel != VERSION_FILE:
                    added.append(rel)
            with open(os.path.join(bdir, "backup.json"), "w", encoding="utf-8") as f:
                json.dump({"added": added, "components": sorted(comps), "prev_update": prev}, f, ensure_ascii=False)
            self._jlog(f"已备份 {len(todo) + len(dels) - len(added)} 个原文件")
            # 2) 写入
            with tarfile.open(os.path.join(self.dir, "updates", uid, "pkg.tgz"), "r:*") as tf:
                for f in todo:
                    self._write(f["path"], tf.extractfile("files/" + f["path"]).read(), f.get("mode", 0o644))
            for rel in dels:
                try:
                    os.remove(os.path.join(ROOT, rel))
                except OSError:
                    pass
            self._write(VERSION_FILE, f"{m['version']} {m.get('title', '')}\n".encode())
            self._jlog("文件已写入")
            # 3) 编译 + 重启
            ok = self._finish(comps, opts, [f["path"] for f in todo])
            if not ok:
                self._jlog("编译失败，自动恢复原文件", "error")
                self._restore(uid)
                self._finish({c for c in comps if c.startswith("build:")}, {"restart_instances": False}, [])
                m = self._load(uid); m["status"] = "failed"; self._save(m)
                return self._end_job(False, "更新失败，已恢复到更新前的文件")
            m = self._load(uid); m.update(status="applied", applied_at=time.time()); self._save(m)
            self._end_job(True, f"更新 {m['version']} 已应用")
            if "hub" in comps:
                self._restart_hub_later()
        except Exception as e:
            try:
                m = self._load(uid); m["status"] = "failed"; self._save(m)
            except Exception:
                pass
            self._end_job(False, f"更新出错: {e}")

    def _restore(self, uid):
        bdir = os.path.join(self.dir, "backups", uid)
        with open(os.path.join(bdir, "backup.json"), encoding="utf-8") as f:
            b = json.load(f)
        fdir = os.path.join(bdir, "files")
        n = 0
        for dp, _dns, fns in os.walk(fdir):
            for fn in fns:
                src = os.path.join(dp, fn)
                rel = os.path.relpath(src, fdir)
                dst = os.path.join(ROOT, rel)
                os.makedirs(os.path.dirname(dst), exist_ok=True)
                shutil.copy2(src, dst + ".agvtmp")
                os.replace(dst + ".agvtmp", dst)
                n += 1
        for rel in b.get("added", []):
            try:
                os.remove(os.path.join(ROOT, rel))
            except OSError:
                pass
        self._jlog(f"已恢复 {n} 个文件，移除 {len(b.get('added', []))} 个新增文件")
        return set(b.get("components") or [])

    def _run_rollback(self, j, uid, opts):
        try:
            m = self._load(uid); m["status"] = "rolling_back"; self._save(m)
            comps = self._restore(uid)
            ok = self._finish(comps, opts, [])
            m = self._load(uid); m.update(status="rolled_back" if ok else "applied", rolled_back_at=time.time() if ok else None)
            self._save(m)
            if not ok:
                return self._end_job(False, "回滚时编译失败 (文件已恢复，请检查编译日志)")
            self._end_job(True, f"已回滚到更新 {m['version']} 之前的版本 ({self.version()['version']})")
            if "hub" in comps:
                self._restart_hub_later()
        except Exception as e:
            self._end_job(False, f"回滚出错: {e}")

    def _finish(self, comps: set, opts: dict, files: list) -> bool:
        builds = sorted(c[6:] for c in comps if c.startswith("build:"))
        if builds and not self._build(builds):
            return False
        if "termux" in comps:
            self._copy_termux(files)
        if "agent" in comps:
            self._restart_agents(opts.get("restart_instances", True))
        elif "instances" in comps and opts.get("restart_instances", True):
            self._restart_instances(None)
        elif "instances" in comps:
            self._jlog("运行中的实例没有重启，新代码在实例下次启动时生效")
        if "web" in comps:
            self._jlog("网页文件已更新，浏览器刷新后生效")
        return True

    def _copy_termux(self, files):
        if not os.path.isdir(TERMUX_HOME):
            return self._jlog("不在 Termux 环境，跳过启动脚本复制")
        n = 0
        for rel in files:
            name = rel.split("/")[-1]
            if rel.startswith("deploy/android/") and name in TERMUX_SCRIPTS:
                try:
                    shutil.copy2(os.path.join(ROOT, rel), os.path.join(TERMUX_HOME, name)); n += 1
                except OSError as e:
                    self._jlog(f"复制 {name} 到 Termux 主目录失败: {e}", "warn")
        self._jlog(f"已更新 Termux 主目录里的 {n} 个启动脚本 (下次启动平台时生效)")

    # ------------------------------------------------------------------ 编译
    def _build(self, pkgs: list) -> bool:
        ws = os.path.join(ROOT, "ros2")
        if not os.path.isfile("/opt/ros/humble/setup.bash") or not os.path.isdir(ws):
            self._jlog(f"本机没有 ROS 2 Humble 编译环境，跳过编译 {', '.join(pkgs)} (实例用的是镜像里的版本)", "warn")
            return True
        self._jlog(f"编译 ROS 插件: {' '.join(pkgs)} (手机上约 3~5 分钟)")
        logp = os.path.join(self.dir, "jobs", self.job["id"] + ".build.log")
        cmd = (". /opt/ros/humble/setup.bash && { [ -f install/local_setup.bash ] && . install/local_setup.bash || true; } && "
               f"colcon build --packages-select {' '.join(pkgs)} --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF")
        rc = self._spawn_wait("admin-build", ["bash", "-c", cmd], ws, logp, 1800)
        tail = ""
        try:
            with open(logp, encoding="utf-8", errors="replace") as f:
                tail = "\n".join(f.read().splitlines()[-12:])
        except OSError:
            pass
        if rc != 0:
            self._jlog(f"编译失败 (退出码 {rc}):\n{tail}", "error")
            return False
        self._jlog("编译完成" + (f"\n{tail.splitlines()[-1]}" if tail else ""))
        return True

    def _spawn_wait(self, name, argv, cwd, logp, timeout) -> int:
        """手机上经派生服务在独立 proot 会话里跑 (不和平台抢同一个 proot 追踪进程)；没有派生服务就直接子进程"""
        try:
            from common import spawn
            if spawn.available():
                p = spawn.popen(name, argv, cwd=cwd, env=dict(os.environ), log=logp, top_level=True)
                t0 = time.time()
                while p.poll() is None:
                    if time.time() - t0 > timeout:
                        p.terminate()
                        return -9
                    time.sleep(2)
                return p.returncode
        except Exception as e:
            self._jlog(f"派生服务不可用，改为本进程内编译: {e}", "warn")
        with open(logp, "ab") as lf:
            try:
                return subprocess.run(argv, cwd=cwd, stdout=lf, stderr=subprocess.STDOUT, timeout=timeout).returncode
            except subprocess.TimeoutExpired:
                return -9

    # ------------------------------------------------------------------ 重启
    def _running(self, node_id: Optional[str]) -> list:
        return [i for i in self.hub.dep.list() if i.get("status") in ("running", "degraded", "deploying") and not i.get("external")
                and (node_id is None or node_id in (i.get("sim_node"), i.get("nav_node")))]

    def _wait_instances(self, ids: list, timeout: float = 600):
        t0 = time.time()
        while ids and time.time() - t0 < timeout:
            left = []
            for iid in ids:
                st = self.hub.dep.get(iid).get("status")
                if st == "deploying":
                    left.append(iid)
                else:
                    self._jlog(f"实例 {iid}: {'已运行' if st == 'running' else st}", "ok" if st == "running" else "warn")
            ids = left
            time.sleep(3)
        for iid in ids:
            self._jlog(f"实例 {iid} 部署超时，请在首页查看", "warn")

    def _restart_instances(self, node_id: Optional[str]):
        ids = [i["id"] for i in self._running(node_id)]
        if not ids:
            return self._jlog("没有运行中的实例需要重启")
        for iid in ids:
            self._jlog(f"重新部署实例 {iid}")
            try:
                self.hub.dep.restart(iid)
            except Exception as e:
                self._jlog(f"实例 {iid} 重启失败: {e}", "warn")
        self._wait_instances(ids)

    def _restart_agents(self, redeploy: bool = True):
        """本机代码目录上跑的节点代理 (源码部署) 都要重启: 先停它上面的实例，重启代理，等它上线，再重新部署实例"""
        for n in self.hub.nodes.list():
            if n.get("status") == "offline":
                continue
            rt = ((n.get("info") or {}).get("runtime") or {})
            if rt.get("runtime") != "process" or os.path.realpath(rt.get("code_dir") or "") != os.path.realpath(ROOT):
                continue
            insts = [i["id"] for i in self._running(n["id"])]
            for iid in insts:
                self._jlog(f"停止实例 {iid} (节点代理 {n.get('name')} 要重启)")
                try:
                    self.hub.dep.stop(iid)
                except Exception as e:
                    self._jlog(f"停止 {iid} 出错: {e}", "warn")
            try:
                self.hub.nodes.client(n).call("POST", "/api/v1/admin/restart", {}, timeout=10)
            except Exception as e:
                self._jlog(f"节点代理 {n.get('name')} 不支持远程重启 ({e})；请在设备上手动重启 (bash ~/stop_agv.sh && bash ~/start_agv.sh)", "warn")
                continue
            self._jlog(f"节点代理 {n.get('name')} 正在重启")
            t0, seen = time.time(), n.get("last_seen") or 0
            time.sleep(4)
            while time.time() - t0 < 90:
                nn = self.hub.nodes.get(n["id"])
                if (nn.get("last_seen") or 0) > t0 + 2 and nn.get("status") != "offline":
                    break
                time.sleep(2)
            else:
                self._jlog(f"节点代理 {n.get('name')} 90 秒内没有重新上线", "warn")
                continue
            self._jlog(f"节点代理 {n.get('name')} 已重新上线")
            if insts and redeploy:
                for iid in insts:
                    self._jlog(f"重新部署实例 {iid}")
                    try:
                        self.hub.dep.restart(iid)
                    except Exception as e:
                        self._jlog(f"实例 {iid} 重新部署失败: {e}", "warn")
                self._wait_instances(insts)

    def _restart_hub_later(self, delay: float = 1.5):
        def go():
            time.sleep(delay)
            log("平台按运维请求重启 (exec)")
            sys.stdout.flush(); sys.stderr.flush()
            os.execv(sys.executable, [sys.executable, "-m", "hub.server"])
        threading.Thread(target=go, daemon=True).start()

    def restart(self, target: str, by: str = "", node_id: str = "") -> dict:
        if target == "hub":
            if self.job and self.job["status"] == "running":
                raise ApiError(409, "有任务在执行，稍后再重启平台")
            self._restart_hub_later()
            return {"ok": True, "msg": "平台将在 2 秒后重启，约 10 秒后恢复"}
        if target == "agent":
            j = self._new_job("restart", "重启节点代理", by)

            def run():
                try:
                    self._restart_agents(True)
                    self._end_job(True, "完成")
                except Exception as e:
                    self._end_job(False, f"出错: {e}")
            threading.Thread(target=run, daemon=True).start()
            return j
        raise ApiError(400, "未知的重启目标")

    # ------------------------------------------------------------------ 服务与日志
    def services(self) -> dict:
        v = self.version()
        nodes = []
        for n in self.hub.nodes.list():
            rt = ((n.get("info") or {}).get("runtime") or {})
            nodes.append({"id": n["id"], "name": n.get("name"), "host": n.get("host"), "online": n.get("status") != "offline",
                          "last_seen": n.get("last_seen"), "runtime": rt.get("runtime"),
                          "local_code": rt.get("runtime") == "process" and os.path.realpath(rt.get("code_dir") or "") == os.path.realpath(ROOT)})
        insts = [{"id": i["id"], "name": i.get("name"), "status": i.get("status"), "started": i.get("started")}
                 for i in self.hub.dep.list() if i.get("status") in ("running", "degraded", "deploying", "stopping")]
        helper = self.helper("GET", "/status", timeout=3, quiet=True)
        return {"hub": {"version": v["version"], "title": v["title"], "pid": os.getpid(), "started": self.started, "root": ROOT,
                        "python": sys.version.split()[0], "host": socket.gethostname()},
                "nodes": nodes, "instances": insts, "helper": helper, "termux": os.path.isdir(TERMUX_HOME),
                "job": {k: self.job.get(k) for k in ("id", "title", "status")} if self.job else None}

    def logs(self) -> list:
        out = []
        for name in TERMUX_LOGS:
            p = os.path.join(TERMUX_HOME, name)
            if os.path.isfile(p):
                st = os.stat(p)
                out.append({"name": name, "size": st.st_size, "mtime": st.st_mtime})
        return out

    def log_file(self, name: str) -> bytes:
        if name not in TERMUX_LOGS:
            raise ApiError(404, "没有这个日志")
        p = os.path.join(TERMUX_HOME, name)
        try:
            with open(p, "rb") as f:
                f.seek(0, 2)
                size = f.tell()
                f.seek(max(0, size - (8 << 20)))
                return f.read()
        except OSError:
            raise ApiError(404, "日志文件不存在")

    # ------------------------------------------------------------------ 安卓助手 / App
    def _helper_key(self) -> str:
        try:
            with open(os.path.join(TERMUX_HOME, ".agv-helper.key"), encoding="utf-8") as f:
                return f.read().strip()
        except OSError:
            return ""

    def helper(self, method: str, path: str, body=None, timeout: float = 30, quiet: bool = False):
        u = urllib.parse.urlparse(HELPER_URL)
        c = HTTPConnection(u.hostname, u.port, timeout=timeout)
        try:
            c.request(method, path, body=json.dumps(body).encode() if body is not None else None,
                      headers={"Content-Type": "application/json", "X-Helper-Key": self._helper_key()})
            r = c.getresponse()
            obj = json.loads(r.read().decode() or "{}")
        except (OSError, ValueError) as e:
            if quiet:
                return None
            raise ApiError(503, f"安卓助手未运行 ({e})。在 Termux 里执行: bash ~/start_agv.sh，或 nohup python ~/android_helper.py &", "no_helper")
        finally:
            c.close()
        if r.status >= 400:
            if quiet:
                return None
            raise ApiError(r.status, obj.get("error") or f"安卓助手出错 {r.status}")
        return obj

    def apps(self) -> dict:
        d = os.path.join(self.dir, "apps")
        items = []
        for f in os.listdir(d):
            if f.endswith(".json"):
                try:
                    with open(os.path.join(d, f), encoding="utf-8") as fh:
                        items.append(json.load(fh))
                except Exception:
                    pass
        items.sort(key=lambda a: a.get("uploaded") or 0, reverse=True)
        installed = {}
        for pkg in {a["package"] for a in items} | {"com.agvsim.cover"}:
            r = self.helper("GET", f"/app/version?package={pkg}", timeout=8, quiet=True)
            installed[pkg] = r
        adb = self.helper("GET", "/adb/devices", timeout=8, quiet=True)
        return {"apps": items, "installed": installed, "adb": adb, "helper": bool(adb is not None)}

    def app_upload(self, data: bytes, filename: str, by: str = "") -> dict:
        if len(data) > MAX_APK:
            raise ApiError(413, "APK 过大")
        info = apk_info(data)
        aid = f"{info['package']}-{info.get('versionCode')}-{sha256(data)[:8]}"
        d = os.path.join(self.dir, "apps")
        with open(os.path.join(d, aid + ".apk"), "wb") as f:
            f.write(data)
        doc = dict(info, id=aid, filename=filename or aid + ".apk", size=len(data), sha256=sha256(data), uploaded=time.time(), uploaded_by=by)
        with open(os.path.join(d, aid + ".json"), "w", encoding="utf-8") as f:
            json.dump(doc, f, ensure_ascii=False)
        return doc

    def app_delete(self, aid: str):
        d = os.path.join(self.dir, "apps")
        for ext in (".apk", ".json"):
            try:
                os.remove(os.path.join(d, safe_rel(aid) + ext))
            except OSError:
                pass
        return {"ok": True}

    def app_install(self, aid: str, mode: str, serial: str = "") -> dict:
        src = os.path.join(self.dir, "apps", safe_rel(aid) + ".apk")
        if not os.path.isfile(src):
            raise ApiError(404, "APK 不存在")
        if not os.path.isdir(TERMUX_HOME):
            raise ApiError(400, "平台不在安卓 (Termux) 上运行，不能在本机安装 APK")
        dst_dir = os.path.join(TERMUX_HOME, "agv-apps")
        os.makedirs(dst_dir, exist_ok=True)
        dst = os.path.join(dst_dir, aid + ".apk")
        shutil.copy2(src, dst)
        return self.helper("POST", "/app/install", {"path": dst, "mode": mode, "serial": serial}, timeout=180)

    def adb(self, action: str, body: dict) -> dict:
        if action not in ("pair", "connect", "disconnect", "mdns"):
            raise ApiError(400, "未知操作")
        return self.helper("POST", f"/adb/{action}", body or {}, timeout=40)

    def panel(self, action: str) -> dict:
        if action not in ("open", "close"):
            raise ApiError(400, "未知操作")
        return self.helper("POST", "/panel", {"action": action}, timeout=15)
