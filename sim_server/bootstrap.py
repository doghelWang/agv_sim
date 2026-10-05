#!/usr/bin/env python3
"""
仿真实例启动准备 (容器入口/进程运行时在启动仿真进程前调用)

  1. 初始化数据目录: 首次运行拷入默认模型 (已有文件不覆盖)
  2. 平台部署 (HUB_API + MODEL_ID/SCENE_ID): 从平台下载模型包 (基线+补全) 与场景定义/栅格地图
       /data/robot_config.base.json  /data/model_overrides.json  → /data/robot_config.json  /data/robot.urdf
       /data/scene/scene.json  /data/scene/map.pgm|yaml
     模型版本未变且本地已有补全时保留本地补全 (实例内修改不因重启丢失)
  3. 输出 export 语句 (供 shell 入口 eval): ROBOT_CONFIG / SIM_SCENE_FILE
"""

import json
import os
import shutil
import sys
import time
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


def log(m):
    print(f"[bootstrap] {m}", file=sys.stderr, flush=True)


def fetch(url: str, timeout: float = 10.0, tries: int = 15) -> bytes:
    last = None
    for _ in range(tries):
        try:
            with urllib.request.urlopen(url, timeout=timeout) as r:
                return r.read()
        except Exception as e:
            last = e
            time.sleep(2)
    raise RuntimeError(f"下载 {url} 失败: {last}")


def init_defaults(data: str):
    d = os.path.join(ROOT, "defaults")
    if not os.path.isdir(d):
        d = ROOT
    for f in ("robot_config.json", "sensor_overrides.json", "robot.urdf"):
        src = os.path.join(d, f)
        if not os.path.exists(os.path.join(data, f)) and os.path.exists(src):
            shutil.copy(src, os.path.join(data, f))
    cm = os.path.join(d, "cmodel") if os.path.isdir(os.path.join(d, "cmodel")) else os.path.join(ROOT, "tests", "data")
    dst = os.path.join(data, "cmodel")
    if os.path.isdir(cm) and not (os.path.isdir(dst) and os.listdir(dst)):
        # 只拷内容，不用 copytree/copy2: 它们会连扩展属性一起拷，Android 上 (proot 不伪装 root 时) 写 security.selinux
        # 被拒绝 (EACCES)，整个初始化抛异常 —— 新建实例因此拿不到平台下发的模型和场景，悄悄退回默认的九宫格
        os.makedirs(dst, exist_ok=True)
        for fn in os.listdir(cm):
            sp = os.path.join(cm, fn)
            if os.path.isfile(sp):
                shutil.copyfile(sp, os.path.join(dst, fn))


def pull_model(hub: str, data: str, mid: str, ver: str):
    from cmodel_parser import generate_urdf
    from model_overrides import apply_overrides, save_overrides
    b = json.loads(fetch(f"{hub}/api/hub/models/{mid}/versions/{ver or 'latest'}/bundle"))
    ref_p = os.path.join(data, ".model_ref")
    ref = f"{b['model_id']}@{b['version']}"
    old = open(ref_p).read().strip() if os.path.exists(ref_p) else ""
    ov_p = os.path.join(data, "model_overrides.json")
    with open(os.path.join(data, "robot_config.base.json"), "w", encoding="utf-8") as f:
        json.dump(b["base"], f, indent=2, ensure_ascii=False)
    if old != ref or not os.path.exists(ov_p):
        save_overrides(b.get("overrides") or {"version": 1}, ov_p)
        for legacy in ("sensor_overrides.json",):
            p = os.path.join(data, legacy)
            if os.path.exists(p):
                os.remove(p)
    with open(ov_p, encoding="utf-8") as f:
        ov = json.load(f)
    spec = apply_overrides(b["base"], ov)
    spec["repo"] = {k: b.get(k) for k in ("model_id", "version", "name", "material_no", "vtype", "project")}
    with open(os.path.join(data, "robot_config.json"), "w", encoding="utf-8") as f:
        json.dump(spec, f, indent=2, ensure_ascii=False)
    with open(os.path.join(data, "robot.urdf"), "w", encoding="utf-8") as f:
        f.write(generate_urdf(spec))
    with open(ref_p, "w") as f:
        f.write(ref)
    log(f"模型 {b.get('name')} {b['version']} 已就绪 (补全 {'沿用本地' if old == ref else '来自平台'})")


def pull_scene(hub: str, data: str, sid: str) -> str:
    sd = os.path.join(data, "scene")
    os.makedirs(sd, exist_ok=True)
    sc = json.loads(fetch(f"{hub}/api/hub/scenes/{sid}/definition"))
    try:
        pgm = fetch(f"{hub}/api/hub/scenes/{sid}/map?part=pgm", timeout=60, tries=3)
        yml = fetch(f"{hub}/api/hub/scenes/{sid}/map?part=yaml", tries=3).decode("utf-8")
        with open(os.path.join(sd, "map.pgm"), "wb") as f:
            f.write(pgm)
        with open(os.path.join(sd, "map.yaml"), "w", encoding="utf-8") as f:
            f.write(yml)
        sc["map_files"] = {"pgm": os.path.join(sd, "map.pgm"), "yaml": os.path.join(sd, "map.yaml")}
    except Exception as e:
        log(f"栅格地图下载失败 (仿真进程将按几何生成): {e}")
    p = os.path.join(sd, "scene.json")
    with open(p, "w", encoding="utf-8") as f:
        json.dump(sc, f, ensure_ascii=False, indent=2)
    log(f"场景 {sc.get('name')} ({sid}) 已就绪")
    return p


def main():
    data = os.environ.get("AGV_DATA", "/data")
    os.makedirs(data, exist_ok=True)
    try:
        init_defaults(data)
    except Exception as e:
        log(f"默认数据初始化失败 (继续): {e}")
    hub = (os.environ.get("HUB_API") or "").rstrip("/")
    exports = {"ROBOT_CONFIG": os.path.join(data, "robot_config.json")}
    if hub and os.environ.get("MODEL_ID"):
        try:
            pull_model(hub, data, os.environ["MODEL_ID"], os.environ.get("MODEL_VER", ""))
        except Exception as e:
            # 平台指定了模型却拿不到: 不能悄悄换成本地默认模型去跑 (实例名写的是 A，跑的是 B)
            log(f"[错误] 从平台加载模型 {os.environ['MODEL_ID']} 失败: {e}")
            sys.exit(3)
    if hub and os.environ.get("SCENE_ID"):
        sid = os.environ["SCENE_ID"]
        try:
            exports["SIM_SCENE_FILE"] = pull_scene(hub, data, sid)
        except Exception as e:
            from planning.dijkstra_planner import SCENARIO_DEFINITIONS
            if sid in SCENARIO_DEFINITIONS:       # 内置场景: 本地就有同一份定义，按 ID 加载
                log(f"从平台加载场景失败，使用本地内置的同名场景 {sid}: {e}")
                exports["SIM_SCENARIO"] = sid
            else:
                log(f"[错误] 从平台加载场景 {sid} 失败: {e}")
                sys.exit(3)
    for k, v in exports.items():
        print(f"export {k}='{v}'")


if __name__ == "__main__":
    main()
