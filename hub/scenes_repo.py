#!/usr/bin/env python3
"""
仿真场景仓库

场景包 (zip) 内容:
  scene.json      场景定义 (与 planning/dijkstra_planner.SCENARIO_DEFINITIONS 同格式):
                  {id, name, description, origin{x,y,yaw}, walls[[x0,y0,x1,y1]...] (前 4 条为外墙),
                   shelves[{name,x1,y1,x2,y2}], stations[{id,name,x,y,dock_yaw}], nodes{id:[x,y]}, connections[[a,b]],
                   reflectors[{x,y}], meta{...}}
  topology.json   (可选) {nodes, connections, stations} —— 覆盖/补充 scene.json 的拓扑
  map.pgm/.yaml   (可选) 2D SLAM 栅格；只有栅格没有 walls 时，由栅格轮廓生成碰撞墙体
  mesh.obj|glb    (可选) 3D 外观，仅显示
"""

import io
import json
import math
import os
import re
import shutil
import sys
import time
import zipfile
from typing import List, Optional, Tuple

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from common.rest import ApiError  # noqa: E402
from hub.store import Store, slug  # noqa: E402

BUILTIN_TAGS = {"grid_9_square": "立体仓储", "standard_cross": "十字交叉", "narrow_aisle": "窄巷道", "rect_loop": "环线",
                "fms_workshop": "柔性车间", "fms_workshop_xl": "柔性车间·大车"}


# ====================================================================== 栅格 (PGM/YAML)
def parse_yaml_map(text: str) -> dict:
    d = {}
    for line in text.splitlines():
        line = line.split("#", 1)[0].strip()
        if ":" not in line:
            continue
        k, v = [x.strip() for x in line.split(":", 1)]
        if v.startswith("["):
            d[k] = [float(x) for x in v.strip("[]").split(",") if x.strip()]
        else:
            try:
                d[k] = float(v) if re.match(r"^-?[\d.eE+-]+$", v) else v.strip("'\"")
            except ValueError:
                d[k] = v
    return d


def parse_pgm(data: bytes) -> Tuple[int, int, int, bytes]:
    """→ (W, H, maxval, 像素字节[行优先，第 0 行为图像顶部])"""
    toks, i = [], 0
    magic = data[:2]
    i = 2
    while len(toks) < 3:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while i < len(data) and data[i:i + 1] not in (b"\n", b"\r"):
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        toks.append(int(data[i:j]))
        i = j
    W, H, mx = toks
    i += 1
    if magic == b"P5":
        px = data[i:i + W * H]
    elif magic == b"P2":
        px = bytes(min(255, int(v) * 255 // max(1, mx)) for v in data[i:].split()[:W * H])
    else:
        raise ValueError("仅支持 PGM (P5/P2)")
    if len(px) < W * H:
        raise ValueError("PGM 数据不完整")
    return W, H, mx, px


def grid_to_walls(W: int, H: int, px: bytes, res: float, origin, negate: int = 0, occ_thresh: float = 0.65,
                  max_segments: int = 4000) -> List[list]:
    """占据栅格 → 墙体线段 (占据格与非占据格之间的边界，按行/列合并)"""
    import numpy as np
    a = np.frombuffer(px, np.uint8)[:W * H].reshape(H, W).astype(np.float32)
    p = (a / 255.0) if negate else (255.0 - a) / 255.0
    occ = p > occ_thresh
    k = max(1, int(round(0.10 / res)))
    while True:
        Hk, Wk = H // k, W // k
        g = occ[:Hk * k, :Wk * k].reshape(Hk, k, Wk, k).any(axis=(1, 3))
        segs = _edges(g, res * k, origin, H * res)
        if len(segs) <= max_segments or k > 64:
            return segs
        k *= 2


def _edges(g, cell: float, origin, height_m: float) -> List[list]:
    import numpy as np
    Hk, Wk = g.shape
    ox, oy = origin[0], origin[1]
    pad = np.pad(g, 1)
    segs = []
    # 水平边: 第 r 行格子的上边 (与 r-1 行不同) —— 图像行向下，世界 y 向上
    for r in range(Hk + 1):
        diff = pad[r, 1:-1] != pad[r + 1, 1:-1]
        y = oy + height_m - r * cell
        c = 0
        while c < Wk:
            if diff[c]:
                c0 = c
                while c < Wk and diff[c]:
                    c += 1
                segs.append([round(ox + c0 * cell, 3), round(y, 3), round(ox + c * cell, 3), round(y, 3)])
            else:
                c += 1
    for c in range(Wk + 1):
        diff = pad[1:-1, c] != pad[1:-1, c + 1]
        x = ox + c * cell
        r = 0
        while r < Hk:
            if diff[r]:
                r0 = r
                while r < Hk and diff[r]:
                    r += 1
                segs.append([round(x, 3), round(oy + height_m - r0 * cell, 3), round(x, 3), round(oy + height_m - r * cell, 3)])
            else:
                r += 1
    return segs


# ====================================================================== 仓库
class SceneRepo:
    def __init__(self, store: Store):
        self.s = store

    def sdir(self, sid: str) -> str:
        return self.s.path("scenes", sid)

    def get(self, sid: str) -> dict:
        d = self.s.get("scenes", sid)
        if not d:
            raise ApiError(404, f"场景 {sid} 不存在")
        return d

    def scene_def(self, sid: str) -> dict:
        self.get(sid)
        sc = Store.read_json(os.path.join(self.sdir(sid), "scene.json"))
        if sc is None:
            raise ApiError(500, "场景定义缺失")
        return sc

    # ------------------------------------------------------------------ 内置场景入库
    def seed_builtins(self):
        from planning.dijkstra_planner import SCENARIO_DEFINITIONS
        for sid, sc in SCENARIO_DEFINITIONS.items():
            if self.s.get("scenes", sid):
                continue
            d = {k: ([list(c) for c in v] if k in ("walls", "connections") else
                     ({n: list(xy) for n, xy in v.items()} if k == "nodes" else v)) for k, v in sc.items()}
            d.setdefault("meta", {})["resolution"] = 0.05
            self._save(sid, d, {"builtin": True, "tags": [BUILTIN_TAGS.get(sid, "内置")], "source": "内置场景库"})

    # ------------------------------------------------------------------ 概要
    @staticmethod
    def summarize(sc: dict, sdir: Optional[str] = None) -> dict:
        walls = sc.get("walls", [])
        pts = [(w[0], w[1]) for w in walls] + [(w[2], w[3]) for w in walls]
        if pts:
            b = [min(p[0] for p in pts), min(p[1] for p in pts), max(p[0] for p in pts), max(p[1] for p in pts)]
        else:
            b = [0, 0, 0, 0]
        W, H = b[2] - b[0], b[3] - b[1]
        meta = sc.get("meta", {})
        res = float(meta.get("resolution", 0.05))
        has_pgm = bool(sdir) and os.path.exists(os.path.join(sdir, "map.pgm"))
        mesh = None
        if sdir and os.path.isdir(sdir):
            mesh = next((f for f in os.listdir(sdir) if f.lower().startswith("mesh.")), None)
        return {"bounds": b, "size": [round(W, 2), round(H, 2)], "area_m2": round(W * H, 1), "resolution": res,
                "grid": [int(math.ceil(W / res)), int(math.ceil(H / res))], "walls": len(walls),
                "shelves": len(sc.get("shelves", [])), "stations": len(sc.get("stations", [])),
                "nodes": len(sc.get("nodes", {})), "edges": len(sc.get("connections", [])),
                "reflectors": len(sc.get("reflectors", [])), "map": "pgm" if has_pgm else "generated", "mesh": mesh,
                "origin": sc.get("origin"), "ceiling_m": meta.get("ceiling_m", 6.0), "shelf_height_m": meta.get("shelf_height_m", 2.5)}

    def view(self, d: dict) -> dict:
        v = dict(d)
        v["summary"] = d.get("summary") or {}
        return v

    def list(self) -> list:
        return [self.view(d) for d in sorted(self.s.list("scenes"), key=lambda d: (not d.get("builtin"), d.get("created", 0)))]

    # ------------------------------------------------------------------ 保存/导入
    def _save(self, sid: str, sc: dict, extra: dict) -> dict:
        sc = normalize(sc, sid)
        sd = self.sdir(sid)
        os.makedirs(sd, exist_ok=True)
        self.s.write_json(os.path.join(sd, "scene.json"), sc)
        d = self.s.get("scenes", sid) or {"id": sid, "created": time.time()}
        d.update({"name": sc["name"], "description": sc.get("description", ""), **extra})
        d["summary"] = self.summarize(sc, sd)
        return self.s.put("scenes", d)

    def import_upload(self, filename: str, data: bytes, meta: dict) -> dict:
        fn = filename.lower()
        files = {}
        if fn.endswith(".zip"):
            try:
                z = zipfile.ZipFile(io.BytesIO(data))
            except zipfile.BadZipFile:
                raise ApiError(400, "不是有效的 zip 场景包")
            for n in z.namelist():
                if n.endswith("/"):
                    continue
                files[os.path.basename(n).lower()] = z.read(n)
        elif fn.endswith(".json"):
            files["scene.json"] = data
        else:
            raise ApiError(400, "场景包需为 .zip (scene.json / map.pgm+map.yaml / topology.json / mesh.obj|glb) 或 scene.json")
        sc = {}
        sj = files.get("scene.json") or next((v for k, v in files.items() if k.endswith(".json") and "topo" not in k), None)
        if sj:
            try:
                sc = json.loads(sj.decode("utf-8"))
            except ValueError as e:
                raise ApiError(400, f"scene.json 解析失败: {e}")
        topo = files.get("topology.json") or next((v for k, v in files.items() if "topo" in k and k.endswith(".json")), None)
        if topo:
            t = json.loads(topo.decode("utf-8"))
            for k in ("nodes", "connections", "stations"):
                if t.get(k):
                    sc[k] = t[k]
        pgm = next((v for k, v in files.items() if k.endswith(".pgm")), None)
        yml = next((v for k, v in files.items() if k.endswith((".yaml", ".yml"))), None)
        ymap = parse_yaml_map(yml.decode("utf-8", "replace")) if yml else {}
        if pgm is not None:
            try:
                W, H, mx, px = parse_pgm(pgm)
            except Exception as e:
                raise ApiError(400, f"PGM 解析失败: {e}")
            res = float(ymap.get("resolution", 0.05))
            org = ymap.get("origin", [0.0, 0.0, 0.0])
            sc.setdefault("meta", {}).update({"resolution": res, "map_origin": org[:2], "map_size_px": [W, H]})
            if not sc.get("walls"):
                inner = grid_to_walls(W, H, px, res, org, int(ymap.get("negate", 0) or 0), float(ymap.get("occupied_thresh", 0.65)))
                x0, y0 = org[0], org[1]
                x1, y1 = x0 + W * res, y0 + H * res
                sc["walls"] = [[x0, y0, x1, y0], [x1, y0, x1, y1], [x1, y1, x0, y1], [x0, y1, x0, y0]] + inner
                sc.setdefault("meta", {})["walls_from_grid"] = True
        if not sc.get("walls"):
            raise ApiError(400, "场景包缺少几何: 需要 scene.json 的 walls，或 map.pgm + map.yaml")
        name = (meta.get("name") or sc.get("name") or os.path.splitext(filename)[0]).strip()
        sc["name"] = name
        if meta.get("description"):
            sc["description"] = meta["description"]
        sid = meta.get("scene_id") or ("s-" + slug(name, "scene"))
        base, i = sid, 2
        while self.s.get("scenes", sid) and not meta.get("scene_id"):
            sid, i = f"{base}-{i}", i + 1
        sd = self.sdir(sid)
        if os.path.isdir(sd):
            shutil.rmtree(sd)
        os.makedirs(sd, exist_ok=True)
        if pgm is not None:
            with open(os.path.join(sd, "map.pgm"), "wb") as f:
                f.write(pgm)
            ymap["image"] = "map.pgm"
            with open(os.path.join(sd, "map.yaml"), "w", encoding="utf-8") as f:
                f.write(_yaml_text(ymap))
        for k, v in files.items():
            if k.endswith((".obj", ".glb", ".gltf", ".stl")):
                with open(os.path.join(sd, "mesh" + os.path.splitext(k)[1]), "wb") as f:
                    f.write(v)
                break
        tags = [t for t in (meta.get("tags") or "").split(",") if t.strip()] if isinstance(meta.get("tags"), str) else meta.get("tags") or []
        return self._save(sid, sc, {"builtin": False, "tags": tags or ["自定义"], "source": filename,
                                    "operator": meta.get("operator", "")})

    def update_meta(self, sid: str, fields: dict) -> dict:
        d = self.get(sid)
        sc = self.scene_def(sid)
        for k in ("name", "description"):
            if k in fields:
                sc[k] = fields[k]
                d[k] = fields[k]
        if "tags" in fields:
            d["tags"] = fields["tags"]
        self.s.write_json(os.path.join(self.sdir(sid), "scene.json"), sc)
        self.s.put("scenes", d)
        return self.view(d)

    def delete(self, sid: str):
        d = self.get(sid)
        if d.get("builtin"):
            raise ApiError(400, "内置场景不能删除")
        shutil.rmtree(self.sdir(sid), ignore_errors=True)
        self.s.delete("scenes", sid)

    # ------------------------------------------------------------------ 地图/场景包
    def map_files(self, sid: str) -> Tuple[bytes, str]:
        sd = self.sdir(sid)
        pgm_p, yml_p = os.path.join(sd, "map.pgm"), os.path.join(sd, "map.yaml")
        if os.path.exists(pgm_p) and os.path.exists(yml_p):
            with open(pgm_p, "rb") as f, open(yml_p, encoding="utf-8") as g:
                return f.read(), g.read()
        gen_p, gyml_p = os.path.join(sd, "map.gen.pgm"), os.path.join(sd, "map.gen.yaml")
        if not os.path.exists(gen_p):
            from tools.scenario_to_map import rasterize
            sc = self.scene_def(sid)
            res = float(sc.get("meta", {}).get("resolution", 0.05))
            img, W, H, (ox, oy) = rasterize(sc, res)
            with open(gen_p, "wb") as f:
                f.write(f"P5\n# scene {sid}\n{W} {H}\n255\n".encode() + bytes(img))
            with open(gyml_p, "w", encoding="utf-8") as f:
                f.write(f"image: map.pgm\nmode: trinary\nresolution: {res}\norigin: [{ox:.3f}, {oy:.3f}, 0.0]\n"
                        f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
        with open(gen_p, "rb") as f, open(gyml_p, encoding="utf-8") as g:
            return f.read(), g.read()

    def package_zip(self, sid: str) -> bytes:
        sc = self.scene_def(sid)
        sd = self.sdir(sid)
        pgm, yml = self.map_files(sid)
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
            z.writestr("scene.json", json.dumps(sc, ensure_ascii=False, indent=2))
            z.writestr("topology.json", json.dumps({"nodes": sc.get("nodes", {}), "connections": sc.get("connections", []),
                                                    "stations": sc.get("stations", [])}, ensure_ascii=False, indent=2))
            z.writestr("map.pgm", pgm)
            z.writestr("map.yaml", yml.replace("map.gen.pgm", "map.pgm"))
            for f in os.listdir(sd):
                if f.startswith("mesh."):
                    z.write(os.path.join(sd, f), f)
            tf = os.path.join(sd, "taskflows.json")
            if os.path.exists(tf):
                z.write(tf, "taskflows.json")
        return buf.getvalue()

    # ------------------------------------------------------------------ 任务流 (属于场景)
    def taskflows(self, sid: str) -> list:
        self.get(sid)
        tf = Store.read_json(os.path.join(self.sdir(sid), "taskflows.json"))
        if tf is None:
            tf = default_taskflows(self.scene_def(sid))
        return tf

    def save_taskflows(self, sid: str, flows: list) -> list:
        self.get(sid)
        if not isinstance(flows, list):
            raise ApiError(400, "任务流需为数组")
        self.s.write_json(os.path.join(self.sdir(sid), "taskflows.json"), flows)
        return flows


def _yaml_text(d: dict) -> str:
    out = []
    for k in ("image", "mode", "resolution", "origin", "negate", "occupied_thresh", "free_thresh"):
        if k in d:
            v = d[k]
            out.append(f"{k}: [{', '.join(str(x) for x in v)}]" if isinstance(v, list) else f"{k}: {v}")
    return "\n".join(out) + "\n"


def normalize(sc: dict, sid: str) -> dict:
    sc = dict(sc)
    sc["id"] = sid
    sc.setdefault("name", sid)
    sc.setdefault("description", "")
    walls = [list(map(float, w[:4])) for w in sc.get("walls", [])]
    if walls:
        xs = [c for w in walls for c in (w[0], w[2])]
        ys = [c for w in walls for c in (w[1], w[3])]
        b0 = walls[:4]
        bx = [c for w in b0 for c in (w[0], w[2])]
        by = [c for w in b0 for c in (w[1], w[3])]
        # 约定前 4 条为外墙；若不满足 (外墙未包住全部几何)，自动补一圈外墙
        if len(walls) < 4 or min(bx) > min(xs) + 1e-6 or max(bx) < max(xs) - 1e-6 or min(by) > min(ys) + 1e-6 or max(by) < max(ys) - 1e-6:
            x0, y0, x1, y1 = min(xs) - 1.0, min(ys) - 1.0, max(xs) + 1.0, max(ys) + 1.0
            walls = [[x0, y0, x1, y0], [x1, y0, x1, y1], [x1, y1, x0, y1], [x0, y1, x0, y0]] + walls
    sc["walls"] = walls
    sc["shelves"] = sc.get("shelves", [])
    sc["stations"] = [dict(s, id=str(s.get("id") or f"P{i}"), name=s.get("name") or str(s.get("id") or f"P{i}"),
                           dock_yaw=float(s.get("dock_yaw", s.get("yaw", 0.0))))
                      for i, s in enumerate(sc.get("stations", []))]
    nodes = sc.get("nodes", {})
    if isinstance(nodes, list):          # [{id,x,y}] → {id:[x,y]}
        nodes = {str(n["id"]): [float(n["x"]), float(n["y"])] for n in nodes}
    sc["nodes"] = {str(k): [float(v[0]), float(v[1])] for k, v in nodes.items()}
    sc["connections"] = [[str(a), str(b)] for a, b in (list(c)[:2] for c in sc.get("connections", sc.get("edges", [])))]
    sc.pop("edges", None)
    if not sc.get("origin"):
        s0 = next((s for s in sc["stations"] if s["id"].upper() == "P0"), sc["stations"][0] if sc["stations"] else None)
        if s0:
            sc["origin"] = {"x": s0["x"], "y": s0["y"], "yaw": s0.get("dock_yaw", 0.0)}
        elif walls:
            sc["origin"] = {"x": (walls[0][0] + walls[1][0]) / 2, "y": (walls[0][1] + walls[1][3]) / 2, "yaw": 0.0}
        else:
            sc["origin"] = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    sc.setdefault("meta", {})
    return sc


def default_taskflows(sc: dict) -> list:
    """没有保存过任务流的场景: 用前几个工位生成一条示例任务流"""
    st = [s for s in sc.get("stations", [])][:4]
    if len(st) < 2:
        return []
    steps = []
    for i, s in enumerate(st[1:]):
        steps.append({"type": "move", "target": s["id"], "speed": 1.0})
        steps.append({"type": "lift" if i % 2 == 0 else "drop", "target": s["id"]})
    steps.append({"type": "move", "target": st[0]["id"], "speed": 1.0})
    return [{"id": "TF-01", "name": f"{sc.get('name', '')} 示例巡回", "tid": "26001", "loop": "single",
             "description": " → ".join(s["id"] for s in st) + " → " + st[0]["id"], "steps": steps}]
