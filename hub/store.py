#!/usr/bin/env python3
"""
平台持久化: SQLite (文档表，每类资源一张表 id → JSON) + 文件目录

  <HUB_DATA>/hub.db
  <HUB_DATA>/models/<mid>/<ver>/   model.cmodel · base.json · overrides.json
  <HUB_DATA>/scenes/<sid>/         scene.json · map.pgm · map.yaml · mesh.* · taskflows.json
  <HUB_DATA>/packages/<pid>.tar[.gz]
  <HUB_DATA>/records/<rid>.json
"""

import json
import os
import sqlite3
import threading
import time
from typing import Any, Dict, List, Optional

KINDS = ("nodes", "tokens", "models", "scenes", "packages", "instances", "records", "settings")


class Store:
    def __init__(self, root: str):
        self.root = os.path.abspath(os.path.expanduser(root))
        for d in ("models", "scenes", "packages", "records", "tmp"):
            os.makedirs(os.path.join(self.root, d), exist_ok=True)
        self.db = sqlite3.connect(os.path.join(self.root, "hub.db"), check_same_thread=False, isolation_level=None)
        self.db.execute("PRAGMA journal_mode=WAL")
        self.lock = threading.RLock()
        for k in KINDS:
            self.db.execute(f"CREATE TABLE IF NOT EXISTS {k} (id TEXT PRIMARY KEY, data TEXT NOT NULL, updated REAL)")

    def path(self, *parts) -> str:
        return os.path.join(self.root, *parts)

    # ------------------------------------------------------------------ 文档
    def get(self, kind: str, id_: str) -> Optional[dict]:
        with self.lock:
            r = self.db.execute(f"SELECT data FROM {kind} WHERE id=?", (id_,)).fetchone()
        return json.loads(r[0]) if r else None

    def put(self, kind: str, doc: dict) -> dict:
        doc = dict(doc)
        doc.setdefault("created", time.time())
        doc["updated"] = time.time()
        with self.lock:
            self.db.execute(f"INSERT OR REPLACE INTO {kind} (id, data, updated) VALUES (?,?,?)",
                            (doc["id"], json.dumps(doc, ensure_ascii=False), doc["updated"]))
        return doc

    def update(self, kind: str, id_: str, **fields) -> Optional[dict]:
        with self.lock:
            d = self.get(kind, id_)
            if d is None:
                return None
            d.update(fields)
            return self.put(kind, d)

    def delete(self, kind: str, id_: str):
        with self.lock:
            self.db.execute(f"DELETE FROM {kind} WHERE id=?", (id_,))

    def list(self, kind: str, **match) -> List[dict]:
        with self.lock:
            rows = self.db.execute(f"SELECT data FROM {kind} ORDER BY updated DESC").fetchall()
        out = [json.loads(r[0]) for r in rows]
        if match:
            out = [d for d in out if all(d.get(k) == v for k, v in match.items())]
        return out

    # ------------------------------------------------------------------ 设置
    def setting(self, key: str, default: Any = None) -> Any:
        d = self.get("settings", key)
        return d["value"] if d else default

    def set_setting(self, key: str, value: Any):
        self.put("settings", {"id": key, "value": value})

    # ------------------------------------------------------------------ 文件
    def write_json(self, path: str, obj: Any):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(obj, f, ensure_ascii=False, indent=2)
        os.replace(tmp, path)

    @staticmethod
    def read_json(path: str, default: Any = None) -> Any:
        try:
            with open(path, encoding="utf-8") as f:
                return json.load(f)
        except (OSError, ValueError):
            return default


def new_id(prefix: str, existing: Dict[str, Any] = None, n: int = 6) -> str:
    import secrets
    while True:
        i = f"{prefix}{secrets.token_hex(n // 2 + 1)[:n]}"
        if not existing or i not in existing:
            return i


def slug(s: str, fallback: str = "x") -> str:
    import re
    t = re.sub(r"[^A-Za-z0-9_.-]+", "-", s or "").strip("-.").lower()
    return t[:40] or fallback
