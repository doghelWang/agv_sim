#!/usr/bin/env python3
"""
生成离线接口文档: 从运行中的各服务取 OpenAPI 3.1 文档，写到
  docs/openapi/<服务>.json     平台「接口文档」页在服务没运行时显示它
  docs/API_REFERENCE.md         全部接口一览 (自动生成，勿手改)

  python3 tools/gen_api_docs.py --hub http://127.0.0.1:8082 --agent http://127.0.0.1:8070 \\
      --sim http://127.0.0.1:8090 --nav http://127.0.0.1:8091 --gateway http://127.0.0.1:8088
缺哪个就跳过哪个 (保留已有的 json)。
"""
import argparse
import json
import os
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SVCS = [("hub", "/api/v1/openapi.json"), ("agent", "/api/v1/openapi.json"), ("sim", "/api/v1/openapi.json"),
        ("nav", "/api/v1/openapi.json"), ("gateway", "/api/v2/openapi.json")]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    for k, _ in SVCS:
        ap.add_argument("--" + k, help=f"{k} 服务地址")
    a = ap.parse_args()
    out = os.path.join(ROOT, "docs", "openapi")
    os.makedirs(out, exist_ok=True)
    for k, p in SVCS:
        base = getattr(a, k)
        if not base:
            continue
        with urllib.request.urlopen(base.rstrip("/") + p, timeout=10) as r:
            spec = json.load(r)
        spec.pop("x-source", None)
        spec.get("info", {}).pop("x-build", None)
        with open(os.path.join(out, f"{k}.json"), "w", encoding="utf-8") as f:
            json.dump(spec, f, ensure_ascii=False, indent=1)
            f.write("\n")
        print(f"{k}: {sum(len(v) for v in spec['paths'].values())} 个接口")
    lines = ["# 接口一览 (自动生成)", "",
             "> 由 `tools/gen_api_docs.py` 从各服务的 OpenAPI 3.1 文档 (`docs/openapi/*.json`) 生成，请勿手改。",
             "> 约定与规范依据见 [API.md](API.md)；运行中的服务可直接访问 `GET <base>/openapi.json`，平台网页「接口文档」可浏览。", ""]
    for k, _ in SVCS:
        f = os.path.join(out, f"{k}.json")
        if not os.path.exists(f):
            continue
        with open(f, encoding="utf-8") as fh:
            spec = json.load(fh)
        info = spec.get("info", {})
        lines += [f"## {info.get('title', k)}", "", info.get("description", ""), "", f"OpenAPI: `docs/openapi/{k}.json` · 契约版本 {info.get('version', '')}", "",
                  "| 方法 | 路径 | 说明 |", "|---|---|---|"]
        for path, item in spec["paths"].items():
            for m, op in item.items():
                desc = (op.get("description") or op.get("summary") or "").replace("|", "\\|").replace("\n", " ")
                lines.append(f"| {m.upper()} | `{path}` | {desc} |")
        lines.append("")
    with open(os.path.join(ROOT, "docs", "API_REFERENCE.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print("docs/API_REFERENCE.md 已更新")


if __name__ == "__main__":
    main()
