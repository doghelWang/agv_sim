#!/usr/bin/env python3
"""
生成平台「运维管理 → 平台更新」用的更新包 (.tgz)

  python3 tools/make_update.py --base fc69b28                   # base..HEAD 之间改动的文件
  python3 tools/make_update.py --base fc69b28 --to main -o x.tgz
  python3 tools/make_update.py --paths hub/admin.py web/js/platform.js --version test1

包内容: agv_update.json (格式、版本、标题、base、文件清单与 sha256、要删除的文件) + files/<路径>
版本默认取 --to 的短提交号，标题取提交说明首行。设备上 .agv_version 的版本与 base 不一致时，平台会提示 (仍可应用)。
"""
import argparse
import hashlib
import io
import json
import os
import subprocess
import sys
import tarfile
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SKIP = ("phone_snapshot/", "tests/", ".github/")


def git(*a) -> bytes:
    return subprocess.run(["git", "-C", ROOT] + list(a), check=True, stdout=subprocess.PIPE).stdout


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base", help="设备当前的版本 (提交号)；打包 base..to 之间改动的文件")
    ap.add_argument("--to", default="HEAD")
    ap.add_argument("--paths", nargs="*", help="指定文件 (取 --to 版本的内容)，可与 --base 合用")
    ap.add_argument("--version")
    ap.add_argument("--title")
    ap.add_argument("-o", "--out")
    a = ap.parse_args()
    if not a.base and not a.paths:
        ap.error("需要 --base 或 --paths")
    to = git("rev-parse", "--short", a.to).decode().strip()
    files, deletes = set(), set()
    if a.base:
        for line in git("diff", "--name-status", "--no-renames", f"{a.base}..{a.to}").decode().splitlines():
            st, path = line.split("\t", 1)
            if path.startswith(SKIP):
                continue
            (deletes if st.startswith("D") else files).add(path)
    for p in a.paths or []:
        files.add(os.path.relpath(os.path.abspath(p), ROOT) if os.path.exists(p) else p)
    version = a.version or to
    title = a.title or git("log", "-1", "--format=%s", a.to).decode().strip()
    man = {"format": "agv-update/1", "version": version, "title": title,
           "base": git("rev-parse", "--short", a.base).decode().strip() if a.base else None,
           "created": time.time(), "files": [], "delete": sorted(deletes)}
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w:gz") as tf:
        for path in sorted(files):
            data = git("show", f"{a.to}:{path}")
            mode = 0o755 if (git("ls-tree", a.to, "--", path).decode().split() or ["100644"])[0] == "100755" else 0o644
            man["files"].append({"path": path, "sha256": hashlib.sha256(data).hexdigest(), "size": len(data), "mode": mode})
            ti = tarfile.TarInfo("files/" + path); ti.size = len(data); ti.mode = mode; ti.mtime = int(time.time())
            tf.addfile(ti, io.BytesIO(data))
        mb = json.dumps(man, ensure_ascii=False, indent=1).encode()
        ti = tarfile.TarInfo("agv_update.json"); ti.size = len(mb); ti.mtime = int(time.time())
        tf.addfile(ti, io.BytesIO(mb))
    out = a.out or f"agv-update-{version}.tgz"
    with open(out, "wb") as f:
        f.write(buf.getvalue())
    print(f"{out}: 版本 {version}，{len(man['files'])} 个文件，删除 {len(deletes)} 个，{len(buf.getvalue()) / 1024:.0f} KB")


if __name__ == "__main__":
    sys.exit(main())
