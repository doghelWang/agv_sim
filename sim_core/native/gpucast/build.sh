#!/usr/bin/env bash
# 编译 gpucastd (GPU 射线求交服务)。手机上在 Termux 里编译 (不是 proot 容器里): bash build.sh [输出路径]
set -e
cd "$(dirname "$0")"
OUT=${1:-./gpucastd}
${CC:-cc} -O2 -Wall -o "$OUT" gpucastd.c -ldl -lm -lpthread
echo "built $OUT"
