#!/usr/bin/env bash
# 编译 libagvnav (纯 C，无 Python 头文件依赖；规划 agvnav.c + 内置 SLAM 计算内核 slamcore.c)。用法: bash planning/native/build.sh [CC]
# -ffp-contract=off: 不把 a*b+c 合成 FMA (arm64 默认会)，舍入与 Python 逐步计算一致 —— 路程相同的并列路线才会选得一样
set -e
cd "$(dirname "$0")"
CC=${1:-${CC:-cc}}
EXT=so
[ "$(uname)" = "Darwin" ] && EXT=dylib
$CC -O2 -ffp-contract=off -fPIC -shared -std=gnu99 -Wall -Wextra -o "libagvnav.$EXT" agvnav.c slamcore.c -lm
echo "built $(pwd)/libagvnav.$EXT"
