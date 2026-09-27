#!/usr/bin/env bash
# 编译 libsimcore.so (纯 C，无 Python 头文件依赖)。用法: bash sim_core/native/build.sh [CC]
set -e
cd "$(dirname "$0")"
CC=${1:-${CC:-cc}}
EXT=so
[ "$(uname)" = "Darwin" ] && EXT=dylib
$CC -O2 -fPIC -shared -std=c99 -Wall -Wextra -Wno-unused-parameter -o "libsimcore.$EXT" simcore.c -lm
echo "built $(pwd)/libsimcore.$EXT"
