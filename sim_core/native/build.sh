#!/usr/bin/env bash
# 编译 libsimcore (纯 C，无 Python 头文件依赖)。用法: bash sim_core/native/build.sh [CC] [PYTHON]
#   找得到 MuJoCo 的 C 头文件 (pip 包自带 include/) 时启用 SC_WITH_MUJOCO: 实时循环直接调用 MuJoCo C API
#   (运行时 dlopen 同一个 pip 包里的 libmujoco，版本须与头文件一致)。
set -e
cd "$(dirname "$0")"
CC=${1:-${CC:-cc}}
PY=${2:-${PYTHON:-python3}}
EXT=so
[ "$(uname)" = "Darwin" ] && EXT=dylib
MJ=""
INC=$($PY -c "import mujoco, os; p = os.path.join(os.path.dirname(mujoco.__file__), 'include'); print(p if os.path.exists(os.path.join(p, 'mujoco', 'mujoco.h')) else '')" 2>/dev/null || true)
[ -n "$INC" ] && MJ="-DSC_WITH_MUJOCO -I$INC"
LIBS="-lm -lpthread"
[ "$(uname)" != "Darwin" ] && LIBS="$LIBS -ldl"
$CC -O2 -fPIC -shared -std=gnu99 -Wall -Wextra -Wno-unused-parameter $MJ -o "libsimcore.$EXT" simcore.c simcore_rt.c $LIBS
echo "built $(pwd)/libsimcore.$EXT ${MJ:+(MuJoCo C API: $INC)}"
