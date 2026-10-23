#!/bin/sh
# cargo linker: zig cc cross-links to aarch64 glibc 2.35 (phone chroot = Ubuntu 22.04)
ZIG=${ZIG:-/tmp/claude-0/work/zig/ziglang/zig}
args=""
for a in "$@"; do
  case "$a" in
    --target=*) ;;
    *) args="$args '$a'";;
  esac
done
eval exec "$ZIG" cc -target aarch64-linux-gnu.2.35 $args
