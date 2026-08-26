#!/usr/bin/env bash
# Host unit tests cho kinematics (không cần hardware).
# Lý do không dùng `pio test -e native`: PIO Core 6.1.19 báo "Nothing to build"
# với env native của dự án này (xem docs/IMPLEMENTATION_LOG.md).
set -euo pipefail
cd "$(dirname "$0")/.."
OUT=/tmp/opencode/kinematics_test
mkdir -p /tmp/opencode
g++ -std=gnu++17 -Wall -Wextra -I src \
    src/kinematics.cpp test/kinematics/test_kinematics.cpp \
    -o "$OUT"
"$OUT"
