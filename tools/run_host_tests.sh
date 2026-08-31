#!/usr/bin/env bash
# Host unit tests (kinematics + joint logic + work plane). No hardware required.
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p /tmp/opencode

echo "=== kinematics ==="
KIN_OUT=/tmp/opencode/kinematics_test
g++ -std=gnu++17 -Wall -Wextra -I test/host -I src \
    src/kinematics.cpp src/differential_wrist.cpp test/kinematics/test_kinematics.cpp \
    -o "$KIN_OUT"
"$KIN_OUT"

echo "=== joint logic ==="
JOINT_OUT=/tmp/opencode/joint_logic_test
g++ -std=gnu++17 -Wall -Wextra -I test/host -I src \
    test/host/test_joint_logic.cpp \
    -o "$JOINT_OUT"
"$JOINT_OUT"

echo "=== work plane ==="
WP_OUT=/tmp/opencode/work_plane_test
g++ -std=gnu++17 -Wall -Wextra -I test/host -I src \
    src/work_plane.cpp test/host/test_work_plane.cpp \
    -o "$WP_OUT"
"$WP_OUT"

echo "=== homing logic ==="
HOMING_OUT=/tmp/opencode/homing_logic_test
g++ -std=gnu++17 -Wall -Wextra -I test/host -I src \
    test/host/test_homing_logic.cpp \
    -o "$HOMING_OUT"
"$HOMING_OUT"

echo "=== ALL HOST TESTS PASSED ==="

