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
"$KIN_OUT" || exit 1

echo "=== joint logic ==="
JOINT_OUT=/tmp/opencode/joint_logic_test
g++ -std=gnu++17 -Wall -Wextra -I test/host -I src \
    test/host/test_joint_logic.cpp \
    -o "$JOINT_OUT"
"$JOINT_OUT" || exit 1

echo "=== work plane ==="
WP_OUT=/tmp/opencode/work_plane_test
g++ -std=gnu++17 -Wall -Wextra -I test/host -I src \
    src/work_plane.cpp test/host/test_work_plane.cpp \
    -o "$WP_OUT"
"$WP_OUT" || exit 1

echo "=== trajectory validator ==="
TRAJ_OUT=/tmp/opencode/trajectory_validator_test
g++ -std=c++17 -Wall -Wextra -I test/host -I src \
    src/trajectory_validator.cpp src/kinematics.cpp src/work_plane.cpp test/host/test_trajectory_validator.cpp \
    -o "$TRAJ_OUT"
"$TRAJ_OUT" || exit 1

echo "=== homing logic ==="
HOMING_OUT=/tmp/opencode/homing_logic_test
g++ -std=gnu++17 -Wall -Wextra -I test/host -I src \
    test/host/test_homing_logic.cpp \
    -o "$HOMING_OUT"
"$HOMING_OUT" || exit 1

echo "=== safety manager ==="
SAFETY_OUT=/tmp/opencode/safety_manager_test
g++ -std=c++17 -Wall -Wextra -I test/host -I src \
    test/host/test_safety_manager.cpp src/safety_manager.cpp \
    -o "$SAFETY_OUT"
"$SAFETY_OUT" || exit 1

echo "=== homing nonblocking ==="
HNB_OUT=/tmp/opencode/homing_nonblocking_test
g++ -std=c++17 -Wall -Wextra -I test/host -I src \
    test/host/test_homing_nonblocking.cpp \
    -o "$HNB_OUT"
"$HNB_OUT" || exit 1

echo "=== ALL HOST TESTS PASSED ==="

