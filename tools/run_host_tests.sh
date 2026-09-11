#!/usr/bin/env bash
# Host unit tests (kinematics + joint/calibration logic + work plane). No hardware required.
set -e
cd "$(dirname "$0")/.."
mkdir -p /tmp/opencode

run_bin() {
    if [ -f "$1.exe" ]; then
        "$1.exe"
    else
        "$1"
    fi
}

echo "=== kinematics ==="
KIN_OUT=/tmp/opencode/kinematics_test
g++ -std=gnu++17 -Wall -Wextra -I test/host -I src \
    src/kinematics.cpp test/kinematics/test_kinematics.cpp \
    -o "$KIN_OUT"
run_bin "$KIN_OUT" || exit 1

echo "=== drawing workspace ==="
DRAWING_WORKSPACE_OUT=/tmp/opencode/drawing_workspace_test
g++ -std=gnu++17 -Wall -Wextra -I test/host -I src \
    src/drawing_workspace.cpp src/kinematics.cpp test/host/test_drawing_workspace.cpp \
    -o "$DRAWING_WORKSPACE_OUT"
run_bin "$DRAWING_WORKSPACE_OUT" || exit 1

echo "=== joint logic ==="
JOINT_OUT=/tmp/opencode/joint_logic_test
g++ -std=gnu++17 -Wall -Wextra -I test/host -I src \
    test/host/test_joint_logic.cpp \
    -o "$JOINT_OUT"
run_bin "$JOINT_OUT" || exit 1

echo "=== work plane ==="
WP_OUT=/tmp/opencode/work_plane_test
g++ -std=gnu++17 -Wall -Wextra -I test/host -I src \
    src/work_plane.cpp test/host/test_work_plane.cpp \
    -o "$WP_OUT"
run_bin "$WP_OUT" || exit 1

echo "=== trajectory validator ==="
TRAJ_OUT=/tmp/opencode/trajectory_validator_test
g++ -std=c++17 -Wall -Wextra -I test/host -I src \
    src/trajectory_validator.cpp src/kinematics.cpp src/work_plane.cpp test/host/test_trajectory_validator.cpp \
    -o "$TRAJ_OUT"
run_bin "$TRAJ_OUT" || exit 1

echo "=== homing logic ==="
g++ -std=gnu++17 -Wall -Wextra -I test/host -I src test/host/test_homing_fsm.cpp -o /tmp/opencode/homing_fsm_test
run_bin /tmp/opencode/homing_fsm_test || exit 1
HOMING_OUT=/tmp/opencode/homing_logic_test
g++ -std=gnu++17 -Wall -Wextra -I test/host -I src \
    test/host/test_homing_logic.cpp \
    -o "$HOMING_OUT"
run_bin "$HOMING_OUT" || exit 1

echo "=== safety manager ==="
SAFETY_OUT=/tmp/opencode/safety_manager_test
g++ -std=c++17 -Wall -Wextra -I test/host -I src \
    test/host/test_safety_manager.cpp src/safety_manager.cpp \
    -o "$SAFETY_OUT"
run_bin "$SAFETY_OUT" || exit 1

echo "=== web validation ==="
WEB_VALIDATION_OUT=/tmp/opencode/web_validation_test
g++ -std=c++17 -Wall -Wextra -I test/host -I src \
    test/host/test_web_validation.cpp \
    -o "$WEB_VALIDATION_OUT"
run_bin "$WEB_VALIDATION_OUT" || exit 1

echo "=== ALL HOST TESTS PASSED ==="
