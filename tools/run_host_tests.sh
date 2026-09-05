#!/usr/bin/env bash
# Host unit tests (kinematics + joint/calibration logic + work plane). No hardware required.
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p /tmp/opencode

echo "=== kinematics ==="
KIN_OUT=/tmp/opencode/kinematics_test
g++ -std=gnu++17 -Wall -Wextra -I test/host -I src \
    src/kinematics.cpp src/differential_wrist.cpp test/kinematics/test_kinematics.cpp \
    -o "$KIN_OUT"
"$KIN_OUT" || exit 1

echo "=== drawing workspace ==="
DRAWING_WORKSPACE_OUT=/tmp/opencode/drawing_workspace_test
g++ -std=gnu++17 -Wall -Wextra -I test/host -I src \
    src/drawing_workspace.cpp src/kinematics.cpp test/host/test_drawing_workspace.cpp \
    -o "$DRAWING_WORKSPACE_OUT"
"$DRAWING_WORKSPACE_OUT" || exit 1

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

echo "=== drift policy ==="
DRIFT_OUT=/tmp/opencode/drift_policy_test
g++ -std=c++17 -Wall -Wextra -I test/host -I src \
    test/host/test_drift_policy.cpp \
    -o "$DRIFT_OUT"
"$DRIFT_OUT" || exit 1

echo "=== web API contract ==="
WEB_API_OUT=/tmp/opencode/web_api_contract_test
g++ -std=c++17 -Wall -Wextra -I test/host -I src \
    test/host/test_web_api_contract.cpp \
    -o "$WEB_API_OUT"
"$WEB_API_OUT" || exit 1

echo "=== web validation ==="
WEB_VALIDATION_OUT=/tmp/opencode/web_validation_test
g++ -std=c++17 -Wall -Wextra -I test/host -I src \
    test/host/test_web_validation.cpp \
    -o "$WEB_VALIDATION_OUT"
"$WEB_VALIDATION_OUT" || exit 1

echo "=== sensor snapshot contract ==="
SENSOR_SNAPSHOT_OUT=/tmp/opencode/sensor_snapshot_contract_test
g++ -std=c++17 -Wall -Wextra -I test/host -I src \
    test/host/test_sensor_snapshot_contract.cpp \
    -o "$SENSOR_SNAPSHOT_OUT"
"$SENSOR_SNAPSHOT_OUT" || exit 1

echo "=== NVS contract ==="
NVS_OUT=/tmp/opencode/nvs_contract_test
g++ -std=c++17 -Wall -Wextra -I test/host -I src \
    test/host/test_nvs_contract.cpp \
    -o "$NVS_OUT"
"$NVS_OUT" || exit 1

echo "=== ISR and stop contract ==="
ISR_OUT=/tmp/opencode/isr_stop_contract_test
g++ -std=c++17 -Wall -Wextra -I test/host -I src \
    test/host/test_isr_latency.cpp src/safety_manager.cpp \
    -o "$ISR_OUT"
"$ISR_OUT" || exit 1

echo "=== ALL HOST TESTS PASSED ==="

