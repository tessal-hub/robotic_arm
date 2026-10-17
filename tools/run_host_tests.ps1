$ErrorActionPreference = "Stop"
$binDir = "$PSScriptRoot\..\build_test"
if (!(Test-Path $binDir)) { New-Item -ItemType Directory -Path $binDir | Out-Null }

function Run-Step($name, $cmd, $exe) {
    Write-Host "=== $name ==="
    Invoke-Expression $cmd
    & $exe
    if ($LASTEXITCODE -ne 0) {
        Write-Error "Test failed: $name"
        exit 1
    }
}

Run-Step "kinematics" `
    "g++ -std=gnu++17 -Wall -Wextra -I test/host -I src src/kinematics.cpp test/kinematics/test_kinematics.cpp -o $binDir\kinematics_test.exe" `
    "$binDir\kinematics_test.exe"

Run-Step "drawing workspace" `
    "g++ -std=gnu++17 -Wall -Wextra -I test/host -I src src/drawing_workspace.cpp src/kinematics.cpp test/host/test_drawing_workspace.cpp -o $binDir\drawing_workspace_test.exe" `
    "$binDir\drawing_workspace_test.exe"

Run-Step "joint logic" `
    "g++ -std=gnu++17 -Wall -Wextra -I test/host -I src test/host/test_joint_logic.cpp -o $binDir\joint_logic_test.exe" `
    "$binDir\joint_logic_test.exe"

Run-Step "work plane" `
    "g++ -std=gnu++17 -Wall -Wextra -I test/host -I src src/work_plane.cpp test/host/test_work_plane.cpp -o $binDir\work_plane_test.exe" `
    "$binDir\work_plane_test.exe"

Run-Step "trajectory validator" `
    "g++ -std=c++17 -Wall -Wextra -I test/host -I src src/trajectory_validator.cpp src/kinematics.cpp src/work_plane.cpp test/host/test_trajectory_validator.cpp -o $binDir\trajectory_validator_test.exe" `
    "$binDir\trajectory_validator_test.exe"

Run-Step "homing fsm" `
    "g++ -std=gnu++17 -Wall -Wextra -I test/host -I src test/host/test_homing_fsm.cpp -o $binDir\homing_fsm_test.exe" `
    "$binDir\homing_fsm_test.exe"

Run-Step "homing logic" `
    "g++ -std=gnu++17 -Wall -Wextra -I test/host -I src test/host/test_homing_logic.cpp -o $binDir\homing_logic_test.exe" `
    "$binDir\homing_logic_test.exe"

Run-Step "safety manager" `
    "g++ -std=c++17 -Wall -Wextra -I test/host -I src test/host/test_safety_manager.cpp src/safety_manager.cpp -o $binDir\safety_manager_test.exe" `
    "$binDir\safety_manager_test.exe"

Run-Step "web validation" `
    "g++ -std=c++17 -Wall -Wextra -I test/host -I src test/host/test_web_validation.cpp -o $binDir\web_validation_test.exe" `
    "$binDir\web_validation_test.exe"

Run-Step "firmware failure regressions" `
    "g++ -std=gnu++17 -Wall -Wextra -DARDUINO -I test/host/firmware -I src test/host/test_firmware_regression.cpp src/arm.cpp src/motor.cpp src/sensor.cpp src/joint_model.cpp src/nvs_store.cpp src/endstop.cpp src/safety_manager.cpp src/homing.cpp src/planner.cpp src/kinematics.cpp src/work_plane.cpp src/trajectory_validator.cpp -o $binDir\firmware_regression_test.exe" `
    "$binDir\firmware_regression_test.exe"

Write-Host "=== ALL HOST TESTS PASSED ===" -ForegroundColor Green
