#!/usr/bin/env bash
# Host unit tests cho kinematics (không cần hardware).
# Mở rộng: gọi tools/run_host_tests.sh (kinematics + joint logic + work plane).
set -euo pipefail
exec "$(dirname "$0")/run_host_tests.sh"
