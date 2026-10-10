#!/usr/bin/env bash
set -euo pipefail
project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$project_root"
test_build_dir="${TMPDIR:-/tmp}/vrcontroller-calibration-tests"
mkdir -p "$test_build_dir"
cc -std=c11 -Wall -Wextra -Werror -Imain \
  tests/test_calibration_math.c main/calibration_math.c main/tracker_protocol.c \
  -lm -o "$test_build_dir/math"
"$test_build_dir/math"
cc -std=c11 -Wall -Wextra -Werror -Imain \
  tests/test_calibration_protocol.c main/tracker_protocol.c \
  -lm -o "$test_build_dir/protocol"
"$test_build_dir/protocol"
cc -std=c11 -Wall -Wextra -Werror -Itests/stubs -Imain \
  tests/test_calibration_wizard.c main/calibration.c main/calibration_math.c \
  main/tracker_protocol.c -lm -o "$test_build_dir/wizard"
"$test_build_dir/wizard"
