#!/usr/bin/env bash
set -euo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
python3 "$root/build/linux/prepare_linux.py"
build_dir=${BUILD_DIR:-"$root/build/work/linux"}
args=()
if [[ -n "${V32_ARM_TOOLCHAIN_ROOT:-}" ]]; then
    args+=("-DCMAKE_TOOLCHAIN_FILE=$root/build/linux/toolchain_arm.cmake")
fi
cmake -S "$root/src/linux" -B "$build_dir" "${args[@]}"
cmake --build "$build_dir" --parallel "${V32_BUILD_JOBS:-4}"
