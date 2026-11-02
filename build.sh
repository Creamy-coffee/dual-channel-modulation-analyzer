#!/bin/bash
# 在 Ubuntu 主机上交叉编译。
#
#   ./build.sh          增量编译（改完代码重编只要几秒）
#   CLEAN=1 ./build.sh  全量重编（LVGL 约 1 分钟）
#
# 交叉工具链根目录从环境变量取，仓库里不写死任何人的本机路径：
#   export SDK_TOOLCHAIN_ROOT=/path/to/gcc-arm-10.3-2021.07-x86_64-arm-none-linux-gnueabihf
set -e
cd "$(dirname "$0")"

BUILD_DIR=${BUILD_DIR:-build}
LVGL_DIR=${LVGL_DIR:-$(cd .. && pwd)/lvgl-release-v9.1}
TC_ROOT=${TC_ROOT:-${SDK_TOOLCHAIN_ROOT:-}}

echo "LVGL 源码: $LVGL_DIR"
echo "工具链根目录: ${TC_ROOT:-<未设置>}"

if [ -z "$TC_ROOT" ]; then
    echo "请先 export SDK_TOOLCHAIN_ROOT=<gcc-arm-*-arm-none-linux-gnueabihf 的路径>" >&2
    exit 1
fi

if [ "x$CLEAN" = "x1" ]; then
    echo "==> 清理 $BUILD_DIR/"
    rm -rf "$BUILD_DIR"
fi

if [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
    echo "==> 配置 CMake"
    cmake -S . -B "$BUILD_DIR" \
          -DCMAKE_TOOLCHAIN_FILE=toolchain.cmake \
          -DCMAKE_BUILD_TYPE=Release \
          -DTC_ROOT="$TC_ROOT" \
          -DLVGL_DIR="$LVGL_DIR"
fi

echo "==> 编译"
cmake --build "$BUILD_DIR" -j"$(nproc)"

mkdir -p ../out
for artifact in waveform_demo ook_capture_decode fsk_capture_decode bpsk_capture_decode; do
    cp -f "$BUILD_DIR/$artifact" ../out/
done

echo
echo "================ 产物 ================"
ls -la ../out/waveform_demo
file ../out/waveform_demo 2>/dev/null || true
