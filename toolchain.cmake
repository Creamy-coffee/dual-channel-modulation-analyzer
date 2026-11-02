# CMake 交叉编译工具链：RK3506（Cortex-A7，ARMv7-A hard-float）
set(CMAKE_SYSTEM_NAME      Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

# 工具链根目录不写死在仓库里（仓库是公开的，写死等于把别人的家目录发出去）。
# 取值顺序：
#   1. cmake 命令行  -DTC_ROOT=/path/to/gcc-arm-...-arm-none-linux-gnueabihf
#   2. 环境变量      export SDK_TOOLCHAIN_ROOT=/path/to/...
# 例如（RK3506 SDK 自带的正式工具链）：
#   export SDK_TOOLCHAIN_ROOT=/opt/rk3506_linux6.1_sdk_v1.2.0_iot_evb/prebuilts/gcc/linux-x86/arm/gcc-arm-10.3-2021.07-x86_64-arm-none-linux-gnueabihf
if(NOT TC_ROOT AND DEFINED ENV{SDK_TOOLCHAIN_ROOT})
    set(TC_ROOT "$ENV{SDK_TOOLCHAIN_ROOT}")
endif()
set(TC_ROOT "${TC_ROOT}" CACHE PATH "交叉编译工具链根目录（gcc-arm-*-arm-none-linux-gnueabihf）")

if(NOT TC_ROOT)
    message(FATAL_ERROR
        "未指定交叉编译工具链根目录。请设置环境变量 SDK_TOOLCHAIN_ROOT，"
        "或用 cmake -DTC_ROOT=<路径> 指定。")
endif()

set(CMAKE_C_COMPILER   "${TC_ROOT}/bin/arm-none-linux-gnueabihf-gcc")
set(CMAKE_CXX_COMPILER "${TC_ROOT}/bin/arm-none-linux-gnueabihf-g++")

# Arm 官方工具链自带的 sysroot
set(CMAKE_SYSROOT "${TC_ROOT}/arm-none-linux-gnueabihf/libc")

set(CMAKE_FIND_ROOT_PATH "${CMAKE_SYSROOT}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# RK3506 是 Cortex-A7 + NEON：ARMv7-A / hard-float
set(ARCH_FLAGS "-march=armv7-a -mtune=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard")

set(CMAKE_C_FLAGS_INIT   "${ARCH_FLAGS} -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS_INIT "${ARCH_FLAGS} -ffunction-sections -fdata-sections")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${ARCH_FLAGS} -static -Wl,--gc-sections")
