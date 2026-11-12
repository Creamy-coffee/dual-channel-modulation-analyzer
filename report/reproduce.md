# 从零复现

## FPGA

安装 Vivado 2018.3 与 Artix-7 支持。使用 2018.3 的原因是原工程和布线报告以该版本生成，历史独立构建见 [记录](build-verify-20261102.md)。在仓库根目录运行：

    vivado -mode batch -source build/fpga/build.tcl

器件 XC7A35T-FGG484-2，顶层 daq_top。生成文件位于 build/work/，交付参考位流位于 board/fpga/。验证报告中的 WNS、WHS、资源和 DRC，记录新构建的文件 SHA-256。

## Linux 屏端

安装 Python 3、CMake、GNU 构建工具。ARM 交叉编译时设置 V32_ARM_TOOLCHAIN_ROOT 为 RK3506 SDK 的 GNU ARM 工具链根目录：

    export V32_ARM_TOOLCHAIN_ROOT=/path/to/arm-toolchain
    bash build/linux/build.sh

构建产物为 build/work/linux/waveform_demo。省略环境变量可做主机原生构建。LVGL 与 liquid-dsp 源码依赖均随仓库提供。历史脚本见 board/linux 和 build/linux，实际部署前先核对设备节点。

## 上板

1. 使用 Vivado Hardware Manager 烧入 board/fpga/fpga_a222905e.bit。
2. 接好自制 ADC 板，串口 AB5 接 Linux RX、AA5 接 TX，并共地。FPGA 与屏端用 3.3 V 逻辑，UART 2 Mbaud、8N1。
3. 在 RK3506 备份现有应用，停止占用 DRM 的进程。将 board/linux/waveform_demo 或对应 ARM 新构建安装为 /usr/bin/waveform_demo，并把 board/linux/waveform_service.sh 安装为 /etc/init.d/S90waveform_demo。设置执行权限。写入 echo 2000000 > /etc/waveform_demo.baud，再启动服务。默认串口 /dev/ttyS1。
4. 在系统页核对有效帧数增长。图片验收可用 Windows 下的 src/pc/image_sender/server.py。需 64 位 VISA 驱动及匹配的仪器序列号。发生器 CH2 连 FPGA ADC CH0，屏端选 CH1、OOK、1000 Bd。参考位图与验收记录见 data/image 和 board/screenshots。

## 离线验证

- bash sim/run_rtl_sim.sh，需 Icarus Verilog，验证谐波 UART 命令。
- python skill/self_test.py，验证 UART 载荷校验。
- python3 src/linux/tools/run_image_transport_test.py --build build/work/linux --out build/work/image_test，需完成原生构建，运行 OOK、2FSK、BPSK 合成图像回归。

硬件证据对应随包参考位流和既有板端程序。整机指标及版本边界见 [设计报告](design_report.md)。
