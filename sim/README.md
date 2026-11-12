# 迁移后的验证入口

- bash sim/run_rtl_sim.sh 用 Icarus Verilog 检查 UART 命令。
- python3 sim/fpga/test_fft.py 对两个不同频点的合成 ADC 输入执行 FFT 与谐波测试，并用独立 DFT 比对；结果写到 build/work/harmonic_fft。
- python3 sim/compare_image_screen.py 用 Pillow 复核板测截图与参考位图的 2048 个像素。
- Linux 解码回归和构建步骤见 [复现指南](../report/reproduce.md)。

fpga/common_tests 归档了多个历史 testbench；部分历史驱动依赖当时的外部参考包。独立复跑从以上入口开始。
