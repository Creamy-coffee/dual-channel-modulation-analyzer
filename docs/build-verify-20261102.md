# 干净环境重建验收记录（2026-11-02）

本文件记录在**仓库之外的独立副本**上，从源码重新构建 FPGA 侧与屏端程序的过程与结果。
目的只有一个：回答"换一台机器，照着仓库能不能重建出东西"。所有产物都不进仓库。

## 一、环境

| 项 | 值 |
|---|---|
| 操作系统 | Windows 10（LAPTOP-K2SKPF0L），64 位 |
| Vivado | v2018.3 (win64) Build 2405991 |
| FPGA 器件 | xc7a35tfgg484-2 |
| 交叉编译器 | `arm-linux-gnueabihf-gcc` 7.3.1（随 Xilinx SDK 2018.3 提供）|
| LVGL | v9.1.0（GitHub 标签 `v9.1.0`，`lvgl.h` 中 `LVGL_VERSION_MAJOR/MINOR/PATCH = 9/1/0`）|
| liquid-dsp | 1.7.0（仓库 `third_party/liquid-dsp-1.7.0.tar.gz`，SHA256 = `33c42ebc…4dd4`，与 `third_party/liquid.cmake` 中钉住的值一致）|
| 构建位置 | `D:\_wb_rebuild\`（仓库 `fpga/` 的完整副本 + 外部依赖），**未改动仓库工作树** |

**与正式构建流程的差异，先说清楚：**

1. 团队正式构建用 ARM 官方 `gcc-arm-10.3-2021.07` 工具链（`toolchain.cmake` 里写死的路径），
   本机没有那套工具链，用的是 Xilinx SDK 自带的 `arm-linux-gnueabihf-gcc 7.3.1`。
   两者都是 ARMv7-A hard-float，但**编译器版本不同**，产物不能直接当作发布二进制。
2. 本机没有 cmake，屏端不是走 `./build.sh`，而是用**等价的命令行**完成同样的三件事：
   编 LVGL、编应用源码、静态链接 liquid。编译选项与 `toolchain.cmake` 一致
   （`-march=armv7-a -mtune=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard`）。
3. FPGA 侧完全走仓库自带的 `fpga/build.tcl`，没有手工改脚本或参数。

## 二、屏端重建结果

| 项 | 结果 |
|---|---|
| LVGL 目标文件 | 284 / 284 全部编译成功 |
| 应用源码 | 21 / 21 全部编译成功（含 8 个位图字体文件）|
| liquid-dsp | 160 个目标文件，静态库 1,151,514 B |
| 最终可执行文件 | 2,258,844 B，`ELF 32-bit`、`Machine: ARM`、`Type: EXEC` |
| 架构与 ABI | `Tag_CPU_arch: v7`、`Tag_CPU_arch_profile: Application`、`Tag_FP_arch: VFPv4`、`Tag_ABI_VFP_args: VFP registers`（硬浮点调用约定）|
| 命令行工具 | `decode_bpsk_capture`、`decode_ook_capture`、`fsk_capture_decode`、`test_bpsk_decoder`、`test_raw_capture` 全部构建成功 |

结论：**`src/` 与 `third_party/` 的代码是自洽可编译的**，不是"能看不能编"的状态。
上一版 `docs/VERSION.md` 里"迁移后尚未编译"的缺口至此关闭，但替换工具链这一点仍未验证。

### 同一套代码在主机上跑自测

用宿主 gcc 6.2.0（同样来自 Vivado 的 msys64）把算法源码编成 Windows 程序，按仓库 `tools/`
里的自测用例实跑，结果如下。这些是**软件层**结论，不代表实机。

| 用例 | 命令 | 结果 |
|---|---|---|
| BPSK 解码 | `test_bpsk_decoder` | `BPSK_DECODER_PASS`：12 个用例（两种相位极性 × 3 个频偏档 × 4 个符号相位），每例恢复 3778–3780 bit，PN7 全部锁定（`pn=1`），`errors=0`、`checks≈3491`，EVM 0.0277–0.0338 |
| OOK 解码 | `test_ook_decoder` | `OOK_DECODER_PASS` |
| 2FSK 解码 | `test_fsk_decoder` | `FSK_DECODER_PASS` |
| FFT 三窗 | `test_fft_windows` | `Hann` / `FlatTop` 离格幅度与相邻音、空帧、边界用例全部 PASS |
| FFT + 演示源 | `test_fft` | 512 点 @ 4 kHz，峰在 101.6 / 296.9 / 500.0 Hz，幅度 0.975 / 0.44→0.63 / 0.200，三次谐波逐帧增长（调制生效）|
| 图片传输、谐波置信度 | `test_image_transport`、`test_harmonic_confidence` | 需要输入文件参数（`argc==2`），本次未提供样本，未运行 |

> `tools/test_fft.c` 原本引用了已改名的宏 `WAVE_FRAME_LEN` / `WAVE_SAMPLE_RATE`，编译不过；
> 本次按当前 `wave_source.h` 的 `frame_len` / `sample_rate` 字段修正，上表是修正后的实跑结果。

## 三、FPGA 综合结果

| 项 | 本次干净重建（综合后） | 历史报告（实现后，`fpga/build_evidence/`）|
|---|---|---|
| Slice LUTs | 20370（97.93%） | 20294 |
| LUT as Logic | 20314 | 20238 |
| LUT as Memory | 56 | 56 |
| Slice Registers | 13137 | 13111 |
| DSP48E1 | 80（88.89%） | 80 |
| Block RAM Tile | 36（32×RAMB36 + 8×RAMB18） | 36 |
| WNS | −0.466 ns（10 个失败端点） | +0.172 ns |
| WHS | −0.151 ns | +0.010 ns |

综合后时序未收敛是正常的：这套流程把收敛压力放在 `place_design`（`ExtraTimingOpt`）与
`phys_opt_design`（`ExploreWithAggressiveHoldFix`）上，综合阶段只给出悲观估计。
判定是否达标要看实现后的时序报告，见下一节。

**资源结论（与器件上限对照）**：LUT 97.93%、DSP 88.89%、BRAM 72%。这是一块 35T 上
接近跑满的状态，也是 `build.tcl` 里那些"砍调试显示、保测量功能"取舍的原因。
资源占比本身不是成绩，真正的实现难点在于这份共享结构能否在剩不到 2% 的 LUT 里塞下
分时复用的 FFT、DDC、锁相与成帧。

## 四、实现结果

`build.tcl` 一路跑到 `write_bitstream` 成功（`write_bitstream completed successfully`），
中途没有改脚本或参数。实现后的报告与历史 `fpga/build_evidence/` 对比：

| 项 | 本次干净重建 | 历史 `build_evidence/` | 是否一致 |
|---|---|---|---|
| WNS | **+0.172 ns**（0 个失败端点） | +0.172 ns | 一致 |
| WHS | **+0.010 ns**（0 个失败端点） | +0.010 ns | 一致 |
| WPWS | 6.562 ns（0 个失败端点） | — | — |
| Slice LUTs | 20294（97.57%） | 20294 | 一致 |
| LUT as Logic / Memory | 20238 / 56 | 20238 / 56 | 一致 |
| Slice Registers | 13111 | 13111 | 一致 |
| DSP48E1 | 80（88.89%） | 80 | 一致 |
| Block RAM Tile | 36（32×RAMB36 + 8×RAMB18） | 36 | 一致 |
| DRC | 138 条，**全部 Warning，无 Error** | 138 条，同样 6 条规则 | 一致（CHECK-3 2、DPIP-1 52、DPOP-1 19、DPOP-2 25、REQP-1839 20、REQP-1840 20）|
| bit 文件 | 2,192,115 B，SHA256 `0a64f12e…6af39` | SHA256 `76afefdb…0b051` | **不一致** |

**时序、资源、DRC 三项逐项复现，bit 哈希不同。** 原因在 bit 头里写着：

```
daq_top;UserID=0XFFFFFFFF;Version=2018.3 7a35tfgg484 2026/11/02 17:29:09
```

Vivado 会把构建日期时间写进 bit 头，所以同一份源码、同一版本工具在不同时刻重建，
SHA256 必然不同。**不能用 bit 的 SHA256 判断两次构建是否等价**，可用的判据是：
USR_ACCESS（`build.tcl` 设为 `0xa222905e`）、上表的时序与资源、DRC 规则分布，以及上板功能验证。

> 注意：`build.tcl` 里那两处 `phys_opt_design -directive ExploreWithAggressiveHoldFix`
> 与 `place_design -directive ExtraTimingOpt` 是收敛的关键。综合后 WNS 是 −0.466 ns，
> 靠这两步才拉到 +0.172 ns。少跑一步就会掉出时序，所以不要手工删减 flow。

## 五、这次验收没覆盖的部分

1. **没有上板**：本次只验证"能重建、时序收敛、资源与历史一致"，没有把 bit 下载到板子
   跑功能。上板验证仍是提交前必做项。
2. **屏端编译器不是正式的**：见第一节差异说明，发布二进制要用 `gcc-arm-10.3-2021.07` 重编。
3. **图片与谐波两个自测需要输入样本**：`test_image_transport`、`test_harmonic_confidence`
   以文件路径为参数（`argc==2`），本次没有对应样本，未运行。
4. **界面字体仍是 SimHei 位图子集**：换字体需要 `lv_font_conv`（Node 工具）且必须在屏上
   逐字确认，本次不动，授权现状与替换步骤见 [LICENSING.md](../LICENSING.md)。
