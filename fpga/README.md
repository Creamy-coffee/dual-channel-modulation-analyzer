# 双通道65 MSPS FPGA 源码

对应实现源 ID：`a222905e5799`，目标器件 `XC7A35T FGG484 -2`。本目录由实际参赛工程整理，RTL及约束与对应实现分支逐文件核对一致。仓库根目录仍是 RK3506 屏端程序。

## 目录

- `rtl/`：双通道 ADC、4096 点共享 FFT、多速率处理、DDC、锁相、谐波功率统计与 UART。
- `constraints/`：引脚、时钟、CDC 和保持约束。
- `tests/`、`common_tests/`：仿真平台及历史辅助测试源码。
- `build.tcl`：按本目录位置创建工程、实现并生成 bit 的 Vivado 入口。
- `build_evidence/`：历史资源、时序和 DRC 报告；其中历史 Tcl 保留原机路径，仅供追溯。
- `candidate.json`：历史构建快照，下载状态应另查实际读回记录。
- `SHA256SUMS.json`：上传前文件哈希（不包含本说明及清单自身）。

## 构建

在已安装 Vivado 的终端运行：

```sh
vivado -mode batch -source fpga/build.tcl
```

原工程使用 Vivado 2018.3。脚本会创建 `fpga/build` 和 `fpga/reports`；不自动烧录。新机器上的完整重建尚未验收。脚本写入历史 USR_ACCESS `a222905e`，因此重建后必须重新计算 bit SHA256，不能仅靠 USR_ACCESS 认定与历史产物逐字节相同。

历史实际 bit SHA256：`76afefdbf47d3cdb31ea5eaba13fd96468875622ede87bce231b217367e0b051`。布线报告：20294 LUT、80 DSP、36 块等效 BRAM，WNS +0.172 ns，WHS +0.010 ns。

## 接口和边界

AB5 为 FPGA TX，AA5 为 FPGA RX，2 Mbaud / 8N1；模拟输入须满足实际 ADC 前端要求。波形、测量、频谱、调制分析和谐波结果按独立帧传输，Linux 负责页面及选路数字判决。H1～H5、有限阶 THD 已有指定输入功能测试，尚非可溯源失真校准；QPSK 历史 PC 恢复不等于屏端 bit 已实现。

历史 `test*.py` 和部分辅助测试依赖原工程布局、工具或输入数据，需要按当前目录调整，不能批量直接运行。本次上传没有重新烧录、改变屏端部署或声称全部测试重跑通过。源码历史归属与第三方声明应继续保留，本次未额外授予新的开源许可证。
