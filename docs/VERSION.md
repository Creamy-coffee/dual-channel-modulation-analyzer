# 参赛版本清单

本文件说明提交参赛的三样东西（FPGA 实现、屏端程序、文档）分别对应哪个版本，以及已知的不一致处。
**README 里的能力表述与截图，以本文件列出的版本为准。**

## 一、FPGA 侧

| 项 | 值 |
|---|---|
| 目标器件 | XC7A35T FGG484-2（Artix-7 35T） |
| 工具版本 | Vivado 2018.3（win64）Build 2405991 |
| 实现 ID（USR_ACCESS） | `a222905e`（完整 `a222905e5799`） |
| bit 文件 SHA256 | `76afefdbf47d3cdb31ea5eaba13fd96468875622ede87bce231b217367e0b051` |
| RTL 源码 | `fpga/rtl/`（main 分支） |
| 约束 | `fpga/constraints/` |
| 资源 / 时序 | 20294 LUT、13111 FF、80 DSP、32×RAMB36 + 8×RAMB18；WNS +0.172 ns、WHS +0.010 ns |
| 报告原件 | `fpga/build_evidence/{utilization,timing,drc}.rpt` |

> `fpga/build.tcl` 会把 USR_ACCESS 写成 `a222905e`，因此重建后必须重新计算 bit 的 SHA256，
> 不能只凭 USR_ACCESS 判定与历史产物逐字节一致。干净环境下的完整重建尚未验收。

## 二、屏端（RK3506）

| 项 | 值 |
|---|---|
| 发布程序页面数 | 七页（示波器 / 频谱 / 锁相 / 重建 / 调制 / 系统 / 图片） |
| 源码所在分支 | `fpga-modulation-ui-20261026`，HEAD 提交 `76f7aff`（2026-10-29） |
| `main` 的 `src/` | 六页版本（无「图片」页），最后同步于 `0c11e0d` |
| 依赖 | LVGL v9.1（MIT，需外部目录）、ARMv7 hard-float 交叉工具链 |
| 截图来源 | README 与 `docs/screenshots/` 的 13 张图取自七页版本运行画面 |

**已知不一致**：`main` 的 `src/` 落后于发布程序，缺少「图片」页以及对应的判决 / EVM 显示。
该分支与 main 的历史没有共同祖先（是独立快照历史），不能直接 merge，需人工合并后再更新 main。
在合并完成前，**以分支 `fpga-modulation-ui-20261026` 的代码为屏端准据**。

## 三、文档

| 项 | 值 |
|---|---|
| 技术文档 | 《双通道 65 MSPS 信号分析仪技术文档》v1.3（2026-10-30），含图 1–23 |
| 协议 | `docs/V32CAP3_FPGA_TX_SPEC.md` |
| 截图 | `docs/screenshots/`，1024×600，13 张 |

## 四、提交前检查

- [ ] 参赛 tag 指向本文件所述的提交
- [ ] bit 文件 SHA256 与上表一致
- [ ] 屏端二进制来自「发布程序」对应的提交
- [ ] README 的 13 张截图与发布程序一致
