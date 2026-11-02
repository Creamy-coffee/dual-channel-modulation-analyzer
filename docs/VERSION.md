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
| 源码位置 | `src/`（main 分支） |
| 内容来源 | 分支 `fpga-modulation-ui-20261026` 提交 `76f7aff`（2026-10-29） |
| 依赖 | LVGL v9.1（MIT，需外部目录）、liquid-dsp 1.7.0（随仓库提供于 `third_party/`）、ARMv7 hard-float 交叉工具链 |
| 截图来源 | README 与 `docs/screenshots/` 的 13 张图取自该版本运行画面 |

**迁移说明**：该分支与 main 没有共同祖先（独立快照历史），不能 `git merge`。本次以
`git checkout <branch> -- src/ third_party/ CMakeLists.txt build.sh tools/` 的方式把屏端内容
落到 main，因此 main 上 `src/` 的文件内容与该分支一致，但提交历史不同源。

**迁移后尚未验证的一项**：本次文件对齐**没有**在交叉工具链上重新编译。已做的静态核对为
——`CMakeLists.txt` 列出的 28 个源文件全部存在；`src/`、`tools/` 的本地 `#include` 除
`liquid.h`（`third_party/`）与 `lvgl.h`（外部 LVGL 目录）外全部可解析。
**首次在板子上部署前，请先用 `LVGL_DIR=<你的 LVGL 路径> ./build.sh` 完整编译一次。**

## 三、文档

| 项 | 值 |
|---|---|
| 技术文档 | 《双通道 65 MSPS 信号分析仪技术文档》v1.3（2026-10-30），含图 1–23 |
| 协议 | `docs/V32CAP3_FPGA_TX_SPEC.md` |
| 截图 | `docs/screenshots/`，1024×600，13 张 |
| 解调实现记录 | `docs/{ook,fsk,bpsk}-bit-decoder-20261026.md`、`docs/bpsk-acquisition-capture-20261027.md` |
| 图片接收记录 | `docs/image-receiver-20261027.md`、`docs/images-128x256-20261029.md`、`docs/landscape-images-20261028.md` |
| 频谱与谐波记录 | `docs/spectrum-resolution-20261029.md`、`docs/spectrum-geometry-20261029.md`、`docs/harmonic-confidence-20261029.md`、`docs/display-clarity-harmonics-20261028.md` |

## 四、提交前检查

- [ ] 参赛 tag 指向本文件所述的提交
- [ ] bit 文件 SHA256 与上表一致
- [ ] 屏端二进制来自「发布程序」对应的提交
- [ ] README 的 13 张截图与发布程序一致
- [ ] 在交叉工具链上完整编译一次屏端程序，确认产物为 ARMv7 hard-float
