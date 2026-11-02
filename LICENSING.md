# 项目许可证

本仓库项目自身的 FPGA 与 Linux 屏端代码采用 GNU General Public License version 3，SPDX 标识为 `GPL-3.0-only`。完整条款见根目录 `LICENSE`。

第三方代码、依赖、字体及随附素材保留其原有许可证和版权声明；本声明不改变其许可条件，也不授予项目贡献者无权授予的权利。分发时应同时遵守适用的第三方许可要求。

## 一、第三方组件清单

| 组件 | 版本 | 许可证 | 是否随仓库分发 | 用途 |
|---|---|---|---|---|
| LVGL | v9.1 | MIT | 否（需自行准备，`LVGL_DIR=` 指定） | 屏端 GUI 框架 |
| liquid-dsp | 1.7.0 | MIT（见 `third_party/liquid-dsp.LICENSE`） | 是，`third_party/liquid-dsp-1.7.0.tar.gz` | BPSK 符号定时、重采样、载波跟踪、均衡 |
| 中易黑体 SimHei | 随 Windows 提供 | 商业字体 | 仅位图子集，见下文 | 界面汉字 |
| DejaVu Sans | 随 fontTools / 系统提供 | Bitstream Vera 衍生自由许可 | 仅位图子集 | 拉丁字符与度量符号 |
| Segoe UI Symbol | 随 Windows 提供 | 商业字体 | 仅位图子集 | 界面装饰图标 |

`liquid-dsp` 的固定版源码在配置阶段解包到构建目录，只做四处构建系统改动（C-only 工程、静态库、去掉上游无条件 SIMD 运行期探测、给摘要打印的空标志加引号），算法与数值类型不做降精度替换；静态链接，不需要部署 `libliquid.so`。细节见 `third_party/README.md`。

## 二、字体的许可状况（需要处理）

界面字体不是现成字体文件，而是由 `tools/gen_v32_fonts.py` 从三套系统字体中**按实际用到的字符抽取位图子集**，生成 `src/fonts/lv_font_v32_*.c`：

- `simhei` → 汉字及中文标点
- `DejaVuSans` → 可打印 ASCII 与度量符号
- `seguisym` → `↺⌁⏸▥◎⚙⛶✧` 等装饰图标

其中 **SimHei 与 Segoe UI Symbol 是随 Windows 提供的商业字体**。生成位图子集再随仓库分发，是否落在这些字体的授权范围内存在争议；DejaVu Sans 是自由许可，没有这个问题。

**结论与待办**：参赛材料内部评审不存在实际影响，但作为公开开源仓库长期分发时应当替换。替换路径已具备——把 `tools/gen_v32_fonts.py` 里的 `simhei` 换成**思源黑体（Noto Sans CJK / Source Han Sans，SIL OFL 1.1）**，装饰图标改用 DejaVu Sans 或自绘矢量图标，重跑脚本后重新编译。**本版本尚未执行该替换**，因此在上表中如实标为商业字体位图子集，而不是笼统写作"自有字体"。

## 三、代码来源边界

- `fpga/rtl/`、`src/`、`tools/` 为本队编写；FFT 蝶形、CIC/FIR 抽取、NCO 混频、锁相、谐波统计、成帧与 CRC32 均为自研 RTL 与 C 代码，未调用 Xilinx FFT IP。
- `third_party/liquid-dsp` 为上游开源库，仅用于屏端 BPSK 的符号跟踪与重采样，未修改其算法实现。
- `docs/` 中的截图、报告、日志由本队在自有硬件上采集；技术文档图 1–23 为自绘。
- 仓库不包含任何厂商 SDK 源码、Vivado 许可证文件或第三方数据手册全文。

## 四、需要各自声明的权利

`LICENSING.md` 无法替代字体的授权证明。若后续替换为思源黑体，应同时把该字体的 `OFL.txt` 放入 `third_party/` 并在上表登记。
