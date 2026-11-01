`timescale 1ns/1ps
`default_nettype none
// ---------------------------------------------------------------------------
// 可变抽取器（块平均 / boxcar），给 AB5 波形帧做"自动采样率"。
//
// R = 2^decim_log2，R 取 1..32768。每 R 个输入点输一个点，值是这 R 点的
// 算术平均（加半个 LSB 做四舍五入，算术右移）。R=1 时直通。
//
// 时间戳不在这里维护：v32cap3_tx 的 start_index 是自由运行的 ADC 时钟计数，
// 与抽取比无关，所以插入本模块不改变帧关联与时间轴语义。Linux 侧按帧头的
// decimation 字段换算等效采样率（fs/R），不需要改任何代码。
//
// 注意：这是数字抽取，不是抗混叠。混叠在 ADC 采样那一刻就已经发生，
// 本模块只能保证时间轴和显示窗口正确，不能恢复被混叠污染的低频成分。
// ---------------------------------------------------------------------------
module wave_decim #(
    parameter integer MAX_LOG2 = 15,   // 最大 R = 2^MAX_LOG2
    parameter integer SAMPLE_W = 12
)(
    input  wire                       clk,        // ADC 采样时钟域
    input  wire                       rst_n,
    input  wire                       sample_valid,   // 输入点有效
    input  wire signed [SAMPLE_W-1:0] ch0,
    input  wire signed [SAMPLE_W-1:0] ch1,
    input  wire [1:0]                 otr,           // 该点 ADC 溢出
    input  wire [3:0]                 decim_log2,    // 0..MAX_LOG2
    output reg                        out_valid,     // 每 R 点一次，单拍
    output reg  signed [SAMPLE_W-1:0] out0,
    output reg  signed [SAMPLE_W-1:0] out1,
    output reg                        out_fault      // 本块内出现过 otr
);
    // MAX_LOG2 退化为 0 时仍要有一个可用的位宽
    localparam integer LG    = (MAX_LOG2 < 1) ? 1 : MAX_LOG2;
    localparam integer ACC_W = SAMPLE_W + LG + 1;

    reg  [LG-1:0]      cnt;
    reg  signed [ACC_W-1:0] acc0, acc1;
    reg                otr_acc;
    reg  [3:0]         lg_q;

    // 本块的最后一个计数值 R-1。decim_log2 只在帧边界缓慢变化，
    // 这里用组合逻辑实时算，避免额外状态。
    wire [LG-1:0] last = ({{(LG-1){1'b0}}, 1'b1} << decim_log2) - 1'b1;

    wire signed [ACC_W-1:0] s0 = {{(ACC_W-SAMPLE_W){ch0[SAMPLE_W-1]}}, ch0};
    wire signed [ACC_W-1:0] s1 = {{(ACC_W-SAMPLE_W){ch1[SAMPLE_W-1]}}, ch1};
    wire signed [ACC_W-1:0] sum0 = acc0 + s0;
    wire signed [ACC_W-1:0] sum1 = acc1 + s1;

    // 四舍五入：加 R/2 后算术右移 log2(R)。R=1 时不加。
    wire signed [ACC_W-1:0] rnd = (decim_log2 == 4'd0)
                                ? {ACC_W{1'b0}}
                                : ({{(ACC_W-1){1'b0}}, 1'b1} << (decim_log2 - 4'd1));

    // 平均值严格落在输入范围内，不需要饱和：所有输入 <= 2047 时
    // (2047*R + R/2) >> log2R = 2047 + 0，不会进位到 2048。
    wire signed [ACC_W-1:0] avg0 = (sum0 + rnd) >>> decim_log2;
    wire signed [ACC_W-1:0] avg1 = (sum1 + rnd) >>> decim_log2;

    // 换取比的那一拍丢弃当前块。不这样做的话，旧档位（R 大）已经积累的点
    // 会按新档位（R 小）相除，均值被放大到满量程之外：实测 R 从 256 切到
    // 128 时出过 2839，截断到 12 位回绕成 -1257 的坏点。
    wire lg_changed = (decim_log2 != lg_q);

    // 防御性饱和：上面已证明正常路径不会溢出，这里只是保证换档、
    // 参数异常等情况下绝不回绕。
    localparam signed [ACC_W-1:0] VMAX = (1 << (SAMPLE_W-1)) - 1;
    localparam signed [ACC_W-1:0] VMIN = -(1 << (SAMPLE_W-1));
    wire signed [ACC_W-1:0] cl0 = (avg0 > VMAX) ? VMAX : ((avg0 < VMIN) ? VMIN : avg0);
    wire signed [ACC_W-1:0] cl1 = (avg1 > VMAX) ? VMAX : ((avg1 < VMIN) ? VMIN : avg1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lg_q      <= 4'd0;
            cnt       <= {LG{1'b0}};
            acc0      <= {ACC_W{1'b0}};
            acc1      <= {ACC_W{1'b0}};
            otr_acc   <= 1'b0;
            out_valid <= 1'b0;
            out0      <= {SAMPLE_W{1'b0}};
            out1      <= {SAMPLE_W{1'b0}};
            out_fault <= 1'b0;
        end else begin
            lg_q      <= decim_log2;
            out_valid <= 1'b0;
            if (lg_changed) begin
                cnt     <= {LG{1'b0}};
                acc0    <= {ACC_W{1'b0}};
                acc1    <= {ACC_W{1'b0}};
                otr_acc <= 1'b0;
            end else if (sample_valid) begin
                // 用 >= 而不是 ==：抽取比在块中间被调小时也能立刻收尾，
                // 不会卡住不输出。
                if (cnt >= last) begin
                    out0      <= cl0[SAMPLE_W-1:0];
                    out1      <= cl1[SAMPLE_W-1:0];
                    out_fault <= otr_acc | (|otr);
                    out_valid <= 1'b1;
                    cnt       <= {LG{1'b0}};
                    acc0      <= {ACC_W{1'b0}};
                    acc1      <= {ACC_W{1'b0}};
                    otr_acc   <= 1'b0;
                end else begin
                    cnt     <= cnt + 1'b1;
                    acc0    <= sum0;
                    acc1    <= sum1;
                    otr_acc <= otr_acc | (|otr);
                end
            end
        end
    end
endmodule
`default_nettype wire
