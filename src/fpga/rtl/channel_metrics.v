`timescale 1ns / 1ps
`default_nettype none
// ============================================================================
// 模块名称：channel_metrics_fix
// 模块功能：单通道波形指标计算（定点数版本）
//
// 功能概述：
//   本模块对输入的 ADC 采样数据流进行分窗统计，每个窗口包含 2^LOG2_N 个采样点，
//   窗口之间互不重叠（non-overlapping windows）。对每个窗口计算以下三项核心指标：
//
//     1. 均值 (mean)  —— 信号的直流分量（DC offset）
//        物理意义：一个周期内信号的平均电平，反映信号的直流偏置大小。
//        计算方法：mean = floor(sum(x_i) / N)，即累加和右移 LOG2_N 位（定点除法）。
//
//     2. 峰峰值 (Vpp) —— 信号的最大摆幅
//        物理意义：正峰值与负峰值之间的差值，反映信号幅度的峰到峰范围。
//        计算方法：Vpp = max_sample - min_sample，遍历窗口内所有样本取极值后相减。
//        峰值检测算法：
//          - 正峰值（max_sample）：每来一个新样本，与当前最大值比较，取较大者更新。
//          - 负峰值（min_sample）：每来一个新样本，与当前最小值比较，取较小者更新。
//          - 窗口第一个样本（count==0）直接作为初始最大/最小值。
//
//     3. 交流有效值 (rms_ac) —— 信号交流分量的均方根值
//        物理意义：将交流信号在一个周期内的发热效应等效为相同数值的直流信号，
//                  即"等效直流发热值"。有效值是信号能量的度量。
//        计算方法：RMS = sqrt( (N * sum(x_i^2) - sum(x_i)^2) / N^2 )
//                  该公式直接由 sum((x_i - mean)^2) / N 推导而来，
//                  避免了先截断均值再平方带来的精度损失（尤其在直流偏置较大时）。
//                  具体计算由子模块 rms_ac_calc 完成。
//
// 注意：本模块不包含过零检测与频率估算功能。若需频率测量，需在外部另设过零检测
//       或 FFT 等频率分析模块。
//
// 定点格式说明：
//   - sample        : DATA_W 位有符号整数，直接对应 ADC 采样码值
//   - sum_acc       : DATA_W + LOG2_N 位有符号整数，N 个样本的累加和
//   - sumsq_acc     : 64 位无符号整数，N 个样本的平方和
//   - mean          : DATA_W 位有符号整数，均值（与采样位宽一致）
//   - vpp           : DATA_W + 1 位无符号整数，峰峰值（最大值-最小值，必为正）
//   - rms_ac        : DATA_W 位无符号整数，交流有效值（与采样位宽一致）
//
// 刷新周期与数据有效机制：
//   - 每个窗口包含 N = 2^LOG2_N 个采样点，sample_valid 每脉冲一次推进一个样本。
//   - 窗口采集完成后（第 N 个 sample_valid），将均值、峰峰值锁存到 pending 寄存器，
//     同时将 sum 和 sumsq 送入 rms_ac_calc 子模块开始 RMS 计算。
//   - RMS 计算完成后，若下游 ready，则将所有指标（mean/vpp/rms_ac/window_id）
//     一起输出并置位 metrics_valid；若下游未就绪，则保持输出直到被读取。
//   - 若窗口采集完成时 RMS 计算器仍忙（window_ready=0），则该窗口被丢弃，
//     window_dropped 脉冲一个时钟周期，window_id 跳号。
//   - window_id 从 1 开始递增，每个完整输出的窗口对应一个唯一 ID。
//
// 接口协议：
//   - sample_valid 是采样选通信号（strobe），不是带反压的 AXI 接口。
//   - 输出端 metrics_valid / metrics_ready 为标准 valid/ready 握手协议。
// ============================================================================
module channel_metrics_fix #(
    parameter integer LOG2_N = 11,   // 窗口大小的以2为底的对数，N = 2^LOG2_N，默认2048点
    parameter integer DATA_W = 12    // 采样数据位宽，默认12位（对应12位ADC）
)(
    input  wire               clk,           // 系统时钟
    input  wire               rst_n,         // 异步复位，低电平有效
    input  wire               sample_valid,  // 采样有效选通，高电平时 sample 被采入
    input  wire signed [DATA_W-1:0] sample,  // 输入采样数据，有符号定点数
    output reg  signed [DATA_W-1:0] mean,    // 输出：窗口均值（直流分量），有符号
    output reg  [DATA_W:0]        vpp,       // 输出：窗口峰峰值（正峰值-负峰值），无符号
    output reg  [DATA_W-1:0]        rms_ac,  // 输出：窗口交流有效值（AC RMS），无符号
    output reg                metrics_valid, // 输出：指标数据有效信号，握手用
    input  wire               metrics_ready, // 输入：下游就绪信号，握手用
    output reg  [31:0]        window_id,     // 输出：窗口序号，从1开始递增
    output wire               window_ready,  // 输出：RMS计算器空闲，可以接受新窗口
    output wire               calc_busy,     // 输出：RMS计算正在进行中
    output reg                window_dropped,// 输出：窗口丢弃脉冲，因RMS计算器忙而丢窗
    output reg                arithmetic_error // 输出：算术错误标志（RMS计算溢出等）
);
    // -----------------------------------------------------------------------
    // 内部参数与信号定义
    // -----------------------------------------------------------------------
    // 累加和位宽 = 数据位宽 + LOG2_N，保证 N 个 DATA_W 位数相加不溢出
    localparam integer SUM_W = DATA_W + LOG2_N;

    // 窗口内采样计数，满 N 个即完成一窗
    reg [LOG2_N-1:0] count;
    // 采样累加和（有符号），用于计算均值
    reg signed [SUM_W-1:0] sum_acc;
    // 采样平方和（无符号，64位），用于计算 RMS
    reg [63:0] sumsq_acc;
    // 窗口内最大/最小采样值，用于计算峰峰值
    reg signed [DATA_W-1:0] max_sample, min_sample;
    // 已处理窗口总数（含被丢弃的），32位防止溢出
    reg [31:0] window_count;
    // 暂存寄存器：等待 RMS 计算完成后一起输出
    reg signed [DATA_W-1:0] pending_mean;  // 待输出的均值
    reg [DATA_W:0] pending_vpp;            // 待输出的峰峰值
    reg [31:0] pending_id;                 // 待输出的窗口ID

    // -----------------------------------------------------------------------
    // 组合逻辑：各项指标的下一状态计算
    // -----------------------------------------------------------------------

    // 将采样值符号扩展到累加和位宽，便于有符号加法
    wire signed [SUM_W-1:0] sample_ext =
        {{LOG2_N{sample[DATA_W-1]}}, sample};

    // 累加和的下一值：当前和 + 符号扩展后的新样本
    wire signed [SUM_W-1:0] sum_next = sum_acc + sample_ext;

    // 当前样本的平方（有符号乘法，结果为正）
    wire signed [2*DATA_W-1:0] sample_sq_signed = sample * sample;

    // 平方和的下一值：64位零扩展后累加
    wire [63:0] sumsq_next = sumsq_acc + {{(64-2*DATA_W){1'b0}}, sample_sq_signed};

    // 正峰值检测：窗口第一个样本直接赋值，否则取较大者
    wire signed [DATA_W-1:0] max_next =
        ((count == 0) || (sample > max_sample)) ? sample : max_sample;

    // 负峰值检测：窗口第一个样本直接赋值，否则取较小者
    wire signed [DATA_W-1:0] min_next =
        ((count == 0) || (sample < min_sample)) ? sample : min_sample;

    // 将最大/最小值扩展到 DATA_W+1 位（带符号位扩展），用于相减计算 Vpp
    wire signed [DATA_W:0] max_ext = {max_next[DATA_W-1], max_next};
    wire signed [DATA_W:0] min_ext = {min_next[DATA_W-1], min_next};

    // 峰峰值 = 最大值 - 最小值（结果必为非负，故可用无符号表示）
    wire signed [DATA_W:0] vpp_next = max_ext - min_ext;

    // 均值计算：累加和右移 LOG2_N 位 = sum / N（向下取整的定点除法）
    // 使用算术右移 >>> 保证有符号数的正确符号扩展
    wire signed [SUM_W-1:0] mean_wide = sum_next >>> LOG2_N;

    // 窗口最后一个样本标志：count 全1表示计到 N-1，下一个 sample_valid 将完成窗口
    wire last_sample = &count;

    // 提交窗口标志：当采样有效且是窗口最后一个样本时，窗口数据完整
    wire submit_window = sample_valid && last_sample;

    // RMS 子模块输出有效信号
    wire rms_valid;
    // RMS 子模块就绪信号：当前输出寄存器空闲或下游已读取
    wire rms_ready = !metrics_valid || metrics_ready;
    // RMS 计算结果（交流有效值）
    wire [DATA_W-1:0] calculated_rms;
    // RMS 计算算术错误标志
    wire rms_error;

    // -----------------------------------------------------------------------
    // RMS 计算子模块实例化
    // 完成一窗数据的精确整数 AC RMS 计算
    // -----------------------------------------------------------------------
    rms_ac_calc #(.LOG2_N(LOG2_N), .DATA_W(DATA_W)) u_rms (
        .clk(clk), .rst_n(rst_n),
        .in_valid(submit_window),    // 输入：窗口完成，提交计算
        .in_ready(window_ready),     // 输出：子模块空闲，可以接受新窗口
        .sum_in(sum_next),           // 输入：窗口累加和
        .sumsq_in(sumsq_next),       // 输入：窗口平方和
        .out_valid(rms_valid),       // 输出：RMS结果有效
        .out_ready(rms_ready),       // 输入：下游就绪，可以读取结果
        .rms_ac(calculated_rms),     // 输出：RMS计算结果
        .arithmetic_error(rms_error),// 输出：计算过程中是否发生算术错误
        .busy(calc_busy)             // 输出：计算正在进行中
    );

    // -----------------------------------------------------------------------
    // 主时序逻辑：窗口采集、指标暂存、输出握手
    // -----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 异步复位：所有寄存器清零
            count <= 0; sum_acc <= 0; sumsq_acc <= 0;
            max_sample <= 0; min_sample <= 0;
            window_count <= 0;
            pending_mean <= 0; pending_vpp <= 0; pending_id <= 0;
            mean <= 0; vpp <= 0; rms_ac <= 0; window_id <= 0;
            metrics_valid <= 0; window_dropped <= 0;
            arithmetic_error <= 0;
        end else begin
            // 默认窗口丢弃脉冲为低（每个周期清除，需要时单脉冲置高）
            window_dropped <= 1'b0;

            // 输出握手：若数据有效且下游就绪，则清除有效标志
            if (metrics_valid && metrics_ready)
                metrics_valid <= 1'b0;

            // RMS 计算完成且输出就绪：将暂存的指标一起推送到输出端口
            if (rms_valid && rms_ready) begin
                mean <= pending_mean;           // 输出均值
                vpp <= pending_vpp;             // 输出峰峰值
                rms_ac <= calculated_rms;       // 输出交流有效值
                window_id <= pending_id;        // 输出窗口ID
                arithmetic_error <= rms_error;  // 输出算术错误标志
                metrics_valid <= 1'b1;          // 置位输出有效
            end

            // 采样有效时更新窗口内统计量
            if (sample_valid) begin
                if (last_sample) begin
                    // 这是窗口的最后一个样本（第N个），sum_next/vpp_next 已包含全部N个样本
                    if (window_ready) begin
                        // RMS 计算器空闲，将均值、峰峰值、窗口ID暂存，等待 RMS 结果
                        pending_mean <= mean_wide[DATA_W-1:0];  // 截取均值低 DATA_W 位
                        pending_vpp <= vpp_next[DATA_W:0];      // 峰峰值
                        pending_id <= window_count + 32'd1;      // 窗口ID从1开始
                    end else begin
                        // RMS 计算器仍忙，无法接受新窗口，丢弃本窗口数据
                        window_dropped <= 1'b1;
                    end
                    // 窗口计数递增（无论是否丢弃都递增，保证ID连续语义）
                    window_count <= window_count + 32'd1;
                    // 清零累加器和极值寄存器，为下一窗口做准备
                    count <= 0; sum_acc <= 0; sumsq_acc <= 0;
                    max_sample <= 0; min_sample <= 0;
                end else begin
                    // 非最后一个样本，正常累加和更新极值
                    count <= count + 1'b1;      // 采样计数+1
                    sum_acc <= sum_next;         // 更新累加和
                    sumsq_acc <= sumsq_next;     // 更新平方和
                    max_sample <= max_next;      // 更新正峰值
                    min_sample <= min_next;      // 更新负峰值
                end
            end
        end
    end

    // -----------------------------------------------------------------------
    // 仿真断言：参数合法性检查（仅仿真时生效，综合时被移除）
    // -----------------------------------------------------------------------
    // synthesis translate_off
    initial begin
        if (LOG2_N < 1 || LOG2_N > 16 || DATA_W < 2 || DATA_W > 24)
            $fatal(1, "channel_metrics_fix requires LOG2_N in 1..16");
    end
    // synthesis translate_on
endmodule
`default_nettype wire


// ============================================================================
// 模块名称：rms_ac_calc
// 模块功能：精确整数交流有效值（AC RMS）计算器
//
// 功能概述：
//   本模块接收一个完整窗口的累加和 (sum_in) 与平方和 (sumsq_in)，通过以下公式
//   计算信号的交流分量均方根值（AC RMS）：
//
//                    sqrt( N * sum(x_i^2) - sum(x_i)^2 )
//       RMS_ac  =  ---------------------------------------
//                                  N
//
//   即 RMS = floor( sqrt( (N*sum_sq - sum^2) / N^2 ) )
//
// 计算原理说明：
//   - 标准 RMS 公式：RMS = sqrt( sum(x_i^2) / N )
//   - AC RMS（去除直流分量后的有效值）：RMS_ac = sqrt( sum( (x_i - mean)^2 ) / N )
//   - 展开推导：sum((x_i-mean)^2) = sum(x_i^2) - 2*mean*sum(x_i) + N*mean^2
//                                   = sum(x_i^2) - sum(x_i)^2 / N
//   - 因此：RMS_ac = sqrt( (N*sum(x_i^2) - sum(x_i)^2) / N^2 )
//   - 这种"先算整体再减去均值平方"的方法，避免了先截断均值再平方带来的精度损失，
//     尤其在直流偏置较大而交流信号较小时，精度优势非常明显。
//
// 物理意义：
//   - RMS（均方根值）表征信号的能量大小，等同于产生相同热量的直流信号幅值。
//   - AC RMS 是去掉直流分量后的纯交流能量，反映信号中交流成分的强度。
//
// 定点格式与位宽设计：
//   - sum_in     : DATA_W + LOG2_N 位有符号，N 个样本的累加和
//   - sumsq_in   : 64 位无符号，N 个样本的平方和
//   - sum_squared: 2*(DATA_W+LOG2_N) 位，sum_in 的平方
//   - scaled_sumsq: 64 + LOG2_N 位，sumsq_in * N（左移 LOG2_N 位实现）
//   - numerator  : 64 + LOG2_N 位，分子 = N*sum_sq - sum^2
//   - variance   : 右移 2*LOG2_N 位后得到方差（即 RMS^2）
//   - rms_ac     : DATA_W 位无符号，开方后的结果（截取低 DATA_W 位）
//
// 计算流程（状态机）：
//   IDLE       : 空闲状态，等待输入数据
//   MULTIPLY   : 计算 sum^2（使用 DSP 资源）
//   SUBTRACT   : 计算 N*sum_sq - sum^2（即分子），若结果为负则报错
//   START_SQRT : 启动整数开方运算
//   WAIT_SQRT  : 等待开方完成（isqrt_u64 需要32个周期）
//   OUTPUT_RESULT: 输出结果，等待下游读取
//
// 依赖模块：isqrt_u64（32次迭代的整数平方根计算器）
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none
module rms_ac_calc #(
    parameter integer LOG2_N = 11,  // 窗口大小的以2为底的对数
    parameter integer DATA_W = 12   // 采样数据位宽
)(
    input  wire                       clk,            // 系统时钟
    input  wire                       rst_n,          // 异步复位，低电平有效
    input  wire                       in_valid,       // 输入数据有效
    output wire                       in_ready,       // 输入就绪（空闲时可接收）
    input  wire signed [DATA_W+LOG2_N-1:0] sum_in,    // 输入：窗口累加和（有符号）
    input  wire [63:0]                sumsq_in,       // 输入：窗口平方和（无符号，64位）
    output wire                       out_valid,      // 输出结果有效
    input  wire                       out_ready,      // 下游就绪
    output reg  [DATA_W-1:0]                rms_ac,   // 输出：交流有效值（无符号）
    output reg                        arithmetic_error,// 输出：算术错误标志
    output wire                       busy            // 输出：计算忙信号
);
    // -----------------------------------------------------------------------
    // 参数定义
    // -----------------------------------------------------------------------
    localparam integer SUM_W  = DATA_W + LOG2_N;  // 累加和位宽
    localparam integer PROD_W = 2 * SUM_W;          // sum 平方后的位宽
    localparam integer NUM_W  = 64 + LOG2_N;        // 分子位宽（=sumsq左移LOG2_N位）

    // -----------------------------------------------------------------------
    // 状态机状态定义
    // -----------------------------------------------------------------------
    localparam [2:0] IDLE=0,         // 空闲，等待输入
                     MULTIPLY=1,     // 计算 sum^2
                     SUBTRACT=2,     // 计算分子 N*sum_sq - sum^2
                     START_SQRT=3,   // 启动开方运算
                     WAIT_SQRT=4,    // 等待开方完成
                     OUTPUT_RESULT=5;// 输出结果，等待握手

    // -----------------------------------------------------------------------
    // 内部寄存器定义
    // -----------------------------------------------------------------------
    reg [2:0] state;                  // 状态机当前状态
    reg signed [SUM_W-1:0] sum_hold;  // 锁存的输入累加和
    // sum 的平方结果，使用 DSP 资源实现乘法（综合器提示用DSP48）
    (* use_dsp = "yes" *) reg [PROD_W-1:0] sum_squared;
    reg [NUM_W-1:0] scaled_sumsq;     // 左移 LOG2_N 位后的平方和（即 N * sum_sq）
    reg [NUM_W-1:0] numerator;        // 分子 = scaled_sumsq - sum_squared
    reg [63:0] sqrt_value;            // 送入开方器的被开方数（方差，即 RMS^2）
    reg sqrt_start;                   // 开方启动脉冲
    wire [31:0] sqrt_result;          // 开方结果（32位）
    wire sqrt_done;                   // 开方完成标志
    wire sqrt_busy_unused;            // 开方忙信号（本模块未使用，由 done 替代）

    // -----------------------------------------------------------------------
    // 组合逻辑
    // -----------------------------------------------------------------------

    // 有符号乘法：sum_hold * sum_hold，结果位宽 2*SUM_W
    wire signed [PROD_W-1:0] square_signed = sum_hold * sum_hold;

    // 将平方结果零扩展到分子位宽 NUM_W（平方必为非负，故零扩展）
    wire [NUM_W-1:0] square_extended =
        {{(NUM_W-PROD_W){1'b0}}, sum_squared};

    // 方差（RMS^2）= 分子 / N^2 = 分子右移 2*LOG2_N 位
    wire [NUM_W-1:0] variance_wide = numerator >> (2*LOG2_N);

    // 状态编码对应的握手信号
    assign in_ready = (state == IDLE);        // 空闲时可以接受输入
    assign out_valid = (state == OUTPUT_RESULT); // 输出状态时数据有效
    assign busy = (state != IDLE);            // 非空闲即为忙

    // -----------------------------------------------------------------------
    // 整数开方器实例化
    // 64位被开方数，32次迭代，32个周期出结果
    // -----------------------------------------------------------------------
    isqrt_u64 u_sqrt (
        .clk(clk), .rst_n(rst_n), .start(sqrt_start),
        .radicand(sqrt_value),    // 被开方数（64位）
        .result(sqrt_result),     // 开方结果（32位，取整）
        .busy(sqrt_busy_unused),  // 忙信号（未使用）
        .done(sqrt_done)          // 完成标志
    );

    // -----------------------------------------------------------------------
    // 主时序逻辑：状态机控制 RMS 计算流程
    // -----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 异步复位
            state <= IDLE;
            sum_hold <= 0;
            sum_squared <= 0;
            scaled_sumsq <= 0;
            numerator <= 0;
            sqrt_value <= 0;
            sqrt_start <= 0;
            rms_ac <= 0;
            arithmetic_error <= 0;
        end else begin
            sqrt_start <= 1'b0;  // 默认开方启动为低（脉冲式）
            case (state)
                // ---- 状态：空闲，等待输入 ----
                IDLE: if (in_valid) begin
                    // 锁存输入数据
                    sum_hold <= sum_in;
                    // 平方和左移 LOG2_N 位 = sumsq_in * N
                    scaled_sumsq <= {sumsq_in, {LOG2_N{1'b0}}};
                    arithmetic_error <= 1'b0;  // 清除错误标志
                    state <= MULTIPLY;          // 进入乘法状态
                end

                // ---- 状态：计算 sum^2 ----
                MULTIPLY: begin
                    // 计算累加和的平方，完整保留 2*SUM_W 位乘积
                    sum_squared <= square_signed[PROD_W-1:0];
                    state <= SUBTRACT;  // 进入减法状态
                end

                // ---- 状态：计算分子 N*sum_sq - sum^2 ----
                SUBTRACT: begin
                    if (scaled_sumsq >= square_extended) begin
                        // 正常情况：N*sum_sq >= sum^2，分子非负
                        numerator <= scaled_sumsq - square_extended;
                    end else begin
                        // 异常情况：理论上不可能发生（除非数据溢出或不一致）
                        // 若发生，将分子置0并标记算术错误
                        numerator <= 0;
                        arithmetic_error <= 1'b1;
                    end
                    state <= START_SQRT;  // 进入启动开方状态
                end

                // ---- 状态：启动开方运算 ----
                START_SQRT: begin
                    // 取方差的低 64 位送入开方器
                    // （方差 = 分子 / N^2，已经通过右移实现）
                    sqrt_value <= variance_wide[63:0];
                    sqrt_start <= 1'b1;     // 发出开方启动脉冲
                    state <= WAIT_SQRT;     // 进入等待开方状态
                end

                // ---- 状态：等待开方完成 ----
                WAIT_SQRT: if (sqrt_done) begin
                    // 开方完成，检查结果是否超出 DATA_W 位范围
                    if (|sqrt_result[31:DATA_W]) begin
                        // 高位有1，结果超出位宽，饱和输出全1并标记错误
                        rms_ac <= {DATA_W{1'b1}};
                        arithmetic_error <= 1'b1;
                    end else begin
                        // 结果在范围内，截取低 DATA_W 位输出
                        rms_ac <= sqrt_result[DATA_W-1:0];
                    end
                    state <= OUTPUT_RESULT;  // 进入输出状态
                end

                // ---- 状态：输出结果，等待握手 ----
                OUTPUT_RESULT: if (out_ready) begin
                    // 下游已读取，返回空闲状态
                    state <= IDLE;
                end

                // 默认：异常情况回到空闲
                default: state <= IDLE;
            endcase
        end
    end

    // -----------------------------------------------------------------------
    // 仿真断言：参数合法性检查
    // -----------------------------------------------------------------------
    // synthesis translate_off
    initial begin
        if (LOG2_N < 1 || LOG2_N > 16 || DATA_W < 2 || DATA_W > 24)
            $fatal(1, "rms_ac_calc requires LOG2_N in 1..16");
    end
    // synthesis translate_on
endmodule
`default_nettype wire
