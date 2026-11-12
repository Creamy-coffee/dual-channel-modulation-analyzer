`timescale 1ns / 1ps
`default_nettype none
// =====================================================================
// fixed_divider_generic
//
// Parameterized multi-cycle restoring unsigned integer divider.
// Latency = NUM_W clock cycles after start.
//
//   quotient = floor(numerator / denominator)
//
// This is a standalone module intended to be shared by all blocks that
// need a non-combinational divider (FFT interpolation, lock-in FCW,
// THD computation, etc.).  Do NOT define local copies inside other
// files — use this one.
//
// Port sizes:
//   numerator   : NUM_W bits
//   denominator : DEN_W bits
//   quotient    : NUM_W bits  (full width to avoid truncation)
//
// AXI-style handshake:
//   - Assert start for one cycle when inputs are valid.
//   - busy stays high during computation.
//   - valid pulses for one cycle when quotient is ready.
//   - Do NOT assert start while busy == 1.
// =====================================================================
// =====================================================================
// 【模块功能概述】
//   参数化定点无符号整数除法器，采用多周期逐位试商（恢复式）算法实现。
//   输入分子（numerator）和分母（denominator），输出商（quotient）。
//   商 = floor(numerator / denominator)，即向下取整的整数除法。
//
// 【算法原理】
//   恢复式除法（Restoring Division），也叫逐位试商法：
//   1. 初始时余数 rem = 0，商 quo = numerator
//   2. 每一步将 {rem, quo} 整体左移 1 位（相当于余数乘2，带下一位被除数）
//   3. 用移位后的余数减去除数：
//      - 若够减（rem_shifted >= den），则当前商位为 1，余数更新为差值
//      - 若不够减，则当前商位为 0，余数保持移位后的值（即"恢复"）
//   4. 重复 NUM_W 次后，quo 即为最终商，rem 即为最终余数
//   这是经典的"恢复式"除法（与"非恢复式"的区别在于：不够减时余数保持不变，
//   而非恢复式除法会在下一步用加法修正）。
//
// 【参数化设计】
//   NUM_W : 分子（被除数）位宽，可灵活配置，默认 64 位
//   DEN_W : 分母（除数）位宽，可灵活配置，默认 64 位
//   两个参数独立，支持分子分母不同位宽的除法运算。
//   CNT_W : 计数器位宽，由 NUM_W 自动推导 = ceil(log2(NUM_W+1))
//
// 【状态机结构】
//   本模块采用隐式状态机，以 busy 信号作为状态标志：
//   - IDLE 状态（busy = 0）：空闲状态，等待 start 信号启动
//   - 计算状态（busy = 1）：正在进行除法迭代，通过 count 计数器记录迭代次数
//   注：代码中没有单独的 SHIFT 和 SUBTRACT 状态，移位和比较/减法在同一
//   时钟周期内完成（移位通过组合逻辑实现，减法结果在时钟沿更新）。
//
// 【迭代次数与位宽的关系】
//   N 位无符号整数除法需要 N 个时钟周期完成（即 NUM_W 个周期）。
//   每一位商需要一个周期来确定，从最高位到最低位逐位计算。
//   总延迟（Latency）= NUM_W 个时钟周期（从 start 有效到 valid 有效）。
//
// 【商和余数的输出格式】
//   quotient : NUM_W 位无符号整数，即 floor(numerator / denominator)
//   余数 rem 仅在内部计算，未作为输出端口引出。
//   若需要余数，可添加 output 端口 rem 并在 valid 有效时输出。
//
// 【除零处理】
//   本模块未内置除零保护逻辑。若 denominator = 0：
//   - q_bit 恒为 1（因为 rem_shifted >= 0 始终成立）
//   - rem_shifted - 0 = rem_shifted，余数持续左移累积
//   - 最终商全为 1（即 2^NUM_W - 1），无意义
//   建议：在上层模块中对 denominator == 0 的情况进行预判和处理。
//
// 【握手协议（start / busy / valid）】
//   采用类 AXI 握手协议，时序如下：
//   1. 上游模块在输入数据有效时，将 start 置高 1 个时钟周期
//   2. start 有效后，busy 立即变高，表示模块正在计算
//   3. 计算期间 busy 保持高电平，此时不可再次发出 start
//   4. NUM_W 个周期后计算完成，busy 变低，valid 置高 1 个周期
//   5. valid 为高时 quotient 输出有效，可被下游采样
//   约束：busy == 1 时禁止断言 start，否则会覆盖内部状态。
//
// 【资源占用】
//   纯 LUT + FF 实现，不占用乘法器（DSP）资源。
//   主要资源消耗：
//   - 比较器：NUM_W+1 位比较（判断 rem_shifted >= den）
//   - 减法器：NUM_W 位减法（rem_shifted - den）
//   - 移位逻辑：组合逻辑拼接，不消耗寄存器
//   - 寄存器：rem(NUM_W) + quo(NUM_W) + den(DEN_W) + count(CNT_W)
//              + busy(1) + valid(1) + quotient(NUM_W) ≈ 2*NUM_W + DEN_W + 若干
//   适合 FPGA 中 DSP 资源紧张但 LUT 充裕的场景。
// =====================================================================

module fixed_divider_generic #(
    parameter integer NUM_W = 64,   // 分子（被除数）位宽，默认64位
    parameter integer DEN_W = 64,   // 分母（除数）位宽，默认64位
    // Opt-in exact optimization: after every restoring step rem < den.
    // min(NUM_W,DEN_W) remainder bits suffice; comparison keeps the carry.
    // Default preserves the existing implementation for all legacy clients.
    parameter integer COMPACT_REMAINDER = 0
)(
    input  wire                 clk,        // 时钟输入，上升沿触发
    input  wire                 rst_n,      // 异步复位，低电平有效
    input  wire                 start,      // 启动信号，高电平有效（脉冲1周期）

    input  wire [NUM_W-1:0]     numerator,  // 分子（被除数），NUM_W 位无符号数
    input  wire [DEN_W-1:0]     denominator,// 分母（除数），DEN_W 位无符号数

    output reg                  busy,       // 忙信号，高电平表示正在计算
    output reg                  valid,      // 输出有效信号，高电平脉冲1周期表示商已就绪
    output reg [NUM_W-1:0]      quotient    // 商输出，NUM_W 位无符号数
);
    // -----------------------------------------------------------------
    // 局部参数：计数器位宽
    // 由 NUM_W 自动推导，确保 count 能计数到 NUM_W-1
    // $clog2(NUM_W + 1) 表示对 NUM_W+1 取以2为底的对数并向上取整
    // 例如 NUM_W=64 时，CNT_W = clog2(65) = 7
    // -----------------------------------------------------------------
    localparam integer CNT_W = $clog2(NUM_W + 1);

    // -----------------------------------------------------------------
    // 内部寄存器定义
    //   rem   : 余数寄存器，NUM_W 位，存储每一步的中间余数
    //   quo   : 商寄存器，NUM_W 位，初始装载分子，逐位移出并填入商位
    //   den   : 分母寄存器，DEN_W 位，计算期间保持不变
    //   count : 迭代计数器，CNT_W 位，记录当前迭代步数（0 ~ NUM_W-1）
    // -----------------------------------------------------------------
    localparam integer REM_W = COMPACT_REMAINDER && DEN_W<NUM_W ? DEN_W : NUM_W;
    reg [REM_W-1:0] rem;
    reg [NUM_W-1:0] quo;
    reg [DEN_W-1:0] den;
    reg [CNT_W-1:0] count;

    // -----------------------------------------------------------------
    // 组合逻辑：移位后的余数 + 当前商位计算
    //
    // rem_shifted : 将余数左移 1 位，移入商寄存器的最高位 quo[NUM_W-1]
    //               位宽为 NUM_W+1 位，防止溢出
    //               相当于 {rem, quo} 整体左移 1 位后的高 NUM_W+1 位
    //
    // q_bit       : 当前位的商值（1 位）
    //               比较移位后的余数与除数：
    //                 rem_shifted >= den ? 1 : 0
    //               注意：den 被高位补零扩展到 NUM_W+1 位后再比较
    //               补零位数 = NUM_W - DEN_W + 1 位
    // -----------------------------------------------------------------
    // 余数左移1位，移入商的最高位（共 NUM_W+1 位）
    wire [REM_W:0] rem_shifted = {rem, quo[NUM_W-1]};
    // 当前商位：移位后余数 >= 除数 则为 1，否则为 0（恢复式除法的核心判断）
    wire q_bit;
    wire [REM_W-1:0] remainder_after_subtraction;
    generate if(COMPACT_REMAINDER)begin: exact_shared_subtractor
        localparam integer WORK_W=REM_W+1>DEN_W?REM_W+1:DEN_W;
        (* use_dsp="yes" *) wire [WORK_W:0] difference=
            {{(WORK_W-REM_W){1'b0}},rem_shifted}-{{(WORK_W+1-DEN_W){1'b0}},den};
        assign q_bit=!difference[WORK_W];
        assign remainder_after_subtraction=difference[REM_W-1:0];
    end else begin: legacy_subtractor
        assign q_bit=rem_shifted>=den;
        assign remainder_after_subtraction=rem_shifted[REM_W-1:0]-den;
    end endgenerate

    // -----------------------------------------------------------------
    // 主时序逻辑：复位 + 状态机 + 除法迭代
    // 异步复位（rst_n 低电平有效），时钟上升沿触发
    // -----------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // ---------------------------------------------------------
            // 复位状态：所有寄存器清零
            // ---------------------------------------------------------
            busy     <= 1'b0;
            valid    <= 1'b0;
            rem      <= {REM_W{1'b0}};
            quo      <= {NUM_W{1'b0}};
            den      <= {DEN_W{1'b0}};
            count    <= {CNT_W{1'b0}};
            quotient <= {NUM_W{1'b0}};
        end else begin
            // ---------------------------------------------------------
            // valid 默认清零，仅在计算完成的最后一个周期置1
            // （valid 是脉冲信号，持续 1 个时钟周期）
            // ---------------------------------------------------------
            valid <= 1'b0;

            // ---------------------------------------------------------
            // IDLE → 计算状态转换
            // 条件：start 有效 且 当前不忙（防止重复启动）
            // 操作：
            //   - rem 清零（初始余数为 0）
            //   - quo 装载分子（被除数）
            //   - den 装载分母（除数）
            //   - count 清零（从第 0 步开始计数）
            //   - busy 置 1，进入计算状态
            // ---------------------------------------------------------
            if (start && !busy) begin
                rem   <= {REM_W{1'b0}};
                quo   <= numerator;
                den   <= denominator;
                count <= {CNT_W{1'b0}};
                busy  <= 1'b1;
            end
            // ---------------------------------------------------------
            // 计算状态：逐位试商迭代
            // 每个时钟周期完成 1 位商的计算
            // ---------------------------------------------------------
            else if (busy) begin
                // -----------------------------------------------------
                // 余数更新：
                //   若够减（q_bit=1），余数 = 移位后余数 - 除数
                //   若不够减（q_bit=0），余数 = 移位后余数（恢复）
                // 取低 NUM_W 位作为新的余数
                // -----------------------------------------------------
                rem  <= q_bit ? remainder_after_subtraction : rem_shifted[REM_W-1:0];

                // -----------------------------------------------------
                // 商寄存器左移 1 位，移入本轮 q_bit，保留此前计算的商位。
                // 左移后 quo[NUM_W-1] 被移出，作为下一步 rem_shifted 的输入
                // -----------------------------------------------------
                quo  <= {quo[NUM_W-2:0], q_bit};

                // -----------------------------------------------------
                // 迭代完成判断：count == NUM_W - 1 表示已完成最后一位
                // -----------------------------------------------------
                if (count == NUM_W[CNT_W-1:0] - 1'b1) begin
                    // 计算完成：busy 拉低，valid 置 1，输出最终商
                    // 最终商 = {左移后的 quo 的高 NUM_W-1 位, 最后一位商 q_bit}
                    busy     <= 1'b0;
                    valid    <= 1'b1;
                    quotient <= {quo[NUM_W-2:0], q_bit};
                end else begin
                    // 未完成：计数器加 1，继续下一次迭代
                    count <= count + 1'b1;
                end
            end
        end
    end

endmodule
`default_nettype wire
