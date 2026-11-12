`timescale 1ns / 1ps
// ============================================================================
// 模块名称：isqrt_u64
// 功能概述：64位无符号整数开方器（Integer Square Root）
//           计算输入被开方数的整数平方根，结果向下取整
//           result = floor(sqrt(radicand))
//
// 算法原理：逐位试探法（非恢复式开方算法，Non-Restoring Square Root）
//           从最高位到最低位逐位确定平方根的每一位是0还是1
//           每次迭代尝试将当前位设为1，若试算结果不超过剩余被开方数则保留该位
//
// 数学原理：基于完全平方公式 (a + b)² = a² + 2ab + b²
//           设当前已确定的平方根高位部分为 q，剩余余数为 r
//           每次左移两位被开方数（等效于处理下两位），尝试将 q 左移一位并末位置1
//           即 trial = 2*q + 1，若 r >= trial 则该位为1，同时 r = r - trial
//           否则该位为0，r 保持不变
//
// 迭代次数：共需 32 次迭代，得到 32 位的整数平方根结果
//           （64位被开方数的整数平方根最大为32位）
//
// 状态机结构：
//   - IDLE 状态：空闲等待，busy=0，检测 start 信号启动计算
//   - CALC 状态：计算中，busy=1，逐位迭代计算平方根
//   - DONE 状态：计算完成，busy=0，done=1，输出有效结果
//
// 输入输出位宽和格式：
//   - 输入 radicand：64位无符号整数，被开方数，范围 [0, 2^64 - 1]
//   - 输出 result：  32位无符号整数，整数平方根，范围 [0, 2^32 - 1]
//   - 输入 start：   1位，启动信号，高电平有效
//   - 输出 busy：    1位，忙信号，高电平表示正在计算
//   - 输出 done：    1位，完成信号，高电平脉冲表示计算完成
//
// 精度说明：仅输出整数部分，精确到个位，无小数位
//           结果为向下取整的整数平方根，即 result² <= radicand < (result+1)²
//
// 握手协议：valid-ready 握手变体
//   - 上游通过 start 信号发起请求（start 脉冲触发）
//   - 模块通过 busy 信号表示是否可接收新请求（busy=0 时可接收）
//   - 模块通过 done 信号输出完成脉冲，同时 result 有效
//   - 使用方式：在 busy=0 时给出 radicand 并置 start=1，等待 done 脉冲
//
// 资源占用：纯组合逻辑 + 寄存器实现，无需乘法器/DSP 资源
//   - 寄存器：x_work(64bit) + r_work(66bit) + q_work(32bit) +
//             count(6bit) + result(32bit) + busy(1bit) + done(1bit)
//             总计约 200 个触发器
//   - 组合逻辑：移位、比较、减法运算，全部由 LUT 实现
//   - 无乘法器、无 DSP、无 Block RAM 占用
//
// 延迟（Latency）：start 信号后 32 个时钟周期输出结果
// ============================================================================
module isqrt_u64 #(
    // Exact optional implementation: before the last iteration r < 2^32;
    // shifting in the final two bits therefore needs at most 34 bits.
    // Default leaves all existing clients unchanged.
    parameter integer COMPACT_REMAINDER=0
)(
    input  wire        clk,       // 时钟输入，上升沿触发
    input  wire        rst_n,     // 异步复位，低电平有效
    input  wire        start,     // 启动信号，高电平有效，需在 busy=0 时置位
    input  wire [63:0] radicand,  // 被开方数，64位无符号整数
    output reg  [31:0] result,    // 开方结果，32位无符号整数，done=1时有效
    output reg         busy,      // 忙标志，高电平表示正在计算中
    output reg         done       // 完成标志，高电平脉冲，表示计算完成且result有效
);

// ---------------------------------------------------------------------------
// 内部寄存器定义
// ---------------------------------------------------------------------------
reg [63:0] x_work;  // 被开方数工作寄存器，每次迭代左移2位，逐次取出高2位参与运算
localparam integer REM_W=COMPACT_REMAINDER?34:66;
reg [REM_W-1:0] r_work;
reg [31:0] q_work;  // 平方根工作寄存器，保存已确定的平方根各位
reg [5:0]  count;   // 迭代计数器，0~31，共32次迭代

// ---------------------------------------------------------------------------
// 组合逻辑：迭代计算的中间信号
// ---------------------------------------------------------------------------

// r_shift：余数左移2位，并拼接被开方数的最新2位（x_work的高2位）
//          等效于将余数乘以4，加上被开方数的下2位，参与下一轮试算
//          位宽66位：原余数66位左移2位+2位新数据=66位
wire [REM_W-1:0] r_shift =
    {r_work[REM_W-3:0],x_work[63:62]};

// q_shift：当前平方根结果左移1位，为下一位的试探做准备
//          等效于 q = q * 2，腾出最低位用于试算
wire [31:0] q_shift = q_work << 1;

// trial：试探值，将 q_shift 最低位置1，即 trial = 2*q + 1
//        对应公式 (q + 1)² - q² = 2q + 1，用于判断该位是否可取1
//        位宽33位，避免溢出
wire [32:0] trial    = {q_shift, 1'b1};

// take：试位结果，若 r_shift >= trial，则该位可以取1
//       即如果余数足够大，能容纳试探值 2q+1，则将当前位设为1
wire [REM_W:0] difference;
generate if(COMPACT_REMAINDER)begin: exact_shared_subtractor
    (* use_dsp="yes" *) wire [REM_W:0] dsp_difference={1'b0,r_shift}-{{(REM_W-32){1'b0}},trial};
    assign difference=dsp_difference;
end else begin: legacy_subtractor
    assign difference={1'b0,r_shift}-{{(REM_W-32){1'b0}},trial};
end endgenerate
wire take = !difference[REM_W];

// r_next：下一拍余数值
//         若 take=1，余数减去试探值（非恢复式），否则余数保持不变
//         位宽66位，与 r_shift 一致
wire [REM_W-1:0] r_next =
    take ? difference[REM_W-1:0] : r_shift;

// q_next：下一拍平方根值
//         若 take=1，最低位置1（该位确定为1），否则保持为0
wire [31:0] q_next =
    take ? (q_shift | 32'd1) : q_shift;

// ---------------------------------------------------------------------------
// 时序逻辑：状态机与迭代控制
// ---------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // 异步复位：所有寄存器清零
        x_work <= 64'd0;   // 被开方数工作寄存器清零
        r_work <= 66'd0;   // 余数寄存器清零
        q_work <= 32'd0;   // 平方根工作寄存器清零
        count  <= 6'd0;    // 迭代计数器清零
        result <= 32'd0;   // 输出结果寄存器清零
        busy   <= 1'b0;    // 忙标志清零，处于空闲状态
        done   <= 1'b0;    // 完成标志清零
    end else begin
        done <= 1'b0;      // done 为脉冲信号，默认清零

        if (start && !busy) begin
            // ------------------------------
            // IDLE -> CALC：启动新的计算
            // ------------------------------
            x_work <= radicand;  // 加载被开方数到工作寄存器
            r_work <= 66'd0;     // 余数初始化为0
            q_work <= 32'd0;     // 平方根初始化为0
            count  <= 6'd0;      // 迭代计数从0开始
            busy   <= 1'b1;      // 置忙，进入计算状态
        end
        else if (busy) begin
            // ------------------------------
            // CALC 状态：迭代计算
            // ------------------------------
            x_work <= x_work << 2;  // 被开方数左移2位，下一次取高2位参与运算
            r_work <= r_next;       // 更新余数
            q_work <= q_next;       // 更新平方根

            if (count == 6'd31) begin
                // 第31次迭代完成（共32次迭代，count从0到31）
                result <= q_next;   // 锁存最终结果到输出寄存器
                busy   <= 1'b0;     // 清除忙标志，回到空闲状态
                done   <= 1'b1;     // 输出完成脉冲，通知下游数据有效
            end else begin
                // 未完成，继续下一次迭代
                count <= count + 1'b1;  // 迭代计数器加1
            end
        end
    end
end

endmodule
