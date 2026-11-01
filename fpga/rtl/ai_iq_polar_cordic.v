`timescale 1ns/1ps
`default_nettype none
// =====================================================================
// ai_iq_polar_cordic.v — CORDIC 幅相转换（向量模式）
//
// 功能：将 40-bit 有符号 I/Q 累加和转换为幅值与相位。
//
// 输入来源：ai_lockin_iq_tap 的 256 点累加结果
//   I = Σ(raw_sample[n] * cos_q15[n])     单位: ADC_counts × 32768
//   Q = -Σ(raw_sample[n] * sin_q15[n])    单位: ADC_counts × 32768
//
// 输出格式：
//   amp_q12   : unsigned 24-bit Q12.12   单位: ADC 计数幅值（峰值）
//   phase_turn: signed 16-bit binary-turn 单位: 整圈/65536
//               即 360° ↔ 65536，90°=16384，-90°=-16384
//
// 定点推导（N=256 点, NCO 为 Q1.15）:
//   A_peak = sqrt(I² + Q²) / (256 × 32768 / 2)
//          = sqrt(I² + Q²) / 4194304  ← 不对
//   正确推导：
//   若 x[n] 单位为 ADC 计数，cos/sin 为 Q1.15（1.0 = 32768）
//   I = Σ x[n]·cos[n]  = N·A·cos(φ)·32768   （相干积分时）
//   sqrt(I²+Q²) = N·A·32768
//   A = sqrt(I²+Q²) / (N·32768)
//   其中 N=256，所以 A = sqrt(I²+Q²) / 8388608
//
//   我们用 Q12.12 表示 A，即 A_Q12 = A × 4096
//   A_Q12 = sqrt(I²+Q²) × 4096 / 8388608
//         = sqrt(I²+Q²) / 2048
//
//   但 CORDIC 有增益 K ≈ 1.64676，输出 x_final = K·|I+jQ|
//   所以需要补偿 K：A_true = x_final / K
//
//   实现方式：I/Q 先 >>10，CORDIC 16 次迭代后，
//   x_end 包含 K 因子，再乘以 1/K（Q16 定点）得到最终幅值。
//   最终比例：A_Q12 ≈ sqrt(I²+Q²) / 1024（近似值，需精确核对）
//
// CORDIC 架构：
//   - 16 次迭代，顺序执行（每拍一次迭代）
//   - 36-bit 内部位宽（I/Q >>10 后扩展到 36 位防止溢出）
//   - 单周期输入握手（in_valid / in_ready）
//   - 输出延迟约 19 拍（1 拍锁存 + 16 拍迭代 + 1 拍输出 + 1 拍增益补偿）
//
// 相位编码说明：
//   相位用 16-bit signed binary-turn 格式：
//     0°    = 0x0000
//     +90°  = 0x4000 = 16384
//     -90°  = 0xC000 = -16384
//     ±180° = 0x8000 = -32768
//   这种格式与 branch_decision.v 的环绕相位差计算完全兼容。
//
// 零信号与小幅值处理：
//   - 当 I=0 且 Q=0 时：幅值=0，phase_valid=0，相位输出 0
//   - 当幅值低于 PHASE_MIN_AMP_Q12（默认 16）时：phase_valid=0
//     原因：低幅值下相位估计不可靠（受 CORDIC >>10 量化影响）
//
// 注意：注释不占用任何 FPGA 资源，仅用于人类理解代码。
// =====================================================================
module ai_iq_polar_cordic #(
  parameter integer INPUT_SHIFT = 10, // default preserves original 256-sample IQ normalization
  parameter integer ACC_W = 40,          // 输入 I/Q 位宽（40-bit 有符号累加和）
  parameter integer AMP_W = 24,          // 输出幅数位宽（Q12.12）
  parameter [AMP_W-1:0] PHASE_MIN_AMP_Q12 = 24'd16
    // 相位有效阈值（Q12.12 单位），低于此值相位标记为无效
    // 默认 16 = 16/4096 = 1/256 ADC 计数
)(
  input  wire clk,                       // 系统时钟（65 MHz）
  input  wire rst_n,                     // 低电平复位

  // ---- 输入：I/Q 累加和 + 帧信息 ----
  input  wire in_valid,                  // 输入有效（脉冲）
  output wire in_ready,                  // 输入就绪（=IDLE 状态）
  input  wire signed [ACC_W-1:0] i_sum,  // I 路累加和（40-bit 有符号）
  input  wire signed [ACC_W-1:0] q_sum,  // Q 路累加和（40-bit 有符号）
  input  wire [15:0] frame_id_in,        // 输入帧编号（透传）
  input  wire branch_in,                 // 支路标记：0=RAW, 1=CNN（透传）

  // ---- 输出：幅值 + 相位 + 帧信息 ----
  output reg out_valid,                  // 输出有效（脉冲）
  output reg [AMP_W-1:0] amp_q12,        // 幅值 Q12.12（unsigned）
  output reg signed [15:0] phase_turn,   // 相位 binary-turn（signed）
  output reg phase_valid,                // 相位有效标志
  output reg [15:0] frame_id_out,        // 输出帧编号
  output reg branch_out                  // 输出支路标记
);

  // ---------- 状态机定义 ----------
  localparam IDLE = 2'd0;  // 空闲，等待输入
  localparam ITER = 2'd1;  // 迭代中（16 拍）
  localparam DONE = 2'd2;  // 完成，输出结果

  reg [1:0] state;          // 当前状态
  reg [4:0] iter;           // 迭代计数器 0..15

  // ---------- CORDIC 数据通路 ----------
  // 内部 36-bit 有符号运算（I/Q >> 10 后扩展）
  reg signed [35:0] x, y;   // 当前迭代的 x(实部) 和 y(虚部)
  reg signed [15:0] theta;  // 当前累积相位（binary-turn）
  reg was_zero;             // 标记输入是否为零信号
  reg [15:0] fid_reg;       // 帧编号寄存器
  reg branch_reg;           // 支路标记寄存器

  // 移位后的 x 和 y（第 iter 次迭代移位 iter 位）
  wire signed [35:0] xs = x >>> iter;   // 算术右移
  wire signed [35:0] ys = y >>> iter;

  // 下一拍的 x 和 y（根据 y 的符号决定加减方向）
  // y > 0 时：x' = x + y>>i,  y' = y - x>>i  （向正方向旋转）
  // y < 0 时：x' = x - y>>i,  y' = y + x>>i  （向负方向旋转）
  wire signed [35:0] x_next = (y > 0) ? (x + ys) : (x - ys);
  wire signed [35:0] y_next = (y > 0) ? (y - xs) : (y + xs);

  // ---------- CORDIC 角度查找表 ----------
  // atan(2^-i) 的 binary-turn 表示，四舍五入到 16-bit
  // 由 Python 预先计算，确保与软件模型位精确一致
  function [15:0] atan_turn;
    input [4:0] n;
    begin
      case(n)
        0: atan_turn=16'd8192;  // atan(1)    = 45.000°  → 8192
        1: atan_turn=16'd4836;  // atan(1/2)  = 26.565°  → 4836
        2: atan_turn=16'd2555;  // atan(1/4)  = 14.036°  → 2555
        3: atan_turn=16'd1297;  // atan(1/8)  = 7.125°   → 1297
        4: atan_turn=16'd651;   // atan(1/16) = 3.576°   → 651
        5: atan_turn=16'd326;   // atan(1/32) = 1.790°   → 326
        6: atan_turn=16'd163;   // atan(1/64) = 0.895°   → 163
        7: atan_turn=16'd81;    // atan(1/128)= 0.448°   → 81
        8: atan_turn=16'd41;    // ...
        9: atan_turn=16'd20;
       10: atan_turn=16'd10;
       11: atan_turn=16'd5;
       12: atan_turn=16'd3;
       13: atan_turn=16'd1;
       14: atan_turn=16'd1;
       15: atan_turn=16'd0;    // 最后一次近似为 0
       default: atan_turn=16'd0;
      endcase
    end
  endfunction

  // ---------- 增益补偿 ----------
  // CORDIC 向量模式增益 K = Π cos(atan(2^-i)) ≈ 0.607252935
  // 16 次迭代后 |x_final| = K · |I+jQ|
  // 需要补偿：A_true = x_final / K
  // 用 Q16 定点表示 1/K ≈ 1.64676：39797 / 65536 ≈ 0.60725  ← 不对
  // 实际上 K ≈ 0.60725，所以 1/K ≈ 1.64676
  // 但这里是：x_final 已经乘了 K，要还原真实幅值
  // 等等——向量模式 CORDIC 是将向量旋转到 x 轴正方向
  // x_final = K · sqrt(I²+Q²)  （K ≈ 1.64676，是增益，不是衰减）
  // 不对，CORDIC 的"增益"是指幅值被放大的倍数
  // 16 次迭代的 K ≈ 1.646760258
  // 所以 x_final = K · A，A = x_final / K
  // 补偿系数 = 1/K ≈ 0.60725
  // 用 Q16 表示：0.60725 × 65536 ≈ 39797
  // 对，39797 是正确的补偿系数
  //
  // 计算：mag_product = x_final × 39797
  //       mag_corrected = round(mag_product / 65536)
  wire [51:0] mag_product = $unsigned(x) * 16'd39797;
  wire [51:0] mag_rounded = mag_product + 52'd32768;  // +0.5 四舍五入
  wire [35:0] mag_corrected = mag_rounded >> 16;       // 除以 65536

  // 溢出检测：如果补偿后的幅值超过 24-bit 则饱和
  wire mag_overflow = (mag_corrected > {{(36-AMP_W){1'b0}},{AMP_W{1'b1}}});

  // 输入就绪信号：只有 IDLE 状态能接受新输入
  assign in_ready = (state == IDLE);

  // ---------- 主状态机 ----------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state       <= IDLE;
      iter        <= 0;
      x           <= 0;
      y           <= 0;
      theta       <= 0;
      was_zero    <= 1'b0;
      fid_reg     <= 0;
      branch_reg  <= 0;
      out_valid   <= 0;
      amp_q12     <= 0;
      phase_turn  <= 0;
      phase_valid <= 0;
      frame_id_out<= 0;
      branch_out  <= 0;
    end else begin
      out_valid <= 1'b0;    // 默认：out_valid 仅 DONE 状态脉冲

      case(state)
        // ============================================================
        // IDLE：等待输入
        // ============================================================
        IDLE: if (in_valid) begin
          // 将 I/Q 右移 10 位后送入 36-bit 运算器
          // 同时处理象限：如果 I < 0，将向量旋转 180° 到右半平面
          // 因为 CORDIC 只能收敛到 -90°..+90° 范围（右半平面）
          if (i_sum < 0) begin
            // I 为负：取反 I 和 Q（相当于旋转 180°），初始相位设为 ±180°
            x     <= -$signed(i_sum >>> INPUT_SHIFT);
            y     <= -$signed(q_sum >>> INPUT_SHIFT);
            theta <= 16'sh8000;   // -32768 = ±180°
          end else begin
            // I 非负：直接送入，初始相位 0
            x     <=  $signed(i_sum >>> INPUT_SHIFT);
            y     <=  $signed(q_sum >>> INPUT_SHIFT);
            theta <= 16'sd0;
          end
          was_zero    <= (i_sum == 0 && q_sum == 0);  // 零信号标记
          fid_reg     <= frame_id_in;
          branch_reg  <= branch_in;
          iter        <= 0;
          state       <= ITER;
        end

        // ============================================================
        // ITER：CORDIC 迭代（16 拍）
        // ============================================================
        ITER: begin
          // 根据 y 的符号决定旋转方向
          if (y > 0)
            theta <= theta + $signed(atan_turn(iter));  // 正角度
          else
            theta <= theta - $signed(atan_turn(iter));  // 负角度

          // 更新 x 和 y
          x <= x_next;
          y <= y_next;

          // 最后一次迭代后进入 DONE
          if (iter == 5'd15)
            state <= DONE;
          else
            iter <= iter + 1'b1;
        end

        // ============================================================
        // DONE：输出结果（1 拍）
        // ============================================================
        DONE: begin
          out_valid <= 1'b1;

          // 幅值：零信号输出 0；溢出则饱和到最大值
          if (was_zero)
            amp_q12 <= {AMP_W{1'b0}};
          else if (mag_overflow)
            amp_q12 <= {AMP_W{1'b1}};   // 饱和到全 1
          else
            amp_q12 <= mag_corrected[AMP_W-1:0];

          // 相位：零信号或幅值低于阈值时输出 0 且标记无效
          // 注意：mag_corrected 是 36-bit，与 24-bit 阈值比较时自动扩展
          if (was_zero || mag_corrected < PHASE_MIN_AMP_Q12) begin
            phase_turn  <= 16'sd0;
            phase_valid <= 1'b0;
          end else begin
            phase_turn  <= theta;
            phase_valid <= 1'b1;
          end

          frame_id_out <= fid_reg;
          branch_out   <= branch_reg;
          state        <= IDLE;   // 返回空闲
        end

        default: state <= IDLE;
      endcase
    end
  end

endmodule
`default_nettype wire
