`timescale 1ns/1ps
`default_nettype none
// =============================================================================
// ddc_nco_mixer.v —— 数字下变频正交混频器（DDC 前端）
//
// 参考：red-pitaya-notes sdr_receiver 架构（ADC -> I/Q DDC -> 抽取 -> 主机）。
// 本模块只做“混频”：相位累加器 + sin/cos 查找表 + 与输入样点相乘，
// 输出 I/Q 基带复数流，抽取由后级 CIC/FIR 完成。
//
//   acc[31:0] 每 in_valid 拍累加 fcw（32 位频率控制字，fs=CLK_FREQ）。
//   中心频率 f_c = fcw * CLK_FREQ / 2^32。
//   LUT 深 1024（相位取高 LUT_AW 位），16bit Q15 有符号，复用现有
//   sin_lut_q15.mem / cos_lut_q15.mem（与 lockin_core_2ch 同源）。
//   乘法用 DSP48：in(16bit) * cos(16bit Q15) >> 15 -> 16bit。
//
// 流水（每拍一级）：
//   t0 相位累加 -> t1 ROM 读出 -> t2 相乘 -> t3 输出（与 in_valid 对齐后 3 拍）。
// =============================================================================
module ddc_nco_mixer #(
    parameter integer CLK_FREQ = 65_000_000,
    parameter integer LUT_AW   = 10,            // 1024-entry sin/cos ROM
    parameter integer DATA_W   = 16             // signed sample width
)(
    input  wire                        clk,
    input  wire                        rst_n,
    input  wire                        in_valid,  // ADC sample pulse
    input  wire signed [DATA_W-1:0]    in,        // sign-extended ADC sample
    input  wire [31:0]                 fcw,       // center-frequency control word
    output reg                         out_valid,
    output reg  signed [DATA_W-1:0]    i_out,     // baseband I
    output reg  signed [DATA_W-1:0]    q_out      // baseband Q
);
    localparam integer LUT_DEPTH = 1 << LUT_AW;

    // ---- phase accumulator ----
    reg [31:0] acc;
    wire [LUT_AW-1:0] idx = acc[31:32-LUT_AW];

    // ---- sin/cos ROM (reuse existing Q15 tables) ----
    (* ram_style = "block" *) reg signed [DATA_W-1:0] sin_rom [0:LUT_DEPTH-1];
    (* ram_style = "block" *) reg signed [DATA_W-1:0] cos_rom [0:LUT_DEPTH-1];
    initial begin
        $readmemh("sin_lut_q15.mem", sin_rom);
        $readmemh("cos_lut_q15.mem", cos_rom);
    end

    reg  signed [DATA_W-1:0] s_q, c_q;       // ROM stage
    reg  signed [DATA_W-1:0] in_q;           // aligned sample
    reg  [DATA_W-1:0] i_p, q_p;              // product high word (unsigned view)
    reg  [2:0] v_pipe;                       // 4-stage valid pipeline == data path

    // DSP48 multiply: in(16bit) x cos/sin(16bit Q15) -> 32bit
    (* use_dsp48 = "yes" *) reg signed [31:0] macc_i, macc_q;

    always @(posedge clk) begin
        if(!rst_n) begin
            acc <= 32'd0;
            s_q <= 16'sd0; c_q <= 16'sd0; in_q <= 16'sd0;
            i_p <= 16'd0;  q_p <= 16'd0;
            macc_i <= 32'sd0; macc_q <= 32'sd0;
            v_pipe <= 0;
            out_valid <= 1'b0;
            i_out <= 16'sd0; q_out <= 16'sd0;
        end else begin
            // stage 0: phase accumulator (acc holds the phase of THIS sample)
            if(in_valid) acc <= acc + fcw;
            // stage 1: ROM read with the CURRENT phase; align the sample
            s_q <= -sin_rom[idx];
            c_q <= cos_rom[idx];
            in_q <= in;
            // stage 2: DSP48 multiply (in x cos / in x sin)
            macc_i <= in_q * c_q;
            macc_q <= in_q * s_q;
            // stage 3: scale >>15 with rounding; input is ~12 bit so the
            // 16-bit result cannot overflow the int16 range
            i_p <= macc_i[30:15] + (macc_i[14] ? 16'd1 : 16'd0);
            q_p <= macc_q[30:15] + (macc_q[14] ? 16'd1 : 16'd0);
            // valid pipeline: 4 stages so out_valid aligns with i_out (data
            // takes 4 clocks: acc/ROM -> mult -> scale -> i_out register)
            v_pipe <= {v_pipe[1:0], in_valid};
            out_valid <= v_pipe[2];
            i_out <= $signed(i_p);
            q_out <= $signed(q_p);
        end
    end

    // synthesis translate_off
    initial begin
        if(LUT_AW < 8 || LUT_AW > 12) $fatal(1,"ddc_nco_mixer: LUT_AW out of range");
        if(DATA_W != 16) $fatal(1,"ddc_nco_mixer: DATA_W must be 16");
    end
    // synthesis translate_on
endmodule
`default_nettype wire
