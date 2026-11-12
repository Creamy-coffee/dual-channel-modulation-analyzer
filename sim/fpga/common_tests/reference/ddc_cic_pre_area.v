`timescale 1ns/1ps
`default_nettype none
// =============================================================================
// ddc_cic_decimator.v —— 三级 CIC 抽取器（DDC 抽取级 1）
//
// 参考：red-pitaya-notes sdr_receiver 的抽取链 + 本工程 cic3_decimate_2ch.v
// 的流水结构（积分器 DSP48 映射、梳状流水、模算术全增益）。
//
//   参数  LOG2_R = 10 -> R = 1024（用户方案总抽取 4096 = 1024 x 4）
//   输入  16bit I/Q 复数流（ddc_nco_mixer 输出）
//   输出  16bit I/Q，输出率 = 65 MS/s / 1024 = 63476.5625 sps
//   增益  R^3 = 2^30 在输出端归一化（右移 30 + 舍入 + 饱和）
//   位宽  W = 16 + 30 = 46（DSP48 ALU 48bit 内可用）
//
// 积分器在输入率域、梳状在输出率域，与 cic3_decimate_2ch 一致；I/Q 两路
// 完全并行（6 个 DSP48 积分器）。
// =============================================================================
(* use_dsp = "yes" *) module ddc_cic_pre_area #(
    parameter integer DATA_W = 16,
    parameter integer LOG2_R = 10
)(
    input  wire                      clk,
    input  wire                      rst_n,
    input  wire                      clear,     // flush integrators (config change)
    input  wire                      in_valid,
    input  wire signed [DATA_W-1:0]  i_in,
    input  wire signed [DATA_W-1:0]  q_in,
    output reg                       out_valid,
    output reg  signed [DATA_W-1:0]  i_out,
    output reg  signed [DATA_W-1:0]  q_out
);
    localparam integer GAIN_BITS = 3 * LOG2_R;      // 30
    localparam integer W         = DATA_W + GAIN_BITS; // 46
    localparam integer R         = 1 << LOG2_R;

    reg [LOG2_R-1:0] count;
    reg [GAIN_BITS+2:0] warm;                        // settle after 4R samples

    (* use_dsp = "yes" *) reg signed [W-1:0] ii[0:2], qi[0:2];   // integrators
    reg signed [W-1:0] iz[0:2], qz[0:2];             // comb delays
    reg signed [W-1:0] ic0,ic1,qc0,qc1;
    reg v0, v1;

    wire take = in_valid && (&count);
    wire signed [W-1:0] ext_i = {{GAIN_BITS{i_in[DATA_W-1]}}, i_in};
    wire signed [W-1:0] ext_q = {{GAIN_BITS{q_in[DATA_W-1]}}, q_in};
    wire signed [W-1:0] last_i = ic1 - iz[2];
    wire signed [W-1:0] last_q = qc1 - qz[2];

    function signed [DATA_W-1:0] norm;
        input signed [W-1:0] value;
        reg signed [W:0] wide, q;
        begin
            wide = $signed({value[W-1], value});
            if(wide >= 0) q = (wide + (1 << (GAIN_BITS-1))) >>> GAIN_BITS;
            else          q = -(((-wide) + (1 << (GAIN_BITS-1))) >>> GAIN_BITS);
            if(q >  ((1 << (DATA_W-1))-1)) norm = {1'b0, {(DATA_W-1){1'b1}}};
            else if(q < -(1 << (DATA_W-1))) norm = {1'b1, {(DATA_W-1){1'b0}}};
            else norm = q[DATA_W-1:0];
        end
    endfunction

    integer j;
    always @(posedge clk) begin
        if(!rst_n || clear) begin
            count <= 0; warm <= 0; v0 <= 0; v1 <= 0;
            out_valid <= 0; i_out <= 0; q_out <= 0;
            ic0 <= 0; ic1 <= 0; qc0 <= 0; qc1 <= 0;
            for(j=0;j<3;j=j+1) begin ii[j] <= 0; qi[j] <= 0; iz[j] <= 0; qz[j] <= 0; end
        end else begin
            v0 <= take; v1 <= v0; out_valid <= 0;
            if(in_valid) begin
                count <= count + 1'b1;
                ii[0] <= ii[0] + ext_i;  qi[0] <= qi[0] + ext_q;
                ii[1] <= ii[1] + ii[0];  qi[1] <= qi[1] + qi[0];
                ii[2] <= ii[2] + ii[1];  qi[2] <= qi[2] + qi[1];
                if(warm < 4*R) warm <= warm + 1'b1;
            end
            if(take) begin
                ic0 <= ii[2] - iz[0];  qc0 <= qi[2] - qz[0];
                iz[0] <= ii[2];        qz[0] <= qi[2];
            end
            if(v0) begin
                ic1 <= ic0 - iz[1];    qc1 <= qc0 - qz[1];
                iz[1] <= ic0;          qz[1] <= qc0;
            end
            if(v1) begin
                iz[2] <= ic1;          qz[2] <= qc1;
                i_out <= norm(last_i);
                q_out <= norm(last_q);
                out_valid <= (warm >= 4*R);
            end
        end
    end

    // synthesis translate_off
    initial begin
        if(LOG2_R < 2 || LOG2_R > 10) $fatal(1,"ddc_cic_decimator: LOG2_R out of range");
        if(DATA_W != 16) $fatal(1,"ddc_cic_decimator: DATA_W must be 16");
    end
    // synthesis translate_on
endmodule
`default_nettype wire
