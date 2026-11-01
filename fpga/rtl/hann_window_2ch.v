// Symmetric Hann: w[n]=w[N-1-n]. Valid follows ROM, product and output.
`timescale 1ns/1ps
`default_nettype none
module hann_window_2ch #(
    parameter integer LOG2_N=13,
    parameter COEFF_FILE="hann_8192_half_q15.mem",
    parameter HALF_WINDOW_OPT=1,
    parameter FRAME_GATED=0
)(
    input wire clk, rst_n,
    input wire in_valid,
    input wire frame_enable,
    input wire signed [11:0] ch0_in,ch1_in,
    output wire input_accepted,
    output reg out_valid,
    output reg signed [15:0] ch0_out,ch1_out
);
    localparam integer N=1<<LOG2_N;
    localparam integer DEPTH=HALF_WINDOW_OPT ? N/2 : N;
    (* rom_style="block" *) reg [15:0] coeff_rom[0:DEPTH-1];
    reg [LOG2_N-1:0] addr;
    wire [LOG2_N-1:0] rom_addr=(HALF_WINDOW_OPT && addr>=N/2) ? N-1-addr : addr;
    reg frame_full;
    wire accept=in_valid && (!FRAME_GATED || (frame_enable && !frame_full));
    assign input_accepted=accept;
    reg [15:0] coeff_q;
    reg signed [11:0] ch0_q,ch1_q;
    (* use_dsp="yes" *) reg signed [28:0] prod0,prod1;
    reg valid_q,valid_product;
    // The midpoint mirrors N/2-1. Read only the 4096 used words.
    initial $readmemh(COEFF_FILE,coeff_rom,0,DEPTH-1);
    // Synchronous ROM/input stage; no reset on ROM output for BRAM inference.
    always @(posedge clk) if(accept) begin
        coeff_q<=coeff_rom[rom_addr]; ch0_q<=ch0_in; ch1_q<=ch1_in;
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            addr<=0;frame_full<=0;valid_q<=0;valid_product<=0;
            prod0<=0;prod1<=0;out_valid<=0;ch0_out<=0;ch1_out<=0;
        end else begin
            valid_q<=accept;
            valid_product<=valid_q;
            out_valid<=valid_product;
            if(valid_q) begin
                prod0<=ch0_q*$signed({1'b0,coeff_q});
                prod1<=ch1_q*$signed({1'b0,coeff_q});
            end
            if(valid_product) begin ch0_out<=prod0>>>15;ch1_out<=prod1>>>15;end
            if(FRAME_GATED && !frame_enable) begin addr<=0;frame_full<=0;end
            else if(accept) begin
                if(addr==N-1) begin addr<=0;frame_full<=1;end
                else addr<=addr+1'b1;
            end
        end
    end
endmodule
`default_nettype wire
