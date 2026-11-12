`timescale 1ns/1ps
`default_nettype none
// Same-clock true dual-port RAM; no reset on the memory or read registers.
// The caller consumes RAM output only after a full frame has been written,
// and never writes both ports to the same address. Reads may be undefined
// during reset/capture; those values are not used. Synchronous read ports
// permit block RAM inference.
module fft_tdp_ram #(
    parameter integer AW=11
)(
    input wire clk,
    input wire [AW-1:0] addr_a, addr_b,
    input wire we_a, we_b,
    input wire [31:0] din_a, din_b,
    output reg [31:0] dout_a, dout_b
);
    (* ram_style="block" *) reg [31:0] mem[0:(1<<AW)-1];
    always @(posedge clk) begin
        if(we_a) mem[addr_a]<=din_a;
        dout_a<=mem[addr_a];
    end
    always @(posedge clk) begin
        if(we_b) mem[addr_b]<=din_b;
        dout_b<=mem[addr_b];
    end
endmodule
`default_nettype wire
