`timescale 1ns/1ps
`default_nettype none
// Master twiddle table for the 8192-point radix-2 FFT.
// Entry k stores {imaginary, real}, signed Q1.15.
// The FFT uses addresses 0..N/2-1. Smaller transforms can reuse the
// table by selecting the appropriate address width/stride.
module fft_twiddle_rom #(
    parameter integer ADDR_W=12,
    parameter ROM_FILE="twiddle_8192.mem"
)(
    input wire clk,
    input wire [ADDR_W-1:0] addr,
    output reg [31:0] q
);
    localparam integer DEPTH=(1<<ADDR_W);
    (* rom_style="block" *) reg [31:0] rom[0:DEPTH-1];
    initial $readmemh(ROM_FILE,rom);
    always @(posedge clk) q<=rom[addr];
endmodule
`default_nettype wire
