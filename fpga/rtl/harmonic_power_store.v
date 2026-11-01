`timescale 1ns/1ps
`default_nettype none
// Exact result storage in spare BRAM, avoiding a 600-bit variable LUT mux.
// There is no RAM reset: all five words in all banks are written before a
// completed result is published. The result-hold handshake forbids writes
// during transmission. UART byte intervals cover the one-clock read latency.
module harmonic_power_store(
 input wire clk,write_valid,
 input wire [2:0] write_index,read_index,
 input wire [39:0] write_band,write_noise,write_signal,
 input wire [11:0] write_peak,
 output reg [39:0] band,noise,signal_power,
 output reg [11:0] peak
);
 (* ram_style="block" *) reg [51:0] band_memory[0:7];
 (* ram_style="block" *) reg [39:0] noise_memory[0:7],signal_memory[0:7];
 always @(posedge clk)begin
  if(write_valid)begin
   band_memory[write_index]<={write_peak,write_band};
   noise_memory[write_index]<=write_noise;signal_memory[write_index]<=write_signal;
  end
  {peak,band}<=band_memory[read_index];
  noise<=noise_memory[read_index];signal_power<=signal_memory[read_index];
 end
endmodule
`default_nettype wire
