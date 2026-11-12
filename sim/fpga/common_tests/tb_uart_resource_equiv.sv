`timescale 1ns/1ps
module uart_equiv_lane #(parameter BAUD=2000000)(input clk,rst_n,input [7:0] data,input valid);
 wire tx_new,tx_old,ready_new,ready_old;
 uart_tx_v32 #(.BAUD(BAUD)) candidate(.clk(clk),.rst_n(rst_n),.tx_data(data),.tx_valid(valid),.tx_ready(ready_new),.tx(tx_new));
 uart_tx_v32_reference #(.BAUD(BAUD)) reference(.clk(clk),.rst_n(rst_n),.tx_data(data),.tx_valid(valid),.tx_ready(ready_old),.tx(tx_old));
 always @(negedge clk) if(rst_n && ({tx_new,ready_new} !== {tx_old,ready_old}))
  $fatal(1,"Divider differs from baseline at baud %0d",BAUD);
endmodule
module tb_uart_resource_equiv;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,valid=0;
 reg [7:0] data=0;
 reg [31:0] noise=32'h13657bad;
 uart_equiv_lane #(.BAUD(2000000)) fractional(.clk(clk),.rst_n(rst_n),.data(data),.valid(valid));
 uart_equiv_lane #(.BAUD(921600)) legacy(.clk(clk),.rst_n(rst_n),.data(data),.valid(valid));
 uart_equiv_lane #(.BAUD(1000000)) integer_rate(.clk(clk),.rst_n(rst_n),.data(data),.valid(valid));
 integer cycle;
 initial begin
  for(cycle=0;cycle<300000;cycle=cycle+1)begin
   @(negedge clk);#1;
   noise={noise[30:0],noise[31]^noise[21]^noise[1]^noise[0]};
   rst_n=(cycle%60000)>4;
   // Long back-to-back bursts and idle stretches, plus resets in mid-byte.
   valid=(cycle%5000)<4500 && noise[0];data=noise[15:8];
  end
  $display("UART_RESOURCE_EQUIV_PASS cycles=300000 bauds=2000000,921600,1000000");$finish;
 end
endmodule
