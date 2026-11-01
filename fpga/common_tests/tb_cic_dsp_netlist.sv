`timescale 1ns/1ps
module tb_cic_dsp_netlist;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,clear=0,in_valid=0;
 reg signed [19:0] in0=0,in1=0;
 wire valid_ref,valid_net;
 wire signed [19:0] r0,r1,n0,n1;
 cic3_decimate_2ch reference(.clk(clk),.rst_n(rst_n),.clear(clear),.in_valid(in_valid),.in0(in0),.in1(in1),.out_valid(valid_ref),.out0(r0),.out1(r1));
 cic3_dsp_trial mapped(.clk(clk),.rst_n(rst_n),.clear(clear),.in_valid(in_valid),.in0(in0),.in1(in1),.out_valid(valid_net),.out0(n0),.out1(n1));
 reg [31:0] noise=32'h283517ab;
 integer k,outputs=0;
 initial begin
  repeat(30)@(negedge clk);
  for(k=0;k<30000;k=k+1)begin
   @(negedge clk);#1;
   if({valid_ref,r0,r1} !== {valid_net,n0,n1})$fatal(1,"DSP mapping mismatch at cycle %0d",k);
   if(valid_ref)outputs=outputs+1;
   noise={noise[30:0],noise[31]^noise[21]^noise[1]^noise[0]};
   rst_n=(k%5000)>2;clear=(k%1301)==0;in_valid=noise[0] || noise[1];
   // Include full-width signed values, small signals and many modular wraps.
   in0=noise[2]?noise[19:0]:$signed(noise[15:4])>>>3;
   in1=noise[3]?~noise[19:0]:-$signed(in0);
  end
  if(outputs<500)$fatal(1,"insufficient valid output coverage %0d",outputs);
  $display("CIC_DSP_NETLIST_PASS cycles=30000 valid_outputs=%0d",outputs);$finish;
 end
endmodule
