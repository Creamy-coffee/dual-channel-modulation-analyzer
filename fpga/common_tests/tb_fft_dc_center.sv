`timescale 1ns/1ps
module tb_fft_dc_center;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,enable=1,frame_enable=0,accept=0;
 reg signed [11:0] in0=0,in1=0;
 wire signed [11:0] out0,out1;
 wire frame_valid,frame_clipped;
 fft_dc_center #(.LOG2_N(3)) dut(.*);
 integer checks=0,i,j,expected0=0,expected1=0,s0,s1,known=0,clipped,x0,x1;
 function integer clamp12(input integer x);
   clamp12=x>2047?2047:x< -2048?-2048:x;
 endfunction
 function integer mean8(input integer x);
   mean8=x>=0?(x+4)/8:-((-x+4)/8);
 endfunction
 task frame(input integer a,input integer b,input integer tail);
 begin
   frame_enable=1;s0=0;s1=0;clipped=0;
   for(i=0;i<8;i=i+1) begin
     @(negedge clk);in0=a+(i==7?tail:0);in1=b-(i==7?tail:0);accept=1;
     x0=$signed(in0)-expected0;x1=$signed(in1)-expected1;
     #1;
     if($signed(out0)!=clamp12(x0)||$signed(out1)!=clamp12(x1)) $fatal(1,"centered sample differs from model");
     clipped=clipped||(x0>2047)||(x0< -2048)||(x1>2047)||(x1< -2048);
     s0=s0+$signed(in0);s1=s1+$signed(in1);
     @(posedge clk);#1;
     if(frame_valid!=known||frame_clipped!=clipped) $fatal(1,"frame metadata differs");
     checks=checks+1;
     // Input gaps must not enter the accepted-sample mean/count.
     @(negedge clk);accept=0;in0=2000;in1=-2000;
     @(posedge clk);#1;
   end
   expected0=mean8(s0);expected1=mean8(s1);known=1;
   @(negedge clk);frame_enable=0;
   repeat(3) @(posedge clk);
 end
 endtask
 initial begin
  repeat(3) @(negedge clk);rst_n=1;
  for(j=-12;j<=12;j=j+1) frame(-4,8,j);
  frame(-2048,2047,0);frame(2047,-2048,0);frame(2047,-2048,0);
  @(negedge clk);enable=0;repeat(2) @(negedge clk);
  if(frame_valid||frame_clipped) $fatal(1,"disable retained stale metadata");
  enable=1;known=0;expected0=0;expected1=0;
  frame(7,-11,0);frame(7,-11,0);
  $display("TB_PASS tb_fft_dc_center %0d accepted-sample/metadata checks",checks);$finish;
 end
endmodule
