`timescale 1ns/1ps
module tb_observer_integrity;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,clear=0,ok=1,v=0;
 reg signed [27:0] x=0,y=0;
 reg [31:0] t=0;
 always @(posedge clk) t<=t+1;
 local_afc_2ch dut(.clk(clk),.rst_n(rst),.clear(clear),.stream_ok(ok),.mix_valid(v),
  .mi0(x),.mq0(y),.mi1(-x),.mq1(-y),.sample_time(t),
  .nominal_fcw(32'd1982292598),.current_fcw(32'd1982292598),
  .target_fcw(),.commit(),.ready_measurement(),.state(),.residual0_fcw(),.residual1_fcw(),
  .reason(),.updates(),.observation_id(),.last_timestamp());
 integer k;
 task reset;
  begin @(negedge clk);rst=0;v=0;repeat(3) @(negedge clk);rst=1;end
 endtask
 initial begin
  reset();
  for(k=0;k<4096;k=k+1) begin @(negedge clk);v=1;x=(k==4095)?32768:16384;y=-16384;end
  @(negedge clk);v=0;
  if(!dut.obs || dut.oi0!=4097 || dut.oq0!=-4096 || dut.oi1!=-4097 || dut.oq1!=4096)
   $fatal(1,"Last sample/sign/normalization failed: %0d/%0d",dut.oi0,dut.oq0);
  reset();
  for(k=0;k<4096;k=k+1) begin @(negedge clk);v=1;x=28'sh7ffffff;y=-28'sh7ffffff;end
  @(negedge clk);v=0;
  if(dut.oi0!=24'sh7fffff || dut.oq0!=24'sh800000) $fatal(1,"Saturation failed");
  reset();
  repeat(100) begin @(negedge clk);v=1;x=100000;y=100000;end
  @(negedge clk);ok=0;@(negedge clk);ok=1;v=0;
  for(k=0;k<4096;k=k+1) begin @(negedge clk);v=1;x=0;y=0;end
  @(negedge clk);v=0;
  if(!dut.obs || dut.oi0!=0 || dut.oq0!=0) $fatal(1,"Gap retained partial data");
  $display("PASS observer last sample, signed scaling, saturation, gap flush");$finish;
 end
endmodule
