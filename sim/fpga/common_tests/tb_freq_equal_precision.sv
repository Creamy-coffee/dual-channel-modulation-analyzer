`timescale 1ns/1ps
// Standalone legacy counter: valid-sample gating, hysteresis, repeated gates,
// signal absence and reset. The canonical daq_top does not instantiate it.
module tb_freq_equal_precision;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,av=0;reg signed [11:0] x=0;
 wire [31:0] hz,id;wire valid;integer n,seen=0,stage=0;
 freq_equal_precision #(.CLK_FREQ(1024),.GATE_SAMPLES(256),.USE_DC_TRACKER(0)) dut(
  .clk(clk),.rst_n(rst_n),.adc_valid(av),.adc_data(x),.trig_level(12'sd0),
  .freq_hz(hz),.result_id(id),.result_valid(valid));
 always @(negedge clk)begin #1;if(rst_n && valid)begin
  seen=seen+1;if(hz!==(stage==0?32'd32:32'd0))$fatal(1,"frequency gate result stage=%0d hz=%0d",stage,hz);
 end end
 initial begin
  repeat(3)@(negedge clk);rst_n=1;
  for(n=0;n<1024;n=n+1)begin
   @(negedge clk);av=1;x=(n%32<16)?-100:100;
   @(negedge clk);av=0;repeat(n%4)@(negedge clk);
  end
  repeat(3)@(negedge clk);if(seen!=4 || id!=4)$fatal(1,"gate cadence");
  rst_n=0;repeat(3)@(negedge clk);stage=1;seen=0;rst_n=1;
  for(n=0;n<512;n=n+1)begin
   @(negedge clk);av=1;x=n[0]?-2:2;
   @(negedge clk);av=0;
  end
  repeat(3)@(negedge clk);if(seen!=2 || id!=2)$fatal(1,"noise gate cadence");
  $display("FREQ_EQUAL_PRECISION_PASS valid_strobes=1536 gates=6 hysteresis=OK");$finish;
 end
 initial begin #200000;$fatal(1,"timeout");end
endmodule
