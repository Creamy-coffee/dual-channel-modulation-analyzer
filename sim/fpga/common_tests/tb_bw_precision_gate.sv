`timescale 1ns/1ps
module tb_bw_precision_gate;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,valid=0,fine=0,coarse=1;
 wire [1:0] gear;wire settling;
 meas_bw_ctrl #(.INTEGRATION_LOG2(3),.UPGRADE_WINS(2),.SETTLE_WINDOWS(1),.REQUIRE_PRECISION(1)) dut(
  .clk(clk),.rst_n(rst_n),.enable(1'b1),.meas_valid(valid),.amp0(24'd10000),.amp1(24'd10000),
  .phase_diff(16'sd0),.afc_enable(1'b1),.afc_ready(coarse),.precision_ready(fine),
  .gear(gear),.decim_sel(),.integration_log2(),.commit(),.settling(settling),.valid(),
  .change_count(),.good_windows(),.bad_windows(),.afc_lost_windows(),.last_phase_step(),.settle_ticks());
 task win;begin @(negedge clk);valid=1;@(negedge clk);valid=0;@(negedge clk);end endtask
 integer i;
 initial begin
  repeat(3) @(negedge clk);rst_n=1;repeat(3) @(negedge clk);
  win();win();wait(!settling);@(negedge clk);
  if(gear!=1) $fatal(1,"Coarse lock did not reach balanced gear");
  for(i=0;i<20;i=i+1) win();
  if(gear!=1) $fatal(1,"Entered precise gear without fine frequency budget");
  fine=1;win();wait(!settling);@(negedge clk);
  if(gear!=2) $fatal(1,"Fine certificate did not permit precise gear");
  fine=0;for(i=0;i<9;i=i+1) win();
  if(gear!=0) $fatal(1,"Lost fine precision did not widen bandwidth");
  wait(!settling);@(negedge clk);fine=1;
  win();win();wait(!settling);@(negedge clk);
  win();win();wait(!settling);@(negedge clk);
  coarse=0;for(i=0;i<9;i=i+1) win();
  if(gear!=0) $fatal(1,"Coarse loss did not fall back");
  $display("TB_PASS bandwidth_precision_gate (not carrier dependent)");$finish;
 end
endmodule
