`timescale 1ns/1ps
module tb_reconfig_collision;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,commit=0,reconfig=0,tune=0,av=1;
 wire [31:0] active,phase;wire mix_valid;
 lockin_core_2ch core(.clk(clk),.rst_n(rst_n),.adc_valid(av),
  .adc_ch0(12'sd0),.adc_ch1(12'sd0),.cfg_commit(commit),.cfg_reconfig(reconfig),
  .cfg_enable(1'b1),.cfg_freq_word(32'd1982292598),.cfg_decim_sel(2'd3),.cfg_integration_log2(5'd12),
  .afc_fcw(32'd1982292698),.afc_commit(tune),.active_fcw(active),.nco_phase(phase),.mix_valid(mix_valid));
 reg [31:0] oldphase;
 integer i;
 initial begin
  repeat(3) @(negedge clk);rst_n=1;commit=1;
  @(negedge clk);commit=0;
  repeat(5) @(negedge clk);
  oldphase=phase;reconfig=1;tune=1;
  @(negedge clk);
  if(active!==32'd1982292698) $fatal(1,"Concurrent reconfigure swallowed tune");
  if((phase-oldphase)!==32'd1982292598) $fatal(1,"Reconfigure disturbed NCO phase");
  if(!mix_valid) $fatal(1,"Baseband reconfigure interrupted AFC mixed stream");
  reconfig=0;tune=0;
  for(i=0;i<4;i=i+1) begin
   oldphase=phase;@(negedge clk);
   if((phase-oldphase)!==32'd1982292698 || !mix_valid) $fatal(1,"Post-reconfigure phase/stream discontinuity");
  end
  commit=1;reconfig=1;tune=1;@(negedge clk);
  if(active!==32'd1982292598 || phase!==0) $fatal(1,"Full commit lost highest priority");
  commit=0;reconfig=0;tune=0;repeat(5) @(negedge clk);
  av=0;repeat(3) @(negedge clk);
  if(core.gap_count!=3 || !core.flags[0]) $fatal(1,"Gap diagnostic missing");
  av=1;@(negedge clk);
  if(core.gap_count!=3 || core.flags[0]) $fatal(1,"Live gap fault did not clear or history erased");
  $display("TB_PASS reconfig_collision concurrent tune, uninterrupted mixer, phase progression and full-commit priority");$finish;
 end
endmodule
