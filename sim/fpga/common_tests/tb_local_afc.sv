`timescale 1ns/1ps
module tb_local_afc;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,av=1,commit=0;
 reg signed [11:0] x0=0,x1=0;
 wire lv,present,rv,ready;wire [23:0] a0,a1;
 wire signed [15:0] p0,p1,pd;wire [31:0] ratio,fcw,id,age,gap,drops;
 wire [1:0] det,qual;wire [7:0] flags,reason;wire [2:0] state;
 wire signed [31:0] e0,e1;wire [31:0] updates;
 lockin_measurement #(.AFC_ENABLE(1)) dut(
  .clk(clk),.rst_n(rst),.adc_valid(av),.adc_ch0(x0),.adc_ch1(x1),
  .cfg_commit(commit),.cfg_reconfig(1'b0),.cfg_enable(1'b1),.cfg_fcw(32'd1982292598),
  .cfg_decim_sel(2'd0),.cfg_integration_log2(5'd12),
  .disp_fcw(32'd1982292598),.disp_decim(2'd0),.disp_integration(5'd12),
  .lockin_valid(lv),.amp0_q12(a0),.amp1_q12(a1),.phase0_turn(p0),.phase1_turn(p1),
  .phase_diff_turn(pd),.ratio_q16(ratio),.flags(flags),.gap_count(gap),.drop_count(drops),
  .result_present(present),.ratio_valid(rv),.result_id(id),.result_age(age),
  .detected(det),.qualified(qual),.active_fcw(fcw),.tracking_state(state),
  .tracking_ready(ready),.tracking_reason(reason),.residual0_fcw(e0),.residual1_fcw(e1),.tracking_updates(updates));
 integer scenario=0,n=0;reg [31:0] rng=32'h12345678;
 real f0=30001200.0,f1=30001200.0,peak=20.0,v0,v1;
 reg tones=1;
 function integer roundcode(input real v);
  roundcode=v>=0?$rtoi(v+0.5):$rtoi(v-0.5);
 endfunction
 always @(negedge clk) begin
  if(rst) begin
   n=n+1; if(n%400000==0) $display("PROGRESS n=%0d state=%0d fcw=%0d err=%0d reason=%0d",n,state,fcw,e0,reason); rng=rng*1664525+1013904223;
   v0=tones?peak*$sin(6.283185307179586*f0*n/65000000.0):0;
   v1=tones?peak*0.8*$sin(6.283185307179586*f1*n/65000000.0+1.0471975512):0;
   x0=roundcode(v0)+(rng%3)-1-4;
   x1=roundcode(v1)+((rng>>16)%3)-1+8;
  end
 end
 always @(posedge clk) if(rst && lv && (^{a0,a1,pd,fcw,qual} === 1'bx)) $fatal(1,"Unknown numeric result a0=%h a1=%h pd=%h fcw=%h qual=%h",a0,a1,pd,fcw,qual);
 integer qualified_count=0;real sumamp=0,sumpd=0;integer seen=0;
 always @(posedge clk) if(lv && n>((scenario>=3)?200000:2400000)) begin
  seen=seen+1;
  if(qual==3) qualified_count=qualified_count+1;
  sumamp=sumamp+a0/4096.0;sumpd=sumpd+$itor($signed(pd))*360.0/65536.0;
 end
 initial begin
  if(!$value$plusargs("CASE=%d",scenario)) scenario=0;
  if(scenario==1) begin f0=29998800.0;f1=f0;end
  if(scenario==2) peak=4.0;
  if(scenario==3) begin peak=4;tones=0;end
  if(scenario==4) f1=f0+300;
  if(scenario==5) begin f0=30004000;f1=f0;end
  repeat(6) @(negedge clk);rst=1;commit=1;
  @(negedge clk);commit=0;
  repeat((scenario>=3)?1200000:3200000) @(negedge clk);
  $display("CASE %0d seen=%0d good=%0d state=%0d reason=%0d FCW=%0d err=%0d/%0d updates=%0d amp=%f pd=%f",scenario,seen,qualified_count,state,reason,fcw,e0,e1,updates,sumamp/seen,sumpd/seen);
  if(scenario==0 || scenario==1 || scenario==2) begin
   if(qualified_count<8 || !ready) $fatal(1,"Failed capture/qualification");
   if(sumamp/seen<peak*0.93 || sumamp/seen>peak*1.07) $fatal(1,"Amplitude error");
   if(sumpd/seen<58 || sumpd/seen>62) $fatal(1,"Phase difference error");
   tones=0;repeat(400000) @(negedge clk);
   if(qual!=0 || ready) $fatal(1,"Old valid state after signal loss");
   av=0;repeat(4) @(negedge clk);
   if(qual!=0 || present) $fatal(1,"Gap did not invalidate");
  end else if(qualified_count!=0 || ready) $fatal(1,"False qualification");
  $display("PASS CASE %0d",scenario);$finish;
 end
 initial begin #50000000;$fatal(1,"Timeout");end
endmodule
