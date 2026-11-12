`timescale 1ns/1ps
// Full nominal 65 MSPS mixer/CIC/phase/guard/coarse+fine AFC and BW controller.
// Generator phase is independent of DUT NCO. No forced ready/qual/phase values.
module tb_precision_recovery;
 parameter integer CASE=0;
 integer scenario=CASE;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,boot=0,av=1,tones=1;
 reg signed [11:0] x0=0,x1=0;
 wire lv,rv,present,ready,fine;
 wire [23:0] a0,a1;
 wire [1:0] det,qual,gear,ds;
 wire [4:0] il;
 wire signed [15:0] p0,p1,pd;
 wire [31:0] ratio,fcw,mid,age,gaps,drops;
 wire [7:0] flags,reason,gr0,gr1;
 wire [15:0] gs0,gs1;
 wire [28:0] gt0,gt1;
 wire [2:0] state;
 wire signed [31:0] df0,df1;
 wire bw_commit,settling,bwvalid;
 wire [15:0] changes;
 wire coarse=(state==3 && (reason & 8'hef)==0);
 meas_bw_ctrl #(.REQUIRE_PRECISION(1)) bw(
  .clk(clk),.rst_n(rst_n),.enable(1'b1),.meas_valid(lv),.amp0(a0),.amp1(a1),.phase_diff(pd),
  .afc_enable(1'b1),.afc_ready(coarse),.precision_ready(fine),.gear(gear),.decim_sel(ds),
  .integration_log2(il),.commit(bw_commit),.settling(settling),.valid(bwvalid),
  .change_count(changes),.good_windows(),.bad_windows(),.afc_lost_windows(),.last_phase_step(),.settle_ticks());
 lockin_measurement #(.AFC_ENABLE(1)) dut(
  .clk(clk),.rst_n(rst_n),.adc_valid(av),.adc_ch0(x0),.adc_ch1(x1),
  .cfg_commit(boot),.cfg_reconfig(bw_commit),.cfg_enable(1'b1),.cfg_fcw(32'd1982292598),
  .cfg_decim_sel(ds),.cfg_integration_log2(il),.disp_fcw(fcw),.disp_decim(ds),.disp_integration(il),
  .lockin_valid(lv),.amp0_q12(a0),.amp1_q12(a1),.phase0_turn(p0),.phase1_turn(p1),
  .phase_diff_turn(pd),.ratio_q16(ratio),.flags(flags),.gap_count(gaps),.drop_count(drops),
  .result_present(present),.ratio_valid(rv),.result_id(mid),.result_age(age),.detected(det),.qualified(qual),
  .tracking_state(state),.tracking_ready(ready),.precision_ready(fine),.residual0_fcw(df0),.residual1_fcw(df1),
  .tracking_reason(reason),.guard_reason0(gr0),.guard_reason1(gr1),.guard_step0(gs0),.guard_step1(gs1),
  .guard_threshold0(gt0),.guard_threshold1(gt1),.tracking_updates(),.active_fcw(fcw),.nco_phase());
 real freq=30001200.0,freq1=30001200.0,peak=20.0,theta=0.0,theta1=0.0,v0,v1;
 reg [31:0] rng=32'h15490683;
 integer n=0,good=0,consecutive=0,seen_precise=0;
 real amp_sum=0,phase_sum=0,ratio_sum=0;
 function integer nearest(input real v);nearest=v>=0?$rtoi(v+0.5):$rtoi(v-0.5);endfunction
 always @(negedge clk) if(rst_n) begin
  n=n+1;theta=theta+6.283185307179586*freq/65000000.0;
  if(theta>6.283185307179586) theta=theta-6.283185307179586;
  theta1=theta1+6.283185307179586*freq1/65000000.0;
  if(theta1>6.283185307179586) theta1=theta1-6.283185307179586;
  rng=rng*1664525+1013904223;
  v0=tones?peak*$sin(theta):0;
  v1=tones?0.8*peak*$sin(theta1+1.0471975511966):0;
  x0=nearest(v0)+(rng%3)-1-4;
  x1=nearest(v1)+((rng>>16)%3)-1+8;
  if(n%4000000==0) begin
   $display("PROGRESS case=%0d n=%0d gear=%0d afc=%0d fine=%0b qual=%0d gr=%0d/%0d f=%0d df=%0d/%0d fine_e=%0d/%0d reason=%0d",scenario,n,gear,state,fine,qual,gr0,gr1,fcw,df0,df1,dut.tracking.fine.error0_fcw,dut.tracking.fine.error1_fcw,dut.tracking.fine.reason);$fflush;
  end
 end
 always @(posedge clk) if(rst_n && lv && gear==2 && !settling && tones) begin
  seen_precise=seen_precise+1;
  if(qual==3 && ready && rv && flags==0) begin
   good=good+1;consecutive=consecutive+1;
   amp_sum=amp_sum+a0/4096.0;phase_sum=phase_sum+$itor($signed(pd))*360.0/65536.0;
   ratio_sum=ratio_sum+ratio/65536.0;
  end else consecutive=0;
 end
 initial begin
  if(!$value$plusargs("CASE=%d",scenario)) scenario=CASE;
  if(scenario==1) begin peak=10;freq=29998800.0;end
  if(scenario==2) begin peak=4;freq=30000000.9;end
  freq1=freq;
  if(scenario==3) freq1=freq+300;
  if(scenario==4) tones=0;
  repeat(6) @(negedge clk);rst_n=1;boot=1;
  @(negedge clk);boot=0;
  if(scenario>=3) begin
   repeat(1600000) @(negedge clk);
   if(qual!=0 || ready || fine) $fatal(1,"False lock on absent/different source");
   $display("TB_PASS precision_chain negative case=%0d",scenario);$finish;
  end
  wait(consecutive>=8);
  if(gear!=2 || !fine || !ready) $fatal(1,"Precise lock not held");
  if(amp_sum/good<peak*0.94 || amp_sum/good>peak*1.06) $fatal(1,"Amplitude mismatch %f",amp_sum/good);
  if(phase_sum/good<58 || phase_sum/good>62) $fatal(1,"Phase mismatch %f",phase_sum/good);
  if(ratio_sum/good<0.78 || ratio_sum/good>0.82) $fatal(1,"Ratio mismatch %f",ratio_sum/good);
  $display("PRECISE_PASS case=%0d good=%0d windows=%0d amp=%f phase=%f ratio=%f n=%0d",scenario,good,seen_precise,amp_sum/good,phase_sum/good,ratio_sum/good,n);$fflush;
  tones=0;repeat(100000) @(negedge clk);
  if(ready || qual!=0) $fatal(1,"Signal loss retained measurement permission");
  av=0;repeat(5) @(negedge clk);
  if(qual!=0 || present) $fatal(1,"ADC gap retained result");
  av=1;tones=1;consecutive=0;
  wait(consecutive>=4);
  if(!ready || !fine || gear!=2) $fatal(1,"Did not recapture precise measurement after loss");
  $display("RECOVERY_PASS restored precise qualification n=%0d",n);$fflush;
  freq=freq+50.0;freq1=freq;consecutive=0;
  wait(consecutive>=4);
  if((freq-$itor(fcw)*65000000.0/4294967296.0)>0.215 ||
     (freq-$itor(fcw)*65000000.0/4294967296.0)< -0.215)
   $fatal(1,"50 Hz step failed frequency budget");
  if($itor($signed(pd))*360.0/65536.0<58 || $itor($signed(pd))*360.0/65536.0>62)
   $fatal(1,"Phase relation lost after frequency step");
  $display("TB_PASS precision_recovery restore+50Hz step n=%0d",n);$finish;
 end
 initial begin repeat(900000000) @(negedge clk);$fatal(1,"Precision chain timeout case=%0d fine=%0b gear=%0d qual=%0d",scenario,fine,gear,qual);end
endmodule
