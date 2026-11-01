`timescale 1ns/1ps
// Full ADC->mixer->filters->Hann->phase->guard->coarse/fine feedback.
// No internal phase, ready or qualification signals are forced.
module tb_low_afc_reliability;
 parameter integer CASE=0;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,boot=0,av=1,tones=1;
 reg signed [11:0] x0=0,x1=0;
 reg [31:0] nominal=65813;
 wire lv,present,rv,ready,fine;wire [23:0] a0,a1;wire signed [15:0] p0,p1,pd;
 wire [31:0] ratio,fcw,id,age,gap,drops;wire [1:0] det,qual;
 wire [7:0] flags,reason,gr0,gr1;wire [2:0] state;
 wire signed [31:0] e0,e1;wire [31:0] updates;
 reg production_bw=0;
 wire [1:0] gear,ds;
 wire [4:0] il;
 wire bw_commit,bw_settling;
 wire [15:0] bw_changes;
 wire [1:0] effective_ds=production_bw?ds:2'd1;
 wire [4:0] effective_il=production_bw?il:5'd12;
 meas_bw_ctrl #(.REQUIRE_PRECISION(1),.MAX_GEAR(2)) bw(
  .clk(clk),.rst_n(rst),.enable(production_bw),.meas_valid(lv),
  .amp0(a0),.amp1(a1),.phase_diff(pd),.afc_enable(1'b1),
  .afc_ready(state==3 && (reason & 8'hef)==0),.precision_ready(fine),
  .gear(gear),.decim_sel(ds),.integration_log2(il),.commit(bw_commit),
  .settling(bw_settling),.valid(),.change_count(bw_changes),.good_windows(),
  .bad_windows(),.afc_lost_windows(),.last_phase_step(),.settle_ticks());
 lockin_measurement #(.AFC_ENABLE(1)) dut(
  .clk(clk),.rst_n(rst),.adc_valid(av),.adc_ch0(x0),.adc_ch1(x1),
  .cfg_commit(boot),.cfg_reconfig(production_bw && bw_commit),.cfg_enable(1'b1),.cfg_fcw(nominal),
  .cfg_decim_sel(effective_ds),.cfg_integration_log2(effective_il),
  .disp_fcw(fcw),.disp_decim(effective_ds),.disp_integration(effective_il),
  .lockin_valid(lv),.amp0_q12(a0),.amp1_q12(a1),.phase0_turn(p0),.phase1_turn(p1),
  .phase_diff_turn(pd),.ratio_q16(ratio),.flags(flags),.gap_count(gap),.drop_count(drops),
  .result_present(present),.ratio_valid(rv),.result_id(id),.result_age(age),
  .detected(det),.qualified(qual),.active_fcw(fcw),.tracking_state(state),
  .tracking_ready(ready),.precision_ready(fine),.tracking_reason(reason),
  .residual0_fcw(e0),.residual1_fcw(e1),.tracking_updates(updates),
  .guard_reason0(gr0),.guard_reason1(gr1),.guard_step0(),.guard_step1(),
  .guard_threshold0(),.guard_threshold1(),.precision_estimate_valid(),
  .precision_error0(),.precision_error1(),.nco_phase());
 real f=1000,f1=1000,peak=200,theta=0,theta1=0;
 integer scenario=CASE,n=0,consecutive=0,good=0;reg [31:0] rng=32'h9821ab31;
 function integer nearest(input real v);nearest=v>=0?$rtoi(v+0.5):$rtoi(v-0.5);endfunction
 always @(negedge clk)if(rst)begin
  n=n+1;theta=theta+6.283185307179586*f/65000000.0;theta1=theta1+6.283185307179586*f1/65000000.0;
  if(theta>6.283185307179586)theta=theta-6.283185307179586;
  if(theta1>6.283185307179586)theta1=theta1-6.283185307179586;
  rng=rng*1664525+1013904223;
  x0=(tones?nearest(peak*$sin(theta)):0)+(rng%3)-1;
  x1=(tones?nearest(0.5*peak*$sin(theta1+1.570796326794897)):0)+((rng>>16)%3)-1;
  if(n%10000000==0)begin
   $display("LOW_PROGRESS case=%0d n=%0d f=%f ready=%0d fine=%0d qual=%0d gr=%0d/%0d state=%0d reason=%0d",scenario,n,fcw*65000000.0/4294967296.0,ready,fine,qual,gr0,gr1,state,reason);$fflush;
  end
 end
 real measured,phase_deg,amp,rat;
 always @(posedge clk)if(rst && lv)begin
  if(qual==3 && ready && fine && rv && flags==0 && (!production_bw || (gear==2 && !bw_settling)))begin good=good+1;consecutive=consecutive+1;end
  else consecutive=0;
 end
 task check_numeric;
  begin
   measured=fcw*65000000.0/4294967296.0;phase_deg=$itor($signed(pd))*360.0/65536.0;amp=a0/4096.0;rat=ratio/65536.0;
   if(measured<f-0.15 || measured>f+0.15)$fatal(1,"frequency accuracy f=%f expected=%f",measured,f);
   if(phase_deg<89 || phase_deg>91)$fatal(1,"phase error %f",phase_deg);
   if(amp<peak*0.94 || amp>peak*1.06)$fatal(1,"amplitude error %f",amp);
   if(rat<0.49 || rat>0.51)$fatal(1,"ratio error %f",rat);
   $display("LOW_LOCK case=%0d n=%0d hz=%f phase=%f amp=%f ratio=%f",scenario,n,measured,phase_deg,amp,rat);$fflush;
  end
 endtask
 initial begin
  if(!$value$plusargs("CASE=%d",scenario))scenario=CASE;
  if(scenario==1)begin f=500;f1=f;nominal=(64'd4294967296*496)/65000000;end
  if(scenario==2)begin f=2000;f1=f;nominal=(64'd4294967296*1992)/65000000;end
  if(scenario==3)begin f=30001200;f1=f;peak=20;nominal=(64'd4294967296*30000000)/65000000;end
  if(scenario==4)tones=0;
  if(scenario==5)f1=1100;
  if(scenario>=8)production_bw=1;
  if(scenario==9)begin f=500;f1=f;nominal=(64'd4294967296*496)/65000000;end
  if(scenario==10)begin f=10001000;f1=f;nominal=(64'd4294967296*10000000)/65000000;end
  if(scenario==11)begin f=30001200;f1=f;peak=20;nominal=(64'd4294967296*30000000)/65000000;end
  repeat(6)@(negedge clk);rst=1;boot=1;@(negedge clk);boot=0;
  if(scenario==4 || scenario==5)begin
   repeat(20000000)@(negedge clk);
   if(good!=0 || (qual==3 && ready && fine))$fatal(1,"false precision lock");
  end else begin
   wait(consecutive>=8);check_numeric();
   if(scenario==6 || scenario==12)begin
    f=1020;f1=f;consecutive=0;repeat(2000000)@(negedge clk);
    wait(consecutive>=8);check_numeric();
   end
   // At R1024 the Hann result itself spans 64.5 ms; a 30.8 ms wait
   // cannot require that the next complete result already reflect dropout.
   tones=0;repeat(production_bw?20000000:2000000)@(negedge clk);
   // A numeric nonzero denominator can still yield a diagnostic ratio from
   // noise. Qualification, not mere ratio presence, certifies a measurement.
   if(qual!=0)$fatal(1,"stale qualified result after signal loss");
   av=0;repeat(8)@(negedge clk);
   if(qual!=0 || present)$fatal(1,"ADC gap retained result");
   if(scenario==7)begin
    tones=1;av=1;consecutive=0;wait(consecutive>=8);check_numeric();
   end
  end
  $display("LOW_AFC_RELIABILITY_PASS case=%0d good=%0d cycles=%0d production_bw=%0d gear_changes=%0d",scenario,good,n,production_bw,bw_changes);$finish;
 end
 initial begin repeat(520000000)@(negedge clk);$fatal(1,"low AFC timeout case=%0d f=%f qual=%0d state=%0d reason=%0d",scenario,fcw*65000000.0/4294967296.0,qual,state,reason);end
endmodule
