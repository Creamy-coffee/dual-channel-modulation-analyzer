`timescale 1ns/1ps
module tb_harmonic_stream_stats;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,enable=1,channel=0,valid=0,saturated=0;
 reg [31:0] cfg=1,ref_fcw=32'd268435456,age=17;
 reg [1:0] scan_ch=0,window_id=0;
 reg [7:0] bin=0;reg [32:0] power=0;
 wire rv,have,tvalid;wire [31:0] rid,rcfg,rref,flags,rage,thd,drops;
 wire rch;wire [3:0] rate,pref;wire [1:0] rwindow;wire [4:0] mask,nmask;
 wire [199:0] bands,noises,signals;wire [39:0] peaks;
 harmonic_stream_stats #(.LOG2_N(8)) dut(
  .clk(clk),.rst_n(rst_n),.enable(enable),.result_hold(1'b0),.cfg_id(cfg),.reference_fcw(ref_fcw),.channel(channel),.supported_rates(5'b00001),
  .scan_valid(valid),.scan_channel(scan_ch),.scan_bin(bin),.scan_power(power),.scan_rate_log2(4'd0),
  .scan_window(window_id),.scan_saturated(saturated),.scan_age_ticks(age),
  .result_valid(rv),.result_have(have),.thd_valid(tvalid),.result_id(rid),.result_cfg(rcfg),.result_reference_fcw(rref),
  .result_channel(rch),.result_rate_log2(rate),.result_window(rwindow),.result_mask(mask),.result_noise_mask(nmask),
  .result_flags(flags),.result_age_ticks(rage),.thd_q24(thd),.dropped_frames(drops),
  .result_band_power(bands),.result_noise_power(noises),.result_signal_power(signals),.result_peak_bins(peaks),.preferred_rate(pref));
 reg [31:0] low_ref=66076;wire [3:0] low_pref;
 harmonic_stream_stats #(.LOG2_N(12)) selection(
  .clk(clk),.rst_n(rst_n),.enable(enable),.result_hold(1'b0),.cfg_id(32'd1),.reference_fcw(low_ref),.channel(1'b0),.supported_rates(5'b11111),
  .scan_valid(1'b0),.scan_channel(2'd0),.scan_bin(12'd0),.scan_power(33'd0),.scan_rate_log2(4'd0),
  .scan_window(2'd0),.scan_saturated(1'b0),.scan_age_ticks(32'd0),.preferred_rate(low_pref));
 task configure(input [31:0] f,input [1:0] w);
  begin @(negedge clk);cfg=cfg+1;ref_fcw=f;window_id=w;repeat(12)@(posedge clk);end
 endtask
 task send_frame(input integer base,input [32:0] p1,input [32:0] p2,input [32:0] p3,input integer omit);
  integer k;reg [32:0] value;
  begin
   for(k=1;k<=128;k=k+1)begin
    @(negedge clk);valid=(k!=omit);bin=k;
    value=100;
    if(k==base)value=value+p1;
    if(k==base*2)value=value+p2;
    if(k==base*3)value=value+p3;
    power=value;
   end
   @(negedge clk);valid=0;
  end
 endtask
 task await_result;
  integer k;reg found;
  begin found=0;
   for(k=0;k<500;k=k+1)begin @(posedge clk);#1;if(rv)begin found=1;k=500;end end
   if(!found)$fatal(1,"harmonic result timeout");
  end
 endtask
 integer prior_id,k;
 initial begin
  repeat(3)@(posedge clk);@(negedge clk);rst_n=1;repeat(15)@(posedge clk);#1;
  if(low_pref!=8)$fatal(1,"1kHz must choose exact /256 band");
  send_frame(16,1000000,10000,2500,0);await_result();
  if(!have || !tvalid || mask!=31 || thd!=1875749 || rcfg!=cfg || rref!=ref_fcw)$fatal(1,"THD known ratio mismatch %d flags=%h",thd,flags);
  if(signals[39:0]!=6000000 || signals[79:40]!=60000 || signals[119:80]!=15000)$fatal(1,"noise subtraction/precision mismatch");
  if(bands[39:0]!=1000700 || noises[39:0]!=600 || peaks[7:0]!=16 || peaks[15:8]!=32)$fatal(1,"power/peak mismatch");
  if((nmask & 5'b11000)!=5'b11000 || !(flags & 16))$fatal(1,"weak harmonics not flagged");
  // Earliest usable noise bin is 7: serial RAM clearing must have finished.
  configure(32'd218103808,0);send_frame(13,1000000,10000,2500,0);await_result();
  if(!tvalid || thd!=1875749 || signals[39:0]!=6000000)$fatal(1,"minimum window spacing / RAM initialization");
  configure(32'd268435456,0);
  // Explicit H1 must not silently become the largest (second-harmonic) peak.
  send_frame(16,25000,100000,0,0);await_result();
  if(!tvalid || thd!=33554432 || signals[39:0]!=150000)$fatal(1,"manual fundamental / THD>100percent");
  send_frame(16,33'd8589934491,33'd8589934491,0,0);await_result();
  if(!tvalid || thd!=16777216 || signals[39:0]!=40'd51539606946)$fatal(1,"33-bit input / 40-bit power precision lost");
  send_frame(16,0,0,0,0);await_result();
  if(tvalid || !(flags & 1))$fatal(1,"noise-only base accepted");
  configure(32'd436207616,0);send_frame(26,1000000,10000,2500,0);await_result();
  if(tvalid || mask!=15 || !(flags & 4))$fatal(1,"out-of-band H5 accepted");
  configure(32'd33554432,0);send_frame(2,1000000,10000,2500,0);await_result();
  if(tvalid || mask!=0 || !(flags & 2))$fatal(1,"unresolved base accepted");
  configure(32'd268435456,1);send_frame(16,1000000,10000,2500,0);await_result();
  if(tvalid || !(flags & 128) || rwindow!=1)$fatal(1,"unsupported window mislabeled");
  configure(32'd268435456,0);saturated=1;send_frame(16,1000000,10000,2500,0);await_result();
  if(tvalid || !(flags & 8))$fatal(1,"FFT overflow accepted");saturated=0;
  prior_id=rid;send_frame(16,1000000,10000,2500,10);repeat(200)begin @(posedge clk);#1;if(rv)$fatal(1,"broken scan published");end
  if(rid!=prior_id || drops==0)$fatal(1,"broken scan not accounted");
  send_frame(16,1000000,10000,2500,0);await_result();if(!tvalid || thd!=1875749)$fatal(1,"scan recovery failed");
  prior_id=rid;send_frame(16,1000000,10000,2500,0);
  // Reconfiguration while divide/sqrt is active must discard the old result.
  configure(32'd268435456,0);
  repeat(160)begin @(posedge clk);#1;if(rv)$fatal(1,"old arithmetic crossed configuration");end
  if(rid!=prior_id || have)$fatal(1,"configuration failed to invalidate old result");
  send_frame(16,1000000,10000,2500,0);await_result();
  age=32'hfffffffc;send_frame(16,1000000,10000,2500,0);await_result();
  if(rage!=32'hffffffff)$fatal(1,"age wrapped into fresh result");age=17;
  // Channel filtering and configuration changes must not leak old results.
  scan_ch=1;prior_id=rid;send_frame(16,1000000,10000,2500,0);repeat(160)@(posedge clk);
  if(rid!=prior_id)$fatal(1,"wrong channel published");
  @(negedge clk);channel=1;cfg=cfg+1;repeat(12)@(posedge clk);
  send_frame(16,1000000,10000,2500,0);await_result();if(!tvalid || !rch || rcfg!=cfg)$fatal(1,"channel change failed");
  @(negedge clk);low_ref=13215;repeat(12)@(posedge clk);#1;if(low_pref!=12)$fatal(1,"200Hz partial-band selection");
  @(negedge clk);low_ref=1982292598;repeat(12)@(posedge clk);#1;if(low_pref!=0)$fatal(1,"30MHz selection");
  @(negedge clk);enable=0;repeat(3)@(posedge clk);#1;if(have || tvalid)$fatal(1,"disabled result retained");
  $display("HARMONIC_STATS_PASS linear_power_noise_reference_THD_range_stream_channel_age");$finish;
 end
 initial begin #1000000;$fatal(1,"global timeout");end
endmodule
