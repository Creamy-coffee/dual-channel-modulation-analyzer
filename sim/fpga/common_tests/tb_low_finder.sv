`timescale 1ns/1ps
module tb_low_finder;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,valid=1,drive=1;
 reg signed [15:0] lut[0:4095];
 reg [31:0] phase=0;
 reg signed [11:0] x0=-4,x1=8;
 wire [31:0] ref_fcw,f0,f1;wire commit,locked;
 wire [1:0] ffv,update;wire [3:0] rate0,rate1,m;
 integer k,n=0;real hz,error;
 fft_find_ctrl #(.LOW_FFT_ENABLE(1),.F_MIN_FCW(32'd528),.AMP_THRESHOLD(64)) d(
  .clk(clk),.rst_n(rst),.enable(1'b1),.adc_valid(valid),.adc_ch0(x0),.adc_ch1(x1),
  .afc_en(1'b1),.afc_ready(1'b0),.li_active_fcw(32'd528),.rescan(1'b0),
  .find_fcw(ref_fcw),.commit(commit),.locked(locked),.busy(),.state(),
  .fft_freq0(f0),.fft_freq1(f1),.fft_bin0(),.fft_bin1(),.fft_mag0(),.fft_mag1(),.frame_count(),.lost_count(),
  .fft_valid_channels(ffv),.fft_update(update),.fft_rate0(rate0),.fft_rate1(rate1),.fft_age0(),.fft_age1());
 auto_timebase #(.MAX_M(15),.DWELL(1)) t(.clk(clk),.rst_n(rst),.enable(1'b1),
  .freq_hz(f0),.freq_valid(ffv[0]),.freq_update(update[0]),.m(m),.auto_active(),.saturated(),.changes());
 always @(negedge clk)begin
  phase<=phase+32'd661;
  x0<=drive?(-4+(200*$signed(lut[phase[31:20]]))/32768):-4;
 end
 initial begin
  for(k=0;k<4096;k=k+1)lut[k]=$rtoi(32767*$sin(6.283185307179586*k/4096));
  repeat(5)@(negedge clk);rst=1;
  while(!commit && n<80000000)begin @(negedge clk);n=n+1;end
  if(!commit)$fatal(1,"LF finder never committed");
  hz=$itor(ref_fcw)*65000000.0/4294967296.0;error=hz-661*65000000.0/4294967296.0;if(error<0)error=-error;
  if(error>0.25 || ffv!=1 || f0!=10 || f1!=0 || rate0!=14 || locked || m!=15)
   $fatal(1,"LF conversion/independent validity mismatch fcw_hz=%f ffv=%0d rate=%0d m=%0d",hz,ffv,rate0,m);
  $display("PASS LF finder: CH0=%0dHz ref=%fHz mask=%0d rate=%0d auto_m=%0d with AFC not-ready",f0,hz,ffv,rate0,m);
  valid=0;@(negedge clk);@(negedge clk);
  if(ffv!=0)$fatal(1,"ADC gap left frequency marked valid");
  drive=0;valid=1;
  repeat(10000)begin @(negedge clk);if(ffv!=0)$fatal(1,"aborted LF frame reused after gap");end
  $display("TB_PASS LOW_FINDER physical 10Hz, fractional FCW, single lane, timebase and gap invalidation");$finish;
 end
endmodule
