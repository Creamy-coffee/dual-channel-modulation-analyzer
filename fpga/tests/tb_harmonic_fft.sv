`timescale 1ns/1ps
module tb_harmonic_fft #(parameter ROUND=1);
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0;integer at=0;
 reg signed [11:0] samples[0:4095];
 reg [31:0] reference_fcw;
 wire fft_busy,accepted,hvalid,sv;wire [1:0] sch;wire [11:0] sbin;wire [32:0] spower;
 wire signed [15:0] h0,h1;
 wire [11:0] adc=at<4096?samples[at]:12'd0;
 initial begin
  $readmemh("../tests/stimulus.mem",samples);
  if(!$value$plusargs("REF=%d",reference_fcw))reference_fcw=134217728;
  repeat(3)@(posedge clk);@(negedge clk);rst_n=1;
 end
 always @(posedge clk)if(rst_n && accepted)at<=at+1;
 hann_window_2ch #(.LOG2_N(12),.COEFF_FILE("hann_4096_half_q15.mem"),.HALF_WINDOW_OPT(1),.FRAME_GATED(1)) window(
  .clk(clk),.rst_n(rst_n),.in_valid(at<4096),.frame_enable(fft_busy),.ch0_in(adc),.ch1_in(12'sd0),
  .input_accepted(accepted),.out_valid(hvalid),.ch0_out(h0),.ch1_out(h1));
 fft_frame_4ch #(.CONVERGENT_ROUNDING(ROUND),.CHANNELS(2),.LOG2_N(12),.ADC_FS_HZ(65000000),.FRAME_PERIOD_CLKS(0),.ROM_FILE("twiddle_4096.mem")) fft(
  .clk(clk),.rst_n(rst_n),.sample_valid(hvalid),.samples({24'd0,h1[11:0],h0[11:0]}),.busy(fft_busy),
  .result_valid(),.result_packet(),.saturated(),.spectrum_valid(sv),.spectrum_channel(sch),.spectrum_bin(sbin),.spectrum_power(spower),
  .peak_re_packed(),.peak_im_packed(),.interp_freq_packed());
 wire rv,tv;wire [31:0] thd,flags;wire [4:0] mask,noise_mask;
 wire [199:0] band,noise,signal;wire [59:0] peak_bins;
 harmonic_stream_stats stats(
  .clk(clk),.rst_n(rst_n),.enable(1'b1),.result_hold(1'b0),.cfg_id(32'd1),.reference_fcw(reference_fcw),.channel(1'b0),.supported_rates(5'b00001),
  .scan_valid(sv),.scan_channel(sch),.scan_bin(sbin),.scan_power(spower),.scan_rate_log2(4'd0),
  .scan_window(2'd0),.scan_saturated(fft.frame_saturated),.scan_age_ticks(32'd0),
  .result_valid(rv),.thd_valid(tv),.thd_q24(thd),.result_flags(flags),.result_mask(mask),.result_noise_mask(noise_mask),
  .result_band_power(band),.result_noise_power(noise),.result_signal_power(signal),.result_peak_bins(peak_bins));
 integer f;
 initial f=$fopen("../tests_build/fft_powers.csv","w");
 always @(posedge clk)begin
  if(sv && sch==0)$fdisplay(f,"%0d,%0d",sbin,spower);
  if(rv)begin
   if(!tv || mask!=31)$fatal(1,"real FFT harmonic invalid flags=%h",flags);
   $display("HARMONIC_FFT_RESULT thd_q24=%0d flags=%0d noise_mask=%0d signal=%h band=%h noise=%h peaks=%h",thd,flags,noise_mask,signal,band,noise,peak_bins);
   $fclose(f);$finish;
  end
 end
 initial begin #10000000;$fatal(1,"FFT harmonic timeout");end
endmodule
