`timescale 1ns/1ps
`default_nettype none
// One FFT engine, five independently prepared time windows. Low-rate capture
// never holds the FFT while waiting for physical samples. Native RAW is served
// between ready low-rate frames, so a 1-second low window cannot block RF search.
// Analysis bands intentionally occupy <= Fs_out/16 of each CIC output: the
// CIC is not a brick-wall LPF. Aliases entering that restricted passband are
// attenuated; behaviour outside those bands is never advertised as calibrated.
module multirate_fft_frontend #(
 parameter integer CLK_FREQ=65000000,LOG2_N=12,MULTIRATE_ENABLE=1,SPECTRUM_AUTO=1,
 parameter [31:0] AMP_THRESHOLD=64
)(
 input wire clk,rst_n,enable,adc_valid,
 input wire signed [11:0] adc_ch0,adc_ch1,
 output wire result_valid,saturated,
 output wire [511:0] result_packet,
 output wire [127:0] interp_freq,
 output wire [127:0] conversion_frequency,
 output wire [1:0] qualified,
 output wire [3:0] analysis_log2,
 output wire [31:0] age_ticks,
 output wire spectrum_valid,
 output wire [31:0] spectrum_fs_hz,
    output wire [LOG2_N-1:0] spectrum_bin,
    output wire [32:0] spectrum_power,
    output wire [2:0] analysis_band,
    output wire harmonic_scan_valid,harmonic_scan_saturated,
    output wire [1:0] harmonic_scan_channel
);
 wire clear=!enable || !adc_valid;
 reg core_ok;
 always @(posedge clk)if(!rst_n)core_ok<=0;else core_ok<=!clear;
 wire core_rst_n=rst_n && core_ok;
 wire [4:0] sv;
 wire signed [11:0] data0[0:4],data1[0:4];
 assign sv[0]=adc_valid && enable;
 assign data0[0]=adc_ch0;assign data1[0]=adc_ch1;
 function signed [11:0] round_sample;
  input signed [19:0] x;
  reg signed [20:0] wide,q;
  begin
   wide=$signed({x[19],x});
   q=wide>=0?((wide+21'sd128)>>>8):-(((-wide+21'sd128)>>>8));
   if(q>2047)round_sample=2047;
   else if(q< -2048)round_sample=-2048;
   else round_sample=q[11:0];
  end
 endfunction
 wire signed [19:0] filt0[0:4],filt1[0:4];
 assign filt0[0]={adc_ch0,8'd0};assign filt1[0]={adc_ch1,8'd0};
 genvar g;
 generate for(g=1;g<5;g=g+1)begin: filters
  if(MULTIRATE_ENABLE!=0)begin: active
   cic3_decimate_2ch #(.LOG2_R(g==4?2:4)) u_filter(
    .clk(clk),.rst_n(rst_n),.clear(clear),.in_valid(sv[g-1]),
    .in0(filt0[g-1]),.in1(filt1[g-1]),.out_valid(sv[g]),
    .out0(filt0[g]),.out1(filt1[g]));
   assign data0[g]=round_sample(filt0[g]);assign data1[g]=round_sample(filt1[g]);
  end else begin: absent
   assign sv[g]=0;assign data0[g]=0;assign data1[g]=0;
   assign filt0[g]=0;assign filt1[g]=0;
  end
 end endgenerate
 wire [4:0] ready;
 wire signed [11:0] rd0[0:4],rd1[0:4],mean0[0:4],mean1[0:4];
 wire [31:0] cap_end[0:4];
 reg [LOG2_N-1:0] rd_addr;
 reg [2:0] band;
 localparam SELECT=0,PRIME=1,FEED=2,WAIT_RESULT=3;
 reg [1:0] state;
 reg [LOG2_N-1:0] sent;
 wire input_accepted;
 wire release_now=state==FEED && (&sent) && input_accepted;
 generate for(g=0;g<5;g=g+1)begin: banks
  if(g==0 || MULTIRATE_ENABLE!=0)begin: active
   fft_capture_bank #(.AW(LOG2_N)) u_bank(
    .clk(clk),.rst_n(rst_n),.clear(clear),.in_valid(sv[g]),
    .in0(data0[g]),.in1(data1[g]),.release_frame(release_now && band==g),
    .rd_addr(rd_addr),.rd0(rd0[g]),.rd1(rd1[g]),
    .mean0(mean0[g]),.mean1(mean1[g]),.ready(ready[g]),.capture_end(cap_end[g]));
  end else begin: absent
   assign rd0[g]=0;assign rd1[g]=0;assign mean0[g]=0;assign mean1[g]=0;
   assign ready[g]=0;assign cap_end[g]=0;
  end
 end endgenerate
 reg prefer_raw;
 reg [2:0] rr;
 reg [2:0] choice;
 reg available;
 integer j,probe;
 always @*begin
  choice=0;available=0;probe=1;
  if(prefer_raw && ready[0])begin choice=0;available=1;end
  else begin
   for(j=3;j>=0;j=j-1)begin
    probe=rr+j;if(probe>4)probe=probe-4;
    if(ready[probe])begin choice=probe;available=1;end
   end
   if(!available && ready[0])begin choice=0;available=1;end
  end
 end
 reg signed [11:0] m0,m1;
 reg [31:0] ticks,end_hold;
 reg clipped_frame;
 // A sub-cycle arc can acquire a spurious high bin from fixed-point window /
 // FFT quantization. The declared analysis bands start at ~8 cycles/frame;
 // require six real low-to-high traversals with a two-count noise deadband.
 // This is a single-dominant-periodic-signal acquisition condition, not a
 // general waveform classifier or a replacement for measured noise limits.
 reg [1:0] polarity0,polarity1;
 reg [7:0] crossings0,crossings1;
 wire signed [12:0] x0=$signed({rd0[band][11],rd0[band]})-$signed({m0[11],m0});
 wire signed [12:0] x1=$signed({rd1[band][11],rd1[band]})-$signed({m1[11],m1});
 wire clipped=(x0>2047 || x0< -2048 || x1>2047 || x1< -2048);
 wire signed [11:0] centered0=x0>2047?12'sd2047:x0< -2048?-12'sd2048:x0[11:0];
 wire signed [11:0] centered1=x1>2047?12'sd2047:x1< -2048?-12'sd2048:x1[11:0];
 wire fft_busy,fft_valid,fft_sat,hann_valid;
 wire spec_valid;wire [1:0] spec_ch;wire fft_scan_saturated;
 assign harmonic_scan_valid=spec_valid && core_ok && !clear;
 assign harmonic_scan_channel=spec_ch;
 assign harmonic_scan_saturated=fft_scan_saturated || clipped_frame;
 // Publish the CH0 analysis band which actually qualified a signal. Selection
 // changes only at fft_valid (after both complete channel scans), never in
 // the middle of a spectrum frame. Wait for initial qualification rather
 // than publishing a misleading full-rate first bin for a low-frequency input.
 reg [2:0] display_band;
 reg display_band_valid;
 assign spectrum_valid=spec_valid && spec_ch==0 && core_ok && !clear &&
   (SPECTRUM_AUTO ? (display_band_valid && band==display_band) : band==0);
 // V1 has an integer-Hz effective-rate field. Round each implemented rate;
 // retain the exact ADC rate and ratio separately in the measurement packet.
 assign spectrum_fs_hz=band==0 ? CLK_FREQ :
   band==1 ? ((CLK_FREQ+8)>>4) : band==2 ? ((CLK_FREQ+128)>>8) :
   band==3 ? ((CLK_FREQ+2048)>>12) : ((CLK_FREQ+8192)>>14);
 wire signed [15:0] hann0,hann1;
 wire [127:0] raw_freq;
 hann_window_2ch #(.LOG2_N(LOG2_N),.COEFF_FILE("hann_4096_half_q15.mem"),
  .HALF_WINDOW_OPT(1),.FRAME_GATED(1)) u_hann(
  .clk(clk),.rst_n(core_rst_n),.in_valid(state==FEED && !clear),.frame_enable(fft_busy),
  .ch0_in(centered0),.ch1_in(centered1),.input_accepted(input_accepted),
  .out_valid(hann_valid),.ch0_out(hann0),.ch1_out(hann1));
 fft_frame_4ch #(.CHANNELS(2),.LOG2_N(LOG2_N),.ADC_FS_HZ(CLK_FREQ),
  .DECIM_LOG2(0),.FRAME_PERIOD_CLKS(0),.ROM_FILE("twiddle_4096.mem")) u_fft(
  .clk(clk),.rst_n(core_rst_n),.sample_valid(hann_valid),
  .samples({24'd0,hann1[11:0],hann0[11:0]}),.busy(fft_busy),
  .result_valid(fft_valid),.result_packet(result_packet),.saturated(fft_sat),
  .interp_freq_packed(raw_freq),.spectrum_valid(spec_valid),.spectrum_channel(spec_ch),
  .spectrum_bin(spectrum_bin),.spectrum_power(spectrum_power),.peak_re_packed(),.peak_im_packed(),
  .scan_saturated(fft_scan_saturated));
 function [3:0] shift;
  input [2:0] b;
  begin case(b)0:shift=0;1:shift=4;2:shift=8;3:shift=12;default:shift=14;endcase end
 endfunction
 // Shift only between the five implemented constants (not a general divider).
 function [31:0] physical_hz;
  input [31:0] value;input [2:0] b;
  begin case(b)
   0:physical_hz=value;1:physical_hz=(value+8)>>4;2:physical_hz=(value+128)>>8;
   3:physical_hz=(value+2048)>>12;default:physical_hz=(value+8192)>>14;
  endcase end
 endfunction
 function in_band;
  input [31:0] hz;input [2:0] b;
  begin case(b)
   0:in_band=MULTIRATE_ENABLE==0 || (hz>=120000 && hz<CLK_FREQ/2);
   1:in_band=hz>=8000 && hz<=200000;
   2:in_band=hz>=500 && hz<=12500;
   3:in_band=hz>=30 && hz<=800;
   default:in_band=hz>=8 && hz<=200;
  endcase end
 endfunction
 wire [31:0] hz0=physical_hz(raw_freq[31:0],band),hz1=physical_hz(raw_freq[63:32],band);
 wire [32:0] mag0={result_packet[96],result_packet[95:64]},mag1={result_packet[224],result_packet[223:192]};
 assign interp_freq={64'd0,hz1,hz0};
 assign conversion_frequency=raw_freq;
 assign result_valid=fft_valid && core_ok && !clear;
 assign saturated=fft_sat || clipped_frame;
 assign qualified[0]=!saturated && crossings0>=6 && mag0>={1'b0,AMP_THRESHOLD} && in_band(hz0,band);
 assign qualified[1]=!saturated && crossings1>=6 && mag1>={1'b0,AMP_THRESHOLD} && in_band(hz1,band);
 assign analysis_log2=shift(band);assign analysis_band=band;
 assign age_ticks=ticks-end_hold;
 always @(posedge clk) begin
  if(!rst_n || clear || !core_ok)begin display_band<=0;display_band_valid<=0;end
  // Existing analysis bands overlap. Keep a valid selected band across that
  // overlap instead of making the screen axis alternate on every FFT visit.
  else if(result_valid && qualified[0] &&
          (!display_band_valid || band==display_band || !in_band(hz0,display_band)))begin
   display_band<=band;display_band_valid<=1;
  end
 end
 always @(posedge clk)begin
  if(!rst_n || clear || !core_ok)begin
   state<=SELECT;band<=0;rd_addr<=0;sent<=0;m0<=0;m1<=0;
   ticks<=0;end_hold<=0;clipped_frame<=0;prefer_raw<=1;rr<=1;
   polarity0<=0;polarity1<=0;crossings0<=0;crossings1<=0;
  end else begin
   ticks<=ticks+1'b1;
   case(state)
    SELECT:if(available)begin
     band<=choice;m0<=mean0[choice];m1<=mean1[choice];end_hold<=cap_end[choice];
     rd_addr<=0;sent<=0;clipped_frame<=0;state<=PRIME;
     polarity0<=0;polarity1<=0;crossings0<=0;crossings1<=0;
     prefer_raw<=choice!=0;
     if(choice!=0)rr<=choice==4?1:choice+1'b1;
    end
    PRIME:begin rd_addr<=1;state<=FEED;end
    FEED:if(input_accepted)begin
     if(clipped)clipped_frame<=1;
     if(x0< -1)polarity0<=1;
     else if(x0>1)begin
      if(polarity0==1 && crossings0!=255)crossings0<=crossings0+1'b1;
      polarity0<=2;
     end
     if(x1< -1)polarity1<=1;
     else if(x1>1)begin
      if(polarity1==1 && crossings1!=255)crossings1<=crossings1+1'b1;
      polarity1<=2;
     end
     rd_addr<=rd_addr+1'b1;
     if(&sent)state<=WAIT_RESULT;else sent<=sent+1'b1;
    end
    WAIT_RESULT:if(fft_valid)state<=SELECT;
   endcase
  end
 end
endmodule
`default_nettype wire
