`timescale 1ns/1ps
`default_nettype none
// H1..H5 statistics from the uncompressed linear FFT-power scan. No second FFT.
// A user-selected fundamental is required; the largest peak is NOT assumed H1.
// Hann only: +/-3 signal bins and +/-4..6 local noise bins. Signal powers are
// stored as 6*band_power - 7*noise_power so no precision is lost to /6 rounding.
module harmonic_stream_stats #(
 parameter integer LOG2_N=12,
 parameter integer FPGA_THD=1,
 parameter [39:0] MIN_BASE_SIGNAL=40'd384
)(
 input wire clk,rst_n,enable,
 input wire result_hold,
 input wire [31:0] cfg_id,reference_fcw,
 input wire channel,
 input wire [4:0] supported_rates,
 input wire scan_valid,
 input wire [1:0] scan_channel,
 input wire [LOG2_N-1:0] scan_bin,
 input wire [32:0] scan_power,
 input wire [3:0] scan_rate_log2,
 input wire [1:0] scan_window,
 input wire scan_saturated,
 input wire [31:0] scan_age_ticks,
 output reg result_valid,result_have,thd_valid,
 output reg [31:0] result_id,result_cfg,result_reference_fcw,
 output reg result_channel,
 output reg [3:0] result_rate_log2,
 output reg [1:0] result_window,
 output reg [4:0] result_mask,result_noise_mask,
 output reg [31:0] result_flags,result_age_ticks,thd_q24,dropped_frames,
 output reg [199:0] result_band_power,result_noise_power,result_signal_power,
 output reg [5*LOG2_N-1:0] result_peak_bins,
 output reg [3:0] preferred_rate,
 output wire power_write_valid,
 output wire [2:0] power_write_index,
 output wire [39:0] power_write_band,power_write_noise,power_write_signal,
 output wire [LOG2_N-1:0] power_write_peak
);
 localparam integer N=1<<LOG2_N,HALF=N/2;
 localparam [31:0] NO_BASE=1,RESOLUTION=2,PARTIAL=4,CLIPPED=8,
   BELOW_NOISE=16,THD_OVERFLOW=32,UNSUPPORTED_WINDOW=128;
 localparam [47:0] RAW_LIM=((HALF-7)*65536)+32767;
 localparam [47:0] LOW_LIM=((N/16-6)*65536)+32767;
 function [3:0] rate_at;
  input [2:0] i;
  begin case(i)0:rate_at=0;1:rate_at=4;2:rate_at=8;3:rate_at=12;default:rate_at=14;endcase end
 endfunction
 function [47:0] reference_bins_q16;
  input [31:0] fcw;input [3:0] rate;integer shift;
  reg [47:0] wide;
  begin wide={16'd0,fcw};shift=LOG2_N+rate-16;
   reference_bins_q16=shift>=0?(wide<<shift):(wide>>(-shift));end
 endfunction
 function [2:0] eligible_count;
  input [47:0] base;input [3:0] rate;
  begin
   if(base<(48'd13<<16))eligible_count=0;
   else if(base<=(rate==0?RAW_LIM/5:LOW_LIM/5))eligible_count=5;
   else if(base<=(rate==0?RAW_LIM/4:LOW_LIM/4))eligible_count=4;
   else if(base<=(rate==0?RAW_LIM/3:LOW_LIM/3))eligible_count=3;
   else if(base<=(rate==0?RAW_LIM/2:LOW_LIM/2))eligible_count=2;
   else if(base<=(rate==0?RAW_LIM:LOW_LIM))eligible_count=1;
   else eligible_count=0;
  end
 endfunction
 function [4:0] count_mask;
  input [2:0] count;
  begin case(count)1:count_mask=1;2:count_mask=3;3:count_mask=7;4:count_mask=15;5:count_mask=31;default:count_mask=0;endcase end
 endfunction
 reg configured,selecting,held_channel;
 reg [31:0] held_cfg,held_ref;
 reg [4:0] held_rates;
 reg [2:0] select_index,best_count;
 // Only an eligible candidate is retained: base <= (N/2-7)*65536+32767.
 // LOG2_N+15 bits therefore retain every bit of every accepted candidate.
 reg [LOG2_N+14:0] selected_base;
 wire [3:0] candidate_rate=rate_at(select_index);
 wire [47:0] candidate_base=reference_bins_q16(held_ref,candidate_rate);
 wire [2:0] candidate_count=eligible_count(candidate_base,candidate_rate);
 wire candidate_better=held_rates[select_index] &&
   ((candidate_count>best_count) || (candidate_count!=0 && candidate_count==best_count));
 wire config_changed=cfg_id!=held_cfg || reference_fcw!=held_ref || channel!=held_channel || supported_rates!=held_rates;
 // Rate selection already computed these exact values. Retain them rather
 // than instantiating a second barrel shifter and range-comparator chain.
 wire [LOG2_N+14:0] base_bins=selected_base;
 wire [2:0] harmonic_count=best_count;
 reg active,frame_clip;
 reg [LOG2_N-1:0] expected_bin;
 reg [1:0] frame_window;
 reg [4:0] frame_mask;
 // At most five accepted base intervals; LOG2_N+18 bits cover that bound.
 reg [LOG2_N+17:0] center_cursor;
 reg [2:0] center_index;
 reg [LOG2_N-1:0] center_bin,peak_bins[0:4];
 reg [32:0] peaks[0:4];
 // Five-cycle clearing finishes before the first eligible bin (>=7).
 // Separate asynchronous-read distributed memories retain exact 40-bit sums
 // without resetting 400 flip-flops or a ten-word parallel write decoder.
 (* ram_style="distributed" *) reg [39:0] band_sums[0:4],noise_sums[0:4];
 reg sums_clearing;
 reg [2:0] clear_index;
 reg hit,hit_main;reg [3:0] hit_address;reg [2:0] hit_harmonic;
 integer j;
 always @* begin
  hit=0;hit_main=0;hit_address=0;hit_harmonic=0;
  // The >=13-bin fundamental spacing makes the 13-bin windows disjoint.
  // The FFT scan is ascending, so only one window needs comparison per bin.
  if(frame_mask[center_index] && scan_bin>=center_bin-6 && scan_bin<=center_bin+6)begin
   hit=1;hit_main=scan_bin>=center_bin-3 && scan_bin<=center_bin+3;
   hit_address={center_index,!hit_main};hit_harmonic=center_index;
  end
 end
 wire [39:0] current_sum=hit_main?band:noise;
 (* use_dsp="yes" *) wire [39:0] next_sum=current_sum+{7'd0,scan_power};
 reg [3:0] processing;
 reg [2:0] correction_index;
 reg [41:0] harmonic_signal_sum;
 reg [31:0] calculation_age,calculation_clocks;
 reg [39:0] base_signal;
 wire [2:0] sum_read_index=processing!=0?correction_index:center_index;
 wire [39:0] band=band_sums[sum_read_index];
 wire [39:0] noise=noise_sums[sum_read_index];
 wire [42:0] wide_band={3'd0,band},wide_noise={3'd0,noise};
 (* use_dsp="yes" *) wire [42:0] six_band=(wide_band<<2)+(wide_band<<1);
 (* use_dsp="yes" *) wire [42:0] seven_noise=(wide_noise<<3)-wide_noise;
 (* use_dsp="yes" *) wire [43:0] corrected_difference={1'b0,six_band}-{1'b0,seven_noise};
 wire [39:0] corrected=corrected_difference[43]?40'd0:corrected_difference[39:0];
 (* use_dsp="yes" *) wire [41:0] harmonic_sum_next=harmonic_signal_sum+{2'd0,corrected};
 (* use_dsp="yes" *) wire [43:0] base_noise_limit=({1'b0,seven_noise}<<3)+({1'b0,seven_noise}<<1);
 (* use_dsp="yes" *) wire [LOG2_N+17:0] center_next=center_cursor+base_bins;
 (* use_dsp="yes" *) wire [LOG2_N+17:0] center_next_rounded=center_next+32768;
 (* use_dsp="yes" *) wire [31:0] calculation_clocks_next=calculation_clocks+1'b1;
 (* use_dsp="yes" *) wire [32:0] final_age={1'b0,calculation_age}+{1'b0,calculation_clocks};
 (* use_dsp="yes" *) wire [31:0] result_age_next=result_age_ticks+1'b1;
 reg div_start,sqrt_start;
 reg [89:0] numerator;
 reg [39:0] denominator;
 wire div_busy,div_valid,sqrt_busy,sqrt_done;
 wire [89:0] quotient;
 reg [63:0] radicand;
 wire [31:0] square_root;
 wire arithmetic_rst_n=rst_n && enable && configured && !config_changed;
 assign power_write_valid=rst_n && enable && !config_changed && !result_hold && processing==1;
 assign power_write_index=correction_index;
 assign power_write_band=band;assign power_write_noise=noise;assign power_write_signal=corrected;
 assign power_write_peak=peak_bins[correction_index];
 wire matching_scan=scan_valid && scan_channel=={1'b0,held_channel} && scan_rate_log2==preferred_rate;
 wire start_frame=enable && configured && !selecting && !config_changed && !result_hold && matching_scan && scan_bin==1 && !active && processing==0;
 wire memory_enabled=rst_n && enable && configured && !config_changed && !result_hold;
 wire clear_sum=start_frame || sums_clearing;
 wire accumulate_sum=active && matching_scan && scan_bin==expected_bin && scan_window==frame_window && hit;
 wire [2:0] sum_write_index=start_frame?3'd0:sums_clearing?clear_index:center_index;
 wire [39:0] sum_write_data=clear_sum?40'd0:next_sum;
 always @(posedge clk)begin
  if(memory_enabled && (clear_sum || (accumulate_sum && hit_main)))band_sums[sum_write_index]<=sum_write_data;
  if(memory_enabled && (clear_sum || (accumulate_sum && !hit_main)))noise_sums[sum_write_index]<=sum_write_data;
 end
 generate if(FPGA_THD)begin:g_fpga_thd
 fixed_divider_generic #(.NUM_W(90),.DEN_W(40),.COMPACT_REMAINDER(1)) u_ratio(
  .clk(clk),.rst_n(arithmetic_rst_n),.start(div_start),.numerator(numerator),
  .denominator(denominator),.busy(div_busy),.valid(div_valid),.quotient(quotient));
 isqrt_u64 #(.COMPACT_REMAINDER(1)) u_root(.clk(clk),.rst_n(arithmetic_rst_n),.start(sqrt_start),
  .radicand(radicand),.result(square_root),.busy(sqrt_busy),.done(sqrt_done));
 end else begin:g_host_thd
  assign div_busy=0;assign div_valid=0;assign quotient=0;
  assign sqrt_busy=0;assign sqrt_done=0;assign square_root=0;
 end endgenerate
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   configured<=0;selecting<=0;held_channel<=0;held_cfg<=0;held_ref<=0;held_rates<=0;
   select_index<=0;best_count<=0;selected_base<=0;preferred_rate<=0;
   active<=0;frame_clip<=0;expected_bin<=0;frame_window<=0;frame_mask<=0;
   center_cursor<=0;center_index<=0;center_bin<=0;
   processing<=0;correction_index<=0;harmonic_signal_sum<=0;base_signal<=0;
   calculation_age<=0;calculation_clocks<=0;div_start<=0;sqrt_start<=0;
   numerator<=0;denominator<=0;radicand<=0;
   result_valid<=0;result_have<=0;thd_valid<=0;result_id<=0;result_cfg<=0;
   result_reference_fcw<=0;result_channel<=0;result_rate_log2<=0;result_window<=0;
   result_mask<=0;result_noise_mask<=0;result_flags<=0;result_age_ticks<=0;thd_q24<=0;
   dropped_frames<=0;result_band_power<=0;result_noise_power<=0;result_signal_power<=0;result_peak_bins<=0;
   sums_clearing<=0;clear_index<=0;
   for(j=0;j<5;j=j+1)begin peaks[j]<=0;peak_bins[j]<=0;end
  end else begin
   result_valid<=0;div_start<=0;sqrt_start<=0;
   if(result_have && result_age_ticks!=32'hffffffff)result_age_ticks<=result_age_next;
   // Hold is asserted only for an already completed result (active=0,
   // processing=0). Gate the entry points, not every arithmetic register.
   if(!enable && !result_hold)begin configured<=0;selecting<=0;active<=0;processing<=0;sums_clearing<=0;result_have<=0;thd_valid<=0;end
   else if(!result_hold && (config_changed || (!configured && !selecting)))begin
    held_cfg<=cfg_id;held_ref<=reference_fcw;held_channel<=channel;held_rates<=supported_rates;
    configured<=0;selecting<=1;select_index<=0;best_count<=0;selected_base<=0;preferred_rate<=0;
    active<=0;processing<=0;sums_clearing<=0;result_have<=0;thd_valid<=0;
   end else if(selecting && !result_hold)begin
    if(candidate_better)begin best_count<=candidate_count;preferred_rate<=candidate_rate;selected_base<=candidate_base;end
    if(select_index==4)begin configured<=1;selecting<=0;end
    else select_index<=select_index+1'b1;
   end else if(configured)begin
    if(sums_clearing)begin
     if(clear_index==4)sums_clearing<=0;else clear_index<=clear_index+1'b1;
    end
    if(!result_hold && scan_valid && scan_channel=={1'b0,held_channel} && scan_rate_log2==preferred_rate)begin
     if(scan_bin==1)begin
      if(active || processing!=0)begin active<=0;dropped_frames<=dropped_frames+1'b1;end
      else begin
       active<=1;expected_bin<=2;frame_clip<=scan_saturated;frame_window<=scan_window;
       frame_mask<=count_mask(harmonic_count);center_index<=0;center_cursor<=base_bins;
       center_bin<=(base_bins+48'd32768)>>16;
       sums_clearing<=1;clear_index<=1;
       for(j=0;j<5;j=j+1)begin peaks[j]<=0;peak_bins[j]<=0;end
      end
     end else if(active)begin
      if(scan_bin!=expected_bin || scan_window!=frame_window)begin active<=0;dropped_frames<=dropped_frames+1'b1;end
      else begin
       expected_bin<=expected_bin+1'b1;frame_clip<=frame_clip || scan_saturated;
       if(scan_bin==center_bin+6 && center_index<4)begin
        center_index<=center_index+1'b1;center_cursor<=center_next;
        center_bin<=center_next_rounded>>16;
       end
       if(hit)begin
        if(hit_main && scan_power>peaks[hit_harmonic])begin peaks[hit_harmonic]<=scan_power;peak_bins[hit_harmonic]<=scan_bin;end
       end
       if(scan_bin==HALF)begin
        active<=0;processing<=1;correction_index<=0;harmonic_signal_sum<=0;result_have<=0;
        calculation_age<=scan_age_ticks;calculation_clocks<=0;
        result_cfg<=held_cfg;result_reference_fcw<=held_ref;result_channel<=held_channel;
        result_rate_log2<=preferred_rate;result_window<=frame_window;
        result_mask<=frame_mask;result_noise_mask<=0;thd_valid<=0;thd_q24<=0;
        result_flags<=(frame_mask==31?32'd0:PARTIAL) |
          (base_bins<(48'd13<<16)?RESOLUTION:32'd0) |
          ((frame_clip || scan_saturated)?CLIPPED:32'd0) |
          (frame_window!=0?UNSUPPORTED_WINDOW:32'd0);
       end
      end
     end
    end
    if(processing!=0)calculation_clocks<=calculation_clocks_next;
    case(processing)
     1:begin
      // Shift in H1..H5; after five cycles H1 occupies the least-significant
      // word. This avoids a wide variable-write decoder without losing bits.
      result_band_power<={band,result_band_power[199:40]};
      result_noise_power<={noise,result_noise_power[199:40]};
      result_signal_power<={corrected,result_signal_power[199:40]};
      result_peak_bins<={peak_bins[correction_index],result_peak_bins[5*LOG2_N-1:LOG2_N]};
      if(frame_mask[correction_index] && {3'd0,corrected}<=seven_noise)begin
       result_noise_mask[correction_index]<=1;
       if(correction_index!=0)result_flags<=result_flags | BELOW_NOISE;
      end
      if(correction_index==0)begin
       base_signal<=corrected;
       if(!frame_mask[0] || corrected<MIN_BASE_SIGNAL || {4'd0,corrected}<=base_noise_limit)
        result_flags<=result_flags | NO_BASE;
      end else harmonic_signal_sum<=harmonic_sum_next;
      if(correction_index==4)processing<=2;else correction_index<=correction_index+1'b1;
     end
     2:begin
      if((result_flags & (NO_BASE|RESOLUTION|PARTIAL|CLIPPED|UNSUPPORTED_WINDOW))!=0 || base_signal==0)processing<=6;
      else if(!FPGA_THD)begin thd_valid<=1;processing<=6;end
      else begin numerator<={harmonic_signal_sum,48'd0};denominator<=base_signal;div_start<=1;processing<=3;end
     end
     3:if(div_valid)begin
      if(|quotient[89:64])begin result_flags<=result_flags | THD_OVERFLOW;thd_q24<=32'hffffffff;processing<=6;end
      else begin radicand<=quotient[63:0];sqrt_start<=1;processing<=4;end
     end
     4:if(sqrt_done)begin thd_q24<=square_root;thd_valid<=1;processing<=6;end
     6:begin
      result_id<=result_id+1'b1;
      result_age_ticks<=final_age[32]?32'hffffffff:final_age[31:0];
      result_have<=1;result_valid<=1;processing<=0;
     end
     default:begin end
    endcase
   end
  end
 end
 // synthesis translate_off
 initial if(LOG2_N<8 || LOG2_N>13)$fatal(1,"Unsupported harmonic FFT length");
 // synthesis translate_on
endmodule
`default_nettype wire
