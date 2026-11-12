`timescale 1ns/1ps
`default_nettype none
// One exact statistics engine and one bounded UART result slot. Acquisition
// of the existing FFT/WAVE/ANA paths never waits for this optional link.
module harmonic_analysis_link #(
 parameter integer CLK_FREQ=65000000,BAUD=2000000,STOP_BITS=1,LOG2_N=12,FPGA_THD=1
)(
 input wire clk,rst_n,enable,channel,
 input wire [31:0] cfg_id,reference_fcw,
 input wire [4:0] supported_rates,
 input wire scan_valid,scan_saturated,
 input wire [1:0] scan_channel,scan_window,
 input wire [LOG2_N-1:0] scan_bin,
 input wire [32:0] scan_power,
 input wire [3:0] scan_rate_log2,
 input wire [31:0] scan_age_ticks,
 input wire send_start,
 output wire ready,busy,tx,
 output wire [31:0] completed
);
 wire valid,hold,thd_valid,result_channel;
 wire [31:0] id,cfg,reference_word,flags,age,thd,drops;
 wire [3:0] rate;wire [1:0] window_id;wire [4:0] mask,noise_mask;
 wire [39:0] band_power,noise_power,signal_power,write_band,write_noise,write_signal;
 wire [LOG2_N-1:0] write_peak;
 wire [11:0] read_peak;
 wire write_valid;wire [2:0] write_index,read_index;
 harmonic_power_store powers(.clk(clk),.write_valid(write_valid),.write_index(write_index),.read_index(read_index),
  .write_band(write_band),.write_noise(write_noise),.write_signal(write_signal),
  .write_peak({{(12-LOG2_N){1'b0}},write_peak}),.band(band_power),.noise(noise_power),.signal_power(signal_power),.peak(read_peak));
 harmonic_stream_stats #(.LOG2_N(LOG2_N),.FPGA_THD(FPGA_THD)) stats(
  .clk(clk),.rst_n(rst_n),.enable(enable),.result_hold(hold),.channel(channel),
  .cfg_id(cfg_id),.reference_fcw(reference_fcw),.supported_rates(supported_rates),
  .scan_valid(scan_valid),.scan_saturated(scan_saturated),.scan_channel(scan_channel),.scan_window(scan_window),
  .scan_bin(scan_bin),.scan_power(scan_power),.scan_rate_log2(scan_rate_log2),.scan_age_ticks(scan_age_ticks),
  .result_valid(valid),.result_have(),.thd_valid(thd_valid),.result_id(id),.result_cfg(cfg),
  .result_reference_fcw(reference_word),.result_channel(result_channel),.result_rate_log2(rate),
  .result_window(window_id),.result_mask(mask),.result_noise_mask(noise_mask),
  .result_flags(flags),.result_age_ticks(age),.thd_q24(thd),.dropped_frames(drops),
  .result_band_power(),.result_noise_power(),.result_signal_power(),.result_peak_bins(),.preferred_rate(),
  .power_write_valid(write_valid),.power_write_index(write_index),.power_write_band(write_band),
  .power_write_noise(write_noise),.power_write_signal(write_signal),.power_write_peak(write_peak));
 harmonic_result_tx #(.CLK_FREQ(CLK_FREQ),.BAUD(BAUD),.STOP_BITS(STOP_BITS),.NFFT(1<<LOG2_N),.HOST_THD(!FPGA_THD)) sender(
  .clk(clk),.rst_n(rst_n),.result_valid(valid),.send_start(send_start),.result_hold(hold),
  .result_id(id),.result_cfg(cfg),.reference_fcw(reference_word),.age_ticks(age),.flags(flags),
  .thd_q24(thd),.dropped_frames(drops),.channel(result_channel),.thd_valid(thd_valid),
  .rate_log2(rate),.window_id(window_id),.valid_mask(mask),.noise_mask(noise_mask),
  .band_power(band_power),.noise_power(noise_power),.signal_power(signal_power),.peak_bin(read_peak),.power_read_index(read_index),
  .ready(ready),.busy(busy),.tx(tx),.completed(completed));
endmodule
`default_nettype wire
