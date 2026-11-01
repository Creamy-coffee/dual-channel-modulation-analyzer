`timescale 1ns/1ps
`default_nettype none
// ---------------------------------------------------------------------------
// freq_equal_precision : equal-precision frequency measurement, one channel.
//
// Method: same equal-precision scheme as the cymometer in the
// peiceliu/PicoRV32_ProtableLab reference (fx = cnt_fx * fs / Ns), kept here
// in the single 65 MHz clock domain:
//   1. A pulse_gen style hysteresis comparator (+/-THR_DATA around a mid
//      level) turns the ADC code stream into a pulse train; every rising edge
//      is one signal period.
//   2. A software gate of exactly GATE_SAMPLES valid samples counts the
//      rising edges (cnt_fx).  The gate is counted in SAMPLES (adc_valid),
//      not in clock cycles, so the interval stays exact even if the ADC
//      stalls (the gate then simply lasts longer in wall-clock time).
//   3. fx = cnt_fx * CLK_FREQ / GATE_SAMPLES.
//
// Because GATE_SAMPLES is a compile-time constant, CLK_FREQ/GATE_SAMPLES is
// folded into SCALE_SHIFT and the reading is a plain shift:
//      freq_hz = cnt_fx << SCALE_SHIFT          (no multiplier, no divider)
// GATE_SAMPLES must therefore be CLK_FREQ/2^m; the assertion below makes a
// non-conforming gate a build error.  Practical gates at 65 MHz are 1 s,
// 0.5 s, 0.25 s, 125 ms, 62.5 ms, 31.25 ms and 15.6 ms.  Longer gate ->
// finer resolution (quantisation ~ 1/cnt_fx); the reference project used a
// 1.5 s gate, the default here is 1 s.
//
// Mid level: with USE_DC_TRACKER the threshold follows a slow IIR estimate of
// the signal mean (time constant 2^TRACK_SHIFT samples, ~15 ms at 65 MHz),
// so a DC offset does not shift the crossing point.  The estimate is far too
// slow to follow the signal itself, and the edge COUNT is unaffected by
// threshold wobble anyway: every period still produces exactly one rising
// edge as long as the threshold stays inside the swing.  With
// USE_DC_TRACKER=0 the fixed trig_level input is used instead.
//
// Outputs hold their value between measurements; result_id increments once
// per completed gate so a consumer can tell a fresh reading from a stale one
// (a 1 s gate means the value only changes about once per second).
// ---------------------------------------------------------------------------
module freq_equal_precision #(
 parameter integer CLK_FREQ = 65000000,
 parameter integer GATE_SAMPLES = 65000000,   // 1 s of valid samples at 65 MHz
 parameter integer DATA_W = 12,
 parameter [DATA_W-1:0] THR_DATA = 12'd3,     // hysteresis half width, codes
 parameter integer TRACK_SHIFT = 20,          // DC tracker IIR shift (samples)
 parameter integer USE_DC_TRACKER = 1
)(
 input wire clk,
 input wire rst_n,
 input wire adc_valid,                        // sample strobe (full rate)
 input wire signed [DATA_W-1:0] adc_data,
 input wire signed [DATA_W-1:0] trig_level,   // used when USE_DC_TRACKER=0
 output reg [31:0] freq_hz,                   // Hz, average over the gate
 output reg [31:0] result_id,                 // increments per completed gate
 output reg result_valid                      // 1-cycle pulse per result
);
 // SCALE_SHIFT = log2(CLK_FREQ/GATE_SAMPLES).  The power-of-two gate keeps
 // the division exact and free.  A non-conforming gate is a build error.
 localparam integer SCALE_SHIFT = $clog2(CLK_FREQ/GATE_SAMPLES);
 localparam integer ACC_W = TRACK_SHIFT + DATA_W;   // DC tracker accumulator

 // ---- mid level: slow DC tracker or the fixed input ----
 reg signed [ACC_W-1:0] dc_acc;
 reg tracker_init;
 wire signed [DATA_W-1:0] dc_est = dc_acc[TRACK_SHIFT+DATA_W-1:TRACK_SHIFT];
 wire signed [DATA_W-1:0] threshold =
   (USE_DC_TRACKER != 0) ? dc_est : trig_level;
 wire signed [ACC_W-1:0] data_ext =
   {{(ACC_W-DATA_W){adc_data[DATA_W-1]}}, adc_data};
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   dc_acc <= {ACC_W{1'b0}};
   tracker_init <= 1'b0;
  end else if(USE_DC_TRACKER != 0) begin
   if(!tracker_init && adc_valid) begin
    dc_acc <= data_ext <<< TRACK_SHIFT;       // seed with the first sample
    tracker_init <= 1'b1;
   end else if(adc_valid) begin
    dc_acc <= dc_acc + ((data_ext - dc_acc) >>> TRACK_SHIFT);
   end
  end
 end

 // ---- hysteresis comparator (pulse_gen style) ----
 wire signed [DATA_W:0] level_ext = {threshold[DATA_W-1], threshold};
 wire signed [DATA_W:0] data_ext_w = {adc_data[DATA_W-1], adc_data};
 wire signed [DATA_W:0] thr_hi = level_ext + $signed({1'b0, THR_DATA});
 wire signed [DATA_W:0] thr_lo = level_ext - $signed({1'b0, THR_DATA});
 reg pulse, pulse_dly;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) pulse <= 1'b0;
  else if(adc_valid) begin
   if(data_ext_w < thr_lo) pulse <= 1'b0;
   else if(data_ext_w > thr_hi) pulse <= 1'b1;
  end
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) pulse_dly <= 1'b0;
  else if(adc_valid) pulse_dly <= pulse;
 end
 wire pulse_rise = pulse & ~pulse_dly;

 // ---- software gate + rising-edge counter ----
 // The gate advances only on valid samples, so it spans exactly
 // GATE_SAMPLES samples; cnt_fx counts pulse rises inside that span.
 reg [31:0] gate_cnt;
 reg [31:0] cnt_fx;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   gate_cnt <= 32'd0;
   cnt_fx <= 32'd0;
   freq_hz <= 32'd0;
   result_id <= 32'd0;
   result_valid <= 1'b0;
  end else begin
   result_valid <= 1'b0;
   if(adc_valid) begin
    if(gate_cnt == GATE_SAMPLES-1) begin
     // Gate complete.  This sample is inside the window, so a rising edge on
     // it belongs to the result; include it before clearing the counter.
     freq_hz <= (cnt_fx + (pulse_rise ? 32'd1 : 32'd0)) <<< SCALE_SHIFT;
     result_valid <= 1'b1;
     result_id <= result_id + 32'd1;
     gate_cnt <= 32'd0;
     cnt_fx <= 32'd0;
    end else begin
     gate_cnt <= gate_cnt + 32'd1;
     if(pulse_rise) cnt_fx <= cnt_fx + 32'd1;
    end
   end
  end
 end
 // synthesis translate_off
 initial begin
  if(GATE_SAMPLES < 2 || (GATE_SAMPLES << SCALE_SHIFT) != CLK_FREQ)
   $fatal(1,"freq_equal_precision: GATE_SAMPLES must be CLK_FREQ/2^m, m>=1");
  if(DATA_W < 2 || DATA_W > 24)
   $fatal(1,"freq_equal_precision: DATA_W must be 2..24");
  if(TRACK_SHIFT < 1 || TRACK_SHIFT > 24)
   $fatal(1,"freq_equal_precision: TRACK_SHIFT must be 1..24");
 end
 // synthesis translate_on
endmodule
`default_nettype wire
