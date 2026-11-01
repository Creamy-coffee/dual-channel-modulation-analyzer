`timescale 1ns/1ps
`default_nettype none
// ---------------------------------------------------------------------------
// fft_find_ctrl - 4096-point shared-FFT carrier finder + AFC bridge.
//
// WHY THIS EXISTS
// ---------------
// freq_scan_ctrl finds the carrier by sweeping the lock-in reference across
// the band (15 kHz coarse steps -> 900 Hz fine steps -> park, ~220 ms per
// full pass).  It works, but it burns one lock-in window per candidate and
// it needs the mixer/DDS datapath to do the searching.
//
// This module replaces the sweep with a 4096-point Hann-windowed FFT that
// covers the WHOLE band in a single ~5.5 ms frame (one shared engine, CH0
// and CH1 time-multiplexed), then hands the 3-point interpolated peak
// frequency to the EXISTING local_afc_2ch + lock-in chain.  Division of
// labour, which is the whole point of the port:
//
//   fft_frame_4ch : coarse frequency + per-frame spectrum peak, full band.
//   local_afc_2ch : fine tracking, +/-2 kHz capture around the committed FCW.
//   lockin_core   : precision amplitude / phase at the AFC-tuned reference.
//
// The FFT never produces amplitude or phase results, and the AFC never
// sweeps: each block does the one job it is good at.  find_fcw only moves on
// a committed frame; local_afc_2ch owns the fine frequency afterwards.
//
// FRAME TIMING (65 MHz, LOG2_N=12, CHANNELS=2, FRAME_PERIOD_CLKS=0)
//   capture 4096 samples, then per channel: 12 stages x 2048 butterflies
//   x 7 cycles + 2049-bin scan + 48-cycle interpolation.  Two channels
//   ~= 356k cycles ~= 5.5 ms per full-band frame.  The FFT keeps running
//   while locked so loss detection and the per-frame telemetry stay live.
//
// DETECTION SCALE
//   fft_frame_4ch normalises by 1/N and scales its input <<4, and the Hann
//   coherent gain is 0.5, so a tone of peak amplitude A (ADC counts) lights
//   the peak bin with mag = re^2+im^2 = 16*A^2.  AMP_THRESHOLD is in those
//   units; the default 256 corresponds to A = 4 counts.  The frequency
//   handed to the AFC is the INTERPOLATED one (interp_freq_packed, Q16
//   bins): the packet's own freq field is only the bin centre, and a tone
//   can sit up to +/-7.9 kHz (half a bin) away from it - outside the +/-2 kHz
//   AFC capture range.
//
// STATE MACHINE (state output)
//   0 IDLE    disabled; find_fcw parked at F_MIN_FCW so config_ok stays true
//   1 SEARCH  waiting for an FFT frame; frames evaluated one by one
//   2 CONV    Hz -> FCW division in flight (64 cycles)
//   3 SETTLE  committed; waiting SETTLE_CLKS for local_afc_2ch to lock
//   4 TRACK   locked; FFT keeps running for loss detection + telemetry
//   5 LOST    one-cycle marker, then back to SEARCH.  'committed' is cleared
//             on every loss so the re-acquire commit is never blocked by the
//             MIN_STEP guard.
//
// AFC-OFF BUILDS (AFC_ENABLE=0) force afc_ready=1 inside lockin_measurement,
// so pass afc_en=(AFC_ENABLE!=0): the drift term of track_loss is then the
// only mover and a stepped carrier is still re-acquired.
// ---------------------------------------------------------------------------
module fft_find_ctrl #(
    parameter [31:0]  CLK_FREQ = 32'd65000000,
    parameter integer LOW_FFT_ENABLE=0,
    parameter integer LOG2_N   = 12,               // 4096-point shared FFT
    parameter [31:0]  F_MIN_FCW = 32'd13215,       // 200 Hz
    parameter [31:0]  F_MAX_FCW = 32'h7FFFFFFF,    // Nyquist
    parameter [31:0]  AMP_THRESHOLD = 32'd256,     // mag = 16*A^2 -> A = 4 counts
    parameter [31:0]  MIN_STEP_FCW = 32'd1048576,  // 1 bin (15.87 kHz) anti-dither
    parameter [31:0]  REACQ_FCW = 32'd264306,      // 4 kHz adopt / keep window
    parameter [31:0]  FAST_REACQ_FCW = (32'd2 << (32-LOG2_N)), // 2 FFT bins (~31.7 kHz) fast re-scan floor
    parameter [31:0]  SETTLE_CLKS = 32'd2000000,   // 30.8 ms AFC settle budget
    parameter [15:0]  SUPPRESS_FRAMES = 16'd8,     // ignore a failed freq for N frames
    parameter [31:0]  LOST_HOLD_CLKS = 32'd13000000, // 200 ms !afc_ready while locked
    parameter [7:0]   LOST_FRAMES = 8'd8           // sub-threshold frames while locked
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,          // daq_top cfg_enable
    input  wire        adc_valid,       // gated ADC valid (raw_valid && !otr)
    input  wire signed [11:0] adc_ch0,  // zero-bias calibrated samples
    input  wire signed [11:0] adc_ch1,
    input  wire        afc_en,          // AFC_ENABLE of the build
    input  wire        afc_ready,       // lock-in tracking_ready
    input  wire [31:0] li_active_fcw,   // lock-in active reference (AFC tuned)
    input  wire        rescan,          // host request: drop lock, search again
    output reg  [31:0] find_fcw,        // -> cfg_fcw (parked frequency)
    output reg         commit,          // -> cfg_commit pulse
    output reg         locked,
    output reg         busy,            // not locked yet: measurements not usable
    output reg  [2:0]  state,
    output reg  [31:0] fft_freq0,       // CH0 interpolated peak, Hz
    output reg  [31:0] fft_freq1,       // CH1 interpolated peak, Hz
    output reg  [11:0] fft_bin0,
    output reg  [11:0] fft_bin1,
    output reg  [23:0] fft_mag0,        // mag>>3 (telemetry scale)
    output reg  [23:0] fft_mag1,
    output reg  [31:0] frame_count,     // published frames
    output reg  [15:0] lost_count,
    output reg [1:0] fft_valid_channels,fft_update,
    output reg [3:0] fft_rate0,fft_rate1,
    output wire spectrum_valid,
    output wire [31:0] spectrum_fs_hz,
    output wire [LOG2_N-1:0] spectrum_bin,
    output wire [32:0] spectrum_power,
    output reg [31:0] fft_age0,fft_age1,
    output wire harmonic_scan_valid,harmonic_scan_saturated,
    output wire [1:0] harmonic_scan_channel,
    output wire [3:0] harmonic_scan_rate,
    output wire [31:0] harmonic_scan_age
);
    localparam [2:0] S_IDLE=3'd0, S_SEARCH=3'd1, S_CONV=3'd2,
                     S_SETTLE=3'd3, S_TRACK=3'd4, S_LOST=3'd5;
    // FCW <-> Hz constants, folded at elaboration.  64-bit on purpose: a
    // plain 32-bit F_MIN_FCW*CLK_FREQ overflows (13215*65e6 = 8.6e11).
    localparam [63:0] F_MIN_HZ = ({32'd0,F_MIN_FCW} * {32'd0,CLK_FREQ}) >> 32;
    localparam [63:0] F_MAX_HZ = ({32'd0,F_MAX_FCW} * {32'd0,CLK_FREQ}) >> 32;
    localparam [63:0] REACQ_HZ = ({32'd0,REACQ_FCW} * {32'd0,CLK_FREQ}) >> 32;
    localparam [63:0] FAST_REACQ_HZ = ({32'd0,FAST_REACQ_FCW} * {32'd0,CLK_FREQ}) >> 32;

    // ---- helpers ----------------------------------------------------------
    function [32:0] fdiff;                       // |a-b| on FCW words
        input [31:0] a,b;
        begin
            fdiff = (a>=b) ? ({1'b0,a}-{1'b0,b}) : ({1'b0,b}-{1'b0,a});
        end
    endfunction
    function [31:0] fdhz;                        // |a-b| on Hz values (< 2^31)
        input [31:0] a,b;
        begin
            fdhz = (a>=b) ? (a-b) : (b-a);
        end
    endfunction
    function [23:0] sat24;                       // mag>>3, saturating at 2^27
        input [32:0] m;
        begin
            sat24 = (|m[32:27]) ? 24'hffffff : m[26:3];
        end
    endfunction

    wire fft_observed,fft_result_valid,fft_saturated;
    wire [511:0] fft_packet;
    wire [127:0] fft_interp;
    wire [127:0] conversion_frequency;
    wire [3:0] analysis_rate;
    wire [31:0] analysis_age;
    wire [1:0] analysis_mask;
    reg [3:0] tracked_rate,suppress_rate;
    wire [31:0] step_window=(MIN_STEP_FCW>>cand_rate)<66?32'd66:(MIN_STEP_FCW>>cand_rate);
    // Only frames belonging to the tracked rate may provide absence evidence.
    assign fft_result_valid=fft_observed && ((|analysis_mask) || analysis_rate==tracked_rate);
    generate if(LOW_FFT_ENABLE!=0)begin: multirate
      multirate_fft_frontend #(.CLK_FREQ(CLK_FREQ),.LOG2_N(LOG2_N),
        .AMP_THRESHOLD(AMP_THRESHOLD)) u_front(
        .clk(clk),.rst_n(rst_n),.enable(enable),.adc_valid(adc_valid),
        .adc_ch0(adc_ch0),.adc_ch1(adc_ch1),.result_valid(fft_observed),
        .saturated(fft_saturated),.result_packet(fft_packet),.interp_freq(fft_interp),.conversion_frequency(conversion_frequency),
        .spectrum_valid(spectrum_valid),.spectrum_fs_hz(spectrum_fs_hz),.spectrum_bin(spectrum_bin),.spectrum_power(spectrum_power),
        .qualified(analysis_mask),.analysis_log2(analysis_rate),.age_ticks(analysis_age),.analysis_band(),
        .harmonic_scan_valid(harmonic_scan_valid),.harmonic_scan_saturated(harmonic_scan_saturated),
        .harmonic_scan_channel(harmonic_scan_channel));
    end else begin: legacy
    // First harmonic release is qualified only with the multirate frontend.
    assign harmonic_scan_valid=0;assign harmonic_scan_saturated=0;assign harmonic_scan_channel=0;
    // ---- Hann window -> shared 4096-point FFT -----------------------------
    // FRAME_GATED=1 ties the window to the FFT frame: the engine's busy output
    // enables the window for exactly the 4096 samples of one capture, so w[0]
    // always lands on the first sample of the frame (same wiring as the
    // AI_V32 daq_processing_core, where frame_enable comes from the XFFT IP).
    wire fft_busy;
    wire spec_valid;wire [1:0] spec_ch;
    assign spectrum_valid=spec_valid && spec_ch==0 && centered_frame_valid;
    assign spectrum_fs_hz=CLK_FREQ;
    wire fft_raw_valid, fft_raw_saturated, input_accepted;
    wire centered_frame_valid, centered_frame_clipped;
    wire signed [11:0] centered0, centered1;
    // Search-only previous-frame mean subtraction. RAW and DDC remain untouched.
    // Do not publish the first, not-yet-centered calibration frame.
    fft_dc_center #(.LOG2_N(LOG2_N)) u_center (
        .clk(clk),.rst_n(rst_n),.enable(enable),.frame_enable(fft_busy),
        .accept(input_accepted),.in0(adc_ch0),.in1(adc_ch1),
        .out0(centered0),.out1(centered1),
        .frame_valid(centered_frame_valid),.frame_clipped(centered_frame_clipped));
    assign fft_observed=fft_raw_valid && centered_frame_valid;
    assign fft_saturated=fft_raw_saturated || centered_frame_clipped;
    wire hann_valid;
    wire signed [15:0] hann_ch0, hann_ch1;
    hann_window_2ch #(
        .LOG2_N(LOG2_N),
        .COEFF_FILE("hann_4096_half_q15.mem"),
        .HALF_WINDOW_OPT(1),
        .FRAME_GATED(1)
    ) u_hann(
        .clk(clk), .rst_n(rst_n),
        .in_valid(adc_valid && enable),
        .frame_enable(fft_busy),
        .ch0_in(centered0), .ch1_in(centered1),
        .input_accepted(input_accepted),
        .out_valid(hann_valid),
        .ch0_out(hann_ch0), .ch1_out(hann_ch1));
    fft_frame_4ch #(
        .CHANNELS(2), .LOG2_N(LOG2_N), .ADC_FS_HZ(CLK_FREQ),
        .DECIM_LOG2(0), .FRAME_PERIOD_CLKS(0), .ROM_FILE("twiddle_4096.mem")
    ) u_fft(
        .clk(clk), .rst_n(rst_n),
        .sample_valid(hann_valid),
        .samples({24'd0, hann_ch1[11:0], hann_ch0[11:0]}),
        .busy(fft_busy), .result_valid(fft_raw_valid),
        .result_packet(fft_packet), .saturated(fft_raw_saturated),
        .spectrum_valid(spec_valid), .spectrum_channel(spec_ch), .spectrum_bin(spectrum_bin), .spectrum_power(spectrum_power),
        .peak_re_packed(), .peak_im_packed(), .interp_freq_packed(fft_interp));

    assign analysis_rate=0;assign analysis_age=0;assign conversion_frequency=fft_interp;
    assign analysis_mask=fft_saturated?0:{(mag1_33>={1'b0,AMP_THRESHOLD}),(mag0_33>={1'b0,AMP_THRESHOLD})};
    end endgenerate

    assign harmonic_scan_rate=analysis_rate;assign harmonic_scan_age=analysis_age;
    // ---- result packet unpack (layout: fft_frame_4ch header) --------------
    wire [31:0] pkt_bin0 = fft_packet[63:32];
    wire [31:0] pkt_bin1 = fft_packet[191:160];
    wire [32:0] mag0_33  = {fft_packet[96],  fft_packet[95:64]};
    wire [32:0] mag1_33  = {fft_packet[224], fft_packet[223:192]};
    wire [31:0] interp0  = fft_interp[31:0];   // CH0 interpolated peak, Hz
    wire [31:0] interp1  = fft_interp[63:32];  // CH1 interpolated peak, Hz
    // Strongest channel wins (tie -> CH0); one channel may be unconnected.
    wire        sel_ch1  = analysis_mask[1] && (!analysis_mask[0] || mag1_33 > mag0_33);
    wire [32:0] sel_mag  = sel_ch1 ? mag1_33 : mag0_33;
    wire [31:0] sel_freq = sel_ch1 ? interp1 : interp0;
    wire        cand_ok  = (|analysis_mask) && (sel_mag >= {1'b0,AMP_THRESHOLD}) &&
                           ({32'd0,sel_freq} >= F_MIN_HZ) &&
                           ({32'd0,sel_freq} <= F_MAX_HZ);

    // ---- Hz -> FCW divider (one shared instance, 64-cycle latency) --------
    // FCW = freq * 2^32 / CLK_FREQ.  A frame is ~356k cycles, so the 64-cycle
    // division always finishes long before the next result_valid.
    reg        div_start;
    wire       div_busy, div_valid;
    wire [63:0] div_q;
    fixed_divider_generic #(.NUM_W(64), .DEN_W(32)) u_fcw_div(
        .clk(clk), .rst_n(rst_n), .start(div_start),
        .numerator({cand_conversion_hz,32'd0}), .denominator(CLK_FREQ),
        .busy(div_busy), .valid(div_valid), .quotient(div_q));
    wire [63:0] scaled_quotient=div_q>>cand_rate;
    wire [31:0] conv_fcw = (scaled_quotient > {32'd0,F_MAX_FCW}) ? F_MAX_FCW : scaled_quotient[31:0];

    // ---- search / lock bookkeeping ----------------------------------------
    reg [31:0] cand_freq;         // selected peak frequency, Hz (latched)
    reg [31:0] cand_conversion_hz;
    reg [3:0] cand_rate;
    reg [31:0] cand_fcw;          // converted + clamped FCW
    reg [31:0] track_hz;          // Hz of the frequency we parked on
    reg [31:0] last_commit_fcw;
    reg        committed;
    reg [31:0] suppress_fcw;      // frequency that failed to settle
    reg [15:0] suppress_cnt;      // frames left to ignore it for
    reg [31:0] settle_timer;
    reg [31:0] lost_timer;
    reg [7:0]  mag_lost_frames;

    // Loss evaluation, only meaningful in S_TRACK.  With AFC on, a carrier
    // step shows up as !afc_ready first and the drift term only makes the
    // reaction faster (one frame instead of the 200 ms hold).  With AFC off
    // (afc_ready tied 1) the drift term is the only mover.
    // The fast re-scan floor (FAST_REACQ_HZ) is set to 2 FFT bins (~31.7 kHz),
    // ABOVE the 4096-point bin spacing (~15.9 kHz).  The 3-point interpolator of
    // a weak tone near the noise floor can jitter by up to ~1 bin frame-to-frame;
    // a 4 kHz floor (the old REACQ_HZ) let that jitter trip a spurious re-scan
    // during a legitimate sub-bin carrier step (e.g. +50 Hz) while AFC is briefly
    // not-ready.  Real carrier loss is still caught by the sustained mag/hold
    // terms above; the fast term now only fires on a genuinely large displacement.
    wire track_loss =
           (mag_lost_frames >= LOST_FRAMES)
        || (!afc_ready && (lost_timer >= LOST_HOLD_CLKS))
        || (fft_result_valid && (sel_mag >= {1'b0,AMP_THRESHOLD})
            && (|analysis_mask)
            && (fdhz(sel_freq,track_hz) > ((afc_en?FAST_REACQ_HZ[31:0]:REACQ_HZ[31:0])>>tracked_rate))
            && (!afc_ready || !afc_en));

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=S_IDLE; find_fcw<=F_MIN_FCW; commit<=1'b0;
            locked<=1'b0; busy<=1'b1;
            fft_freq0<=32'd0; fft_freq1<=32'd0;
            fft_bin0<=12'd0;  fft_bin1<=12'd0;
            fft_mag0<=24'd0;  fft_mag1<=24'd0;
            frame_count<=32'd0; lost_count<=16'd0;
            cand_freq<=32'd0; cand_fcw<=F_MIN_FCW; track_hz<=32'd0;cand_conversion_hz<=0;cand_rate<=0;
            last_commit_fcw<=F_MIN_FCW; committed<=1'b0;
            suppress_fcw<=32'd0; suppress_cnt<=16'd0;
            settle_timer<=32'd0; lost_timer<=32'd0; mag_lost_frames<=8'd0;
            div_start<=1'b0;
            tracked_rate<=0;suppress_rate<=0;
            fft_valid_channels<=0;fft_update<=0;fft_rate0<=0;fft_rate1<=0;fft_age0<=32'hffffffff;fft_age1<=32'hffffffff;
        end else if(!enable) begin
            // Disabled: park at F_MIN_FCW (config_ok stays true) and forget
            // the search so a re-enable starts from a clean slate.
            state<=S_IDLE; locked<=1'b0; busy<=1'b1;
            committed<=1'b0; suppress_cnt<=16'd0;
            commit<=1'b0; div_start<=1'b0;
            lost_timer<=32'd0; mag_lost_frames<=8'd0;
            find_fcw<=F_MIN_FCW;
            tracked_rate<=0;suppress_rate<=0;fft_valid_channels<=0;fft_update<=0;fft_age0<=32'hffffffff;fft_age1<=32'hffffffff;
        end else begin
            commit<=1'b0; div_start<=1'b0;
            // Frequency validity is independent per channel, not a precision
            // lock-in permission. Sparse telemetry retains only qualified rates
            // and expires stale estimates; an empty low band cannot overwrite RF.
            fft_update<=0;
            if(fft_age0!=32'hffffffff)fft_age0<=fft_age0+1'b1;
            if(fft_age1!=32'hffffffff)fft_age1<=fft_age1+1'b1;
            if(fft_age0>=3*CLK_FREQ)begin fft_valid_channels[0]<=0;fft_freq0<=0;fft_mag0<=0;end
            if(fft_age1>=3*CLK_FREQ)begin fft_valid_channels[1]<=0;fft_freq1<=0;fft_mag1<=0;end
            if(fft_observed)begin
                frame_count<=frame_count+1'b1;
                if(analysis_mask[0])begin
                    fft_freq0<=interp0;fft_bin0<=pkt_bin0[11:0];fft_mag0<=sat24(mag0_33);
                    fft_rate0<=analysis_rate;fft_age0<=analysis_age;fft_update[0]<=1;fft_valid_channels[0]<=1;
                end
                if(analysis_mask[1])begin
                    fft_freq1<=interp1;fft_bin1<=pkt_bin1[11:0];fft_mag1<=sat24(mag1_33);
                    fft_rate1<=analysis_rate;fft_age1<=analysis_age;fft_update[1]<=1;fft_valid_channels[1]<=1;
                end
            end
            if(!adc_valid)begin
                fft_valid_channels<=0;fft_freq0<=0;fft_freq1<=0;
                fft_mag0<=0;fft_mag1<=0;
            end
            if(rescan && state!=S_IDLE) begin
                // Abandon the current lock/pass and start over.
                state<=S_SEARCH; busy<=1'b1; locked<=1'b0;
                committed<=1'b0; suppress_cnt<=16'd0;
                lost_timer<=32'd0; mag_lost_frames<=8'd0;
            end else case(state)
            // --------------------------------------------------------------
            S_IDLE: begin
                busy<=1'b1; locked<=1'b0; committed<=1'b0;
                suppress_cnt<=16'd0;
                state<=S_SEARCH;
            end
            // --------------------------------------------------------------
            S_SEARCH: if(fft_result_valid) begin
                if(suppress_cnt!=16'd0) suppress_cnt<=suppress_cnt-16'd1;
                if(!fft_saturated && cand_ok) begin
                    cand_freq<=sel_freq;
                    cand_conversion_hz<=sel_ch1?conversion_frequency[63:32]:conversion_frequency[31:0];
                    cand_rate<=analysis_rate;
                    div_start<=1'b1;          // Hz -> FCW, 64 cycles
                    state<=S_CONV;
                end
                // else: keep waiting for the next frame
            end
            // --------------------------------------------------------------
            S_CONV: if(div_valid) begin
                cand_fcw<=conv_fcw;
                if(afc_ready && (fdiff(conv_fcw,li_active_fcw)<={1'b0,REACQ_FCW})) begin
                    // AFC is already tracking this tone: adopt it and do NOT
                    // commit - the running lock must not be disturbed.
                    find_fcw<=li_active_fcw;
                    track_hz<=cand_freq;
                    locked<=1'b1; busy<=1'b0;
                    lost_timer<=32'd0; mag_lost_frames<=8'd0;
                    tracked_rate<=cand_rate;state<=S_TRACK;
                end else if((suppress_cnt!=16'd0) && cand_rate==suppress_rate &&
                            (fdiff(conv_fcw,suppress_fcw)<={1'b0,step_window})) begin
                    state<=S_SEARCH;          // this frequency already failed
                end else if(!committed || !afc_ready ||
                            (fdiff(conv_fcw,last_commit_fcw)>{1'b0,step_window})) begin
                    find_fcw<=conv_fcw; commit<=1'b1;
                    track_hz<=cand_freq;
                    committed<=1'b1; last_commit_fcw<=conv_fcw;tracked_rate<=cand_rate;
                    settle_timer<=32'd0;
                    state<=S_SETTLE;
                end else begin
                    state<=S_SEARCH;          // nothing new to commit
                end
            end
            // --------------------------------------------------------------
            S_SETTLE: begin
                if(afc_ready) begin
                    locked<=1'b1; busy<=1'b0;
                    lost_timer<=32'd0; mag_lost_frames<=8'd0;
                    state<=S_TRACK;
                end else if(settle_timer>=SETTLE_CLKS) begin
                    // AFC never captured: keep this frequency out of the way
                    // for a few frames so the FFT can pick something else.
                    suppress_fcw<=last_commit_fcw;suppress_rate<=tracked_rate;
                    suppress_cnt<=SUPPRESS_FRAMES;
                    settle_timer<=32'd0;
                    state<=S_SEARCH;
                end else settle_timer<=settle_timer+32'd1;
            end
            // --------------------------------------------------------------
            S_TRACK: begin
                if(fft_result_valid) begin
                    if(suppress_cnt!=16'd0) suppress_cnt<=suppress_cnt-16'd1;
                    if(sel_mag<{1'b0,AMP_THRESHOLD}) begin
                        if(mag_lost_frames!=8'hff) mag_lost_frames<=mag_lost_frames+8'd1;
                    end else mag_lost_frames<=8'd0;
                end
                if(afc_ready) lost_timer<=32'd0;
                else if(lost_timer<LOST_HOLD_CLKS) lost_timer<=lost_timer+32'd1;
                if(track_loss) begin
                    locked<=1'b0; busy<=1'b1;
                    committed<=1'b0;          // re-acquire must not be blocked
                    if(lost_count!=16'hffff) lost_count<=lost_count+16'd1;
                    lost_timer<=32'd0; mag_lost_frames<=8'd0;
                    state<=S_LOST;
                end
            end
            // --------------------------------------------------------------
            S_LOST: state<=S_SEARCH;           // one-cycle marker
            default: state<=S_IDLE;
            endcase
        end
    end

    // synthesis translate_off
    initial begin
        if(CLK_FREQ<32'd1 || LOG2_N<2 || LOG2_N>13 || F_MIN_FCW>F_MAX_FCW)
            $fatal(1,"fft_find_ctrl: invalid parameters");
    end
    // synthesis translate_on
endmodule
`default_nettype wire
