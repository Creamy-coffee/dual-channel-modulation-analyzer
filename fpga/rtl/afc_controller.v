`timescale 1ns/1ps
`default_nettype none
// =============================================================================
// afc_controller — Stage 1 local AFC (KNOWN-frequency-near capture & tracking).
//
// Design notes (see reports/stage1_diag_afc_design_20261018.md):
//  * Tracking branch is SEPARATE from the precision-measurement (long window)
//    branch. This module only produces a phase-continuous FCW microtune.
//  * It does NOT reset the NCO phase: lockin_core_2ch applies afc_fcw on
//    afc_commit while keeping its 'phase' accumulator running.
//  * First-order loop with LIMIT and SMOOTHING. Closed loop is OURS — borrowed
//    only the config-interface separation idea from PyRPL and the "track a
//    reference" idea from SingularitySurfer (whose low-speed front-end / coarse
//    shift-error controller are NOT portable to 65 MSPS).
//  * Out of capture range / no reliable signal / stream break => explicit
//    UNLOCK + revoke_measurement (do NOT keep stale "qualified").
//
// INPUT `obs_i/obs_q/obs_valid`: one short-window phasor per obs_valid pulse,
// produced by a separate short-window observer (not built yet in Stage 1; the
// TB models it). The phasor is the baseband residual after mixing; its phase
// rotates at the residual frequency error Δf.
//
// Δf -> FCW correction (signed), 32-bit accumulator @65 MSPS:
//   1 FCW = FS / 2^32 = 0.0151339918 Hz     1 Hz = 2^32 / FS = 66.076420 FCW
// A previous revision of the notes wrongly used "527 FCW = 2 kHz"; 527 FCW is only
// 7.98 Hz. Reference table (reports/stage1_diag_afc_design_20261018.md sec.3.0):
//     1199 Hz -> 79 226 FCW        2000 Hz -> 132 153 FCW
//     7567 Hz -> 500 000 FCW       7812.5 Hz(=1/(2*T_OBS)) -> 516 222 FCW
//     30 MHz  -> 1 982 292 598 FCW
//
//   corr_fcw = (dphi_turn / 65536) * (2^32 / (FS * T_OBS))
//            = (dphi_turn * FCW_PER_TURN_64) >>> 16
//   FCW_PER_TURN_64 = 2^32 / (FS * T_OBS)   (FCW per one phase_turn LSB)
// Example: FS=65e6, T_OBS=64us => FCW_PER_TURN_64 = 1 032 492.
//   Δf=1199 Hz => dphi = 1199*64e-6 cycles = 5027.8 LSB => corr = 79 226 FCW.
//
// SCOPE LIMITATION: this module only lets the lock-in reference follow SMALL
// drift around a known carrier. It does NOT raise the ADC sample rate and it
// does NOT demonstrate 0-30 MHz waveform integrity / distortion. Acceptance of
// this feature is recorded SEPARATELY from RAW / FFT / analogue sampling quality.
// =============================================================================
module afc_controller #(
    parameter integer FS_HZ        = 65000000,
    parameter real    T_OBS_S      = 64e-6,   // interval between successive obs phasors
    parameter integer OBS_W        = 24,      // width of obs_i / obs_q
    parameter integer CAPTURE_RANGE_FCW = 500000, // +/- capture half-width (~7.6 kHz @65MS/s)
    parameter integer SMOOTH_SHIFT = 3,       // integral loop gain: alpha = 1/2^SHIFT per window.
                                              // NOTE: the NCO is only updated every COMMIT_INT
                                              // windows between afc_commits, so the effective
                                              // gain per plant update is COMMIT_INT/2^SHIFT and
                                              // stability needs that to be < 2 (here 4/8 = 0.5).
    parameter integer COMMIT_INT   = 4,       // emit afc_commit every COMMIT_INT windows
    parameter integer MIN_OBS_AMP  = 2048,    // below this the channel is "too weak"
    parameter integer STABLE_WIN   = 32,      // windows of small correction => afc_stable
    parameter integer STABLE_TARG_FCW = 64,   // |residual| below this (FCW) counts as stable
                                              // 64 FCW ~= 0.97 Hz @65MS/s; must exceed one
                                              // dphi LSB (~15.8 FCW) to reject quantization
    parameter integer HOLD_WIN     = 16,      // weak/absent for HOLD_WIN => UNLOCK
    // ---- real-timestamp guards (rule: never assume every window is exactly T_OBS) ----
    parameter real    DT_TOL_LO    = 0.5,     // accept dt down to this x nominal
    parameter real    DT_TOL_HI    = 4.0,     // accept dt up to this x nominal
    parameter integer SETTLE_WIN   = 8,       // windows to mark invalid after each FCW update
    // ---- dual-channel agreement thresholds -------------------------------------
    // If the second channel's residual differs from the tracked one by more than
    // DUAL_DIFF_FCW (default 660 FCW ~= 10 Hz), the two channels are not at one
    // frequency: their phase difference drifts and must not be published as valid.
    parameter integer DUAL_DIFF_FCW = 660,     // ~10 Hz @65MS/s
    parameter integer DUAL_CONFIRM  = 8,       // consecutive windows to assert
    parameter integer DUAL_CLEAR    = 16       // consecutive windows to deassert hysteresis
)(
    input  wire clk, rst_n,
    input  wire adc_valid,                       // stream health (gates UNLOCK on break)
    // ---- short-window observation, TRACKED channel (from separate observer) ----
    input  wire signed [OBS_W-1:0] obs_i, obs_q,
    input  wire                  obs_valid,     // one phasor ready
    input  wire [23:0]           obs_amp,       // signal strength proxy of tracked ch
    input  wire [31:0]           obs_timestamp, // cycle counter stamped at sampling instant
    // ---- same-window observation of the NON-tracked channel -------------------
    // Rule: the shared NCO tracks only the selected channel. If the two channels
    // are NOT at the same frequency, publishing an inter-channel phase difference
    // as "qualified" is invalid, so we raise dual_freq_flag.
    input  wire signed [OBS_W-1:0] obs2_i, obs2_q,
    input  wire                  obs2_valid,
    input  wire [23:0]           obs2_amp,
    // ---- configuration ----
    input  wire [31:0] nominal_fcw,              // known-frequency-near nominal FCW
    // ---- outputs ----
    output reg  [31:0] afc_fcw,                  // microtuned phase increment word
    output reg         afc_commit,               // pulse: phase-continuous FCW update
    output reg  [1:0]  afc_state,                // 0 INIT 1 CAPTURE 2 TRACK 3 UNLOCK
    output reg         afc_locked, afc_stable,   // status
    output reg         revoke_measurement,       // 1 => measurement "qualified" revoked
    output reg         limit_flag,               // correction hit capture clamp
    // ---- transition-period + aggregated validity ------------------------------
    // Keeping the NCO phase continuous is NOT the same as the measurement being
    // valid immediately: the old filter history / integrator contents are still
    // shaped by the previous frequency. afc_settling marks that window.
    output reg         afc_settling,             // 1 during the post-update settling window
    output reg         afc_valid,                // AFC's own contribution to "qualified"
    // Glue rule: afc_valid does NOT replace the existing measurement guard.
    // Releasing a qualified result requires ALL of:
    //   acquisition OK + measurement warm-up complete + signal quality pass
    //   + afc_valid (locked AND stable AND settling done AND no clamp AND not revoked)
    output reg         dual_freq_flag,           // channels are not at one frequency
    output reg         dt_anomaly,               // observation interval out of band
    output reg  [31:0] drop_count,               // count of rejected (gap) observations
    output reg  [31:0] diag_dt,                  // ACTUAL interval used, in cycles
    output reg  [31:0] diag_fcw,                 // ACTUAL FCW in force at this sample
    // ---- continuous diagnosis stream (Linux-friendly) ----
    output reg         diag_valid,
    output reg  signed [OBS_W-1:0] diag_i, diag_q,
    output reg  signed [15:0] diag_phase,
    output reg  [23:0] diag_amp,
    output reg  [31:0] diag_timestamp,
    output reg  [31:0] diag_frame
);

    localparam integer AF_INIT=0, AF_CAPTURE=1, AF_TRACK=2, AF_UNLOCK=3;

    // explicitly-signed thresholds: avoids any signed/unsigned ambiguity in comparisons
    localparam signed [31:0] STABLE_TARG_S  =  STABLE_TARG_FCW;
    localparam signed [31:0] CAPTURE_RANGE_S =  CAPTURE_RANGE_FCW;
    localparam signed [31:0] DUAL_DIFF_S     =  DUAL_DIFF_FCW;

    // ---- REAL observation interval, taken from obs_timestamp -------------------
    // Nominal interval in clock cycles; used ONLY to bound-check the measured dt.
    localparam integer DT_NOM   = FS_HZ * T_OBS_S;              // 4160 @65MHz/64us
    localparam integer DT_LO    = $rtoi(DT_NOM * DT_TOL_LO);    // below this = dropped/extra obs
    localparam integer DT_HI    = $rtoi(DT_NOM * DT_TOL_HI);    // above this = pause / stall

    // FCW per one phase_turn LSB (65536/cycle), given T_OBS_S & FS_HZ.
    localparam real    FCW_PER_CYCLE_R  = (2.0**32) / (FS_HZ * T_OBS_S);
    localparam [63:0]  FCW_PER_TURN_64  = $rtoi(FCW_PER_CYCLE_R);

    // ---- CORDIC phase of each observation phasor (reuse project IP) ----
    wire               in_ready, out_valid, phase_valid;
    wire signed [15:0] phase_turn;
    wire [23:0]        amp_q12;
    reg  signed [39:0] i_sum, q_sum;
    reg                in_valid;
    reg                req_pending;
    reg                waiting;
    reg  signed [15:0] phase_prev;
    reg  signed [15:0] dphi;                    // wrapped phase delta (turns*65536)
    reg                phi_ready;               // STICKY: set on fresh result, cleared by FSM
    reg                first_done;              // suppress the very first window (absolute phase)

    // ==========================================================================
    // REAL-interval scaling: K = floor(2^40 / dt_cycles).
    //   corr_fcw = dphi_turn * 2^32 / (65536 * dt_cycles) = (dphi * K) >>> 24
    // A division is used (not the nominal T_OBS) because the observation interval
    // MUST be measured, not assumed: dropped windows, pauses and observer latency
    // all change dt, and assuming T_OBS would silently scale the estimate wrong.
    // Cost is 41 cycles of a 41-bit subtract-shift — negligible next to a 4160-cycle
    // observation interval.
    // ==========================================================================
    reg [41:0] div_r;
    reg [31:0] div_q;
    reg [5:0]  div_cnt;
    reg        div_busy, div_start, div_seen;
    reg [31:0] k_used;            // latched K actually applied to this window
    reg [31:0] ts_prev;
    reg [31:0] dt_meas;
    reg        dv_trig;

    always @(posedge clk) begin
        if(!rst_n) begin
            div_busy<=0; div_start<=0; div_seen<=0; div_r<=0; div_q<=0; div_cnt<=0;
            k_used<=0; ts_prev<=0; dt_meas<=0; dv_trig<=0;
        end else begin
            div_start<=0; dv_trig<=0;
            if(obs_valid) begin
                // measured interval to the previous accepted observation
                dt_meas  <= obs_timestamp - ts_prev;
                ts_prev  <= obs_timestamp;
                div_r    <= 42'd0;
                div_q    <= 32'd0;
                div_cnt  <= 0;
                div_busy <= (obs_timestamp - ts_prev) >= 256;  // guard: need dt>=256 so K<2^32
                div_seen <= (obs_timestamp - ts_prev) >= 256;
            end else if(div_busy) begin
                // restoring long division of 2^40 (single '1' bit brought in at cnt==0)
                if(div_cnt==41) begin
                    div_busy <= 0;
                    k_used   <= div_q;
                    dv_trig  <= 1;
                end else begin
                    // standard restoring step: t = (R<<1)|bit_in ; R = t-D if t>=D.
                    // NOTE the shift is already inside t -- shifting again after the
                    // subtraction doubles the remainder every iteration (this bug made
                    // the quotient saturate at 0xFFFFFFFF and the loop do nothing).
                    if({div_r[40:0], (div_cnt==6'd0)} >= {10'd0, dt_meas}) begin
                        div_r <= {div_r[40:0], (div_cnt==6'd0)} - {10'd0, dt_meas};
                        div_q <= {div_q[30:0], 1'b1};
                    end else begin
                        div_r <= {div_r[40:0], (div_cnt==6'd0)};
                        div_q <= {div_q[30:0], 1'b0};
                    end
                    div_cnt <= div_cnt + 1'b1;
                end
            end
        end
    end

    // Frequency residual (FCW units) implied by the latest phase slope.
    // This is a RESIDUAL because the NCO already carries corr_fcw_f, so the
    // loop must INTEGRATE it. (An earlier leaky version,
    // corr += (targ-corr)>>>k, settles at HALF the true offset — see notes.)
    // NOTE: the intermediate MUST be declared wider than 33 bits. Verilog sizes a
    // multiply by the context width, so writing
    //     ($signed(dphi) * $signed({1'b0,k_used})) >>> 24
    // directly evaluates the product at only 33 bits and the truncation wraps the
    // SIGN as well as the magnitude (observed: a +79 kFCW residual came out as -119).
    reg  signed [63:0] targ_full;
    always @* targ_full = $signed(dphi) * $signed({1'b0, k_used});
    reg  signed [31:0] targ_fcw;
    always @* targ_fcw = targ_full >>> 24;
    reg  signed [40:0] corr_next;

    // the interval to the previous observation, continuously available so it can be
    // band-checked on the same edge that obs_valid is seen
    wire [31:0] dt_now   = obs_timestamp - ts_prev;
    wire        dt_in_band = (dt_now >= DT_LO) && (dt_now <= DT_HI);

    reg  phi_pending;   // CORDIC result captured, waiting for the interval scaler
    reg  skip_win;      // this window must be re-baselined, not differentiated

    // ==========================================================================
    // NON-TRACKED channel observation (same window, same NCO).
    // Rule: the shared NCO tracks ONE channel only. If this second channel is not
    // at the same frequency, an inter-channel phase comparison is meaningless and
    // must not be published as "qualified". We keep a separate slope estimate and
    // flag divergence; area cost is one more CORDIC (stage-1 diagnostic choice).
    // ==========================================================================
    wire               in_ready2, out_valid2, phase_valid2;
    wire signed [15:0] phase_turn2;
    wire [23:0]        amp_q12_2;
    reg  signed [39:0] i2_sum, q2_sum;
    reg                in_valid2, req2_pending, waiting2;
    reg  signed [15:0] phase2_prev;
    reg  signed [15:0] dphi2;
    reg                phi2_latched;
    reg  signed [63:0] targ2_full;
    always @* targ2_full = $signed(dphi2) * $signed({1'b0, k_used});
    reg  signed [31:0] targ2_fcw;
    always @* targ2_fcw = targ2_full >>> 24;
    reg  [15:0]        dual_ok_count, dual_bad_count;

    ai_iq_polar_cordic #(.INPUT_SHIFT(0)) u_cordic2 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid2), .in_ready(in_ready2),
        .i_sum(i2_sum), .q_sum(q2_sum),
        .frame_id_in(16'd0), .branch_in(1'b1),
        .out_valid(out_valid2), .amp_q12(amp_q12_2), .phase_turn(phase_turn2),
        .phase_valid(phase_valid2), .frame_id_out(), .branch_out()
    );

    ai_iq_polar_cordic #(.INPUT_SHIFT(0)) u_cordic (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid), .in_ready(in_ready),
        .i_sum(i_sum), .q_sum(q_sum),
        .frame_id_in(16'd0), .branch_in(1'b0),
        .out_valid(out_valid), .amp_q12(amp_q12), .phase_turn(phase_turn),
        .phase_valid(phase_valid), .frame_id_out(), .branch_out()
    );

    // No explicit wrap function is needed: for a phase expressed in binary-turns
    // (65536/turn), a two's-complement 16-bit subtraction IS the modulo-2*pi wrap.
    // (An earlier wrap16(a) helper could never trigger, since its own 16-bit input
    // was already wrapped, and only produced harmless truncation warnings.)

    // ---- CORDIC request handshake + result capture ----
    always @(posedge clk) begin
        if(!rst_n) begin
            req_pending<=0; in_valid<=0; i_sum<=0; q_sum<=0;
            waiting<=0; phase_prev<=0; dphi<=0; phi_ready<=0; first_done<=0;
            phi_pending<=0; skip_win<=0; dt_anomaly<=0; drop_count<=0;
            diag_dt<=0; diag_fcw<=0; dual_freq_flag<=0;
            phase2_prev<=0; dphi2<=0; waiting2<=0; req2_pending<=0; in_valid2<=0;
            phi2_latched<=0; dual_ok_count<=0; dual_bad_count<=0;
        end else begin
            if(!adc_valid) phi_ready<=0;  // drop any stale result during a stream break

            // ---- non-tracked channel: same-window slope, independent of the loop ----
            if(!adc_valid) begin phi2_latched<=0; waiting2<=0; end
            if(obs2_valid) begin
                i2_sum <= {{(40-OBS_W-1){obs2_i[OBS_W-1]}}, obs2_i, 1'b0};
                q2_sum <= {{(40-OBS_W-1){obs2_q[OBS_W-1]}}, obs2_q, 1'b0};
                req2_pending <= 1;
            end
            in_valid2 <= req2_pending && in_ready2;
            if(in_valid2) begin req2_pending <= 0; waiting2 <= 1; end
            if(out_valid2 && waiting2 && phase_valid2) begin
                waiting2 <= 0;
                dphi2    <= phase_turn2 - phase2_prev;
                phase2_prev <= phase_turn2;
                phi2_latched <= 1;
            end else if(out_valid2 && waiting2) begin
                waiting2 <= 0;
            end

            // ---- real-interval check on the tracked channel ----
            // A dropped window, a pause, or extra observer latency changes dt, so a
            // phase delta taken across such a gap is meaningless. Detect it, count it,
            // and re-baseline instead of feeding a bogus slope into the integrator.
            if(obs_valid) begin
                if(!dt_in_band) begin
                    dt_anomaly <= 1;
                    if(drop_count!=32'hffffffff) drop_count <= drop_count + 1'b1;
                    skip_win   <= 1;            // this pair must not produce a slope
                end else begin
                    dt_anomaly <= 0;
                end
            end

            if(obs_valid) begin
                i_sum <= {{(40-OBS_W-1){obs_i[OBS_W-1]}}, obs_i, 1'b0};
                q_sum <= {{(40-OBS_W-1){obs_q[OBS_W-1]}}, obs_q, 1'b0};
                req_pending <= 1;
            end
            in_valid <= req_pending && in_ready;
            if(in_valid) begin
                req_pending <= 0;
                waiting <= 1;              // wait for CORDIC to return its result
            end

            // The slope may only be used once BOTH the CORDIC result and the
            // real-interval scaling factor K for this window are available.
            if(phi_pending && !div_busy) begin
                phi_pending <= 0;
                phi_ready   <= 1;
            end

            // phi_ready is STICKY: set on a fresh valid result; the FSM clears it.
            if(out_valid && waiting && phase_valid) begin
                waiting  <= 0;
                dphi    <= phase_turn - phase_prev;   // mod-2*pi wrap is automatic
                phase_prev <= phase_turn;
                phi_pending <= 1;
                diag_dt <= dt_meas;                   // report the ACTUAL interval used
            end else if(out_valid && waiting) begin
                waiting <= 0;             // result without phase_valid: ignore
            end
        end
    end

    // ---- main AFC loop / FSM ----
    reg  signed [31:0] corr_fcw_f;              // integrated correction (FCW units)
    reg  [31:0]        win_count;
    reg  [31:0]        stable_count;
    reg  [31:0]        weak_count;
    reg  [31:0]        good_count;   // strong-stream debounce while UNLOCK
    reg  [31:0]        ticks;
    reg  [1:0]         state;
    reg  [3:0]         commit_div;             // COMMIT_INT up to 16
    reg  [15:0]        settle_cnt;             // countdown for the settling window

    always @(posedge clk) begin
        if(!rst_n) begin
            state<=AF_INIT; afc_fcw<=nominal_fcw; afc_commit<=0;
            afc_locked<=0; afc_stable<=0; revoke_measurement<=1; limit_flag<=0;
            corr_fcw_f<=0; win_count<=0; stable_count<=0; weak_count<=0; good_count<=0;
            ticks<=0; commit_div<=0; afc_state<=AF_INIT; settle_cnt<=0;
            afc_settling<=0; afc_valid<=0;
            diag_valid<=0; diag_i<=0; diag_q<=0; diag_phase<=0;
            diag_amp<=0; diag_timestamp<=0; diag_frame<=0;
        end else begin
            ticks<=ticks+1'b1;
            afc_commit<=0; diag_valid<=0;

            // ---- stream break => immediate UNLOCK, do not keep stale qualified ----
            if(!adc_valid) begin
                state<=AF_UNLOCK; afc_locked<=0; afc_stable<=0;
                revoke_measurement<=1; corr_fcw_f<=0; afc_fcw<=nominal_fcw;
            end else begin
                case(state)
                AF_INIT: begin
                    revoke_measurement<=1; afc_locked<=0; afc_stable<=0;
                    afc_fcw<=nominal_fcw; corr_fcw_f<=0;
                    if(adc_valid) state<=AF_CAPTURE;
                end
                AF_CAPTURE, AF_TRACK: begin
                    // weak / absent tracked channel bookkeeping
                    if(obs_valid) begin
                        if(obs_amp < MIN_OBS_AMP) begin
                            if(weak_count!=32'hffffffff) weak_count<=weak_count+1'b1;
                        end else weak_count<=0;
                    end
                    if(weak_count >= HOLD_WIN) begin
                        state<=AF_UNLOCK; afc_locked<=0; afc_stable<=0;
                        revoke_measurement<=1;
                    end else begin
                        revoke_measurement<=0;
                        if(phi_ready) begin
                            win_count<=win_count+1'b1;
                            // Countdown for the post-update settling window. Placed
                            // BEFORE the commit code below so that a commit issued in
                            // this same cycle correctly (re)loads the counter.
                            if(settle_cnt!=0) settle_cnt<=settle_cnt-1'b1;
                            afc_settling <= (settle_cnt > 1);
                            if(!first_done) begin
                                // First window after (re)acquire: the captured
                                // "delta" is really an absolute phase (phase_prev
                                // was reset), so suppress the slope update.
                                first_done <= 1;
                            end else begin
                                // ---- integral frequency loop (see notes above) ----
                                corr_next = corr_fcw_f + (targ_fcw >>> SMOOTH_SHIFT);
                                if(corr_next >  CAPTURE_RANGE_S) begin
                                    corr_fcw_f <=  CAPTURE_RANGE_FCW;
                                    limit_flag <= 1;   // demand exceeds local capture range
                                end else if(corr_next < -CAPTURE_RANGE_S) begin
                                    corr_fcw_f <= -CAPTURE_RANGE_FCW;
                                    limit_flag <= 1;
                                end else begin
                                    corr_fcw_f <= corr_next[31:0];
                                    limit_flag <= 0;   // in range: self-clearing indicator
                                end
                                // stability bookkeeping: RESIDUAL must be small
                                if((targ_fcw > -STABLE_TARG_S) && (targ_fcw < STABLE_TARG_S)) begin
                                    if(stable_count!=32'hffffffff) stable_count<=stable_count+1'b1;
                                end else stable_count<=0;
                                // commit cadence: emit afc_commit every COMMIT_INT windows
                                if(commit_div >= (COMMIT_INT-1)) begin
                                    afc_fcw<=nominal_fcw + corr_fcw_f;   // already clamped
                                    afc_commit<=1;
                                    commit_div<=0;
                                    // Phase continuity is NOT enough: old filter
                                    // history / integrator content is still shaped by
                                    // the previous frequency. Mark the transition.
                                    settle_cnt   <= SETTLE_WIN;
                                    afc_settling <= 1;
                                end else begin
                                    commit_div<=commit_div+1'b1;
                                end
                            end
                            phi_ready <= 0;   // consume the sticky flag
                        end
                        // The window pair after a gap is re-baselined, not differentiated.
                        if(skip_win) begin
                            skip_win    <= 0;
                            first_done  <= 0;      // next pair starts a fresh reference
                            stable_count<= 0;      // do not claim stability across a gap
                        end
                        // state promotion
                        if(state==AF_CAPTURE && stable_count>=STABLE_WIN) begin
                            state<=AF_TRACK; afc_locked<=1;
                        end else if(state==AF_TRACK) begin
                            afc_locked<=1; afc_stable<=(stable_count>=STABLE_WIN);
                        end
                    end
                end
                AF_UNLOCK: begin
                    afc_locked<=0; afc_stable<=0; revoke_measurement<=1;
                    afc_fcw<=nominal_fcw;       // hold nominal, no stale correction
                    // Recovery: once the stream is healthy and the tracked channel
                    // is strong for HOLD_WIN consecutive windows, re-acquire. This
                    // is required so a transient break / weak spell does NOT lock
                    // the loop out forever (stage-1 must re-lock on return of signal).
                    if(!adc_valid) good_count<=0;
                    else if(obs_valid) begin
                        if(obs_amp >= MIN_OBS_AMP) begin
                            if(good_count!=32'hffffffff) good_count<=good_count+1'b1;
                        end else good_count<=0;
                    end
                    if(good_count >= HOLD_WIN) begin
                        state<=AF_CAPTURE; first_done<=0; stable_count<=0;
                    end
                end
                default: state<=AF_INIT;
                endcase
            end

            // ---- diagnosis stream: emit every processed window ----
            if(phi_ready) begin
                diag_valid<=1; diag_i<=obs_i; diag_q<=obs_q;
                diag_phase<=phase_turn; diag_amp<=obs_amp;
                diag_timestamp<=ts_prev; diag_frame<=win_count;
                diag_fcw<=afc_fcw;
                diag_dt <=dt_meas;            // ACTUAL interval, cycles
            end
        end
        afc_state<=state;
        // AFC's own contribution to "qualified". It does NOT replace the existing
        // measurement guard: acquisition OK, measurement warm-up, and the signal
        // quality check must ALL still pass upstream before anything is published.
        afc_valid <= afc_locked && afc_stable && !limit_flag &&
                     !afc_settling && !revoke_measurement;
    end

    always @(posedge clk) begin
        if(!rst_n) begin
            dual_freq_flag<=0;
        end else begin
            // ======================================================
            // Dual-channel frequency agreement.
            // The shared NCO tracks ONE channel. If the other channel rotates at a
            // different rate, the inter-channel phase difference is meaningless and
            // must NOT be published as "qualified same-frequency phase difference".
            // We compare the two channels' residuals within the same window.
            // ======================================================
            if(phi2_latched) begin
                phi2_latched <= 0;
                if(((targ2_fcw - targ_fcw) >  DUAL_DIFF_S) ||
                   ((targ_fcw - targ2_fcw) >  DUAL_DIFF_S)) begin
                    dual_ok_count  <= 0;
                    if(dual_bad_count!=16'hffff) dual_bad_count<=dual_bad_count+1'b1;
                end else begin
                    dual_bad_count <= 0;
                    if(dual_ok_count!=16'hffff) dual_ok_count<=dual_ok_count+1'b1;
                end
            end
            if(dual_bad_count >= DUAL_CONFIRM)       dual_freq_flag <= 1;
            if(dual_ok_count  >= DUAL_CLEAR)         dual_freq_flag <= 0;
        end
    end
endmodule
`default_nettype wire
