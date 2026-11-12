`timescale 1ns/1ps
`default_nettype none
// ---------------------------------------------------------------------------
// On-chip coarse-to-fine frequency acquisition for the lock-in reference.
//
// WHY THIS EXISTS
// ---------------
// cfg_fcw used to be a compile-time constant (REF_FCW), so every carrier
// frequency needed its own bitstream.  This module drives cfg_fcw at run time
// and searches for the carrier, so ONE bitstream covers the whole band.
//
// ARCHITECTURE (matches the published "FFT coarse estimate + PLL fine lock"
// pattern, e.g. IEEE 9129659 "FPGA-Based Digital Lock-in Amplifier With
// High-Precision Automatic Frequency Tracking"; the coarse estimator here is a
// short-window DDC sweep instead of an FFT because the mixer/DDS already
// exists and an FFT would cost a second datapath):
//
//   1. COARSE  short window (N=256, R=16 -> T_win 63.0 us), step 15 kHz,
//              sweep the whole band, keep the largest |amplitude|.
//   2. FINE    long window (N=4096, R=16 -> T_win 1.008 ms), step 900 Hz,
//              sweep +/-15 kHz around the coarse winner.
//   3. LOCK    park on the fine winner; the residual (< +/-450 Hz) is inside
//              the +/-2 kHz capture range of local_afc_2ch, which then tracks.
//
// STEP SELECTION IS NOT ARBITRARY
// -------------------------------
// The lock-in window is a Hann window, so a carrier detuned by df is
// attenuated by
//      G(x) = sinc(x) + 0.5*sinc(x-1) + 0.5*sinc(x+1),   x = df * T_win
// (G(0)=1, G(1)=0.5 i.e. -6.02 dB, G(2)=0).  With step S the worst-case
// detune is S/2, so choosing S = 1/T_win gives x = 0.5 -> only -1.42 dB.
//   coarse: 1/63.02us  = 15.87 kHz -> step 15 kHz (worst -1.27 dB)
//   fine:   1/1.0083ms =   992 Hz  -> step 900 Hz  (worst -1.17 dB)
// A coarser step would leave holes; a finer one only costs sweep time.
//
// SWEEP COST
// ----------
//   coarse  2160 points x 82 us  = 177 ms
//   fine      33 points x 1.31 ms =  43 ms
//   total ~220 ms, then local_afc_2ch tracks continuously.
//
// WHAT THIS IS NOT
// ----------------
// It picks the STRONGEST single tone.  With several comparable tones it is
// free to choose any of them, so it is not a multi-tone selector.  It reports
// best_amp so the host can see how strong the pick was.
// ---------------------------------------------------------------------------
module freq_scan_ctrl #(
    // fcw = f * 2^32 / 65e6  (1 Hz = 66.0764 fcw)
    parameter [31:0] F_MIN_FCW      = 32'd13215,       //        200 Hz
    parameter [31:0] F_MAX_FCW      = 32'd2140876000,  //     32.40 MHz (Nyquist 32.5)
    parameter [31:0] STEP_C_FCW     = 32'd991146,      //     15 kHz
    parameter [31:0] STEP_F_FCW     = 32'd59469,       //       900 Hz
    parameter [31:0] FINE_SPAN_FCW  = 32'd991146,      //  +/- 15 kHz
    parameter [4:0]  INTEG_C        = 5'd8,            //  256-point windows
    parameter [4:0]  INTEG_F        = 5'd12,           // 4096-point windows
    // Detection floor on the Q12 lock-in amplitude.  Same scale as amp0/amp1.
    parameter [23:0] AMP_THRESHOLD  = 24'd2048,
    // Give up on a point if no result arrives (clock starvation, no ADC data).
    parameter [31:0] TIMEOUT_CLKS   = 32'd200000,      // 3.08 ms
    // Consecutive sub-threshold windows before dropping lock and rescanning.
    parameter [7:0]  LOST_WINDOWS   = 8'd64
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,
    input  wire        meas_valid,          // one result window completed
    input  wire [23:0] meas_amp0,
    input  wire [23:0] meas_amp1,
    input  wire        rescan,              // host/button request: search again
    output reg  [31:0] active_fcw,          // -> cfg_fcw
    output reg  [4:0]  active_integ,        // -> cfg_integration_log2
    output reg         commit,              // re-load cfg into the lock-in core
    output reg         busy,                // sweeping; measurements not usable
    output reg         locked,              // parked on a detected carrier
    output reg  [2:0]  state,               // 0 idle 1 coarse 2 fine 3 lock
    output reg  [31:0] best_fcw,            // winning frequency word
    output reg  [23:0] best_amp,            // winning amplitude (Q12)
    output reg  [15:0] scan_count,          // points evaluated in this pass
    output reg  [7:0]  lost_count,          // sub-threshold windows while locked
    output reg  [15:0] pass_count           // completed sweeps
);
    localparam [2:0] S_IDLE   = 3'd0;
    localparam [2:0] S_COARSE = 3'd1;
    localparam [2:0] S_FINE   = 3'd2;
    localparam [2:0] S_LOCK   = 3'd3;

    // amax is the larger channel: one channel may be unconnected.
    wire [23:0] amax = (meas_amp0 >= meas_amp1) ? meas_amp0 : meas_amp1;

    reg [31:0] cur_fcw;
    reg [31:0] fine_lo, fine_hi;
    reg [31:0] timer;
    reg        armed;      // 1 while waiting for the result of cur_fcw
    reg [31:0] cand_fcw;   // winner of the pass being accumulated
    reg [23:0] cand_amp;

    // ---- frequency-step helpers with saturation --------------------------
    function [31:0] sat_add;
        input [31:0] a, b;
        reg [32:0] s;
        begin
            s = {1'b0, a} + {1'b0, b};
            sat_add = s[32] ? 32'hffffffff : s[31:0];
        end
    endfunction
    function [31:0] sat_sub;
        input [31:0] a, b;
        begin
            sat_sub = (a > b) ? (a - b) : 32'd0;
        end
    endfunction
    function [31:0] clamp;
        input [31:0] v;
        begin
            if (v < F_MIN_FCW)      clamp = F_MIN_FCW;
            else if (v > F_MAX_FCW) clamp = F_MAX_FCW;
            else                    clamp = v;
        end
    endfunction

    integer i;
    always @(posedge clk) begin
        if (!rst_n) begin
            active_fcw  <= F_MIN_FCW;
            active_integ<= INTEG_C;
            commit      <= 1'b0;
            busy        <= 1'b1;
            locked      <= 1'b0;
            state       <= S_IDLE;
            best_fcw    <= F_MIN_FCW;
            best_amp    <= 24'd0;
            scan_count  <= 16'd0;
            lost_count  <= 8'd0;
            pass_count  <= 16'd0;
            cur_fcw     <= F_MIN_FCW;
            fine_lo     <= F_MIN_FCW;
            fine_hi     <= F_MAX_FCW;
            timer       <= 32'd0;
            armed       <= 1'b0;
            cand_fcw    <= F_MIN_FCW;
            cand_amp    <= 24'd0;
        end else begin
            commit <= 1'b0;   // one-clock pulse unless re-asserted below
            if (armed) begin
                timer <= (timer == TIMEOUT_CLKS) ? timer : timer + 32'd1;
            end

            if (!enable) begin
                state       <= S_IDLE;
                busy        <= 1'b1;
                locked      <= 1'b0;
                armed       <= 1'b0;
                active_integ<= INTEG_C;
            end else if (rescan && state != S_IDLE) begin
                // Abandon the current pass and start over from the bottom.
                state       <= S_IDLE;
                busy        <= 1'b1;
                locked      <= 1'b0;
                armed       <= 1'b0;
                lost_count  <= 8'd0;
            end else begin
                case (state)
                // ----------------------------------------------------------
                S_IDLE: begin
                    // Load the first candidate of a new coarse pass.
                    busy        <= 1'b1;
                    locked      <= 1'b0;
                    cur_fcw     <= F_MIN_FCW;
                    active_fcw  <= F_MIN_FCW;
                    active_integ<= INTEG_C;
                    cand_amp    <= 24'd0;
                    cand_fcw    <= F_MIN_FCW;
                    scan_count  <= 16'd0;
                    lost_count  <= 8'd0;
                    commit      <= 1'b1;
                    armed       <= 1'b1;
                    timer       <= 32'd0;
                    state       <= S_COARSE;
                end
                // ----------------------------------------------------------
                S_COARSE: begin
                    if (armed) begin
                        if (meas_valid) begin
                            armed <= 1'b0;
                            if (scan_count != 16'hffff) scan_count <= scan_count + 16'd1;
                            if (amax > cand_amp) begin
                                cand_amp <= amax;
                                cand_fcw <= cur_fcw;
                            end
                        end else if (timer == TIMEOUT_CLKS) begin
                            armed <= 1'b0;   // no result: skip this point
                            if (scan_count != 16'hffff) scan_count <= scan_count + 16'd1;
                        end
                    end else begin
                        if (cur_fcw >= F_MAX_FCW || (cur_fcw + STEP_C_FCW) > F_MAX_FCW) begin
                            // End of the coarse pass.
                            best_amp <= cand_amp;
                            best_fcw <= cand_fcw;
                            pass_count <= pass_count + 16'd1;
                            if (cand_amp >= AMP_THRESHOLD) begin
                                // active_fcw MUST move together with cur_fcw:
                                // the result that arrives next belongs to the
                                // frequency actually committed, and the sweep
                                // scores it against cur_fcw.  Leaving active_fcw
                                // at the last coarse point scored the first
                                // fine candidate against a measurement taken at
                                // the top of the band (found in simulation).
                                fine_lo <= clamp(sat_sub(cand_fcw, FINE_SPAN_FCW));
                                fine_hi <= clamp(sat_add(cand_fcw, FINE_SPAN_FCW));
                                cur_fcw <= clamp(sat_sub(cand_fcw, FINE_SPAN_FCW));
                                active_fcw <= clamp(sat_sub(cand_fcw, FINE_SPAN_FCW));
                                cand_amp<= 24'd0;
                                scan_count <= 16'd0;
                                active_integ <= INTEG_F;
                                commit  <= 1'b1;
                                armed   <= 1'b1;
                                timer   <= 32'd0;
                                state   <= S_FINE;
                            end else begin
                                // Nothing above the floor: sweep again.
                                state <= S_IDLE;
                            end
                        end else begin
                            cur_fcw     <= sat_add(cur_fcw, STEP_C_FCW);
                            active_fcw  <= sat_add(cur_fcw, STEP_C_FCW);
                            commit      <= 1'b1;
                            armed       <= 1'b1;
                            timer       <= 32'd0;
                        end
                    end
                end
                // ----------------------------------------------------------
                S_FINE: begin
                    if (armed) begin
                        if (meas_valid) begin
                            armed <= 1'b0;
                            if (scan_count != 16'hffff) scan_count <= scan_count + 16'd1;
                            if (amax > cand_amp) begin
                                cand_amp <= amax;
                                cand_fcw <= cur_fcw;
                            end
                        end else if (timer == TIMEOUT_CLKS) begin
                            armed <= 1'b0;
                            if (scan_count != 16'hffff) scan_count <= scan_count + 16'd1;
                        end
                    end else begin
                        if (cur_fcw >= fine_hi || (cur_fcw + STEP_F_FCW) > fine_hi) begin
                            // Park on the fine winner and hand over to AFC.
                            best_amp <= cand_amp;
                            best_fcw <= cand_fcw;
                            if (cand_amp >= AMP_THRESHOLD) begin
                                active_fcw   <= cand_fcw;
                                active_integ <= INTEG_F;
                                commit  <= 1'b1;
                                busy    <= 1'b0;
                                locked  <= 1'b1;
                                state   <= S_LOCK;
                                lost_count <= 8'd0;
                            end else begin
                                state <= S_IDLE;     // signal vanished: rescan
                            end
                        end else begin
                            cur_fcw     <= sat_add(cur_fcw, STEP_F_FCW);
                            active_fcw  <= sat_add(cur_fcw, STEP_F_FCW);
                            commit      <= 1'b1;
                            armed       <= 1'b1;
                            timer       <= 32'd0;
                        end
                    end
                end
                // ----------------------------------------------------------
                S_LOCK: begin
                    // Stay parked; auto_rate_ctrl owns the decimation gear and
                    // local_afc_2ch owns the fine frequency.  Drop lock only
                    // after a long run of sub-threshold windows.
                    if (meas_valid) begin
                        if (amax >= AMP_THRESHOLD) begin
                            if (lost_count != 0) lost_count <= lost_count - 8'd1;
                        end else if (lost_count != 8'hff) begin
                            lost_count <= lost_count + 8'd1;
                        end
                    end
                    if (lost_count >= LOST_WINDOWS) begin
                        state  <= S_IDLE;
                        busy   <= 1'b1;
                        locked <= 1'b0;
                    end
                end
                default: state <= S_IDLE;
                endcase
            end
        end
    end
endmodule
`default_nettype wire
