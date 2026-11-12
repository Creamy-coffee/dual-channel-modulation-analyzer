`timescale 1ns/1ps
`default_nettype none
// ---------------------------------------------------------------------------
// Two-channel frequency consistency guard.
// ---------------------------------------------------------------------------
// Both channels are measured against the SAME NCO, so a per-window phase step
// is proportional to that channel's own offset from the reference:
//
//      step_ch = 360 deg * (f_ch - f_nco) * T_win      (mod 360)
//
// If the two inputs come from one source, step0 == step1 to within noise and
// the phase DIFFERENCE is meaningful.  If they are two different generators,
// the steps differ by 360*|f0-f1|*T_win and any "stable" phase difference the
// display shows is an artefact of the NCO drifting between them.
//
// This module flags that condition so the host can refuse to publish a phase
// difference, instead of printing a number that looks precise and is not.
//
// Threshold is in 16-bit turn LSB per window: 96 LSB = 0.53 deg, i.e. 1.5 Hz
// at the 1 ms window and 0.023 Hz at the 64 ms window.  Two independent
// crystals differ by ~100 ppm (100 Hz at 1 MHz), which is ~36000 LSB: the flag
// is unambiguous.  A leaky up/down score keeps phase noise on a weak signal
// from firing it: a random excursion is cancelled by the next good window.
// ---------------------------------------------------------------------------
module ch_freq_guard #(
    parameter [15:0] STEP_LIMIT = 16'd96,
    parameter [7:0]  CONFIRM    = 8'd24,   // net windows before the flag is set
    parameter integer ENABLE    = 1
)(
    input  wire        clk, rst_n, enable,
    input  wire        meas_valid,
    // Signed circular per-window steps; separate from magnitude telemetry.
    input  wire signed [15:0] step0, step1,
    input  tri1        step_valid,
    input  wire        amp_ok,
    output reg         mismatch,
    output reg  [15:0] delta_avg,   // slow average of |step0-step1|
    output reg  [7:0]  score
);
    wire signed [15:0] signed_diff = step0-step1;
    wire [15:0] diff = signed_diff[15] ? -signed_diff : signed_diff;
    wire        over = (diff > STEP_LIMIT) && amp_ok;
    // 1/16 averaging: a single wild window moves delta by 1/16 only.
    reg [19:0] acc;
    always @(posedge clk) begin
        if (!rst_n) begin
            acc       <= 20'd0;
            delta_avg <= 16'd0;
            score     <= 8'd0;
            mismatch  <= 1'b0;
        end else if (!enable || ENABLE == 0 || !step_valid) begin
            acc <= 20'd0; delta_avg <= 16'd0; score <= 8'd0; mismatch <= 1'b0;
        end else if (meas_valid) begin
            acc       <= acc + {4'b0,diff} - {4'b0,acc[19:4]};
            delta_avg <= acc[19:4];
            if (over) begin
                if (score != 8'hff) score <= score + 8'd1;
            end else if (score != 8'd0) begin
                score <= score - 8'd1;
            end
            if (score >= CONFIRM)      mismatch <= 1'b1;
            else if (score == 8'd0)    mismatch <= 1'b0;
        end
    end
endmodule
`default_nettype wire
