`timescale 1ns/1ps
`default_nettype none
// =============================================================================
// afc_observer — short-window observer feeding afc_controller.
//
// Purpose: turn the mixed baseband I/Q stream into ONE phasor per observation
// window, stamped with the REAL time at which that window ended, plus a cheap
// amplitude proxy. It deliberately does NOT estimate frequency: the frequency
// estimate belongs to afc_controller, which combines the phase whose provenance
// is documented here.
//
// Both channels are accumulated inside the SAME window and share one timestamp,
// which is what makes the dual-channel comparison in afc_controller meaningful.
//
// Window response: accumulating N samples boxcar-shaped gives
//     H(f) = sinc(f * N / FS),   first null at FS/N
// With N=4160 @65 MSPS the first null is at 15.625 kHz and gain at the 7.8125 kHz
// unambiguous edge is sinc(0.5) = 0.637 (-3.9 dB). This is a REAL property of the
// observer and must be accounted for when reading amplitudes near the band edge;
// it does not bias the PHASE (hence the frequency estimate), only the magnitude.
//
// The timestamp is latched at the ACCUMULATION END (not the start). Since every
// consumer takes differences, a constant offset cancels; what must be consistent
// is that consecutive stamps differ by the ACTUAL elapsed cycles.
// =============================================================================
module afc_observer #(
    parameter integer IQ_W        = 18,   // input I/Q width
    parameter integer OBS_LEN     = 4160, // samples accumulated per window
    parameter integer ACC_W       = 40,   // accumulator width (must hold OBS_LEN*max|I|)
    parameter integer OUT_W       = 24,   // width handed to afc_controller
    parameter integer TS_W        = 32
)(
    input  wire                  clk, rst_n,
    input  wire                  iq_valid,          // one input sample pair ready
    input  wire signed [IQ_W-1:0] mix_i0, mix_q0,   // ch0 baseband after mixer
    input  wire signed [IQ_W-1:0] mix_i1, mix_q1,   // ch1 baseband after mixer
    input  wire [TS_W-1:0]        ts_in,            // free-running cycle index

    output reg                   obs_valid,         // one phasor pair ready
    output reg  signed [OUT_W-1:0] obs_i,  obs_q,   // ch0 phasor (tracked channel)
    output reg  signed [OUT_W-1:0] obs2_i, obs2_q,  // ch1 phasor (same window)
    output reg  [TS_W-1:0]        obs_timestamp,    // REAL stamp (window end)
    output reg  [23:0]            obs_amp,          // |I|+|Q| magnitude proxy, ch0
    output reg  [23:0]            obs2_amp
);

    localparam integer CNT_W = (OBS_LEN<=2) ? 2 : $clog2(OBS_LEN);

    reg [CNT_W-1:0] cnt;
    reg signed [ACC_W-1:0] acc_i0, acc_q0, acc_i1, acc_q1;

    function automatic [ACC_W-1:0] sgn_ext;
        input signed [IQ_W-1:0] v;
        begin
            sgn_ext = {{(ACC_W-IQ_W){v[IQ_W-1]}}, v};
        end
    endfunction

    // cheap |I| + |Q| magnitude proxy (no CORDIC here: the controller already has one
    // and it only needs to separate "signal present" from "too weak")
    reg [ACC_W-1:0] a0i, a0q, a1i, a1q;
    always @* begin
        a0i = acc_i0[ACC_W-1] ? -acc_i0 : acc_i0;
        a0q = acc_q0[ACC_W-1] ? -acc_q0 : acc_q0;
        a1i = acc_i1[ACC_W-1] ? -acc_i1 : acc_i1;
        a1q = acc_q1[ACC_W-1] ? -acc_q1 : acc_q1;
    end

    function automatic [23:0] shrink(input [ACC_W-1:0] v);
        begin
            shrink = (v > 24'hffffff) ? 24'hffffff : v[23:0];
        end
    endfunction

    always @(posedge clk) begin
        if(!rst_n) begin
            cnt<=0; acc_i0<=0; acc_q0<=0; acc_i1<=0; acc_q1<=0;
            obs_valid<=0; obs_i<=0; obs_q<=0; obs2_i<=0; obs2_q<=0;
            obs_timestamp<=0; obs_amp<=0; obs2_amp<=0;
        end else begin
            obs_valid<=0;
            if(iq_valid) begin
                acc_i0 <= acc_i0 + sgn_ext(mix_i0);
                acc_q0 <= acc_q0 + sgn_ext(mix_q0);
                acc_i1 <= acc_i1 + sgn_ext(mix_i1);
                acc_q1 <= acc_q1 + sgn_ext(mix_q1);
                if(cnt == OBS_LEN[CNT_W-1:0]-1) begin
                    cnt <= 0;
                    obs_i  <= acc_i0[OUT_W-1:0];
                    obs_q  <= acc_q0[OUT_W-1:0];
                    obs2_i <= acc_i1[OUT_W-1:0];
                    obs2_q <= acc_q1[OUT_W-1:0];
                    obs_amp  <= shrink(a0i + a0q);
                    obs2_amp <= shrink(a1i + a1q);
                    obs_timestamp <= ts_in;      // REAL stamp, taken at window end
                    obs_valid <= 1;
                    acc_i0<=0; acc_q0<=0; acc_i1<=0; acc_q1<=0;
                end else begin
                    cnt <= cnt + 1'b1;
                end
            end
        end
    end
endmodule
`default_nettype wire
