`timescale 1ns/1ps
`default_nettype none
// ---------------------------------------------------------------------------
// Automatic decimation-gear controller for the lock-in measurement chain.
//
// WHAT THIS DOES AND DOES NOT DO
// ------------------------------
//   DOES:   pick the CIC decimation ratio from the reference frequency, so a
//           low-frequency carrier gets a long integration window (narrow ENBW)
//           while a high-frequency carrier keeps the fast gear.
//   DOES NOT: touch the ADC clock.  fs_in stays 65 MHz in every gear.  The RAW
//           snapshot path is a separate full-rate 65 MS/s datapath and is
//           completely unaffected, so no high-frequency capability is given up.
//
// WHY THE GEAR DEPENDS ON THE REFERENCE FREQUENCY
// -----------------------------------------------
//   The CIC runs AFTER the quadrature mixer, so the wanted tone always sits at
//   DC.  The decimation bandwidth therefore does NOT have to cover the carrier
//   -- that is why 30 MHz works at R=16 even though fs_eff is only 4.0625 MHz.
//   What R actually buys is WINDOW LENGTH:
//
//       T_win = N * R / fs_in      ENBW ~ 1.5 / T_win   (Hann)
//
//   R=16 -> 1.008 ms (ENBW 1488 Hz);  R=1024 -> 64.5 ms (ENBW 23 Hz).
//   Measured against a brick-wall decimator the 3rd-order CIC leaks a constant
//   -2.60 dB of aliased noise regardless of R (scripts/rate_plan.py), so the
//   net noise-power gain is 6.03 / 12.05 / 18.07 dB for R=64/256/1024.
//
//   Raising N instead is NOT equivalent here: lockin_core_2ch clamps
//   integration_log2 to 3..16 (N <= 65536), so at R=16 the longest reachable
//   window is 16.1 ms.  The 64.5 ms window exists only at R >= 1024.
//
// GEAR TABLE  (decim_log2 = 4 + 2*decim_sel, see lockin_core_2ch.v)
// ---------------------------------------------------------------------------
//   gear  decim_sel  R      fs_eff        T_win     ENBW      f_ref range
//    0       0        16    4.0625 MHz    1.008 ms  1487.7 Hz  >= 100 kHz
//    1       1        64    1.0156 MHz    4.033 ms   371.9 Hz  10k..100k
//    2       2       256    253.9 kHz    16.13 ms     93.0 Hz  1k..10k
//    3       3      1024     63.5 kHz    64.53 ms     23.2 Hz  < 1 kHz
//
//   NOTE ON HEADROOM: at R=1024 the 3rd-order CIC integrator needs ~53.4 bit
//   of the available 58 (scripts/rate_plan.py).  Do NOT add a 5th gear
//   (R=4096) without widening the accumulator -- it will silently saturate.
//
// TRANSITION
// ----------
//   A gear change re-commits the lock-in configuration, which resets the NCO
//   phase and flushes the CIC.  Measurements published during that time are
//   meaningless, so 'settling' is raised and 'valid' is dropped for three
//   window periods.  This is the same "phase continuous != measurement valid"
//   rule already used by afc_controller; auto_rate_ctrl does not replace the
//   lock-in guard, it only adds a gear-change invalidation on top of it.
// ---------------------------------------------------------------------------
module auto_rate_ctrl #(
    // Gear thresholds as FCW (fcw = f * 2^32 / 65e6; 1 Hz = 66.0764 FCW).
    // Computed exactly: 100 kHz -> 6607642, 10 kHz -> 660764, 1 kHz -> 66076.
    parameter [31:0] G1_MIN_FCW        = 32'd6607642,   // gear 0 floor: 100 kHz
    parameter [31:0] G2_MIN_FCW        = 32'd660764,    // gear 1 floor:  10 kHz
    parameter [31:0] G3_MIN_FCW        = 32'd66076,     // gear 2 floor:   1 kHz
    parameter [4:0]  INTEGRATION_LOG2  = 5'd12,         // 4096 points/window
    parameter [3:0]  HYST_SHIFT        = 4'd6,          // hysteresis = 1/64
    parameter [4:0]  SETTLE_WINDOWS    = 5'd3,          // windows to discard
    parameter [1:0]  GEAR_BOOT         = 2'd0           // boot gear (fastest)
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,          // lock-in enabled
    input  wire [31:0] ref_fcw,         // reference frequency word
    input  wire        ref_valid,       // reference word is meaningful
    output reg  [1:0]  decim_sel,       // -> cfg_decim_sel
    output reg  [4:0]  integration_log2,// -> cfg_integration_log2
    output reg         commit,          // re-commit request (1 clock pulse)
    output reg         settling,        // gear/params unstable; results invalid
    output reg         valid,           // gear settled and reference valid
    output reg  [1:0]  gear,            // current gear (diagnostic)
    output reg  [15:0] change_count,    // number of gear changes (diagnostic)
    output reg  [31:0] settle_ticks     // cycles left in the settle period
);

    // Hysteresis band: 1/64 of the threshold (~1.6 %).  Without it a reference
    // sitting exactly on a boundary would toggle the gear forever, and every
    // toggle costs a full re-settle.
    wire [31:0] h1 = G1_MIN_FCW >> HYST_SHIFT;
    wire [31:0] h2 = G2_MIN_FCW >> HYST_SHIFT;
    wire [31:0] h3 = G3_MIN_FCW >> HYST_SHIFT;

    // ---- next-gear decision (combinational, with hysteresis) ---------------
    reg [1:0] next_gear;
    always @* begin
        next_gear = gear;
        if (ref_valid) begin
            case (gear)
                2'd0: begin
                    // fastest gear: only descend when clearly below 100 kHz
                    if (ref_fcw < G1_MIN_FCW - h1) next_gear = 2'd1;
                end
                2'd1: begin
                    if (ref_fcw > G1_MIN_FCW + h1)      next_gear = 2'd0;
                    else if (ref_fcw < G2_MIN_FCW - h2) next_gear = 2'd2;
                end
                2'd2: begin
                    if (ref_fcw > G2_MIN_FCW + h2)      next_gear = 2'd1;
                    else if (ref_fcw < G3_MIN_FCW - h3) next_gear = 2'd3;
                end
                2'd3: begin
                    // slowest gear: only climb when clearly above 1 kHz
                    if (ref_fcw > G3_MIN_FCW + h3) next_gear = 2'd2;
                end
                default: next_gear = 2'd0;
            endcase
        end
    end

    // ---- settle period ------------------------------------------------------
    // Window length in clocks = N * R = 2^(integration_log2 + decim_log2).
    // decim_log2 = 4 + 2*decim_sel, so the exponent never exceeds 12+10 = 22.
    wire [5:0] decim_log2      = 6'd4 + {3'd0, decim_sel, 1'b0};
    wire [5:0] window_exp      = {1'b0, integration_log2} + decim_log2;
    wire [31:0] window_ticks   = 32'd1 << window_exp;
    // Guard against a pathological SETTLE_WINDOWS * window_ticks overflow.
    wire [31:0] settle_target  = window_ticks * {27'd0, SETTLE_WINDOWS};

    reg [31:0] settle_count;
    reg        boot_done;

    always @(posedge clk) begin
        if (!rst_n) begin
            gear            <= GEAR_BOOT;
            decim_sel       <= GEAR_BOOT;
            integration_log2<= INTEGRATION_LOG2;
            commit          <= 1'b0;
            settling        <= 1'b1;      // boot always needs one settle period
            valid           <= 1'b0;
            change_count    <= 16'd0;
            settle_count    <= 32'd0;
            settle_ticks    <= 32'd0;
            boot_done       <= 1'b0;
        end else begin
            commit <= 1'b0;

            // Boot commit: the lock-in only latches configuration on cfg_commit,
            // and daq_top previously produced that pulse from "first clock after
            // reset".  auto_rate_ctrl now owns the commit, so it must reproduce
            // it once, otherwise the measurement chain never starts.
            if (!boot_done) begin
                boot_done <= 1'b1;
                commit    <= 1'b1;
            end

            if (settling) begin
                if (settle_count >= settle_target) begin
                    settling     <= 1'b0;
                    settle_count <= 32'd0;
                    settle_ticks <= 32'd0;
                end else begin
                    settle_count <= settle_count + 32'd1;
                    settle_ticks <= settle_target - settle_count;
                end
            end else if (enable && ref_valid && next_gear != gear) begin
                // Gear change: publish the new parameters and re-commit.
                gear            <= next_gear;
                decim_sel       <= next_gear;
                integration_log2<= INTEGRATION_LOG2;
                commit          <= 1'b1;
                settling        <= 1'b1;
                settle_count    <= 32'd0;
                settle_ticks    <= settle_target;
                if (change_count != 16'hffff) change_count <= change_count + 16'd1;
            end

            valid <= !settling && enable && ref_valid;
        end
    end

endmodule
`default_nettype wire
