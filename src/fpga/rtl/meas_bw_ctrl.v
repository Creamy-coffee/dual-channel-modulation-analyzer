`timescale 1ns/1ps
`default_nettype none
// ---------------------------------------------------------------------------
// Measurement bandwidth controller  --  picks the lock-in integration window
// from how STABLE the signal is.  It must NEVER look at the carrier frequency.
// ---------------------------------------------------------------------------
// WHY NOT THE CARRIER FREQUENCY
// -----------------------------
//   The CIC runs AFTER the quadrature mixer, so the wanted tone always sits at
//   DC.  The decimation ratio therefore does not have to cover the carrier --
//   that is why a 30 MHz tone measures fine at R=16 (fs_eff 4.06 MHz).  What R
//   buys is WINDOW LENGTH:  T_win = N*R/fs_in,  ENBW ~ 1.5/T_win (Hann).
//   auto_rate_ctrl picked the gear from f_ref, which pinned every carrier above
//   100 kHz to the 1 ms gear forever -- i.e. a weak 30 MHz tone could never
//   reach the narrow band it needs.  This module decouples the two.
//
// WHAT DECIDES THE GEAR
// ---------------------
//   Narrow the band when the signal is STABLE; drop back to the fast gear the
//   moment it is not.  Deliberately NOT included: "ripple is small".  Ripple is
//   LARGEST before narrowing -- that is the reason to narrow.  Gating on ripple
//   would deadlock every weak signal at the widest band.  So the upgrade test
//   only asks "is it there, is it locked, did it jump", never "is it quiet".
//
//      gear 0  fast     R=16     T_win 1.008 ms   ENBW 1488 Hz   ~992 rd/s
//      gear 1  balanced R=64     T_win 4.033 ms   ENBW  372 Hz   ~248 rd/s
//      gear 2  precise  R=1024   T_win 64.53 ms   ENBW   23 Hz   ~15.5 rd/s
//
//   R=256 (16 ms) is skipped: it sits too close to R=1024 to justify another
//   state, and the 16x jump 64->1024 is absorbed by the settling window anyway.
//   Do NOT add R=4096: at R=1024 the 3rd-order CIC integrator already uses the
//   full 58-bit accumulator (scripts/rate_plan.py).
//
// TRANSITION
// ----------
//   A gear change reconfigures only baseband filters and integration. NCO and
//   coarse AFC continue; filter/guard flush still means transition readings are
//   meaningless.  'settling' is raised and 'valid' is dropped for three windows
//   of the NEW gear.  This is the same "phase continuous != measurement valid"
//   rule used by afc_controller and auto_rate_ctrl.
// ---------------------------------------------------------------------------
module meas_bw_ctrl #(
    parameter [4:0]  INTEGRATION_LOG2 = 5'd12,   // 4096 points per window
    parameter [23:0] AMP_MIN          = 24'd2048,// below this the tone is gone
    parameter [7:0]  UPGRADE_WINS     = 8'd8,    // consecutive good windows to narrow
    parameter [7:0]  DROP_WINS        = 8'd2,    // consecutive bad windows to fall back
    parameter [7:0]  AMP_JUMP_SHIFT   = 8'd1,    // amp moved >1/2 -> step, not noise
    parameter [15:0] PHASE_JUMP       = 16'd9102,// 50 deg; 65536 counts = 360 deg
    parameter [7:0]  SETTLE_WINDOWS   = 8'd3,    // windows to invalidate after a change
    // A gear change re-commits the lock-in, which resets the AFC observer.  AFC
    // then needs several windows to re-converge, so "AFC not ready" right after
    // a change is SELF-INFLICTED, not a signal fault.  Falling back on that
    // would make the controller fight itself: widen -> AFC resets -> widen
    // again -> never narrows.  Give it this many windows before treating an
    // unlocked AFC as a real loss of lock.
    parameter [7:0]  AFC_LOST_WINS    = 8'd8,
    // Ceiling for the automatic narrowing.  Keep at 2 on the bench; drop to 1
    // if a high carrier turns out to alias into the 23 Hz band (the 2*f_ref
    // image is not attenuated by the CIC, only by the window itself).
    parameter integer REQUIRE_PRECISION=0,
    parameter [1:0]  MAX_GEAR         = 2'd2
)(
    input  wire        clk, rst_n, enable,
    // one pulse per finished integration window, results already latched
    input  wire        meas_valid,
    input  wire [23:0] amp0, amp1,
    input  wire signed [15:0] phase_diff,
    input  wire        afc_enable, afc_ready, precision_ready,
    output reg  [1:0]  gear,            // 0=fast 1=balanced 2=precise
    output reg  [1:0]  decim_sel,       // -> cfg_decim_sel
    output reg  [4:0]  integration_log2,// -> cfg_integration_log2
    output reg         commit,
    output reg         settling,
    output reg         valid,
    output reg  [15:0] change_count,
    output reg  [7:0]  good_windows,
    output reg  [7:0]  bad_windows,      // consecutive windows below AMP_MIN
    output reg  [7:0]  afc_lost_windows, // consecutive windows without AFC lock
    output reg  [15:0] last_phase_step, // |circular d(pdiff)|, 65536 = 360 deg
    output reg  [31:0] settle_ticks
);

    // gear -> decim_sel.  Gear 2 jumps straight to R=1024 (decim_sel 3).
    function [1:0] gear_to_decim;
        input [1:0] g;
        begin
            case (g)
                2'd0: gear_to_decim = 2'd0;
                2'd1: gear_to_decim = 2'd1;
                2'd2: gear_to_decim = 2'd3;
                default: gear_to_decim = 2'd0;
            endcase
        end
    endfunction

    wire [5:0] decim_log2 = 6'd4 + {3'd0, decim_sel, 1'b0};
    // One window in clocks = 2^(integration_log2 + decim_log2), max 2^22.
    // The settlement has to be measured in windows of the NEW gear: decim_sel is
    // assigned non-blocking, so a wire built from it would still carry the OLD
    // ratio and under-wait by up to 16x on the 64 -> 1024 step.
    wire [1:0]  dsel_next   = gear_to_decim(gear + 2'd1);
    wire [31:0] settle_up   = ((32'd1 << ({1'b0, INTEGRATION_LOG2} + 6'd4 + {3'd0, dsel_next, 1'b0}))
                               * SETTLE_WINDOWS);
    wire [31:0] settle_down = ((32'd1 << ({1'b0, INTEGRATION_LOG2} + 6'd4))
                               * SETTLE_WINDOWS);

    // ---- per-window stability observation ----------------------------------
    reg [23:0] amp0_prev, amp1_prev;
    reg signed [15:0] pdiff_prev;
    reg        have_prev;

    wire [23:0] d0 = (amp0 > amp0_prev) ? (amp0 - amp0_prev) : (amp0_prev - amp0);
    wire [23:0] d1 = (amp1 > amp1_prev) ? (amp1 - amp1_prev) : (amp1_prev - amp1);
    wire amp_jump = have_prev &&
                    ((d0 > (amp0_prev >> AMP_JUMP_SHIFT)) ||
                     (d1 > (amp1_prev >> AMP_JUMP_SHIFT)));

    // circular distance on a signed 16-bit turn counter (-32768..32767 = -180..180 deg)
    wire signed [16:0] dp_raw  = {phase_diff[15], phase_diff} - {pdiff_prev[15], pdiff_prev};
    wire signed [16:0] dp_wrap = (dp_raw >  17'sd32767) ? (dp_raw - 17'sd65536)
                               : (dp_raw < -17'sd32768) ? (dp_raw + 17'sd65536)
                               : dp_raw;
    wire [15:0] dp_abs = dp_wrap[16] ? (~dp_wrap[15:0] + 16'd1) : dp_wrap[15:0];
    wire phase_jump = have_prev && (dp_abs > PHASE_JUMP);

    wire amp_ok  = (amp0 >= AMP_MIN) && (amp1 >= AMP_MIN);
    wire freq_ok = !afc_enable || (afc_ready && (!REQUIRE_PRECISION || gear!=2 || precision_ready));
    wire upgrade_ok = !REQUIRE_PRECISION || !afc_enable || gear==0 || precision_ready;
    wire good    = amp_ok && freq_ok && !amp_jump && !phase_jump;

    reg [31:0] settle_count, settle_goal;
    reg [7:0]  afc_lost;
    reg        boot_done;

    always @(posedge clk) begin
        if (!rst_n) begin
            gear <= 2'd0; decim_sel <= 2'd0; integration_log2 <= INTEGRATION_LOG2;
            commit <= 1'b0; settling <= 1'b0; valid <= 1'b0;
            change_count <= 16'd0; good_windows <= 8'd0; bad_windows <= 8'd0;
            afc_lost <= 8'd0; afc_lost_windows <= 8'd0;
            last_phase_step <= 16'd0; settle_ticks <= 32'd0;
            settle_count <= 32'd0; settle_goal <= 32'd0; boot_done <= 1'b0;
            amp0_prev <= 24'd0; amp1_prev <= 24'd0; pdiff_prev <= 16'sd0;
            have_prev <= 1'b0;
        end else begin
            commit <= 1'b0;

            // Boot commit: the lock-in only latches configuration on cfg_commit.
            // This module owns the commit, so it must reproduce the startup pulse
            // or the measurement chain never starts.
            if (!boot_done) begin
                boot_done <= 1'b1;
                commit    <= 1'b1;
            end

            if (settling) begin
                if (settle_count >= settle_goal) begin
                    settling     <= 1'b0;
                    settle_count <= 32'd0;
                    settle_ticks <= 32'd0;
                end else begin
                    settle_count <= settle_count + 32'd1;
                    settle_ticks <= settle_goal - settle_count;
                end
            end

            if (meas_valid) begin
                last_phase_step <= dp_abs;
                amp0_prev       <= amp0;
                amp1_prev       <= amp1;
                pdiff_prev      <= phase_diff;
                have_prev       <= 1'b1;

                if (!enable || settling) begin
                    // transitions are not judged; hold the counters
                    good_windows <= 8'd0;
                    bad_windows  <= 8'd0;
                    afc_lost     <= 8'd0;
                end else if (good) begin
                    bad_windows <= 8'd0;
                    afc_lost    <= 8'd0;
                    if (good_windows != 8'hff) good_windows <= good_windows + 8'd1;
                    if (good_windows + 8'd1 >= UPGRADE_WINS && gear < MAX_GEAR && upgrade_ok) begin
                        gear            <= gear + 2'd1;
                        decim_sel       <= gear_to_decim(gear + 2'd1);
                        integration_log2<= INTEGRATION_LOG2;
                        commit          <= 1'b1;
                        settling        <= 1'b1;
                        settle_count    <= 32'd0;
                        settle_goal     <= settle_up;
                        settle_ticks    <= settle_up;
                        good_windows    <= 8'd0;
                        bad_windows     <= 8'd0;
                        afc_lost        <= 8'd0;
                        if (change_count != 16'hffff) change_count <= change_count + 16'd1;
                    end
                end else begin
                    good_windows <= 8'd0;
                    // Count the two failure kinds separately.  Losing the tone
                    // and losing AFC lock need very different debounce times:
                    // a dropout is the user switching the source off, while an
                    // unlocked AFC right after a gear change is self-inflicted
                    // and must not bounce the gear straight back.
                    if (!amp_ok) begin
                        if (bad_windows != 8'hff) bad_windows <= bad_windows + 8'd1;
                    end else begin
                        bad_windows <= 8'd0;   // an AFC-only failure is not a dropout
                    end
                    if (!freq_ok) begin
                        if (afc_lost != 8'hff) afc_lost <= afc_lost + 8'd1;
                    end else begin
                        afc_lost <= 8'd0;
                    end
                    // A step is a ONE-WINDOW event: by the next window amp0_prev
                    // has already moved to the new level, so the jump test is
                    // false again and a "N consecutive bad windows" debounce
                    // would never fire.  Steps therefore fall back at once.
                    if (gear != 2'd0 &&
                        (amp_jump || phase_jump ||
                         (bad_windows + 8'd1 >= DROP_WINS) ||
                         (afc_lost + 8'd1 >= AFC_LOST_WINS))) begin
                        gear            <= 2'd0;
                        decim_sel       <= 2'd0;
                        integration_log2<= INTEGRATION_LOG2;
                        commit          <= 1'b1;
                        settling        <= 1'b1;
                        settle_count    <= 32'd0;
                        settle_goal     <= settle_down;
                        settle_ticks    <= settle_down;
                        good_windows    <= 8'd0;
                        bad_windows     <= 8'd0;
                        afc_lost        <= 8'd0;
                        if (change_count != 16'hffff) change_count <= change_count + 16'd1;
                    end
                end
            end

            afc_lost_windows <= afc_lost;

            valid <= !settling && enable;
        end
    end

endmodule
`default_nettype wire
