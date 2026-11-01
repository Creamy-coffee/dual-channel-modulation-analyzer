`timescale 1ns/1ps
// ---------------------------------------------------------------------------
// tb_fft_find - state-machine verification of fft_find_ctrl (the 4096-point
// shared-FFT carrier finder that replaces the sweep), driven through a simple
// AFC model that mimics local_afc_2ch: on every commit the "NCO" jumps to
// find_fcw, then walks toward the true carrier FCW in fixed steps and raises
// afc_ready once it is inside AFC_LOCK_FCW.
//
// Parameters are shortened via instantiation overrides so the multi-frame
// sequences run in seconds instead of minutes:
//   SETTLE_CLKS   2e6 -> 120000   LOST_HOLD_CLKS 13e6 -> 400000
//   LOST_FRAMES   8   -> 4         SUPPRESS_FRAMES 8    -> 2
//
// CASES
//   T1 commit + settle + track : tone on bin 200, AFC captures after commit
//   T2 adopt                   : AFC already tracking -> no commit, find_fcw
//                                := li_active_fcw, straight to TRACK
//   T3 carrier vanish          : mag-driven loss (AFC stays ready), re-acquire
//   T4 carrier step            : drift-driven loss (AFC drops ready), re-commit
//                                at the NEW frequency
//   T5 settle failure          : AFC never locks -> suppress window blocks the
//                                failed frequency, then a retry commit
//   T6 rescan                  : host request drops the lock and re-searches
//   T7 disable                 : enable=0 parks find_fcw at F_MIN_FCW
//
// USAGE
//   iverilog -g2012 -DSIMULATION -s tb_fft_find -o tb_fft_find.vvp \
//       tb_fft_find.sv sim_prims.v ../rtl/*.v
//   (run vvp from ../rtl so the .mem files resolve)
// ---------------------------------------------------------------------------
module tb_fft_find;
    localparam integer FS     = 65000000;
    localparam real    BIN_HZ = 65000000.0/4096.0;
    localparam real    TWO_PI = 6.283185307179586;
    localparam [2:0]  S_IDLE=3'd0, S_SEARCH=3'd1, S_CONV=3'd2,
                     S_SETTLE=3'd3, S_TRACK=3'd4, S_LOST=3'd5;

    reg clk = 1'b0;
    always #10 clk = ~clk;

    reg rst_n = 1'b0;
    reg adc_valid = 1'b1;
    reg signed [11:0] adc_ch0 = 12'sd0;
    reg signed [11:0] adc_ch1 = 12'sd0;
    reg afc_en = 1'b1;
    reg rescan = 1'b0;
    reg enable = 1'b1;

    // AFC model outputs (driven below)
    wire        afc_ready;
    wire [31:0] li_active_fcw;

    wire [31:0] find_fcw;  wire commit, locked, busy;  wire [2:0] state;
    wire [31:0] fft_freq0, fft_freq1, frame_count;
    wire [11:0] fft_bin0, fft_bin1;
    wire [23:0] fft_mag0, fft_mag1;
    wire [15:0] lost_count;

    fft_find_ctrl #(
        .SETTLE_CLKS(32'd120000), .LOST_HOLD_CLKS(32'd400000),
        .LOST_FRAMES(8'd4), .SUPPRESS_FRAMES(16'd2)
    ) dut (
        .clk(clk), .rst_n(rst_n), .enable(enable),
        .adc_valid(adc_valid), .adc_ch0(adc_ch0), .adc_ch1(adc_ch1),
        .afc_en(afc_en), .afc_ready(afc_ready), .li_active_fcw(li_active_fcw),
        .rescan(rescan),
        .find_fcw(find_fcw), .commit(commit), .locked(locked), .busy(busy),
        .state(state),
        .fft_freq0(fft_freq0), .fft_freq1(fft_freq1),
        .fft_bin0(fft_bin0), .fft_bin1(fft_bin1),
        .fft_mag0(fft_mag0), .fft_mag1(fft_mag1),
        .frame_count(frame_count), .lost_count(lost_count));

    // ---- stimulus: one tone on CH0 (CH1 quiet) -----------------------------
    real ph0, ph1, step0, step1, amp0, amp1;
    real t0, t1;
    always @(posedge clk) begin
        if (!rst_n) begin
            ph0 <= 0.0; ph1 <= 0.0;
            adc_ch0 <= 12'sd0; adc_ch1 <= 12'sd0;
        end else begin
            t0 = ph0 + step0;
            t1 = ph1 + step1;
            if (t0 >= TWO_PI) t0 = t0 - TWO_PI;
            if (t1 >= TWO_PI) t1 = t1 - TWO_PI;
            ph0 <= t0; ph1 <= t1;
            adc_ch0 <= $rtoi(amp0 * $sin(t0));
            adc_ch1 <= $rtoi(amp1 * $sin(t1));
        end
    end

    // ---- AFC model ----------------------------------------------------------
    // target_fcw is the FCW of the tone as the finder sees it
    // (bin * 2^32/4096); model_fcw is what the lock-in NCO currently runs at.
    reg  [31:0] model_fcw, target_fcw;
    reg         mdl_ready, mdl_tracking, afc_hold;
    reg  [31:0] mdl_timer;
    reg         commit_d;
    localparam [31:0] AFC_STEP_CLKS = 32'd3000;   // one loop update
    localparam [31:0] AFC_STEP_FCW  = 32'd24000;  // ~366 Hz per update
    localparam [31:0] AFC_LOCK_FCW  = 32'd4000;   // ~61 Hz lock window
    assign afc_ready     = mdl_ready && !afc_hold;   // hold forces a loss
    assign li_active_fcw = model_fcw;

    always @(posedge clk) commit_d <= commit;

    always @(posedge clk) begin
        if (!rst_n) begin
            model_fcw<=32'd0; target_fcw<=32'd0; mdl_ready<=1'b0;
            mdl_tracking<=1'b0; afc_hold<=1'b0; mdl_timer<=32'd0;
        end else begin
            if (commit_d) begin
                // NCO jumps to the freshly committed frequency; the loop must
                // re-capture from there.
                model_fcw<=find_fcw; mdl_ready<=1'b0;
                mdl_timer<=32'd0; mdl_tracking<=1'b1;
            end else if (mdl_tracking && !afc_hold) begin
                if (mdl_timer>=AFC_STEP_CLKS) begin
                    mdl_timer<=32'd0;
                    if (model_fcw < target_fcw-AFC_STEP_FCW)
                        model_fcw<=model_fcw+AFC_STEP_FCW;
                    else if (model_fcw > target_fcw+AFC_STEP_FCW)
                        model_fcw<=model_fcw-AFC_STEP_FCW;
                    else
                        model_fcw<=target_fcw;
                    if (model_fcw > target_fcw-AFC_LOCK_FCW &&
                        model_fcw < target_fcw+AFC_LOCK_FCW)
                        mdl_ready<=1'b1;
                end else mdl_timer<=mdl_timer+32'd1;
            end
        end
    end

    // ---- helpers ------------------------------------------------------------
    integer errors = 0;
    integer commits_seen = 0;
    always @(posedge clk) if (commit) commits_seen = commits_seen + 1;

    task check;
        input c;
        input [8*56:1] msg;
        begin
            if (!c) begin
                errors = errors + 1;
                $display("  FAIL: %0s (state=%0d t=%0t)", msg, state, $time);
            end
        end
    endtask

    function [31:0] fabs32;
        input [31:0] a, b;
        begin
            fabs32 = (a>=b) ? (a-b) : (b-a);
        end
    endfunction

    // wait until the finder reaches st, or fail after ~timeout cycles
    task wait_state;
        input [2:0] st;
        input integer timeout;
        integer c;
        begin
            c = 0;
            while (state !== st && c < timeout) begin
                @(posedge clk); c = c + 1;
            end
            check(state===st, "wait_state reached");
            if (state !== st)
                $display("    (wanted state %0d, got %0d after %0d cycles)", st, state, c);
        end
    endtask

    // wait for n more published FFT frames (frame_count is the finder's own
    // telemetry counter, one per published frame)
    task wait_frames;
        input integer n;
        integer base;
        begin
            base = frame_count;
            while (frame_count < base + n) @(posedge clk);
            @(posedge clk);
        end
    endtask

    real tone_bin;
    reg [31:0] exp_fcw;
    integer c0;

    initial begin
        step0 = 0.0; step1 = 0.0; amp0 = 0.0; amp1 = 0.0;
        rst_n = 1'b0;
        #100 rst_n = 1'b1;

        // =================================================================
        // T1: commit -> settle -> track
        // =================================================================
        $display("T1 commit+settle+track (tone bin 200)");
        tone_bin = 200.0;
        exp_fcw  = $rtoi(tone_bin * 4294967296.0 / 4096.0);
        target_fcw = exp_fcw;
        amp0 = 200.0; amp1 = 0.0;
        step0 = TWO_PI * tone_bin / 4096.0;
        step1 = 0.0;
        afc_hold = 1'b0;
        wait_state(S_TRACK, 4000000);         // ~12 frames worst case
        check(locked===1'b1, "T1 locked");
        check(busy===1'b0,   "T1 not busy");
        check(commits_seen>=1, "T1 at least one commit");
        // handoff accuracy: interpolation bias is < 0.15 bin (~2.4 kHz)
        check(fabs32(find_fcw, exp_fcw) < 32'd160000, "T1 find_fcw within 0.15 bin");
        $display("   find_fcw=%0d exp=%0d err=%0d FCW (%.1f Hz)",
                 find_fcw, exp_fcw, fabs32(find_fcw, exp_fcw),
                 fabs32(find_fcw, exp_fcw)*65000000.0/4294967296.0);
        // telemetry of the frame that locked it
        check(fft_bin0>=12'd199 && fft_bin0<=12'd201, "T1 telemetry bin ~200");
        check(fft_freq0 > 32'd3100000 && fft_freq0 < 32'd3300000, "T1 telemetry freq");
        check(fft_mag0 > 24'd70000, "T1 telemetry mag (16A^2>>3)");
        wait_frames(1);
        check(commits_seen==1, "T1 no dither commits while tracking");

        // =================================================================
        // T2: adopt - AFC already tracking this tone, no commit allowed
        // =================================================================
        $display("T2 adopt (AFC pre-tracking)");
        // reset the whole scene: fresh search with the model already locked
        rst_n = 1'b0;
        #100 rst_n = 1'b1;
        model_fcw = exp_fcw; mdl_ready = 1'b1; mdl_tracking = 1'b0;
        afc_hold = 1'b0; commit_d = 1'b0;
        c0 = commits_seen;
        wait_state(S_TRACK, 4000000);
        check(commits_seen==c0, "T2 adopt path never commits");
        check(find_fcw===li_active_fcw, "T2 find_fcw := li_active_fcw");
        check(locked===1'b1, "T2 locked");
        wait_frames(1);
        check(commits_seen==c0, "T2 still no commits in track");

        // =================================================================
        // T3: carrier vanishes while the AFC stays ready -> mag-driven loss
        // =================================================================
        $display("T3 carrier vanish (mag-driven loss, re-acquire)");
        c0 = commits_seen;
        amp0 = 0.0;
        wait_state(S_SEARCH, 4000000);        // LOST is one cycle, then SEARCH
        check(locked===1'b0, "T3 unlocked after loss");
        check(busy===1'b1,   "T3 busy after loss");
        check(lost_count>=16'd1, "T3 lost_count incremented");
        // Carrier returns.  Hold the AFC "not ready" so the adopt path is
        // unavailable: the re-acquire MUST go through a commit, which also
        // proves 'committed' was cleared at the loss (no MIN_STEP deadlock).
        amp0 = 200.0;
        afc_hold = 1'b1;
        wait_state(S_SETTLE, 4000000);
        check(commits_seen==c0+1, "T3 re-acquire commits (not adopt)");
        afc_hold = 1'b0;                      // AFC captures again
        wait_state(S_TRACK, 6000000);
        check(locked===1'b1, "T3 re-locked");
        check(fabs32(find_fcw, exp_fcw) < 32'd160000, "T3 re-acquire same freq");
        $display("   re-acquire commits=%0d find_fcw=%0d", commits_seen-c0, find_fcw);

        // =================================================================
        // T4: carrier steps out of the AFC window -> drift-driven loss,
        //     re-acquire at the NEW frequency
        // =================================================================
        $display("T4 carrier step (drift-driven loss, re-commit)");
        tone_bin = 260.0;
        exp_fcw  = $rtoi(tone_bin * 4294967296.0 / 4096.0);
        target_fcw = exp_fcw;
        step0 = TWO_PI * tone_bin / 4096.0;
        afc_hold = 1'b1;                       // AFC loses the stepped carrier
        c0 = commits_seen;
        wait_state(S_SEARCH, 4000000);
        check(locked===1'b0, "T4 unlocked after step");
        check(lost_count>=16'd2, "T4 lost_count incremented again");
        afc_hold = 1'b0;                       // AFC can capture again
        wait_state(S_SETTLE, 6000000);         // re-commit (not adopt: the
        check(commits_seen==c0+1, "T4 re-commits at the new frequency");
        afc_hold = 1'b0;
        wait_state(S_TRACK, 6000000);
        check(locked===1'b1, "T4 re-locked");
        check(fabs32(find_fcw, exp_fcw) < 32'd160000, "T4 find_fcw at new freq");
        $display("   new find_fcw=%0d exp=%0d commits=%0d",
                 find_fcw, exp_fcw, commits_seen-c0);

        // =================================================================
        // T5: AFC never locks -> settle timeout -> suppress window -> retry
        // =================================================================
        $display("T5 settle failure suppression");
        rst_n = 1'b0;
        #100 rst_n = 1'b1;
        model_fcw = 32'd0; mdl_ready = 1'b0; mdl_tracking = 1'b0;
        afc_hold = 1'b1;                       // AFC will never become ready
        c0 = commits_seen;
        // The finder must commit anyway (!afc_ready is a commit enabler), then
        // SETTLE runs out.  Wait for the commit FIRST: SEARCH is also the
        // state the machine passes through on its way to that commit.
        while (commits_seen != c0+1) @(posedge clk);
        check(commits_seen==c0+1, "T5 commit happens despite no AFC lock");
        // settle timeout bounces back to SEARCH with the frequency suppressed
        wait_state(S_SEARCH, 2000000);
        check(commits_seen==c0+1, "T5 no extra commit before suppress");
        // SUPPRESS_FRAMES=2: the same frequency must not re-commit
        wait_frames(1);
        check(commits_seen==c0+1, "T5 suppressed frame 1 does not commit");
        wait_frames(1);
        check(commits_seen==c0+1, "T5 suppressed frame 2 does not commit");
        // window expires -> retry (afc_ready=0 keeps the retry condition true,
        // so the search can never stall permanently)
        wait_frames(3);
        check(commits_seen>c0+1, "T5 retries after the window expires");
        $display("   commits after suppress window=%0d", commits_seen-c0);

        // =================================================================
        // T6: rescan drops the lock and re-searches
        // =================================================================
        $display("T6 rescan");
        rst_n = 1'b0;
        #100 rst_n = 1'b1;
        tone_bin = 200.0;
        exp_fcw  = $rtoi(tone_bin * 4294967296.0 / 4096.0);
        target_fcw = exp_fcw;
        step0 = TWO_PI * tone_bin / 4096.0;
        amp0 = 200.0;
        afc_hold = 1'b0;
        wait_state(S_TRACK, 4000000);
        check(locked===1'b1, "T6 locked before rescan");
        c0 = commits_seen;
        rescan = 1'b1;
        @(posedge clk);
        rescan = 1'b0;
        @(posedge clk);
        check(state===S_SEARCH, "T6 rescan returns to SEARCH");
        check(locked===1'b0,   "T6 rescan drops locked");
        check(busy===1'b1,     "T6 rescan raises busy");
        // Hold the AFC not-ready so the re-acquire must commit (the adopt path
        // would be equally legal, but the commit path is the one under test).
        afc_hold = 1'b1;
        wait_state(S_SETTLE, 6000000);
        check(commits_seen==c0+1, "T6 re-acquires after rescan");
        afc_hold = 1'b0;
        wait_state(S_TRACK, 6000000);
        check(locked===1'b1, "T6 locked again");
        check(fabs32(find_fcw, exp_fcw) < 32'd160000, "T6 same frequency");

        // =================================================================
        // T7: disable parks find_fcw at F_MIN_FCW (config_ok stays true)
        // =================================================================
        $display("T7 disable parks find_fcw");
        enable = 1'b0;
        repeat (4) @(posedge clk);
        check(state===S_IDLE, "T7 state IDLE when disabled");
        check(locked===1'b0,  "T7 not locked when disabled");
        check(busy===1'b1,    "T7 busy when disabled");
        check(find_fcw===32'd13215, "T7 find_fcw parked at F_MIN_FCW");
        enable = 1'b1;
        repeat (4) @(posedge clk);
        check(state===S_SEARCH, "T7 re-enable searches again");

        if (errors==0) $display("TB_PASS tb_fft_find");
        else           $display("TB_FAIL tb_fft_find: %0d errors", errors);
        $finish;
    end

    // safety timeout: ~10 frames of FFT plus state waits; 400 ms of TB time
    // (20M cycles) is far more than every case above needs.
    initial begin
        #800000000;
        $display("TB_FAIL tb_fft_find: timeout");
        $finish;
    end
endmodule
