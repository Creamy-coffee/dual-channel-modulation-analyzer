`timescale 1ns/1ps
// ---------------------------------------------------------------------------
// meas_bw_ctrl unit test
//   The controller must narrow the band while the tone is STABLE and drop back
//   to the fast gear the moment it steps or disappears.  It must never look at
//   the carrier frequency -- there is no such input, and that is the point.
//
//   Two different fallback paths are exercised on purpose:
//     step  (amp/phase) -> single-window event, must fire immediately
//     state (AFC lost)  -> persistent, must survive the DROP_WINS debounce
// ---------------------------------------------------------------------------
module tb_meas_bw_ctrl;
    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg rst_n = 1'b0, enable = 1'b1;
    reg meas_valid = 1'b0;
    reg [23:0] amp0 = 24'd0, amp1 = 24'd0;
    reg signed [15:0] pdiff = 16'sd0;
    reg afc_enable = 1'b1, afc_ready = 1'b1;

    wire [1:0] gear, decim_sel;
    wire [4:0] integ;
    wire commit, settling, valid;
    wire [15:0] change_count, last_phase_step;
    wire [7:0] good_windows, bad_windows;
    wire [7:0] afc_lost_windows;
    wire [31:0] settle_ticks;

    integer errors = 0;
    integer i;

    // Short window keeps the 64 ms gear affordable in simulation.  N = 2^3 = 8
    // points, so one gear-2 window is 8*1024 = 8192 clocks and three are 24576.
    meas_bw_ctrl #(.INTEGRATION_LOG2(5'd3)) dut (
        .clk(clk), .rst_n(rst_n), .enable(enable),
        .meas_valid(meas_valid),
        .amp0(amp0), .amp1(amp1), .phase_diff(pdiff),
        .afc_enable(afc_enable), .afc_ready(afc_ready),
        .gear(gear), .decim_sel(decim_sel), .integration_log2(integ),
        .commit(commit), .settling(settling), .valid(valid),
        .change_count(change_count),
        .good_windows(good_windows), .bad_windows(bad_windows),
        .afc_lost_windows(afc_lost_windows),
        .last_phase_step(last_phase_step), .settle_ticks(settle_ticks)
    );

    task check(input cond, input [255:0] msg);
        begin
            if (!cond) begin
                errors = errors + 1;
                $display("  FAIL  %0s", msg);
            end
        end
    endtask

    // one finished integration window; drive on negedge to avoid races
    task win(input [23:0] a, input signed [15:0] p);
        begin
            @(negedge clk);
            amp0 = a; amp1 = a; pdiff = p; meas_valid = 1'b1;
            @(negedge clk);
            meas_valid = 1'b0;
            @(negedge clk);
        end
    endtask

    task n_wins(input integer n, input [23:0] a, input signed [15:0] p);
        begin
            for (i = 0; i < n; i = i + 1) win(a, p);
        end
    endtask

    // Wait for `settling` to fall WITHOUT issuing a measurement window.
    //
    // This task used to loop on @(negedge clk) with meas_valid already low, so
    // it was fine; the bug was in the callers, which passed through win() again
    // while a 24576-clock settle (gear 2, R=1024, N=8) was still running.  The
    // controller deliberately raises `settling` for three windows of the NEW
    // gear, so a spare meas_valid pulse lands INSIDE the settle and is counted
    // by the settling branch: good_windows is cleared and the accumulation for
    // check 6 restarts from zero.  The bench then sees gear=1 where it expects
    // gear=0.  Poll it here and let the caller do one explicit window after.
    task wait_settled;
        integer n;
        begin
            n = 0;
            while (settling && n < 4000000) begin
                @(negedge clk);
                n = n + 1;
            end
            // one extra cycle so `valid` (registered from !settling) is updated
            @(negedge clk);
            if (settling) begin
                errors = errors + 1;
                $display("  FAIL  settling never cleared (%0d edges)", n);
            end
        end
    endtask

    // settle_ticks counts down inside the loop, so read it as a range instead
    // of the exact load value.
    task check_settle(input integer want, input [255:0] msg);
        begin
            if (settle_ticks > want || settle_ticks < want - 8) begin
                errors = errors + 1;
                $display("  FAIL  %0s (settle_ticks=%0d want~%0d)", msg, settle_ticks, want);
            end
        end
    endtask

    initial begin
        // ---- 1. reset: fastest gear, exactly one boot commit ----------------
        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        @(negedge clk);
        check(gear == 2'd0, "boot gear is 0");
        check(decim_sel == 2'd0, "boot decim_sel is 0 (R=16)");
        check(commit === 1'b1, "boot commit pulse");
        @(negedge clk);
        check(commit === 1'b0, "commit is a one-cycle pulse");
        $display("  1 boot: gear=%0d decim=%0d", gear, decim_sel);

        // ---- 2. stable tone narrows one gear after UPGRADE_WINS windows ----
        n_wins(7, 24'd100000, 16'sd0);
        check(gear == 2'd0, "7 good windows are not enough to narrow");
        win(24'd100000, 16'sd0);
        @(negedge clk);
        check(gear == 2'd1, "8th good window narrows to gear 1");
        check(decim_sel == 2'd1, "gear 1 -> decim_sel 1 (R=64)");
        check(settling === 1'b1, "settling raised after the change");
        check(valid === 1'b0, "valid dropped during settling");
        check_settle(1536, "settle = 3 windows of the NEW gear (8*64*3)");
        $display("  2 upgrade: gear=%0d decim=%0d settle_ticks=%0d", gear, decim_sel, settle_ticks);
        wait_settled;
        check(valid === 1'b1, "valid returns after settling");

        // ---- 3. keeps narrowing to the precise gear, then stops -------------
        n_wins(8, 24'd100000, 16'sd0);
        @(negedge clk);
        check(gear == 2'd2, "second upgrade reaches gear 2");
        check(decim_sel == 2'd3, "gear 2 -> decim_sel 3 (R=1024)");
        check_settle(24576, "settle = 3 windows of R=1024 (8*1024*3)");
        $display("  3 upgrade: gear=%0d decim=%0d settle_ticks=%0d", gear, decim_sel, settle_ticks);
        wait_settled;
        n_wins(12, 24'd100000, 16'sd0);
        check(gear == 2'd2, "gear 2 is the cap");
        $display("  3 cap: gear stays %0d", gear);

        // ---- 4. amplitude step (>50%) falls back on the FIRST window --------
        win(24'd200000, 16'sd0);            // 100000 -> 200000 = +100%
        @(negedge clk);
        check(gear == 2'd0, "amplitude step falls back at once");
        check(decim_sel == 2'd0, "fallback restores R=16");
        check(change_count == 16'd3, "three gear changes so far");
        $display("  4 amp step: gear=%0d changes=%0d", gear, change_count);
        wait_settled;

        // ---- 5. phase step (90 deg) falls back on the FIRST window ---------
        n_wins(8, 24'd200000, 16'sd0);      // narrow again first
        wait_settled;
        check(gear == 2'd1, "re-narrowed after the step settled");
        win(24'd200000, 16'sd16384);        // 90 deg jump
        @(negedge clk);
        check(last_phase_step == 16'd16384, "phase step observed as 16384 counts");
        check(gear == 2'd0, "phase step falls back at once");
        $display("  5 phase step: gear=%0d last_phase_step=%0d", gear, last_phase_step);
        wait_settled;

        // ---- 6. AFC loss is persistent: AFC_LOST_WINS debounce applies ------
        // The parameter is AFC_LOST_WINS = 8, NOT DROP_WINS = 2.  The bench used
        // to assert a single-unlocked-window-holds / second-drops pattern, which
        // contradicted the RTL's own comment: an AFC that is still re-converging
        // after a gear change is self-inflicted, so it gets a long debounce and
        // only a genuinely persistent loss may narrow the band back.  Measured:
        // afc_lost goes 1,2,...,7 with the gear held, and the gear drops on the
        // 8th unlocked window.  Assert exactly that.
        n_wins(8, 24'd200000, 16'sd16384);
        wait_settled;
        check(gear == 2'd1, "narrowed before the AFC loss");
        afc_ready = 1'b0;
        for (i = 0; i < 7; i = i + 1) begin
            win(24'd200000, 16'sd16384);
            @(negedge clk);
            check(gear == 2'd1, "gear held while AFC loss is not yet persistent");
        end
        check(afc_lost_windows == 8'd7, "seven unlocked windows counted");
        win(24'd200000, 16'sd16384);
        @(negedge clk);
        check(gear == 2'd0, "8th unlocked window falls back (AFC_LOST_WINS)");
        $display("  6 afc loss: gear=%0d", gear);
        afc_ready = 1'b1;
        wait_settled;

        // ---- 7. amplitude below the floor falls back ------------------------
        n_wins(8, 24'd200000, 16'sd16384);
        wait_settled;
        check(gear == 2'd1, "narrowed before dropout");
        win(24'd1000, 16'sd16384);          // below AMP_MIN=2048
        @(negedge clk);
        check(gear == 2'd0, "dropout below AMP_MIN falls back");
        $display("  7 dropout: gear=%0d", gear);
        wait_settled;

        // ---- 8. AFC not ready: hold the fast gear --------------------------
        afc_ready = 1'b0;
        n_wins(20, 24'd200000, 16'sd16384);
        check(gear == 2'd0, "no narrowing while AFC is unlocked");
        $display("  8 afc unlocked: gear=%0d good=%0d", gear, good_windows);
        afc_ready = 1'b1;

        // ---- 9. AFC disabled entirely: stability alone is enough -----------
        afc_enable = 1'b0;
        n_wins(8, 24'd200000, 16'sd16384);
        wait_settled;
        check(gear == 2'd1, "narrows on stability alone when AFC is off");
        $display("  9 afc off: gear=%0d changes=%0d", gear, change_count);

        if (errors == 0) $display("TB_PASS tb_meas_bw_ctrl");
        else             $display("TB_FAIL tb_meas_bw_ctrl: %0d failures", errors);
        $finish;
    end

    initial begin
        #20000000;
        $display("TB_FAIL tb_meas_bw_ctrl: timeout");
        $finish;
    end
endmodule
