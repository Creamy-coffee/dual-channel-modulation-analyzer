`timescale 1ns/1ps
// ---------------------------------------------------------------------------
// Functional test for the run-time frequency search (freq_scan_ctrl).
//
// The DUT is driven by a MODEL of the lock-in measurement chain instead of the
// real one so the sweep can be exercised over the full band in seconds:
//   - a result appears exactly one window after each cfg_commit
//   - its amplitude is AMP_TRUE * |G(df*T_win)|, G = Hann window response
// This reproduces the only property the sweep depends on: a detuned carrier is
// attenuated, so the true carrier is the maximum of the sweep.
//
// The model does NOT replace the real-chain tests (tb_local_afc, tb_lockin_*);
// it only proves the search converges and picks the right candidate.
// ---------------------------------------------------------------------------
module tb_freq_scan;
    // ---- clock -------------------------------------------------------------
    reg clk = 1'b0;
    always #5 clk = ~clk;   // 100 MHz sim clock; only ratios matter

    // ---- scenario from plusargs -------------------------------------------
    //   +F_HZ=<carrier Hz>        carrier present at this frequency
    //   +AMP=<Q12 amplitude>      0 = no signal at all
    integer true_hz  = 1000000;
    integer amp_true = 84254;      // ~100 mVpp at 30 MHz, from board data
    initial begin
        if (!$value$plusargs("F_HZ=%d",  true_hz))  true_hz  = 1000000;
        if (!$value$plusargs("AMP=%d",   amp_true)) amp_true = 84254;
    end
    // fcw = f * 2^32 / 65e6
    localparam real FCW_PER_HZ = 4294967296.0 / 65.0e6;
    wire [31:0] true_fcw = true_hz * FCW_PER_HZ;

    // ---- DUT ---------------------------------------------------------------
    reg rst_n = 1'b0, enable = 1'b0, meas_valid = 1'b0;
    reg [23:0] meas_amp0 = 24'd0, meas_amp1 = 24'd0;
    wire [31:0] active_fcw, best_fcw;
    wire [4:0]  active_integ;
    wire        commit, busy, locked;
    wire [2:0]  state;
    wire [23:0] best_amp;
    wire [15:0] scan_count, pass_count;
    wire [7:0]  lost_count;

    freq_scan_ctrl dut (
        .clk(clk), .rst_n(rst_n), .enable(enable),
        .meas_valid(meas_valid), .meas_amp0(meas_amp0), .meas_amp1(meas_amp1),
        .rescan(1'b0),
        .active_fcw(active_fcw), .active_integ(active_integ), .commit(commit),
        .busy(busy), .locked(locked), .state(state),
        .best_fcw(best_fcw), .best_amp(best_amp),
        .scan_count(scan_count), .lost_count(lost_count), .pass_count(pass_count));

    // ---- measurement-chain model -------------------------------------------
    // Latches the configuration at commit and answers exactly one window later.
    reg [31:0] cfg_fcw   = 32'd0;
    reg [4:0]  cfg_integ = 5'd8;
    integer    win_left  = 0;
    integer    win_len   = 0;

    function real sinc_r;
        input real u;
        begin
            if (u < 1.0e-9 && u > -1.0e-9) sinc_r = 1.0;
            else sinc_r = $sin(3.14159265358979323 * u) / (3.14159265358979323 * u);
        end
    endfunction
    // Hann: G(x) = sinc(x) + 0.5*sinc(x-1) + 0.5*sinc(x+1)
    function real hann_gain;
        input real x;
        begin
            hann_gain = sinc_r(x) + 0.5 * sinc_r(x - 1.0) + 0.5 * sinc_r(x + 1.0);
        end
    endfunction

    real t_win, df_hz, gain, amp_r;
    always @(posedge clk) begin
        meas_valid <= 1'b0;
        if (!rst_n) begin
            win_left <= 0;
        end else if (commit) begin
            cfg_fcw   <= active_fcw;
            cfg_integ <= active_integ;
            win_len   <= (1 << (active_integ + 4)) + 64;   // N*R clocks + slack
            win_left  <= (1 << (active_integ + 4)) + 64;
        end else if (win_left > 0) begin
            win_left <= win_left - 1;
            if (win_left == 1) begin
                // one window finished: evaluate the Hann response
                t_win  = (2.0 ** cfg_integ) * 16.0 / 65.0e6;
                df_hz  = (real'(true_fcw) - real'(cfg_fcw)) * 65.0e6 / 4294967296.0;
                gain   = hann_gain(df_hz * t_win);
                if (gain < 0.0) gain = -gain;
                amp_r  = real'(amp_true) * gain;
                meas_amp0 <= amp_r;
                meas_amp1 <= amp_r;
                meas_valid <= 1'b1;
            end
        end
    end

    // ---- run control -------------------------------------------------------
    integer cycles;
    real    found_hz, err_hz;
    initial begin
        $display("--- tb_freq_scan: F_HZ=%0d AMP=%0d ---", true_hz, amp_true);
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        enable = 1'b1;

        // coarse 2160 pts x ~4160 clks + fine 33 pts x ~65600 clks + margin
        for (cycles = 0; cycles < 40000000; cycles = cycles + 1) begin
            @(posedge clk);
            if (locked && state == 3'd3) begin
                // let a few more windows go by so 'lost' logic is exercised
                repeat (200000) @(posedge clk);
                found_hz = real'(best_fcw) * 65.0e6 / 4294967296.0;
                err_hz   = found_hz - real'(true_hz);
                if (err_hz < 0) err_hz = -err_hz;
                $display("LOCKED best_fcw=%0d (%0.1f Hz) best_amp=%0d passes=%0d",
                         best_fcw, found_hz, best_amp, pass_count);
                $display("state=%0d busy=%0b locked=%0b integ=%0d", state, busy, locked, active_integ);
                if (amp_true == 0) begin
                    $display("FATAL: locked with no signal present");
                    $finish;
                end
                // The fine sweep steps 900 Hz, so the worst-case residual is
                // half a step; allow the fcw quantisation on top.
                if (err_hz > 600.0) begin
                    $display("FATAL: frequency error %0.1f Hz exceeds 600 Hz", err_hz);
                    $finish;
                end
                if (best_amp < 24'd2048) begin
                    $display("FATAL: locked below the detection floor");
                    $finish;
                end
                if (active_integ != 5'd12) begin
                    $display("FATAL: locked window is not the 4096-point window");
                    $finish;
                end
                // A carrier must stay above the floor after locking.
                if (best_amp < (amp_true / 2)) begin
                    $display("FATAL: locked amplitude %0d far below true %0d", best_amp, amp_true);
                    $finish;
                end
                $display("PASS tb_freq_scan F_HZ=%0d -> %0.1f Hz (err %0.1f Hz)",
                         true_hz, found_hz, err_hz);
                $finish;
            end
            if (amp_true == 0 && pass_count >= 2) begin
                if (locked) begin
                    $display("FATAL: locked with no signal present");
                    $finish;
                end
                $display("PASS tb_freq_scan no-signal: stays unlocked after %0d passes", pass_count);
                $finish;
            end
        end
        $display("FATAL: no lock within %0d cycles (state=%0d count=%0d)", cycles, state, scan_count);
        $finish;
    end
endmodule
