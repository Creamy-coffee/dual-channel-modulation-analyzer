`timescale 1ns/1ps
`default_nettype none
// ---------------------------------------------------------------------------
// Block-level diagnostic: what does the lock-in actually do with a given
// cfg_fcw, and what does it report as active_fcw?
//
// Written to settle one question the top-level sweep test could not: with the
// sweep parked on fcw F, the reported amplitude was ~29 instead of ~3.2e6 and
// active_fcw was not F.  This test removes the sweep, the top level and the
// AFC from the picture and drives lockin_measurement directly.
//
//   +FCW=<n>  frequency word to commit
//   +HZ=<n>   stimulus frequency (default 1000000)
// ---------------------------------------------------------------------------
module tb_diag_fcw;
    localparam real PI = 3.14159265358979;
    reg clk = 0;
    always #7.69 clk = ~clk;          // ~65 MHz

    integer stim_hz = 1000000;
    reg [31:0] use_fcw = 32'd66083010;
    initial begin
        if (!$value$plusargs("HZ=%d",  stim_hz)) stim_hz = 1000000;
        if (!$value$plusargs("FCW=%d", use_fcw)) use_fcw = 32'd66083010;
    end

    reg rst_n = 1'b0;
    reg adc_valid = 1'b1;
    reg signed [11:0] x0 = 0, x1 = 0;
    reg commit = 1'b0;

    wire [31:0] active_fcw;
    wire [23:0] amp0, amp1;
    wire        lockin_valid;
    wire [1:0]  detected, qualified;
    wire [7:0]  flags;

    lockin_measurement #(.CLK_FREQ(65000000), .AFC_ENABLE(0)) u (
        .clk(clk), .rst_n(rst_n), .adc_valid(adc_valid),
        .adc_ch0(x0), .adc_ch1(x1),
        .cfg_commit(commit),.cfg_reconfig(1'b0), .cfg_enable(1'b1),
        .cfg_fcw(use_fcw), .cfg_decim_sel(2'd0), .cfg_integration_log2(5'd12),
        .disp_fcw(use_fcw), .disp_decim(2'd0), .disp_integration(5'd12),
        .lockin_valid(lockin_valid), .amp0_q12(amp0), .amp1_q12(amp1),
        .detected(detected), .qualified(qualified), .flags(flags),
        .active_fcw(active_fcw));

    // exact 65-sample period table for 1 MHz @ 65 MS/s
    reg signed [11:0] t0 [0:64];
    integer ti; real a0;
    initial for (ti = 0; ti < 65; ti = ti + 1) begin
        a0 = 800.0 * $cos(2.0 * PI * ti / 65.0);
        t0[ti] = $rtoi(a0 + (a0 >= 0 ? 0.5 : -0.5));
    end
    integer k = 0;
    always @(posedge clk) begin
        k <= (k == 64) ? 0 : k + 1;
        x0 <= t0[k];
        x1 <= t0[(k + 16) % 65];       // 90 deg apart, approximately
    end

    integer wins = 0;
    always @(posedge clk) if (rst_n && lockin_valid) wins <= wins + 1;

    integer commit_seen_core = 0, commit_seen_meas = 0;
    always @(posedge clk) begin
        if (u.cfg_commit)      commit_seen_meas <= commit_seen_meas + 1;
        if (u.u_lockin.cfg_commit) commit_seen_core <= commit_seen_core + 1;
    end

    initial begin
        $display("-- tb_diag_fcw: FCW=%0d (%0.1f Hz)  stimulus=%0d Hz --",
                 use_fcw, real'(use_fcw) * 65.0e6 / 4294967296.0, stim_hz);
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);
        // Drive the commit pulse on the FALLING edge.  Changing a sampled
        // signal right at the rising edge races the always block and the pulse
        // can be missed entirely, which silently leaves the NCO at its reset
        // frequency word and makes the whole sweep look broken.
        @(negedge clk); commit = 1'b1;
        @(negedge clk); commit = 1'b0;
        while (wins < 4) @(posedge clk);
        $display("active_fcw          : %0d (%0.1f Hz)",
                 active_fcw, real'(active_fcw) * 65.0e6 / 4294967296.0);
        $display("core fcw / phase    : %0d / %0d", u.u_lockin.fcw, u.u_lockin.phase);
        $display("core cfg_ok/enabled : %0b / %0b", u.u_lockin.cfg_ok, u.u_lockin.enabled);
        $display("core running/decim  : %0b / %0d", u.u_lockin.running, u.u_lockin.decim_log2);
        $display("core integ/window   : %0d", u.u_lockin.integration_log2);
        $display("commit seen meas/cor: %0d / %0d", commit_seen_meas, commit_seen_core);
        $display("cfg_fcw             : %0d (%0.1f Hz)",
                 use_fcw, real'(use_fcw) * 65.0e6 / 4294967296.0);
        $display("amp0 / amp1 (Q12)   : %0d / %0d", amp0, amp1);
        $display("detected / qualified: %0d / %0d   flags=%0d", detected, qualified, flags);
        $display("windows seen        : %0d", wins);
        $finish;
    end
endmodule
`default_nettype wire
