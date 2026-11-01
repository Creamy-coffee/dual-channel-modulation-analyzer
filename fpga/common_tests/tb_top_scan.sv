`timescale 1ns/1ps
// ---------------------------------------------------------------------------
// Top-level test for the ONE-BITSTREAM full-band acquisition.
//
// WHY THIS EXISTS
//   freq_scan_ctrl now owns cfg_fcw and auto_rate_ctrl owns cfg_commit /
//   cfg_decim_sel.  Both are new owners of signals that used to be constants,
//   and a wiring mistake there is invisible to every block-level testbench.
//   This test drives the real daq_top with a real tone and answers:
//     (a) does the board still transmit (uart_tx alive)?
//     (b) does the sweep actually run and park on the applied tone?
//     (c) does the decimation gear follow the frequency it found?
//     (d) does the lock-in produce a sensible amplitude once parked?
//
//   The sweep upper bound is narrowed with defparam so the run finishes in a
//   few million clocks; the full band is covered by tests/tb_freq_scan.sv,
//   which sweeps the real 200 Hz .. 32.4 MHz range against a Hann-response
//   model.  Step size and window length are NOT changed here.
//
// USAGE
//   iverilog -g2012 -DSIMULATION -s tb_top_scan -o tb_top_scan.vvp \
//       tb_top_scan.sv sim_prims.v ../rtl/*.v -DSTIM_HZ=1000000
// ---------------------------------------------------------------------------
module tb_top_scan;

`ifndef STIM_HZ
`define STIM_HZ 1000000
`endif
    parameter integer STIM_HZ = `STIM_HZ;
    // Sweep ceiling used only to shorten this run: 2x the stimulus keeps the
    // real step size and window length while cutting the point count.
    parameter [31:0] SWEEP_MAX_FCW = (STIM_HZ * 2) * 66;

    parameter integer MAXCYC     = 12000000;
    parameter integer TX_TARGET  = 3000;

    reg        clk_50m   = 1'b0;
    always #10 clk_50m = ~clk_50m;

    reg        rst_n     = 1'b0;
    reg [11:0] adc_d0    = 12'd2048;
    reg [11:0] adc_d1    = 12'd2048;
    reg        adc_otr   = 1'b0;
    reg        adc_otr_b = 1'b0;
    reg [3:0]  digital_in = 4'h0;
    reg [3:0]  key_n      = 4'hF;      // active low: nothing pressed
    reg [2:0]  sw         = 3'b000;

    wire       uart_tx;
    wire [3:0] led;

`ifndef AFC_ENABLE_DEFAULT
`define AFC_ENABLE_DEFAULT 1
`endif
    daq_top #(
        .STARTUP_CYCLES(200),
        .REF_FREQ_HZ(STIM_HZ),
        .AFC_ENABLE(`AFC_ENABLE_DEFAULT),
        .SCAN_ENABLE(1),
        .AUTO_RATE_EN(1)
    ) dut (
        .clk_50m(clk_50m), .rst_n(rst_n),
        .adc_d1(adc_d1), .adc_d0(adc_d0),
        .adc_otr_b(adc_otr_b), .adc_otr(adc_otr),
        .adc_clk_b(), .adc_oeb_b(), .adc_pwdn_b(),
        .adc_clk(),   .adc_oeb(),   .adc_pwdn(),
        .digital_in(digital_in), .key_n(key_n), .sw(sw),
        .led(led), .uart_tx(uart_tx)
    );
    defparam dut.u_scan.F_MAX_FCW = SWEEP_MAX_FCW;
    // Do NOT cap the bandwidth gear here.  MAX_GEAR is an integer parameter
    // read as MAX_GEAR[1:0]; overriding it with a 2-bit defparam (2'd1) is
    // illegal in this tool flow and silently stopped the whole top-level
    // simulation from running at all.  The full 64.5 ms gear simply does not
    // settle inside one simulated run, which only means this test must not
    // require it: tb_meas_bw_ctrl proves the narrowing ladder and the gear
    // table proves the window lengths.  What this test has to prove is that
    // the controller is WIRED to cfg_decim_sel and that the parked NCO is
    // right, which gear 0/1 already show.

    // ---- stimulus: two channels of the same tone, 90 degrees apart ----------
    real step;
    integer n;
    initial begin
        step = 2.0 * 3.141592653589793 * STIM_HZ / 65.0e6;
        n    = 0;
    end
    always @(posedge dut.clk_sys) begin
        n = n + 1;
        adc_d0 <= 12'd2048 + $rtoi(800.0 * $cos(step * n));
        adc_d1 <= 12'd2048 + $rtoi(800.0 * $cos(step * n - 3.141592653589793 / 2.0));
    end

    // ---- observation --------------------------------------------------------
    integer tx_edges = 0;
    integer cyc      = 0;
    integer locked_at = -1;
    integer afc_off  = 0;
    reg     last_tx  = 1'b1;
    real    found_hz, err_hz;

    always @(posedge dut.clk_sys) begin
        cyc <= cyc + 1;
        if (uart_tx !== last_tx) begin tx_edges <= tx_edges + 1; last_tx <= uart_tx; end
        if (locked_at < 0 && dut.scan_locked) locked_at <= cyc;
    end

    initial begin
        $display("-- tb_top_scan: STIM_HZ=%0d SCAN_ENABLE=1 AFC_ENABLE=1 --", STIM_HZ);
        #2000;
        rst_n = 1'b1;

        while (cyc < MAXCYC && (locked_at < 0 || cyc < locked_at + 3000000)) begin
            @(posedge dut.clk_sys);
            if (tx_edges >= TX_TARGET && locked_at >= 0 && cyc > locked_at + 2000000) begin
                cyc <= MAXCYC;   // both goals met, stop early
            end
        end

        found_hz = real'(dut.scan_best_fcw) * 65.0e6 / 4294967296.0;
        err_hz   = found_hz - real'(STIM_HZ);
        if (err_hz < 0) err_hz = -err_hz;

        $display("cycles simulated     : %0d", cyc);
        $display("uart_tx edges        : %0d  (target %0d)", tx_edges, TX_TARGET);
        $display("scan state           : %0d  locked=%0b  (3/locked = parked)",
                 dut.scan_state, dut.scan_locked);
        $display("locked at cycle      : %0d", locked_at);
        $display("found frequency      : %0.1f Hz  (applied %0d, err %0.1f Hz)",
                 found_hz, STIM_HZ, err_hz);
        $display("active_fcw / li_fcw  : %0d / %0d", dut.scan_fcw, dut.li_active_fcw);
        $display("bw gear / decim_sel  : %0d / %0d  integ=%0d  settling=%0b  changes=%0d",
                 dut.bw_gear, dut.cfg_decim_sel_eff, dut.cfg_integ_eff,
                 dut.gear_settling, dut.bw_changes);
        $display("lock-in amp0/amp1 Q12: %0d / %0d", dut.li_amp0, dut.li_amp1);
        $display("afc state/ready/reas : %0d / %0b / %0d  updates=%0d",
                 dut.afc_state, dut.afc_ready, dut.afc_reason, dut.afc_updates);
        $display("afc residual0/1 (Hz) : %0.1f / %0.1f",
                 real'(dut.residual0) * 65.0e6 / 4294967296.0,
                 real'(dut.residual1) * 65.0e6 / 4294967296.0);
        $display("sweep passes/points  : %0d / %0d", dut.scan_passes, dut.scan_points);

        if (tx_edges < TX_TARGET) begin
            $display("TB_FAIL tb_top_scan: uart_tx only %0d edges", tx_edges);
            $finish;
        end
        if (locked_at < 0) begin
            $display("TB_FAIL tb_top_scan: sweep never locked");
            $finish;
        end
        if (dut.scan_state != 3) begin
            $display("TB_FAIL tb_top_scan: scan state %0d != 3", dut.scan_state);
            $finish;
        end
        if (err_hz > 1500.0) begin
            $display("TB_FAIL tb_top_scan: found %0.1f Hz for a %0d Hz tone (err %0.1f)",
                     found_hz, STIM_HZ, err_hz);
            $finish;
        end
        if (dut.li_amp0 == 0) begin
            $display("TB_FAIL tb_top_scan: parked but amplitude is zero");
            $finish;
        end
        // The NCO must actually sit on the frequency the sweep picked.  A
        // dropped final commit leaves it on the last probe: amp is then ~1e5
        // times too small while every sweep status flag still says "locked".
        // AFC is EXPECTED to microtune the NCO away from the sweep's coarse
        // pick -- that is the division of labour (coarse sweep -> AFC fine
        // track).  What must hold is that it stays inside the +/-2 kHz capture
        // range; drifting outside means the sweep parked somewhere wrong and
        // AFC is chasing an alias.
        afc_off = (dut.li_active_fcw > dut.scan_fcw)
                ? (dut.li_active_fcw - dut.scan_fcw)
                : (dut.scan_fcw - dut.li_active_fcw);
        if (afc_off > 132153) begin
            $display("TB_FAIL tb_top_scan: NCO %0d drifted %0d FCW (%0.1f Hz) from the "
                     , dut.li_active_fcw, afc_off, real'(afc_off) * 65.0e6 / 4294967296.0);
            $display("  parked sweep frequency %0d; outside the +/-2 kHz AFC range",
                     dut.scan_fcw);
            $finish;
        end
        // 800-count stimulus => ~3.2e6 Q12 once parked; 1e5 catches a detune.
        if (dut.li_amp0 < 100000) begin
            $display("TB_FAIL tb_top_scan: parked amplitude %0d is far below the "
                     , dut.li_amp0);
            $display("  ~3.2e6 expected for an 800-count tone (NCO detuned?)");
            $finish;
        end
        // The bandwidth controller must actually own cfg_decim_sel.  A constant
        // tone parked for many windows has to leave the fast gear, otherwise
        // the new controller is wired to nothing.
        if (dut.bw_changes == 0) begin
            $display("TB_FAIL tb_top_scan: bandwidth gear never moved (bw_changes=0)");
            $finish;
        end
        if (dut.cfg_decim_sel_eff != dut.bw_gear) begin
            $display("TB_FAIL tb_top_scan: cfg_decim_sel %0d does not follow bw_gear %0d",
                     dut.cfg_decim_sel_eff, dut.bw_gear);
            $finish;
        end
        $display("TB_PASS tb_top_scan");
        $finish;
    end
endmodule
