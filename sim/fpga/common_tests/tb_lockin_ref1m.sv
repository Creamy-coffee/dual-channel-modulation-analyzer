`timescale 1ns/1ps
`default_nettype none
// tb_lockin_ref1m: 1 MHz reference lock-in functional verification.
//
// Mirrors tb_lockin_ref30m but at the 1 MHz reference, which is the
// configuration of release/raw_scope_v4_merged_ref1m.bit used for the
// 1 MHz / 100 mVpp dual-channel RAW measurement step.
//
// cfg_fcw = FCW(1 MHz) = 66076419 (= 999999.9858 Hz), decim_sel=0 (R16 after
// mixing), integration_log2=12 (4096 pts, ~1.008 ms at 65 MS/s).
// 1 MHz at 65 MS/s is exactly 65 samples per period, so the stimulus tables
// are exact (no fractional-period truncation).
//
// Four phases:
//   A: 1 MHz coherent two-tone (600 cts @ -60 deg, 300 cts @ +30 deg)
//      -> detected/qualified=1, amp ~= A*4096, pdiff ~= +90 deg (16384 LSB)
//   B: zero input -> detected/qualified must clear, amp < guard
//   C: reference mismatch (fcw = 3 MHz, input still 1 MHz) -> amp collapses,
//      detected/qualified must clear (no stale valid result)
//   D: re-commit FCW1M -> detection must RECOVER on the same stream
//
// Phase convention (MEASURED, do not assume): the absolute phase carries a
// fixed offset from the NCO phase origin and window alignment. With the
// -60 deg / +30 deg stimulus above the core reports ph0 = -2812 LSB
// (-15.4 deg) and ph1 = +13572 LSB (+74.6 deg) -- i.e. input+44.6 deg, not
// the input phase itself. The DIFFERENCE is exact:
//     pdiff = ph1 - ph0 = 16384 LSB = +90.000 deg
// so this testbench asserts pdiff (and the 30 MHz one likewise) and only
// prints the absolute phases. Any future absolute-phase claim needs its own
// calibration against a known reference path.
module tb_lockin_ref1m;
    localparam FCW1M = 32'd66076419;    // 1 MHz @ 65 MS/s (999999.9858 Hz)
    localparam FCW3M = 32'd198229259;   // 3 MHz: the mismatch case
    localparam real  DEG2LSB = 65536.0/360.0;
    localparam real  PI = 3.14159265358979;
    localparam integer NS = 65;         // 65 samples per period at 1 MHz

    reg clk=0, rst_n=0;
    always #7.69 clk = ~clk;   // ~65 MHz; exact period irrelevant (sample-based)

    reg adc_valid = 1'b1;
    reg signed [11:0] x0=0, x1=0;

    // Exact 65-sample periodic tables for f/fs = 1/65 (1 MHz @ 65 MS/s)
    reg signed [11:0] t0 [0:64];
    reg signed [11:0] t1 [0:64];
    integer ti;
    real a0, a1;
    initial begin
        for (ti = 0; ti < NS; ti = ti + 1) begin
            a0 = 600.0*$cos(2.0*PI*ti/NS - 60.0*PI/180.0);
            a1 = 300.0*$cos(2.0*PI*ti/NS + 30.0*PI/180.0);
            t0[ti] = $rtoi(a0 + (a0 >= 0 ? 0.5 : -0.5));
            t1[ti] = $rtoi(a1 + (a1 >= 0 ? 0.5 : -0.5));
        end
    end

    reg [31:0] k=0;
    // drive: 0=zero input, 1=1 MHz tones
    reg drive_tones = 1'b0;
    always @(posedge clk) begin
        if (drive_tones) begin
            x0 <= t0[k % NS];
            x1 <= t1[k % NS];
        end else begin
            x0 <= 12'sd0;
            x1 <= 12'sd0;
        end
        k <= k + 1;
    end

    // configuration
    reg cfg_commit=0, cfg_enable=0;
    reg [31:0] cfg_fcw = FCW1M;
    reg [1:0]  cfg_decim_sel = 2'd0;
    reg [4:0]  cfg_integration_log2 = 5'd12;

    wire lockin_valid, result_present, ratio_valid;
    wire [23:0] amp0_q12, amp1_q12;
    wire signed [15:0] phase0_turn, phase1_turn, phase_diff_turn;
    wire [31:0] ratio_q16, flags, gap_count, drop_count, result_id, result_age;
    wire [1:0] detected, qualified;
    wire [31:0] active_fcw;

    lockin_measurement #(
        .CLK_FREQ(32'd65000000),
        .MIN_AMPLITUDE_Q12(24'd2048)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .adc_valid(adc_valid), .adc_ch0(x0), .adc_ch1(x1),
        .cfg_commit(cfg_commit),.cfg_reconfig(1'b0), .cfg_enable(cfg_enable),
        .cfg_fcw(cfg_fcw), .cfg_decim_sel(cfg_decim_sel),
        .cfg_integration_log2(cfg_integration_log2),
        .disp_fcw(cfg_fcw), .disp_decim(cfg_decim_sel),
        .disp_integration(cfg_integration_log2),
        .lockin_valid(lockin_valid),
        .amp0_q12(amp0_q12), .amp1_q12(amp1_q12),
        .phase0_turn(phase0_turn), .phase1_turn(phase1_turn),
        .phase_diff_turn(phase_diff_turn),
        .ratio_q16(ratio_q16),
        .flags(flags), .gap_count(gap_count), .drop_count(drop_count),
        .result_present(result_present), .ratio_valid(ratio_valid),
        .result_id(result_id), .result_age(result_age),
        .detected(detected), .qualified(qualified),
        .active_fcw(active_fcw)
    );

    // capture each published result
    reg [23:0] cap_amp0, cap_amp1;
    reg signed [15:0] cap_ph0, cap_ph1, cap_pdiff;
    reg [1:0] cap_det, cap_qual;
    event result_ev;

    always @(posedge clk) begin
        if (lockin_valid) begin
            cap_amp0  <= amp0_q12;
            cap_amp1  <= amp1_q12;
            cap_ph0   <= phase0_turn;
            cap_ph1   <= phase1_turn;
            cap_pdiff <= phase_diff_turn;
            cap_det   <= detected;
            cap_qual  <= qualified;
            ->result_ev;
        end
    end

    // watchdog
    initial begin
        #80_000_000;
        $display("TB_ERROR watchdog timeout");
        $finish;
    end

    integer errors = 0;
    reg [15:0] exp_pdiff;

    task automatic wait_results(input integer n);
        integer got;
        begin
            got = 0;
            while (got < n) begin
                @(result_ev);
                got = got + 1;
            end
        end
    endtask

    task automatic recommit(input [31:0] fcw);
        begin
            @(posedge clk);
            cfg_fcw <= fcw;
            cfg_commit <= 1'b1;
            @(posedge clk);
            cfg_commit <= 1'b0;
        end
    endtask

    function integer wrap16;   // wrap phase difference to (-32768, 32767]
        input [15:0] d;
        begin
            wrap16 = $signed(d);
        end
    endfunction

    initial begin
        #100;
        rst_n = 1;
        @(posedge clk);
        cfg_enable = 1'b1;
        cfg_commit = 1'b1;
        @(posedge clk);
        cfg_commit = 1'b0;

        // ---------------- Phase A: 1 MHz two-tone ----------------
        drive_tones = 1'b1;
        wait_results(15);       // guard: warmup(8) + good_count>=2 + 1 settle window
        $display("A: amp0=%0d amp1=%0d ph0=%0d ph1=%0d pdiff=%0d det=%b qual=%b ratio_q16=%0d fcw=%0d",
                 cap_amp0, cap_amp1, cap_ph0, cap_ph1, cap_pdiff, cap_det, cap_qual, ratio_q16, active_fcw);
        if (active_fcw !== FCW1M) begin errors=errors+1; $display("TB_ERROR A: active_fcw != FCW1M"); end
        if (cap_det !== 2'b11 || cap_qual !== 2'b11) begin errors=errors+1; $display("TB_ERROR A: det/qual not both valid"); end
        // amp0 ~ 600*4096 = 2457600, amp1 ~ 300*4096 = 1228800 (3% tolerance)
        if (cap_amp0 < 582*4096 || cap_amp0 > 618*4096) begin errors=errors+1; $display("TB_ERROR A: amp0 out of range (got %0d)", cap_amp0); end
        if (cap_amp1 < 291*4096 || cap_amp1 > 309*4096) begin errors=errors+1; $display("TB_ERROR A: amp1 out of range (got %0d)", cap_amp1); end
        // pdiff = +90 deg -> +16384 LSB (allow CORDIC quantization +/- 64 LSB ~ 0.35 deg)
        exp_pdiff = 16'h4000;
        if (wrap16(cap_pdiff - exp_pdiff) > 64 || wrap16(cap_pdiff - exp_pdiff) < -64)
            begin errors=errors+1; $display("TB_ERROR A: pdiff != +90deg (got %0d LSB)", cap_pdiff); end
        $display("A(convention): ph0-LSB=%0d expect~%0d ; ph1-LSB=%0d expect~%0d",
                 cap_ph0, $rtoi((-60.0)*DEG2LSB), cap_ph1, $rtoi((30.0)*DEG2LSB));

        // ---------------- Phase B: zero input ----------------
        drive_tones = 1'b0;
        wait_results(3);
        $display("B: amp0=%0d amp1=%0d det=%b qual=%b", cap_amp0, cap_amp1, cap_det, cap_qual);
        if (cap_det !== 2'b00 || cap_qual !== 2'b00) begin errors=errors+1; $display("TB_ERROR B: det/qual must clear on zero input"); end
        if (cap_amp0 >= 24'd2048) begin errors=errors+1; $display("TB_ERROR B: amp0 above guard with zero input"); end

        // ---------------- Phase C: reference mismatch (fcw=3MHz, input 1MHz) ----------------
        drive_tones = 1'b1;
        recommit(FCW3M);
        wait_results(4);
        $display("C: amp0=%0d amp1=%0d det=%b qual=%b", cap_amp0, cap_amp1, cap_det, cap_qual);
        if (active_fcw !== FCW3M) begin errors=errors+1; $display("TB_ERROR C: active_fcw != FCW3M"); end
        if (cap_det !== 2'b00 || cap_qual !== 2'b00) begin errors=errors+1; $display("TB_ERROR C: det/qual must clear on reference mismatch"); end

        // ---------------- Phase D: recover with FCW1M ----------------
        recommit(FCW1M);
        wait_results(15);
        $display("D: amp0=%0d amp1=%0d det=%b qual=%b pdiff=%0d", cap_amp0, cap_amp1, cap_det, cap_qual, cap_pdiff);
        if (active_fcw !== FCW1M) begin errors=errors+1; $display("TB_ERROR D: active_fcw != FCW1M"); end
        if (cap_det !== 2'b11 || cap_qual !== 2'b11) begin errors=errors+1; $display("TB_ERROR D: det/qual must recover after re-commit"); end
        if (wrap16(cap_pdiff - 16'h4000) > 64 || wrap16(cap_pdiff - 16'h4000) < -64)
            begin errors=errors+1; $display("TB_ERROR D: pdiff != +90deg after recovery"); end

        if (gap_count != 0 || drop_count != 0) begin errors=errors+1; $display("TB_ERROR: gap/drop nonzero: %0d/%0d", gap_count, drop_count); end

        $display("TB_DONE errors=%0d", errors);
        if (errors == 0) $display("TB_PASS tb_lockin_ref1m");
        else             $display("TB_FAIL tb_lockin_ref1m");
        $finish;
    end
endmodule
`default_nettype wire
