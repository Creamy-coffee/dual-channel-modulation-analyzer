`timescale 1ns/1ps
// ---------------------------------------------------------------------------
// Auto decimation-gear controller testbench.
//
// The gear thresholds are pure FCW comparisons, so the integration window used
// in simulation is shortened (INTEGRATION_LOG2=8, SETTLE_WINDOWS=2) ONLY to
// keep the settle counters short.  That does not touch the logic under test:
// gear selection depends on ref_fcw against G1/G2/G3 alone.
//
// The decisive assertion is scenario 1: 30 MHz and 1 MHz must stay on the
// FASTEST gear.  Automatic rate adaption must never cost high-frequency
// capability -- that was the whole objection to "just slow the clock down".
// ---------------------------------------------------------------------------
module tb_auto_rate_ctrl;

    reg clk = 0;
    reg rst_n = 0;
    reg enable = 1;
    reg [31:0] ref_fcw = 0;
    reg ref_valid = 0;

    wire [1:0] decim_sel;
    wire [4:0] integration_log2;
    wire commit, settling, valid;
    wire [1:0] gear;
    wire [15:0] change_count;
    wire [31:0] settle_ticks;

    // FCW = f * 2^32 / 65e6
    localparam FCW_100     = 32'd6608;
    localparam FCW_500     = 32'd33038;
    localparam FCW_5K      = 32'd330382;
    localparam FCW_50K     = 32'd3303821;
    localparam FCW_90K     = 32'd5946878;
    localparam FCW_99K     = 32'd6541566;
    localparam FCW_100K    = 32'd6607642;
    localparam FCW_105K    = 32'd6938024;
    localparam FCW_1M      = 32'd66076420;
    localparam FCW_30M     = 32'd1982292598;

    auto_rate_ctrl #(
        .INTEGRATION_LOG2(8), .SETTLE_WINDOWS(2)
    ) dut (
        .clk(clk), .rst_n(rst_n), .enable(enable),
        .ref_fcw(ref_fcw), .ref_valid(ref_valid),
        .decim_sel(decim_sel), .integration_log2(integration_log2),
        .commit(commit), .settling(settling), .valid(valid),
        .gear(gear), .change_count(change_count), .settle_ticks(settle_ticks)
    );

    always #5 clk = ~clk;

    integer errors = 0;
    integer commits_seen = 0;
    // iverilog does not accept declarations inside named begin blocks, so the
    // scratch variables live here instead.
    integer s_before = 0;
    integer p_c0 = 0;
    integer p_w = 0;

    // commit pulse width must be exactly one clock; sampling on the clock edge
    // from the testbench would race with the DUT's non-blocking update, so the
    // width is measured here instead.
    reg prev_commit = 1'b0;
    integer multi_commit = 0;
    always @(posedge clk) if (commit) commits_seen <= commits_seen + 1;
    always @(posedge clk) begin
        prev_commit <= commit;
        if (commit && prev_commit) multi_commit <= multi_commit + 1;
    end

    task set_ref;
        input [31:0] f;
        begin
            ref_fcw = f;
            ref_valid = 1'b1;
        end
    endtask

    task wait_settle;
        input [31:0] max_cycles;
        integer n;
        begin
            n = 0;
            while (settling && n < max_cycles) begin
                @(posedge clk);
                n = n + 1;
            end
            if (settling) begin
                $display("  FAIL: settle timeout after %0d cycles", n);
                errors = errors + 1;
            end
        end
    endtask

    task expect_gear;
        input [1:0] want;
        input [8*32:1] name;
        begin
            if (gear !== want) begin
                $display("  FAIL %s: gear=%0d expected %0d", name, gear, want);
                errors = errors + 1;
            end else begin
                $display("  pass %s: gear=%0d (R=%0d)", name, gear,
                         16 << (2 * gear));
            end
        end
    endtask

    // Apply a new reference and let the gear settle, then check.
    task apply_and_check;
        input [31:0] f;
        input [1:0]  want;
        input [8*32:1] name;
        begin
            set_ref(f);
            repeat (3) @(posedge clk);
            wait_settle(2000000);
            repeat (2) @(posedge clk);
            expect_gear(want, name);
        end
    endtask

    initial begin
        $display("=== tb_auto_rate_ctrl ===");

        // ---- 0. boot -------------------------------------------------------
        rst_n = 0;
        repeat (4) @(posedge clk);
        rst_n = 1;
        set_ref(FCW_1M);
        @(posedge clk);      // DUT sees rst_n=1 here and issues the boot commit
        @(negedge clk);      // ... visible only after the NBA update
        if (commit !== 1'b1) begin
            $display("  FAIL boot: no commit pulse after reset");
            errors = errors + 1;
        end else begin
            $display("  pass boot: commit pulse issued");
        end
        if (settling !== 1'b1 || valid !== 1'b0) begin
            $display("  FAIL boot: settling=%0b valid=%0b at boot", settling, valid);
            errors = errors + 1;
        end
        wait_settle(2000000);
        repeat (2) @(posedge clk);
        if (valid !== 1'b1) begin
            $display("  FAIL boot: valid not raised after settle");
            errors = errors + 1;
        end else begin
            $display("  pass boot: valid raised after settle");
        end
        if (integration_log2 !== 5'd8) begin
            $display("  FAIL boot: integration_log2=%0d expected 8", integration_log2);
            errors = errors + 1;
        end
        if (decim_sel !== gear) begin
            $display("  FAIL: decim_sel != gear");
            errors = errors + 1;
        end

        // ---- 1. high frequency must NEVER leave the fastest gear -----------
        $display("-- high frequency (the whole point of the rule) --");
        apply_and_check(FCW_30M, 2'd0, "30 MHz");
        apply_and_check(FCW_1M, 2'd0, "1 MHz");

        // ---- 2. descending gears -------------------------------------------
        $display("-- descending --");
        apply_and_check(FCW_100K, 2'd0, "100 kHz boundary");
        apply_and_check(FCW_90K, 2'd1, "90 kHz -> R64");
        apply_and_check(FCW_5K, 2'd2, "5 kHz -> R256");
        apply_and_check(FCW_500, 2'd3, "500 Hz -> R1024");
        apply_and_check(FCW_100, 2'd3, "100 Hz stays R1024");

        // ---- 3. hysteresis: no toggle inside the band ----------------------
        $display("-- hysteresis --");
        // gear1 -> gear0 only above G1+h1 (6710886); 99 kHz is inside the band
        apply_and_check(FCW_99K, 2'd1, "99 kHz inside band, stays R64");
        apply_and_check(FCW_105K, 2'd0, "105 kHz -> back to R16");
        // gear0 -> gear1 only below G1-h1 (6504398); 99 kHz is inside the band
        apply_and_check(FCW_99K, 2'd0, "99 kHz inside band, stays R16");
        apply_and_check(FCW_90K, 2'd1, "90 kHz -> R64 again");

        // ---- 4. settling must invalidate, and block re-gearing -------------
        $display("-- settling behaviour --");
        begin : settling_check
            s_before = change_count;
            set_ref(FCW_5K);
            @(posedge clk);
            @(negedge clk);
            if (settling !== 1'b1) begin
                $display("  FAIL: settling not raised on gear change");
                errors = errors + 1;
            end
            if (valid !== 1'b0) begin
                $display("  FAIL: valid still high while settling");
                errors = errors + 1;
            end
            // a second reference change while settling must not re-commit
            set_ref(FCW_500);
            repeat (5) @(posedge clk);
            if (change_count !== s_before + 1) begin
                $display("  FAIL: re-geared while settling (count=%0d)", change_count);
                errors = errors + 1;
            end else begin
                $display("  pass settling: no re-gear during settle");
            end
            wait_settle(2000000);
            repeat (2) @(posedge clk);
            expect_gear(2'd3, "after settle, follows latest ref");
        end

        // ---- 5. ref_valid gate ---------------------------------------------
        $display("-- ref_valid gate --");
        ref_valid = 1'b0;
        set_ref(FCW_30M);
        repeat (20) @(posedge clk);
        if (valid !== 1'b0) begin
            $display("  FAIL: valid high while ref_valid=0");
            errors = errors + 1;
        end else begin
            $display("  pass ref_valid=0 holds valid low");
        end
        ref_valid = 1'b1;
        repeat (2) @(posedge clk);
        wait_settle(2000000);
        repeat (2) @(posedge clk);
        expect_gear(2'd0, "30 MHz back to R16");

        // ---- 6. commit pulse width -----------------------------------------
        $display("-- commit pulse --");
        set_ref(FCW_500);
        wait_settle(2000000);
        repeat (4) @(posedge clk);
        if (multi_commit != 0) begin
            $display("  FAIL: commit stayed high for >1 clock (%0d times)",
                     multi_commit);
            errors = errors + 1;
        end else begin
            $display("  pass commit: never wider than one clock");
        end
        if (commits_seen < 3) begin
            $display("  FAIL: only %0d commits seen, expected several", commits_seen);
            errors = errors + 1;
        end else begin
            $display("  pass commit: %0d commits observed", commits_seen);
        end

        repeat (5) @(posedge clk);
        if (errors == 0) begin
            $display("TB_PASS tb_auto_rate_ctrl (%0d gear changes)", change_count);
        end else begin
            $display("TB_FAIL tb_auto_rate_ctrl errors=%0d", errors);
        end
        $finish;
    end

endmodule
