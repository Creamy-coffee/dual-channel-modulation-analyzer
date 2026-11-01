`timescale 1ns/1ps
// Checks the automatic display time base: which decimation lands on screen.
//   cycles_on_screen = f * 1024 * 2^m / 65e6
// 0.8/4/8/16 cycles per frame are the interesting corners.
module tb_auto_timebase;
    reg clk = 0, rst_n = 0, enable = 1, valid = 1;
    reg [31:0] f = 0;
    wire [3:0] m;
    wire active, sat;
    wire [7:0] changes;
    integer errors = 0;

    auto_timebase #(.MAX_M(10), .DWELL(8'd4)) dut (
        .clk(clk), .rst_n(rst_n), .enable(enable),
        .freq_hz(f), .freq_update(1'b1),.freq_valid(valid),
        .m(m), .auto_active(active), .saturated(sat), .changes(changes));

    always #5 clk = ~clk;

    task check(input [31:0] freq, input [3:0] want_m, input want_sat, input [127:0] tag);
        begin
            // Cold start: the ACQUISITION decision (m starts at 0) rather than
            // the tracking decision, which deliberately holds a time base that
            // already shows 3..12 cycles.
            rst_n = 0; f = freq;
            repeat (2) @(posedge clk);
            rst_n = 1;
            repeat (12) @(posedge clk);
            if (m !== want_m) begin
                $display("FAIL %0s: f=%0d m=%0d want %0d (cycles=%.2f)",
                         tag, freq, m, want_m,
                         freq * 1024.0 * (1 << m) / 65.0e6);
                errors = errors + 1;
            end else if (sat !== want_sat) begin
                $display("FAIL %0s: f=%0d saturated=%0b want %0b", tag, freq, sat, want_sat);
                errors = errors + 1;
            end else begin
                $display("ok   %0s: f=%0d -> m=%0d R=%0d cycles=%.2f%s",
                         tag, freq, m, (1 << m),
                         freq * 1024.0 * (1 << m) / 65.0e6,
                         sat ? " SATURATED" : "");
            end
        end
    endtask

    initial begin
        repeat (3) @(posedge clk);
        rst_n = 1;
        repeat (3) @(posedge clk);
        // 30 MHz: only 2.17 samples/cycle, keep RAW.  The frame shows 473
        // cycles; that is the point -- never decimate a high carrier just
        // because the ideal count cannot be reached.
        check(32'd30000000, 4'd0, 1'b0, "30MHz");
        check(32'd1000000,  4'd0, 1'b0, "1MHz");
        check(32'd360000,   4'd0, 1'b0, "360kHz");
        check(32'd180000,   4'd1, 1'b0, "180kHz");
        check(32'd100000,   4'd2, 1'b0, "100kHz");
        check(32'd50000,    4'd3, 1'b0, "50kHz");
        check(32'd10000,    4'd6, 1'b0, "10kHz");
        check(32'd1000,     4'd9, 1'b0, "1kHz");
        check(32'd500,      4'd10, 1'b0, "500Hz");
        // Below ~370 Hz even R=1024 cannot reach 3 cycles: the flag says so
        // instead of silently showing a flat line.
        check(32'd100,      4'd10, 1'b1, "100Hz");
        // Hysteresis: 501 Hz is still comfortably inside 3..12 cycles at
        // R=1024, so the time base must NOT move.
        f = 32'd501;
        repeat (12) @(posedge clk);
        if (m !== 4'd10) begin
            $display("FAIL hysteresis: 501 Hz moved m to %0d", m); errors = errors + 1;
        end else $display("ok   hysteresis: 501 Hz keeps m=10");
        // Losing the lock must freeze the time base, not reset it.
        valid = 0; f = 32'd1000000;
        repeat (12) @(posedge clk);
        if (m !== 4'd10) begin
            $display("FAIL hold-on-unlock: m=%0d, expected to hold 10", m); errors = errors + 1;
        end else $display("ok   unlock holds m=10");
        valid = 1;
        // Manual handoff must return the knob to m=0.
        enable = 0;
        repeat (4) @(posedge clk);
        if (m !== 4'd0 || active !== 1'b0) begin
            $display("FAIL manual handoff: m=%0d active=%0b", m, active); errors = errors + 1;
        end else $display("ok   manual handoff -> m=0, auto_active=0");
        if (errors == 0) $display("TB_PASS tb_auto_timebase (%0d changes)", changes);
        else $display("TB_FAIL tb_auto_timebase: %0d errors", errors);
        $finish;
    end
endmodule
