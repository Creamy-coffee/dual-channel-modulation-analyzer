`timescale 1ns/1ps
// Two channels at different frequencies must not publish a phase difference.
module tb_ch_freq_guard;
    reg clk = 0, rst_n = 0, enable = 1, meas_valid = 0, amp_ok = 1;
    reg [15:0] s0 = 0, s1 = 0;
    wire mismatch;
    wire [15:0] delta;
    wire [7:0] score;
    integer errors = 0;

    ch_freq_guard #(.STEP_LIMIT(16'd96), .CONFIRM(8'd24)) dut (
        .clk(clk), .rst_n(rst_n), .enable(enable), .meas_valid(meas_valid),
        .step0(s0), .step1(s1), .amp_ok(amp_ok),
        .mismatch(mismatch), .delta_avg(delta), .score(score));

    always #5 clk = ~clk;

    task window(input [15:0] a, input [15:0] b, input integer n);
        integer k;
        begin
            // Stimulus changes on the FALLING edge: driving it from the same
            // posedge that woke this process makes the strobe race the DUT.
            for (k = 0; k < n; k = k + 1) begin
                @(negedge clk); s0 = a; s1 = b; meas_valid = 1;
                @(negedge clk); meas_valid = 0;
            end
        end
    endtask

    initial begin
        repeat (3) @(posedge clk);
        rst_n = 1;
        repeat (3) @(posedge clk);

        // Same source: the two phase steps are identical.
        window(16'd500, 16'd500, 60);
        if (mismatch !== 1'b0) begin
            $display("FAIL same source flagged (score=%0d delta=%0d)", score, delta);
            errors = errors + 1;
        end else $display("ok   same source: mismatch=0 (delta=%0d LSB)", delta);

        // 200 LSB apart = 1.1 deg per window ~ 3 Hz at 1 ms: different sources.
        window(16'd600, 16'd400, 60);
        if (mismatch !== 1'b1) begin
            $display("FAIL different source not flagged (score=%0d)", score);
            errors = errors + 1;
        end else $display("ok   different source: mismatch=1 after %0d windows (delta=%0d)", score, delta);

        // Back to one source: the flag must clear, not latch.
        window(16'd500, 16'd500, 120);
        if (mismatch !== 1'b0) begin
            $display("FAIL mismatch did not clear (score=%0d)", score);
            errors = errors + 1;
        end else $display("ok   flag clears when the channels agree again");

        // Phase noise on a weak tone: alternating excursions must not fire it.
        begin : noise
            integer k;
            for (k = 0; k < 200; k = k + 1) begin
                s0 = 16'd500 + ((k % 2) ? 16'd300 : 16'd0);
                s1 = 16'd500; meas_valid = 1; @(posedge clk); meas_valid = 0; @(posedge clk);
            end
        end
        if (mismatch !== 1'b0) begin
            $display("FAIL random excursions latched the flag (score=%0d)", score);
            errors = errors + 1;
        end else $display("ok   50%% duty-cycle excursions do not latch (score=%0d)", score);

        // No amplitude -> the guard must not claim a mismatch on noise alone.
        amp_ok = 0; window(16'd900, 16'd0, 60);
        if (mismatch !== 1'b0) begin
            $display("FAIL flagged while no signal present"); errors = errors + 1;
        end else $display("ok   no signal: guard stays quiet");

        if (errors == 0) $display("TB_PASS tb_ch_freq_guard");
        else $display("TB_FAIL tb_ch_freq_guard: %0d errors", errors);
        $finish;
    end
endmodule
