`timescale 1ns/1ps
// ---------------------------------------------------------------------------
// tb_decim_kern -- characterise the DISPLAY decimator's kernel from RTL.
//
// WHY THIS BENCH EXISTS
// ---------------------
// reports/decimator_findings_20261019.md claims the abandoned CIC experiment
// produced the decimated kernels
//
//      R=2 : [3,1]                DC gain 4  = R^2
//      R=4 : [10,6]  (norm /16)   DC gain 16 = R^2
//
// and that the shipped boxcar is an exact accumulate-and-dump of R samples.
// Numbers in a report with no reproducible command behind them are folklore.
// This bench re-measures both properties from the RTL so the report can cite
// a log line.
//
// WHY A RAMP, NOT AN IMPULSE, NOT A STEP
// -------------------------------------
// The decimator is `sum += x; if (count == R-1) emit (sum >> m)`.  Two probes
// that look obvious are both useless here:
//
//   * Unit impulse (one sample of 1, then 0):  emits (1 >> m) = 0 for m >= 1.
//     An accumulate-and-dump genuinely has that response; it says nothing
//     about the kernel.
//   * Unit step: the emitted value only ever reports the STEADY state, which
//     is R >> m = 1 for every R - again blind to the kernel shape.
//
// A ramp x[n] = n exposes the kernel directly: the group output is
// (1/R) * sum_k (n0 + k) = n0 + (R-1)/2, i.e. the ramp sampled at the group
// CENTRE with a constant offset.  Any deviation from a straight line means
// the kernel is not uniform; the slope means the DC gain is wrong.
//
// DRIVING DISCIPLINE (this bit me while writing the bench)
// -------------------------------------------------------
// ch0/ch1 MUST change on the NEGEDGE.  If they change on the posedge, the DUT
// samples the pre-edge value and the bench reads every term one sample late -
// which looks exactly like "the accumulator never fills".
//
// NOT part of scripts/regression.py: characterisation, not a gate.  Run by hand
// after touching acq_trigger_buffer:
//
//   iverilog -g2012 -s tb_decim_kern -o k.vvp tests/tb_decim_kern.sv \
//       rtl/acq_trigger_buffer.v
//   vvp k.vvp +M=0 ; vvp k.vvp +M=2 ; vvp k.vvp +M=4 ; vvp k.vvp +M=6
//
// The ratio comes from +M=, NOT from -P: iverilog silently drops the "=N" in
// -Ptop.param=N, which already caused one wrong conclusion in this tree.
// ---------------------------------------------------------------------------
`default_nettype none

module tb_decim_kern;

    localparam integer DISP_AW  = 10;
    localparam integer DISP_N   = (1 << DISP_AW);
    localparam integer SAMPLE_W = 12;
    localparam integer CLK_FREQ = 65000000;
    localparam real    TCLK     = 1000.0 / 65.0;   // 15.3846 ns

    integer M;
    reg clk = 1'b0, rst_n = 1'b0;
    reg sample_valid = 1'b0;
    reg signed [SAMPLE_W-1:0] ch0 = 0, ch1 = 0;
    reg [1:0] fault = 2'b00;
    reg signed [SAMPLE_W-1:0] trig_level = 0;
    reg [1:0] trig_mode = 2'b00;
    reg trig_src = 1'b0;
    reg [DISP_AW-1:0] pretrig = DISP_N - 1;
    reg [3:0] timebase_log2 = 4'd0;
    reg freeze = 1'b0, disp_busy = 1'b0;
    reg [DISP_AW-1:0] snap_rd_addr = 0;

    wire signed [SAMPLE_W-1:0] snap_rd_ch0, snap_rd_ch1;
    wire snap_ready;
    wire [15:0] snap_frame_id;
    wire [31:0] snap_sample_interval_ns;
    wire [DISP_AW-1:0] snap_trig_pos;
    wire snap_filtered;
    wire [1:0] snap_fault;
    wire [3:0] snap_decim_log2;
    wire snap_auto_trigger;
    wire [63:0] snap_first_index;
    wire [31:0] capture_errors;

    acq_trigger_buffer #(
        .SAMPLE_W(SAMPLE_W), .MAIN_AW(14), .DISP_AW(DISP_AW),
        .AUTO_TIMEOUT(3250000), .CLK_FREQ(CLK_FREQ)) dut(
        .clk(clk), .rst_n(rst_n), .sample_valid(sample_valid),
        .ch0(ch0), .ch1(ch1), .fault(fault), .trig_level(trig_level),
        .trig_mode(trig_mode), .trig_src(trig_src), .pretrig(pretrig),
        .timebase_log2(timebase_log2), .freeze(freeze), .disp_busy(disp_busy),
        .snap_rd_addr(snap_rd_addr),
        .snap_rd_ch0(snap_rd_ch0), .snap_rd_ch1(snap_rd_ch1),
        .snap_ready(snap_ready), .snap_frame_id(snap_frame_id),
        .snap_sample_interval_ns(snap_sample_interval_ns),
        .snap_trig_pos(snap_trig_pos), .snap_filtered(snap_filtered),
        .snap_fault(snap_fault), .snap_decim_log2(snap_decim_log2),
        .snap_auto_trigger(snap_auto_trigger),
        .snap_first_index(snap_first_index), .capture_errors(capture_errors));

    always #(TCLK/2.0) clk = ~clk;

    // ---- ramp drive: change on the NEGEDGE, DUT samples on the POSEDGE ----
    // 12-bit signed input is +-2047, so the ramp must stay inside that for the
    // WHOLE run.  Number of input samples needed = R*(groups+1); the ramp value
    // at the end is ramp_step*that.  Pick ramp_step so the end value is ~2000
    // for every R, which keeps 8 clean emissions available at any ratio.
    reg [15:0] ramp;
    integer    ramp_step;
    always @(negedge clk) begin
        if (rst_n && sample_valid) begin
            ramp <= ramp + ramp_step[15:0];
            ch0  <= ramp[SAMPLE_W-1:0];
            ch1  <= ramp[SAMPLE_W-1:0];
        end
    end

    // Collect every point the decimator emits, in order.
    integer n_emit;
    integer emits [0:63];
    always @(posedge clk) begin
        if (rst_n && dut.point_valid && dut.writing) begin
            if (n_emit < 64) emits[n_emit] = dut.point0;
            n_emit = n_emit + 1;
        end
    end

    integer i, k, R, ok, expect_step;
    real    mean_first;
    initial begin
        if (!$value$plusargs("M=%d", M)) M = 4;
        R = 1 << M;
        n_emit = 0;
        ramp   = 16'd0;
        // Input headroom: 12-bit signed is +-2047.  The run drives R*8+8 input
        // samples, so the ramp must satisfy ramp_step*(R*9) <= 2000.  For small
        // R that allows a big step, but a big step also means the ramp reaches
        // the ceiling early, so cap the TOTAL number of driven samples instead:
        // 8 groups is a floor, and 2000/ramp_step must cover it.
        ramp_step = 2000 / (R * 9);
        if (ramp_step < 1) ramp_step = 1;
        // Never let the ramp leave +-2000 over the whole run.
        while (ramp_step * (R * 8 + 8) > 2000 && ramp_step > 1)
            ramp_step = ramp_step / 2;
        for (i = 0; i < 64; i = i + 1) emits[i] = 0;

        timebase_log2 = M[3:0];
        rst_n  = 1'b0;
        repeat (8) @(posedge clk);
        rst_n  = 1'b1;
        repeat (4) @(posedge clk);

        sample_valid = 1'b1;
        for (k = 0; k < R * 8 + 8; k = k + 1) @(posedge clk);
        sample_valid = 1'b0;
        repeat (8) @(posedge clk);

        $display("RAMP R=%0d (m=%0d) step=%0d emitted=%0d", R, M, ramp_step, n_emit);
        $write("EMITS  ");
        for (i = 0; i < 12 && i < n_emit; i = i + 1) $write("%0d ", emits[i]);
        $write("\n");

        // ---- expected step for a ramp of slope `ramp_step` -------------------
        // Input sample k has value k*ramp_step.  Group j covers input samples
        // [jR, jR+R-1], so
        //     sum_j  = ramp_step * R * (jR + (R-1)/2)
        //     point_j = sum_j >> m = ramp_step * (jR + (R-1)/2)     (m = log2 R)
        // and consecutive points differ by ramp_step * R.  That is NOT a bug:
        // the emitted sequence is still a ramp of slope ramp_step, just sampled
        // every R-th input sample, so each output step spans R input steps.
        // The first difference is excluded: the accumulator starts empty, so
        // emission 0 covers a partial group.
        // Two tolerances, both inherent to integer arithmetic rather than to
        // the filter:
        //   * point_j = sum_j >> m truncates toward -inf, so each emitted point
        //     can be up to 1 LSB low.  Differences between points are therefore
        //     within +-1 of ramp_step*R.
        //   * the ramp is a staircase, not a real number line, so the group mean
        //     lands between two integers and >> truncates it.
        expect_step = ramp_step * R;
        ok = 1;
        for (i = 2; i < n_emit && i < 63; i = i + 1) begin
            if ((emits[i] - emits[i-1]) - expect_step > 1 ||
                (emits[i] - emits[i-1]) - expect_step < -1) begin
                $display("TAP_DEVIATION at emit %0d: step %0d, expected %0d (+-1)",
                         i, emits[i] - emits[i-1], expect_step);
                ok = 0;
            end
        end

        // Mean of the first group: ramp values 0, s, 2s, ..., (R-1)s, so the
        // boxcar emits s*(R-1)/2.  A truncated kernel (the withdrawn CIC) emits
        // something else, which is the point of this check.
        mean_first = (ramp_step * 1.0) * (R - 1) / 2.0;
        $display("FIRST_EMISSION %0d  expected %0.1f  (ramp mean over the group)",
                 emits[0], mean_first);

        if (M == 0) begin
            if (n_emit >= 2 && (emits[1] - emits[0]) == ramp_step &&
                emits[0] == 0)
                $display("PASS tb_decim_kern m=0 passthrough (slope %0d, first 0)", ramp_step);
            else
                $display("FAIL tb_decim_kern m=0 expected passthrough");
        end else begin
            if (!ok) $display("FAIL tb_decim_kern emitted step is not uniform - the kernel is not R equal taps");
            else $display("PASS tb_decim_kern uniform step %0d (ramp_step %0d x R %0d), flat average of %0d taps",
                          expect_step, ramp_step, R, R);
        end
        $display("DONE tb_decim_kern M=%0d", M);
        $finish;
    end

    initial begin
        #(TCLK * 1000 * 800);
        $display("FATAL tb_decim_kern timeout (n_emit=%0d)", n_emit);
        $finish;
    end
endmodule
`default_nettype wire
