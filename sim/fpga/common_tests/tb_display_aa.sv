`timescale 1ns/1ps
// Measures what the DISPLAY decimator actually does to a tone, not what a
// textbook filter would do, and not the lock-in filter.
//   case 0 : wanted tone only         -> passband droop must be tiny
//   case 1 : interferer at fsd - f_w  -> folds exactly onto the wanted bin
//   case 2 : interferer at fsd/2+1bin -> folds one bin below the Nyquist
//   case 3 : interferer at fsd/4      -> mid-band, checks the comb shape
//   case 4 : interferer at 7*fsd/8    -> folds to fsd/8, keeps |sinc(0.875)|
//   case 5 : interferer at 15*fsd/8   -> folds to fsd/8, keeps |sinc(1.875)|
//
// Cases 4 and 5 exist to kill one specific wrong generalisation.  "-3.9 dB at
// fsd/2" is the value AT the folding boundary, and it is the WORST CASE, not a
// uniform "every tone above Nyquist keeps 64% of its amplitude".  The retained
// amplitude of an aliased tone is |sinc(f_i/fsd)| evaluated at the PRE-fold
// frequency f_i/fsd (the boxcar filters at the input rate, THEN decimates), so
// it moves continuously.  Cases 4 and 5 land on the SAME output bin (fsd/8)
// yet are retained 0.1392 and 0.0650 - a factor of 2.1 - because their
// pre-fold frequencies differ.
//
// *** NO CASE MAY FOLD EXACTLY ONTO fsd/2 ***  (2026-10-19, measured)
// A tone whose output bin is exactly Nyquist produces a TWO-VALUED sequence
// C*(-1)^m*cos(psi).  Amplitude and phase are no longer separable there, so
// the single-bin correlation returns C*|cos(psi)|, which is set by the
// alignment between the tone and the decimator group boundaries - NOT by the
// filter.  Measured by sweeping the drive phase over 0..2*pi at M=6:
//     interferer at fsd/2   : gain 0.097 .. 0.632   (theory 0.6366)
//     interferer at 3*fsd/2 : gain 0.001 .. 0.212   (theory 0.2122)
// The old case 2 (f_i = fsd/2) reported 0.63187 and "matched" theory to 0.75%
// purely because its alignment happened to give cos(psi) ~= 1.  Case 2 now
// sits one bin off (fsd/2 + fsd/512 -> folds to 255*fsd/512), which is
// non-degenerate and still measures the worst folding point to 0.5%.
// Case 5 was f_i = 3*fsd/2 for the same reason and is now 15*fsd/8.
//
// Cases 4/5 need R large enough that the exact Dirichlet kernel
// D(u)=sin(pi*u)/(R*sin(pi*u/R)) has converged to sinc(u); at R=64 the two
// differ by 0.02%, at R=2 they differ by 3x.  Run them with M=6.
//
// The implemented decimator is a BOXCAR average of R consecutive samples
// (accumulate-and-dump), NOT a CIC.  Its response in output-sample units is
// |sinc(u)| with u = f/fsd, so the expected gains are
//   fsd/4  -> |sinc(0.25)| = 0.9003   (-0.91 dB)
//   fsd/2  -> |sinc(0.50)| = 0.6366   (-3.92 dB)
// The previous version of this bench asserted CIC^3 numbers (0.15..0.32 at
// fsd/2, 0.70..0.76 at fsd/4).  Those were re-baselined on 2026-10-19 after
// the CIC claim was withdrawn; see reports/decimator_findings_20261019.md.
//
// Case 2 is therefore a WEAK assertion: a boxcar only attenuates the worst
// alias by -3.9 dB.  It is recorded here so the bench states the truth rather
// than implying useful anti-aliasing.  Case 1 (-42 dB) is real only because the
// interferer folds to nearly DC, where any low-pass helps.
//
// NOTE: the decimator is flushed when a frame is committed (the buffer waits
// for the reader), so the analysis window lives INSIDE one frame: skip 128 of
// the 1024 points and correlate the next 512, a whole number of cycles.
module tb_display_aa;
    localparam integer FS = 65000000;
    integer MM;
    initial if (!$value$plusargs("M=%d", MM)) MM = 4;

    reg clk = 0, rst_n = 0, sample_valid = 1;
    reg signed [11:0] ch0 = 0, ch1 = 0;
    wire [9:0] rd_addr;
    wire signed [11:0] d0, d1;
    wire ready, filtered, auto_trig;
    wire [15:0] fid;
    wire [31:0] interval, errors;
    wire [9:0] trig_pos;
    wire [1:0] fault;
    wire [3:0] decim;
    wire [63:0] first_index;

    // The boxcar decimator has no MAX_M: R is 2^timebase_log2, clamped to 7.
    // MAX_M was a parameter of the withdrawn CIC experiment.
    acq_trigger_buffer #(.DISP_AW(10), .CLK_FREQ(FS)) dut (
        .clk(clk), .rst_n(rst_n), .sample_valid(sample_valid),
        .ch0(ch0), .ch1(ch1), .fault(2'd0),
        .trig_level(12'sd0), .trig_mode(2'd2), .trig_src(1'b0),
        .pretrig(10'd256), .timebase_log2(MM[3:0]), .freeze(1'b0),
        .disp_busy(disp_busy), .snap_rd_addr(rd_addr),
        .snap_rd_ch0(d0), .snap_rd_ch1(d1),
        .snap_ready(ready), .snap_frame_id(fid),
        .snap_sample_interval_ns(interval), .snap_trig_pos(trig_pos),
        .snap_filtered(filtered), .snap_fault(fault),
        .snap_decim_log2(decim), .snap_auto_trigger(auto_trig),
        .snap_first_index(first_index), .capture_errors(errors));

    always #5 clk = ~clk;

    // Reader model: take the frame and release it four clocks later.
    reg disp_busy = 0;
    reg [2:0] hold;
    always @(posedge clk) begin
        if (!rst_n) begin disp_busy <= 0; hold <= 0; end
        else if (ready) begin disp_busy <= 1; hold <= 3; end
        else if (hold != 0) begin hold <= hold - 1; if (hold == 1) disp_busy <= 0; end
    end

    real fsd, f_w, f_i, amp_in;
    real acc_c, acc_s, amp_out;
    integer case_id, got, skip, in_frame;
    real y, f_eval, t;
    // Drive-phase perturbation.  A tone that folds exactly onto fsd/2 is
    // phase-degenerate, so any case whose gain moves when this changes is
    // measuring the alignment, not the filter.  regression.py runs every case
    // at PHASE=0 and PHASE=1.5708 for exactly this reason.
    real ph;

    initial begin
        if (!$value$plusargs("CASE=%d", case_id)) case_id = 0;
        if (!$value$plusargs("PHASE=%f", ph)) ph = 0.0;
        amp_in = 800.0;
        fsd = 65000000.0 / (1 << MM);
        f_w = 4.0 * fsd / 512.0;           // 8 cycles per 1024-point frame
        f_i = f_w;
        if (case_id == 1) f_i = fsd - f_w; // fold onto the wanted bin
        // One bin ABOVE Nyquist so the folded tone is one bin BELOW it.  A tone
        // that folds exactly onto fsd/2 is phase-degenerate - see the header.
        if (case_id == 2) f_i = fsd * (0.5 + 1.0/512.0);
        if (case_id == 3) f_i = fsd / 4.0; // mid-band: checks the comb delays
        if (case_id == 4) f_i = 7.0 * fsd / 8.0;  // folds to fsd/8
        if (case_id == 5) f_i = 15.0 * fsd / 8.0; // folds to fsd/8 too
        $display("case=%0d M=%0d R=%0d fsd=%0.1f f_w=%0.1f f_i=%0.1f amp=%0.1f",
                 case_id, MM, (1 << MM), fsd, f_w, f_i, amp_in);
        t = 0.0;
        acc_c = 0.0; acc_s = 0.0; got = 0; skip = 0; in_frame = 0;
        repeat (3) @(posedge clk);
        rst_n = 1;
    end

    // Drive the analog input at the FULL rate.
    always @(posedge clk) begin
        t = t + 1.0;
        ch0 <= $rtoi(amp_in * $sin(6.283185307179586 * f_i * t / 65000000.0 + ph));
        ch1 <= ch0;
    end

    // Correlate 512 decimated samples against the bin we expect to see.
    always @(posedge clk) begin
        if (rst_n) begin
            if (dut.state == 3'd0) begin in_frame <= 0; skip <= 0; got <= 0; end
            if (dut.point_valid) begin
                if (skip < 128) begin
                    skip <= skip + 1;
                end else if (got < 512) begin
                    // Correlate at the bin the tone ACTUALLY lands on after
                    // folding, not at the frequency it was driven at.
                    if (case_id == 2) f_eval = fsd * (0.5 - 1.0/512.0);
                    else if (case_id == 3) f_eval = fsd / 4.0;
                    else if (case_id == 4 || case_id == 5) f_eval = fsd / 8.0;
                    else f_eval = f_w;
                    y = $itor(dut.point0);
                    acc_c = acc_c + y * $cos(6.283185307179586 * f_eval * got / fsd);
                    acc_s = acc_s + y * $sin(6.283185307179586 * f_eval * got / fsd);
                    got <= got + 1;
                end else if (got == 512) begin
                    got <= 513;
                    // No case folds exactly onto fsd/2 any more, so the +/-
                    // bins never coincide and the 2/N single-sided scaling is
                    // correct for every case.  (The old code special-cased
                    // case 2 for exactly this reason.)
                    amp_out = 2.0 *
                              $sqrt(acc_c * acc_c + acc_s * acc_s) / 512.0;
                    $display("RESULT case=%0d M=%0d amp_out=%.2f amp_in=%.1f gain=%.5f (%.2f dB)",
                             case_id, MM, amp_out, amp_in, amp_out / amp_in,
                             20.0 * $log10(amp_out / amp_in + 1.0e-12));
                    if (case_id == 0 && (amp_out < 0.97 * amp_in || amp_out > 1.03 * amp_in)) begin
                        $display("TB_FAIL passband droop out of range"); $finish;
                    end
                    if (case_id == 1 && amp_out > 0.02 * amp_in) begin
                        $display("TB_FAIL alias at fsd-f_w not rejected"); $finish;
                    end
                    if (case_id == 2 && (amp_out < 0.60 * amp_in || amp_out > 0.67 * amp_in)) begin
                        $display("TB_FAIL boxcar one bin below fsd/2 outside 0.60..0.67 of input (0.6342 theory)"); $finish;
                    end
                    if (case_id == 3 && (amp_out < 0.88 * amp_in || amp_out > 0.92 * amp_in)) begin
                        $display("TB_FAIL boxcar at fsd/4 outside 0.88..0.92 of input (0.9003 theory)"); $finish;
                    end
                    // 7*fsd/8 folds to fsd/8.  Theory |sinc(0.875)| = 0.13921,
                    // i.e. -17.1 dB, NOT the -3.9 dB of the fsd/2 boundary.
                    if (case_id == 4 && (amp_out < 0.128 * amp_in || amp_out > 0.152 * amp_in)) begin
                        $display("TB_FAIL boxcar at 7*fsd/8 outside 0.128..0.152 of input (0.1392 theory)"); $finish;
                    end
                    // 15*fsd/8 folds to the SAME fsd/8 bin but is retained at
                    // only |sinc(1.875)| = 0.06497 (-23.7 dB): 2.1x less than
                    // case 4.  Same output bin, different pre-fold frequency,
                    // different retention - this is what disproves "64% for
                    // everything above Nyquist".
                    if (case_id == 5 && (amp_out < 0.058 * amp_in || amp_out > 0.072 * amp_in)) begin
                        $display("TB_FAIL boxcar at 15*fsd/8 outside 0.058..0.072 of input (0.0650 theory)"); $finish;
                    end
                    $display("TB_PASS tb_display_aa case=%0d", case_id);
                    $finish;
                end
            end
        end
    end

    initial begin
        #(2000000 * (1 << MM));   // one frame grows with R
        $display("TB_FAIL tb_display_aa timeout (got=%0d skip=%0d)", got, skip);
        $finish;
    end
endmodule
