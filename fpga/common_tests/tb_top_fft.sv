`timescale 1ns/1ps
// ---------------------------------------------------------------------------
// tb_top_fft - end-to-end proof of the 4096-point shared-FFT carrier finder
// inside the REAL daq_top (FFT_ENABLE=1).
//
// WHY THIS EXISTS
//   tb_fft_frame proves the FFT numerics and tb_fft_find proves the finder
//   state machine, but only this test drives the whole chain the way the
//   board does:
//     ADC -> Hann -> 4096-pt FFT (find) -> fft_find_ctrl -> cfg_fcw/commit
//          -> lockin_measurement -> local_afc_2ch (track) -> lock-in (measure)
//          -> vofa_base_display -> uart_tx (proto=6 META line)
//   and checks the division of labour the port was asked for:
//     (a) the FFT locks (fft_locked) and the NCO sits on the applied tone
//     (b) the lock-in still produces a real amplitude once parked
//     (c) the bandwidth gear still narrows while locked
//     (d) the sweep is arbitrated away (scan telemetry stays 0)
//     (e) the wire protocol publishes proto=6 with ffs=4 and a sane fff0/ffm0
//
//   Note on the run length: after the finder locks, the next META line can
//   only be captured once the CURRENT display dump finishes - one dump is the
//   META line (~0.8k bytes) plus 1024 wave rows (~15 bytes each) at 700
//   cycles/byte, i.e. ~11.5M cycles.  That is what sets MAXCYC.
//
// USAGE
//   iverilog -g2012 -DSIMULATION -s tb_top_fft -o tb_top_fft.vvp \
//       tb_top_fft.sv sim_prims.v ../rtl/*.v          (from tests/)
//   cd ../rtl && vvp ../tests/tb_top_fft.vvp           (from rtl/, .mem files)
// ---------------------------------------------------------------------------
module tb_top_fft;

`ifndef STIM_HZ
`define STIM_HZ 30000000
`endif
    parameter integer STIM_HZ = `STIM_HZ;
    // 800 counts -> Q12 amplitude ~3.2e6 once parked (same scale as
    // tb_top_scan); 1e5 catches any detune.
    parameter integer PEAK = 20;
    integer tone_peak=PEAK;
    integer amp_floor;
    integer dc0=-4,dc1=8,zero_mode=0;
    reg [31:0] noise_state=32'h6eadb321;
    integer noise0,noise1;
    string wire_path="../tests/fft_top_uart.bin";
    function integer nearest;
      input real x;
      begin nearest=x>=0 ? $rtoi(x+0.5) : $rtoi(x-0.5);end
    endfunction
    // Cycle budgets.  Lock needs one FFT frame (~356k) + the 64-cycle Hz->FCW
    // division + the AFC settle window; 8M is ~20x margin.  AMP/BW run from
    // the lock; META has to outlast a full display dump (~11.5M cycles).
    parameter integer LOCK_BUDGET =  8000000;
    parameter integer AMP_BUDGET  =  3000000;
    parameter integer BW_BUDGET   = 800000000;
    parameter integer META_BUDGET = 850000000;
    parameter integer MAXCYC      = 900000000;

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

    daq_top #(
        .STARTUP_CYCLES(200),
        .REF_FREQ_HZ(10000000),
        .AFC_ENABLE(1),
        .SCAN_ENABLE(0),
        .FFT_ENABLE(1)
    ) dut (
        .clk_50m(clk_50m), .rst_n(rst_n),
        .adc_d1(adc_d1), .adc_d0(adc_d0),
        .adc_otr_b(adc_otr_b), .adc_otr(adc_otr),
        .adc_clk_b(), .adc_oeb_b(), .adc_pwdn_b(),
        .adc_clk(),   .adc_oeb(),   .adc_pwdn(),
        .digital_in(digital_in), .key_n(key_n), .sw(sw),
        .led(led), .uart_tx(uart_tx)
    );

    // ---- stimulus: two channels of the same tone, 90 degrees apart ----------
    real step,stim_phase=0.0;
    integer n;
    initial begin
        step = 2.0 * 3.141592653589793 * STIM_HZ / 65.0e6;
        n    = 0;
        if($value$plusargs("PEAK=%d",tone_peak)) begin end
        if($value$plusargs("ZERO=%d",zero_mode)) begin end
        amp_floor=tone_peak*3000;
    end
    always @(posedge dut.clk_sys) begin
        n = n + 1;
        stim_phase=stim_phase+step;
        if(stim_phase>=6.283185307179586) stim_phase=stim_phase-6.283185307179586;
        noise_state={noise_state[30:0],noise_state[31]^noise_state[21]^noise_state[1]^noise_state[0]};
        noise0=integer'(noise_state[0])-integer'(noise_state[1]);
        noise1=integer'(noise_state[4])-integer'(noise_state[5]);
        adc_d0 <= 2048+dc0+noise0+nearest((zero_mode?0.0:real'(tone_peak))*$cos(stim_phase));
        adc_d1 <= 2048+dc1+noise1+nearest((zero_mode?0.0:real'(tone_peak))*$cos(stim_phase-3.141592653589793/2.0));
    end

    // Declared before the progress monitor: xsim (unlike Verilator) rejects a
    // forward reference even inside a $display argument list.
    integer   meta_lines = 0;
    integer   bytes_rx   = 0;
    // ---- cycle counter (NBA; sample it only after a #1 settle) -------------
    integer cyc = 0;
    always @(posedge dut.clk_sys) cyc <= cyc + 1;
    // Progress monitor: one line per 500k cycles.  The post-lock META capture
    // alone costs ~12M cycles, so without this a healthy run looks identical
    // to a hung one for a quarter of an hour.
    integer mon = 0;
    always @(posedge dut.clk_sys) begin
        if (cyc >= mon) begin
            mon = cyc + 500000;
            $display("[%0t] cyc=%0d ffs=%0d frames=%0d locked=%0b amp0=%0d "
                     , $time, cyc, dut.fft_state, dut.fft_frames, dut.fft_locked, dut.li_amp0);
            $display("        afc_rdy=%0b bwchg=%0d meta_lines=%0d rx=%0d scan=%0d/%0d"
                     , dut.afc_ready, dut.bw_changes, meta_lines, bytes_rx,
                     dut.scan_state, dut.scan_passes);
            $display("        qual=%0d flags=%0d gr=%0d/%0d fine=%0b gear=%0d cfg_commit=%0b reconfig=%0b",
                     dut.li_qual,dut.li_flags,dut.g_reason0,dut.g_reason1,dut.afc_precision_ready,dut.bw_gear,dut.cfg_commit,dut.cfg_reconfig);
            $fflush;   // vvp block-buffers a pipe; without this a healthy
                       // multi-minute run looks identical to a hung one
        end
    end
    // Watchdog in simulated TIME so a hang can never spin forever.
    initial begin
        #(MAXCYC * 15.38461538 + 2000000);
        $display("TB_FAIL tb_top_fft: watchdog expired near cycle %0d (state=%0d frames=%0d)",
                 MAXCYC, dut.fft_state, dut.fft_frames);
        $finish;
    end

    // ---- independent nominal-baud UART receiver ----------------------------
    // Same three rules as tb_adc_display (arm only after the line has been
    // seen HIGH, sample at the bit centres, check the stop bit) with ONE
    // addition: 'armed' starts at 1.  tx is held high by reset and never
    // posedge's before the first start bit, so a receiver armed only by
    // @(posedge tx) silently eats the very first byte - which is the '#' of
    // the META line this test needs.
    real      bit_ns = 1000000000.0/921600.0;
    reg       armed  = 1'b1;
    always @(posedge uart_tx) armed <= 1'b1;

    localparam integer LINEMAX = 2048;
    reg [7:0] line_buf [0:LINEMAX-1];
    integer   line_len  = 0;
    reg [7:0] meta_buf [0:LINEMAX-1];
    integer   meta_len  = 0;
    integer wire_fd;
    initial begin
      if($value$plusargs("WIRE=%s",wire_path)) begin end
      wire_fd=$fopen(wire_path,"wb");
    end
    final $fclose(wire_fd);
    reg [7:0] value;
    integer   bitnum, li;

    initial begin
        forever begin
            @(negedge uart_tx);
            if (armed) begin
                armed = 1'b0;
                #(1.5*bit_ns);
                for (bitnum=0; bitnum<8; bitnum=bitnum+1) begin
                    value[bitnum] = uart_tx;
                    #(bit_ns);                  // advance to the next bit centre
                end
                if (uart_tx !== 1'b1) begin
                    $display("TB_FAIL tb_top_fft: stop-bit violation on byte %0d (0x%02x)",
                             bytes_rx, value);
                    $finish;
                end
                bytes_rx = bytes_rx + 1;
                $fwrite(wire_fd,"%c",value);
                if (line_len < LINEMAX) line_buf[line_len] = value;
                line_len = line_len + 1;
                if (value == 8'h0a && line_len > 0) begin
                    if (line_buf[0] == 8'h23) begin   // '#'
                        meta_lines = meta_lines + 1;
                        for (li=0; li<line_len && li<LINEMAX; li=li+1)
                            meta_buf[li] = line_buf[li];
                        meta_len = (line_len<LINEMAX) ? line_len : LINEMAX;
                    end
                    line_len = 0;
                end
            end
        end
    end

    // ---- substring search / field extraction over the captured META line ---
    // Pack a pattern with its FIRST character in the most significant byte:
    //   pat = {"proto=6", 8'h00}
    function integer has_sub;
        input [63:0] pat;
        input integer plen;
        integer i, j, ok;
        begin
            has_sub = 0;
            for (i = 0; i <= meta_len - plen; i = i + 1) begin
                ok = 1;
                for (j = 0; j < plen; j = j + 1)
                    if (meta_buf[i+j] !== pat[63 - j*8 -: 8]) ok = 0;
                if (ok) has_sub = 1;
            end
        end
    endfunction

    integer fval;
    reg     ffound;
    task get_field;
        input [63:0] pat;
        input integer plen;
        input integer ndig;
        integer i, j, ok, at, done;
        begin
            ffound = 1'b0; fval = 0; at = -1; done = 0;
            for (i = 0; i <= meta_len - plen && !done; i = i + 1) begin
                ok = 1;
                for (j = 0; j < plen; j = j + 1)
                    if (meta_buf[i+j] !== pat[63 - j*8 -: 8]) ok = 0;
                if (ok) begin at = i + plen; done = 1; end
            end
            if (at >= 0) begin
                ffound = 1'b1;
                for (i = 0; i < ndig; i = i + 1) begin
                    if (at+i < meta_len && meta_buf[at+i] >= 8'h30 && meta_buf[at+i] <= 8'h39)
                        fval = fval*10 + (meta_buf[at+i] - 8'h30);
                    else i = ndig;
                end
            end
        end
    endtask

    // ---- helpers ------------------------------------------------------------
    task fail;
        input [511:0] msg;
        begin
            $display("TB_FAIL tb_top_fft: %0s (cycle %0d, ffs=%0d ffid=%0d)",
                     msg, cyc, dut.fft_state, dut.fft_frames);
            $finish;
        end
    endtask

    real fcw_stim, fft_hz, err_hz, act_hz, act_err, bin_err;
    integer exp_bin, exp_mag, meta_at_lock, locked_at, amp_at, bw_at, checks;
    reg [1023:0] dump;   // printable copy of the captured META line

    initial begin
        checks     = 0;
        locked_at  = -1;
        amp_at     = -1;
        bw_at      = -1;
        // REAL arithmetic: STIM_HZ*4096 overflows a 32-bit integer (8.19e9 >
        // 2^31) and silently wrapped to -5 in the first version of this line.
        exp_bin    = $rtoi(real'(STIM_HZ) * 4096.0 / 65.0e6);
        exp_mag    = (16*tone_peak*tone_peak) >> 3;                        // mag>>3, on-bin
        fcw_stim   = real'(STIM_HZ) * 4294967296.0 / 65.0e6;

        #2000;
        rst_n = 1'b1;
        $display("-- tb_top_fft: STIM_HZ=%0d  FFT_ENABLE=1 AFC_ENABLE=1 SCAN_ENABLE=0 --", STIM_HZ);

        if(zero_mode) begin
          while(cyc<20000000) begin
            @(posedge dut.clk_sys);#1;
            if(dut.fft_locked || dut.fft_state==4) fail("Zero-input plus DC was falsely accepted by FFT finder");
          end
          $display("ZERO_TOP_PASS no FFT TRACK in 20 million clocks, frames=%0d",dut.fft_frames);
          $display("TB_PASS tb_top_fft zero");$finish;
        end
        // ---- (a) the FFT finder must lock -----------------------------------
        while (locked_at < 0) begin
            @(posedge dut.clk_sys); #1;
            if (dut.fft_locked) locked_at = cyc;
            if (cyc > LOCK_BUDGET)
                fail("FFT finder never locked");
        end
        meta_at_lock = meta_lines;    // only lines completed BEFORE the lock count
        $display("finder locked at cycle %0d: ffs=%0d fff0=%0d Hz ffb0=%0d ffm0=%0d (bin %0d expected)",
                 locked_at, dut.fft_state, dut.fft_freq0, dut.fft_bin0, dut.fft_mag0, exp_bin);
        $fflush;

        // The committed reference must be the tone the FFT found, not a
        // default: cfg_fcw_eff IS find_fcw while the FFT owns the search.
        fft_hz = real'(dut.cfg_fcw_eff) * 65.0e6 / 4294967296.0;
        err_hz = fft_hz - real'(STIM_HZ);
        if (err_hz < 0) err_hz = -err_hz;
        if (err_hz > 4000.0)
            fail("cfg_fcw_eff is not the applied tone");
        $display("cfg_fcw_eff = %0.1f Hz (err %0.1f Hz, 0.25 bin = %0.1f Hz)",
                 fft_hz, err_hz, 0.25*65.0e6/4096.0);

        // The AFC must then micro-tune the ACTIVE reference onto the tone.
        act_hz  = real'(dut.li_active_fcw) * 65.0e6 / 4294967296.0;
        act_err = act_hz - real'(STIM_HZ);
        if (act_err < 0) act_err = -act_err;
        if (act_err > 1600.0)
            fail("AFC did not pull the NCO onto the tone");
        $display("li_active_fcw = %0.1f Hz (err %0.1f Hz, afc_ready=%0b)",
                 act_hz, act_err, dut.afc_ready);

        // ---- (b) the lock-in must still measure once parked -----------------
        while (amp_at < 0) begin
            @(posedge dut.clk_sys); #1;
            if (dut.li_amp0 > amp_floor) amp_at = cyc;
            if (cyc > locked_at + AMP_BUDGET)
                fail("lock-in amplitude never came up after the FFT lock");
        end
        $display("lock-in amp0 = %0d at cycle %0d (gear %0d, decim_sel %0d)",
                 dut.li_amp0, amp_at, dut.bw_gear, dut.cfg_decim_sel_eff);
        if (dut.li_amp0 < amp_floor) fail("amp0 below floor");

        // ---- (c) the bandwidth gear must still narrow while locked ----------
        while (bw_at < 0) begin
            @(posedge dut.clk_sys); #1;
            if (dut.bw_gear == 2 && dut.afc_precision_ready) bw_at = cyc;
            if (cyc > locked_at + BW_BUDGET)
                fail("bandwidth gear never moved while locked");
        end
        $display("bw gear %0d after %0d changes at cycle %0d",
                 dut.bw_gear, dut.bw_changes, bw_at);

        // Guard qualification is mandatory: a successful FFT or a nonzero
        // bw_changes counter alone cannot establish precision measurement.
        begin : qualified_windows
          integer consecutive;
          consecutive=0;
          while(consecutive<8) begin
            @(posedge dut.clk_sys); #1;
            if(dut.lockin_valid) begin
              if(dut.bw_gear==2 && !dut.bw_settling && dut.li_qual==3 && dut.afc_ready && dut.li_flags==0)
                consecutive=consecutive+1;
              else consecutive=0;
            end
            if(cyc>locked_at+BW_BUDGET) fail("No eight consecutive qualified precise windows");
          end
          if(dut.g_reason0!=0 || dut.g_reason1!=0) fail("Guard rejects the settled precision result");
          $display("PRECISE_TOP_PASS eight consecutive precision windows at cycle %0d",cyc);
        end
        // A real, phase-continuous frequency step must temporarily withdraw
        // precision permission without telling the FFT that the carrier vanished.
        begin : carrier_step
          integer consecutive,old_lost,seen_wait;
          consecutive=0;old_lost=dut.fft_lost;seen_wait=0;
          step=2.0*3.141592653589793*(real'(STIM_HZ)+50.0)/65.0e6;
          while(consecutive<4 || !seen_wait) begin
            @(posedge dut.clk_sys);#1;
            if(dut.afc_coarse_ready && !dut.afc_precision_ready) seen_wait=1;
            if(dut.fft_state!=4 || dut.fft_lost!=old_lost) fail("False FFT loss during fine frequency reconfirmation");
            if(dut.lockin_valid) begin
              if(dut.bw_gear==2 && !dut.bw_settling && dut.li_qual==3 && dut.afc_ready && dut.li_flags==0)
                consecutive=consecutive+1;
              else consecutive=0;
            end
            if(cyc>locked_at+BW_BUDGET) fail("Top did not recapture precise measurement after +50Hz");
          end
          act_hz=real'(dut.li_active_fcw)*65.0e6/4294967296.0;
          if(act_hz<real'(STIM_HZ)+49.785 || act_hz>real'(STIM_HZ)+50.215) fail("Top frequency residual exceeds precise budget after step");
          $display("TOP_STEP_PASS carrier stayed TRACK, fine permission recovered, active=%0.6f Hz",act_hz);$fflush;
        end
        if($signed(dut.li_pdiff)< -16749 || $signed(dut.li_pdiff)> -16019)
          fail("Top-level channel phase relation differs from -90 degrees");
        if(dut.li_ratio<63569 || dut.li_ratio>67502) fail("Top-level dual-channel ratio mismatch");
        meta_at_lock=meta_lines;

        // ---- (d) the sweep must be arbitrated away --------------------------
        if (dut.scan_state != 0 || dut.scan_passes != 0 || dut.scan_locked)
            fail("sweep telemetry is live - the FFT did not win the arbitration");

        // ---- (e) the wire protocol must publish proto=6 / ffs=4 -------------
        // Wait for a META line that was latched AFTER the lock: its ffs field
        // must read 4 (TRACK).  Lines completing right after the lock may
        // still carry a pre-lock snapshot, so the check is on the CONTENT.
        begin : wait_qualified_meta
          integer checked_line,found;
          checked_line=meta_at_lock;found=0;
          // Scan only newly completed lines: scanning 770 bytes per clock
          // does not add coverage and makes a nominal-baud wait very slow.
          while(!found) begin
            @(posedge dut.clk_sys); #1;
            if(meta_lines>checked_line) begin
              checked_line=meta_lines;
              found=has_sub({"ffs=4",24'd0},5) && has_sub({"qual=03",8'd0},7);
            end
            if(cyc>locked_at+META_BUDGET) fail("No qualified FFT META after precision lock");
          end
        end
        $display("META line %0d captured (%0d bytes) at cycle %0d; bytes_rx=%0d",
                 meta_lines, meta_len, cyc, bytes_rx);

        if (!has_sub("proto=12", 8)) fail("META line does not say proto=12");
        checks = checks + 1;
        if (!has_sub({"scan=0",16'd0}, 6))   fail("META scan field is not 0");
        checks = checks + 1;
        if (!has_sub({"slock=0",8'h00}, 7))  fail("META slock field is not 0");
        checks = checks + 1;

        // fff0 must be the interpolated peak in Hz, within the interpolation
        // tolerance proven in tb_fft_frame (worst case ~0.11 bin).
        get_field({"fff0=",24'd0}, 5, 8);
        if (!ffound) fail("META line has no fff0 field");
        checks = checks + 1;
        $display("fff0 = %0d Hz (applied %0d, err %0d Hz)", fval, STIM_HZ,
                 (fval > STIM_HZ) ? (fval - STIM_HZ) : (STIM_HZ - fval));
        if (fval < STIM_HZ - 5000 || fval > STIM_HZ + 5000)
            fail("fff0 is not the applied tone");

        // ffm0 = mag>>3; full scale is 16*A^2 for A ADC counts, so an
        // 800-count tone lights ~1.28e6.  Hann scalloping moves it a few
        // percent for a 2 MHz tone (bin 126.04); 0.6..1.4x is a safe window.
        get_field({"ffm0=",24'd0}, 5, 8);
        if (!ffound) fail("META line has no ffm0 field");
        checks = checks + 1;
        $display("ffm0 = %0d (expected ~%0d for current peak counts)", fval, exp_mag);
        if (fval < (exp_mag*6)/10 || fval > (exp_mag*14)/10)
            fail("ffm0 is not the expected magnitude scale");

        // The measurement itself must reach the wire: amp0 ~3.2e6 Q12.
        get_field({"amp0=",24'd0}, 5, 8);
        if (!ffound) fail("META line has no amp0 field");
        checks = checks + 1;
        if (fval < amp_floor) fail("META amp0 below floor");
        $display("META amp0 = %0d", fval);

        // ffid must count published frames (>= 2 by now: one before the lock,
        // at least one after).
        get_field({"ffid=",24'd0}, 5, 10);
        if (!ffound) fail("META line has no ffid field");
        checks = checks + 1;
        if (fval < 2) fail("ffid did not advance");
        $display("ffid = %0d, ffs=%0d (RTL), fflost present=%0b",
                 fval, dut.fft_state, has_sub({"fflost=",8'd0}, 7));

        get_field({"qual=",24'd0},5,2);
        if(!ffound || fval!=3) fail("Wire result lacks dual-channel qualification");
        get_field({"ar=",40'd0},3,1);
        if(!ffound || fval!=1) fail("Wire result lacks AFC precision permission");
        if(!has_sub({"fflost=",8'd0},7)) fail("Missing FFT loss field on UART");
        // ---- report ---------------------------------------------------------
        bin_err = real'(dut.fft_bin0) - real'(exp_bin);
        $display("---- summary ----");
        $display("finder      : locked at %0d, ffs=%0d, fff0=%0d Hz, ffb0=%0d (exp %0d, err %0.2f bin)",
                 locked_at, dut.fft_state, dut.fft_freq0, dut.fft_bin0, exp_bin, bin_err);
        $display("measurement : amp0=%0d amp1=%0d ratio_q16=%0d pdiff=%0d",
                 dut.li_amp0, dut.li_amp1, dut.li_ratio, dut.li_pdiff);
        $display("tracking    : afc_state=%0d ready=%0b updates=%0d residual0=%0d FCW",
                 dut.afc_state, dut.afc_ready, dut.afc_updates, dut.residual0);
        $display("bandwidth   : gear=%0d changes=%0d decim_sel=%0d integ=%0d",
                 dut.bw_gear, dut.bw_changes, dut.cfg_decim_sel_eff, dut.cfg_integ_eff);
        $display("sweep       : state=%0d locked=%0b passes=%0d (must stay 0/0)",
                 dut.scan_state, dut.scan_locked, dut.scan_passes);
        $display("uart        : %0d bytes, %0d META lines, last line %0d bytes",
                 bytes_rx, meta_lines, meta_len);
        $display("TB_PASS tb_top_fft (%0d wire checks + RTL checks)", checks);
        $finish;
    end
endmodule
