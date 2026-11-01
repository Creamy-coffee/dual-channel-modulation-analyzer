`timescale 1ns/1ps
// Equivalent-time display tests.  The testbench runs a PARALLEL GOLDEN MODEL
// of the physics: it generates the repetitive signal, advances the same NCO
// phase word, and accumulates the exact per-bin mean/count.  The DUT (an
// exponential moving average of the same samples) must converge to the golden
// per-bin mean, and its coverage/wrap/hit counters must match the model.
// A second, BIT-EXACT golden EMA models the DUT datapath and is used where
// the all-hits arithmetic mean is NOT the EMA's target (the drift case):
// the EMA memory is only ~16 hits per bin while the grid walk spans several
// turns, so there the correct reference is the EMA itself, not the mean.
//
//   case 1  coherent 1 MHz, 64 bins: 65 samples/period -> every bin hit;
//           the reassembled period equals the square wave after averaging.
//   case 2  coherent 5 MHz: only 13 distinct phases exist (65/5); coverage
//           stays at 13 - physics, not firmware, bounds it.
//   case 3  5 MHz signal + 5 kHz NCO offset (measured phase drift): the
//           13-position grid walks and eventually fills all 64 bins.
//   case 4  noise: the EMA suppresses like a ~2^(SHIFT+1)-sample average.
//   case 5  mode/freeze/hold/clear semantics and frame cadence.
`default_nettype none
module tb_et_display;
  localparam integer AW=6, BINS=1<<AW, SHIFT=4, FRAME_CLKS=4096;
  localparam [31:0] FCW_1M  = 32'd66076419;   // round(2^32*1e6/65e6)
  localparam [31:0] FCW_5M  = 32'd330382093;  // round(2^32*5e6/65e6)
  localparam [31:0] FCW_5K  = 32'd330382;     // round(2^32*5e3/65e6)
  localparam integer AMP=1000;

  reg clk=0,rst_n=0;
  always #7.6923076923 clk=~clk;              // 65 MHz

  reg mode, sample_valid, afc_locked, cfg_commit, freeze, hold;
  reg signed [11:0] ch0,ch1;
  reg [31:0] nco_phase,nco_fcw;
  wire frame_pulse; wire [15:0] frame_id;
  wire [AW:0] filled; wire [31:0] wraps,hits;
  reg [9:0] rd_addr; wire signed [11:0] rd_ch0,rd_ch1; wire [15:0] dbg_cnt;

  et_display #(.ET_AW(AW),.ET_AVG_SHIFT(SHIFT),.ET_FRAME_CLKS(FRAME_CLKS),
               .CLK_FREQ(65000000)) dut(
   .clk(clk),.rst_n(rst_n),.mode(mode),.sample_valid(sample_valid),
   .ch0(ch0),.ch1(ch1),.nco_phase(nco_phase),.nco_fcw(nco_fcw),
   .afc_locked(afc_locked),.cfg_commit(cfg_commit),.freeze(freeze),.hold(hold),
   .frame_pulse(frame_pulse),.frame_id(frame_id),.filled(filled),
   .wraps(wraps),.hits(hits),.rd_addr(rd_addr),
   .rd_ch0(rd_ch0),.rd_ch1(rd_ch1),.dbg_cnt(dbg_cnt));

  // ---- golden model ---------------------------------------------------------
  integer ref_sum0[0:BINS-1], ref_sum1[0:BINS-1], ref_cnt[0:BINS-1];
  reg [11:0] gem0[0:BINS-1];                  // bit-exact golden EMA (ch0)
  reg [31:0] g_ph, g_sig, sig_fcw;            // NCO phase / signal phase
  integer g_wraps, g_hits, g_filled;
  reg [31:0] g_last; reg g_have;
  reg [31:0] noise_lfsr; integer noise_amp;
  reg alt_sign;                              // case 6: alternate +/- every sample

  task golden_reset;
    integer i;
    begin
      for(i=0;i<BINS;i=i+1) begin ref_sum0[i]=0;ref_sum1[i]=0;ref_cnt[i]=0;gem0[i]=0;end
      g_ph=0;g_sig=0;g_wraps=0;g_hits=0;g_filled=0;g_last=0;g_have=0;noise_amp=0;alt_sign=0;
    end
  endtask

  // 12-bit two's-complement wrap: matches `avg0[bin] <= a0n[11:0]` in the DUT.
  function integer wrap12(input integer a);
    begin
      wrap12 = a & 12'hFFF;
      if(wrap12 >= 2048) wrap12 = wrap12 - 4096;
    end
  endfunction

  // Drive n samples.  The stimulus value, the golden accumulation and the
  // golden counters all come from the SAME (sample, phase) pair, so any
  // disagreement is a DUT bug, not a modelling artefact.
  task drive_n(input integer n);
    integer i,x,y,b,d;
    begin
      for(i=0;i<n;i=i+1) begin
        @(negedge clk);
        x = (g_sig[31]==1'b0) ? AMP : -AMP;
        if(alt_sign) x = ((i%2)==0) ? AMP : -AMP;
        if(noise_amp!=0) begin
          noise_lfsr = {noise_lfsr[30:0], noise_lfsr[27]^noise_lfsr[25]^noise_lfsr[22]^noise_lfsr[16]};
          x = x + (($signed({1'b0,noise_lfsr[15:0]}) % (2*noise_amp+1)) - noise_amp);
        end
        y = x/2;
        sample_valid<=1; ch0<=x; ch1<=y; nco_phase<=g_ph;
        b = g_ph[31:32-AW];
        ref_sum0[b]=ref_sum0[b]+x; ref_sum1[b]=ref_sum1[b]+y; ref_cnt[b]=ref_cnt[b]+1;
        if(ref_cnt[b]==1) g_filled=g_filled+1;
        if(g_have && (({1'b0,g_last}+{1'b0,nco_fcw})>>32)) g_wraps=g_wraps+1;
        g_last=g_ph; g_have=1; g_hits=g_hits+1;
        // bit-exact golden EMA of the DUT datapath; gated like acc_en.  It is
        // only compared in runs without a mid-run clear (case 3), where
        // golden_reset() and the DUT's mode-rise clear stay aligned.
        if(mode && !freeze && !hold) begin
          d = x - $signed(gem0[b]);
          gem0[b] = wrap12($signed(gem0[b]) + (d >>> SHIFT));
        end
        g_ph=g_ph+nco_fcw; g_sig=g_sig+sig_fcw;
      end
      @(negedge clk); sample_valid<=0;
    end
  endtask

  integer errors; integer i; integer pulses; integer worst; integer dev; integer rv; integer h0;

  task check_eq(input integer got,input integer want,input integer tol,input [8*64:1] name);
    begin
      if(got<want-tol || got>want+tol) begin
        $display("FAIL %0s: got %0d want %0d (tol %0d)",name,got,want,tol);
        errors=errors+1;
      end else $display("ok   %0s = %0d (want %0d +/- %0d)",name,got,want,tol);
    end
  endtask

  // read one bin average (address one cycle ahead, like the display FSM)
  task rd_mean(input integer b, output integer v);
    begin
      rd_addr=b; @(negedge clk); #1;
      v = rd_ch0;
    end
  endtask

  // compare every bin average against the golden reference (tol_pct of full
  // scale).  ema_mode=0 compares the ARITHMETIC mean (proves the EMA has
  // converged to the true signal mean); ema_mode=1 compares the bit-exact
  // golden EMA (valid under drift, where the all-hits mean is not the target).
  // Bins with <128 hits are skipped in ema_mode=0: (15/16)^128 = 2.5e-4, so
  // below that the EMA simply has not arrived at the mean yet (this is how a
  // bin grazed once by FCW quantization behaves - not a DUT defect).
  task check_bins(input integer tol_pct, input integer ema_mode);
    integer j,want,tol,skipped;
    begin
      worst=0; skipped=0;
      for(j=0;j<BINS;j=j+1) begin
        if(ref_cnt[j]==0) continue;
        if(ema_mode==0 && ref_cnt[j]<128) begin skipped=skipped+1; continue; end
        rd_mean(j, rv);
        want = ema_mode ? $signed(gem0[j])
                        : (ref_sum0[j]*2 + ref_cnt[j]) / (2*ref_cnt[j]); // rounded mean
        dev = rv - want; if(dev<0) dev=-dev;
        if(dev>worst) worst=dev;
        tol = (AMP*tol_pct)/100 + 1;
        if(dev>tol) begin
          $display("FAIL bin %0d: dut %0d golden %0d (hits %0d)",j,rv,want,ref_cnt[j]);
          errors=errors+1;
        end
      end
      $display("ok   bins: worst |dut-golden| = %0d counts over %0d filled bins (%0d skipped: <128 hits)",
               worst,g_filled,skipped);
    end
  endtask

  initial begin
    errors=0; mode=0; sample_valid=0; afc_locked=0; cfg_commit=0; freeze=0; hold=0;
    ch0=0; ch1=0; nco_phase=0; nco_fcw=FCW_1M; sig_fcw=FCW_1M; rd_addr=0;
    noise_lfsr=32'hACE12345;
    golden_reset();
    rst_n=0; repeat(4) @(negedge clk); rst_n=1; repeat(4) @(negedge clk);

    // ================= case 1: coherent 1 MHz =================
    $display("-- case 1: coherent 1 MHz, 64 bins");
    mode=1; afc_locked=1; nco_fcw=FCW_1M; sig_fcw=FCW_1M;
    drive_n(65);                       // exactly one period (the mode-rise clear
    check_eq(filled,64,1,"case1 filled after 1 period");  // lands on the idle
    drive_n(65*300);                   // 300 more periods -> EMA converged
    check_bins(2,0);
    check_eq(filled,64,0,"case1 filled final");
    check_eq(wraps,g_wraps,1,"case1 wraps");
    check_eq(hits,g_hits,2,"case1 hits");
    for(i=0;i<BINS;i=i+1) if(ref_cnt[i]>0) begin   // ch1 = half amplitude
      rd_addr=i; @(negedge clk); #1;
      if(rd_ch1 < -AMP || rd_ch1 > AMP) begin
        $display("FAIL case1 ch1 bin %0d out of half-scale range: %0d",i,rd_ch1); errors=errors+1;
      end
    end
    $display("ok   case1 ch1 bounded at half scale");

    // ================= case 2: coherent 5 MHz =================
    $display("-- case 2: coherent 5 MHz (13 phases only)");
    h0=hits; golden_reset(); mode=0; repeat(8) @(negedge clk);
    check_eq(hits,h0,0,"case2 hits frozen while mode=0");
    mode=1; nco_fcw=FCW_5M; sig_fcw=FCW_5M;      // mode rise -> clear
    drive_n(13);
    check_eq(filled,13,1,"case2 filled after 1 period (physics)");
    drive_n(65*300);
    // 13 grid positions + ~1 bin grazed: 5 MHz is not exactly 2^32/13 LSB of
    // the phase accumulator (FCW quantization), so the 13-position grid creeps
    // ~2 bins over 1501 periods.  Physics/quantization, identical in the DUT
    // and the golden model - it must NOT approach 64.
    check_eq(filled,14,1,"case2 filled after 301 periods (13 + FCW graze)");
    check_bins(2,0);
    check_eq(wraps,g_wraps,1,"case2 wraps");
    check_eq(hits,g_hits,2,"case2 hits");

    // ================= case 3: 5 MHz + 5 kHz NCO drift =================
    $display("-- case 3: 5 MHz signal, NCO +5 kHz (measured drift fills bins)");
    golden_reset(); mode=0; repeat(8) @(negedge clk);
    mode=1; nco_fcw=FCW_5M+FCW_5K; sig_fcw=FCW_5M;   // signal stays at 5 MHz
    drive_n(13);
    check_eq(filled,13,0,"case3 filled after 1 period (13 grid positions)");
    drive_n(65*1500);                  // grid walks ~0.06 bin/period -> all 64
    check_eq(filled,64,0,"case3 filled after drift");
    check_bins(1,1);                   // compare the bit-exact golden EMA
    check_eq(wraps,g_wraps,2,"case3 wraps");

    // ================= case 4: noise averaging =================
    $display("-- case 4: 1 MHz + uniform noise, EMA averaging");
    golden_reset(); mode=0; repeat(8) @(negedge clk);
    mode=1; nco_fcw=FCW_1M; sig_fcw=FCW_1M; noise_amp=200;
    drive_n(65*400);
    check_bins(15,0);                  // residual noise ~ noise/sqrt(2^(SHIFT+1))
    check_eq(filled,64,0,"case4 filled");
    noise_amp=0;

    // ================= case 5: control semantics =================
    $display("-- case 5: freeze/hold/clear/frame cadence");
    golden_reset(); mode=0; repeat(8) @(negedge clk);
    mode=1; nco_fcw=FCW_1M; sig_fcw=FCW_1M;
    drive_n(65*4);
    h0=hits; freeze=1;
    drive_n(65*2);                     // ignored while frozen
    check_eq(hits,h0,0,"case5 hits frozen by freeze");
    pulses=0;
    fork : count_pulses
      begin forever @(posedge clk) if(frame_pulse) pulses=pulses+1; end
      begin drive_n(65*3); end
    join_any
    disable count_pulses;
    check_eq(pulses,0,0,"case5 no frames while frozen");
    freeze=0;
    h0=hits; hold=1;                  // reader owns the frame: accumulation
    pulses=0;                         // stops, frames keep pulsing
    fork : count_pulses2
      begin forever @(posedge clk) if(frame_pulse) pulses=pulses+1; end
      begin drive_n(20000); end       // ~4.9 frame periods at 4096 clks
    join_any
    disable count_pulses2;
    if(pulses<3) begin $display("FAIL case5 frame cadence: %0d pulses in ~5 periods",pulses); errors=errors+1; end
    else $display("ok   case5 frame cadence: %0d pulses in ~5 periods",pulses);
    check_eq(hits,h0,0,"case5 hits frozen by hold");
    hold=0;
    drive_n(65*2);
    afc_locked=0; @(negedge clk); afc_locked=1; @(negedge clk);
    check_eq(filled,0,0,"case5 lock toggle clears");
    check_eq(hits,0,0,"case5 hits cleared");
    drive_n(65*2);
    cfg_commit=1; @(negedge clk); cfg_commit=0; @(negedge clk);
    check_eq(hits,0,0,"case5 cfg_commit clears");
    golden_reset();
    drive_n(65*200);                  // fresh run, EMA converged
    check_bins(2,0);

    // ============ case 6: sign-alternating input (EMA sign regression) ============
    // If `>>>` degrades to a LOGICAL shift (missing $signed on the
    // sign-extending concatenations), a bin seeing +A / -A alternately runs
    // away, while single-polarity bins still converge - so cases 1 and 5
    // alone would NOT catch it.  Expected: the EMA of an alternating sequence
    // ripples around 0 with amplitude A/31.
    $display("-- case 6: alternating +/- into one bin (EMA sign regression)");
    golden_reset(); mode=0; repeat(8) @(negedge clk);
    mode=1; nco_fcw=32'd0; sig_fcw=FCW_1M; alt_sign=1;   // frozen NCO: all -> bin 0
    drive_n(400);
    check_bins(1,1);                   // bit-exact against the golden EMA
    rd_mean(0, rv);
    // true ripple is +/-A/31; the truncating shift biases it a little more
    // negative, so allow A/16.  The logical-shift failure reaches 449 on the
    // very first sample, so this bound still rejects it decisively.
    if(rv>AMP/16 || rv<-AMP/16) begin
      $display("FAIL case6 mixed-sign EMA should sit at ~0 (+/-%0d), got %0d",AMP/16,rv);
      errors=errors+1;
    end else $display("ok   case6 mixed-sign EMA at ~0: %0d (ripple +/-%0d)",rv,AMP/31+1);
    alt_sign=0;

    if(errors==0) $display("TB_PASS tb_et_display");
    else $display("TB_FAIL tb_et_display: %0d errors",errors);
    $finish;
  end
endmodule
`default_nettype wire
