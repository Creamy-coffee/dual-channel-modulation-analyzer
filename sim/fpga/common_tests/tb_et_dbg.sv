`timescale 1ns/1ps
// Debug bench: per-sample visibility into the DUT internals vs the golden
// model.  Reproduces the case-3 style setup (5 MHz signal, NCO +5 kHz) with
// no noise and dumps bin/avg/cnt for every sample, flagging the first sample
// where the DUT and the golden EMA disagree.
`default_nettype none
module tb_et_dbg;
  localparam integer AW=6, BINS=1<<AW, SHIFT=4, FRAME_CLKS=4096;
  localparam [31:0] FCW_5M  = 32'd330382093;
  localparam [31:0] FCW_5K  = 32'd330382;
  localparam [31:0] FCW_TOT = FCW_5M+FCW_5K;
  localparam integer AMP=1000;

  reg clk=0,rst_n=0;
  always #7.6923076923 clk=~clk;

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

  // golden (bit-exact EMA + exact counters)
  reg [11:0] gem0[0:BINS-1];
  integer ref_cnt[0:BINS-1];
  reg [31:0] g_ph, g_sig;
  integer n, x, b, d, bad;

  function integer wrap12(input integer a);
    begin
      wrap12 = a & 12'hFFF;
      if(wrap12 >= 2048) wrap12 = wrap12 - 4096;
    end
  endfunction

  initial begin
    bad=0;
    mode=0; sample_valid=0; afc_locked=0; cfg_commit=0; freeze=0; hold=0;
    ch0=0; ch1=0; nco_phase=0; nco_fcw=FCW_TOT; rd_addr=0;
    g_ph=0; g_sig=0;
    for(b=0;b<BINS;b=b+1) begin gem0[b]=0; ref_cnt[b]=0; end
    rst_n=0; repeat(4) @(negedge clk); rst_n=1; repeat(4) @(negedge clk);
    mode=1; afc_locked=1;
    for(n=0;n<400;n=n+1) begin
      @(negedge clk);
      x = (g_sig[31]==1'b0) ? AMP : -AMP;
      sample_valid<=1; ch0<=x; nco_phase<=g_ph;
      b = g_ph[31:26];
      // golden step (same arithmetic as DUT)
      d = x - $signed(gem0[b]);
      gem0[b] = wrap12($signed(gem0[b]) + (d >>> SHIFT));
      ref_cnt[b]=ref_cnt[b]+1;
      #1;
      if(dut.bin !== b)
        $display("n=%0d BIN MISMATCH: golden %0d dut %0d (ph=%0d)",n,b,dut.bin,g_ph);
      if(dut.acc_en !== 1'b1)
        $display("n=%0d ACC_EN=%b clear=%b (ph=%0d)",n,dut.acc_en,dut.clear,g_ph);
      // NOTE: dut.* shown here is the state BEFORE this sample lands;
      // gem0[b] below has already taken this sample's step.
      $display("n=%0d x=%0d ph=%08h bin=%0d | dut avg=%0d cnt=%0d | golden ema=%0d cnt=%0d",
               n,x,g_ph,b,dut.avg0[b],dut.cnt[b],
               $signed(gem0[b]), ref_cnt[b]);
      g_ph=g_ph+nco_fcw; g_sig=g_sig+FCW_5M;
    end
    @(negedge clk); sample_valid<=0;
    // final per-bin comparison after everything settled
    $display("---- final bin comparison ----");
    for(b=0;b<BINS;b=b+1) begin
      rd_addr=b; @(negedge clk); #1;
      if(ref_cnt[b]>0 && rd_ch0 !== $signed(gem0[b])) begin
        $display("BIN %0d: dut %0d golden %0d (cnt dut %0d golden %0d)",
                 b,rd_ch0,$signed(gem0[b]),dut.cnt[b],ref_cnt[b]);
        bad=bad+1;
      end
    end
    if(bad==0) $display("DBG_PASS"); else $display("DBG_FAIL %0d bins",bad);
    $finish;
  end
endmodule
`default_nettype wire
