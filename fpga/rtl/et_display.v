`timescale 1ns/1ps
`default_nettype none
// ET revision 2: Q8 states, real-hit validity, bounded phase history.
// Two live banks use distributed RAM. A frozen bank is serialized into a
// BRAM snapshot by ONE pair of rounding units; the other bank keeps acquiring.
// No memory is globally reset. Seen masks discard stale epoch contents.
module et_display #(
 parameter integer ET_AW=6,ET_AVG_SHIFT=4,ET_FRAME_CLKS=2097152,CLK_FREQ=65000000,
 parameter integer FRAC=8,MIN_HITS=128,UNCERTAINTY_FCW=529,MAX_RESIDUAL_FCW=1322
)(
 input wire clk,rst_n,mode,sample_valid,
 input wire signed [11:0] ch0,ch1,
 input wire [31:0] nco_phase,nco_fcw,
 input wire afc_locked,reference_ok,
 input wire signed [31:0] residual0_fcw,residual1_fcw,
 input wire cfg_commit,freeze,hold,
 output reg frame_pulse,output reg [15:0] frame_id,
 output reg [ET_AW:0] filled,ready_bins,
 output reg [31:0] wraps,hits,epoch_id,
 output reg snapshot_locked,snapshot_reference_ok,
 input wire [9:0] rd_addr,
 output wire signed [11:0] rd_ch0,rd_ch1,
 output wire rd_valid,output wire [15:0] rd_hits,dbg_cnt
);
 localparam integer BINS=1<<ET_AW,W=12+FRAC,LW=2*W+16;
 localparam [31:0] FRAME_MAX=ET_FRAME_CLKS-1,WAIT_MAX=FRAME_MAX+2*MIN_HITS*BINS;
 localparam [32:0] PHASE_BUDGET=33'd1<<(32-ET_AW-2);
 (* ram_style="distributed" *) reg [LW-1:0] live[0:2*BINS-1];
 (* ram_style="block" *) reg [40:0] snapshot[0:BINS-1];
 reg [40:0] read_word;
 reg bank,copy_bank,copy_busy,have_snapshot;
 reg [ET_AW-1:0] copy_addr;
 reg [BINS-1:0] seen0,seen1,frozen_seen;
 reg [ET_AW:0] live_filled,live_ready;
 reg [31:0] live_wraps,live_hits,live_epoch,timer,age,last_phase,last_fcw;
 reg [32:0] phase_spent;
 reg have_last,mode_q,lock_q,ref_q,freeze_q;
 function [31:0] abs32;
  input signed [31:0] v;begin abs32=v[31]?-v:v;end
 endfunction
 wire [31:0] ar0=abs32(residual0_fcw),ar1=abs32(residual1_fcw);
 wire [31:0] worst_residual=ar0>ar1?ar0:ar1;
 wire phase_ok=afc_locked && reference_ok && nco_fcw!=0 &&
               ar0<=MAX_RESIDUAL_FCW && ar1<=MAX_RESIDUAL_FCW;
 wire [32:0] phase_next=phase_spent+{1'b0,worst_residual}+UNCERTAINTY_FCW;
 wire restart=(mode!=mode_q)||(afc_locked!=lock_q)||(phase_ok!=ref_q)||
   (freeze!=freeze_q)||cfg_commit||(have_last && nco_fcw!=last_fcw)||
   (mode && (!sample_valid || !phase_ok))||(age>=FRAME_MAX)||(phase_next>=PHASE_BUDGET);
 wire take_snapshot=mode && !freeze && !hold && !copy_busy && !frame_pulse &&
   timer>=FRAME_MAX && (!phase_ok || live_hits>=(MIN_HITS+2)*BINS || timer>=WAIT_MAX);
 // The bank-switch clock is excluded from ET accumulation and counted in
 // neither epoch. RAW acquisition is independent and never pauses.
 wire accumulate=mode && phase_ok && sample_valid && !freeze && !restart && !take_snapshot;
 wire [ET_AW-1:0] bin=nco_phase[31:32-ET_AW];
 wire live_seen=bank?seen1[bin]:seen0[bin];
 wire [LW-1:0] word=live[{bank,bin}];
 wire [15:0] count=live_seen?word[LW-1:2*W]:16'd0;
 wire signed [W-1:0] a0=live_seen?$signed(word[W-1:0]):{W{1'b0}};
 wire signed [W-1:0] a1=live_seen?$signed(word[2*W-1:W]):{W{1'b0}};
 wire signed [W:0] x0=$signed({ch0[11],ch0,{FRAC{1'b0}}});
 wire signed [W:0] x1=$signed({ch1[11],ch1,{FRAC{1'b0}}});
 wire signed [W:0] old0=$signed({a0[W-1],a0}),old1=$signed({a1[W-1],a1});
 wire signed [W:0] d0=x0-old0,d1=x1-old1;
 function signed [W:0] ema_step;
  input signed [W:0] d;
  reg signed [W:0] q;
  reg inc;
  begin
   q=d>>>ET_AVG_SHIFT;
   inc=d[ET_AVG_SHIFT-1] && (!d[W] || (|(d & ((1<<(ET_AVG_SHIFT-1))-1))));
   ema_step=q+$signed({1'b0,inc});
  end
 endfunction
 function signed [11:0] round_count;
  input signed [W-1:0] a;
  reg signed [W:0] q;
  reg inc;
  begin
   q=$signed(a)>>>FRAC;
   inc=a[FRAC-1] && (!a[W-1] || (|a[FRAC-2:0]));
   q=q+$signed({1'b0,inc});
   if(q>2047)round_count=12'sh7ff;
   else if(q< -2048)round_count=12'sh800;
   else round_count=q[11:0];
  end
 endfunction
 wire signed [W:0] next0=old0+ema_step(d0),next1=old1+ema_step(d1);
 wire [15:0] next_count=count==16'hffff ? count : count+1'b1;
 wire [LW-1:0] write_word={next_count,(live_seen?next1[W-1:0]:x1[W-1:0]),
                                         (live_seen?next0[W-1:0]:x0[W-1:0])};
 wire [LW-1:0] frozen_word=live[{copy_bank,copy_addr}];
 wire frozen_valid=frozen_seen[copy_addr];
 wire [15:0] frozen_count=frozen_valid?frozen_word[LW-1:2*W]:16'd0;
 // Only two converters exist, independent of the number of phase bins.
 wire signed [11:0] frozen0=round_count($signed(frozen_word[W-1:0]));
 wire signed [11:0] frozen1=round_count($signed(frozen_word[2*W-1:W]));
 wire [LW-1:0] debug_word=live[{bank,rd_addr[ET_AW-1:0]}];
 assign dbg_cnt=(bank?seen1[rd_addr[ET_AW-1:0]]:seen0[rd_addr[ET_AW-1:0]])?debug_word[LW-1:2*W]:16'd0;
 assign rd_ch0=have_snapshot?$signed(read_word[11:0]):12'sd0;
 assign rd_ch1=have_snapshot?$signed(read_word[23:12]):12'sd0;
 assign rd_hits=have_snapshot?read_word[39:24]:16'd0;
 assign rd_valid=have_snapshot && read_word[40];
 always @(posedge clk) begin
  if(accumulate)live[{bank,bin}]<=write_word;
  if(copy_busy)snapshot[copy_addr]<={frozen_valid && frozen_count>=MIN_HITS,frozen_count,frozen1,frozen0};
  read_word<=snapshot[rd_addr[ET_AW-1:0]];
 end
 always @(posedge clk) begin
  frame_pulse<=0;
  if(!rst_n)begin
   bank<=0;copy_bank<=0;copy_busy<=0;copy_addr<=0;have_snapshot<=0;
   seen0<=0;seen1<=0;frozen_seen<=0;
   live_filled<=0;live_ready<=0;live_wraps<=0;live_hits<=0;live_epoch<=0;
   timer<=0;age<=0;phase_spent<=0;have_last<=0;last_phase<=0;last_fcw<=0;
   mode_q<=0;lock_q<=0;ref_q<=0;freeze_q<=0;
   frame_id<=0;filled<=0;ready_bins<=0;wraps<=0;hits<=0;epoch_id<=0;
   snapshot_locked<=0;snapshot_reference_ok<=0;
  end else begin
   mode_q<=mode;lock_q<=afc_locked;ref_q<=phase_ok;freeze_q<=freeze;
   if(mode && !freeze && !hold && !copy_busy && !frame_pulse)begin
    if(timer<WAIT_MAX)timer<=timer+1'b1;
   end else if(!mode)timer<=0;
   if(copy_busy)begin
    if(&copy_addr)begin
     copy_busy<=0;frame_pulse<=1;frame_id<=frame_id+1'b1;have_snapshot<=1;
    end else copy_addr<=copy_addr+1'b1;
   end
   if(take_snapshot)begin
    timer<=0;copy_busy<=1;copy_addr<=0;copy_bank<=bank;
    frozen_seen<=restart?{BINS{1'b0}}:(bank?seen1:seen0);
    snapshot_locked<=afc_locked;snapshot_reference_ok<=phase_ok && !restart;
    filled<=restart?0:live_filled;ready_bins<=restart?0:live_ready;
    wraps<=restart?0:live_wraps;hits<=restart?0:live_hits;epoch_id<=live_epoch;
    bank<=!bank;
    if(bank)seen0<=0;else seen1<=0;
   end
   if(restart || take_snapshot)begin
    if(restart && !take_snapshot)begin if(bank)seen1<=0;else seen0<=0;end
    live_filled<=0;live_ready<=0;live_wraps<=0;live_hits<=0;
    age<=0;phase_spent<=0;have_last<=0;
    if(live_hits!=0)live_epoch<=live_epoch+1'b1;
   end else begin
    if(have_last && mode && !freeze)begin age<=age+1'b1;phase_spent<=phase_next;end
    if(accumulate)begin
     if(bank)seen1[bin]<=1;else seen0[bin]<=1;
     if(!live_seen)live_filled<=live_filled+1'b1;
     if(count==MIN_HITS-1)live_ready<=live_ready+1'b1;
     if(live_hits!=32'hffffffff)live_hits<=live_hits+1'b1;
     if(have_last && nco_phase<last_phase && live_wraps!=32'hffffffff)live_wraps<=live_wraps+1'b1;
     last_phase<=nco_phase;last_fcw<=nco_fcw;have_last<=1;
    end
   end
  end
 end
 // synthesis translate_off
 initial begin
  if(ET_AW<1 || ET_AW>9 || FRAC<ET_AVG_SHIFT || FRAC<2 || FRAC>12 ||
     ET_AVG_SHIFT<1 || ET_AVG_SHIFT>8 || MIN_HITS<1 || MIN_HITS>65535 || ET_FRAME_CLKS<16)
    $fatal(1,"Invalid ET parameters");
 end
 // synthesis translate_on
endmodule
`default_nettype wire
