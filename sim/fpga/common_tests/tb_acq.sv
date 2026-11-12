`timescale 1ns/1ps
module tb_acq;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,valid=0,freeze=0,busy=0;reg [1:0] fault=0;
 reg signed [11:0] c0=0,c1=0,level=0;reg [1:0] edge_mode=0;
 reg src=0;reg [4:0] pre=8,addr=0;reg [3:0] mode=0;
 wire signed [11:0] q0,q1;wire ready,filtered,aut;
 wire [15:0] frame;wire [31:0] dt,errs;wire [4:0] pos;wire [3:0] rate;
 wire [63:0] first;
 integer tick=0,frames=0,i,m,p,j,idx,r,e0,e1,saved_errors,pattern=0;
 integer hist0[0:500000],hist1[0:500000];
 // Scratch area for the UART snapshot immutability check (the frame that has
 // been handed to the reader must not change underneath it).  Named "cic_"
 // historically, when this bench was checking a CIC; the decimator is a boxcar
 // now, but the name is kept so the diffs stay readable.
 reg [23:0] saved[0:31];reg [15:0] saved_frame;
 acq_trigger_buffer #(.DISP_AW(5),.AUTO_TIMEOUT(8000)) dut(
  .clk(clk),.rst_n(rst),.sample_valid(valid),.ch0(c0),.ch1(c1),.fault(fault),
  .trig_level(level),.trig_mode(edge_mode),.trig_src(src),.pretrig(pre),
  .timebase_log2(mode),.freeze(freeze),.disp_busy(busy),.snap_rd_addr(addr),
  .snap_rd_ch0(q0),.snap_rd_ch1(q1),.snap_ready(ready),.snap_frame_id(frame),
  .snap_sample_interval_ns(dt),.snap_trig_pos(pos),.snap_filtered(filtered),
  .snap_fault(),.snap_decim_log2(rate),.snap_auto_trigger(aut),
  .snap_first_index(first),.capture_errors(errs));
 always @(negedge clk)begin
  c0=pattern?-2048:((tick*7)%4096)-2048;
  c1=pattern?2047:2047-((tick*13)%4096);
 end
 // hist[k] is the sample present on the k-th posedge.  Everything is sampled
 // in ONE block on the posedge: c0/c1 were last driven on the preceding
 // negedge, so reading them together with `tick` keeps the pair aligned.
 // (A separate negedge latch for the index samples tick BEFORE the edge that
 // increments it, which silently shifted the whole history by one sample.)
 always @(posedge clk) if(rst) begin
  hist0[tick]=$signed(c0);hist1[tick]=$signed(c1);tick=tick+1;
 end else if(!rst) tick=0;
 // ---- exact reference for the decimator actually in the RTL ----
 // The display decimation is an accumulate-and-dump boxcar: the point that
 // ends at input sample `last` is
 //   point = ( sum of x[last-r+1 .. last] ) >> m
 // which is bit-exact identity at r=1 (m=0).  See
 // reports/decimator_findings_20261019.md for why this is NOT a CIC and why
 // the earlier CIC^3 claim was withdrawn.
 task decim_expect;
  input integer last,r;
  integer k,v0,v1,sh;
  begin
   v0=0;v1=0;
   for(k=0;k<r;k=k+1)begin
    if(last-k>=0)begin v0=v0+hist0[last-k];v1=v1+hist1[last-k];end
   end
   sh=mode;
   e0=v0>>>sh;e1=v1>>>sh;
  end
 endtask
 task check_frame;
  begin
   wait(ready);#1;busy=1;r=1<<rate;
   if(pos!==pre || rate!==mode || filtered!==(mode!=0))$fatal(1,"metadata");
   e0=$rtoi(1000000000.0*r/65000000.0+0.5);
   if(dt!=e0)$fatal(1,"time interval units dt=%0d expected=%0d",dt,e0);
   for(i=0;i<32;i=i+1)begin
    @(negedge clk);addr=i;repeat(2)@(posedge clk);#1;

    // Row i is the block that ends at input sample `first+i*r`.  first_index is
    // defined as the tick of the point AFTER the trigger point, and the reader
    // walks forward from start_addr = wr_ptr - pre_q, which is exactly that
    // point, so the two agree without any extra offset.
    idx=first+i*r;
    decim_expect(idx,r);
    if(frames==0 && i==0)begin
      $display("D hist75..70=%0d,%0d,%0d,%0d,%0d,%0d",hist0[75],hist0[74],hist0[73],hist0[72],hist0[71],hist0[70]);
      $display("D ram 9,8,7,6,5,4=%0d,%0d,%0d,%0d,%0d,%0d",
        $signed(dut.samples[9][11:0]),$signed(dut.samples[8][11:0]),$signed(dut.samples[7][11:0]),
        $signed(dut.samples[6][11:0]),$signed(dut.samples[5][11:0]),$signed(dut.samples[4][11:0]));
    end

    if(q0!==e0[11:0] || q1!==e1[11:0])$fatal(1,"sample mismatch mode=%0d row=%0d first=%0d last=%0d got=%0d/%0d expected=%0d/%0d",mode,i,first,idx,q0,q1,e0,e1);
    saved[i]={q1,q0};
   end
   // Live ADC keeps changing; all exported samples must remain immutable.
   saved_frame=frame;repeat(50)@(negedge clk);
   for(i=0;i<32;i=i+1)begin
    @(negedge clk);addr=i;repeat(2)@(posedge clk);#1;
    if({q1,q0}!==saved[i] || frame!=saved_frame)$fatal(1,"reader ownership violated");
   end
   frames=frames+1;@(negedge clk);busy=0;
  end
 endtask
 initial begin
  repeat(5)@(negedge clk);rst=1;valid=1;
  // Exact bit comparisons for all supported strides and both edge directions.
  for(m=0;m<8;m=m+1)begin
   for(p=0;p<3;p=p+1)begin
    mode=m;pre=p==0?0:(p==1?8:31);edge_mode=p==2?1:0;src=p[0];
    check_frame();repeat(5)@(negedge clk);
   end
  end
  // Constant signal: no real crossing, bounded auto trigger.
  level=2047;pre=8;mode=0;
  check_frame();if(!aut)$fatal(1,"missing auto trigger flag");
  // Worst-case signed accumulator magnitudes at the maximum display stride.
  pattern=1;mode=7;check_frame();pattern=0;
  // Freeze must not create a new frame, then resume correctly.
  freeze=1;saved_frame=frame;repeat(100)@(negedge clk);
  if(frame!=saved_frame || ready)$fatal(1,"freeze failed");
  freeze=0;level=0;check_frame();
  // Reset a partially acquired history on a fault or data gap.
  mode=7;pre=31;repeat(50)@(negedge clk);fault=1;
  saved_errors=errs;repeat(5)@(negedge clk);fault=0;
  if(errs<=saved_errors)$fatal(1,"fault not counted");
  repeat(50)@(negedge clk);valid=0;repeat(4)@(negedge clk);valid=1;
  check_frame();
  $display("PASS acquisition: %0d exact frames, R1..128, both channels, pretrigger0/8/31, edges, auto, freeze, fault, gaps, immutable UART snapshot",frames);$finish;
 end
 initial begin #10000000;$fatal(1,"acquisition timeout");end
endmodule
