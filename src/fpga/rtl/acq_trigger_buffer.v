`timescale 1ns/1ps
`default_nettype none
// Independent online display sampler and trigger ring. No CNN dependency.
//
// DECIMATION: R=1 is native RAW.  R>1 is a BOXCAR average of R consecutive
// samples - an accumulate-and-dump, NOT an anti-alias filter.
//
// This header used to claim a 3rd-order CIC with "-40 dB worst-case alias
// rejection".  That claim was measured and is FALSE for this configuration,
// so it has been withdrawn.  See reports/decimator_findings_20261019.md.
// Summary of what was actually measured:
//   * A normalised CIC (M=R) with the downsampler moved before the combs does
//     NOT reproduce a boxcar(R)^3 kernel.  At R=4 the decimated kernel is
//     [10,6]/16 -> normalised [0.0625, 0.75, 0.1875]: DC gain R^2, not R^3, and
//     the passband droop is nothing like the Sinc^3 the CIC definition predicts.
//   * The full boxcar^3 kernel (length R^3, DC gain R^3) only exists when the
//     decimation is taken AFTER all three combs, i.e. the comb delays are R
//     INPUT samples.  Sampling that kernel at stride R (which is what a
//     downsampler does) keeps only [10,6] of it and drops 3/4 of the DC gain.
//   * Forming the whole 104-bit cascade combinationally in one clock does not
//     close timing: the routed build reported 41 logic levels and 15.291 ns
//     against a 15.385 ns period (WNS = -0.077 ns).
// Therefore the honest, shipping behaviour is the boxcar that is implemented
// below: it is exact, it costs one adder, it closes timing, and it is NOT
// presented as an anti-alias filter.
//
// ADC consumption never depends on UART. Only this historical display window
// stops writing while its reader owns it; the ADC and measurement path run on.
module acq_trigger_buffer #(
 parameter integer SAMPLE_W=12, MAIN_AW=14, MAIN_DEPTH=(1<<MAIN_AW),
 parameter integer DISP_AW=10, DISP_N=(1<<DISP_AW),
 parameter integer AUTO_TIMEOUT=3250000, T_SMP_NS=15385,
 parameter integer MAX_DECIM_LOG2=15, CLK_FREQ=65000000
)(
 input wire clk,rst_n,sample_valid,
 input wire signed [SAMPLE_W-1:0] ch0,ch1,input wire [1:0] fault,
 input wire signed [SAMPLE_W-1:0] trig_level,
 input wire [1:0] trig_mode,input wire trig_src,
 input wire [DISP_AW-1:0] pretrig,input wire [3:0] timebase_log2,
 input wire freeze,disp_busy,input wire [DISP_AW-1:0] snap_rd_addr,
 output reg signed [SAMPLE_W-1:0] snap_rd_ch0,snap_rd_ch1,
 output reg snap_ready,output reg [15:0] snap_frame_id,
 output reg [31:0] snap_sample_interval_ns,
 output reg [DISP_AW-1:0] snap_trig_pos,
 output reg snap_filtered,output reg [1:0] snap_fault,
 output reg [3:0] snap_decim_log2,output reg snap_auto_trigger,
 output reg [63:0] snap_first_index,output reg [31:0] capture_errors
);
 function [31:0] interval_for_mode;
  input [3:0] m;
  begin case(m)
   4'd0: interval_for_mode=(64'd1000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd1: interval_for_mode=(64'd2000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd2: interval_for_mode=(64'd4000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd3: interval_for_mode=(64'd8000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd4: interval_for_mode=(64'd16000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd5: interval_for_mode=(64'd32000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd6: interval_for_mode=(64'd64000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd7: interval_for_mode=(64'd128000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd8: interval_for_mode=(64'd256000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd9: interval_for_mode=(64'd512000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd10: interval_for_mode=(64'd1024000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd11: interval_for_mode=(64'd2048000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd12: interval_for_mode=(64'd4096000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd13: interval_for_mode=(64'd8192000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd14: interval_for_mode=(64'd16384000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd15: interval_for_mode=(64'd32768000000000+CLK_FREQ/2)/CLK_FREQ;
   default: interval_for_mode=0;
  endcase end
 endfunction
 localparam FILL=0,ARMED=1,POST=2,WAIT_READER=3,HOLD_READER=4;
 localparam ACC_W=SAMPLE_W+MAX_DECIM_LOG2+1;
 localparam COUNT_W=MAX_DECIM_LOG2<1?1:MAX_DECIM_LOG2;
 reg [2:0] state;
 (* ram_style="block" *) reg [2*SAMPLE_W-1:0] samples[0:DISP_N-1];
 reg [DISP_AW-1:0] wr_ptr,start_addr;
 reg [DISP_AW:0] filled,remaining;
 reg [3:0] mode_q;
 reg [DISP_AW-1:0] pre_q;
 reg src_q;reg [1:0] edge_q;
 reg signed [SAMPLE_W-1:0] level_q,prev;
 reg have_prev;
 reg [COUNT_W-1:0] sub_count;
 reg signed [ACC_W-1:0] sum0,sum1;
 reg [31:0] wait_clks;
 reg [63:0] tick;
 wire [3:0] mode_requested=timebase_log2>MAX_DECIM_LOG2?MAX_DECIM_LOG2:timebase_log2;
 wire changed=(mode_q!=mode_requested || pre_q!=pretrig ||
               src_q!=trig_src || edge_q!=trig_mode || level_q!=trig_level);
 wire [COUNT_W-1:0] last_sub=(({{COUNT_W{1'b0}},1'b1})<<mode_q)-1'b1;
 wire signed [ACC_W-1:0] next0=sum0+{{(ACC_W-SAMPLE_W){ch0[SAMPLE_W-1]}},ch0};
 wire signed [ACC_W-1:0] next1=sum1+{{(ACC_W-SAMPLE_W){ch1[SAMPLE_W-1]}},ch1};
 wire signed [SAMPLE_W-1:0] point0=next0>>>mode_q;
 wire signed [SAMPLE_W-1:0] point1=next1>>>mode_q;
 wire signed [SAMPLE_W-1:0] selected=src_q?point1:point0;
 wire rising=have_prev && prev<level_q && selected>=level_q;
 wire falling=have_prev && prev>level_q && selected<=level_q;
 wire crossed=edge_q==0?rising:(edge_q==1?falling:(rising||falling));
 wire point_valid=sample_valid && !(|fault) && sub_count==last_sub;
 wire writing=state<=POST && !changed && !freeze;
 wire auto_fire=wait_clks>=AUTO_TIMEOUT-1;
 wire [DISP_AW-1:0] rd_phys=start_addr+snap_rd_addr;
 // Neither RAM nor its output register has an async reset, allowing BRAM.
 always @(posedge clk) begin
  if(rst_n && point_valid && writing) samples[wr_ptr]<={point1,point0};
  {snap_rd_ch1,snap_rd_ch0}<=samples[rd_phys];
 end
 always @(posedge clk) begin
  snap_ready<=0;
  if(!rst_n) begin
   state<=FILL;wr_ptr<=0;start_addr<=0;filled<=0;remaining<=0;
   mode_q<=0;pre_q<=0;src_q<=0;edge_q<=0;level_q<=0;
   prev<=0;have_prev<=0;sub_count<=0;sum0<=0;sum1<=0;wait_clks<=0;
   tick<=0;snap_frame_id<=0;snap_sample_interval_ns<=0;
   snap_trig_pos<=0;snap_filtered<=0;snap_fault<=0;
   snap_decim_log2<=0;snap_auto_trigger<=0;snap_first_index<=0;capture_errors<=0;
  end else begin
   tick<=tick+1'b1;
   // A committed buffer and its metadata are immutable until reader release.
   if(state==WAIT_READER) begin
    if(disp_busy)state<=HOLD_READER;
   end else if(state==HOLD_READER) begin
    if(!disp_busy && !freeze)begin
     state<=FILL;filled<=0;wr_ptr<=0;sub_count<=0;sum0<=0;sum1<=0;
     have_prev<=0;wait_clks<=0;
    end
   end else if(changed || freeze || !sample_valid || (|fault)) begin
    if((|fault) || (!sample_valid && filled!=0))
     if(capture_errors!=32'hffffffff)capture_errors<=capture_errors+1'b1;
    state<=FILL;filled<=0;wr_ptr<=0;sub_count<=0;sum0<=0;sum1<=0;
    have_prev<=0;wait_clks<=0;
    mode_q<=mode_requested;pre_q<=pretrig;src_q<=trig_src;
    edge_q<=trig_mode;level_q<=trig_level;
   end else begin
    if(state==ARMED && wait_clks<AUTO_TIMEOUT)wait_clks<=wait_clks+1'b1;
    if(sample_valid) begin
     if(point_valid)begin sub_count<=0;sum0<=0;sum1<=0;end
     else begin sub_count<=sub_count+1'b1;sum0<=next0;sum1<=next1;end
    end
    if(point_valid) begin
     wr_ptr<=wr_ptr+1'b1;prev<=selected;have_prev<=1;
     case(state)
      FILL:begin
       if(filled<DISP_N)filled<=filled+1'b1;
       if(filled>=pre_q)begin state<=ARMED;wait_clks<=0;end
      end
      ARMED:if(crossed || auto_fire)begin
       start_addr<=wr_ptr-pre_q;
       remaining<=DISP_N-1-pre_q;
       snap_first_index<=tick-({{(64-DISP_AW){1'b0}},pre_q}<<mode_q);
       snap_trig_pos<=pre_q;snap_decim_log2<=mode_q;
       snap_auto_trigger<=!crossed;snap_filtered<=mode_q!=0;snap_fault<=0;
       snap_sample_interval_ns<=interval_for_mode(mode_q);
       if(pre_q==DISP_N-1)begin
        snap_frame_id<=snap_frame_id+1'b1;snap_ready<=1;state<=WAIT_READER;
       end else state<=POST;
      end
      POST:begin
       remaining<=remaining-1'b1;
       if(remaining==1)begin
        snap_frame_id<=snap_frame_id+1'b1;snap_ready<=1;state<=WAIT_READER;
       end
      end
      default:state<=FILL;
     endcase
    end
   end
  end
 end
 // synthesis translate_off
 initial begin
  if(DISP_N!=(1<<DISP_AW) || SAMPLE_W<2 || CLK_FREQ<=0 || AUTO_TIMEOUT<1 || MAX_DECIM_LOG2<0 || MAX_DECIM_LOG2>15)
   $fatal(1,"Invalid acquisition parameters");
 end
 // synthesis translate_on
endmodule
`default_nettype wire
