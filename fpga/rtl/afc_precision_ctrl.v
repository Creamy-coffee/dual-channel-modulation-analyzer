`timescale 1ns/1ps
`default_nettype none
// Fine frequency feedback from NUMERICALLY valid lock-in phases (not qual).
// The independent short observer retains ownership of acquisition/loss/range.
// Only a constant-window, constant-NCO epoch enters this estimate.  A 2^24
// physical-clock baseline gives sub-Hz precision without shrinking the noisy
// 4096-sample observer deadband. No ADC samples or phase measurements change.
module afc_precision_ctrl #(
 parameter integer BASELINE_LOG2=24, DEAD_FCW=4, WAKE_FCW=8,
 parameter integer MAX_STEP_FCW=529, AGREE_FCW=26, CAPTURE_FCW=132153,
 parameter [23:0] MIN_AMP=24'd2048
)(
 input wire clk,rst_n,clear,reconfig,
 input wire in_valid,
 input wire [31:0] timestamp,current_fcw,nominal_fcw,
 input wire [5:0] window_log2,
 input wire signed [15:0] phase0,phase1,
 input wire [1:0] phase_valid,
 input wire [23:0] amp0,amp1,
 output reg commit,
 output reg [31:0] target_fcw,
 output reg ready,estimate_valid,
 output reg signed [31:0] error0_fcw,error1_fcw,
 output reg [7:0] reason,
 output reg [31:0] estimates,updates
);
 reg previous;
 reg [31:0] prevtime,anchor_time,age;
 reg [5:0] previous_window;
 reg signed [15:0] prev0,prev1;
 reg signed [31:0] sum0,sum1;
 reg [1:0] skip,good;
 wire signed [15:0] dp0=phase0-prev0,dp1=phase1-prev1;
 wire signed [31:0] total0=sum0+{{16{dp0[15]}},dp0};
 wire signed [31:0] total1=sum1+{{16{dp1[15]}},dp1};
 wire [31:0] dt=timestamp-anchor_time;
 wire [31:0] period_clocks=32'd1<<window_log2;
 function [31:0] ab;
  input signed [31:0] v; begin ab=v[31]?-v:v;end
 endfunction
 function signed [31:0] to_fcw;
  input signed [31:0] v;
  reg signed [31:0] quotient;
  reg [31:0] half;
  reg increment;
  begin
   if(BASELINE_LOG2>16) begin
    quotient=v>>>(BASELINE_LOG2-16);
    half=32'd1<<(BASELINE_LOG2-17);
    // Signed floor + guard/sticky gives nearest, ties away from zero.
    // One narrow increment replaces abs/round/negate carry chains.
    increment=((v & half)!=0) && (!v[31] || ((v & (half-1'b1))!=0));
    to_fcw=quotient+$signed({31'd0,increment});
   end else if(BASELINE_LOG2<16) to_fcw=v<<<(16-BASELINE_LOG2);
   else to_fcw=v;
  end
 endfunction
 reg signed [31:0] e0,e1,phase_total0,phase_total1;
 reg calc1,calc2,apply,proposal_allowed;
 reg [31:0] proposal_nominal;
 reg signed [33:0] proposed,offset;
 always @(posedge clk) begin
  commit<=0;calc1<=0;calc2<=0;apply<=0;
  if(!rst_n || clear) begin
   previous<=0;prevtime<=0;anchor_time<=0;age<=0;previous_window<=0;
   prev0<=0;prev1<=0;sum0<=0;sum1<=0;skip<=2;good<=0;
   target_fcw<=nominal_fcw;ready<=0;estimate_valid<=0;
   error0_fcw<=0;error1_fcw<=0;reason<=1;estimates<=0;updates<=0;
   e0<=0;e1<=0;phase_total0<=0;phase_total1<=0;proposed<=0;
   proposal_nominal<=0;proposal_allowed<=0;
  end else begin
   if(age!=32'hffffffff) age<=age+1'b1;
   // Feedback must remain fresh. Allows long-window flush/settling, not an
   // unlimited old fine-lock permission. Main ADC/coarse loss clears sooner.
   if(age>=(32'd1<<(BASELINE_LOG2+3))) begin
    ready<=0;estimate_valid<=0;previous<=0;good<=0;reason<=8;
   end
   if(reconfig || commit) begin
    // A gear change preserves the frequency certificate, but cannot combine
    // old/new window centres. A fine tune already withdrew ready below.
    previous<=0;sum0<=0;sum1<=0;skip<=2;good<=0;
    calc1<=0;calc2<=0;apply<=0;
   end else begin
    if(in_valid) begin
    age<=0;
    if(phase_valid!=3 || amp0<MIN_AMP || amp1<MIN_AMP) begin
     ready<=0;estimate_valid<=0;previous<=0;good<=0;skip<=2;reason<=1;
    end else if(skip!=0) begin skip<=skip-1'b1;previous<=0;end
    else if(!previous || previous_window!=window_log2) begin
     previous<=1;prevtime<=timestamp;anchor_time<=timestamp;
     prev0<=phase0;prev1<=phase1;previous_window<=window_log2;
     sum0<=0;sum1<=0;
    end else begin
     prevtime<=timestamp;prev0<=phase0;prev1<=phase1;
     if(timestamp-prevtime!=period_clocks || window_log2>BASELINE_LOG2 || dt>(32'd1<<BASELINE_LOG2)) begin
      previous<=0;sum0<=0;sum1<=0;ready<=0;estimate_valid<=0;good<=0;reason<=8;
     end else if(dt==(32'd1<<BASELINE_LOG2)) begin
      // Stage 1: capture the complete physical-baseline phase sums.
      phase_total0<=total0;phase_total1<=total1;calc1<=1;
      anchor_time<=timestamp;sum0<=0;sum1<=0;
     end else begin sum0<=total0;sum1<=total1;end
    end
    end
    // Stage 2: frequency conversion, isolated from phase accumulation.
    if(calc1) begin e0<=to_fcw(phase_total0);e1<=to_fcw(phase_total1);calc2<=1;end
    // Stage 3: confidence and candidate-frequency arithmetic.
    if(calc2) begin
     error0_fcw<=e0;error1_fcw<=e1;estimate_valid<=1;estimates<=estimates+1'b1;
     if(ab(e0-e1)>AGREE_FCW) begin ready<=0;good<=0;reason<=16;end
     else if((ready && ab(e0)<=WAKE_FCW && ab(e1)<=WAKE_FCW) ||
             (ab(e0)<=DEAD_FCW && ab(e1)<=DEAD_FCW)) begin
      reason<=0;if(good!=3) good<=good+1'b1;if(good>=1) ready<=1;
     end else begin
      ready<=0;good<=0;reason<=4;
      proposed<=$signed({2'b00,current_fcw})+{{2{e0[31]}},e0};
      proposal_nominal<=nominal_fcw;
      proposal_allowed<=ab(e0)<=MAX_STEP_FCW && ab(e1)<=MAX_STEP_FCW;
      apply<=1;
     end
    end
    // Stage 4: range validation and commit. Config/coarse-loss/reconfigure
    // cancel pending work through the outer priority branches.
    if(apply) begin
     offset=proposed-$signed({2'b00,proposal_nominal});
     if(!proposal_allowed || offset>CAPTURE_FCW || offset< -CAPTURE_FCW ||
        proposed<66 || proposed>=34'sh080000000) reason<=32;
     else begin target_fcw<=proposed[31:0];commit<=1;updates<=updates+1'b1;end
    end
   end
  end
 end
endmodule
`default_nettype wire
