`timescale 1ns/1ps
`default_nettype none
// Local frequency follower for a KNOWN carrier. Original implementation.
// Frequency-dependent observers use the same full-rate mixer and physical
// clock stamps. At low carriers a 4096-point average does not reject the 2*f
// mixer image. Lengthen the observation, never the measurement/guard window.
// Acquisition must start near the FFT estimate: unambiguous residual range
// is +/- Fs/(2*observer_length), not a full-band frequency search.
module local_afc_2ch #(
 parameter integer CLK_FREQ=65000000, ADAPTIVE_WINDOW=1,
 parameter integer CAPTURE_FCW=132153, MAX_STEP_FCW=33038,
 parameter integer MIN_AMP=2048, DEAD_FCW=198, WAKE_FCW=529,
 parameter integer AGREE_FCW=1322
)(
 input wire clk,rst_n,clear,stream_ok,mix_valid,
 input wire signed [27:0] mi0,mq0,mi1,mq1,
 input wire [31:0] sample_time,nominal_fcw,current_fcw,
 output reg [31:0] target_fcw,
 output reg commit,
 output wire ready_measurement,
 output reg [2:0] state, // 0 absent/reset, 1 acquire, 2 settle, 3 stable, 4 rejected
 output reg signed [31:0] residual0_fcw,residual1_fcw,
 output reg [7:0] reason, // bit0 absent,1 timing/overrun,2 disagreement,3 range
 output reg [31:0] updates,observation_id,
 output reg [31:0] last_timestamp
);
 localparam [31:0] F120K=(64'd4294967296*120000)/CLK_FREQ;
 localparam [31:0] F8K=(64'd4294967296*8000)/CLK_FREQ;
 localparam [31:0] F500=(64'd4294967296*500)/CLK_FREQ;
 localparam [31:0] F30=(64'd4294967296*30)/CLK_FREQ;
 wire [4:0] obs_shift=(!ADAPTIVE_WINDOW || nominal_fcw>=F120K)?12:
    nominal_fcw>=F8K?16:nominal_fcw>=F500?20:nominal_fcw>=F30?24:26;
 wire [31:0] obs_period=obs_shift==12?32'd4096:obs_shift==16?32'd65536:
    obs_shift==20?32'd1048576:obs_shift==24?32'd16777216:32'd67108864;
 reg [25:0] count;
 reg signed [53:0] si0,sq0,si1,sq1;
 wire signed [53:0] ni0=si0+{{26{mi0[27]}},mi0};
 wire signed [53:0] nq0=sq0+{{26{mq0[27]}},mq0};
 wire signed [53:0] ni1=si1+{{26{mi1[27]}},mi1};
 wire signed [53:0] nq1=sq1+{{26{mq1[27]}},mq1};
 function signed [23:0] scaled;
  input signed [53:0] v;
  reg signed [53:0] a;
  begin
   case(obs_shift)
    12:a=v>>>14;16:a=v>>>18;20:a=v>>>22;24:a=v>>>26;default:a=v>>>28;
   endcase
   if(a>8388607) scaled=24'sh7fffff;
   else if(a< -8388608) scaled=24'sh800000;
   else scaled=a[23:0];
  end
 endfunction
 wire reset_n=rst_n && !clear && stream_ok;
 reg obs;
 reg signed [23:0] oi0,oq0,oi1,oq1;
 reg [31:0] ot;
 always @(posedge clk) begin
  obs<=0;
  if(!reset_n) begin count<=0;si0<=0;sq0<=0;si1<=0;sq1<=0;
   oi0<=0;oq0<=0;oi1<=0;oq1<=0;ot<=0;end
  else if(mix_valid) begin
   if({6'd0,count}==obs_period-1'b1) begin
    oi0<=scaled(ni0);oq0<=scaled(nq0);oi1<=scaled(ni1);oq1<=scaled(nq1);
    ot<=sample_time;obs<=1;count<=0;si0<=0;sq0<=0;si1<=0;sq1<=0;
   end else begin count<=count+1'b1;si0<=ni0;sq0<=nq0;si1<=ni1;sq1<=nq1;end
  end
 end
 wire prdy,pvalid,pok;
 wire signed [15:0] phase;
 wire [23:0] amp;
 // 'busy' must be DECLARED before its first use (xvlog VRFC 10-3380; iverilog
 // tolerated the old order).  Declaration moved up; behaviour unchanged.
 reg [2:0] busy;
 wire send=(busy==1 || busy==3) && prdy;
 wire chan=(busy==3);
 reg signed [23:0] bi0,bq0,bi1,bq1;
 reg [31:0] bt;
 reg signed [15:0] ph0,ph1;
 reg [23:0] amp0,amp1;
 reg valid0,valid1,pair;
 wire signed [23:0] ii=chan?bi1:bi0,qq=chan?bq1:bq0;
 ai_iq_polar_cordic #(.INPUT_SHIFT(0)) polar(
  .clk(clk),.rst_n(reset_n),.in_valid(send),.in_ready(prdy),
  .i_sum({{16{ii[23]}},ii}),.q_sum({{16{qq[23]}},qq}),
  .frame_id_in(16'd0),.branch_in(chan),.out_valid(pvalid),
  .amp_q12(amp),.phase_turn(phase),.phase_valid(pok),.frame_id_out(),.branch_out());
 always @(posedge clk) begin
  pair<=0;
  if(!reset_n) begin busy<=0;bi0<=0;bq0<=0;bi1<=0;bq1<=0;bt<=0;
   ph0<=0;ph1<=0;amp0<=0;amp1<=0;valid0<=0;valid1<=0;end
  else begin
   if(obs && busy==0) begin bi0<=oi0;bq0<=oq0;bi1<=oi1;bq1<=oq1;bt<=ot;busy<=1;end
   if(send) busy<=busy+1'b1;
   if(pvalid && busy==2) begin ph0<=phase;amp0<=amp;valid0<=pok;busy<=3;end
   if(pvalid && busy==4) begin ph1<=phase;amp1<=amp;valid1<=pok;busy<=0;pair<=1;end
  end
 end
 reg previous;
 reg signed [15:0] prev0,prev1;
 wire signed [15:0] dp0=ph0-prev0,dp1=ph1-prev1;
 reg [31:0] prevtime;
 reg [3:0] batch;
 reg signed [31:0] sum0,sum1;
 wire signed [31:0] total0=sum0+{{16{dp0[15]}},dp0};
 wire signed [31:0] total1=sum1+{{16{dp1[15]}},dp1};
 // Keep the legacy 16-interval estimate at R1/4096. Longer observations
 // use one interval; FCW = delta_turn * 65536 / physical_interval.
 // Constant shifts avoid a divider and preserve the full accumulator width.
 function signed [31:0] physical_error;
  input signed [31:0] value;
  begin case(obs_shift)
   12:physical_error=value;
   16:physical_error=value;
   20:physical_error=(value+(value[31]?32'sd7:32'sd8))>>>4;
   24:physical_error=(value+(value[31]?32'sd127:32'sd128))>>>8;
   default:physical_error=(value+(value[31]?32'sd511:32'sd512))>>>10;
  endcase end
 endfunction
 reg signed [31:0] e0,e1,step;
 reg signed [33:0] proposed,off;
 reg [5:0] settle_batches;
 reg [2:0] quiet_batches;
 reg calc,decide;
 reg signed [31:0] decision0,decision1;
 // Stable tracking uses a dual-lane smoothed wake-up decision. A single
 // noisy CH0 short window must not destroy an accumulating fine estimate.
 reg signed [31:0] smooth0,smooth1;
 reg [2:0] wake_count;
 reg signed [31:0] control0,control1;
 reg stable_wake;
 function [31:0] ab;
  input signed [31:0] v;
  begin ab=v[31]?-v:v;end
 endfunction
 assign ready_measurement=reset_n && state==3 && reason==0;
 always @(posedge clk) begin
  commit<=0;calc<=0;decide<=0;
  if(!reset_n) begin
   target_fcw<=nominal_fcw;state<=0;reason<=1;updates<=0;observation_id<=0;
   residual0_fcw<=0;residual1_fcw<=0;last_timestamp<=0;previous<=0;
   prev0<=0;prev1<=0;prevtime<=0;batch<=0;sum0<=0;sum1<=0;
   e0<=0;e1<=0;settle_batches<=0;quiet_batches<=0;
   smooth0<=0;smooth1<=0;wake_count<=0;decision0<=0;decision1<=0;
  end else begin
   if(obs && busy!=0) begin state<=4;reason<=2;previous<=0;end
   if(pair) begin
    observation_id<=observation_id+1'b1;last_timestamp<=bt;
    prev0<=ph0;prev1<=ph1;prevtime<=bt;
    if(!valid0 || !valid1 || amp0<MIN_AMP || amp1<MIN_AMP) begin
     previous<=0;batch<=0;sum0<=0;sum1<=0;state<=0;reason<=1;quiet_batches<=0;
    end else if(!previous) begin previous<=1;batch<=0;sum0<=0;sum1<=0;state<=1;reason<=0;end
    else if(bt-prevtime!=obs_period) begin
     previous<=0;batch<=0;sum0<=0;sum1<=0;state<=4;reason<=2;quiet_batches<=0;
    end else begin
     if((obs_shift==12 && batch==15) || obs_shift!=12) begin
      e0<=physical_error(total0);e1<=physical_error(total1);calc<=1;sum0<=0;sum1<=0;batch<=0;
     end
     else begin sum0<=total0;sum1<=total1;batch<=batch+1'b1;end
    end
   end
   if(calc) begin
    residual0_fcw<=e0;residual1_fcw<=e1;
    if(state==3) begin
     control0=smooth0+((e0-smooth0)>>>3);
     control1=smooth1+((e1-smooth1)>>>3);
    end else begin control0=e0;control1=e1;end
    smooth0<=control0;smooth1<=control1;
    decision0<=control0;decision1<=control1;decide<=1;
   end
   if(decide) begin
    control0=decision0;control1=decision1;
    stable_wake=(ab(control0)>WAKE_FCW && ab(control1)>WAKE_FCW && control0[31]==control1[31]);
    if(state==3 && stable_wake) begin
     if(wake_count!=4) wake_count<=wake_count+1'b1;
    end else wake_count<=0;
    if(ab(control0-control1)>AGREE_FCW) begin state<=4;reason<=4;quiet_batches<=0;end
    else begin
     // Track CH0 only. CH1 agreement is mandatory for shared-reference validity.
     step=control0>>>1;
     if(step>MAX_STEP_FCW) step=MAX_STEP_FCW;
     if(step< -MAX_STEP_FCW) step=-MAX_STEP_FCW;
     proposed=$signed({2'b00,current_fcw})+{{2{step[31]}},step};
     off=proposed-$signed({2'b00,nominal_fcw});
     if(off>CAPTURE_FCW || off< -CAPTURE_FCW || proposed<66 || proposed>=34'sh080000000) begin
      state<=4;reason<=8;quiet_batches<=0;
     end else begin
      reason<=0;
      if(settle_batches!=0) begin settle_batches<=settle_batches-1'b1;state<=2;end
      else if((state==3 && (!stable_wake || wake_count<3)) ||
              (ab(e0)<=DEAD_FCW && ab(e1)<=DEAD_FCW)) begin
       if(quiet_batches!=7) quiet_batches<=quiet_batches+1'b1;
       if(quiet_batches>=3) state<=3;
      end else begin
       target_fcw<=proposed[31:0];commit<=1;updates<=updates+1'b1;
       state<=2;settle_batches<=1;quiet_batches<=0;wake_count<=0;
       // Observations straddling an update must not enter the next estimate.
       previous<=0;batch<=0;sum0<=0;sum1<=0;
      end
     end
    end
   end
  end
 end
endmodule
`default_nettype wire
