`timescale 1ns/1ps
`default_nettype none
// Detection confidence, not an amplitude/phase accuracy certificate.
// IQ units are half-peak ADC counts *4096. Noise proxy is the smoothed
// L1 change of consecutive IQ vectors; no true noise sigma is supplied.
module lockin_signal_guard #(
 parameter [23:0] MIN_AMPLITUDE_Q12=24'd2048,
 parameter [15:0] MAX_PHASE_STEP=16'd910
)(
 input wire clk,rst_n,invalidate,in_valid,
 input wire [31:0] frame_id,
 input wire signed [23:0] i0,q0,i1,q1,
 input wire [23:0] amp0,amp1,
 input wire signed [15:0] phase0,phase1,
 input wire [1:0] numerical_phase_valid,
 output reg out_valid,
 output reg [1:0] detected,qualified,
 output reg [7:0] reason0,reason1,
 output reg [15:0] phase_step0,phase_step1,
 output reg signed [15:0] phase_delta0,phase_delta1,
 output reg phase_delta_valid,
 output reg [28:0] threshold0,threshold1,
 output wire [25:0] noise0,noise1
);
 reg [31:0] previous_frame;
 reg previous;
 reg [1:0] previous_numeric;
 reg [3:0] warmup;
 reg signed [23:0] pi[0:1],pq[0:1];
 reg signed [15:0] pp[0:1];
 reg [25:0] noise_est[0:1];
 reg [1:0] good_count[0:1],bad_count[0:1];
 assign noise0=noise_est[0];assign noise1=noise_est[1];
 function [24:0] abs25;
  input signed [24:0] value;
  begin abs25=value[24]?-value:value;end
 endfunction
 integer ch;
 reg signed [23:0] ci,cq;
 reg signed [24:0] di,dq;
 reg signed [15:0] dp;
 reg [15:0] phase_change;
 reg [25:0] change;
 reg [28:0] threshold;
 reg [23:0] amplitude;
 reg signed [15:0] phase_now;
 reg good;
 always @(posedge clk) begin
  out_valid<=0;
  if(!rst_n || invalidate) begin
   previous<=0;previous_numeric<=0;previous_frame<=0;warmup<=0;detected<=0;qualified<=0;
   phase_delta0<=0;phase_delta1<=0;phase_delta_valid<=0;
   reason0<=1;reason1<=1;phase_step0<=0;phase_step1<=0;threshold0<=0;threshold1<=0;
   for(ch=0;ch<2;ch=ch+1) begin
    pi[ch]<=0;pq[ch]<=0;pp[ch]<=0;noise_est[ch]<=0;good_count[ch]<=0;bad_count[ch]<=0;
   end
  end else if(in_valid) begin
   out_valid<=1;
   previous<=1;previous_frame<=frame_id;
   previous_numeric<=numerical_phase_valid;
   phase_delta_valid<=previous && frame_id==previous_frame+1'b1 &&
      warmup==8 && numerical_phase_valid==3 && previous_numeric==3;
   if(!previous || frame_id!=previous_frame+1'b1) begin
    warmup<=0;detected<=0;qualified<=0;reason0<=2;reason1<=2;
    for(ch=0;ch<2;ch=ch+1) begin
     noise_est[ch]<=0;good_count[ch]<=0;bad_count[ch]<=0;
    end
   end else begin
    if(warmup!=8) warmup<=warmup+1'b1;
    for(ch=0;ch<2;ch=ch+1) begin
     ci=ch?i1:i0;cq=ch?q1:q0;amplitude=ch?amp1:amp0;phase_now=ch?phase1:phase0;
     di={ci[23],ci}-{pi[ch][23],pi[ch]};dq={cq[23],cq}-{pq[ch][23],pq[ch]};
     change={1'b0,abs25(di)}+{1'b0,abs25(dq)};
     if(warmup==0) noise_est[ch]<=change;
     else if(change>=noise_est[ch]) noise_est[ch]<=noise_est[ch]+((change-noise_est[ch])>>3);
     else noise_est[ch]<=noise_est[ch]-((noise_est[ch]-change)>>3);
     dp=phase_now-pp[ch];phase_change=dp[15]?-dp:dp;
     threshold={noise_est[ch],3'b000};
     if(threshold<MIN_AMPLITUDE_Q12) threshold={5'd0,MIN_AMPLITUDE_Q12};
     good=(warmup==8 && numerical_phase_valid[ch] &&
           {5'd0,amplitude}>threshold && phase_change<=MAX_PHASE_STEP);
     if(ch==0) begin
      reason0<={4'd0,(phase_change>MAX_PHASE_STEP),({5'd0,amplitude}<=threshold),!numerical_phase_valid[ch],(warmup!=8)};
      phase_step0<=phase_change;phase_delta0<=dp;threshold0<=threshold;
     end else begin
      reason1<={4'd0,(phase_change>MAX_PHASE_STEP),({5'd0,amplitude}<=threshold),!numerical_phase_valid[ch],(warmup!=8)};
      phase_step1<=phase_change;phase_delta1<=dp;threshold1<=threshold;
     end
     qualified[ch]<=0;
     if(good) begin
      bad_count[ch]<=0;
      if(good_count[ch]!=3) good_count[ch]<=good_count[ch]+1'b1;
      if(good_count[ch]>=2) begin detected[ch]<=1;qualified[ch]<=1;end
     end else begin
      good_count[ch]<=0;
      if(bad_count[ch]!=2) bad_count[ch]<=bad_count[ch]+1'b1;
      if(bad_count[ch]>=1) detected[ch]<=0;
     end
    end
   end
   pi[0]<=i0;pq[0]<=q0;pp[0]<=phase0;
   pi[1]<=i1;pq[1]<=q1;pp[1]<=phase1;
  end
 end
endmodule
`default_nettype wire
