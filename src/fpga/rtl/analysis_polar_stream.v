`timescale 1ns/1ps
`default_nettype none
// Selected-channel I/Q -> one result word per original I/Q sample.
// AM/OOK: unsigned Q8 magnitude, including legitimate zero/OOK-off samples.
// FM/FSK: signed 24-bit wrapped phase step (turn/65536); Linux scales to Hz.
// PM: signed 24-bit wrapped phase (turn/65536).
// Non-IQ result bit31=invalid sample. PSK retains the complete original I/Q.
module analysis_polar_stream #(
 parameter [23:0] PHASE_FLOOR_Q8=24'd512
)(
 input wire clk,rst_n,in_valid,
 input wire signed [15:0] i_in,q_in,
 input wire [3:0] mode,
 output wire [7:0] kind,
 output wire out_valid,
 output wire [31:0] out_word,
 output reg overrun
);
 wire raw_iq=mode==4 || mode==6;
 assign kind=raw_iq?0:(mode==1 || mode==5)?1:(mode==2 || mode==3)?2:3;
 wire ready,pvalid,phase_valid;
 wire [23:0] amplitude_q8;
 wire signed [15:0] phase;
 wire signed [39:0] ii={{16{i_in[15]}},i_in,8'b0};
 wire signed [39:0] qq={{16{q_in[15]}},q_in,8'b0};
 ai_iq_polar_cordic #(.INPUT_SHIFT(0),.PHASE_MIN_AMP_Q12(PHASE_FLOOR_Q8)) polar(
  .clk(clk),.rst_n(rst_n),.in_valid(in_valid && !raw_iq && ready),.in_ready(ready),
  .i_sum(ii),.q_sum(qq),.frame_id_in(16'd0),.branch_in(1'b0),
  .out_valid(pvalid),.amp_q12(amplitude_q8),.phase_turn(phase),.phase_valid(phase_valid),.frame_id_out(),.branch_out());
 reg previous_valid;
 reg signed [15:0] previous_phase;
 wire signed [15:0] step=phase-previous_phase;
 wire [15:0] step_abs=step[15]?-step:step;
 wire step_invalid=!previous_valid || !phase_valid || step_abs>16'd29491;
 wire [23:0] value=kind==1?amplitude_q8:kind==2?{{8{step[15]}},step}:{{8{phase[15]}},phase};
 wire bad=kind==1?1'b0:kind==2?step_invalid:!phase_valid;
 assign out_valid=raw_iq?in_valid:pvalid;
 assign out_word=raw_iq?{q_in,i_in}:{bad,7'b0,value};
 always @(posedge clk)begin
  if(!rst_n)begin previous_valid<=0;previous_phase<=0;overrun<=0;end
  else begin
   if(in_valid && !raw_iq && !ready)begin overrun<=1;previous_valid<=0;end
   if(pvalid)begin previous_valid<=phase_valid;previous_phase<=phase;end
  end
 end
endmodule
`default_nettype wire
