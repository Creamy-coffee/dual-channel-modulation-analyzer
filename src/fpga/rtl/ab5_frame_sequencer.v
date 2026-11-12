`timescale 1ns/1ps
`default_nettype none
// Drain the last physical stop bit before transferring ownership of TX.
// An unavailable spectrum snapshot must never stop waveform capture.
// The completed counter is edge-detected EVERY cycle: comparing against the
// last spectrum handover instead would allow a late snapshot to steal TX in
// the middle of a subsequent waveform frame.
// Pause combinationally on that completion edge, before the waveform sender
// leaves IDLE to capture another frame. Waiting for state=WAVE_DRAIN is one
// cycle too late and lets an entire waveform run behind the spectrum mux.
// SPEC_START has a bounded wait for the spectrum sender's acknowledgement.
module ab5_frame_sequencer #(
 parameter integer CLK_FREQ=65000000, BAUD=2000000,SPEC_ENABLE=1
)(
 input wire clk,rst_n,
 input wire [31:0] wave_completed,
 input wire snapshot_ready,spec_busy,wave_tx,spec_tx,
 output wire tx,wave_pause,
 output reg spec_start
);
 localparam integer DRAIN=((CLK_FREQ+BAUD-1)/BAUD)*11+4;
 localparam integer CW=$clog2(DRAIN+1);
 localparam integer STALL=CLK_FREQ/1000;
 localparam integer SW=$clog2(STALL+1);
 localparam WAVE=0,WAVE_DRAIN=1,SPEC_START=2,SPEC_WAIT=3,SPEC_DRAIN=4;
 reg [2:0] state;
 reg [CW-1:0] timer;
 reg [SW-1:0] stall;
 reg [31:0] previous;
 wire handover=SPEC_ENABLE && snapshot_ready && wave_completed!=previous;
 assign wave_pause=SPEC_ENABLE && ((state!=WAVE) || handover);
 assign tx=(state==SPEC_START || state==SPEC_WAIT || state==SPEC_DRAIN)?spec_tx:wave_tx;
 always @(posedge clk) begin
  if(!rst_n) begin state<=WAVE;timer<=0;stall<=0;previous<=0;spec_start<=0;end
  else begin
   spec_start<=0;
   previous<=wave_completed;
   case(state)
    // Hand TX to the spectrum frame only when a snapshot is actually ready;
    // otherwise stay in WAVE and keep streaming waveform frames.
    WAVE:if(handover)begin
     timer<=DRAIN;state<=WAVE_DRAIN;
    end
    WAVE_DRAIN:if(timer!=0)timer<=timer-1'b1;else begin spec_start<=1;stall<=STALL;state<=SPEC_START;end
    SPEC_START:if(spec_busy)state<=SPEC_WAIT;
     else if(stall!=0)stall<=stall-1'b1;
     else state<=WAVE;
    SPEC_WAIT:if(!spec_busy)begin timer<=DRAIN;state<=SPEC_DRAIN;end
    SPEC_DRAIN:if(timer!=0)timer<=timer-1'b1;else state<=WAVE;
    default:state<=WAVE;
   endcase
  end
 end
endmodule
`default_nettype wire
