`timescale 1ns/1ps
`default_nettype none
// Priority is analysis backlog, then a rate-limited RAW/MEAS group, then SPEC.
// Acquisition is independent: wave_ready means frozen RAM, not "start capture".
module parallel_frame_arbiter #(
 parameter integer CLK_FREQ=65000000,BAUD=2000000,
 parameter integer WAVE_NORMAL_PERIOD=CLK_FREQ/20,
 parameter integer WAVE_ANALYSIS_PERIOD=CLK_FREQ/5,
 parameter integer SPEC_NORMAL_PERIOD=CLK_FREQ/5,
 parameter integer SPEC_ANALYSIS_PERIOD=CLK_FREQ/2,
 parameter integer HARMONIC_PERIOD=CLK_FREQ/2
)(
 input wire clk,rst_n,analysis_enable,
 input wire wave_ready,wave_busy,wave_tx,
 input wire [31:0] wave_completed,
 input wire analysis_ready,analysis_busy,analysis_tx,
 input wire [31:0] analysis_completed,
 input wire spec_ready,spec_busy,spec_tx,
 input wire [31:0] spec_completed,
 input wire harmonic_ready,harmonic_busy,harmonic_tx,
 input wire [31:0] harmonic_completed,
 output wire wave_grant,analysis_grant,
 output reg spec_start,harmonic_start,
 output wire tx,
 output reg [31:0] abort_count,
 output wire [2:0] owner_debug
);
 localparam IDLE=0,WAVE=1,ANALYSIS=2,SPEC=3,DRAIN=4,HARMONIC=5;
 localparam integer DRAIN_TICKS=((CLK_FREQ+BAUD-1)/BAUD)*12+8;
 localparam integer ACK_TICKS=CLK_FREQ/1000+16;
 reg [2:0] owner,drain_owner;
 reg [31:0] wave_seen,analysis_seen,spec_seen,harmonic_seen;
 reg [31:0] wave_due,spec_due,harmonic_due,wait_ticks;
 reg started;
 wire wave_done=wave_completed!=wave_seen;
 wire analysis_done=analysis_completed!=analysis_seen;
 wire spec_done=spec_completed!=spec_seen;
 wire harmonic_done=harmonic_completed!=harmonic_seen;
 // Completion is detected combinationally, preventing an extra hidden frame.
 assign wave_grant=owner==WAVE && !wave_done;
 assign analysis_grant=owner==ANALYSIS && !analysis_done && analysis_enable;
 assign owner_debug=owner;
 wire [2:0] selected=owner==DRAIN?drain_owner:owner;
 assign tx=selected==WAVE?wave_tx:selected==ANALYSIS?analysis_tx:selected==SPEC?spec_tx:selected==HARMONIC?harmonic_tx:1'b1;
 wire current_busy=owner==WAVE?wave_busy:owner==ANALYSIS?analysis_busy:owner==HARMONIC?harmonic_busy:spec_busy;
 wire current_done=owner==WAVE?wave_done:owner==ANALYSIS?analysis_done:owner==HARMONIC?harmonic_done:spec_done;
 always @(posedge clk)begin
  if(!rst_n)begin
   owner<=IDLE;drain_owner<=IDLE;wave_seen<=0;analysis_seen<=0;spec_seen<=0;
   harmonic_seen<=0;harmonic_due<=0;harmonic_start<=0;
   wave_due<=0;spec_due<=0;wait_ticks<=0;started<=0;spec_start<=0;abort_count<=0;
  end else begin
   wave_seen<=wave_completed;analysis_seen<=analysis_completed;spec_seen<=spec_completed;
   harmonic_seen<=harmonic_completed;harmonic_start<=0;
   spec_start<=0;
   if(wave_due!=0)wave_due<=wave_due-1;
   if(spec_due!=0)spec_due<=spec_due-1;
   if(harmonic_due!=0)harmonic_due<=harmonic_due-1;
   case(owner)
    IDLE:begin
     started<=0;wait_ticks<=ACK_TICKS;
     if(analysis_enable && analysis_ready)owner<=ANALYSIS;
     else if(wave_ready && wave_due==0)begin
      owner<=WAVE;wave_due<=analysis_enable?WAVE_ANALYSIS_PERIOD:WAVE_NORMAL_PERIOD;
     end else if(harmonic_ready && harmonic_due==0)begin
      owner<=HARMONIC;harmonic_start<=1;harmonic_due<=HARMONIC_PERIOD;
     end else if(spec_ready && spec_due==0)begin
      owner<=SPEC;spec_start<=1;
      spec_due<=analysis_enable?SPEC_ANALYSIS_PERIOD:SPEC_NORMAL_PERIOD;
     end
    end
    WAVE,ANALYSIS,SPEC,HARMONIC:begin
     if(current_busy)started<=1;
     if(current_done || (started && !current_busy) ||
        (owner==ANALYSIS && !analysis_enable) || (!started && wait_ticks==0))begin
      if(!current_done)abort_count<=abort_count+1;
      drain_owner<=owner;owner<=DRAIN;wait_ticks<=DRAIN_TICKS;
     end else if(wait_ticks!=0)wait_ticks<=wait_ticks-1;
    end
    DRAIN:if(wait_ticks!=0)wait_ticks<=wait_ticks-1;else owner<=IDLE;
    default:owner<=IDLE;
   endcase
  end
 end
endmodule
`default_nettype wire
