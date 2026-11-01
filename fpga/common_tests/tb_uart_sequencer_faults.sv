`timescale 1ns/1ps
module tb_uart_sequencer_faults;
 reg clk=0,rst_n=0,ready=0,busy=0;
 reg [31:0] completed=0;
 wire pause,start,tx;
 integer i,starts=0;
 always #5 clk=~clk;
 ab5_frame_sequencer #(.CLK_FREQ(1000000),.BAUD(100000)) dut(
  .clk(clk),.rst_n(rst_n),.wave_completed(completed),.snapshot_ready(ready),
  .spec_busy(busy),.wave_tx(1'b1),.spec_tx(1'b1),.tx(tx),
  .wave_pause(pause),.spec_start(start));
 always @(negedge clk)if(start)starts=starts+1;
 initial begin
  repeat(4)@(negedge clk);rst_n=1;
  for(i=0;i<100;i=i+1)begin
   @(negedge clk);completed=completed+1;#1;
   if(pause)$fatal(1,"snapshot missing blocks waveform");
  end
  // Let the completion be observed, then make the snapshot available in the
  // middle of the NEXT waveform. An old completion is not a frame boundary.
  repeat(3)@(negedge clk);ready=1;
  repeat(20)begin @(negedge clk);if(pause||start)$fatal(1,"late snapshot stole TX");end
  completed=completed+1;#1;
  if(!pause)$fatal(1,"next capture not blocked on completion edge");
  // No spectrum acknowledgement: bounded return, without further starts.
  repeat(1500)@(negedge clk);
  if(pause || dut.state!=0 || starts!=1)$fatal(1,"no-ack recovery failed");
  // A later frame must still be able to hand over normally.
  completed=completed+1;
  wait(start);@(negedge clk);busy=1;ready=0;
  repeat(30)@(negedge clk);busy=0;
  repeat(200)@(negedge clk);
  if(pause || dut.state!=0 || starts!=2)$fatal(1,"normal handover failed");
  $display("UART_SEQUENCER_FAULTS_PASS missing_snapshot late_snapshot no_ack recovery");
  $finish;
 end
 initial begin #100000;$fatal(1,"testbench timeout");end
endmodule
