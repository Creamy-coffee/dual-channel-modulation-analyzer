`timescale 1ns/1ps
// Use the real waveform/spectrum senders. A short frame makes repeated handover
// cheap to simulate; the UART and CRC paths are the production modules.
module tb_uart_frame_ownership;
 parameter integer SCAN_DELAY=5000;
 parameter integer POINTS=16,SPEC_BINS=16,CLK_FREQ=1000000,BAUD=100000;
 parameter integer RUN_CYCLES=220000;
 localparam real BIT_NS=10.0*CLK_FREQ/BAUD;
 reg clk=0,rst_n=0;
 always #5 clk=~clk;
 wire wave_tx,spec_tx,tx,pause,start,ready,wave_busy,spec_busy;
 wire [31:0] completed;
 reg [1023:0] filename;
 integer fd,bit_no,received=0,hidden_wave_cycles=0,clock_count=0;
 reg [7:0] value;
 wire scan_done = !ready && !spec_busy && clock_count>SCAN_DELAY;
 ab5_frame_sequencer #(.CLK_FREQ(CLK_FREQ),.BAUD(BAUD)) seq(
  .clk(clk),.rst_n(rst_n),.wave_completed(completed),
  .snapshot_ready(ready),.spec_busy(spec_busy),.wave_tx(wave_tx),
  .spec_tx(spec_tx),.tx(tx),.wave_pause(pause),.spec_start(start));
 v32cap3_tx #(.POINTS(POINTS),.LINK_ENABLE(1),.MEAS_ENABLE(1),
  .CLK_FREQ(CLK_FREQ),.BAUD(BAUD),.FRAME_PERIOD(0)) wave(
  .clk(clk),.rst_n(rst_n),.link_pause(pause),.sample_valid(1'b1),
  .decim_log2(4'd0),.fault(1'b0),.ch0(12'sd100),.ch1(-12'sd50),
  .meas_body({864'd0,32'd128,32'd1,64'h215341454d323356}),.tx(wave_tx),.busy(wave_busy),
  .completed(completed),.aborted(),.skipped());
 v32spec_tx #(.MANUAL_START(1),.BINS(SPEC_BINS),.CLK_FREQ(CLK_FREQ),.BAUD(BAUD)) spec(
  .clk(clk),.rst_n(rst_n),.wr_en(1'b0),.wr_bin(12'd0),.wr_mag(33'd0),
  .scan_done(scan_done),.send_start(start),.snapshot_ready(ready),
  .wave_frame_id(completed),.window_sel(2'd0),.tx(spec_tx),.busy(spec_busy),
  .completed(),.result_id());
 always @(posedge clk) if(rst_n)begin
  clock_count<=clock_count+1;
  if(seq.state!=0 && wave_busy)hidden_wave_cycles<=hidden_wave_cycles+1;
 end
 initial begin
  if(!$value$plusargs("OUT=%s",filename))$fatal(1,"missing OUT");
  fd=$fopen(filename,"wb");if(!fd)$fatal(1,"open output");
  repeat(4)@(negedge clk);rst_n=1;
  repeat(RUN_CYCLES)@(negedge clk);
  $fclose(fd);
  $display("FRAME_OWNERSHIP completed=%0d received=%0d hidden_wave_cycles=%0d",completed,received,hidden_wave_cycles);
  $finish;
 end
 initial forever begin
  @(negedge tx);#(BIT_NS*1.5);
  for(bit_no=0;bit_no<8;bit_no=bit_no+1)begin value[bit_no]=tx;#(BIT_NS);end
  if(tx!==1)$fatal(1,"invalid stop bit at received=%0d",received);
  $fwrite(fd,"%c",value);received=received+1;
 end
endmodule
