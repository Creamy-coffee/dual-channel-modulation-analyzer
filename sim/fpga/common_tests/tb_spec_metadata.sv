`timescale 1ns/1ps
module tb_spec_metadata;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,done=0,start=0;reg [31:0] fs=253906;
 wire tx,busy,ready;wire [31:0] completed;
 integer fd,k;reg [7:0] b;reg [1023:0] filename;
 v32spec_tx #(.MANUAL_START(1),.RUNTIME_FS(1),.BINS(16),.CLK_FREQ(1000000),.BAUD(100000)) dut(
  .clk(clk),.rst_n(rst),.wr_en(1'b0),.wr_bin(12'd0),.wr_mag(33'd0),
  .scan_done(done),.snapshot_fs_hz(fs),.send_start(start),.snapshot_ready(ready),
  .wave_frame_id(32'd9),.window_sel(2'd0),.tx(tx),.busy(busy),.completed(completed),.result_id());
 task frame(input[31:0] actual);begin
  @(negedge clk);fs=actual;done=1;@(negedge clk);done=0;
  if(!ready)$fatal(1,"snapshot not ready");
  // A new FFT band completes while the pending snapshot waits for ownership.
  fs=3967;done=1;@(negedge clk);done=0;
  repeat(30)@(negedge clk);start=1;@(negedge clk);start=0;fs=15869;
  wait(!busy);repeat(120)@(negedge clk);
 end endtask
 initial begin
  if(!$value$plusargs("OUT=%s",filename))$fatal(1,"OUT missing");fd=$fopen(filename,"wb");
  repeat(4)@(negedge clk);rst=1;
  frame(253906);frame(65000000);
  if(completed!=2)$fatal(1,"wrong completed");
  $fclose(fd);$display("SPEC_METADATA_SNAPSHOT_PASS");$finish;
 end
 initial forever begin
  @(negedge tx);#150;
  for(k=0;k<8;k=k+1)begin b[k]=tx;#100;end
  if(tx!==1)$fatal(1,"stop bit");$fwrite(fd,"%c",b);
 end
 initial begin repeat(30000)@(negedge clk);$fatal(1,"timeout");end
endmodule
