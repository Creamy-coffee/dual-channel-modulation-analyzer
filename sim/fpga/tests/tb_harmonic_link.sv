`timescale 1ns/1ps
module tb_harmonic_link;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,enable=1,channel=0,sv=0,start=0;
 reg [31:0] cfg=1,reference_fcw=268435456;
 reg [1:0] scan_channel=0,window_id=0;
 reg [7:0] bin=0;reg [32:0] power=0;
 wire ready,busy,tx;wire [31:0] completed;
 harmonic_analysis_link #(.CLK_FREQ(20000000),.BAUD(2000000),.LOG2_N(8)) dut(
  .clk(clk),.rst_n(rst),.enable(enable),.channel(channel),.cfg_id(cfg),.reference_fcw(reference_fcw),
  .supported_rates(5'b00001),.scan_valid(sv),.scan_saturated(1'b0),.scan_channel(scan_channel),.scan_window(window_id),
  .scan_bin(bin),.scan_power(power),.scan_rate_log2(4'd0),.scan_age_ticks(32'd17),
  .send_start(start),.ready(ready),.busy(busy),.tx(tx),.completed(completed));
 integer received=0,fout,i,k;reg [7:0] byte_value;
 initial begin
  fout=$fopen("tests_build/link_bytes.hex","w");
  for(i=0;i<432;i=i+1)begin
   @(negedge tx);repeat(15)@(posedge clk);#1;
   for(k=0;k<8;k=k+1)begin byte_value[k]=tx;repeat(10)@(posedge clk);#1;end
   if(tx!==1'b1)$fatal(1,"invalid UART stop bit at byte %d",i);
   $fwrite(fout,"%02x\n",byte_value);received=received+1;
  end
  $fclose(fout);
 end
 task scan;
  integer j;
  begin
   for(j=1;j<=128;j=j+1)begin
    @(negedge clk);sv=1;bin=j;power=100;
    if(j==16)power=1000100;
    if(j==32)power=10100;
    if(j==48)power=2600;
   end
   @(negedge clk);sv=0;
  end
 endtask
 task send;
  begin @(negedge clk);start=1;@(negedge clk);start=0;end
 endtask
 initial begin
  repeat(3)@(negedge clk);rst=1;repeat(16)@(negedge clk);
  scan();wait(ready);@(negedge clk);
  // A result waiting for its turn owns its registers even if the user changes
  // configuration or disables measurement. It must remain an intact OLD frame.
  enable=0;cfg=2;channel=1;scan_channel=1;window_id=1;
  repeat(100)@(negedge clk);send();
  wait(completed==1);wait(received==216);repeat(20)@(negedge clk);
  if(ready || busy || dut.hold)$fatal(1,"transport did not release result");
  enable=1;repeat(16)@(negedge clk);scan();wait(ready);send();
  wait(completed==2);wait(received==432);repeat(20)@(negedge clk);
  $display("HARMONIC_LINK_PASS UART_2M_CRC_hold_config_disable_release");$finish;
 end
 initial begin #1000000;$fatal(1,"link watchdog ready=%b busy=%b completed=%d rx=%d",ready,busy,completed,received);end
endmodule
