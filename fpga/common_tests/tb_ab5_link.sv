`timescale 1ns/1ps
module tb_ab5_link;
 reg clk50=0;always #10 clk50=~clk50;
 reg rst=0,rx=1;
 reg [11:0] adc0=2048,adc1=2048;
 wire tx;
 daq_top #(.STARTUP_CYCLES(32),.LOCKIN_ENABLE(1),.ET_ENABLE(0),.AUTO_TIMEBASE(0)) dut(
  .clk_50m(clk50),.rst_n(rst),.adc_d0(adc0),.adc_d1(adc1),.adc_otr(1'b0),.adc_otr_b(1'b0),
  .digital_in(4'd0),.key_n(4'hf),.sw(3'd0),.v32_rx(rx),.v32_tx(tx));
 integer tick=0;
 reg [11:0] stimulus0[0:64],stimulus1[0:64];integer si;
 initial for(si=0;si<65;si=si+1)begin
  stimulus0[si]=2048+$rtoi(800*$sin(6.283185307179586*si/65.0));
  stimulus1[si]=2048+$rtoi(400*$sin(6.283185307179586*si/65.0+0.7));
 end
 always @(posedge dut.clk_sys) if(tick%500000==0)
  $display("AB5_PROGRESS tick=%0d bytes=%0d spec_ready=%b wave_busy=%b",tick,count,dut.spec_ready,dut.wave_busy);
 always @(negedge dut.clk_sys)begin
  tick=tick+1;
  adc0=stimulus0[tick%65];
  adc1=stimulus1[tick%65];
 end
 task send(input [7:0] b);
  integer j;
  begin
   rx=0;#500;
   for(j=0;j<8;j=j+1)begin rx=b[j];#500;end
   rx=1;#500;
  end
 endtask
 initial begin
  #100;rst=1;
  wait(dut.sys_rst_n);#100;
  send(8'ha5);send(2);
  #1000;if(!dut.command_override || dut.command_m!=2)$fatal(1,"A5 command not received");
  wait(dut.wave_busy);
  send(8'ha5);send(8);
  #1000;if(dut.wave_applied_m!=2)$fatal(1,"changed decimation inside wave");
 end
 integer fd,count=0,b;
 reg [7:0] value;
 initial begin
  fd=$fopen("../tests/ab5_link_wire.bin","wb");
  forever begin
   @(negedge tx);#750;
   for(b=0;b<8;b=b+1)begin value[b]=tx;#500;end
   if(tx!==1)$fatal(1,"AB5 stop bit at byte %0d",count);
   $fwrite(fd,"%c",value);count=count+1;
   if(count==6364)begin
    if(dut.wave_applied_m!=8)$fatal(1,"pending command did not apply after wave");
    $fclose(fd);$display("AB5_LINK_PASS bytes=%0d",count);$finish;
   end
  end
 end
 initial begin #150000000;$fatal(1,"timeout bytes=%0d",count);end
endmodule
