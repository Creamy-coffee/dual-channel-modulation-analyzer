`timescale 1ns/1ps
// Actual top-level ADC -> FFT -> harmonic statistics -> shared UART.
// Production clocks, FFT size and UART rate; only startup delay is shortened.
module tb_harmonic_top;
 reg clk50=0,rst=0,rx=1;always #10 clk50=~clk50;
 reg [11:0] a=2048,b=2048;wire tx;
 daq_top #(.STARTUP_CYCLES(8),.COM9_TEXT_ENABLE(0)) dut(
  .clk_50m(clk50),.rst_n(rst),.adc_d0(a),.adc_d1(b),.adc_otr(1'b0),.adc_otr_b(1'b0),
  .digital_in(4'b0),.key_n(4'hf),.sw(3'b0),.v32_rx(rx),.v32_tx(tx),.uart_tx(),
  .adc_clk(),.adc_clk_b(),.adc_oeb(),.adc_oeb_b(),.adc_pwdn(),.adc_pwdn_b(),.led());
 real ph=0;
 always @(negedge dut.clk_sys) if(dut.sys_rst_n)begin
  ph=ph+6.283185307179586/32;
  if(ph>=6.283185307179586)ph=ph-6.283185307179586;
  a=2048+$rtoi(700*$cos(ph)+70*$cos(2*ph)+35*$cos(3*ph));
  b=2048+$rtoi(350*$cos(ph));
 end
 task send(input [7:0] value);integer k;begin
  rx=0;#500;for(k=0;k<8;k=k+1)begin rx=value[k];#500;end rx=1;#500;
 end endtask
 task cmd(input [7:0] op,input [7:0] value);begin send('ha5);send(op);send(value);end endtask
 task wordcmd(input [7:0] op,input [15:0] value);begin send('ha5);send(op);send(value[7:0]);send(value[15:8]);end endtask
 integer fd,k,bytes=0;reg [7:0] value;reg [31:0] saved_cfg;
 initial forever begin
  @(negedge tx);#750;
  for(k=0;k<8;k=k+1)begin value[k]=tx;#500;end
  if(tx!==1)$fatal(1,"UART stop error");
  $fwrite(fd,"%c",value);bytes=bytes+1;
 end
 initial begin
  fd=$fopen("harmonic_top.bin","wb");#100;rst=1;wait(dut.adc_ready);
  // Configure the existing AM stream independently of the harmonic reference.
  wordcmd('he1,16'h0800);wordcmd('he2,16'h0000);cmd('he3,0);cmd('he4,1);cmd('he0,1);
  repeat(100)@(negedge dut.clk_sys);saved_cfg=dut.rx_cfg_id;
  cmd('hf0,0);wordcmd('hf1,16'h0800);wordcmd('hf2,16'h0000);cmd('hf3,1);
  repeat(100)@(negedge dut.clk_sys);
  if(dut.harmonic_reference!=32'h08000000 || !dut.harmonic_enable || dut.harmonic_channel!=0)
   $fatal(1,"harmonic serial command was not applied");
  if(dut.rx_cfg_id!=saved_cfg)$fatal(1,"harmonic command modified DDC config");
  // FIR warmup + a complete ANA block may take longer than the first FFT.
  // Require both complete streams, rather than testing ANA at FFT completion.
  wait(dut.harmonic_completed>=1 && dut.iq_completed>=1);
  repeat(10000)@(negedge dut.clk_sys);
  if(dut.iq_completed<1)$fatal(1,"ANA stopped while harmonic enabled");
  if(dut.u_parallel_link.abort_count!=0)$fatal(1,"shared UART arbitration aborted");
  cmd('hf3,0);repeat(100)@(negedge dut.clk_sys);
  if(dut.harmonic_enable || dut.rx_cfg_id!=saved_cfg)$fatal(1,"disable/config isolation failed");
  $fclose(fd);$display("HARMONIC_TOP_PASS bytes=%0d",bytes);$finish;
 end
 initial begin #500000000;$fatal(1,"harmonic top watchdog");end
endmodule
