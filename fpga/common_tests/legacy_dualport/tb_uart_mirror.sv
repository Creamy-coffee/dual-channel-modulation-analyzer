`timescale 1ns/1ps
module tb_uart_mirror;
 reg clk=0;always #10 clk=~clk;
 reg mode=0,wave_line=1,iq_line=1;wire tx,copy;
 daq_top #(.FFT_ENABLE(0),.LOW_FFT_ENABLE(0),.LOCKIN_ENABLE(0),.ET_ENABLE(0)) dut(
  .clk_50m(clk),.rst_n(1'b0),.adc_d0(12'd0),.adc_d1(12'd0),.adc_otr(1'b0),.adc_otr_b(1'b0),
  .digital_in(4'd0),.key_n(4'hf),.sw(3'd0),.v32_rx(1'b1),.v32_rx_copy(1'b1),.v32_tx(tx),.v32_tx_copy(copy));
 integer n;
 initial begin
  force dut.ab5_seq_tx=wave_line;force dut.iq_tx=iq_line;force dut.demod_active=mode;
  for(n=0;n<2048;n=n+1)begin
   mode=n[0];wave_line=n[1]^n[3];iq_line=n[2]^n[5];#1;
   if(tx!==(mode?iq_line:wave_line)||copy!==tx)$fatal(1,"UART mirror mismatch");#499;
  end
  $display("UART_MIRROR_PASS 2048 patterns both mux modes one shared final signal");$finish;
 end
endmodule
