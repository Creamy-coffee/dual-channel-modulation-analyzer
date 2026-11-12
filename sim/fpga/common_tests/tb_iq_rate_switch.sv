`timescale 1ns/1ps
module tb_iq_rate_switch;
reg clk50=0;always #10 clk50=~clk50;
reg rst=0,rx=1;wire tx;
daq_top #(.STARTUP_CYCLES(32),.FFT_ENABLE(0),.LOW_FFT_ENABLE(0),.LOCKIN_ENABLE(0),
 .ET_ENABLE(0),.AUTO_TIMEBASE(0)) dut(.clk_50m(clk50),.rst_n(rst),
 .adc_d0(12'd3048),.adc_d1(12'd2448),.adc_otr(1'b0),.adc_otr_b(1'b0),
 .digital_in(4'd0),.key_n(4'hf),.sw(3'd0),.v32_rx(rx),.v32_tx(tx));
// Short integration frames keep simulation bounded; the production default
// 256-point frame is checked separately by tb_iq_top. ADC/DDC rates unchanged.
defparam dut.g_ddc.u_iq_tx.POINTS=32;
task send(input[7:0] b);integer j;begin
 rx=0;#500;for(j=0;j<8;j=j+1)begin rx=b[j];#500;end rx=1;#1000;
end endtask
task rate(input[7:0] r);begin
 send('ha5);send('he4);send(r);
 if(dut.iq_rate!=r || dut.g_ddc.u_iq_tx.wcnt!=0)$fatal(1,"rate commit did not flush FIFO");
 wait(dut.g_ddc.u_iq_tx.completed==(r==2?3:r==0?1:2));#6000;
end endtask
integer fd,k,count=0;reg[7:0] b;
initial begin
 fd=$fopen("../tests/iq_rate_switch_wire.bin","wb");#100;rst=1;wait(dut.sys_rst_n);#100;
 send('ha5);send('he3);send(1);
 send('ha5);send('he1);send(0);send(0);send('ha5);send('he2);send(0);send(0);
 send('ha5);send('he0);send(1);
 rate(2);rate(0);rate(1);
 send('ha5);send('he0);send(0);
 if(dut.demod_active || count!=1032)$fatal(1,"mode off/byte count");
 $fclose(fd);$display("IQ_RATE_SWITCH_PASS frames=6 points=32 bytes=%0d",count);$finish;
end
initial begin wait(dut.demod_active);
 forever begin @(negedge tx);#750;for(k=0;k<8;k=k+1)begin b[k]=tx;#500;end
  if(tx!==1)$fatal(1,"rate UART stop");$fwrite(fd,"%c",b);count=count+1;
 end
end
initial begin #120000000;$fatal(1,"rate switch timeout");end
endmodule
