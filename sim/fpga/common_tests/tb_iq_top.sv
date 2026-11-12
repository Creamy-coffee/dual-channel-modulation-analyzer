`timescale 1ns/1ps
module tb_iq_top;
reg clk50=0;always #10 clk50=~clk50;reg rst=0,rx=1;
reg[11:0] adc0=2048,adc1=2048;wire tx;
daq_top #(.STARTUP_CYCLES(32),.FFT_ENABLE(0),.LOW_FFT_ENABLE(0),.LOCKIN_ENABLE(0),.ET_ENABLE(0),.AUTO_TIMEBASE(0)) dut(
 .clk_50m(clk50),.rst_n(rst),.adc_d0(adc0),.adc_d1(adc1),.adc_otr(1'b0),.adc_otr_b(1'b0),
 .digital_in(4'd0),.key_n(4'hf),.sw(3'd0),.v32_rx(rx),.v32_tx(tx));
integer tick=0;real t;
always @(negedge dut.clk_sys)begin tick=tick+1;t=tick/65000000.0;
 adc0=2048+$rtoi(1000*$cos(6.283185307179586*1000000*t));
 adc1=2048+$rtoi(400*(1+0.6*$sin(6.283185307179586*500*t))*$cos(6.283185307179586*1000000*t));end
task send(input[7:0] b);integer j;begin rx=0;#500;for(j=0;j<8;j=j+1)begin rx=b[j];#500;end rx=1;#1000;end endtask
integer fd,k,count=0;reg[7:0] b;
initial begin fd=$fopen("../tests/iq_top_wire.bin","wb");#100;rst=1;wait(dut.sys_rst_n);#100;
 if(dut.led[0]!==dut.locked || dut.led[1]!==dut.adc_ready ||
    dut.led[2]!== (dut.disp_timebase==0) || dut.led[3]!== (|dut.errors))
  $fatal(1,"Core user LED active-high mapping");
 send('ha5);send('he3);send(1);send('ha5);send('he0);send(1);
 if(!dut.demod_active||!dut.iq_ch)$fatal(1,"top command mux");
 wait(dut.g_ddc.u_iq_tx.completed==2);#6000;
 send('ha5);send('he0);send(0);
 if(dut.demod_active)$fatal(1,"mode off");
 $fclose(fd);$display("IQ_TOP_PASS bytes=%0d channel=1 mode_off=1",count);$finish;end
initial begin wait(dut.demod_active);repeat(2136)begin @(negedge tx);#750;for(k=0;k<8;k=k+1)begin b[k]=tx;#500;end
 if(tx!==1)$fatal(1,"top stop");$fwrite(fd,"%c",b);count=count+1;end end
initial begin #50000000;$fatal(1,"top timeout");end
endmodule

