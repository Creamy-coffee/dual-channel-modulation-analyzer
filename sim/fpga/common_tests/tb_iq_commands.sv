`timescale 1ns/1ps
module tb_iq_commands;
reg clk=0;always #7.6923076923 clk=~clk;
reg rst=0,rx=1;wire en,ch,ov;wire[3:0] mode,dec;wire[31:0] fcw,cfg;wire[1:0] rate;
ab5_uart_commands dut(.clk(clk),.rst_n(rst),.rx(rx),.demod_en(en),.demod_mode(mode),.ddc_fcw(fcw),.iq_ch(ch),.cfg_id(cfg),.decim_log2(dec),.override_auto(ov),.iq_rate(rate));
task send(input[7:0] b,input bad);integer j;begin
 rx=0;#500;for(j=0;j<8;j=j+1)begin rx=b[j];#500;end rx=!bad;#500;rx=1;#500;
end endtask
integer k;reg[31:0] old;
initial begin
 #100;rst=1;#100;
 if(rate!=1)$fatal(1,"default rate");
 for(k=0;k<3;k=k+1)begin old=cfg;send('ha5,0);send('he4,0);send(k,0);
  if(rate!=k || cfg!=old+1)$fatal(1,"rate command");end
 old=cfg;send('ha5,0);send('he4,0);send(3,0);if(cfg!=old || rate!=2)$fatal(1,"invalid rate");
 send('ha5,0);send('he4,0);send(0,1);if(cfg!=old || rate!=2)$fatal(1,"rate stop bit");
 send('ha5,0);send('he4,0);#120000;send(0,0);if(cfg!=old || rate!=2)$fatal(1,"rate timeout");
 for(k=0;k<=7;k=k+1)begin send('hA5,0);send('hE0,0);send(k,0);if(mode!=k || en!=(k!=0))$fatal(1,"mode");end
 old=cfg;send('hA5,0);send('hE0,0);send(8,0);if(cfg!=old)$fatal(1,"invalid mode");
 send('hA5,0);send('hE3,0);send(1,0);if(ch!=1)$fatal(1,"channel");
 old=fcw;send('hA5,0);send('hE1,0);send('h34,0);send('h12,0);if(fcw!=old)$fatal(1,"non atomic FCW");
 send('hA5,0);send('hE2,0);send('h78,0);send('h56,0);if(fcw!=32'h12345678)$fatal(1,"FCW bytes");
 old=cfg;send('hA5,0);send('hE0,0);#120000;send(1,0);if(cfg!=old)$fatal(1,"timeout");
 send('hA5,0);send('hE0,0);send(1,1);if(cfg!=old)$fatal(1,"bad stop");
 send('hA5,0);send(3,0);if(dec!=3||!ov)$fatal(1,"legacy");
 send('hA5,0);send('hFF,0);if(ov)$fatal(1,"restore");
 $display("IQ_COMMANDS_PASS");$finish;
end
endmodule
