`timescale 1ns/1ps
module tb_dual_uart_commands;
 reg clk=0;always #7.6923076923 clk=~clk;
 reg rst=0,rx0=1,rx1=1;
 wire [3:0] decim,mode;wire over,en,ch;wire [31:0] fcw,cfg;wire [1:0] rate;
 ab5_uart_commands #(.DUAL_RX(1)) dut(.clk(clk),.rst_n(rst),.rx(rx0),.rx_copy(rx1),
  .decim_log2(decim),.override_auto(over),.demod_en(en),.demod_mode(mode),
  .ddc_fcw(fcw),.iq_ch(ch),.cfg_id(cfg),.iq_rate(rate));
 task automatic send(input integer port,input [7:0] value);
  integer k;begin
   if(port)rx1=0;else rx0=0;#500;
   for(k=0;k<8;k=k+1)begin if(port)rx1=value[k];else rx0=value[k];#500;end
   if(port)rx1=1;else rx0=1;#1000;
  end
 endtask
 task automatic command(input integer port,input [7:0] opcode,input [7:0] operand);
  begin send(port,8'ha5);send(port,opcode);send(port,operand);end
 endtask
 task automatic halfword(input integer port,input [7:0] opcode,input [15:0] value);
  begin send(port,8'ha5);send(port,opcode);send(port,value[7:0]);send(port,value[15:8]);end
 endtask
 reg [31:0] previous_cfg,previous_fcw;
 initial begin
  #100;rst=1;#100;
  command(0,8'he0,2);if(mode!=2||!en)$fatal(1,"AA5 command failed");#150000;
  command(1,8'he3,1);if(!ch)$fatal(1,"AA4 channel failed");#150000;
  command(1,8'he4,2);if(rate!=2)$fatal(1,"AA4 rate failed");#150000;
  previous_cfg=cfg;
  fork command(0,8'he0,2);command(1,8'he0,1);join
  #150000;if(mode!=2||cfg!=previous_cfg+1)$fatal(1,"simultaneous-start priority/mixing");
  previous_cfg=cfg;
  fork command(1,8'he0,1);begin #200;command(0,8'he0,2);end join
  #150000;if(mode!=1||cfg!=previous_cfg+1)$fatal(1,"first-start ownership failed");
  halfword(0,8'he1,16'h1234);#200000;
  halfword(0,8'he2,16'h5678);if(fcw!=32'h12345678)$fatal(1,"same-source staging lost");#150000;
  previous_cfg=cfg;previous_fcw=fcw;
  halfword(0,8'he1,16'h1111);#150000;
  halfword(1,8'he2,16'h2222);
  if(cfg!=previous_cfg||fcw!=previous_fcw)$fatal(1,"cross-source FCW mixed");#150000;
  command(0,8'he3,0);#150000;
  send(0,8'ha5);send(0,8'he3);#150000;send(1,1);
  if(ch)$fatal(1,"cross-source partial command mixed");#150000;
  command(1,8'he0,7);if(mode!=7)$fatal(1,"AA4 PM mode failed");#150000;
  command(0,8'he0,0);if(en)$fatal(1,"AA5 disable failed");
  $display("DUAL_UART_COMMANDS_PASS primary secondary priority ownership staging partial commands");$finish;
 end
 initial begin #5000000;$fatal(1,"timeout");end
endmodule
