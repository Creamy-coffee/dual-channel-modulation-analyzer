`timescale 1ns/1ps
module tb_ab5_commands;
 reg clk=0;always #7.6923076923 clk=~clk;
 reg rst=0,rx=1;
 wire [3:0] m;wire override_auto;
 ab5_uart_commands dut(.clk(clk),.rst_n(rst),.rx(rx),.decim_log2(m),.override_auto(override_auto));
 task send(input [7:0] b,input bad_stop);
  integer j;
  begin
   rx=0;#500;
   for(j=0;j<8;j=j+1)begin rx=b[j];#500;end
   rx=!bad_stop;#500;rx=1;#500;
  end
 endtask
 integer k;
 initial begin
  #100;rst=1;#100;
  for(k=0;k<=8;k=k+1)begin
   send(8'ha5,0);send(k,0);
   if(m!=k || !override_auto)$fatal(1,"command %0d",k);
  end
  send(8'ha5,0);send(9,0);if(m!=8)$fatal(1,"invalid operand changed config");
  send(8'ha5,0);#120000;send(2,0);if(m!=8)$fatal(1,"partial command did not expire");
  send(8'ha5,0);send(1,1);if(m!=8)$fatal(1,"bad stop changed config");
  send(8'ha5,0);send(8'hff,0);if(override_auto)$fatal(1,"auto restore failed");
  $display("AB5_COMMANDS_PASS");$finish;
 end
endmodule
