`timescale 1ns/1ps
module tb_controls;
 reg clk=0,rst=0;always #5 clk=~clk;
 reg [3:0] keys=15;reg [2:0] sw=0;wire freeze,src,falling;wire [3:0] rate;
 basic_controls #(.DEBOUNCE_CLKS(4)) dut(.clk(clk),.rst_n(rst),.key_n(keys),.sw(sw),
 .freeze(freeze),.trig_src(src),.falling(falling),.timebase_log2(rate));
 integer n;
 task press(input integer k);
  begin keys[k]=0;repeat(12)@(negedge clk);keys[k]=1;repeat(12)@(negedge clk);end
 endtask
 initial begin
  repeat(3)@(negedge clk);rst=1;repeat(4)@(negedge clk);
  if(rate!=4 || freeze || src || falling)$fatal(1,"control reset");
  // A single-cycle bounce cannot toggle the state.
  keys[0]=0;@(negedge clk);keys[0]=1;repeat(12)@(negedge clk);
  if(freeze)$fatal(1,"debounce failed");
  press(0);if(!freeze)$fatal(1,"freeze toggle");press(0);if(freeze)$fatal(1,"resume toggle");
  press(1);if(!src)$fatal(1,"trigger source toggle");
  press(3);if(!falling)$fatal(1,"edge toggle");
  for(n=0;n<4;n=n+1)begin sw=n;repeat(12)@(negedge clk);if(rate!=4+n)$fatal(1,"timebase");end
  sw=4;repeat(12)@(negedge clk);if(rate!=0)$fatal(1,"native RAW mode absent");
  $display("PASS controls: two-stage sync, debounce, freeze/source/edge, four slow views and native RAW");$finish;
 end
endmodule
