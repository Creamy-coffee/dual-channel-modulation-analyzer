`timescale 1ns/1ps
module tb_keys_raw;
 reg clk=0;always #5 clk=~clk;
 reg rst=0;reg [3:0] keys=15;reg [2:0] sw=0;
 wire freeze,src,falling;wire [3:0] mode;
 basic_controls #(.DEBOUNCE_CLKS(3),.KEY_TIMEBASE(1)) u(
 .clk(clk),.rst_n(rst),.key_n(keys),.sw(sw),.freeze(freeze),
 .trig_src(src),.falling(falling),.timebase_log2(mode));
 task press;input integer i;begin
  @(negedge clk);keys[i]=0;repeat(8)@(negedge clk);
  keys[i]=1;repeat(8)@(negedge clk);
 end endtask
 initial begin
  repeat(3)@(negedge clk);rst=1;repeat(8)@(negedge clk);
  if(mode!=0)$fatal(1,"Default is not native RAW");
  press(2);if(mode!=4)$fatal(1,"R16");
  press(2);if(mode!=5)$fatal(1,"R32");
  press(2);if(mode!=6)$fatal(1,"R64");
  press(2);if(mode!=7)$fatal(1,"R128");
  // The manual list now reaches the slow display bases used by the
  // automatic time base (R=256/512/1024 -> 8/9/10) before wrapping to RAW.
  press(2);if(mode!=8)$fatal(1,"R256");
  press(2);if(mode!=9)$fatal(1,"R512");
  press(2);if(mode!=10)$fatal(1,"R1024");
  press(2);if(mode!=0)$fatal(1,"RAW wrap");
  sw=7;repeat(12)@(negedge clk);if(mode!=0)$fatal(1,"Unwired GPIO overrides board key mode");
  press(0);if(!freeze)$fatal(1,"Freeze");press(0);if(freeze)$fatal(1,"Resume");
  press(1);if(!src)$fatal(1,"Trigger source");press(3);if(!falling)$fatal(1,"Edge");
  $display("PASS board key mode: native RAW default, eight timebases up to R=1024, no external switch dependency, freeze/source/edge");$finish;
 end
endmodule
