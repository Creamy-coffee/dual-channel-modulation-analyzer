`timescale 1ns/1ps
// wave_decim 逐拍对照参考模型。
// 覆盖: R=1 直通、2..32768 各档块的均值、负数四舍五入、块内 OTR、
//       抽取比在累加中途变大/变小（自动时基换档）。
module tb_wave_decim;
 reg clk=0, rst_n=0, sv=0;
 reg [1:0] otr=0;
 reg signed [11:0] c0=0, c1=0;
 reg [3:0] lg=0;
 wire ov; wire signed [11:0] o0, o1; wire of;

 wave_decim #(.MAX_LOG2(15), .SAMPLE_W(12)) dut(
  .clk(clk), .rst_n(rst_n), .sample_valid(sv), .ch0(c0), .ch1(c1), .otr(otr),
  .decim_log2(lg), .out_valid(ov), .out0(o0), .out1(o1), .out_fault(of));

 always #5 clk = ~clk;

 integer errors=0, checks=0, valid_cnt=0;
 reg signed [27:0] acc0=0, acc1=0;
 integer cnt=0;
 reg oacc=0;
 reg signed [27:0] e0=0, e1=0;
 reg ev=0, ef=0;
 reg [3:0] rlg_q=0;

 wire [14:0] last = (15'd1 << lg) - 15'd1;
 reg signed [27:0] rnd;
 reg signed [27:0] x0, x1;
 always @* begin
  rnd = (lg == 4'd0) ? 28'sd0 : (28'sd1 << (lg - 4'd1));
  x0  = {{(28-12){c0[11]}}, c0};
  x1  = {{(28-12){c1[11]}}, c1};
 end

 // 参考模型：与 DUT 完全相同的算术，独立写一遍
 always @(posedge clk) begin
  if (!rst_n) begin
   rlg_q = lg; acc0 = 0; acc1 = 0; cnt = 0; oacc = 0; ev = 0; ef = 0;
  end else begin
   ev = 0;
   if (lg != rlg_q) begin
    rlg_q = lg; acc0 = 0; acc1 = 0; cnt = 0; oacc = 0;
   end else if (sv) begin
    if (cnt >= last) begin
     e0   = (acc0 + x0 + rnd) >>> lg;
     e1   = (acc1 + x1 + rnd) >>> lg;
     if (e0 > 27'sd2047) e0 = 27'sd2047; else if (e0 < -27'sd2048) e0 = -27'sd2048;
     if (e1 > 27'sd2047) e1 = 27'sd2047; else if (e1 < -27'sd2048) e1 = -27'sd2048;
     ef   = oacc | (|otr);
     ev   = 1;
     acc0 = 0; acc1 = 0; cnt = 0; oacc = 0;
    end else begin
     acc0 = acc0 + x0;
     acc1 = acc1 + x1;
     cnt  = cnt + 1;
     oacc = oacc | (|otr);
    end
   end
  end
 end

 task feed(input signed [11:0] a, input signed [11:0] b, input o);
  begin
   @(negedge clk); c0 = a; c1 = b; otr = {1'b0, o}; sv = 1;
   @(posedge clk); #1;
   checks = checks + 1;
   if (ov !== ev) begin
    errors = errors + 1;
    $display("FAIL t=%0t lg=%0d valid dut=%b exp=%b cnt=%0d", $time, lg, ov, ev, cnt);
   end else if (ev) begin
    valid_cnt = valid_cnt + 1;
    if (o0 !== e0 || o1 !== e1) begin
     errors = errors + 1;
     $display("FAIL t=%0t lg=%0d data dut=(%0d,%0d) exp=(%0d,%0d) c0=%0d x0=%0d rnd=%0d", $time, lg, o0, o1, e0, e1, c0, x0, rnd);
     $display("     dut.acc0=%0d dut.cnt=%0d dut.last=%0d | ref.acc0=%0d ref.cnt=%0d ref.last=%0d",
              dut.acc0, dut.cnt, dut.last, acc0, cnt, last);
    end
    if (of !== ef) begin
     errors = errors + 1;
     $display("FAIL t=%0t lg=%0d fault dut=%b exp=%b", $time, lg, of, ef);
    end
   end
  end
 endtask

 integer li, k;
 reg signed [11:0] a, b;

 task run_points(input integer n, input integer otr_period);
  begin
   for (k = 0; k < n; k = k + 1) begin
    a = $random; b = $random;
    if (k % 19 == 0) a = 12'sh7ff;
    if (k % 19 == 1) a = 12'sh800;
    if (k % 19 == 2) a = 12'sd0;
    if (k % 19 == 3) a = -12'sd1;
    feed(a, b, (otr_period != 0 && (k % otr_period) == 0) ? 1'b1 : 1'b0);
   end
  end
 endtask

 initial begin
  rst_n = 0; sv = 0;
  repeat (4) @(negedge clk);
  rst_n = 1;
  repeat (2) @(negedge clk);

  // 1) 各档递增：R=1 直通到 R=32768
  for (li = 0; li <= 15; li = li + 1) begin
   lg = li[3:0];
   repeat (2) @(negedge clk);
   run_points(220, 11);
  end

  // 2) 各档递减：抽取比在累加中途变小，不能卡住
  for (li = 15; li >= 0; li = li - 1) begin
   lg = li[3:0];
   run_points(220, 7);
  end

  // 3) 极值块：R=8 全 +2047 和全 -2048，检查不溢出
  lg = 4'd3;
  for (k = 0; k < 8; k = k + 1) feed(12'sh7ff, 12'sh800, 1'b0);
  for (k = 0; k < 8; k = k + 1) feed(12'sh800, 12'sh7ff, 1'b0);

  $display("checks=%0d valid=%0d errors=%0d", checks, valid_cnt, errors);
  if (errors == 0) $display("WAVE_DECIM_PASS");
  else             $display("WAVE_DECIM_FAIL");
  $finish;
 end
endmodule
