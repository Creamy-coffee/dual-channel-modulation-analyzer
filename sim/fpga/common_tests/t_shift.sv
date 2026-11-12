`timescale 1ns/1ps
// Minimal reproduction of the ET-display signedness defect (2026-10-19).
// A concatenation operand makes the enclosing expression UNSIGNED, which
// silently turns `>>>` into a LOGICAL shift.  Run it and read the four lines:
// only v1 (the original et_display style) is wrong.
module t;
  reg signed [11:0] a, b;
  wire signed [12:0] d1w = {a[11],a} - {b[11],b};
  wire signed [12:0] s1  = {b[11],b} + (d1w >>> 4);     // WRONG: 507
  wire signed [12:0] d2w = $signed({a[11],a}) - $signed({b[11],b});
  wire signed [12:0] s2  = $signed({b[11],b}) + (d2w >>> 4);  // -5
  wire signed [12:0] sh3 = d1w >>> 4;
  wire signed [12:0] s3  = {b[11],b} + sh3;             // -5
  reg signed [12:0] d4;
  wire signed [12:0] s4 = d4 >>> 4;                     // -67 (after d4 is set)
  initial begin
    a = -1000; b = 62; #1;
    d4 = a - b;
    #1;
    $display("expect d=-1062 shift=-67 sum=-5");
    $display("v1 concatenation, shift inside sum : d=%0d s=%0d   <-- wrong", d1w, s1);
    $display("v2 $signed() casts                : d=%0d s=%0d", d2w, s2);
    $display("v3 separate signed shift wire      : sh=%0d s=%0d", sh3, s3);
    $display("v4 signed operands                  : d=%0d s=%0d", d4, s4);
    $finish;
  end
endmodule
