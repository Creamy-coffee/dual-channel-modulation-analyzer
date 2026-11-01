`timescale 1ns/1ps
// Numerical contracts, full-width boundaries, reset cancellation and repeat use.
module tb_math_primitives;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,ds=0,ss=0;reg [63:0] num=0,den=1,rad=0;
 wire db,dv,sb,sv;wire [63:0] quotient;wire [31:0] root;
 fixed_divider_generic divider(.clk(clk),.rst_n(rst_n),.start(ds),.numerator(num),.denominator(den),.busy(db),.valid(dv),.quotient(quotient));
 isqrt_u64 sqrt_dut(.clk(clk),.rst_n(rst_n),.start(ss),.radicand(rad),.result(root),.busy(sb),.done(sv));
 integer checks=0,j,latency;reg [63:0] rng=64'h1a39bf72013589cd;
 reg [65:0] low_square,high_square;reg [32:0] next_root;
 task check_div(input [63:0] a,input [63:0] b);
  begin
   @(negedge clk);num=a;den=b;ds=1;@(negedge clk);ds=0;latency=0;
   while(!dv && latency<66)begin @(negedge clk);latency=latency+1;end
   if(!dv || db || quotient!==(a/b))$fatal(1,"division mismatch %h/%h -> %h",a,b,quotient);
   @(negedge clk);if(dv)$fatal(1,"division valid not pulsed");checks=checks+1;
  end
 endtask
 task check_sqrt(input [63:0] a);
  begin
   @(negedge clk);rad=a;ss=1;@(negedge clk);ss=0;latency=0;
   while(!sv && latency<35)begin @(negedge clk);latency=latency+1;end
   next_root={1'b0,root}+1'b1;
   low_square={34'd0,root}*{34'd0,root};high_square={33'd0,next_root}*{33'd0,next_root};
   if(!sv || sb || low_square>{2'd0,a} || high_square<={2'd0,a})$fatal(1,"sqrt contract rad=%h root=%h",a,root);
   @(negedge clk);if(sv)$fatal(1,"sqrt done not pulsed");checks=checks+1;
  end
 endtask
 initial begin
  repeat(4)@(negedge clk);rst_n=1;
  check_div(0,1);check_div(64'hffffffffffffffff,1);check_div(64'hffffffffffffffff,64'hffffffffffffffff);check_div(7,11);
  check_sqrt(0);check_sqrt(1);check_sqrt(2);check_sqrt(64'hffffffffffffffff);
  check_sqrt(64'hfffffffe00000001);check_sqrt(64'hfffffffe00000000);
  for(j=0;j<256;j=j+1)begin
   rng=rng^(rng<<13);rng=rng^(rng>>7);rng=rng^(rng<<17);
   check_div(rng,(rng>>17)|64'd1);check_sqrt(rng);
  end
  @(negedge clk);num=12345;den=7;rad=12345;ds=1;ss=1;
  @(negedge clk);ds=0;ss=0;repeat(5)@(negedge clk);rst_n=0;
  repeat(2)@(negedge clk);if(db||dv||sb||sv)$fatal(1,"reset failed to cancel work");rst_n=1;
  check_div(123456789,123);check_sqrt(123456789);
  $display("MATH_PRIMITIVES_PASS checks=%0d",checks);$finish;
 end
 initial begin #1000000;$fatal(1,"timeout");end
endmodule
