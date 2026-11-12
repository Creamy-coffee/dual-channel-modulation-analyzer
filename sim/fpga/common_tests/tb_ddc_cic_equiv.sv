`timescale 1ns/1ps
module tb_ddc_cic_equiv;
reg clk=0;always #5 clk=~clk;reg rst=0,clear=0,v=0;
reg signed[15:0] i=0,q=0;integer n,k;reg[31:0] rng=32'h71e318a5;
genvar g;generate for(g=2;g<=10;g=g+2)begin:rates
wire a,b;wire signed[15:0] ai,aq,bi,bq;
ddc_cic_decimator #(.LOG2_R(g)) new_dut(clk,rst,clear,v,i,q,a,ai,aq);
ddc_cic_pre_area #(.LOG2_R(g)) old_dut(clk,rst,clear,v,i,q,b,bi,bq);
always @(negedge clk)if(rst && ({a,ai,aq}!=={b,bi,bq}))$fatal(1,"CIC mismatch R=%0d",1<<g);
end endgenerate
initial begin #30;rst=1;
for(n=0;n<60000;n=n+1)begin @(negedge clk);rng=(rng>>1)^(rng[0]?32'hd0000001:0);
v=(n%17!=0);i=rng[15:0];q=rng[31:16];clear=(n==31000);end
$display("DDC_CIC_EQUIV_PASS cycles=60000 ratios=4,16,64,256,1024");$finish;end
endmodule
