`timescale 1ns/1ps
module tb_ddc_fir;
reg clk=0;always #5 clk=~clk;reg rst=0,v=0;reg signed[15:0] i=0,q=0;wire ov;wire signed[15:0] oi,oq;
ddc_fir_decimator dut(.clk(clk),.rst_n(rst),.in_valid(v),.i_in(i),.q_in(q),
 .rate_sel(2'd1),.out_valid(ov),.i_out(oi),.q_out(oq));
reg[31:0] data[0:1023];integer n,fd;
initial begin $readmemh("../tests/iq_fir_input.mem",data);fd=$fopen("../tests/iq_fir_output.txt","w");#30;rst=1;
for(n=0;n<1024;n=n+1)begin @(negedge clk);i=data[n][15:0];q=data[n][31:16];v=1;@(negedge clk);v=0;repeat(1022)@(negedge clk);end
repeat(256)@(posedge clk);$fclose(fd);$display("DDC_FIR_PASS");$finish;end
always @(negedge clk)if(ov)$fwrite(fd,"%0d %0d\n",oi,oq);
endmodule
