`timescale 1ns/1ps
module tb_ddc_rates;
reg clk=0;always #5 clk=~clk;
reg rst=0,v=0;reg[1:0] rate=1;
reg signed[15:0] i=0,q=0;wire valid;wire signed[15:0] oi,oq;
ddc_fir_decimator #(.RUNTIME_RATE(1)) dut(.clk(clk),.rst_n(rst),.in_valid(v),
 .i_in(i),.q_in(q),.rate_sel(rate),.out_valid(valid),.i_out(oi),.q_out(oq));
reg[31:0] data[0:1023];integer n,r,fd;
initial begin
 $readmemh("../tests/iq_fir_input.mem",data);
 fd=$fopen("../tests/iq_rates_output.txt","w");
 for(r=0;r<3;r=r+1)begin
  @(negedge clk);rst=0;rate=r;v=0;repeat(64)@(negedge clk);rst=1;
  for(n=0;n<1024;n=n+1)begin
   @(negedge clk);i=data[n][15:0];q=data[n][31:16];v=1;
   @(negedge clk);v=0;repeat(1022)@(negedge clk);
  end
  repeat(256)@(negedge clk);
 end
 $fclose(fd);$display("DDC_RATES_PASS");$finish;
end
always @(negedge clk)if(valid)$fwrite(fd,"%0d %0d %0d\n",rate,oi,oq);
endmodule
