`timescale 1ns/1ps
module tb_iq_stream;
parameter STRESS=0;
parameter [1:0] RATE=1;
parameter RATE_TEST=0;
parameter POINTS=16;
reg clk=0;always #7.6923076923 clk=~clk;
reg rst=0,valid=0;reg signed[15:0] i=0,q=0;wire tx;wire[31:0] completed,overflow;
iq_stream_tx #(.POINTS(POINTS),.FIFO_DEPTH(2*POINTS),.RUNTIME_RATE(1)) dut(.clk(clk),.rst_n(rst),.iq_valid(valid),.i_in(i),.q_in(q),.rate_sel(RATE),.cfg_id(32'd19),.tx(tx),.busy(),.completed(completed),.overflow(overflow));
integer n,fd,k,count=0;reg[7:0] b;
initial begin #100;rst=1;for(n=0;n<10*POINTS;n=n+1)begin
 repeat(STRESS?1:(RATE==0?8191:RATE==2?2047:4095))@(negedge clk);i=n-80;q=80-n;valid=1;@(negedge clk);valid=0;
end
 wait(completed==(STRESS?2:10));#6000;
 if(!STRESS && overflow!=0)$fatal(1,"unexpected overflow");
 if(STRESS && overflow==0)$fatal(1,"missing overflow");
 $display("IQ_STREAM_PASS frames=%0d overflow=%0d",completed,overflow);$fclose(fd);$finish;
end
initial begin
 if(RATE_TEST)fd=$fopen($sformatf("../tests/iq_rate_%0d.bin",RATE),"wb");
 else fd=$fopen(STRESS?"../tests/iq_stress.bin":"../tests/iq_stream.bin","wb");
 forever begin @(negedge tx);#750;for(k=0;k<8;k=k+1)begin b[k]=tx;#500;end
 if(tx!==1)$fatal(1,"stop bit");$fwrite(fd,"%c",b);count=count+1;end
end
initial begin #500000000;$fatal(1,"stream timeout");end
endmodule
