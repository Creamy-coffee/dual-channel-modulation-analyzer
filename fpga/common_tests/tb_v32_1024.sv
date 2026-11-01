`timescale 1ns/1ps
module tb_v32_1024;
 reg clk=0;always #7.6923076923 clk=~clk;
 reg rst=0,fault=0;
 reg [63:0] tick=0;
 wire signed [11:0] c0=tick[11:0],c1=~tick[11:0];
 wire tx;wire [31:0] completed,aborted;
 v32cap3_tx #(.POINTS(1024),.FRAME_PERIOD(0)) dut(
  .clk(clk),.rst_n(rst),.sample_valid(1'b1),.decim_log2(4'd0),.fault(fault),.ch0(c0),.ch1(c1),
  .tx(tx),.busy(),.completed(completed),.aborted(aborted),.skipped());
 always @(posedge clk)if(!rst)tick<=0;else tick<=tick+1'b1;
 initial begin
  repeat(5)@(negedge clk);rst=1;
  repeat(100)@(negedge clk);fault=1;@(negedge clk);fault=0;
 end
 integer fd,count=0,b;
 reg armed=1;
 reg [7:0] value;
 real bit_ns=1000000000.0/921600.0;
 always @(posedge tx)armed=1;
 initial begin
  fd=$fopen("../tests/v32_1024_wire.bin","wb");
  forever begin
   @(negedge tx);
   if(armed)begin
    armed=0;#(1.5*bit_ns);
    for(b=0;b<8;b=b+1)begin value[b]=tx;#(bit_ns);end
    if(tx!==1)$fatal(1,"V32 stop bit error at %0d",count);
    $fwrite(fd,"%c",value);count=count+1;
    if(count==2*4140)begin
     $fclose(fd);
     if(aborted!=1)$fatal(1,"fault did not abort partial capture: %0d",aborted);
     $display("TB_PASS V32_1024 physical_UART bytes=%0d frames=2 aborted=%0d",count,aborted);$finish;
    end
   end
  end
 end
 initial begin #250000000;$fatal(1,"V32 1024 timeout count=%0d",count);end
endmodule
