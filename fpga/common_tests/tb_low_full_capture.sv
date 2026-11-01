`timescale 1ns/1ps
module tb_low_full_capture;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,busy=0,update=0;
 reg [9:0] addr=0;
 wire [3:0] m,actual;
 wire snap;wire signed [11:0] y0,y1;
 reg [31:0] phase=0;
 reg signed [15:0] lut[0:4095];
 reg signed [11:0] x0=0,x1=0;
 wire [31:0] interval_ns;
 integer i,n,fd,k;
 auto_timebase #(.MAX_M(15)) t(.clk(clk),.rst_n(rst),.enable(1'b1),
  .freq_hz(32'd10),.freq_valid(1'b1),.freq_update(update),.m(m),.auto_active(),.saturated(),.changes());
 acq_trigger_buffer #(.AUTO_TIMEOUT(3250000),.MAX_DECIM_LOG2(15)) a(
  .clk(clk),.rst_n(rst),.sample_valid(1'b1),.ch0(x0),.ch1(x1),.fault(2'd0),
  .trig_level(12'sd0),.trig_mode(2'd0),.trig_src(1'b0),.pretrig(10'd256),.timebase_log2(m),
  .freeze(m!=15),.disp_busy(busy),.snap_rd_addr(addr),.snap_ready(snap),
  .snap_rd_ch0(y0),.snap_rd_ch1(y1),.snap_decim_log2(actual),.snap_sample_interval_ns(interval_ns));
 // Independent physical 10-Hz source (FCW quantization is explicit).
 always @(negedge clk)begin
  phase<=phase+32'd661;
  x0<= -4+(200*$signed(lut[phase[31:20]]))/32768;
  x1<=8+(100*$signed(lut[(phase[31:20]+12'd1024)&12'hfff]))/32768;
 end
 initial begin
  for(i=0;i<4096;i=i+1)lut[i]=$rtoi(32767*$sin(6.283185307179586*i/4096));
  repeat(5)@(negedge clk);rst=1;
  for(k=0;k<4;k=k+1)begin update=1;@(negedge clk);update=0;repeat(2)@(negedge clk);end
  n=0;while(!snap && n<50000000)begin @(negedge clk);n=n+1;end
  if(!snap || actual!=15 || interval_ns!=504123)$fatal(1,"full capture timebase mismatch");
  busy=1;fd=$fopen("../tests/low_full_capture.csv","w");$fwrite(fd,"ch0,ch1\n");
  for(i=0;i<1024;i=i+1)begin addr=i;repeat(2)@(negedge clk);$fwrite(fd,"%0d,%0d\n",y0,y1);end
  $fclose(fd);
  $display("TB_PASS FULL_LOW_CAPTURE samples=1024 ratio=32768 sample_ns=%0d cycles=%0d",interval_ns,n);$finish;
 end
endmodule
