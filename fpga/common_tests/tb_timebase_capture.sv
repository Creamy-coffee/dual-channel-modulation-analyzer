`timescale 1ns/1ps
module tb_timebase_capture;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,update=0,freeze=1,busy=0;
 reg [31:0] f=0;
 wire [3:0] m,actual;wire sat,snap;
 reg [9:0] addr=0;wire signed [11:0] y0,y1;wire [31:0] interval_ns;
 auto_timebase #(.MAX_M(10)) t(.clk(clk),.rst_n(rst_n),.enable(1'b1),.freq_hz(f),.freq_valid(1'b1),
  .freq_update(update),.m(m),.saturated(sat),.auto_active(),.changes());
 acq_trigger_buffer #(.AUTO_TIMEOUT(16),.MAX_DECIM_LOG2(10)) a(
  .clk(clk),.rst_n(rst_n),.sample_valid(1'b1),.ch0(12'sd2047),.ch1(-12'sd2048),.fault(2'd0),
  .trig_level(12'sd0),.trig_mode(2'd0),.trig_src(1'b0),.pretrig(10'd256),.timebase_log2(m),
  .freeze(freeze),.disp_busy(busy),.snap_rd_addr(addr),.snap_ready(snap),.snap_decim_log2(actual),
  .snap_rd_ch0(y0),.snap_rd_ch1(y1),.snap_sample_interval_ns(interval_ns));
 integer cases=0,k,cycles;
 task run_case(input integer hz,input integer expected_m,input integer expected_sat);
 begin
  @(negedge clk);rst_n=0;freeze=1;busy=0;f=hz;update=0;
  repeat(4)@(negedge clk);rst_n=1;
  // A held validity level must never count as independent measurements.
  repeat(50)@(negedge clk);
  if(m!=0)$fatal(1,"dwell advanced without frequency update");
  for(k=0;k<4;k=k+1)begin
   update=1;@(negedge clk);update=0;
   repeat(10)@(negedge clk);
   if(k<3 && m!=0)$fatal(1,"changed before four updates");
  end
  if(m!=expected_m || sat!=expected_sat)$fatal(1,"controller setting f=%0d m=%0d sat=%0d",hz,m,sat);
  freeze=0;cycles=0;
  while(!snap && cycles<1200000)begin @(posedge clk);#1;cycles=cycles+1;end
  if(!snap || actual!=expected_m)$fatal(1,"capture ceiling differs from controller");
  if(interval_ns!=((64'd1000000000*(64'd1<<expected_m)+32500000)/65000000))$fatal(1,"incorrect interval");
  @(negedge clk);busy=1;
  for(k=0;k<1024;k=k+1)begin
   addr=k;repeat(2)@(negedge clk);
   if(y0!=2047 || y1!= -2048)$fatal(1,"signed full-scale average overflow at m=%0d",expected_m);
  end
  cases=cases+1;$display("PASS timebase/capture %0d Hz m=%0d interval_ns=%0d saturation=%0d",hz,m,interval_ns,sat);
 end
 endtask
 initial begin
  run_case(1000,9,0);run_case(500,10,0);run_case(100,10,1);
  run_case(30000000,0,0);
  $display("TB_PASS timebase_capture cases=%0d 4096 paired boundary samples",cases);$finish;
 end
endmodule
