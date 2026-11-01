`timescale 1ns/1ps
module tb_low_timebase;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,update=0,freeze=1,busy=0;
 reg [31:0] f=0;
 wire [3:0] m,actual;wire sat,snap;
 reg [3:0] addr=0;wire signed [11:0] y0,y1;wire [31:0] interval_ns;
 auto_timebase #(.MAX_M(15)) t(.clk(clk),.rst_n(rst),.enable(1'b1),.freq_hz(f),.freq_valid(1'b1),
  .freq_update(update),.m(m),.saturated(sat),.auto_active(),.changes());
 // Smaller RAM shortens simulation; the accumulator/count/interval are the
 // exact production widths. Controller still selects against 1024 display pts.
 acq_trigger_buffer #(.DISP_AW(4),.AUTO_TIMEOUT(16),.MAX_DECIM_LOG2(15)) a(
  .clk(clk),.rst_n(rst),.sample_valid(1'b1),.ch0(12'sd2047),.ch1(-12'sd2048),.fault(2'd0),
  .trig_level(12'sd0),.trig_mode(2'd0),.trig_src(1'b0),.pretrig(4'd4),.timebase_log2(m),
  .freeze(freeze),.disp_busy(busy),.snap_rd_addr(addr),.snap_ready(snap),.snap_decim_log2(actual),
  .snap_rd_ch0(y0),.snap_rd_ch1(y1),.snap_sample_interval_ns(interval_ns));
 integer k,n,cases=0;
 task check(input integer hz,input integer want,input integer saturation);
 begin
  @(negedge clk);rst=0;freeze=1;busy=0;f=hz;update=0;
  repeat(4)@(negedge clk);rst=1;
  repeat(20)@(negedge clk);if(m!=0)$fatal(1,"counted held valid as updates");
  for(k=0;k<4;k=k+1)begin update=1;@(negedge clk);update=0;repeat(2)@(negedge clk);end
  if(m!=want || sat!=saturation)$fatal(1,"timebase hz=%0d m=%0d sat=%0d",hz,m,sat);
  freeze=0;n=0;while(!snap && n<600000)begin @(negedge clk);n=n+1;end
  if(!snap || actual!=want)$fatal(1,"capture didn't execute slow rate");
  if(interval_ns!=((64'd1000000000*(64'd1<<want)+32500000)/65000000))$fatal(1,"wrong interval");
  busy=1;for(k=0;k<16;k=k+1)begin addr=k;repeat(2)@(negedge clk);
   if(y0!=2047 || y1!= -2048)$fatal(1,"signed average overflow at m=%0d",want);
  end
  cases=cases+1;$display("PASS slow timebase hz=%0d m=%0d sample_ns=%0d",hz,m,interval_ns);
 end
 endtask
 initial begin
  check(10,15,0);check(100,12,0);check(1000,9,0);check(100000,2,0);check(30000000,0,0);check(1,15,1);
  $display("TB_PASS LOW_TIMEBASE cases=%0d",cases);$finish;
 end
endmodule
