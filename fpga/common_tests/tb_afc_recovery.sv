`timescale 1ns/1ps
module tb_afc_recovery;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,ok=1,clear=0;
 reg [31:0] t=32'hfff00000,active=1982292598;
 wire [31:0] target;wire commit,ready;wire [2:0] state;wire [7:0] reason;
 reg signed [27:0] i0=0,q0=0,i1=0,q1=0;
 integer count=0;real theta=0,offset=1200,peak=4,delta;
 local_afc_2ch dut(.clk(clk),.rst_n(rst),.clear(clear),.stream_ok(ok),.mix_valid(ok),
  .mi0(i0),.mq0(q0),.mi1(i1),.mq1(q1),.sample_time(t),
  .nominal_fcw(32'd1982292598),.current_fcw(active),.target_fcw(target),.commit(commit),
  .ready_measurement(ready),.state(state),.reason(reason),.residual0_fcw(),.residual1_fcw(),
  .updates(),.observation_id(),.last_timestamp());
 always @(posedge clk) begin t<=t+1;if(commit) active<=target;end
 always @(negedge clk) if(rst && ok) begin
  if(count==0) begin
   delta=offset-($itor(active)-1982292598.0)*65000000.0/4294967296.0;
   theta=theta+6.283185307179586*delta*4096/65000000.0;
   i0=$rtoi(peak*16384*$cos(theta));q0=$rtoi(peak*16384*$sin(theta));
   i1=$rtoi(peak*16384*$cos(theta+0.5));q1=$rtoi(peak*16384*$sin(theta+0.5));
  end
  count=(count+1)%4096;
 end
 task acquire;
  integer n;
  begin
   n=0;
   while(!ready && n<2200000) begin @(negedge clk);n=n+1;end
   if(!ready) $fatal(1,"Did not acquire state=%0d reason=%0d",state,reason);
   delta=offset-($itor(active)-1982292598.0)*65000000.0/4294967296.0;
   if(delta>8 || delta< -8) $fatal(1,"Residual too large %f",delta);
  end
 endtask
 initial begin
  repeat(4) @(negedge clk);rst=1;
  acquire();$display("PASS initial capture and timestamp wrap");
  offset=1300;repeat(200000) @(negedge clk);acquire();$display("PASS drift following");
  ok=0;repeat(5) @(negedge clk);if(ready) $fatal(1,"Gap valid");
  ok=1;count=0;acquire();$display("PASS gap recovery using active FCW");
  peak=0;repeat(20000) @(negedge clk);if(ready) $fatal(1,"No-signal valid");
  peak=4;acquire();$display("PASS signal return reacquisition");
  $finish;
 end
 initial begin #120000000;$fatal(1,"Recovery timeout");end
endmodule
