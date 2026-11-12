`timescale 1ns/1ps
module spec_equiv_lane #(parameter MANUAL=0)(
 input clk,rst_n,wr_en,scan_done,send_start,
 input [11:0] bin,input [32:0] mag,input [31:0] wave_id,input [1:0] window_sel
);
 wire tx_new,tx_old,busy_new,busy_old,ready_new,ready_old;
 wire [31:0] done_new,done_old,result_new,result_old;
 v32spec_tx #(.BINS(64),.MANUAL_START(MANUAL)) candidate(
  .clk(clk),.rst_n(rst_n),.wr_en(wr_en),.wr_bin(bin),.wr_mag(mag),.scan_done(scan_done),
  .send_start(send_start),.snapshot_ready(ready_new),.wave_frame_id(wave_id),.window_sel(window_sel),
  .tx(tx_new),.busy(busy_new),.completed(done_new),.result_id(result_new));
 v32spec_tx_reference #(.BINS(64),.MANUAL_START(MANUAL)) reference(
  .clk(clk),.rst_n(rst_n),.wr_en(wr_en),.wr_bin(bin),.wr_mag(mag),.scan_done(scan_done),
  .send_start(send_start),.snapshot_ready(ready_old),.wave_frame_id(wave_id),.window_sel(window_sel),
  .tx(tx_old),.busy(busy_old),.completed(done_old),.result_id(result_old));
 always @(negedge clk)if(rst_n &&
  ({tx_new,busy_new,ready_new,done_new,result_new} !== {tx_old,busy_old,ready_old,done_old,result_old}))
  $fatal(1,"Spectrum differs from baseline manual=%0d",MANUAL);
endmodule
module tb_spec_resource_equiv;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,wr_en=0,scan_done=0,send_start=0;
 reg [11:0] bin=0;
 reg [32:0] mag=0;
 reg [31:0] wave_id=1;
 reg [1:0] window_sel=0;
 spec_equiv_lane automatic_lane(.clk(clk),.rst_n(rst_n),.wr_en(wr_en),.scan_done(scan_done),.send_start(send_start),.bin(bin),.mag(mag),.wave_id(wave_id),.window_sel(window_sel));
 spec_equiv_lane #(.MANUAL(1)) manual_lane(.clk(clk),.rst_n(rst_n),.wr_en(wr_en),.scan_done(scan_done),.send_start(send_start),.bin(bin),.mag(mag),.wave_id(wave_id),.window_sel(window_sel));
 integer frame,k;
 initial begin
  repeat(5)@(negedge clk);#1;rst_n=1;
  for(frame=0;frame<2;frame=frame+1)begin
   for(k=0;k<64;k=k+1)begin
    @(negedge clk);#1;wr_en=1;bin=k;mag=(k%3==0)?0:(33'd16<<(k%24));
   end
   @(negedge clk);#1;wr_en=0;scan_done=1;
   @(negedge clk);#1;scan_done=0;
   repeat(500)@(negedge clk);
   #1;send_start=1;
   @(negedge clk);#1;send_start=0;
   // Incoming scans and metadata must not disturb the frozen packet.
   for(k=0;k<37000;k=k+1)begin
    @(negedge clk);#1;wr_en=1;bin=k%64;mag=33'd8192+k;
    wave_id=wave_id+1;window_sel=k%3;
   end
   wr_en=0;
   if(manual_lane.done_new!=frame+1 || automatic_lane.done_new!=frame+1)$fatal(1,"missing packet");
  end
  $display("SPEC_RESOURCE_EQUIV_PASS frames=2 modes=auto,manual bytes=448");$finish;
 end
endmodule
