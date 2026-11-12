`timescale 1ns/1ps
module tb_cic_low;
 parameter integer LR=4;
 localparam R=1<<LR;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,clear=0,iv=1;
 reg signed [19:0] x0=0,x1=0;
 wire ov;wire signed [19:0] y0,y1;
 cic3_decimate_2ch #(.LOG2_R(LR)) d(.clk(clk),.rst_n(rst),.clear(clear),
  .in_valid(iv),.in0(x0),.in1(x1),.out_valid(ov),.out0(y0),.out1(y1));
 integer t,n,k,cases=0,seen;
 real f,w,si,co,a,ratio,pass_amp,alias_amp,theory,db;
 task reset_d;
 begin @(negedge clk);rst=0;repeat(4)@(negedge clk);rst=1;end
 endtask
 task dc(input integer value);
 begin
  reset_d();x0=value*256;x1= -value*256;seen=0;
  for(t=0;t<12*R;t=t+1)begin
   @(negedge clk);
   if(ov && t>8*R)begin
    if(y0!=value*256 || y1!= -value*256)$fatal(1,"DC gain/sign LR=%0d value=%0d y=%0d/%0d",LR,value,y0,y1);
    seen=seen+1;
   end
  end
  if(seen==0)$fatal(1,"no DC output");cases=cases+1;
 end
 endtask
 task tone(input real normalized_input,output real measured);
 begin
  reset_d();f=normalized_input;w=6.283185307179586*f;si=0;co=0;n=0;
  for(t=0;t<4096*R+16*R;t=t+1)begin
   x0=$rtoi(900.0*256*$sin(w*t));x1=x0;
   @(negedge clk);
   if(ov && t>=16*R)begin
    si=si+$itor(y0)*$sin(w*t);co=co+$itor(y0)*$cos(w*t);n=n+1;
   end
  end
  measured=2*$sqrt(si*si+co*co)/n/256;
 end
 endtask
 initial begin
  dc(2047);dc(-2047);dc(1);dc(-1);
  tone(1.0/(16*R),pass_amp);
  theory=900*$pow($sin(3.141592653589793/16)/(R*$sin(3.141592653589793/(16*R))),3);
  if($abs(pass_amp-theory)>0.2)$fatal(1,"passband LR=%0d got=%f theory=%f",LR,pass_amp,theory);
  tone(15.0/(16*R),alias_amp);
  ratio=alias_amp/pass_amp;db=20*$ln(ratio)/$ln(10.0);
  if(db> -65)$fatal(1,"alias rejection LR=%0d db=%f",LR,db);
  $display("TB_PASS CIC LR=%0d dc_cases=%0d pass_amp=%f theory=%f alias_amp=%f alias_dBc=%f",LR,cases,pass_amp,theory,alias_amp,db);
  $finish;
 end
endmodule
