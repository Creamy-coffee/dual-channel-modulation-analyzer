`timescale 1ns/1ps
module tb_precision_ctrl;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,clear=0,reconfig=0,valid=0;
 reg [31:0] stamp=32'hfff00000,active=1982292598;
 reg [5:0] wl=18;
 reg signed [15:0] p0=0,p1=0;
 reg [1:0] pv=3;
 reg [23:0] a0=81920,a1=65536;
 wire commit,ready,ev;
 wire [31:0] target;
 wire signed [31:0] e0,e1;
 wire [7:0] reason;
 wire [31:0] estimates,updates;
 afc_precision_ctrl dut(.clk(clk),.rst_n(rst_n),.clear(clear),.reconfig(reconfig),
  .in_valid(valid),.timestamp(stamp),.current_fcw(active),.nominal_fcw(32'd1982292598),
  .window_log2(wl),.phase0(p0),.phase1(p1),.phase_valid(pv),.amp0(a0),.amp1(a1),
  .commit(commit),.target_fcw(target),.ready(ready),.estimate_valid(ev),
  .error0_fcw(e0),.error1_fcw(e1),.reason(reason),.estimates(estimates),.updates(updates));
 always @(posedge clk) if(commit) active<=target;
 real angle=32000.0,source_word=1982292598.0;
 integer i,j,checks=0;real offsets[0:7];
 function integer nearest(input real v);nearest=v>=0?$rtoi(v+0.5):$rtoi(v-0.5);endfunction
 task emit;
  input integer bad;
  begin
   @(negedge clk);
   angle=angle+(source_word-$itor(active))*(2.0**wl)/65536.0;
   stamp=stamp+(32'd1<<wl)+bad;p0=nearest(angle);p1=nearest(angle+3600.0);valid=1;
   @(negedge clk);valid=0;
   repeat(3) @(negedge clk);
  end
 endtask
 initial begin
  offsets[0]=0.5;offsets[1]=1;offsets[2]=3;offsets[3]=8;
  offsets[4]=-0.5;offsets[5]=-1;offsets[6]=-3;offsets[7]=-8;
  repeat(3) @(negedge clk);rst_n=1;
  for(j=0;j<8;j=j+1) begin
   clear=1;@(negedge clk);clear=0;active=1982292598;
   source_word=1982292598.0+nearest(offsets[j]*4294967296.0/65000000.0);
   for(i=0;i<350;i=i+1) emit(0);
   if(!ready || (source_word-$itor(active)>8) || (source_word-$itor(active)< -8))
    $fatal(1,"Fine lock failed df=%f active=%0d target=%f ready=%0b reason=%0d error=%0d",offsets[j],active,source_word,ready,reason,e0);
   checks=checks+1;$display("PASS offset=%f residual_fcw=%f fine_ready=%0b",offsets[j],source_word-$itor(active),ready);
  end
  reconfig=1;@(negedge clk);reconfig=0;wl=22;
  for(i=0;i<8;i=i+1) emit(0);
  if(!ready) $fatal(1,"Precision permission lost across clean gear transition");checks=checks+1;
  emit(1);if(ready || reason!=8) $fatal(1,"Bad physical timestamp accepted");checks=checks+1;
  for(i=0;i<8;i=i+1) emit(0);
  a1=0;emit(0);if(ready) $fatal(1,"Single-channel loss retained precision lock");checks=checks+1;
  a1=65536;for(i=0;i<8;i=i+1) emit(0);
  pv=1;emit(0);if(ready) $fatal(1,"Invalid numerical phase retained precision lock");checks=checks+1;
  $display("TB_PASS precision_ctrl checks=%0d (timestamps include wrap, phase crosses +/-180)",checks);$finish;
 end
endmodule
