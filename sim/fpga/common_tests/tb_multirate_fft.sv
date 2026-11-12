`timescale 1ns/1ps
module tb_multirate_fft;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,enable=1,valid=1;
 reg signed [11:0] x0=0,x1=0;
 wire rv,sat;wire [511:0] pkt;wire [127:0] freq,conversion;
 wire [1:0] q;wire [3:0] rate;wire [31:0] age;wire [2:0] band;
 multirate_fft_frontend d(.clk(clk),.rst_n(rst),.enable(enable),.adc_valid(valid),
  .adc_ch0(x0),.adc_ch1(x1),.result_valid(rv),.saturated(sat),.result_packet(pkt),
  .interp_freq(freq),.conversion_frequency(conversion),.qualified(q),.analysis_log2(rate),.age_ticks(age),.analysis_band(band));
 reg signed [15:0] lut[0:65535];
 reg [31:0] phase=0,fcw,lfsr=32'h3a9f821d;
 integer hz=10,amp=200,want=14,tolerance=2,i,cycles=0,matched=0,raw_frames=0,frames=0,zeros=0;
 integer budget;
 real arg;
 initial begin
  if($value$plusargs("HZ=%d",hz))begin end
  if($value$plusargs("AMP=%d",amp))begin end
  if($value$plusargs("RATE=%d",want))begin end
  if($value$plusargs("TOL=%d",tolerance))begin end
  if($value$plusargs("ZERO=%d",zeros))begin end
  fcw=(64'd4294967296*hz+32500000)/65000000;
  // Two visits may have other ready-rate analyses and native RF frames between
  // them; budget includes those mandatory shared-engine service slots.
  budget=want>=14?150000000:want==12?40000000:want==8?8000000:8000000;
  for(i=0;i<65536;i=i+1)lut[i]=$rtoi(32767*$sin(6.283185307179586*i/65536));
  repeat(5)@(negedge clk);rst=1;
 end
 always @(negedge clk)if(rst)begin
  phase<=phase+fcw;
  lfsr<={lfsr[30:0],lfsr[31]^lfsr[21]^lfsr[1]^lfsr[0]};
  x0<=(zeros?0:(amp*$signed(lut[phase[31:16]]))/32768)-4+$signed({1'b0,lfsr[0]})-$signed({1'b0,lfsr[1]});
  x1<=8+$signed({1'b0,lfsr[2]})-$signed({1'b0,lfsr[3]});
  cycles<=cycles+1;
  if(rv)begin
   frames=frames+1;
   if(rate==0)raw_frames=raw_frames+1;
   if(q[1])$fatal(1,"empty CH1 falsely qualified band=%0d mag=%0d",band,{pkt[224],pkt[223:192]});
   if(zeros && q!=0)$fatal(1,"zero input falsely qualified");
   if(!zeros && q[0] && (freq[31:0]+tolerance<hz || freq[31:0]>hz+tolerance))
    $fatal(1,"wrong-band false candidate hz=%0d got=%0d rate=%0d",hz,freq[31:0],rate);
   if(q[0] && rate==want)begin
    if(freq[31:0]+tolerance<hz || freq[31:0]>hz+tolerance)
     $fatal(1,"wrong physical frequency hz=%0d got=%0d rate=%0d",hz,freq[31:0],rate);
    matched=matched+1;
    $display("MATCH hz=%0d amp=%0d f=%0d rate=%0d age_ticks=%0d raw_frames=%0d cycles=%0d",hz,amp,freq[31:0],rate,age,raw_frames,cycles);
   end
   if(matched==2)begin
    if(raw_frames<2)$fatal(1,"LF capture blocked RF service");
    $display("TB_PASS MULTIRATE hz=%0d amp=%0d matches=%0d frames=%0d raw_frames=%0d",hz,amp,matched,frames,raw_frames);$finish;
   end
  end
  if(cycles>=budget)begin
   if(zeros && frames>10 && raw_frames>3)begin $display("TB_PASS MULTIRATE_ZERO frames=%0d raw_frames=%0d",frames,raw_frames);$finish;end
   else $fatal(1,"timeout hz=%0d amp=%0d matched=%0d frames=%0d raw=%0d",hz,amp,matched,frames,raw_frames);
  end
 end
endmodule
