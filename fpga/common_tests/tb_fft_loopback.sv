`timescale 1ns/1ps
module tb_fft_loopback;
 parameter [31:0] FCW0=32'd66076,FCW1=32'd66076420;
 parameter integer AMP0=400,AMP1=200,LOW_EXPECT_CH=1,TRANSITION=0;
 reg clk=0;always #5 clk=~clk;
 reg rst=0;reg [31:0] phase0=0,phase1=0;
 reg signed [15:0] lut[0:4095];reg signed [11:0] x0=0,x1=0;
 wire valid,rv;wire [31:0] spec_fs;wire [11:0] bin;wire [32:0] power;
 wire [127:0] frequencies;wire [1:0] qual;wire [3:0] rate;
 wire tx,busy,ready;wire [31:0] completed;
 reg active=0;reg low_seen=0;
 reg drive_high=0,high_seen=0;integer before_high;
 integer fd,j,k;reg [7:0] b;reg [1023:0] filename;
 multirate_fft_frontend fft(
  .clk(clk),.rst_n(rst),.enable(1'b1),.adc_valid(1'b1),.adc_ch0(x0),.adc_ch1(x1),
  .result_valid(rv),.interp_freq(frequencies),.qualified(qual),.analysis_log2(rate),
  .spectrum_valid(valid),.spectrum_fs_hz(spec_fs),.spectrum_bin(bin),.spectrum_power(power));
 wire done=valid && active && bin==2048;
 v32spec_tx #(.RUNTIME_FS(1)) spec(.clk(clk),.rst_n(rst),
  .wr_en(valid && (active||bin==1)),
  .wr_bin(bin==2048?12'd0:bin),.wr_mag(bin==2048?33'd0:power),
  .scan_done(done),.snapshot_fs_hz(spec_fs),.send_start(1'b0),.snapshot_ready(ready),
  .wave_frame_id(32'd7),.window_sel(2'd0),.tx(tx),.busy(busy),.completed(completed),.result_id());
 always @(negedge clk)begin
  phase0<=phase0+(drive_high?FCW1:FCW0);phase1<=phase1+FCW1;
  x0<=(AMP0*$signed(lut[phase0[31:20]]))/32768;
  x1<=(AMP1*$signed(lut[phase1[31:20]]))/32768;
 end
 always @(posedge clk)if(!rst)begin active<=0;low_seen<=LOW_EXPECT_CH==0;end else begin
  if(!busy && valid && bin==1)active<=1;
  if(busy || done)active<=0;
  if(rv)begin
   $display("FINDER rate=%0d valid=%0d f0=%0d f1=%0d",rate,qual,frequencies[31:0],frequencies[63:32]);
   if(rate==8 && LOW_EXPECT_CH==1 && qual[0] && frequencies[31:0]>970 && frequencies[31:0]<1030)low_seen<=1;
   if(rate==8 && LOW_EXPECT_CH==2 && qual[1] && frequencies[63:32]>970 && frequencies[63:32]<1030)low_seen<=1;
   if(drive_high && rate==0 && qual[0] && frequencies[31:0]>990000 && frequencies[31:0]<1010000)high_seen<=1;
  end
 end
 initial begin
  if(!$value$plusargs("OUT=%s",filename))$fatal(1,"OUT missing");fd=$fopen(filename,"wb");
  for(j=0;j<4096;j=j+1)lut[j]=$rtoi(32767*$sin(6.283185307179586*j/4096));
  repeat(5)@(negedge clk);rst=1;
  if(AMP0==0)begin wait(low_seen);repeat(1000000)@(negedge clk);end
  else begin
   wait(completed>=2 && low_seen);wait(!busy);repeat(400)@(negedge clk);
   if(TRANSITION)begin
    drive_high=1;wait(high_seen);before_high=completed;
    wait(completed>=before_high+2);wait(!busy);repeat(400)@(negedge clk);
   end
  end
  $fclose(fd);$display("FFT_LOOPBACK_DONE frames=%0d low_seen=%0d",completed,low_seen);$finish;
 end
 initial forever begin
  @(negedge tx);#487.5;for(k=0;k<8;k=k+1)begin b[k]=tx;#325;end
  if(tx!==1)$fatal(1,"bad UART stop");$fwrite(fd,"%c",b);
 end
 initial begin repeat(8000000)@(negedge clk);$fatal(1,"timeout");end
endmodule
