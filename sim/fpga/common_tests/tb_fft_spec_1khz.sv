`timescale 1ns/1ps
module tb_fft_spec_1khz;
 reg clk=0;always #5 clk=~clk;
 reg rst=0;reg [31:0] phase=0;
 reg signed [15:0] lut[0:4095];reg signed [11:0] x=0;
 wire valid;wire [11:0] bin;wire [32:0] power;
 wire tx,busy;wire [31:0] completed;
 reg active=0,finished_scan=0;
 integer fd,j,k;reg [7:0] b;reg [1023:0] filename;
 multirate_fft_frontend #(.MULTIRATE_ENABLE(0),.SPECTRUM_AUTO(0)) fft(
  .clk(clk),.rst_n(rst),.enable(1'b1),.adc_valid(1'b1),.adc_ch0(x),.adc_ch1(12'sd0),
  .spectrum_valid(valid),.spectrum_bin(bin),.spectrum_power(power));
 wire done=valid && active && bin==2048 && !finished_scan;
 v32spec_tx spec(.clk(clk),.rst_n(rst),
  .wr_en(valid && (active||bin==1) && !finished_scan),
  .wr_bin(bin==2048?12'd0:bin),.wr_mag(bin==2048?33'd0:power),
  .scan_done(done),.send_start(1'b0),.snapshot_ready(),
  .wave_frame_id(32'd1),.window_sel(2'd0),.tx(tx),.busy(busy),.completed(completed),.result_id());
 always @(negedge clk)begin phase<=phase+32'd66076;x<=(400*$signed(lut[phase[31:20]]))/32768;end
 always @(posedge clk)if(!rst)begin active<=0;finished_scan<=0;end else begin
  if(valid && bin==1 && !finished_scan)active<=1;
  if(done)begin active<=0;finished_scan<=1;end
 end
 initial begin
  if(!$value$plusargs("OUT=%s",filename))$fatal(1,"OUT missing");
  fd=$fopen(filename,"wb");
  for(j=0;j<4096;j=j+1)lut[j]=$rtoi(32767*$sin(6.283185307179586*j/4096));
  repeat(5)@(negedge clk);rst=1;
  wait(completed==1);repeat(400)@(negedge clk);$fclose(fd);
  $display("FFT_SPEC_1KHZ_SERIALIZED_PASS");$finish;
 end
 initial forever begin
  @(negedge tx);#487.5;
  for(k=0;k<8;k=k+1)begin b[k]=tx;#325;end
  if(tx!==1)$fatal(1,"bad UART stop");$fwrite(fd,"%c",b);
 end
 initial begin repeat(4000000)@(negedge clk);$fatal(1,"timeout");end
endmodule
