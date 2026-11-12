`timescale 1ns/1ps
// Scoreboard is independent of DUT conversion and register names.
module tb_adc_parallel;
 reg clk=0; always #7.692307692 clk=~clk;
 reg rst_n=0,locked=0;
 reg [11:0] a=0,b=0;
 reg [3:0] otr=0;
 wire [47:0] offset_samples,twos_samples;
 wire [3:0] offset_otr,twos_otr;
 wire ov,tv,oready,tready;
 ad9238_capture #(.CHANNELS(2),.PARALLEL_PORTS(1),.USE_EXT_CLK(1),
   .STARTUP_CYCLES(4),.PIPELINE_FLUSH(16),.OFFSET_BINARY(1)) offset_dut(
   .clk(clk),.rst_n(rst_n),.adc_clk_ext(clk),.mmcm_locked(locked),
   .adc_d0(a),.adc_d1(b),.adc_d2(12'hfff),.adc_d3(12'hfff),.adc_otr(otr),
   .adc_clk(),.samples(offset_samples),.sample_otr(offset_otr),.sample_valid(ov),.ready(oready));
 ad9238_capture #(.CHANNELS(2),.PARALLEL_PORTS(1),.USE_EXT_CLK(1),
   .STARTUP_CYCLES(4),.PIPELINE_FLUSH(16),.OFFSET_BINARY(0)) twos_dut(
   .clk(clk),.rst_n(rst_n),.adc_clk_ext(clk),.mmcm_locked(locked),
   .adc_d0(a),.adc_d1(b),.adc_d2(12'hfff),.adc_d3(12'hfff),.adc_otr(otr),
   .adc_clk(),.samples(twos_samples),.sample_otr(twos_otr),.sample_valid(tv),.ready(tready));
 reg [11:0] prev_a=0,prev_b=0;
 reg [3:0] prev_otr=0;
 integer checked=0,age=0,epochs=0,i;
 integer ea,eb;
 always @(posedge clk) begin
   if(!rst_n || !locked) age=0; else age=age+1;
   #0.1;
   if(ov !== tv || oready !== tready) $fatal(1,"format modes disagree on validity");
   if(!rst_n || !locked) begin
     if(ov !== 0 || oready !== 0) $fatal(1,"data published during reset/unlock");
   end else if(ov) begin
     if(age<21) $fatal(1,"startup/flush was bypassed age=%0d",age);
     ea=$unsigned(prev_a)-2048; eb=$unsigned(prev_b)-2048;
     if($signed(offset_samples[11:0]) !== ea || $signed(offset_samples[23:12]) !== eb)
       $fatal(1,"offset conversion/channel/latency error a=%h b=%h got=%h",prev_a,prev_b,offset_samples);
     if(twos_samples[23:0] !== {prev_b,prev_a}) $fatal(1,"two's complement mode changed input bits");
     if(offset_samples[47:24] !== 0 || twos_samples[47:24] !== 0) $fatal(1,"inactive lanes not zero");
     if(offset_otr !== (prev_otr & 4'h3) || twos_otr !== (prev_otr & 4'h3)) $fatal(1,"OTR alignment/mask");
     if(!oready) $fatal(1,"valid without ready");
     checked=checked+1;
   end
   prev_a=a;prev_b=b;prev_otr=otr;
 end
 task sweep;
   begin
     // All 4096 codes on each bus, independent permutations, every OTR combination.
     for(i=0;i<4130;i=i+1) begin
       @(negedge clk); a=i%4096;b=(i*17+2048)%4096;otr=i%16;
     end
   end
 endtask
 initial begin
   repeat(3) @(negedge clk);rst_n=1;
   repeat(5) @(negedge clk);locked=1;sweep;
   @(negedge clk);locked=0;repeat(3) @(negedge clk);locked=1;sweep;
   @(negedge clk);rst_n=0;repeat(3) @(negedge clk);rst_n=1;sweep;
   if(checked<12288) $fatal(1,"insufficient exhaustive coverage %0d",checked);
   $display("PASS ADC parallel: all 4096 codes, both formats, same-edge channels, OTR, startup, unlock and reset; checked=%0d",checked);
   $finish;
 end
 initial begin #1000000;$fatal(1,"ADC timeout");end
endmodule
