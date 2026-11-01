`timescale 1ns/1ps
module tb_ddc_wire;
parameter KIND=1; // 1=AM, 2=FM
reg clk=0;always #7.6923076923 clk=~clk;
reg rst=0;reg signed[15:0] adc=0;
wire[47:0] raw;wire[3:0] otr;wire raw_valid;
ad9238_capture #(.STARTUP_CYCLES(16),.PARALLEL_PORTS(1)) capture(
 .clk(clk),.rst_n(rst),.adc_clk_ext(clk),.mmcm_locked(rst),
 .adc_d0(adc[11:0]^12'h800),.adc_d1(12'h800),.adc_d2(12'd0),.adc_d3(12'd0),
 .adc_otr(4'd0),.adc_clk(),.samples(raw),.sample_otr(otr),.sample_valid(raw_valid),.ready());
wire signed[15:0] captured={{4{raw[11]}},raw[11:0]};wire mv,cv,fv,tx;wire signed[15:0] mi,mq,ci,cq,fi,fq;wire[31:0] completed,overflow;
ddc_nco_mixer mix(.clk(clk),.rst_n(rst),.in_valid(raw_valid),.in(captured),.fcw(32'd66076420),.out_valid(mv),.i_out(mi),.q_out(mq));
ddc_cic_decimator cic(.clk(clk),.rst_n(rst),.clear(1'b0),.in_valid(mv),.i_in(mi),.q_in(mq),.out_valid(cv),.i_out(ci),.q_out(cq));
ddc_fir_decimator fir(.clk(clk),.rst_n(rst),.in_valid(cv),.i_in(ci),.q_in(cq),.out_valid(fv),.i_out(fi),.q_out(fq));
iq_stream_tx dut(.clk(clk),.rst_n(rst),.iq_valid(fv),.i_in(fi),.q_in(fq),.cfg_id(32'd1),.tx(tx),.busy(),.completed(completed),.overflow(overflow));
integer n=0,k,fd,ref_fd,count=0;reg[7:0] b;real t,phase;
always @(negedge clk)if(rst)begin
 t=n/65000000.0; phase=6.283185307179586*1000000*t;
 if(KIND==1)adc=$rtoi(1000*(1+0.6*$sin(6.283185307179586*500*t))*$cos(phase));
 else adc=$rtoi(1000*$cos(phase+0.8*$sin(6.283185307179586*500*t)));
 n=n+1;
 if(n%1000000==0)$display("DDC_PROGRESS kind=%0d cycles=%0d bytes=%0d",KIND,n,count);
end
always @(negedge clk)if(fv)$fwrite(ref_fd,"%0d %0d\n",fi,fq);
initial begin #100;rst=1;wait(completed==8);#6000;
if(overflow!=0)$fatal(1,"DDC overflow");$fclose(fd);$fclose(ref_fd);$display("DDC_WIRE_PASS kind=%0d bytes=%0d",KIND,count);$finish;end
initial begin fd=$fopen(KIND==1?"../tests/iq_am_wire.bin":"../tests/iq_fm_wire.bin","wb");
ref_fd=$fopen(KIND==1?"../tests/iq_am_ref.txt":"../tests/iq_fm_ref.txt","w");
forever begin @(negedge tx);#750;for(k=0;k<8;k=k+1)begin b[k]=tx;#500;end
if(tx!==1)$fatal(1,"DDC UART stop");$fwrite(fd,"%c",b);count=count+1;end end
initial begin #180000000;$fatal(1,"DDC wire timeout");end
endmodule
