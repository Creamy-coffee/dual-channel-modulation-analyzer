`timescale 1ns/1ps
module tb_linux_packets;
 reg clk=0;always #7.6923076923 clk=~clk;
 reg rst=0,fault=0;reg [63:0] tick=0;
 wire tx;wire [31:0] completed,aborted;wire [991:0] body;
 wire [1:0] q=(completed==0)?2'd3:2'd0;
 linux_measurement_pack pack(
 .result_id(tick[31:0]),.result_age(32'd650),.active_fcw(32'd1982292598),.ratio_q16(32'd32768),
 .amp0_q12(tick[23:0]),.amp1_q12(24'd1228800),.phase0(-16'sd1000),.phase1(16'sd3000),.pdiff(16'sd4000),
 .qual(q),.det(q),.cal_clip(2'd0),.adc_otr(2'd0),.fft_valid(2'd3),
 .usable(q==3),.present(1'b1),.ratio_valid(q==3),.afc_ready(q==3),.same_source(1'b1),.settling(1'b0),.cal_valid(1'b0),
 .flags(8'd0),.afc_state(q==3?8'd3:8'd0),.afc_reason(8'd0),.guard0(q==3?8'd0:8'd4),.guard1(q==3?8'd0:8'd4),
 .bw_gear(8'd1),.decim_log2(8'd6),.integration_log2(8'd12),.gap(32'd7),.drop(32'd8),
 .fft_hz0(32'd30000000),.fft_hz1(32'd30000001),.fft_age0(32'd12345),.fft_age1(32'd23456),
 .fft_rate0(4'd0),.fft_rate1(4'd4),.residual0(-32'sd123),.residual1(32'sd456),.body(body));
 v32cap3_tx #(.POINTS(1024),.MEAS_ENABLE(1),.FRAME_PERIOD(0)) dut(
 .clk(clk),.rst_n(rst),.sample_valid(1'b1),.decim_log2(4'd0),.fault(fault),.ch0(tick[11:0]),.ch1(~tick[11:0]),.meas_body(body),
 .tx(tx),.busy(),.completed(completed),.aborted(aborted),.skipped());
 always @(posedge clk) if(!rst)tick<=0;else tick<=tick+1;
 initial begin repeat(8)@(negedge clk);rst=1;repeat(100)@(negedge clk);fault=1;@(negedge clk);fault=0;end
 integer fd,count=0,b;reg [7:0] value;reg armed=1;
 real bit_ns=1000000000.0/921600.0;
 initial begin
 fd=$fopen("../tests/linux_packets_wire.bin","wb");
 forever begin
 @(negedge tx);
 if(armed)begin
 armed=0;#(1.5*bit_ns);
 for(b=0;b<8;b=b+1)begin value[b]=tx;#(bit_ns);end
 if(tx!==1)$fatal(1,"Linux packet UART stop bit error");
 $fwrite(fd,"%c",value);count=count+1;armed=1;
 if(count==2*(4140+128))begin
 $fclose(fd);if(aborted!=1)$fatal(1,"Missing partial capture abort");
 $display("TB_PASS LINUX_PACKETS bytes=%0d",count);$finish;end
 end
 end
 end
 initial begin #250000000;$fatal(1,"Linux packets timeout %0d",count);end
endmodule
