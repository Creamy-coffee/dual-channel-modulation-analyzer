`timescale 1ns/1ps
// Production-depth FIFO: force overflow during the UART header, verify the
// retained samples and per-frame sticky loss flag through independent 8N1 RX.
module tb_iq_fifo_boundaries;
 reg clk=0;always #7.6923076923 clk=~clk;
 reg rst=0,valid=0;reg signed [15:0] i=0,q=0;
 wire tx,busy;wire [31:0] completed,overflow;
 iq_stream_tx #(.POINTS(256),.FIFO_DEPTH(512),.RUNTIME_RATE(1)) dut(
  .clk(clk),.rst_n(rst),.iq_valid(valid),.i_in(i),.q_in(q),.rate_sel(2'd2),.cfg_id(32'd33),
  .tx(tx),.busy(busy),.completed(completed),.overflow(overflow));
 reg [7:0] bytes[0:1067];integer at=0,frames=0,k,n,sample_no,expected;
 reg [7:0] value;reg [31:0] crc,received_crc;reg signed [15:0] ri,rq;
 function [31:0] crc_byte(input [31:0] c,input [7:0] d);
  reg [31:0] x;integer b;begin x=c^d;for(b=0;b<8;b=b+1)x=x[0]?(x>>1)^32'hedb88320:x>>1;crc_byte=x;end
 endfunction
 initial forever begin
  @(negedge tx);#750;
  for(k=0;k<8;k=k+1)begin value[k]=tx;#500;end
  if(tx!==1)$fatal(1,"stop bit");bytes[at]=value;at=at+1;
  if(at==1068)begin
   if({bytes[7],bytes[6],bytes[5],bytes[4],bytes[3],bytes[2],bytes[1],bytes[0]}!==64'h2131305149323356)$fatal(1,"IQ magic");
   crc=32'hffffffff;for(n=0;n<1064;n=n+1)crc=crc_byte(crc,bytes[n]);
   received_crc={bytes[1067],bytes[1066],bytes[1065],bytes[1064]};if((~crc)!==received_crc)$fatal(1,"CRC");
   if({bytes[15],bytes[14],bytes[13],bytes[12]}!==frames+1)$fatal(1,"frame ID");
   if(bytes[28]!==((frames==1)?8'd1:8'd0))$fatal(1,"loss flag frame=%0d flags=%h",frames,bytes[28]);
   for(n=0;n<256;n=n+1)begin
    ri={bytes[41+4*n],bytes[40+4*n]};rq={bytes[43+4*n],bytes[42+4*n]};
    expected=frames<2?(frames*256+n):(1000+n);
    if(ri!==expected[15:0] || rq!==(-expected))begin
     // Compare signed widened values to avoid accidental unsigned extension.
     if($signed(ri)!=expected || $signed(rq)!=-expected)$fatal(1,"sample order frame=%0d point=%0d I=%0d Q=%0d",frames,n,ri,rq);
    end
   end
   frames=frames+1;at=0;
  end
 end
 initial begin
  repeat(5)@(negedge clk);rst=1;
  for(sample_no=0;sample_no<1024;sample_no=sample_no+1)begin
   @(negedge clk);i=sample_no;q=-sample_no;valid=1;@(negedge clk);valid=0;
  end
  repeat(4)@(negedge clk);if(overflow!=512)$fatal(1,"expected 512 dropped samples got %0d",overflow);
  wait(frames==2);wait(!busy);repeat(20)@(negedge clk);
  for(sample_no=0;sample_no<256;sample_no=sample_no+1)begin
   repeat(2047)@(negedge clk);i=1000+sample_no;q=-(1000+sample_no);valid=1;@(negedge clk);valid=0;
  end
  wait(frames==3);wait(!busy);if(overflow!=512)$fatal(1,"unexpected later overflow");
  @(negedge clk);rst=0;repeat(3)@(negedge clk);
  if(overflow!=0 || completed!=0 || busy || tx!==1)$fatal(1,"reset left stale FIFO state");
  $display("IQ_FIFO_BOUNDARIES_PASS frames=3 dropped=512 sticky_flag_cleared=1 payload_order=OK CRC=OK");$finish;
 end
 initial begin #40000000;$fatal(1,"FIFO test timeout frames=%0d",frames);end
endmodule
