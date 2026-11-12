`timescale 1ns/1ps
`default_nettype none
// V32HRM1! v1, 216 bytes. A hold handshake borrows the statistics module's
// completed result registers instead of duplicating 600 bits of power storage.
// Connect result_hold to harmonic_stream_stats.result_hold. All result fields
// except age must remain stable while hold=1, including on config changes.
module harmonic_result_tx #(
 parameter integer CLK_FREQ=65000000,BAUD=2000000,STOP_BITS=1,NFFT=4096,HOST_THD=0
)(
 input wire clk,rst_n,result_valid,send_start,
 input wire [31:0] result_id,result_cfg,reference_fcw,age_ticks,flags,thd_q24,dropped_frames,
 input wire channel,thd_valid,
 input wire [3:0] rate_log2,
 input wire [1:0] window_id,
 input wire [4:0] valid_mask,noise_mask,
 input wire [39:0] band_power,noise_power,signal_power,
 input wire [11:0] peak_bin,
 output wire [2:0] power_read_index,
 output wire result_hold,
 output reg ready,busy,
 output wire tx,
 output reg [31:0] completed
);
 reg [7:0] pos;
 reg [31:0] crc,age_hold;
 wire tx_ready;
 reg [31:0] header;
 reg [7:0] byte_data,local_offset;
 reg [2:0] harmonic_index;
 reg [39:0] power_word;
 reg [11:0] peak_word;
 assign result_hold=result_valid || ready || busy;
 function [31:0] crc_byte;
  input [31:0] c;input [7:0] d;integer i;reg [31:0] x;
  begin x=c^{24'd0,d};for(i=0;i<8;i=i+1)x=x[0]?(x>>1)^32'hedb88320:x>>1;crc_byte=x;end
 endfunction
 assign power_read_index=pos<192?harmonic_index:local_offset[4:2];
 always @*begin
  case(pos[6:2])
   0:header=32'h48323356; // V32H
   1:header=32'h21314d52; // RM1!
   2:header=HOST_THD?2:1;3:header=216;4:header=result_id;5:header=result_cfg;
   6:header=CLK_FREQ;7:header=NFFT;8:header=reference_fcw;
   9:header=32'd1<<rate_log2;10:header=age_hold;
   11:header=flags | (thd_valid?32'h80000000:0) | (rate_log2!=0?32'h00000100:0) | (HOST_THD?32'h00000200:0);
   12:header={31'd0,channel};13:header={30'd0,window_id};
   14:header={27'd0,valid_mask};15:header={27'd0,noise_mask};
   16:header=thd_q24;17:header=dropped_frames;default:header=0;
  endcase
  local_offset=pos<112?pos-72:pos<152?pos-112:pos<192?pos-152:pos-192;
  harmonic_index=local_offset[5:3];
  power_word=pos<112?band_power:pos<152?noise_power:signal_power;
  peak_word=peak_bin;
  if(pos<72)byte_data=header>>(8*pos[1:0]);
  else if(pos<192)byte_data=local_offset[2:0]<5?(power_word>>(8*local_offset[2:0])):8'd0;
  else if(pos<212)byte_data=({20'd0,peak_word}>>(8*local_offset[1:0]));
  else byte_data=(~crc)>>(8*pos[1:0]);
 end
 uart_tx_v32 #(.CLK_FREQ(CLK_FREQ),.BAUD(BAUD),.STOP_BITS(STOP_BITS)) uart(
  .clk(clk),.rst_n(rst_n),.tx_data(byte_data),.tx_valid(busy),.tx_ready(tx_ready),.tx(tx));
 always @(posedge clk)begin
  if(!rst_n)begin ready<=0;busy<=0;pos<=0;crc<=32'hffffffff;age_hold<=0;completed<=0;end
  else if(!busy)begin
   if(result_valid)ready<=1;
   if(send_start && ready)begin ready<=0;busy<=1;pos<=0;crc<=32'hffffffff;age_hold<=age_ticks;end
  end else if(tx_ready)begin
   if(pos<212)crc<=crc_byte(crc,byte_data);
   if(pos==215)begin busy<=0;completed<=completed+1'b1;end
   else pos<=pos+1'b1;
  end
 end
endmodule
`default_nettype wire
