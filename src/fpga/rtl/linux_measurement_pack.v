`timescale 1ns/1ps
`default_nettype none
// V32MEAS! v1, 124-byte body. Serializer adds CRC32 and stamps the associated
// RAW frame id/start tick. This is a diagnostic snapshot, NOT the RAW interval.
module linux_measurement_pack #(
 parameter integer FS_HZ=65000000, BUILD=20261020, META_PROTO=12
)(
 input wire [31:0] result_id,result_age,active_fcw,ratio_q16,
 input wire [23:0] amp0_q12,amp1_q12,
 input wire signed [15:0] phase0,phase1,pdiff,
 input wire [1:0] qual,det,cal_clip,adc_otr,fft_valid,
 input wire usable,present,ratio_valid,afc_ready,same_source,settling,cal_valid,
 input wire [7:0] flags,afc_state,afc_reason,guard0,guard1,
 input wire [7:0] bw_gear,decim_log2,integration_log2,
 input wire [31:0] gap,drop,fft_hz0,fft_hz1,fft_age0,fft_age1,
 input wire [3:0] fft_rate0,fft_rate1,
 input wire signed [31:0] residual0,residual1,
 output reg [991:0] body
);
 always @* begin
  body=0;
  body[0*8+:64]=64'h215341454d323356; // bytes V32MEAS!
  body[8*8+:32]=32'd1;
  body[12*8+:32]=32'd128;
  // bytes 16..19: associated RAW frame; 24..31: snapshot tick, serializer.
  body[20*8+:32]=result_id;
  body[32*8+:32]=FS_HZ;
  body[36*8+:32]=result_age;
  body[40*8+:32]=active_fcw;
  body[44*8+:32]={8'd0,amp0_q12};
  body[48*8+:32]={8'd0,amp1_q12};
  body[52*8+:32]=ratio_q16;
  body[56*8+:16]=phase0;body[58*8+:16]=phase1;
  body[60*8+:16]=pdiff;body[62*8+:8]={6'd0,qual};body[63*8+:8]={6'd0,det};
  body[64*8+:32]={20'd0,adc_otr,cal_clip,1'b0,cal_valid,settling,same_source,afc_ready,ratio_valid,present,usable};
  body[68*8+:8]=flags;body[69*8+:8]=afc_state;body[70*8+:8]=afc_reason;body[71*8+:8]=bw_gear;
  body[72*8+:8]=guard0;body[73*8+:8]=guard1;
  body[74*8+:8]=decim_log2;body[75*8+:8]=integration_log2;
  body[76*8+:32]=gap;body[80*8+:32]=drop;
  body[84*8+:32]=fft_hz0;body[88*8+:32]=fft_hz1;
  body[92*8+:8]={6'd0,fft_valid};
  body[93*8+:8]={4'd0,fft_rate0};body[94*8+:8]={4'd0,fft_rate1};
  body[96*8+:32]=fft_age0;body[100*8+:32]=fft_age1;
  body[104*8+:32]=residual0;body[108*8+:32]=residual1;
  body[112*8+:32]=BUILD;body[116*8+:32]=META_PROTO;
 end
endmodule
`default_nettype wire
