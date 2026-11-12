`timescale 1ns/1ps
`default_nettype none
// Finder-only DC removal. Exactly the samples accepted by the Hann stage
// form the mean. The previous complete frame's rounded mean is held constant
// throughout the next frame, avoiding a mid-frame change of reference.
// The first frame is calibration only. This is not voltage zero calibration.
module fft_dc_center #(parameter integer LOG2_N=12)(
 input wire clk,rst_n,enable,frame_enable,accept,
 input wire signed [11:0] in0,in1,
 output wire signed [11:0] out0,out1,
 output reg frame_valid,frame_clipped
);
 localparam integer SW=LOG2_N+13;
 reg signed [SW-1:0] sum0,sum1;
 reg signed [11:0] mean0,mean1;
 reg [LOG2_N-1:0] count;
 reg mean_known;
 wire signed [SW-1:0] total0=sum0+{{(SW-12){in0[11]}},in0};
 wire signed [SW-1:0] total1=sum1+{{(SW-12){in1[11]}},in1};
 wire signed [12:0] diff0=$signed({in0[11],in0})-$signed({mean0[11],mean0});
 wire signed [12:0] diff1=$signed({in1[11],in1})-$signed({mean1[11],mean1});
 wire clipped=(diff0>13'sd2047)||(diff0< -13'sd2048)||
              (diff1>13'sd2047)||(diff1< -13'sd2048);
 assign out0=diff0>13'sd2047 ? 12'sd2047 : diff0< -13'sd2048 ? -12'sd2048 : diff0[11:0];
 assign out1=diff1>13'sd2047 ? 12'sd2047 : diff1< -13'sd2048 ? -12'sd2048 : diff1[11:0];
 function signed [11:0] rounded_mean;
  input signed [SW-1:0] value;
  reg signed [SW-1:0] quotient;
  reg increment;
  begin
   quotient=value>>>LOG2_N;
   increment=value[LOG2_N-1] && (!value[SW-1] || (|value[LOG2_N-2:0]));
   rounded_mean=quotient+$signed({1'b0,increment});
  end
 endfunction
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   sum0<=0;sum1<=0;mean0<=0;mean1<=0;count<=0;
   mean_known<=0;frame_valid<=0;frame_clipped<=0;
  end else if(!enable) begin
   sum0<=0;sum1<=0;mean0<=0;mean1<=0;count<=0;
   mean_known<=0;frame_valid<=0;frame_clipped<=0;
  end else if(accept) begin
   if(count==0) begin frame_valid<=mean_known;frame_clipped<=clipped;end
   else if(clipped) frame_clipped<=1;
   if(&count) begin
    mean0<=rounded_mean(total0);mean1<=rounded_mean(total1);
    mean_known<=1;sum0<=0;sum1<=0;count<=0;
   end else begin sum0<=total0;sum1<=total1;count<=count+1'b1;end
  end else if(!frame_enable) begin
   // Preserve completed-frame metadata until the engine publishes it.
   sum0<=0;sum1<=0;count<=0;
  end
 end
endmodule
`default_nettype wire
