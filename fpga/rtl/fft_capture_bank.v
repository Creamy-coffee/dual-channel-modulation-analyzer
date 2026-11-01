`timescale 1ns/1ps
`default_nettype none
// Single frozen frame per rate. A full bank remains immutable while the shared
// FFT reads it; release starts a new capture. Other rates and the ADC never wait.
// Same-frame DC mean avoids applying a mean measured at a different rate/time.
module fft_capture_bank #(parameter integer AW=12)(
 input wire clk,rst_n,clear,in_valid,release_frame,
 input wire signed [11:0] in0,in1,
 input wire [AW-1:0] rd_addr,
 output reg signed [11:0] rd0,rd1,
 output reg signed [11:0] mean0,mean1,
 output reg ready,
 output reg [31:0] capture_end
);
 (* ram_style="block" *) reg [23:0] mem[0:(1<<AW)-1];
 reg [AW-1:0] wr;
 reg signed [AW+12:0] sum0,sum1;
 reg [31:0] ticks;
 wire signed [AW+12:0] next0=sum0+{{(AW+1){in0[11]}},in0};
 wire signed [AW+12:0] next1=sum1+{{(AW+1){in1[11]}},in1};
 function signed [11:0] mean;
  input signed [AW+12:0] value;
  reg signed [AW+13:0] wide,q;
  begin
   wide=$signed({value[AW+12],value});
   if(wide>=0)q=(wide+(1<<(AW-1)))>>>AW;
   else q=-(((-wide)+(1<<(AW-1)))>>>AW);
   mean=q[11:0];
  end
 endfunction
 always @(posedge clk)begin
  {rd1,rd0}<=mem[rd_addr];
  if(rst_n && !clear && in_valid && !ready)mem[wr]<={in1,in0};
 end
 always @(posedge clk)begin
  if(!rst_n || clear)begin
   wr<=0;sum0<=0;sum1<=0;ready<=0;mean0<=0;mean1<=0;capture_end<=0;ticks<=0;
  end else begin
   ticks<=ticks+1'b1;
   if(release_frame)begin wr<=0;sum0<=0;sum1<=0;ready<=0;end
   else if(in_valid && !ready)begin
    if(&wr)begin
     mean0<=mean(next0);mean1<=mean(next1);capture_end<=ticks;ready<=1;
    end else begin wr<=wr+1'b1;sum0<=next0;sum1<=next1;end
   end
  end
 end
endmodule
`default_nettype wire
