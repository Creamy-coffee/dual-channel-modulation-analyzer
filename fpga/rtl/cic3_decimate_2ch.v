`timescale 1ns/1ps
`default_nettype none
// Conventional order-3 CIC: three input-rate integrators, R decimation,
// three output-rate unit-delay combs. Modular arithmetic retains full gain.
// Q8 is retained between stages. Comb arithmetic is pipelined, not chained
// through a wide integrator/comb combinational path in one physical clock.
module cic3_decimate_2ch #(
 parameter integer DATA_W=20, LOG2_R=4
)(
 input wire clk,rst_n,clear,in_valid,
 input wire signed [DATA_W-1:0] in0,in1,
 output reg out_valid,
 output reg signed [DATA_W-1:0] out0,out1
);
 localparam GAIN_BITS=3*LOG2_R;
 localparam W=DATA_W+GAIN_BITS;
 localparam R=1<<LOG2_R;
 reg [LOG2_R-1:0] count;
 // These full-width modular integrators fit the DSP48 ALU. Keep their exact
 // width, reset, clock enable and latency; only technology mapping changes.
 // LUT carry chains consumed scarce logic while DSP capacity was available.
 (* use_dsp = "yes" *) reg signed [W-1:0] i0[0:2],i1[0:2];
 reg signed [W-1:0] z0[0:2],z1[0:2],c00,c01,c10,c11;
 reg v0,v1,v2;
 reg [GAIN_BITS+2:0] warm;
 wire take=in_valid && (&count);
 wire signed [W-1:0] ext0={{GAIN_BITS{in0[DATA_W-1]}},in0};
 wire signed [W-1:0] ext1={{GAIN_BITS{in1[DATA_W-1]}},in1};
 wire signed [W-1:0] last0=c10-z0[2],last1=c11-z1[2];
 function signed [DATA_W-1:0] norm;
  input signed [W-1:0] value;
  reg signed [W:0] wide,q;
  begin
   wide=$signed({value[W-1],value});
   if(wide>=0)q=(wide+(1<<(GAIN_BITS-1)))>>>GAIN_BITS;
   else q=-(((-wide)+(1<<(GAIN_BITS-1)))>>>GAIN_BITS);
   if(q>((1<<(DATA_W-1))-1))norm={1'b0,{(DATA_W-1){1'b1}}};
   else if(q< -(1<<(DATA_W-1)))norm={1'b1,{(DATA_W-1){1'b0}}};
   else norm=q[DATA_W-1:0];
  end
 endfunction
 integer j;
 always @(posedge clk)begin
  if(!rst_n || clear)begin
   count<=0;warm<=0;v0<=0;v1<=0;v2<=0;out_valid<=0;
   out0<=0;out1<=0;c00<=0;c01<=0;c10<=0;c11<=0;
   for(j=0;j<3;j=j+1)begin i0[j]<=0;i1[j]<=0;z0[j]<=0;z1[j]<=0;end
  end else begin
   v0<=take;v1<=v0;v2<=v1;out_valid<=0;
   if(in_valid)begin
    count<=count+1'b1;
    i0[0]<=i0[0]+ext0;i1[0]<=i1[0]+ext1;
    i0[1]<=i0[1]+i0[0];i1[1]<=i1[1]+i1[0];
    i0[2]<=i0[2]+i0[1];i1[2]<=i1[2]+i1[1];
    if(warm<4*R)warm<=warm+1'b1;
   end
   if(take)begin c00<=i0[2]-z0[0];c01<=i1[2]-z1[0];z0[0]<=i0[2];z1[0]<=i1[2];end
   if(v0)begin c10<=c00-z0[1];c11<=c01-z1[1];z0[1]<=c00;z1[1]<=c01;end
   if(v1)begin
    z0[2]<=c10;z1[2]<=c11;
    out0<=norm(last0);out1<=norm(last1);
    out_valid<=(warm>=4*R);
   end
  end
 end
 // synthesis translate_off
 initial if(LOG2_R<2 || LOG2_R>4 || DATA_W<12 || DATA_W>24)
  $fatal(1,"Unsupported CIC parameters");
 // synthesis translate_on
endmodule
`default_nettype wire
