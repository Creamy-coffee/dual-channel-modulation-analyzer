`timescale 1ns/1ps
`default_nettype none
// 128-tap Q15 lowpass and /4 decimator. Circular sample memory avoids an
// 8192-flop snapshot and 128:1 sample multiplexers. Production samples arrive
// every 1024 clocks; one MAC pass takes TAPS+4 clocks. Coefficients are generated
// by scripts/design_ddc_filter.py. RAM contents are masked until initialized.
module ddc_fir_decimator #(
 parameter integer DATA_W=16,TAPS=128,DECIM=4,RUNTIME_RATE=0,
 parameter COEFF_FILE="ddc_fir_coeff.mem"
)(
 input wire clk,rst_n,in_valid,
 input wire signed[DATA_W-1:0] i_in,q_in,
 input wire [1:0] rate_sel,
 output reg out_valid,
 output reg signed[DATA_W-1:0] i_out,q_out
);
 localparam AW=$clog2(TAPS),ACC_W=40;

 localparam COEFF_N=RUNTIME_RATE ? 4*TAPS : TAPS;
 (* rom_style="block" *) reg signed[15:0] coeff[0:COEFF_N-1];
 initial begin
  $readmemh(COEFF_FILE,coeff,0,TAPS-1);
  if(RUNTIME_RATE)begin
   $readmemh("ddc_fir_narrow.mem",coeff,TAPS,2*TAPS-1);
   $readmemh("ddc_fir_wide.mem",coeff,2*TAPS,3*TAPS-1);
   $readmemh(COEFF_FILE,coeff,3*TAPS,4*TAPS-1);
  end
 end
 reg[AW-1:0] wrp,base,index,read_addr;
 reg[AW:0] filled,window_filled;
 reg[3:0] dcnt;
 reg active,rv,pv,finish_pending;
 wire[31:0] sample_r;
 reg signed[15:0] coeff_r;
 reg signed[31:0] prod_i,prod_q;
 (* use_dsp="yes" *) reg signed[ACC_W-1:0] acc_i,acc_q;
 wire[3:0] divisor=RUNTIME_RATE ? (rate_sel==0 ? 4'd8 : rate_sel==2 ? 4'd2 : 4'd4) : DECIM;
 wire[1:0] bank=rate_sel==0 ? 2'd1 : rate_sel==2 ? 2'd2 : 2'd0;
 wire trigger=in_valid && dcnt==divisor-1;
 wire[AW-1:0] addr=base-index;
 always @(posedge clk) coeff_r<=coeff[RUNTIME_RATE ? {bank,index} : {{2{1'b0}},index}];
`ifdef SIMULATION
 reg[31:0] history[0:TAPS-1];reg[31:0] ram_read;
 assign sample_r=ram_read;
 always @(posedge clk)begin
  if(in_valid && rst_n)history[wrp]<={q_in,i_in};
  ram_read<=history[addr];
 end
`else
 // Vivado 2018.3 otherwise absorbs read registers into the DSP and converts
 // this shallow 32-bit history to LUTRAM. Explicit SDP uses one RAMB18.
 RAMB18E1 #(.RAM_MODE("SDP"),.READ_WIDTH_A(36),.READ_WIDTH_B(0),
  .WRITE_WIDTH_A(0),.WRITE_WIDTH_B(36),.DOA_REG(0),.DOB_REG(0),
  .WRITE_MODE_A("READ_FIRST"),.WRITE_MODE_B("READ_FIRST")) u_history(
  .CLKARDCLK(clk),.CLKBWRCLK(clk),.ENARDEN(1'b1),.ENBWREN(in_valid && rst_n),
  .ADDRARDADDR({{(9-AW){1'b0}},addr,5'b0}),
  .ADDRBWRADDR({{(9-AW){1'b0}},wrp,5'b0}),
  .DIADI(i_in),.DIBDI(q_in),.DIPADIP(2'b0),.DIPBDIP(2'b0),
  .DOADO(sample_r[15:0]),.DOBDO(sample_r[31:16]),.DOPADOP(),.DOPBDOP(),
  .WEA(2'b0),.WEBWE(4'b1111),.REGCEAREGCE(1'b1),.REGCEB(1'b1),
  .RSTRAMARSTRAM(1'b0),.RSTRAMB(1'b0),.RSTREGARSTREG(1'b0),.RSTREGB(1'b0));
`endif
 function signed[15:0] clamp;
  input signed[ACC_W-1:0] x;reg signed[ACC_W-1:0] y;
  begin y=x>>>15;if(y>32767)clamp=16'sh7fff;
   else if(y< -32768)clamp=16'sh8000;else clamp=y[15:0];end
 endfunction
 always @(posedge clk)begin
  if(!rst_n)begin
   wrp<=0;base<=0;index<=0;read_addr<=0;filled<=0;window_filled<=0;dcnt<=0;
   active<=0;rv<=0;pv<=0;finish_pending<=0;prod_i<=0;prod_q<=0;
   acc_i<=0;acc_q<=0;out_valid<=0;i_out<=0;q_out<=0;
  end else begin
   out_valid<=0;
   if(in_valid)begin
    wrp<=wrp+1'b1;if(filled<TAPS)filled<=filled+1'b1;
    dcnt<=trigger?0:dcnt+1'b1;
   end
   rv<=active;
   // Save validity of each RAM read alongside that sample.
   if(active)begin
    read_addr<=index;
    if(index==TAPS-1)active<=0;else index<=index+1'b1;
   end
   pv<=rv;
   if(rv)begin
    prod_i<=(read_addr<window_filled)?$signed(sample_r[15:0])*coeff_r:32'sd0;
    prod_q<=(read_addr<window_filled)?$signed(sample_r[31:16])*coeff_r:32'sd0;
   end
   if(pv)begin
    acc_i<=acc_i+{{(ACC_W-32){prod_i[31]}},prod_i};
    acc_q<=acc_q+{{(ACC_W-32){prod_q[31]}},prod_q};
   end
   // Falling edge of pv occurs one clock after the final accumulation.
   finish_pending<=pv;
   if(finish_pending && !pv)begin
    i_out<=clamp(acc_i);q_out<=clamp(acc_q);
    // A zero-padded startup window is not a settled measurement.
    out_valid<=(window_filled==TAPS);
   end
   if(trigger)begin
    base<=wrp;index<=0;active<=1;rv<=0;pv<=0;finish_pending<=0;
    window_filled<=(filled<TAPS)?filled+1'b1:filled;
    acc_i<=0;acc_q<=0;
   end
  end
 end
 // synthesis translate_off
 initial if(DATA_W!=16 || TAPS<2 || TAPS>512 || (TAPS&(TAPS-1))!=0 || DECIM!=4)
  $fatal(1,"Invalid DDC FIR parameters");
 always @(posedge clk)if(rst_n && in_valid && (active||rv||pv||finish_pending))
  $fatal(1,"DDC FIR input spacing too short");
 // synthesis translate_on
endmodule
`default_nettype wire
