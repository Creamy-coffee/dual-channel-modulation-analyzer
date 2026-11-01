`timescale 1ns/1ps
`default_nettype none
// A5 k selects R=2^k (k=0..15); A5 FF restores automatic timebase.
// DDC demod commands (extend the same A5 prefix):
//   A5 E0 mode: 0=off,1=AM,2=FM,3=2FSK,4=QPSK,5=OOK,6=BPSK,7=PM
//   A5 E1 b0 b1: stage FCW[31:16], little-endian (no commit)
//   A5 E2 b0 b1: commit full FCW with staged upper half, little-endian
//   A5 E3 <ch>    DDC input channel: 0=CH0, 1=CH1
// Only an effective demod configuration change bumps cfg_id so the Linux side can detect that
// parameters changed and re-synchronize.
// Bad stop bits/operands do not change configuration; partial commands expire.
module ab5_uart_commands #(
 parameter integer CLK_FREQ=65000000,BAUD=2000000,DUAL_RX=0
)(
 input wire clk,rst_n,rx,
 input tri1 rx_copy,
 output reg [3:0] decim_log2,
 output reg override_auto,
 output reg demod_en,
 output reg [3:0] demod_mode,
 output reg [31:0] ddc_fcw,
 output reg iq_ch,
 output reg [31:0] cfg_id,
 output reg [1:0] wave_focus,
 output reg [1:0] iq_rate,    // A5 E4 rate: 0=narrow,1=standard,2=wide
 // Independent harmonic reference. F0=channel, F1=FCW high, F2=FCW low,
 // F3=enable. E6..E9 remain unknown (older experimental meanings are not reused).
 // These never change DDC mode/channel/rate/configuration.
 output reg harmonic_en,harmonic_ch,
 output reg [31:0] harmonic_fcw,harmonic_cfg
);
 (* ASYNC_REG="TRUE" *) reg rx_meta,rx_sync;
 (* ASYNC_REG="TRUE" *) reg rx_copy_meta,rx_copy_sync;
 wire rx_sample,rx_release,rx_new_source;
 reg [1:0] state;
 function integer gcd;
  input integer a,b;integer x,y,t;
  begin x=a;y=b;while(y!=0)begin t=x%y;x=y;y=t;end gcd=x;end
 endfunction
 // Scale both phase units by their GCD; doubled units represent half a bit
 // exactly even when CLK_FREQ/GCD is odd. 65M/2M needs 7 rather than 32 bits.
 localparam integer GCD=gcd(CLK_FREQ,BAUD);
 localparam integer PH_CLK=2*(CLK_FREQ/GCD),PH_BAUD=2*(BAUD/GCD);
 reg [$clog2(PH_CLK+PH_BAUD+1)-1:0] phase;
 reg [2:0] bit_index;
 reg [7:0] data;
 reg pending;
 reg operand_high, high_ready;
 reg [7:0] operand_low;
 reg [15:0] staged_high;
 reg [3:0] sub;
 reg harmonic_high_ready;
 reg [15:0] harmonic_staged_high;
 localparam integer TIMEOUT=CLK_FREQ/BAUD*200;
 reg [$clog2(TIMEOUT+1)-1:0] timeout_count;
 // One byte decoder/parser, two synchronized physical inputs. First start
 // edge owns the decoder until both inputs are quiet for > parser timeout.
 // AA5 wins simultaneous starts; the other source is ignored, never ANDed.
 generate if(DUAL_RX)begin : g_dual_rx
  localparam integer QUIET=CLK_FREQ/BAUD*220;
  reg owned,owner,last0,last1;
  reg [$clog2(QUIET+1)-1:0] quiet_count;
  wire start0=last0 && !rx_sync;
  wire start1=last1 && !rx_copy_sync;
  wire choose_copy=!start0;
  assign rx_sample=owned ? (owner?rx_copy_sync:rx_sync) :
                          start0?rx_sync:start1?rx_copy_sync:1'b1;
  assign rx_release=owned && rx_sync && rx_copy_sync && quiet_count==QUIET-1;
  assign rx_new_source=!owned && (start0||start1) && (choose_copy!=owner);
  always @(posedge clk)begin
   if(!rst_n)begin owned<=0;owner<=0;last0<=1;last1<=1;quiet_count<=0;end
   else begin
    last0<=rx_sync;last1<=rx_copy_sync;
    if(!owned)begin
     quiet_count<=0;
     if(start0||start1)begin owned<=1;owner<=choose_copy;end
    end else if(rx_release)begin owned<=0;quiet_count<=0;end
    else if(rx_sync && rx_copy_sync)quiet_count<=quiet_count+1'b1;
    else quiet_count<=0;
   end
  end
 end else begin : g_single_rx
  assign rx_sample=rx_sync;
  assign rx_release=1'b0;
  assign rx_new_source=1'b0;
 end endgenerate
 always @(posedge clk)begin
  if(!rst_n)begin rx_meta<=1;rx_sync<=1;rx_copy_meta<=1;rx_copy_sync<=1;end
  else begin rx_meta<=rx;rx_sync<=rx_meta;rx_copy_meta<=rx_copy;rx_copy_sync<=rx_copy_meta;end
 end
 always @(posedge clk) begin
  if(!rst_n)begin
   state<=0;phase<=0;bit_index<=0;data<=0;pending<=0;sub<=0;
   timeout_count<=0;decim_log2<=0;override_auto<=0;
   operand_high<=0;operand_low<=0;high_ready<=0;staged_high<=0;
   demod_en<=0;demod_mode<=0;ddc_fcw<=32'd66076420; // default 1.000 MHz @65 MS/s
   iq_ch<=0;cfg_id<=0;iq_rate<=1;wave_focus<=2;
   harmonic_en<=0;harmonic_ch<=0;harmonic_fcw<=32'd66076;harmonic_cfg<=0;
   harmonic_high_ready<=0;harmonic_staged_high<=0;
  end else begin
   if(pending)begin
    if(timeout_count==0)begin pending<=0;sub<=0;operand_high<=0;high_ready<=0;harmonic_high_ready<=0;end
    else timeout_count<=timeout_count-1'b1;
   end
   case(state)
    0:if(!rx_sample)begin state<=1;phase<=PH_CLK/2;end
    1:if(phase+PH_BAUD>=PH_CLK)begin
     phase<=phase+PH_BAUD-PH_CLK;
     if(!rx_sample)begin state<=2;bit_index<=0;end else state<=0;
    end else phase<=phase+PH_BAUD;
    2:if(phase+PH_BAUD>=PH_CLK)begin
     phase<=phase+PH_BAUD-PH_CLK;data[bit_index]<=rx_sample;
     if(bit_index==7)state<=3;else bit_index<=bit_index+1'b1;
    end else phase<=phase+PH_BAUD;
    3:if(phase+PH_BAUD>=PH_CLK)begin
     state<=0;
     if(rx_sample)begin
      if(pending && timeout_count!=0)begin
       timeout_count<=TIMEOUT;
       if(sub==0)begin
        if(data<=15)begin decim_log2<=data[3:0];override_auto<=1;pending<=0;end
        else if(data==8'hff)begin override_auto<=0;pending<=0;end
        else if(data>=8'he0 && data<=8'he5)begin
         sub<=data-8'hdf;operand_high<=0;
        end else if(data>=8'hf0 && data<=8'hf3)begin
         sub<=data-8'he9;operand_high<=0;
        end else begin pending<=0;end
       end else if(sub==2 || sub==3 || sub==8 || sub==9)begin
        if(!operand_high)begin operand_low<=data;operand_high<=1;end
        else begin
         if(sub==2)begin staged_high<={data,operand_low};high_ready<=1;end
         else if(sub==3 && high_ready)begin
          if(ddc_fcw!={staged_high,data,operand_low})begin
           ddc_fcw<={staged_high,data,operand_low};cfg_id<=cfg_id+1;
          end
          high_ready<=0;
         end else if(sub==8)begin
          harmonic_staged_high<={data,operand_low};harmonic_high_ready<=1;
         end else if(sub==9 && harmonic_high_ready)begin
          if(harmonic_fcw!={harmonic_staged_high,data,operand_low})begin
           harmonic_fcw<={harmonic_staged_high,data,operand_low};harmonic_cfg<=harmonic_cfg+1;
          end
          harmonic_high_ready<=0;
         end
         pending<=0;sub<=0;operand_high<=0;
        end
       end else begin
        if(sub==1 && data<=7 && (demod_mode!=data[3:0] || demod_en!=(data!=0)))begin
         demod_mode<=data[3:0];demod_en<=(data!=0);cfg_id<=cfg_id+1;
        end else if(sub==4 && data<=1 && iq_ch!=data[0])begin iq_ch<=data[0];cfg_id<=cfg_id+1;end
        else if(sub==5 && data<=2 && iq_rate!=data[1:0])begin iq_rate<=data[1:0];cfg_id<=cfg_id+1;end
        else if(sub==6 && data<=2)begin wave_focus<=data[1:0];end
        else if(sub==7 && data<=1 && harmonic_ch!=data[0])begin harmonic_ch<=data[0];harmonic_cfg<=harmonic_cfg+1;end
        else if(sub==10 && data<=1 && harmonic_en!=data[0])begin harmonic_en<=data[0];harmonic_cfg<=harmonic_cfg+1;end
        pending<=0;sub<=0;
       end
      end else if(data==8'ha5)begin pending<=1;sub<=0;operand_high<=0;timeout_count<=TIMEOUT;end
     end else begin pending<=0;sub<=0;operand_high<=0;high_ready<=0;harmonic_high_ready<=0;end
    end else phase<=phase+PH_BAUD;
   endcase
   if(rx_release)begin state<=0;pending<=0;sub<=0;operand_high<=0;timeout_count<=0;end
   // Never combine one port's staged FCW high half with the other port's low half.
   // Preserve staging across quiet gaps when the same port resumes.
   if(rx_new_source)begin pending<=0;sub<=0;operand_high<=0;high_ready<=0;harmonic_high_ready<=0;end
  end
 end
endmodule
`default_nettype wire
