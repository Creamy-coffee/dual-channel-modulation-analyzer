`timescale 1ns/1ps
`default_nettype none
// Debounced controls; no asynchronous key/switch drives acquisition logic.
//
// TIME BASE
//   KEY2 cycles the manual time base through 0,4,5,...,10 (R=1,16,32,...,1024).
//   sw[2] up selects the AUTOMATIC time base (auto_timebase.v); the keys then
//   have no effect and the switch is the override back to manual.
module basic_controls #(parameter integer DEBOUNCE_CLKS=650000,KEY_TIMEBASE=0,MAX_TIMEBASE=10)(
 input wire clk,rst_n,input wire [3:0] key_n,input wire [2:0] sw,
 output reg freeze,output reg trig_src,output reg falling,
 output wire [3:0] timebase_log2,output wire tb_auto,
 // Debounced switch 0.  With KEY_TIMEBASE=1 the time base comes from KEY2,
 // so sw[0]/sw[1] carry no time-base meaning in the daq_top build; sw[0] is
 // the ET (equivalent-time) display mode selector there.
 output wire sw0
);
 (* ASYNC_REG="TRUE" *) reg [3:0] key_meta,key_sync;
 (* ASYNC_REG="TRUE" *) reg [2:0] switch_meta,switch_sync;
 reg [3:0] stable_keys;reg [2:0] stable_sw;
 reg [19:0] key_count[0:3];reg [19:0] sw_count;
 reg [3:0] timebase_index;
 // index 0 -> R=1 (native), 1..7 -> m=4..10 (R=16..1024).  The sequence
 // starts at R=16 exactly as before (0,4,5,6,7), so the verified key
 // behaviour is unchanged; 8/9/10 are only appended for the slow time bases.
 wire [3:0] key_timebase=timebase_index==0?4'd0:4'd3+timebase_index;
 assign timebase_log2=KEY_TIMEBASE?key_timebase:stable_sw[2]?4'd0:4'd4+{2'd0,stable_sw[1:0]};
 assign tb_auto=stable_sw[2] && (KEY_TIMEBASE!=0);
 assign sw0=stable_sw[0];
 integer i;
 always @(posedge clk)begin
  if(!rst_n)begin
   key_meta<=15;key_sync<=15;switch_meta<=0;switch_sync<=0;
   timebase_index<=0;stable_keys<=15;stable_sw<=0;freeze<=0;trig_src<=0;falling<=0;sw_count<=0;
   for(i=0;i<4;i=i+1)key_count[i]<=0;
  end else begin
   key_meta<=key_n;key_sync<=key_meta;switch_meta<=sw;switch_sync<=switch_meta;
   if(switch_sync==stable_sw)sw_count<=0;
   else if(sw_count==DEBOUNCE_CLKS-1)begin stable_sw<=switch_sync;sw_count<=0;end
   else sw_count<=sw_count+1'b1;
   for(i=0;i<4;i=i+1)begin
    if(key_sync[i]==stable_keys[i])key_count[i]<=0;
    else if(key_count[i]==DEBOUNCE_CLKS-1)begin
     stable_keys[i]<=key_sync[i];key_count[i]<=0;
     if(!key_sync[i])case(i)
      0:freeze<=!freeze;
      1:trig_src<=!trig_src;
      2:if(KEY_TIMEBASE)timebase_index<=timebase_index==MAX_TIMEBASE-3?0:timebase_index+1'b1;
      3:falling<=!falling;
      default:begin end
     endcase
    end else key_count[i]<=key_count[i]+1'b1;
   end
  end
 end
endmodule
`default_nettype wire
