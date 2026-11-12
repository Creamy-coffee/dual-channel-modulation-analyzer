`timescale 1ns/1ps
// Diagnosis only: exercise the actual short AFC using an ADC sine multiplied
// by a quadrature NCO. The comparison removes the 2*f mixer image analytically.
module tb_low_afc_diagnosis;
 parameter integer IMAGE_FREE=0,INPUT_HZ=1000,NOMINAL_HZ=996,RUN_CYCLES=1300000;
 reg clk=0;always #5 clk=~clk;
 reg rst=0;reg signed [27:0] i0=0,q0=0,i1=0,q1=0;
 reg [31:0] tick=0,fcw=(64'd4294967296*NOMINAL_HZ)/65000000;
 localparam [31:0] NOM=(64'd4294967296*NOMINAL_HZ)/65000000;
 wire [31:0] target,updates;wire commit,ready;wire [2:0] state;wire [7:0] reason;
 wire signed [31:0] e0,e1;
 local_afc_2ch #(.ADAPTIVE_WINDOW(0)) dut(.clk(clk),.rst_n(rst),.clear(1'b0),.stream_ok(1'b1),.mix_valid(rst),
   .mi0(i0),.mq0(q0),.mi1(i1),.mq1(q1),.sample_time(tick),.nominal_fcw(NOM),.current_fcw(fcw),
   .target_fcw(target),.commit(commit),.ready_measurement(ready),.state(state),
   .residual0_fcw(e0),.residual1_fcw(e1),.reason(reason),.updates(updates),.observation_id(),.last_timestamp());
 real theta=0,nco=0,x,y;
 integer n=0,rejects=0,last_reason=0,stable_run=0,longest_stable=0;
 always @(negedge clk)if(rst)begin
   theta=theta+6.283185307179586*INPUT_HZ/65000000.0;
   nco=nco+6.283185307179586*fcw/4294967296.0;
   if(theta>6.283185307179586)theta=theta-6.283185307179586;
   if(nco>6.283185307179586)nco=nco-6.283185307179586;
   if(IMAGE_FREE)begin
     i0=$rtoi(400*2047*0.5*$sin(theta-nco));q0=$rtoi(-400*2047*0.5*$cos(theta-nco));
     i1=$rtoi(200*2047*0.5*$sin(theta-nco+1.570796326794897));q1=$rtoi(-200*2047*0.5*$cos(theta-nco+1.570796326794897));
   end else begin
     x=$rtoi(400*$sin(theta));y=$rtoi(200*$sin(theta+1.570796326794897));
     i0=$rtoi(x*2047*$cos(nco));q0=$rtoi(-x*2047*$sin(nco));
     i1=$rtoi(y*2047*$cos(nco));q1=$rtoi(-y*2047*$sin(nco));
   end
 end
 always @(posedge clk)if(rst)begin
   tick<=tick+1;n=n+1;
   if(commit)fcw<=target;
   if(ready)begin stable_run=stable_run+1;if(stable_run>longest_stable)longest_stable=stable_run;end else stable_run=0;
   if(reason==4 && last_reason!=4)rejects=rejects+1;
   last_reason=reason;
   if(n%130000==0)$display("TRACE n=%0d hz=%f state=%0d reason=%0d updates=%0d err=%0d/%0d",n,fcw*65000000.0/4294967296.0,state,reason,updates,e0,e1);
   if(n==RUN_CYCLES)begin
     $display("DIAG_DONE image_free=%0d input_hz=%0d final_hz=%f updates=%0d disagreement_episodes=%0d longest_stable_clocks=%0d",IMAGE_FREE,INPUT_HZ,fcw*65000000.0/4294967296.0,updates,rejects,longest_stable);$finish;
   end
 end
 initial begin repeat(8)@(negedge clk);rst=1;end
endmodule

