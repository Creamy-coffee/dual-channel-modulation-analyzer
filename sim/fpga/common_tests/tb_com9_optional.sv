`timescale 1ns/1ps
module tb_com9_optional;
 reg clk=0,rst=0;always #10 clk=~clk;
 reg [11:0] a=2048,b=2048;
 wire tx_debug,tx_quiet,text_debug,text_quiet;
 integer n=0,edges=0,cycles=0;reg oldtx=1;reg text_seen=0;
 daq_top #(.STARTUP_CYCLES(8),.BINARY_POINTS(16),.DISP_AW(10),
   .FFT_ENABLE(0),.AFC_ENABLE(0),.AUTO_TIMEBASE(0),.BW_MODE(0),
   .REF_FREQ_HZ(1000000),.INTEGRATION_LOG2(8),.COM9_TEXT_ENABLE(1)) debug_on(
   .clk_50m(clk),.rst_n(rst),.adc_d0(a),.adc_d1(b),.adc_otr(1'b0),.adc_otr_b(1'b0),
   .digital_in(4'b0),.key_n(4'hf),.sw(3'b0),.v32_rx(1'b1),.v32_tx(tx_debug),.uart_tx(text_debug));
 daq_top #(.STARTUP_CYCLES(8),.BINARY_POINTS(16),.DISP_AW(10),
   .FFT_ENABLE(0),.AFC_ENABLE(0),.AUTO_TIMEBASE(0),.BW_MODE(0),
   .REF_FREQ_HZ(1000000),.INTEGRATION_LOG2(8),.COM9_TEXT_ENABLE(0)) debug_off(
   .clk_50m(clk),.rst_n(rst),.adc_d0(a),.adc_d1(b),.adc_otr(1'b0),.adc_otr_b(1'b0),
   .digital_in(4'b0),.key_n(4'hf),.sw(3'b0),.v32_rx(1'b1),.v32_tx(tx_quiet),.uart_tx(text_quiet));
 initial begin #100;rst=1;end
 always @(negedge debug_on.clk_sys) begin
   n=n+1;a=2048+$rtoi(400*$sin(6.283185307*n/65.0));
   b=2048+$rtoi(200*$sin(6.283185307*n/65.0+1.570796327));
   if(debug_on.sys_rst_n)begin
     cycles=cycles+1;
     if(tx_debug!==tx_quiet)$fatal(1,"AB5 output changed when COM9 disabled");
     if(debug_on.linux_meas_body!==debug_off.linux_meas_body)$fatal(1,"measurement packet changed");
     if({debug_on.li_amp0,debug_on.li_amp1,debug_on.li_ph0,debug_on.li_ph1,debug_on.li_pdiff,debug_on.li_ratio,debug_on.li_qual}!==
        {debug_off.li_amp0,debug_off.li_amp1,debug_off.li_ph0,debug_off.li_ph1,debug_off.li_pdiff,debug_off.li_ratio,debug_off.li_qual})
       $fatal(1,"lock-in values changed");
     if(text_quiet!==1'b1)$fatal(1,"disabled UART must idle high");
     if(!text_debug)text_seen=1;
     if(tx_debug!=oldtx)edges=edges+1;oldtx=tx_debug;
     if(cycles==200000)begin
       if(edges<100 || !text_seen || debug_on.wave_completed<2)$fatal(1,"insufficient active traffic edges=%0d text=%0d frames=%0d",edges,text_seen,debug_on.wave_completed);
       $display("COM9_OPTIONAL_EQUIV_PASS cycles=%0d wave_frames=%0d edges=%0d",cycles,debug_on.wave_completed,edges);$finish;
     end
   end
 end
 initial begin #4000000;$fatal(1,"timeout");end
endmodule
