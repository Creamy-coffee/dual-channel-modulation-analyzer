`timescale 1ns/1ps
// Real ADC capture RTL -> R16 triggered display RAM -> real UART wire.
module tb_adc_display;
 reg clk=0;always #7.6923076923 clk=~clk;
 reg rst=0;reg [11:0] pin0=12'h800,pin1=12'h800;
 reg signed [11:0] lut0[0:6499],lut1[0:6499];
 integer phase=0,n,j,row,s0,s1,tick=0,fd,expected,bytes=0;
 integer hist0[0:49999],hist1[0:49999];
 wire [47:0] samples;wire [3:0] otr;wire valid,adc_ready;
 ad9238_capture #(.CHANNELS(2),.PARALLEL_PORTS(1),.USE_EXT_CLK(1),
 .STARTUP_CYCLES(4),.PIPELINE_FLUSH(16),.OFFSET_BINARY(1)) adc(
 .clk(clk),.rst_n(rst),.adc_clk_ext(clk),.mmcm_locked(rst),
 .adc_d0(pin0),.adc_d1(pin1),.adc_d2(12'd0),.adc_d3(12'd0),.adc_otr(4'd0),
 .adc_clk(),.samples(samples),.sample_otr(otr),.sample_valid(valid),.ready(adc_ready));
 wire busy,ready,filtered,aut,tx;wire [9:0] rd,pos;
 wire signed [11:0] q0,q1;wire [15:0] frame;wire [31:0] dt;
 wire [3:0] rate;wire [63:0] first;
 acq_trigger_buffer acq(.clk(clk),.rst_n(rst),.sample_valid(valid),
 .ch0(samples[11:0]),.ch1(samples[23:12]),.fault(otr[1:0]),.trig_level(12'sd0),
 .trig_mode(2'd0),.trig_src(1'b0),.pretrig(10'd256),.timebase_log2(4'd0),
 .freeze(1'b0),.disp_busy(busy),.snap_rd_addr(rd),.snap_rd_ch0(q0),.snap_rd_ch1(q1),
 .snap_ready(ready),.snap_frame_id(frame),.snap_sample_interval_ns(dt),
 .snap_trig_pos(pos),.snap_filtered(filtered),.snap_fault(),.snap_decim_log2(rate),
 .snap_auto_trigger(aut),.snap_first_index(first),.capture_errors());
 vofa_base_display display(.clk(clk),.rst_n(rst),.snap_ready(ready),
 .snap_frame_id(frame),.snap_sample_interval_ns(dt),.snap_trig_pos(pos),
 .snap_filtered(filtered),.snap_fault(2'd0),.snap_decim_log2(rate),
 .snap_auto_trigger(aut),.snap_rd_addr(rd),.snap_rd_ch0(q0),.snap_rd_ch1(q1),
 .lockin_valid(1'b0),.measurement_present(1'b0),.ratio_valid(1'b0),
 .measurement_id(32'd0),.measurement_age(32'd0),.amp0_q12(24'd0),.amp1_q12(24'd0),
 .phase0_turn(16'sd0),.phase1_turn(16'sd0),.phase_diff_turn(16'sd0),.ratio_q16(32'd0),
 .lockin_flags(8'h80),.lockin_gap(32'd0),.lockin_drop(32'd0),
 .lockin_detected(2'd0),.lockin_qualified(2'd0),.ref_fcw(32'd660764),
 .cfg_decim(2'd0),.cfg_integration(5'd12),
 .freq0_hz(32'd0),.freq1_hz(32'd0),.freq_result_id(32'd0),.vpp0(13'd0),.vpp1(13'd0),.rms0(12'd0),.cal_off0(12'sd0),.cal_off1(12'sd0),.cal_valid(1'b0),.cal_clip(2'd0),.afc_state(3'd3),.afc_ready(1'b1),.afc_reason(8'd0),.afc_error0(-32'sd123),.afc_error1(32'sd124),.afc_updates(32'd9),.guard_reason0(8'd0),.guard_reason1(8'd0),.guard_step0(16'd12),.guard_step1(16'd13),.guard_threshold0(29'd2048),.guard_threshold1(29'd2048),.frequency_valid(2'd3),.scan_state(3'd0),.scan_locked(1'b0),.scan_best_amp(24'd0),.scan_passes(16'd0),.bw_gear(2'd0),.bw_settling(1'b0),.bw_changes(16'd0),.bw_phase_step(16'd0),
// proto=6 finder telemetry.  These MUST be driven: an unconnected input
// floats to X and the BCD formatter then prints X digits as NUL bytes, which
// silently corrupts the captured META line (tests/test_receiver.py reads it).
// fft_present is the wire-format switch: 1 publishes the proto=6 field set
// (ffs..fflost + proto=6), 0 jumps field 61 -> 71 and publishes proto=5.
// Left unconnected it is X, and the field==61 jump condition then blends into
// X, which re-opens the 16-NUL hole through the missing-case default.
// Values model a locked 2 MHz finder: bin 126, interpolated peaks either side,
// magnitudes on the 16*A^2 scale for A=800 counts.
 .fft_present(1'b1),.et_present(1'b0),
 .et_mode(1'b0),.et_bins(10'd0),.et_locked(1'b0),
 .et_filled(7'd0),.et_wraps(32'd0),.et_hits(32'd0),
 .fft_state(3'd4),.fft_bin0(12'd126),.fft_bin1(12'd126),
 .fft_freq0(32'd1999700),.fft_freq1(32'd2000300),
 .fft_mag0(24'd1280000),.fft_mag1(24'd1275000),
 .fft_frames(32'd7),.fft_lost(16'd0),
 .aa_order(2'd3),.tb_auto(1'b1),.tb_m(4'd0),.tb_sat(1'b0),.ch_mismatch(1'b0),.ch_delta(16'd0),.disp_busy(busy),.tx(tx));
 initial begin
  for(n=0;n<6500;n=n+1)begin
   lut0[n]=$rtoi(1000.0*$sin(6.283185307179586*1000000.0*n/65000000.0));
   lut1[n]=$rtoi(500.0*$sin(6.283185307179586*20000000.0*n/65000000.0+0.5));
  end
  repeat(5)@(negedge clk);rst=1;
 end
 always @(negedge clk)if(rst)begin
  pin0=lut0[phase]^12'h800;pin1=lut1[phase]^12'h800;
  phase=phase==6499?0:phase+1;
 end
 always @(posedge clk)if(rst)begin
  if(tick<50000)begin hist0[tick]=$signed(samples[11:0]);hist1[tick]=$signed(samples[23:12]);end
  tick=tick+1;
 end
 initial begin
  wait(ready);#1;
  if(aut || rate!=0 || filtered || pos!=256)$fatal(1,"sine trigger metadata");
  expected=$fopen("chain_expected.csv","w");$fwrite(expected,"ch0,ch1\n");
  for(row=0;row<1024;row=row+1)begin
   s0=0;s1=0;
   for(j=0;j<1;j=j+1)begin s0=s0+hist0[first+row-j];s1=s1+hist1[first+row-j];end
   $fwrite(expected,"%0d,%0d\n",s0,s1);
  end
  $fclose(expected);
  wait(busy);wait(!busy);repeat(100)@(negedge clk);
  $fclose(fd);$display("PASS ADC-display-UART: 1MHz+20MHz, native RAW R1, 1024 paired points, independent nominal-baud receiver, %0d bytes",bytes);$finish;
 end
 reg [7:0] value;integer bitnum;
 real bit_ns=1000000000.0/921600.0;
 // Receiver, independent of the RTL.  Three things matter and the first
 // version got all three wrong:
 //
 //  1. ARM only after the line has been seen HIGH.  A byte whose bit7/bit6 are
 //     both 1 ('#'=0x23, 'o', 's') holds tx high through its stop bit and the
 //     next start bit, so a bare @(negedge tx) never fires for it; the old
 //     receiver started on the first 1->0 DATA bit and decoded 0x00, injecting
 //     a spurious NUL into the capture while the bench still printed PASS.
 //  2. Sample each data bit at its CENTRE, i.e. (1.5 + n) T from the start edge.
 //     The loop used to step #(bit_ns) after sampling, so the last sample
 //     landed at 8.5 T - one bit late - and the stop-bit check then read bit7.
 //  3. Check the stop bit at 9.5 T.
 //
 // Also note the nominal-vs-actual baud: the DUT divides 65 MHz by an integer
 // CLKS_PER_BIT=70, giving 928.6 kbaud rather than the nominal 921600, and the
 // capture runs ~8 ms per frame.  The error is +0.76%, i.e. 0.07 bit over ten
 // bit times, which a centre-sampled receiver tolerates; it is NOT a fault, and
 // it is the reason the samples must stay on the bit centres.
 reg armed=1'b0;
 always @(posedge tx) armed <= 1'b1;
 initial begin
  fd=$fopen("chain_uart.bin","wb");
  forever begin
   @(negedge tx);
   if (armed) begin
    armed = 1'b0;
    #(1.5*bit_ns);
    for(bitnum=0;bitnum<8;bitnum=bitnum+1)begin
     value[bitnum]=tx;
     #(bit_ns);                      // advance to the next bit centre
    end
    if(tx!==1)begin
     $display("FATAL stop-bit violation on byte %0d (0x%02x) at %0t",bytes,value,$time);
     $fwrite(fd,"%c",value);$fclose(fd);$finish;
    end
    $fwrite(fd,"%c",value);bytes=bytes+1;
   end
  end
 end
 initial begin #300000000;$fatal(1,"integrated display timeout");end
endmodule
