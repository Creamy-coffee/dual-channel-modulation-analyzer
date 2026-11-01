`timescale 1ns/1ps
// The proto field must describe the field set that is REALLY published.
//
// FOUR wire formats exist and they must never be confused.  The protocol number
// is 5 + fft_present + 2*et_present, so each set has its own number:
//   (+neither) the delivered bit : fields 62..76 skipped, ends ... dn=1024 proto=5
//   +FFT       FFT finder build  : 62..70 printed,        ends ... fflost= proto=6
//   +ET        ET display build  : 72..76 printed,        ends ... eth=... proto=7
//   +FFT +ET   both              : 62..70 and 72..76,     ends ... eth=... proto=8
//
// The old RTL printed " proto=6" unconditionally AND walked 62..70 as zeros,
// which told an FFT-less host "finder idle" about a finder that was never
// built.  It also left field 61 out of the case entirely, so PREFIX wrapped a
// 4-bit counter and emitted 16 NUL bytes, and field 70's text_len was 7 for an
// 8-character label, so " fflost=" printed as " fflost".
//
// This bench emits ONE META line and stops, so it runs in milliseconds rather
// than the chain bench's minutes - which is what makes it cheap enough to
// guard all four formats on every regression.  The assertions live in
// scripts/regression.py, same as the chain capture.
//
// NOTE: the file is opened "wb".  In text mode Windows turns every 0x0a into
// 0x0d 0x0a, which silently inserts a CR that is not on the wire at all.
`default_nettype none
module tb_display_proto;
 parameter integer LF_FFT=0;
 // +FFT / +ET select the wire format under test.  Same compiled image, four
 // runs, because the whole point is that the formats must not be confused.
 reg clk=0;always #7.6923076923 clk=~clk;
 reg rst=0;
 reg fft_present=0;
 reg et_present=0;
 initial if($test$plusargs("FFT"))fft_present=1'b1;
 initial if($test$plusargs("ET"))et_present=1'b1;
 // One snap_ready pulse starts a frame; the display emits the META line first.
 reg snap_ready=0;
 reg [15:0] frame_id=16'd1;
 reg [31:0] interval_ns=32'd15385;
 reg [9:0]  trig_pos=10'd256;
 reg filtered=0,auto_trigger=0;reg [1:0] fault=2'd0;reg [3:0] decim_log2=4'd0;
 wire [9:0] rd_addr;wire busy;
 wire signed [11:0] d0=12'sd0,d1=12'sd0;
 wire tx;
 integer fd,bytes=0,i;
 reg [7:0] cap[0:4095];

 // Distinctive FFT values: if the 62..70 skip ever regresses in the
 // fft_present=0 run, these land on the line and the Python tail check fails
 // instead of passing on a row of zeros.
 vofa_base_display #(.LF_FFT(LF_FFT),.CLK_FREQ(65000000),.BAUD(921600),.DISP_AW(10)) dut(
  .fft_present(fft_present),.et_present(et_present),
  .clk(clk),.rst_n(rst),.snap_ready(snap_ready),.snap_frame_id(frame_id),
  .snap_sample_interval_ns(interval_ns),.snap_trig_pos(trig_pos),
  .snap_filtered(filtered),.snap_fault(fault),.snap_decim_log2(decim_log2),
  .snap_auto_trigger(auto_trigger),.snap_rd_addr(rd_addr),.snap_rd_ch0(d0),.snap_rd_ch1(d1),
  .disp_busy(busy),.tx(tx),
  .lockin_valid(1'b0),.measurement_present(1'b0),.ratio_valid(1'b0),
  .measurement_id(32'd0),.measurement_age(32'd0),.amp0_q12(24'd0),.amp1_q12(24'd0),
  .phase0_turn(16'sd0),.phase1_turn(16'sd0),.phase_diff_turn(16'sd0),.ratio_q16(32'd0),
  .lockin_flags(8'h80),.lockin_gap(32'd0),.lockin_drop(32'd0),
  .lockin_detected(2'd0),.lockin_qualified(2'd0),.ref_fcw(32'd660764),
  .cfg_decim(2'd0),.cfg_integration(5'd12),
  .freq0_hz(32'd0),.freq1_hz(32'd0),.freq_result_id(32'd0),.vpp0(13'd0),.vpp1(13'd0),.rms0(12'd0),
  .cal_off0(12'sd0),.cal_off1(12'sd0),.cal_valid(1'b0),.cal_clip(2'd0),
  .afc_state(3'd3),.afc_ready(1'b1),.afc_reason(8'd0),
  .afc_error0(-32'sd123),.afc_error1(32'sd124),.afc_updates(32'd9),
  .guard_reason0(8'd0),.guard_reason1(8'd0),.guard_step0(16'd12),.guard_step1(16'd13),
  .guard_threshold0(29'd2048),.guard_threshold1(29'd2048),.frequency_valid(2'd3),
  .scan_state(3'd0),.scan_locked(1'b0),.scan_best_amp(24'd0),.scan_passes(16'd0),
  .bw_gear(2'd0),.bw_settling(1'b0),.bw_changes(16'd0),.bw_phase_step(16'd0),
  .aa_order(2'd3),.tb_auto(1'b1),.tb_m(4'd0),.tb_sat(1'b0),.ch_mismatch(1'b0),.ch_delta(16'd0),
  .fft_state(3'd4),.fft_bin0(12'd126),.fft_bin1(12'd126),
  .fft_freq0(32'd1999700),.fft_freq1(32'd2000300),
  .fft_mag0(24'd1280000),.fft_mag1(24'd1275000),
  .fft_frames(32'd7),.fft_lost(16'd317),
  .fft_valid_channels(2'd1),.fft_rate0(4'd14),.fft_rate1(4'd0),
  .fft_age0(32'd650000),.fft_age1(32'hffffffff),
  // ET ports: distinctive values in the +ET runs so a missing et= field fails
  // the tail check instead of passing on zeros; all zero in the other runs,
  // which models a real-time-only build (daq_top ties them to 0 when
  // ET_ENABLE=0).  Leaving them open is not an option: X would flow into r=,
  // t= and dt_ps= through the IDLE latch.
  .et_mode(et_present),.et_bins(et_present?10'd64:10'd0),.et_locked(et_present),
  .et_filled(et_present?7'd57:7'd0),.et_wraps(et_present?32'd12345:32'd0),
  .et_hits(et_present?32'd987654:32'd0),
  .et_ready_bins(7'd50),.et_epoch(32'd8),.et_reference_ok(1'b1),
  .et_rd_valid(1'b1),.et_rd_hits(16'd160));

 reg armed=1'b0;
 always @(posedge tx) armed<=1'b1;
 real bit_ns=1000000000.0/921600.0;
 reg [7:0] value;integer bitnum;
 // Same receiver discipline as the chain bench: arm only after the line has
 // been high, sample every bit at its centre, check the stop bit at 9.5 T.
 initial begin
  fd=$fopen("proto_meta.bin","wb");
  forever begin
   @(negedge tx);
   if(armed)begin
    armed=1'b0;
    #(1.5*bit_ns);
    for(bitnum=0;bitnum<8;bitnum=bitnum+1)begin
     value[bitnum]=tx;
     #(bit_ns);
    end
    if(tx!==1)$fatal(1,"stop-bit violation on byte %0d",bytes);
    cap[bytes]=value;bytes=bytes+1;
    if(value==8'h0a)begin
     for(i=0;i<bytes;i=i+1)$fwrite(fd,"%c",cap[i]);
     $fclose(fd);$finish;
    end
    if(bytes>=4000)$fatal(1,"no newline in 4000 bytes");
   end
  end
 end

 initial begin
  repeat(5)@(negedge clk);rst=1;
  repeat(20)@(negedge clk);
  snap_ready=1;
  @(negedge clk);snap_ready=0;
  wait(busy);
  // ~800 bytes at 928.6 kbaud is ~8.6 ms, i.e. ~560k clock edges.  Leave a
  // wide margin: this bench only has to emit ONE line, so time is cheap.
  repeat(1500000)@(negedge clk);
  $fatal(1,"META line never completed (busy=%b)",busy);
 end
endmodule
`default_nettype wire
