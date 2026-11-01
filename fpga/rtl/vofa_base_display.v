`timescale 1ns/1ps
`default_nettype none
// One owner for all state. A byte is advanced ONLY on valid && ready.
// Fixed-width decimal FireWater: every sample line has its own wave: prefix.
module vofa_base_display #(
 parameter integer CLK_FREQ=65000000,BAUD=921600,DISP_AW=10,DISP_N=(1<<DISP_AW),
 // ET_AW only sizes the et_filled port, so it stays a parameter.  Whether the
 // ET fields are PUBLISHED is et_present, a port - see below.
 parameter integer LF_FFT=0,ET_AW=6
)(
 // 1 only when the 4096-point FFT finder is actually built.  The proto field
 // must describe the field set that is really published: with fft_present=0
 // the ffs/ffb*/fff*/ffm*/ffid/fflost slots are tied to zero upstream, so
 // calling the line "proto=6" would invite a host to read "finder idle" out of
 // a finder that does not exist.  It is a port rather than a parameter so a
 // testbench can exercise both wire formats from one compiled image.
 input wire fft_present,
 // 1 only when et_display is actually built (daq_top's ET_ENABLE).  Same
 // reasoning as fft_present: with no ET hardware the etm/etb/etc/etw/eth slots
 // carry zeros that a host would read as "ET mode off, 0 bins" instead of
 // "this build has no ET display".  A port so the bench can test proto=7 too.
 input wire et_present,
 input wire clk,rst_n,snap_ready,input wire [15:0] snap_frame_id,
 input wire [31:0] snap_sample_interval_ns,input wire [DISP_AW-1:0] snap_trig_pos,
 input wire snap_filtered,input wire [1:0] snap_fault,
 input wire [3:0] snap_decim_log2,input wire snap_auto_trigger,
 output reg [DISP_AW-1:0] snap_rd_addr,
 input wire signed [11:0] snap_rd_ch0,snap_rd_ch1,
 // ---- lock-in amplitude/phase measurement (step 2) ----
 // Atomic measurement registers are sampled as one record at snap_ready.
 // present/ratio_valid and age explicitly describe freshness and usability.
 input wire lockin_valid,measurement_present,ratio_valid,
 input wire [31:0] measurement_id,measurement_age,
 input wire [23:0] amp0_q12,amp1_q12,
 input wire signed [15:0] phase0_turn,phase1_turn,phase_diff_turn,
 input wire [31:0] ratio_q16,
 input wire [7:0] lockin_flags,
 input wire [31:0] lockin_gap,lockin_drop,
 input wire [1:0] lockin_detected,lockin_qualified,
 input wire [31:0] ref_fcw,
 input wire [1:0] cfg_decim,
 input wire [4:0] cfg_integration,
 // ---- step 3: wave parameters (equal-precision frequency + window metrics) ----
 // Latched atomically at snap_ready like every other META field.  freq_result_id
 // is the freshness stamp of the 1 s frequency gate (the reading itself only
 // changes once per second, so a stale value must be distinguishable).
 input wire [31:0] freq0_hz,freq1_hz,
 input wire [31:0] freq_result_id,
 input wire [12:0] vpp0,vpp1,
 input wire [11:0] rms0,
 // ---- zero-bias calibration telemetry ----
 input wire signed [11:0] cal_off0, cal_off1,
 input wire cal_valid,
 input wire [1:0] cal_clip,
 input wire [2:0] afc_state,input wire afc_ready,input wire [7:0] afc_reason,
 input wire signed [31:0] afc_error0,afc_error1,input wire [31:0] afc_updates,
 input wire [7:0] guard_reason0,guard_reason1,input wire [15:0] guard_step0,guard_step1,
 input wire [28:0] guard_threshold0,guard_threshold1,input wire [1:0] frequency_valid,
 // ---- run-time frequency search telemetry (proto=3) ----
 // scan: 0 idle 1 coarse sweep 2 fine sweep 3 locked.  samp is the Q12
 // amplitude of the winning candidate, spass counts completed sweep passes.
 input wire [2:0] scan_state,input wire scan_locked,
 input wire [23:0] scan_best_amp,input wire [15:0] scan_passes,
 // ---- measurement bandwidth gear telemetry (proto=4) ----
 // bw: 0 fast (1 ms) 1 balanced (4 ms) 2 precise (64 ms).  bwc counts gear
 // changes (rapid growth = the gear is oscillating).  bwp is the circular
 // phase step between the last two windows, 65536 counts = 360 deg.
 input wire [1:0] bw_gear,input wire bw_settling,
 input wire [15:0] bw_changes,input wire [15:0] bw_phase_step,
 // ---- display telemetry (proto=5) --------------------------------------
 // aa  : display decimator order (0 = native RAW, 1 = boxcar average).
 //       The field was introduced when the display path was believed to hold a
 //       3rd-order CIC.  It does not: measured response is |sinc(f/fsd)|, i.e.
 //       only -3.9 dB at the decimated Nyquist.  Field width and position are
 //       unchanged so the existing host parsers still work; only the meaning
 //       is corrected.  See reports/decimator_findings_20261019.md.
 // tb  : {sat,auto,m}  -> bit4 auto, bit5 saturated (cannot reach 3 cycles)
 // fm  : bit8 two-channel frequency mismatch, low 8 bits = mean |s0-s1|/16
 // dn  : number of wave: rows that follow (DISP_N).  Published so a host can
 //       check it received a whole frame instead of assuming 1024.
 input wire [1:0] aa_order,input wire tb_auto,input wire [3:0] tb_m,
 input wire tb_sat,input wire ch_mismatch,input wire [15:0] ch_delta,
 // ---- 4096-point shared-FFT finder telemetry (proto=6) -------------------
 // ffs    : finder state 0 idle 1 search 2 conv 3 settle 4 track 5 lost.
 // ffb0/1 : peak bin per channel.  fff0/1 : 3-point interpolated peak, Hz.
 // ffm0/1 : peak magnitude, mag>>3 (full scale = 16*A^2 for A ADC counts).
 // ffid   : published frame counter.  fflost : lock losses since reset.
 // These slots are only printed when fft_present=1; see DIGITS.
 input wire [2:0] fft_state,
 input wire [11:0] fft_bin0,fft_bin1,
 input wire [31:0] fft_freq0,fft_freq1,
 input wire [23:0] fft_mag0,fft_mag1,
 input wire [31:0] fft_frames,
 input wire [15:0] fft_lost,
 input wire [1:0] fft_valid_channels,
 input wire [3:0] fft_rate0,fft_rate1,
 input wire [31:0] fft_age0,fft_age1,
 // ---- equivalent-time display telemetry (ET builds only) ------------------
 // etm : bit0 ET display mode active, bit1 AFC locked (phase reference real).
 //       In ET mode the wave: rows are PHASE-BINNED AVERAGES of one signal
 //       period reassembled from many periods - not a real-time snapshot.
 // etb : phase bins per period (2^ET_AW); it is also the wave: row count of
 //       an ET frame (a real-time frame always carries DISP_N rows).
 // etc : bins with at least one real hit.  Physics bounds the coverage
 //       (fs/fi = p/q gives p positions per q periods); an unfilled bin is a
 //       coverage gap, never interpolated data.
 // etw : NCO period wraps since the last clear (signal periods observed).
 // eth : averaged samples since the last clear.
 input wire et_mode,
 input wire [9:0] et_bins,
 input wire et_locked,
 input wire [ET_AW:0] et_filled,
 input wire [31:0] et_wraps,et_hits,
 // ET revision 2 (protocol 9 without FFT / 10 with FFT): per-bin qualification.
 input wire [ET_AW:0] et_ready_bins,
 input wire [31:0] et_epoch,
 input wire et_reference_ok,et_rd_valid,
 input wire [15:0] et_rd_hits,
 output reg disp_busy,output wire tx
);
 function [31:0] interval_for_mode;
  input [3:0] m;
  begin case(m)
   4'd0: interval_for_mode=(64'd1000000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd1: interval_for_mode=(64'd2000000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd2: interval_for_mode=(64'd4000000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd3: interval_for_mode=(64'd8000000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd4: interval_for_mode=(64'd16000000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd5: interval_for_mode=(64'd32000000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd6: interval_for_mode=(64'd64000000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd7: interval_for_mode=(64'd128000000000000+CLK_FREQ/2)/CLK_FREQ;
   // 8..10 cover the slow display time bases: R=256/512/1024 -> 3.94/7.88/15.75 us
   4'd8: interval_for_mode=(64'd256000000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd9: interval_for_mode=(64'd512000000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd10:interval_for_mode=(64'd1024000000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd11:interval_for_mode=(64'd2048000000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd12:interval_for_mode=(64'd4096000000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd13:interval_for_mode=(64'd8192000000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd14:interval_for_mode=(64'd16384000000000000+CLK_FREQ/2)/CLK_FREQ;
   4'd15:interval_for_mode=(64'd32768000000000000+CLK_FREQ/2)/CLK_FREQ;
   default: interval_for_mode=0;
  endcase end
 endfunction
 localparam IDLE=0,PREP=1,BCD=2,PREFIX=3,SIGN=4,DIGITS=5,NEWLINE=6,
            RD_ADDR=7,RD_WAIT=8,DRAIN=9,NAN_ROW=10;
 reg [3:0] state;
 reg [1:0] ffv_hold;reg [3:0] ffr0_hold,ffr1_hold;reg [31:0] ffa0_hold,ffa1_hold;
 reg meta;
// META fields 0..31 include measurement presence, ratio validity, id and age,
// plus the step-3 wave parameters (f0/f1/vpp0/vpp1/rms0/fid, fields 26..31).
// MERGED layout: field32 = build id, fields 33..36 = zero-bias calibration
// telemetry (off0/off1/calv/clip).  This combines the V4 RAW-display build id
// with the lock-in branch's calibration fields in ONE META line.
// Fields 37..49 = AFC/guard, 50 = fsrc, 51..54 = sweep, 55..57 = bandwidth
// gear, 58..61 = display path, 62..70 = 4096-point FFT finder (printed only
// when fft_present=1), 72..76 = equivalent-time display (printed only when
// et_present=1).  The published protocol number is 5 + fft_present +
// 2*et_present, so every buildable field set has its own number:
//   5 = base           6 = base+FFT
//   7 = base+ET        8 = base+FFT+ET
// The counter walks EVERY allocated index with no holes: a field missing from
// the case below falls into the text_len=0 default, which used to make PREFIX
// wrap a 4-bit counter and emit 16 NUL bytes (fields 58..61 did exactly that).
// If a slot has nothing to say, give it a real field - do not leave a hole.
// The only permitted gaps are the two parameter-guarded skips: 62..70 without
// fft_present (jump 61 -> 71) and 72..76 without et_present (is_last on 71).
// The counter width MUST cover the highest field or the line never terminates
// (display deadlock): it grew 4 -> 5 -> 6 -> 7 bits as fields were added.  A
// 6-bit counter wraps 63->0, which would make the proto=6 fields unreachable.
reg [6:0] field;
 reg [2:0] as_hold;reg ar_hold;reg [7:0] ae_hold,gr0_hold,gr1_hold;
 reg signed [31:0] df0_hold,df1_hold;reg [31:0] au_hold;
 reg [15:0] gs0_hold,gs1_hold;reg [28:0] gt0_hold,gt1_hold;reg [1:0] fv_hold;
 reg [15:0] frame_hold;reg [DISP_AW-1:0] trig_hold;
 reg [3:0] decim_hold;reg filt_hold,auto_hold;reg [1:0] fault_hold;
 reg [31:0] interval_ps;
 reg [23:0] amp0_hold,amp1_hold;
 reg signed [15:0] ph0_hold,ph1_hold,pdiff_hold;
 reg [31:0] ratio_hold;
 reg present_hold,ratio_valid_hold;reg [31:0] mid_hold,age_hold;
 reg [7:0] lflags_hold;
 reg [31:0] lgap_hold,ldrop_hold;
 reg [1:0] ldet_hold,lqual_hold;
 reg [31:0] ref_fcw_hold;
 reg [1:0] cfg_decim_hold;
 reg [4:0] cfg_integ_hold;
 reg [31:0] freq0_hold,freq1_hold,freq_id_hold;
 reg [12:0] vpp0_hold,vpp1_hold;reg [11:0] rms0_hold;
 reg signed [11:0] cal_off0_hold, cal_off1_hold; reg cal_valid_hold; reg [1:0] cal_clip_hold;
 reg [2:0] scan_state_hold; reg scan_locked_hold;
 reg [23:0] scan_best_hold; reg [15:0] scan_pass_hold;
 reg [1:0] bw_gear_hold; reg bw_settling_hold;
 reg [15:0] bw_changes_hold, bw_phase_step_hold;
 reg [1:0] aa_hold; reg tbauto_hold, tbsat_hold, fm_hold;
 reg [3:0] tbm_hold; reg [7:0] fmd_hold;
 reg [2:0] fft_state_hold;
 reg [11:0] fft_bin0_hold,fft_bin1_hold;
 reg [31:0] fft_freq0_hold,fft_freq1_hold,fft_frames_hold;
 reg [23:0] fft_mag0_hold,fft_mag1_hold;
 reg [15:0] fft_lost_hold;
 // ET display hold registers (proto=7).  Latched atomically at snap_ready
 // like every other META field.
 reg et_mode_hold,et_locked_hold;
 reg [9:0] et_bins_hold; reg [ET_AW:0] et_filled_hold;
 reg [31:0] et_wraps_hold,et_hits_hold;
 reg [ET_AW:0] et_ready_hold;
 reg [31:0] et_epoch_hold;
 reg et_ref_hold,et_info;
 reg [3:0] nan_index;
 localparam [95:0] NAN_TEXT="wave:nan,nan";
// An ET frame carries et_bins wave: rows; a real-time frame carries DISP_N.
localparam [DISP_AW-1:0] ROWS_RT=DISP_N-1;
// row_last MUST read the LATCHED copies, not the live inputs.  dn= in the META
// line is latched at snap_ready; if the row loop used the live et_mode/et_bins
// a switch flipped during the ~100 ms frame would change the row count after
// the header had already announced a different one.  The host would then see
// a frame that disagrees with its own dn= and could not tell truncation from a
// short frame.  Both hold registers are updated in the same snap_ready cycle
// the header is captured, so header and body always describe one frame.
wire [DISP_AW-1:0] row_last = et_mode_hold
    ? (et_bins_hold[DISP_AW-1:0]-1'b1) : ROWS_RT;
 // Prefix registers are 80 bits (10 bytes): the longest label is 9 chars = 72
 // bits.  Anything narrower silently truncates it (see PREP below).
 reg [79:0] prefix_shift;reg [3:0] prefix_left;
 reg [31:0] magnitude;reg [39:0] bcd;
 reg [5:0] bits_left;reg [3:0] digit;
 reg signed_value,negative,last_field;
 reg [DISP_AW-1:0] row;
 reg [31:0] number;reg [79:0] text_prefix;reg [3:0] text_len,width;
 reg is_signed,is_last;
 always @*begin
  number=0;text_prefix=0;text_len=0;width=1;is_signed=0;is_last=0;
  if(meta)case(field)
   0:begin text_prefix="#META f=";text_len=8;number=frame_hold;width=5;end
   1:begin text_prefix=" fs=";text_len=4;number=CLK_FREQ;width=8;end
   2:begin text_prefix=" r=";text_len=3;number=32'd1<<decim_hold;width=LF_FFT?5:4;end
   3:begin text_prefix=" dt_ps=";text_len=7;number=interval_ps;width=10;end
   4:begin text_prefix=" t=";text_len=3;number=trig_hold;width=4;end
   5:begin text_prefix=" F=";text_len=3;number=filt_hold;end
   6:begin text_prefix=" E=";text_len=3;number=fault_hold;end
   7:begin text_prefix=" A=";text_len=3;number=auto_hold;end
   // Widths must cover the full numeric range of each field (the formatter
   // prints exactly 'width' digits and silently drops higher ones):
   //   amp0/amp1 : Q12 amplitude, max 4095*4096 = 16773120 (8 digits); a
   //              clipped sine can even push the fundamental past 8 digits
   //              worth of headroom, so 7 was not enough.
   //   ratio     : 32-bit Q16 quotient, max 4294967295 (10 digits).
   8:begin text_prefix=" amp0=";text_len=6;number=amp0_hold;width=8;end
   9:begin text_prefix=" amp1=";text_len=6;number=amp1_hold;width=8;end
   10:begin text_prefix=" p0=";text_len=4;number={{16{ph0_hold[15]}},ph0_hold};width=6;is_signed=1;end
   11:begin text_prefix=" p1=";text_len=4;number={{16{ph1_hold[15]}},ph1_hold};width=6;is_signed=1;end
   12:begin text_prefix=" pdiff=";text_len=7;number={{16{pdiff_hold[15]}},pdiff_hold};width=6;is_signed=1;end
   13:begin text_prefix=" ratio=";text_len=7;number=ratio_hold;width=10;end
   14:begin text_prefix=" flags=";text_len=7;number=lflags_hold;width=3;end
   15:begin text_prefix=" gap=";text_len=5;number=lgap_hold;width=10;end
   16:begin text_prefix=" drop=";text_len=6;number=ldrop_hold;width=10;end
   17:begin text_prefix=" ref_fcw=";text_len=9;number=ref_fcw_hold;width=10;end
   18:begin text_prefix=" decim=";text_len=7;number={29'd0,cfg_decim_hold,1'b0}+32'd4;width=3;end
   19:begin text_prefix=" integ=";text_len=7;number={27'd0,cfg_integ_hold};width=3;end
   20:begin text_prefix=" det=";text_len=5;number={30'd0,ldet_hold};width=2;end
   21:begin text_prefix=" qual=";text_len=6;number={30'd0,lqual_hold};width=2;end
   22:begin text_prefix=" mv=";text_len=4;number=present_hold;end
   23:begin text_prefix=" rv=";text_len=4;number=ratio_valid_hold;end
   24:begin text_prefix=" mid=";text_len=5;number=mid_hold;width=10;end
   25:begin text_prefix=" age=";text_len=5;number=age_hold;width=10;end
   // ---- step 3: wave parameters.  Widths cover the full numeric range:
   //   f0/f1  : edge count of a 1 s gate at 65 MHz -> max 65e6 (8 digits);
   //            10 keeps headroom for a longer gate parameter.
   //   vpp0/1 : 13-bit peak-to-peak (max 8191 -> 5 digits).
   //   rms0   : 12-bit AC-RMS (max 4095 -> 5 digits).
   //   fid    : 32-bit gate counter (freshness of f0/f1).
   26:begin text_prefix=" f0=";text_len=4;number=freq0_hold;width=10;end
   27:begin text_prefix=" f1=";text_len=4;number=freq1_hold;width=10;end
   28:begin text_prefix=" vpp0=";text_len=6;number={19'd0,vpp0_hold};width=5;end
   29:begin text_prefix=" vpp1=";text_len=6;number={19'd0,vpp1_hold};width=5;end
   30:begin text_prefix=" rms0=";text_len=6;number={20'd0,rms0_hold};width=5;end
   31:begin text_prefix=" fid=";text_len=5;number=freq_id_hold;width=10;end
   32:begin text_prefix=" build=";text_len=7;number=20261020;width=8;end
   33:begin text_prefix=" off0=";text_len=6;number={{20{cal_off0_hold[11]}},cal_off0_hold};width=5;is_signed=1;end
   34:begin text_prefix=" off1=";text_len=6;number={{20{cal_off1_hold[11]}},cal_off1_hold};width=5;is_signed=1;end
   35:begin text_prefix=" calv=";text_len=6;number={31'd0,cal_valid_hold};width=1;end
   36:begin text_prefix=" clip=";text_len=6;number={30'd0,cal_clip_hold};width=2;end
   37:begin text_prefix=" afc=";text_len=5;number=as_hold;width=3;end
   38:begin text_prefix=" ar=";text_len=4;number=ar_hold;width=1;end
   39:begin text_prefix=" ae=";text_len=4;number=ae_hold;width=3;end
   40:begin text_prefix=" df0=";text_len=5;number=df0_hold;width=10;is_signed=1;end
   41:begin text_prefix=" df1=";text_len=5;number=df1_hold;width=10;is_signed=1;end
   42:begin text_prefix=" au=";text_len=4;number=au_hold;width=10;end
   43:begin text_prefix=" gr0=";text_len=5;number=gr0_hold;width=3;end
   44:begin text_prefix=" gr1=";text_len=5;number=gr1_hold;width=3;end
   45:begin text_prefix=" gs0=";text_len=5;number=gs0_hold;width=5;end
   46:begin text_prefix=" gs1=";text_len=5;number=gs1_hold;width=5;end
   47:begin text_prefix=" gt0=";text_len=5;number=gt0_hold;width=10;end
   48:begin text_prefix=" gt1=";text_len=5;number=gt1_hold;width=10;end
   49:begin text_prefix=" fv=";text_len=4;number=fv_hold;width=1;end
   50:begin text_prefix=" fsrc=";text_len=6;number=32'd1;width=1;end
   // Widths again: scan is a 3-bit state (0..7) and bw carries the gear in
   // its low 2 bits plus the settling flag in bit 2, so both need width 1
   // (one digit).  tb is 6 bits (8..63 -> 2 digits), fm is 9 bits (0..511).
   51:begin text_prefix=" scan=";text_len=6;number={29'd0,scan_state_hold};width=1;end
   52:begin text_prefix=" slock=";text_len=7;number={31'd0,scan_locked_hold};width=1;end
   53:begin text_prefix=" samp=";text_len=6;number={8'd0,scan_best_hold};width=8;end
   54:begin text_prefix=" spass=";text_len=7;number={16'd0,scan_pass_hold};width=5;end
   55:begin text_prefix=" bw=";text_len=4;number={29'd0,bw_settling_hold,bw_gear_hold};width=1;end
   56:begin text_prefix=" bwc=";text_len=5;number={16'd0,bw_changes_hold};width=5;end
   57:begin text_prefix=" bwp=";text_len=5;number={16'd0,bw_phase_step_hold};width=5;end
   // The proto=5 display-path fields were documented above (and their ports
   // and hold registers exist) but the case entries were never written, so
   // the field counter walked 58..61 with text_len=0 and emitted 16 NUL
   // bytes plus a bare '0' for each - invisible garbage inside an otherwise
   // ASCII line.  Restored here per the documented layout:
   //   aa : display decimator order (0 = native RAW, 1 = boxcar average).
   //   tb : {sat,auto,m} -> bit4 auto, bit5 saturated (even R=1024 cannot
   //        reach 3 cycles).
   //   fm : bit8 two-channel frequency mismatch, low 8 bits = mean |s0-s1|/16.
   //   dn : number of wave: rows that follow this META line.  The host used
   //        to have to assume 1024; now it can check the line is complete.
   //        An ET frame (etm bit0=1) carries et_bins rows instead of DISP_N.
   58:begin text_prefix=" aa=";text_len=4;number={30'd0,aa_hold};width=1;end
   59:begin text_prefix=" tb=";text_len=4;number={26'd0,tbsat_hold,tbauto_hold,tbm_hold};width=2;end
   60:begin text_prefix=" fm=";text_len=4;number={23'd0,fm_hold,fmd_hold};width=3;end
   61:begin text_prefix=" dn=";text_len=4;number=et_mode_hold?{22'd0,et_bins_hold}:DISP_N;width=4;end
   // proto bumped to 3: fields 51..54 are the run-time sweep telemetry.  A
   // host that does not know proto=3 must not assume the old field set.
   // proto bumped to 4: fields 55..57 report the measurement bandwidth gear.
   // proto bumped to 5: fields 58..61 report the display path.  tb bit4=auto
   // time base, bit5=saturated (even R=1024 cannot reach 3 cycles).  fm bit8
   // = the two channels are NOT the same source, so pdiff must be ignored.
   // proto bumped to 6: fields 62..70 report the 4096-point shared-FFT carrier
   // finder.  ffs is the finder state (0 idle 1 search 2 conv 3 settle 4 track
   // 5 lost), fff0/fff1 the interpolated peak frequencies in Hz, ffm0/ffm1 the
   // peak magnitudes (mag>>3, full scale 16*A^2 for A ADC counts), ffid the
   // published frame counter, fflost the lock losses.  In FFT builds the sweep
   // fields (scan/slock/samp/spass) stay 0 - read ffs/ffid instead.
   62:begin text_prefix=" ffs=";text_len=5;number={29'd0,fft_state_hold};width=1;end
   63:begin text_prefix=" ffb0=";text_len=6;number={20'd0,fft_bin0_hold};width=4;end
   64:begin text_prefix=" ffb1=";text_len=6;number={20'd0,fft_bin1_hold};width=4;end
   65:begin text_prefix=" fff0=";text_len=6;number=fft_freq0_hold;width=8;end
   66:begin text_prefix=" fff1=";text_len=6;number=fft_freq1_hold;width=8;end
   67:begin text_prefix=" ffm0=";text_len=6;number={8'd0,fft_mag0_hold};width=8;end
   68:begin text_prefix=" ffm1=";text_len=6;number={8'd0,fft_mag1_hold};width=8;end
   69:begin text_prefix=" ffid=";text_len=6;number=fft_frames_hold;width=10;end
   // " fflost=" is EIGHT characters; text_len was 7, which silently dropped
   // the '=' and printed " fflost0" - a host looking for fflost= found
   // nothing.  text_len must equal the label length exactly: one too many
   // emits NUL padding, one too few truncates the label.
   70:begin text_prefix=" fflost=";text_len=8;number={16'd0,fft_lost_hold};width=5;end
   // proto describes the field set actually published (see fft_present and
   // et_present).  Every buildable set has its OWN number, because the ladder
   // must stay strictly cumulative: 5 = base+display, 6 = 5 + FFT slots 62..70,
   // 7 = 5 + ET slots 72..76, 8 = 6 + 7.  Encoding it as 5 + fft + 2*et keeps
   // that true by construction - with a plain "et?7:(fft?6:5)" an ET build
   // without the finder would claim all of proto 6's fields and a host would
   // look for ffs= on a line that never carries it.
   71:begin text_prefix=" proto=";text_len=7;
            number=LF_FFT?(et_present?32'd12:32'd11):((et_present?32'd9:32'd5)+(fft_present?32'd1:32'd0));
            width=(et_present || LF_FFT)?2:1;is_last=(et_present==0 && LF_FFT==0);end
   72:begin text_prefix=" etm=";text_len=5;number={30'd0,et_locked_hold,et_mode_hold};width=1;end
   73:begin text_prefix=" etb=";text_len=5;number={22'd0,et_bins_hold};width=4;end
   74:begin text_prefix=" etc=";text_len=5;number=et_filled_hold;width=4;end
   75:begin text_prefix=" etw=";text_len=5;number=et_wraps_hold;width=10;end
   76:begin text_prefix=" eth=";text_len=5;number=et_hits_hold;width=10;end
   77:begin text_prefix=" etv=";text_len=5;number=et_ready_hold;width=4;end
   78:begin text_prefix=" ete=";text_len=5;number=et_epoch_hold;width=10;end
   79:begin text_prefix=" etq=";text_len=5;number=et_ref_hold;width=1;end
   80:begin text_prefix=" etmin=";text_len=7;number=128;width=5;end
   81:begin text_prefix=" etrev=";text_len=7;number=2;width=1;is_last=(LF_FFT==0);end
   82:begin text_prefix=" ffv=";text_len=5;number=ffv_hold;width=1;end
   83:begin text_prefix=" ffr0=";text_len=6;number=32'd1<<ffr0_hold;width=5;end
   84:begin text_prefix=" ffr1=";text_len=6;number=32'd1<<ffr1_hold;width=5;end
   85:begin text_prefix=" ffa0=";text_len=6;number=ffa0_hold;width=10;end
   86:begin text_prefix=" ffa1=";text_len=6;number=ffa1_hold;width=10;end
   87:begin text_prefix=" ffrev=";text_len=7;number=2;width=1;is_last=1;end
  endcase
  else if(et_info) begin
   case(field)
    0:begin text_prefix="#ET i=";text_len=6;number=row;width=4;end
    1:begin text_prefix=" n=";text_len=3;number=et_rd_hits;width=5;end
    2:begin text_prefix=" v=";text_len=3;number=et_rd_valid;width=1;is_last=1;end
   endcase
  end
  else begin
   // 48 bits hold the longest label actually used ("wave:", "spass=", "proto=").
   text_prefix=field==0?"wave:":",";text_len=field==0?5:1;
   number=field==0?{{20{snap_rd_ch0[11]}},snap_rd_ch0}:{{20{snap_rd_ch1[11]}},snap_rd_ch1};
   width=4;is_signed=1;is_last=field==1;
  end
 end
 reg [39:0] adjusted;integer k;
 always @*begin
  adjusted=bcd;
  for(k=0;k<10;k=k+1)if(bcd[k*4+:4]>=5)adjusted[k*4+:4]=bcd[k*4+:4]+4'd3;
 end
 wire ready;reg valid;reg [7:0] data;
 uart_tx #(.CLK_FREQ(CLK_FREQ),.BAUD(BAUD)) sender(
  .clk(clk),.rst_n(rst_n),.tx_data(data),.tx_valid(valid),.tx_ready(ready),.tx(tx));
 always @*begin
  valid=0;data=0;
  case(state)
   PREFIX:begin valid=1;data=prefix_shift[79:72];end
   SIGN:begin valid=1;data=negative?8'h2d:8'h2b;end
   DIGITS:begin valid=1;data=8'h30+{4'd0,bcd[digit*4+:4]};end
   NEWLINE:begin valid=1;data=8'h0a;end
   NAN_ROW:begin valid=1;data=NAN_TEXT[(11-nan_index)*8+:8];end
   default:begin end
  endcase
 end
 always @(posedge clk)begin
  if(!rst_n)begin
   as_hold<=0;ar_hold<=0;ae_hold<=0;gr0_hold<=0;gr1_hold<=0;
   df0_hold<=0;df1_hold<=0;au_hold<=0;gs0_hold<=0;gs1_hold<=0;gt0_hold<=0;gt1_hold<=0;fv_hold<=0;
   scan_state_hold<=0;scan_locked_hold<=0;scan_best_hold<=0;scan_pass_hold<=0;
   bw_gear_hold<=0;bw_settling_hold<=0;bw_changes_hold<=0;bw_phase_step_hold<=0;
   aa_hold<=0;tbauto_hold<=0;tbm_hold<=0;tbsat_hold<=0;fm_hold<=0;fmd_hold<=0;
   fft_state_hold<=0;fft_bin0_hold<=0;fft_bin1_hold<=0;
   fft_freq0_hold<=0;fft_freq1_hold<=0;fft_frames_hold<=0;
   fft_mag0_hold<=0;fft_mag1_hold<=0;fft_lost_hold<=0;
   et_mode_hold<=0;et_locked_hold<=0;et_bins_hold<=0;et_filled_hold<=0;
   et_wraps_hold<=0;et_hits_hold<=0;
   et_ready_hold<=0;et_epoch_hold<=0;et_ref_hold<=0;et_info<=0;nan_index<=0;
   ffv_hold<=0;ffr0_hold<=0;ffr1_hold<=0;ffa0_hold<=0;ffa1_hold<=0;
   state<=IDLE;disp_busy<=0;meta<=0;field<=0;frame_hold<=0;trig_hold<=0;
   decim_hold<=0;filt_hold<=0;auto_hold<=0;fault_hold<=0;interval_ps<=0;
   amp0_hold<=0;amp1_hold<=0;ph0_hold<=0;ph1_hold<=0;pdiff_hold<=0;
   present_hold<=0;ratio_valid_hold<=0;mid_hold<=0;age_hold<=0;
   ratio_hold<=0;lflags_hold<=0;lgap_hold<=0;ldrop_hold<=0;
   ldet_hold<=0;lqual_hold<=0;ref_fcw_hold<=0;cfg_decim_hold<=0;cfg_integ_hold<=0;
   freq0_hold<=0;freq1_hold<=0;freq_id_hold<=0;vpp0_hold<=0;vpp1_hold<=0;rms0_hold<=0;
   cal_off0_hold<=0;cal_off1_hold<=0;cal_valid_hold<=0;cal_clip_hold<=0;
   prefix_shift<=0;prefix_left<=0;magnitude<=0;bcd<=0;bits_left<=0;
   digit<=0;signed_value<=0;negative<=0;last_field<=0;row<=0;snap_rd_addr<=0;
  end else case(state)
   IDLE:if(snap_ready)begin
    disp_busy<=1;frame_hold<=snap_frame_id;trig_hold<=snap_trig_pos;
    // In ET mode there is no real-time frame behind this META line, so the
    // display-path metadata is neutralised instead of latched from a stale
    // snapshot: decim=0 (r=1), dt_ps=0 (no uniform sample interval - the ET
    // rows are phase bins), t/F/A/E zeroed.  The measurement fields keep
    // their normal meaning; etm says the wave rows are ET averages.
    decim_hold<= et_mode ? 4'd0 : snap_decim_log2;
    filt_hold<= et_mode ? 1'b0 : snap_filtered;
    auto_hold<= et_mode ? 1'b0 : snap_auto_trigger;
    fault_hold<= et_mode ? 2'd0 : snap_fault;
    trig_hold<= et_mode ? {DISP_AW{1'b0}} : snap_trig_pos;
    interval_ps<= et_mode ? 32'd0 : interval_for_mode(snap_decim_log2);
    amp0_hold<=amp0_q12;amp1_hold<=amp1_q12;
    ph0_hold<=phase0_turn;ph1_hold<=phase1_turn;pdiff_hold<=phase_diff_turn;
    present_hold<=measurement_present;ratio_valid_hold<=ratio_valid;
    mid_hold<=measurement_id;age_hold<=measurement_age;
    ratio_hold<=ratio_q16;lflags_hold<=lockin_flags;
    lgap_hold<=lockin_gap;ldrop_hold<=lockin_drop;
    ldet_hold<=lockin_detected;lqual_hold<=lockin_qualified;
    ref_fcw_hold<=ref_fcw;cfg_decim_hold<=cfg_decim;cfg_integ_hold<=cfg_integration;
    freq0_hold<=freq0_hz;freq1_hold<=freq1_hz;freq_id_hold<=freq_result_id;
    vpp0_hold<=vpp0;vpp1_hold<=vpp1;rms0_hold<=rms0;
    cal_off0_hold<=cal_off0;cal_off1_hold<=cal_off1;
    cal_valid_hold<=cal_valid;cal_clip_hold<=cal_clip;
    as_hold<=afc_state;ar_hold<=afc_ready;ae_hold<=afc_reason;
    df0_hold<=afc_error0;df1_hold<=afc_error1;au_hold<=afc_updates;
    gr0_hold<=guard_reason0;gr1_hold<=guard_reason1;gs0_hold<=guard_step0;gs1_hold<=guard_step1;
    gt0_hold<=guard_threshold0;gt1_hold<=guard_threshold1;fv_hold<=frequency_valid;
    scan_state_hold<=scan_state;scan_locked_hold<=scan_locked;
    scan_best_hold<=scan_best_amp;scan_pass_hold<=scan_passes;
    bw_gear_hold<=bw_gear;bw_settling_hold<=bw_settling;
    bw_changes_hold<=bw_changes;bw_phase_step_hold<=bw_phase_step;
    aa_hold<=aa_order;tbauto_hold<=tb_auto;tbm_hold<=tb_m;tbsat_hold<=tb_sat;
    fm_hold<=ch_mismatch;
    fmd_hold<=(ch_delta[15:4]>8'hff)?8'hff:ch_delta[15:4];
    fft_state_hold<=fft_state;fft_bin0_hold<=fft_bin0;fft_bin1_hold<=fft_bin1;
    fft_freq0_hold<=fft_freq0;fft_freq1_hold<=fft_freq1;
    fft_mag0_hold<=fft_mag0;fft_mag1_hold<=fft_mag1;
    fft_frames_hold<=fft_frames;fft_lost_hold<=fft_lost;
    et_mode_hold<=et_mode;et_locked_hold<=et_locked;
    et_bins_hold<=et_bins;et_filled_hold<=et_filled;
    et_wraps_hold<=et_wraps;et_hits_hold<=et_hits;
    et_ready_hold<=et_ready_bins;et_epoch_hold<=et_epoch;et_ref_hold<=et_reference_ok;
    et_info<=0;
    ffv_hold<=fft_valid_channels;ffr0_hold<=fft_rate0;ffr1_hold<=fft_rate1;ffa0_hold<=fft_age0;ffa1_hold<=fft_age1;
    meta<=1;field<=0;row<=0;state<=PREP;
   end
   PREP:begin
    // Left-align the label so the first character lands in prefix_shift[79:72].
    // 80 bits = 10 characters and the shift amount is (10-text_len)*8.
    //
    // THE WIDTH IS LOAD-BEARING, DO NOT "OPTIMISE" IT DOWN.
    // text_prefix is the destination of a plain procedural assignment, so a
    // literal wider than the destination is silently TRUNCATED FROM THE TOP.
    // The longest label in use is " ref_fcw=" / " r=" / " decim=" style at 9
    // characters, which needs the full 72 bits; 64 bits could not hold it and
    // produced "\0\0...660764" (nine NULs) in the live capture.  4 characters
    // ("#META f=" is 8, "spass=" 6, "wave:" 5) all fit, but there is no smaller
    // safe power of two: 64 truncates, so 80 stays.
    if(!meta && !et_info && et_mode_hold && !et_rd_valid)begin
      nan_index<=0;state<=NAN_ROW;
    end else begin
    prefix_shift<=text_prefix<<((10-text_len)*8);prefix_left<=text_len;
    negative<=is_signed && number[31];signed_value<=is_signed;last_field<=is_last;
    magnitude<=is_signed && number[31]?-number:number;
    digit<=width-1'b1;bcd<=0;bits_left<=32;state<=BCD;
    end
   end
   BCD:begin
    bcd<={adjusted[38:0],magnitude[31]};magnitude<={magnitude[30:0],1'b0};
    bits_left<=bits_left-1'b1;
    // A zero-length label has no prefix bytes to send, so skip PREFIX
    // entirely.  Without this a missing case entry drove prefix_left to 0,
    // the 4-bit counter wrapped 0->15->...->1 and the state emitted 16 NUL
    // bytes before the digits.
    if(bits_left==1)state<=(text_len==0)?(signed_value?SIGN:DIGITS):PREFIX;
   end
   // prefix_left<=1, not ==1: with ==1 a text_len of 0 wrapped the 4-bit
   // counter (0-1 = 15) and never matched.  For every real label (>=1) the
   // two conditions are identical, so this changes nothing that works.
   PREFIX:if(ready)begin
    prefix_shift<=prefix_shift<<8;prefix_left<=prefix_left-1'b1;
    if(prefix_left<=1)state<=signed_value?SIGN:DIGITS;
   end
   SIGN:if(ready)state<=DIGITS;
   DIGITS:if(ready)begin
    if(digit!=0)digit<=digit-1'b1;
    else if(last_field)state<=NEWLINE;
    // Fields 62..70 exist only in an FFT build.  Skipping them is what makes
    // "proto=5" mean anything: publishing them as zeros would put " ffs=0" on
    // a line whose reader would take that as "finder idle" when no finder was
    // ever built.  The counter must never walk an unallocated slot, so this
    // jump is the only permitted gap and it is parameter-guarded.
    else begin field<=(field==7'd61 && !fft_present)?7'd71:((field==7'd71 && !et_present && LF_FFT)?7'd82:field+1'b1);state<=PREP;end
   end
   NEWLINE:if(ready)begin
    if(meta)begin meta<=0;row<=0;state<=RD_ADDR;end
    else if(et_info)begin et_info<=0;field<=0;state<=PREP;end
    else if(row==row_last)state<=DRAIN;
    else begin row<=row+1'b1;state<=RD_ADDR;end
   end
   RD_ADDR:begin snap_rd_addr<=row;state<=RD_WAIT;end
   RD_WAIT:begin field<=0;et_info<=et_mode_hold;state<=PREP;end
   NAN_ROW:if(ready)begin
    if(nan_index==11)state<=NEWLINE;
    else nan_index<=nan_index+1'b1;
   end
   DRAIN:if(ready)begin disp_busy<=0;state<=IDLE;end
   default:begin state<=IDLE;disp_busy<=0;end
  endcase
 end
endmodule
`default_nettype wire
