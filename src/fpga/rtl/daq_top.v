`timescale 1ns/1ps
`default_nettype none
// Base project (steps 1-2):
//   Step 1: AD9238 dual-channel capture -> independent trigger RAM -> VOFA.
//   Step 2: full-rate dual-channel lock-in on the SAME ADC data:
//           shared reference NCO + shared integration window -> amplitude,
//           amplitude ratio (CH1/CH0), phase difference, overflow status.
// No CNN, THD or on-board display in this build.  The optional 4096-point
// shared-FFT carrier finder (FFT_ENABLE) replaces the freq_scan_ctrl sweep:
// the FFT finds the frequency, AFC + lock-in do the precision amplitude/phase.
// Keep the previously verified ADC pins and 300-degree forwarded clock phase.
module daq_top #(
 parameter integer DIGITAL_CHANNELS=4,STARTUP_CYCLES=500000,
 parameter integer OFFSET_BINARY=1,ADC_FS_HZ=65000000,UART_BAUD=921600,
 parameter integer DISP_AW=10,
 // Compile-time diagnostic option. Disable only the independent AB6 text
 // renderer; ADC/lock-in/FFT and the AB5 Linux transport keep their precision.
 // Default retains the existing bench/debug interface.
 parameter integer COM9_TEXT_ENABLE=1,
 // RK3506 display payload only. Does not alter ADC rate or FFT length.
 parameter integer BINARY_POINTS=1024,
 parameter integer BINARY_BAUD=2000000,
 // Extra idle bit is optional; packet bytes/CRC/sample precision are unchanged.
 // An 8N1 receiver accepts the additional high stop bit as idle time.
 parameter integer BINARY_STOP_BITS=1,
 // ---- lock-in configuration ----
 parameter integer AFC_ENABLE=1,
 parameter [31:0] REF_FREQ_HZ=32'd30000000,   // reference frequency for lock-in
// ---- ONE-BITSTREAM FULL-BAND ACQUISITION --------------------------------
// REF_FREQ_HZ used to be baked into cfg_fcw, so every carrier frequency
// needed its own build.  With SCAN_ENABLE the reference is searched for at
// run time by freq_scan_ctrl (coarse sweep -> fine sweep -> lock), and the
// measurement-bandwidth controller then picks the CIC decimation gear from
// signal stability (NOT from the carrier - see meas_bw_ctrl).
// ONE bitstream then SWEEPS the design-target band ~200 Hz .. 32.4 MHz
// (F_MIN_FCW .. F_MAX_FCW in freq_scan_ctrl).  That is the range the search
// VISITS; it is NOT a validated measurement band - no on-board data exists at
// either edge and the amplitude/guard limits there are untested.
//   SCAN_ENABLE=0  -> previous behaviour: cfg_fcw = REF_FCW, gear = DECIM_SEL
// NOTE: the RTL default here is 1, but scripts/build.tcl defaults the env var
// to 0 so the legacy per-frequency builds keep their names.  A bare
// "vivado -source scripts/build.tcl" therefore builds the NON-fullband
// variant; always pass SCAN_ENABLE=1 explicitly.
 parameter integer SCAN_ENABLE=0,
 // ---- 4096-point shared-FFT carrier finder --------------------------------
 // FFT_ENABLE=1 replaces the freq_scan_ctrl sweep with a Hann-windowed
 // 4096-point FFT (one shared engine, CH0/CH1 time-multiplexed, ~5.5 ms per
 // full-band frame).  The FFT only finds the frequency; local_afc_2ch tracks
 // it finely and the lock-in core measures amplitude/phase.  In practice
 // mutually exclusive with SCAN_ENABLE: when both are 1 the FFT finder wins
 // and the sweep is disabled.
 parameter integer FFT_ENABLE=1,
 parameter integer LOW_FFT_ENABLE=1,
 // ---- WHO OWNS THE INTEGRATION WINDOW -------------------------------------
 // The CIC sits AFTER the mixer, so the decimation ratio does not have to
 // cover the carrier; it only sets the window length (T_win = N*R/fs_in) and
 // therefore the measurement bandwidth.  The gear must be chosen from how
 // STABLE the signal is, never from the carrier frequency -- picking it from
 // f_ref pinned every carrier above 100 kHz to the 1 ms gear forever, so a
 // weak 30 MHz tone could never reach the narrow band it needs.
 //   BW_MODE=0  fixed DECIM_SEL, no automation (safest A/B reference)
 //   BW_MODE=1  gear from carrier frequency  (auto_rate_ctrl, legacy)
 //   BW_MODE=2  gear from stability          (meas_bw_ctrl, default)
 parameter integer BW_MODE=2,
 // Ceiling for automatic narrowing: 1 = stop at 4 ms (use if a high carrier
 // aliases into the 23 Hz band), 2 = allow the 64.5 ms precise gear.
 parameter integer BW_MAX_GEAR=2,
 parameter integer AUTO_RATE_EN=(BW_MODE==1),
 // ---- AUTOMATIC DISPLAY TIME BASE ----------------------------------------
 // Independent of BW_MODE: it decides how many cycles fill the 1024-point
 // frame, never how long the lock-in integrates.  sw[2] up = automatic,
 // sw[2] down = manual (KEY2 cycles m = 0,3,4,...,10).
 // Mode 2 boots automatic with sw[2] down; up is manual override.
 // Mode 1 retains the earlier up=automatic switch convention.
 parameter integer AUTO_TIMEBASE=2,
 // Display controller and capture share the same physical decimation ceiling.
 parameter integer MAX_DISP_M=15,          // R <= 32768 (~516 ms / 1024 points)
 parameter integer AA_ORDER=3,             // display decimator order, META field aa.
 // Kept ONLY for META/bitstream compatibility: the display path is a boxcar
 // average, so "order" has no filtering meaning.  Do not read this as an
 // anti-alias filter order.  See reports/decimator_findings_20261019.md.
 // Two channels that are not the same source must not publish a phase
 // difference.  See rtl/ch_freq_guard.v.
 parameter integer CH_MISMATCH_EN=1,
 // Integration window = 2^N decimated samples (decimated rate = fs/16).
 // Default 12 -> 4096 samples = 1.008 ms ~= 10 reference cycles at 10 kHz.
 // A coherent measurement needs the window to span many reference cycles:
 // with the old default N=8 the window was only 0.63 cycles at 10 kHz and
 // spectral leakage made the measured ratio swing 0.28..0.88 and the phase
 // 34..96 degrees for an ideal 0.5 / 60 degree input. Raise N for a smoother
 // (but slower, ~1/(16*2^N/fs) publish rate) reading; lower it for speed.
 parameter [4:0]  INTEGRATION_LOG2=5'd12,   // 4096 post-decimation points (~1.008 ms at R=16); tuned for default 10 kHz
 parameter [1:0]  DECIM_SEL=2'd0,          // lock-in CIC decimation select
 parameter integer LOCKIN_ENABLE=1,
 // ---- EQUIVALENT-TIME (ET) DISPLAY -----------------------------------------
 // sw[0] up selects the ET display mode: the wave: rows become PHASE-BINNED
 // AVERAGES of one signal period, reassembled from many physical periods by
 // binning every full-rate sample on the locked NCO phase (rtl/et_display.v).
 // The AFC/lock-in measurement chain is NOT touched - this is a display mode
 // for stable, repetitive waveforms only; it proves nothing about single-shot
 // capture and cannot recover front-end-filtered harmonics.
 // Requires LOCKIN_ENABLE=1 (the NCO is the phase reference).
 //   ET_AW        phase bins per period (2^ET_AW; 6 -> 64 bins)
 //   ET_AVG_SHIFT EMA weight 1/2^S (~2^(S+1)-sample equivalent averaging)
 //   ET_FRAME_CLKS frame request period (default 2^21 ~ 32.2 ms)
 // Physical phase coverage is bounded by fs/fi = p/q (p positions per q
 // periods) and is reported per frame by the META etc field.
 parameter integer ET_ENABLE=1,
 parameter integer ET_AW=6,
 parameter integer ET_AVG_SHIFT=4,
 parameter integer ET_FRAME_CLKS=2097152,
 // ---- zero-bias calibration (per-channel, signed 12-bit, subtracted after capture) ----
 parameter signed [11:0] ZERO_OFFSET_CH0 = 0,
 parameter signed [11:0] ZERO_OFFSET_CH1 = 0,
 // Weak-signal amplitude floor fed to the lock-in validity guard.
 // 2048 is a PRELIMINARY value (see lockin_measurement.v): it must be validated
 // by zero-input false-positive AND >=20 mVpp detection measurements.
 parameter [23:0] LOCKIN_MIN_AMP_Q12 = 24'd2048,
 // ---- DDC demodulation chain (FPGA+Linux digital modulation analyzer) ----
 parameter integer DDC_ENABLE=1
)(
 input wire clk_50m,rst_n,input wire [11:0] adc_d1,adc_d0,
 input wire adc_otr_b,adc_otr,
 output wire adc_clk_b,adc_oeb_b,adc_pwdn_b,adc_clk,adc_oeb,adc_pwdn,
 input wire [DIGITAL_CHANNELS-1:0] digital_in,
 input wire [3:0] key_n,input wire [2:0] sw,
 output wire [3:0] led,output wire uart_tx,
 // V32CAP3 binary snapshot port -> RK3506 (Linux) UART RX.  Physically separate
 // from uart_tx (AB6 -> CH340E -> PC): the text port carries 1024-point display
 // frames, this one carries 1024-point int16 pairs that the Linux renderer
 // consumes.  Without this port the Linux display receives nothing at all.
 // 2026-10-23: single physical pair restored to TX=AB5, RX=AA5.
 // Frame-boundary scheduling fixes are retained; no mirror or RX arbitration.
 input wire v32_rx,
 output wire v32_tx
);
 // reference frequency word = f * 2^32 / fs  (constant-folded)
 // Widen the operand before shifting; based literals cannot contain variable names.
 function [31:0] freq_to_fcw;
  input [31:0] f_hz;
  freq_to_fcw = ({32'd0, f_hz} << 32) / ADC_FS_HZ;
 endfunction
 localparam [31:0] REF_FCW = freq_to_fcw(REF_FREQ_HZ);
 localparam [1:0]  AA_ORDER4 = AA_ORDER;

 wire clk_sys,clk_adc_fwd,locked,sys_rst_n;
 clocking_sys_7series clocks(.clk_50m(clk_50m),.arst_n(rst_n),
  .clk_sys(clk_sys),.clk_adc_fwd(clk_adc_fwd),.locked(locked));
 reset_sync sys_reset(.clk(clk_sys),.arst_n(rst_n&&locked),.rst_n(sys_rst_n));
 assign adc_oeb=0;assign adc_oeb_b=0;assign adc_pwdn=0;assign adc_pwdn_b=0;
 ODDR #(.DDR_CLK_EDGE("OPPOSITE_EDGE"),.INIT(0),.SRTYPE("SYNC")) adc_clk_fwd(
  .Q(adc_clk),.C(clk_adc_fwd),.CE(1'b1),.D1(1'b1),.D2(1'b0),.R(1'b0),.S(1'b0));
 ODDR #(.DDR_CLK_EDGE("OPPOSITE_EDGE"),.INIT(0),.SRTYPE("SYNC")) adc_clk_b_fwd(
  .Q(adc_clk_b),.C(clk_adc_fwd),.CE(1'b1),.D1(1'b1),.D2(1'b0),.R(1'b0),.S(1'b0));
 wire raw_valid,adc_ready;wire [47:0] raw_samples;wire [3:0] raw_otr;
 ad9238_capture #(.CHANNELS(2),.STARTUP_CYCLES(STARTUP_CYCLES),
  .OFFSET_BINARY(OFFSET_BINARY),.USE_EXT_CLK(1),.PARALLEL_PORTS(1)) adc_capture(
  .clk(clk_sys),.rst_n(sys_rst_n),.adc_clk_ext(clk_sys),.mmcm_locked(locked),
  .adc_d0(adc_d0),.adc_d1(adc_d1),.adc_d2(12'd0),.adc_d3(12'd0),
  .adc_otr({2'd0,adc_otr_b,adc_otr}),.adc_clk(),.samples(raw_samples),
  .sample_otr(raw_otr),.sample_valid(raw_valid),.ready(adc_ready));
 wire freeze,trig_src,falling;wire [3:0] timebase_log2;wire tb_auto_sw;wire et_mode_sw;
 basic_controls #(.KEY_TIMEBASE(1),.MAX_TIMEBASE(MAX_DISP_M)) controls(.clk(clk_sys),.rst_n(sys_rst_n),.key_n(key_n),.sw(sw),
  .freeze(freeze),.trig_src(trig_src),.falling(falling),.timebase_log2(timebase_log2),
  .tb_auto(tb_auto_sw),.sw0(et_mode_sw));
 // ---- automatic display time base ---------------------------------------
 // Fed with the TRACKED frequency (searched + AFC corrected), so the window
 // density follows what is actually being measured, not what was compiled.
 wire [3:0] tb_m;wire tb_active,tb_sat;wire [7:0] tb_changes;
 // FFT frequency validity is per channel and independent of dual-lane AFC.
 wire [1:0] fft_valid_channels,fft_update;
 wire use_fft_timebase=(FFT_ENABLE!=0) && (LOW_FFT_ENABLE!=0);

 reg demod_en;reg [3:0] demod_mode;reg [31:0] ddc_fcw;reg iq_ch;reg [31:0] demod_cfg_id;reg [1:0] iq_rate;
 wire rx_demod_en;wire [3:0] rx_demod_mode;wire [31:0] rx_ddc_fcw;wire rx_iq_ch;wire [31:0] rx_cfg_id;wire [1:0] rx_iq_rate;
 wire [2:0] parallel_owner;
 wire config_pending=rx_cfg_id!=demod_cfg_id;
 always @(posedge clk_sys)begin
  if(!sys_rst_n)begin demod_en<=0;demod_mode<=0;ddc_fcw<=32'd66076420;iq_ch<=0;demod_cfg_id<=0;iq_rate<=1;end
  else if(parallel_owner==0 && config_pending)begin
   demod_en<=rx_demod_en;demod_mode<=rx_demod_mode;ddc_fcw<=rx_ddc_fcw;
   iq_ch<=rx_iq_ch;demod_cfg_id<=rx_cfg_id;iq_rate<=rx_iq_rate;
  end
 end

 wire [1:0] wave_focus;
 wire prefer_raw_ch=wave_focus<2?wave_focus[0]:((DDC_ENABLE!=0 && demod_en)?!iq_ch:1'b0);
 wire chosen_raw_ch=fft_valid_channels[prefer_raw_ch]?prefer_raw_ch:!prefer_raw_ch;
 wire [31:0] tb_frequency=use_fft_timebase?(chosen_raw_ch?fft_freq1:fft_freq0):tracked_hz;
 wire tb_frequency_valid=use_fft_timebase?(|fft_valid_channels):freq_valid_raw[0];
 wire tb_frequency_update=use_fft_timebase?(chosen_raw_ch?fft_update[1]:fft_update[0]):lockin_valid;
 wire want_auto_tb=(AUTO_TIMEBASE==2)?!tb_auto_sw:tb_auto_sw;
 auto_timebase #(.FS_HZ(ADC_FS_HZ),.DISP_N(1<<DISP_AW),.MAX_M(MAX_DISP_M))
  u_tbase(.clk(clk_sys),.rst_n(sys_rst_n),
   .enable((AUTO_TIMEBASE!=0) && want_auto_tb),
   .freq_hz(tb_frequency),.freq_valid(tb_frequency_valid),.freq_update(tb_frequency_update),
   .m(tb_m),.auto_active(tb_active),.saturated(tb_sat),.changes(tb_changes));
 wire [3:0] disp_timebase = ((AUTO_TIMEBASE!=0) && want_auto_tb) ? tb_m : timebase_log2;
 wire disp_tb_auto = ((AUTO_TIMEBASE!=0) && want_auto_tb);
 wire snap_ready,filtered,auto_trigger,display_busy;wire [15:0] frame_id;
 wire [31:0] interval_ns,errors;wire [DISP_AW-1:0] trig_pos,rd_addr;
 wire [1:0] fault;wire [3:0] decim;wire [63:0] first_index;
 wire signed [11:0] d0,d1;
 localparam [DISP_AW-1:0] PRETRIG=(1<<DISP_AW)/4;
 // Display decimation is supported at m=0..MAX_DISP_M in both blocks.
 acq_trigger_buffer #(.DISP_AW(DISP_AW),.MAX_DECIM_LOG2(MAX_DISP_M),.CLK_FREQ(ADC_FS_HZ)) u_acq(
  .clk(clk_sys),.rst_n(sys_rst_n),.sample_valid(raw_valid),
  .ch0(raw_samples[11:0]),.ch1(raw_samples[23:12]),.fault(raw_otr[1:0]),
  .trig_level(12'sd0),.trig_mode({1'b0,falling}),.trig_src(trig_src),
  .pretrig(PRETRIG),.timebase_log2(disp_timebase),.freeze(freeze),
  .disp_busy(display_busy),.snap_rd_addr(rd_addr),.snap_rd_ch0(d0),.snap_rd_ch1(d1),
  .snap_ready(snap_ready),.snap_frame_id(frame_id),.snap_sample_interval_ns(interval_ns),
  .snap_trig_pos(trig_pos),.snap_filtered(filtered),.snap_fault(fault),
  .snap_decim_log2(decim),.snap_auto_trigger(auto_trigger),
  .snap_first_index(first_index),.capture_errors(errors));

 // ---- V32CAP3 binary snapshot port (AB5 -> RK3506 / Linux display) ----
 // This is the ONLY path the Linux renderer has: 4096 points x 2ch x int16
 // + header/CRC = 16428 B per frame, ~5.6 fps at 921600.  The text port on
 // uart_tx (AB6) is a different, independent link to the PC and carries only
 // DISP_N=1024 display points, so it cannot stand in for this one.
 // raw_samples[11:0]=CH0, [23:12]=CH1, already two's complement (ad9238_capture
 // converts OFFSET_BINARY), which is exactly the 12-bit signed format the frame
 // expects.  FRAME_PERIOD=0: capture the next frame as soon as the previous one
 // has been sent, so the frame rate is bounded by the UART (about 5.6 fps).
 //
 // ---- 自动采样率（波形帧） ------------------------------------------------
 // 波形帧原来直接吃 raw_samples（R=1），1024 点只有 15.75 us，低于约 63.5 kHz
 // 的信号在 Linux 端连一个完整周期都看不到。这里按实测频率自动选抽取比，
 // 让一帧覆盖约 TARGET_CYCLES=8 个周期，并把真实的 R 写进帧头 offset 20；
 // Linux 侧按 fs/R 换算等效采样率，接收代码不需要改。
 //
 // 复用 auto_timebase：它的 DISP_N 取 BINARY_POINTS(=1024)，与一帧点数相同，
 // 算法（log2 最近幂次 + DWELL 滞回 + 饱和标记）已经在显示链上验证过。
 // 这里的实例 enable 只看编译期开关，不受面板手动时基影响 —— 波形帧始终
 // 按信号自适应，手动时基只改 LCD 显示。AUTO_TIMEBASE=0 时 m 恒 0，即 R=1，
 // 行为退回到原来的原始流。
 //
 // 注意 start_index 语义不变：它是自由运行的 ADC 时钟计数，与抽取比无关，
 // 所以锁相帧的关联判据（snapshot_tick == start_index）继续成立。
 wire [991:0] linux_meas_body;
 wire spec_valid;wire [11:0] spec_bin;wire [32:0] spec_power;
 wire [31:0] spec_fs_hz;
 wire harmonic_enable,harmonic_channel,harmonic_scan_valid,harmonic_scan_saturated;
 wire [31:0] harmonic_cfg,harmonic_reference,harmonic_scan_age,harmonic_completed;
 wire [1:0] harmonic_scan_channel;
 wire [3:0] harmonic_scan_rate;
 wire harmonic_ready,harmonic_busy,harmonic_tx,harmonic_start;
 wire wave_tx,spec_tx,wave_busy,spec_busy,spec_ready,link_pause,spec_start;
 wire [31:0] wave_completed;
 wire [3:0] command_m;wire command_override;

 ab5_uart_commands #(.CLK_FREQ(ADC_FS_HZ),.BAUD(BINARY_BAUD),.DUAL_RX(0)) u_ab5_rx(
  .clk(clk_sys),.rst_n(sys_rst_n),.rx(v32_rx),.rx_copy(1'b1),.decim_log2(command_m),.override_auto(command_override),
  .demod_en(rx_demod_en),.demod_mode(rx_demod_mode),.ddc_fcw(rx_ddc_fcw),.iq_ch(rx_iq_ch),.cfg_id(rx_cfg_id),.wave_focus(wave_focus),.iq_rate(rx_iq_rate),
  .harmonic_en(harmonic_enable),.harmonic_ch(harmonic_channel),.harmonic_fcw(harmonic_reference),.harmonic_cfg(harmonic_cfg));
 harmonic_analysis_link #(.CLK_FREQ(ADC_FS_HZ),.BAUD(BINARY_BAUD),.STOP_BITS(BINARY_STOP_BITS),.FPGA_THD(0)) u_harmonic(
  .clk(clk_sys),.rst_n(sys_rst_n),.enable(harmonic_enable && FFT_ENABLE!=0 && LOW_FFT_ENABLE!=0),
  .channel(harmonic_channel),.cfg_id(harmonic_cfg),.reference_fcw(harmonic_reference),.supported_rates(5'b11111),
  .scan_valid(harmonic_scan_valid),.scan_saturated(harmonic_scan_saturated),.scan_channel(harmonic_scan_channel),
  .scan_window(2'd0),.scan_bin(spec_bin),.scan_power(spec_power),.scan_rate_log2(harmonic_scan_rate),.scan_age_ticks(harmonic_scan_age),
  .send_start(harmonic_start),.ready(harmonic_ready),.busy(harmonic_busy),.tx(harmonic_tx),.completed(harmonic_completed));
 wire demod_active=(DDC_ENABLE!=0) && demod_en;
 wire wave_capture_ready,wave_grant,iq_grant,iq_frame_ready;
 wire iq_busy;wire [31:0] iq_completed;wire [11:0] iq_level;
 wire [31:0] spec_completed;
 wire link_pause_eff=1'b0;
 parallel_frame_arbiter #(.CLK_FREQ(ADC_FS_HZ),.BAUD(BINARY_BAUD)) u_parallel_link(
  .clk(clk_sys),.rst_n(sys_rst_n),.analysis_enable(demod_active),
  .wave_ready(wave_capture_ready),.wave_busy(wave_busy),.wave_completed(wave_completed),.wave_tx(wave_tx),
  .analysis_ready(iq_frame_ready && !config_pending),.analysis_busy(iq_busy),.analysis_completed(iq_completed),.analysis_tx(iq_tx),
  .spec_ready(spec_ready),.spec_busy(spec_busy),.spec_completed(spec_completed),.spec_tx(spec_tx),
  .harmonic_ready(harmonic_ready),.harmonic_busy(harmonic_busy),.harmonic_tx(harmonic_tx),.harmonic_completed(harmonic_completed),.harmonic_start(harmonic_start),
  .wave_grant(wave_grant),.analysis_grant(iq_grant),.spec_start(spec_start),.tx(v32_tx),
  .abort_count(),.owner_debug(parallel_owner));
 // Scan starts at bin 1; the finder deliberately excludes DC. Transmit DC=0.
 reg scan_active;
 always @(posedge clk_sys) begin
  if(!sys_rst_n || spec_ready || spec_busy || !raw_valid) scan_active<=0;
  else if(spec_valid && spec_bin==1) scan_active<=1;
 end
 v32spec_tx #(.MANUAL_START(1),.RUNTIME_FS(1),.FS_HZ(ADC_FS_HZ),.CLK_FREQ(ADC_FS_HZ),.BAUD(BINARY_BAUD),.UART_STOP_BITS(BINARY_STOP_BITS)) u_ab5_spec(
  .clk(clk_sys),.rst_n(sys_rst_n),
  .wr_en(spec_valid && (scan_active || spec_bin==1)),
  .wr_bin(spec_bin==2048?12'd0:spec_bin),.wr_mag(spec_bin==2048?33'd0:spec_power),
  .scan_done(spec_valid && scan_active && spec_bin==2048),.send_start(spec_start),
  .snapshot_fs_hz(spec_fs_hz),
  .snapshot_ready(spec_ready),.wave_frame_id(wave_completed),.window_sel(2'd0),
  .tx(spec_tx),.busy(spec_busy),.completed(spec_completed),.result_id());
 wire wave_auto_en=(AUTO_TIMEBASE!=0);
 wire [3:0] wave_m;wire wave_tb_active,wave_tb_sat;wire [7:0] wave_tb_changes;
 auto_timebase #(.FS_HZ(ADC_FS_HZ),.DISP_N(BINARY_POINTS),.MAX_M(MAX_DISP_M))
  u_wave_tb(.clk(clk_sys),.rst_n(sys_rst_n),
   .enable(wave_auto_en),
   .freq_hz(tb_frequency),.freq_valid(tb_frequency_valid),.freq_update(tb_frequency_update),
   .m(wave_m),.auto_active(wave_tb_active),.saturated(wave_tb_sat),.changes(wave_tb_changes));
 reg [3:0] wave_applied_m;
 always @(posedge clk_sys) begin
  if(!sys_rst_n) wave_applied_m<=0;
  else if(!wave_busy) wave_applied_m<=command_override?command_m:wave_m;
 end
 wire wave_valid,wave_fault;wire signed [11:0] wave_ch0,wave_ch1;
 wave_decim #(.MAX_LOG2(MAX_DISP_M),.SAMPLE_W(12)) u_wave_decim(
  .clk(clk_sys),.rst_n(sys_rst_n),
  .sample_valid(raw_valid),.otr(raw_otr[1:0]),
  .ch0(raw_samples[11:0]),.ch1(raw_samples[23:12]),
  .decim_log2(wave_applied_m),
  .out_valid(wave_valid),.out0(wave_ch0),.out1(wave_ch1),.out_fault(wave_fault));
 v32cap3_tx #(.POINTS(BINARY_POINTS),.LINK_ENABLE(1),.MEAS_ENABLE(1),.CLK_FREQ(ADC_FS_HZ),.BAUD(BINARY_BAUD),.FS_HZ(ADC_FS_HZ),.UART_STOP_BITS(BINARY_STOP_BITS),
  .FRAME_PERIOD(0)) u_v32cap3(
  .clk(clk_sys),.rst_n(sys_rst_n),.sample_valid(wave_valid),.fault(wave_fault),
  .decim_log2(wave_applied_m),
  .ch0(wave_ch0),.ch1(wave_ch1),
  .send_grant(wave_grant),.capture_ready(wave_capture_ready),.link_pause(link_pause_eff),.meas_body(linux_meas_body),.tx(wave_tx),.busy(wave_busy),.completed(wave_completed),.aborted(),.skipped());

 // ---- DDC demodulation chain (DDC_ENABLE) ---------------------------------
 // FPGA+Linux digital modulation analyzer, stage 1 (FPGA side):
 //   raw CH0/CH1 -> NCO mix (configurable center) -> CIC3 R=1024
 //   -> FIR lowpass /4 (anti-alias) -> IQ frame stream on AB5.
 // Total decimation 4096 -> 15869.1406 sps; I/Q 16-bit each.
 // Mode / center / channel are set from Linux over v32_rx:
 //   "A5 E0 <mode>"  mode: 0=off 1=AM 2=FM 3=OOK/2FSK 4=BPSK/QPSK
 //   "A5 E1 <hi8>" "A5 E2 <lo8>"  DDC center FCW
 //   "A5 E3 <ch>"   channel: 0=CH0 1=CH1
 // Reference: red-pitaya-notes sdr_receiver (ADC -> I/Q DDC -> decimate ->
 // host); sync/demod/measure run on RK3506 with liquid-dsp.
 wire iq_tx;
 generate if(DDC_ENABLE!=0) begin : g_ddc
  reg [31:0] iq_cfg_q;
  always @(posedge clk_sys) if(!sys_rst_n) iq_cfg_q<=0; else iq_cfg_q<=demod_cfg_id;
  wire iq_rst_n = sys_rst_n && demod_en && (iq_cfg_q==demod_cfg_id) &&
                  raw_valid && !(iq_ch ? raw_otr[1] : raw_otr[0]);
  wire ddc_in_valid = raw_valid && iq_rst_n;
  wire [11:0] ddc_src = iq_ch ? raw_samples[23:12] : raw_samples[11:0];
  // 显式符号扩展: 条件表达式结果为无符号 12 位, 直接赋给 16 位有符号会零扩展,
  // 导致 ADC 负半周被当作大正值, DDC 混频错误。
  wire signed [15:0] ddc_in = {{4{ddc_src[11]}}, ddc_src};
  wire signed [15:0] mix_i,mix_q;wire mix_valid;
  wire signed [15:0] cic_i,cic_q;wire cic_valid;
  wire signed [15:0] fir_i,fir_q;wire fir_valid;
  ddc_nco_mixer #(.CLK_FREQ(ADC_FS_HZ),.LUT_AW(10),.DATA_W(16)) u_ddc_nco(
   .clk(clk_sys),.rst_n(iq_rst_n),.in_valid(ddc_in_valid),.in(ddc_in),.fcw(ddc_fcw),
   .out_valid(mix_valid),.i_out(mix_i),.q_out(mix_q));
  ddc_cic_decimator #(.DATA_W(16),.LOG2_R(10)) u_ddc_cic(
   .clk(clk_sys),.rst_n(iq_rst_n),.clear(!demod_en),.in_valid(mix_valid),
   .i_in(mix_i),.q_in(mix_q),.out_valid(cic_valid),.i_out(cic_i),.q_out(cic_q));
  ddc_fir_decimator #(.DATA_W(16),.TAPS(128),.DECIM(4),.RUNTIME_RATE(1)) u_ddc_fir(
   .clk(clk_sys),.rst_n(iq_rst_n),.in_valid(cic_valid),
   .i_in(cic_i),.q_in(cic_q),.rate_sel(iq_rate),.out_valid(fir_valid),.i_out(fir_i),.q_out(fir_q));
  wire [31:0] analysis_word;wire analysis_valid,analysis_fault;wire [7:0] analysis_kind;
  analysis_polar_stream u_selected_analysis(.clk(clk_sys),.rst_n(iq_rst_n),.in_valid(fir_valid),
   .i_in(fir_i),.q_in(fir_q),.mode(demod_mode),.kind(analysis_kind),.out_valid(analysis_valid),
   .out_word(analysis_word),.overrun(analysis_fault));
  analysis_stream_tx #(.POINTS(256),.FIFO_DEPTH(2048),.CLK_FREQ(ADC_FS_HZ),.BAUD(BINARY_BAUD),
                 .FS_HZ(ADC_FS_HZ),.DECIM(4096),.RUNTIME_RATE(1)) u_iq_tx(
   .clk(clk_sys),.rst_n(iq_rst_n),.iq_valid(analysis_valid),.result_word(analysis_word),.result_info({8'd0,4'd0,demod_mode,7'd0,iq_ch,analysis_kind}),.center_fcw(ddc_fcw),.producer_fault(analysis_fault),
   .cfg_id(demod_cfg_id),.rate_sel(iq_rate),.send_grant(iq_grant),.frame_ready(iq_frame_ready),.fifo_level(iq_level),.tx(iq_tx),.busy(iq_busy),.completed(iq_completed),.overflow());
 end else begin : g_no_ddc
  assign iq_tx = 1'b1;
  assign iq_frame_ready=0;assign iq_busy=0;assign iq_completed=0;assign iq_level=0;
 end endgenerate

 // ---- zero-bias calibration: subtract measured per-channel DC offset ----
 // RAW (raw_samples) still drives the displayed wave for inspection; the
 // calibrated values feed the lock-in, frequency meter and window metrics.
 wire signed [12:0] ch0_sub = {raw_samples[11], raw_samples[11:0]}
                  - $signed({ZERO_OFFSET_CH0[11], ZERO_OFFSET_CH0});
 wire signed [12:0] ch1_sub = {raw_samples[23], raw_samples[23:12]}
                  - $signed({ZERO_OFFSET_CH1[11], ZERO_OFFSET_CH1});
 wire ch0_sat = (ch0_sub[12] != ch0_sub[11]);
 wire ch1_sat = (ch1_sub[12] != ch1_sub[11]);
 wire signed [11:0] ch0_cal = ch0_sat ? (ch0_sub[12] ? 12'sh800 : 12'sh7ff) : ch0_sub[11:0];
 wire signed [11:0] ch1_cal = ch1_sat ? (ch1_sub[12] ? 12'sh800 : 12'sh7ff) : ch1_sub[11:0];
 // 1 only when a real (nonzero) offset is baked in; a build with no calibration
 // must not claim cal_valid, otherwise "preset applied" is mistaken for "zeroed".
 wire cal_valid = (ZERO_OFFSET_CH0 != 12'sd0) || (ZERO_OFFSET_CH1 != 12'sd0);
 wire [1:0] cal_clip = {ch1_sat, ch0_sat};

 // ---- Step 2: lock-in measurement on the full-rate ADC data ----
 wire li_adc_valid = raw_valid && !(|raw_otr[1:0]);
 reg lockin_configured;
 always @(posedge clk_sys) begin
  if(!sys_rst_n) lockin_configured<=1'b0; else lockin_configured<=1'b1;
 end
 wire cfg_enable = LOCKIN_ENABLE[0];

 // ---- Run-time reference search (coarse -> fine -> lock) ------------------
 // The sweep reads the RAW Q12 amplitude of each finished window.  It must NOT
 // use detected/qualified: the guard needs ~10 consecutive good windows and the
 // sweep only spends one window per candidate, so those flags are always 0
 // while sweeping.  Sweep results are reported through busy/locked and are not
 // valid measurements.
 wire [31:0] scan_fcw,scan_best_fcw;
 wire [4:0]  scan_integ;
 wire        scan_commit,scan_busy,scan_locked;
 wire [2:0]  scan_state;
 wire [23:0] scan_best_amp;
 wire [15:0] scan_points,scan_passes;
 wire [7:0]  scan_lost;
 // ---- finder selection: 4096-point FFT (new) or sweep (legacy) -----------
// (fft_valid_channels/fft_update declared near auto_timebase)
 wire [3:0] fft_rate0,fft_rate1;
 wire [31:0] fft_age0,fft_age1;
 wire fft_on = (FFT_ENABLE!=0);
 wire [31:0] fft_find_fcw;
 wire        fft_commit,fft_locked,fft_busy;
 wire [2:0]  fft_state;
 wire [31:0] fft_freq0,fft_freq1,fft_frames;
 wire [11:0] fft_bin0,fft_bin1;
 wire [23:0] fft_mag0,fft_mag1;
 wire [15:0] fft_lost;
 wire finder_active = (SCAN_ENABLE!=0) || (FFT_ENABLE!=0);
 wire finder_locked = fft_on ? fft_locked : scan_locked;
 wire finder_busy   = fft_on ? fft_busy   : scan_busy;
 freq_scan_ctrl u_scan(
  .clk(clk_sys),.rst_n(sys_rst_n),.enable(cfg_enable && !fft_on),
  .meas_valid(lockin_valid),.meas_amp0(li_amp0),.meas_amp1(li_amp1),
  .rescan(1'b0),
  .active_fcw(scan_fcw),.active_integ(scan_integ),.commit(scan_commit),
  .busy(scan_busy),.locked(scan_locked),.state(scan_state),
  .best_fcw(scan_best_fcw),.best_amp(scan_best_amp),
  .scan_count(scan_points),.lost_count(scan_lost),.pass_count(scan_passes));

 // ---- Measurement bandwidth controller (stability driven) -----------------
 // Same hardware knob as auto_rate_ctrl (the CIC decimation ratio), but a
 // different decision: narrow while the tone is stable, fall back to the fast
 // gear the instant it steps or drops out.  Never looks at the carrier
 // frequency.  See rtl/meas_bw_ctrl.v for why ripple is NOT part of the test.
 wire [1:0] bw_gear,bw_decim_sel;
 wire [4:0] bw_integ;
 wire bw_commit,bw_settling,bw_valid;
 wire [15:0] bw_changes;
 wire [7:0] bw_good,bw_bad;
 wire [15:0] bw_pstep;
 meas_bw_ctrl #(.INTEGRATION_LOG2(INTEGRATION_LOG2),
                .AMP_MIN(LOCKIN_MIN_AMP_Q12),
                .REQUIRE_PRECISION(AFC_ENABLE!=0),
                .MAX_GEAR(BW_MAX_GEAR)) u_bw_ctrl(
  .clk(clk_sys),.rst_n(sys_rst_n),
  .enable(cfg_enable && (BW_MODE==2) && (!finder_active || finder_locked)),
  .meas_valid(lockin_valid),
  .amp0(li_amp0),.amp1(li_amp1),.phase_diff(li_pdiff),
  .afc_enable((AFC_ENABLE!=0)),.afc_ready(afc_coarse_ready),.precision_ready(afc_precision_ready),
  .gear(bw_gear),.decim_sel(bw_decim_sel),.integration_log2(bw_integ),
  .commit(bw_commit),.settling(bw_settling),.valid(bw_valid),
  .change_count(bw_changes),.good_windows(bw_good),.bad_windows(bw_bad),
  .last_phase_step(bw_pstep),.settle_ticks());

 // ---- Automatic decimation gear (legacy: gear from carrier frequency) -----
 // Kept only as an A/B reference (BW_MODE=1).  ADC/processing clock stays
 // 65 MHz in every gear; only the CIC decimation ratio changes, which is what
 // sets the integration window (T_win=N*R/fs_in).
 wire [1:0] rate_decim_sel;
 wire [4:0] rate_integ;
 wire rate_commit,rate_settling,rate_valid;
 wire [1:0] rate_gear;
 wire [15:0] rate_changes;
 auto_rate_ctrl #(.INTEGRATION_LOG2(INTEGRATION_LOG2)) u_rate_ctrl(
  .clk(clk_sys),.rst_n(sys_rst_n),
  .enable(cfg_enable && (AUTO_RATE_EN!=0) && (!finder_active || finder_locked)),
  .ref_fcw(finder_active ? li_active_fcw : REF_FCW),.ref_valid(1'b1),
  .decim_sel(rate_decim_sel),.integration_log2(rate_integ),
  .commit(rate_commit),.settling(rate_settling),.valid(rate_valid),
  .gear(rate_gear),.change_count(rate_changes),.settle_ticks());

 // Configuration arbitration.  While a finder is searching the gear is pinned
 // to the fast gear (R=16) so a sweep point costs 63 us instead of up to
 // 64.5 ms; the FFT finder never reads lock-in results so it has no window
 // preference, but keeps the pin so the gear is deterministic until it locks.
 wire afc_precision_ready,precision_estimate_valid;
 wire signed [31:0] precision_error0,precision_error1;
 wire afc_coarse_ready;
 assign afc_coarse_ready=(afc_state==3 && (afc_reason & 8'hef)==0);
 wire bw_active   = (BW_MODE==2);
 wire rate_active = (AUTO_RATE_EN!=0);
 wire [1:0] auto_decim_sel = bw_active ? bw_decim_sel : rate_decim_sel;
 wire [4:0] auto_integ     = bw_active ? bw_integ     : rate_integ;
 wire        auto_commit   = bw_active ? bw_commit    : rate_commit;
 wire        auto_settling = bw_active ? bw_settling  : rate_settling;
 wire cfg_commit_legacy = sys_rst_n && !lockin_configured;
 wire searching = finder_active && finder_busy;
 wire [31:0] cfg_fcw_eff = fft_on ? fft_find_fcw
                          : ((SCAN_ENABLE!=0) ? scan_fcw : REF_FCW);
 wire [1:0]  cfg_decim_sel_eff = searching ? 2'd0
                              : ((bw_active || rate_active) ? auto_decim_sel : DECIM_SEL);
 wire [4:0]  cfg_integ_eff     = searching ? (fft_on ? INTEGRATION_LOG2 : scan_integ)
                              : ((bw_active || rate_active) ? auto_integ : INTEGRATION_LOG2);
 wire        gear_settling     = (bw_active || rate_active) ? auto_settling : 1'b0;
 // cfg_commit must NOT be gated by 'searching'.  freq_scan_ctrl drops busy on
 // the very same clock it emits the final commit that parks the NCO on the
 // winning frequency, so a busy-gated mux swallows that pulse: the NCO stays on
 // the last sweep probe (19.9 kHz away in simulation) and the amplitude
 // collapses.  fft_find_ctrl has the same one-cycle property.  Commit requests
 // are OR-ed unconditionally instead; gating by busy is only correct for the
 // gear/window selects, which are level signals.
 wire fft_commit_en  = fft_on ? fft_commit : 1'b0;
 wire scan_commit_en = (!fft_on && (SCAN_ENABLE!=0)) ? scan_commit : 1'b0;
wire        cfg_reconfig      = (bw_active || rate_active) ? auto_commit : 1'b0;
wire        cfg_commit        = fft_commit_en | scan_commit_en | cfg_commit_legacy;
 wire lockin_valid,measurement_present,ratio_valid;
 wire [31:0] measurement_id,measurement_age;
 wire [23:0] li_amp0,li_amp1;
 wire signed [15:0] li_ph0,li_ph1,li_pdiff;
 wire [31:0] li_ratio;
 wire [7:0] li_flags;
 wire [31:0] li_gap,li_drop;
 wire [1:0] li_det,li_qual;
 wire [31:0] li_active_fcw;
 wire [31:0] li_nco_phase;   // live NCO phase for the ET display (see et_display)
 wire [2:0] afc_state;wire afc_ready;wire signed [31:0] residual0,residual1;
 wire [7:0] afc_reason,g_reason0,g_reason1;wire [15:0] g_step0,g_step1;
 wire signed [15:0] g_signed_step0,g_signed_step1;wire g_steps_valid;
 wire [28:0] g_thresh0,g_thresh1;wire [31:0] afc_updates;
 lockin_measurement #(.CLK_FREQ(ADC_FS_HZ),.AFC_ENABLE(AFC_ENABLE),.MIN_AMPLITUDE_Q12(LOCKIN_MIN_AMP_Q12)) u_lockin(
  .clk(clk_sys),.rst_n(sys_rst_n),.adc_valid(li_adc_valid),
  .adc_ch0(ch0_cal),.adc_ch1(ch1_cal),
  .cfg_commit(cfg_commit),.cfg_reconfig(cfg_reconfig),.cfg_enable(cfg_enable),
  .cfg_fcw(cfg_fcw_eff),.cfg_decim_sel(cfg_decim_sel_eff),.cfg_integration_log2(cfg_integ_eff),
  // META copies must follow the ACTIVE configuration: the offline analyser
  // derives fs_eff from the META decim field, and ref_fcw is now the searched
  // frequency rather than a compiled constant.
  .disp_fcw(cfg_fcw_eff),.disp_decim(cfg_decim_sel_eff),.disp_integration(cfg_integ_eff),
  .result_present(measurement_present),.ratio_valid(ratio_valid),
  .result_id(measurement_id),.result_age(measurement_age),
  .lockin_valid(lockin_valid),.amp0_q12(li_amp0),.amp1_q12(li_amp1),
  .phase0_turn(li_ph0),.phase1_turn(li_ph1),.phase_diff_turn(li_pdiff),
  .ratio_q16(li_ratio),.flags(li_flags),.gap_count(li_gap),.drop_count(li_drop),
  .detected(li_det),.qualified(li_qual),.active_fcw(li_active_fcw),
  .tracking_state(afc_state),.tracking_ready(afc_ready),.precision_ready(afc_precision_ready),.precision_estimate_valid(precision_estimate_valid),
  .precision_error0(precision_error0),.precision_error1(precision_error1),.tracking_reason(afc_reason),
  .residual0_fcw(residual0),.residual1_fcw(residual1),.tracking_updates(afc_updates),
  .guard_reason0(g_reason0),.guard_reason1(g_reason1),.guard_step0(g_step0),.guard_step1(g_step1),
  .guard_signed_step0(g_signed_step0),.guard_signed_step1(g_signed_step1),.guard_steps_valid(g_steps_valid),
  .guard_threshold0(g_thresh0),.guard_threshold1(g_thresh1),.nco_phase(li_nco_phase));

 // ---- 4096-point shared-FFT carrier finder (FFT_ENABLE) -------------------
 // Taps the SAME gated, zero-bias-calibrated ADC stream as the lock-in.  The
 // FFT finds the frequency; find_fcw/commit feed the same cfg arbitration the
 // sweep used, and local_afc_2ch + lock-in_core keep doing precision
 // amplitude/phase.  FFT_ENABLE=0 ties the outputs off so the rest of the
 // design (and the META field set) is identical in both builds.
 generate if(FFT_ENABLE!=0) begin : g_fft_find
  fft_find_ctrl #(.CLK_FREQ(ADC_FS_HZ),.LOG2_N(12),.LOW_FFT_ENABLE(LOW_FFT_ENABLE),
                  .F_MIN_FCW(LOW_FFT_ENABLE?32'd528:32'd13215),.F_MAX_FCW(32'h7FFFFFFF),
                  .AMP_THRESHOLD(32'd64),.MIN_STEP_FCW(32'd1048576),
                  .REACQ_FCW(32'd264306),.SETTLE_CLKS(32'd2000000),
                  .SUPPRESS_FRAMES(16'd8),.LOST_HOLD_CLKS(32'd13000000),
                  .LOST_FRAMES(8'd8)) u_fft_find(
   .clk(clk_sys),.rst_n(sys_rst_n),
   .enable(cfg_enable),
   .adc_valid(li_adc_valid),
   .adc_ch0(ch0_cal),.adc_ch1(ch1_cal),
   // Finder monitors carrier acquisition, not temporary precision permission.
   .afc_en(AFC_ENABLE!=0),.afc_ready(afc_coarse_ready),.li_active_fcw(li_active_fcw),
   .rescan(1'b0),
   .find_fcw(fft_find_fcw),.commit(fft_commit),.locked(fft_locked),
   .busy(fft_busy),.state(fft_state),
   .fft_freq0(fft_freq0),.fft_freq1(fft_freq1),
   .fft_bin0(fft_bin0),.fft_bin1(fft_bin1),
   .fft_mag0(fft_mag0),.fft_mag1(fft_mag1),
   .frame_count(fft_frames),.lost_count(fft_lost),
   .fft_valid_channels(fft_valid_channels),.fft_update(fft_update),
   .fft_rate0(fft_rate0),.fft_rate1(fft_rate1),.fft_age0(fft_age0),.fft_age1(fft_age1),
   .spectrum_valid(spec_valid),.spectrum_fs_hz(spec_fs_hz),.spectrum_bin(spec_bin),.spectrum_power(spec_power),
   .harmonic_scan_valid(harmonic_scan_valid),.harmonic_scan_saturated(harmonic_scan_saturated),.harmonic_scan_channel(harmonic_scan_channel),
   .harmonic_scan_rate(harmonic_scan_rate),.harmonic_scan_age(harmonic_scan_age));
 end else begin : g_no_fft_find
  assign spec_valid=0;assign spec_bin=0;assign spec_power=0;
  assign spec_fs_hz=ADC_FS_HZ;
  assign harmonic_scan_valid=0;assign harmonic_scan_saturated=0;assign harmonic_scan_channel=0;
  assign harmonic_scan_rate=0;assign harmonic_scan_age=0;
  assign fft_valid_channels=0;assign fft_update=0;assign fft_rate0=0;assign fft_rate1=0;
  assign fft_age0=32'hffffffff;assign fft_age1=32'hffffffff;
  assign fft_find_fcw = 32'd0;
  assign fft_commit   = 1'b0;
  assign fft_locked   = 1'b0;
  assign fft_busy     = 1'b0;
  assign fft_state    = 3'd0;
  assign fft_freq0    = 32'd0;
  assign fft_freq1    = 32'd0;
  assign fft_bin0     = 12'd0;
  assign fft_bin1     = 12'd0;
  assign fft_mag0     = 24'd0;
  assign fft_mag1     = 24'd0;
  assign fft_frames   = 32'd0;
  assign fft_lost     = 16'd0;
 end endgenerate

 // ---- Step 3: wave parameters on the same gated full-rate ADC data ----
 // Frequency: equal-precision style edge counting with a 1 s gate
 // (GATE_SAMPLES = fs -> SCALE_SHIFT = 0, so freq_hz is the edge count in Hz;
 // the division is folded away because the gate is a power-of-two fraction of
 // the clock).  A slow DC tracker keeps the hysteresis threshold on the signal
 // mean, so a DC offset does not shift the crossing point.  Both channels see
 // the same adc_valid stream and the same gate length, so their result ids
 // advance in lockstep; the display stamps freshness with CH0's id.
 // Frequency is an explicitly valid local-reference estimate, never a noisy
 // high-frequency crossing count. Source=1 means tracked NCO, not an independent
 // frequency counter. Invalid fields are zero and fv=0; units use nominal Fs.
 // A gear change re-commits the lock-in, which resets the NCO phase and flushes
 // the CIC, so nothing published during gear_settling may be reported valid.
 wire [1:0] freq_valid_raw=(AFC_ENABLE && afc_ready && measurement_present && !gear_settling)?li_qual:2'd0;
 // Two different sources must not publish a phase difference.  The raw flags
 // above still drive the time base (the display is useful regardless), but
 // everything that claims a MEASUREMENT is gated by the channel guard.
 wire ch_mismatch;wire [15:0] ch_delta;wire [7:0] ch_score;
 ch_freq_guard #(.ENABLE(CH_MISMATCH_EN)) u_chguard(
  .clk(clk_sys),.rst_n(sys_rst_n),
  .enable(cfg_enable && measurement_present && !gear_settling),
  .meas_valid(lockin_valid),.step0(g_signed_step0),.step1(g_signed_step1),.step_valid(g_steps_valid),
  .amp_ok((li_amp0>=LOCKIN_MIN_AMP_Q12)&&(li_amp1>=LOCKIN_MIN_AMP_Q12)),
  .mismatch(ch_mismatch),.delta_avg(ch_delta),.score(ch_score));
 wire [1:0] freq_valid = ch_mismatch ? 2'd0 : freq_valid_raw;
 wire ratio_valid_g = ratio_valid && !ch_mismatch;
 wire linux_meas_usable=measurement_present && ratio_valid_g && li_qual==3 && afc_ready &&
    cal_clip==0 && raw_otr[1:0]==0 && li_flags==0 && freq_valid==3 &&
    !gear_settling && !ch_mismatch && ((FFT_ENABLE!=0)?fft_state==4:((SCAN_ENABLE==0)||scan_locked));
 linux_measurement_pack #(.FS_HZ(ADC_FS_HZ),.META_PROTO(LOW_FFT_ENABLE?12:10)) u_linux_meas(
  .result_id(measurement_id),.result_age(measurement_age),.active_fcw(li_active_fcw),.ratio_q16(li_ratio),
  .amp0_q12(li_amp0),.amp1_q12(li_amp1),.phase0(li_ph0),.phase1(li_ph1),.pdiff(li_pdiff),
  .qual(li_qual),.det(li_det),.cal_clip(cal_clip),.adc_otr(raw_otr[1:0]),.fft_valid(fft_valid_channels),
  .usable(linux_meas_usable),.present(measurement_present),.ratio_valid(ratio_valid_g),.afc_ready(afc_ready),
  .same_source(!ch_mismatch),.settling(gear_settling),.cal_valid(cal_valid),
  .flags(li_flags),.afc_state({5'd0,afc_state}),.afc_reason(afc_reason),.guard0(g_reason0),.guard1(g_reason1),
  .bw_gear({6'd0,bw_gear}),.decim_log2({2'd0,(6'd4+{cfg_decim_sel_eff,1'b0})}),.integration_log2({3'd0,cfg_integ_eff}),
  .gap(li_gap),.drop(li_drop),.fft_hz0(fft_freq0),.fft_hz1(fft_freq1),.fft_age0(fft_age0),.fft_age1(fft_age1),
  .fft_rate0(fft_rate0),.fft_rate1(fft_rate1),.residual0(residual0),.residual1(residual1),.body(linux_meas_body));
 (* use_dsp48="yes" *) reg [63:0] fcw_hz_product;
 reg [31:0] tracked_hz;
 always @(posedge clk_sys) begin
  if(!sys_rst_n) begin fcw_hz_product<=0;tracked_hz<=0;end
  else begin fcw_hz_product<=li_active_fcw*64'd65000000;
   tracked_hz<=fcw_hz_product[63:32]+fcw_hz_product[31];end
 end
 wire [31:0] freq0_hz=freq_valid[0]?tracked_hz:32'd0;
 wire [31:0] freq1_hz=freq_valid[1]?tracked_hz:32'd0;
 wire [31:0] freq0_id=measurement_id,freq1_id=measurement_id;
 wire freq0_valid=freq_valid[0],freq1_valid=freq_valid[1];
 // Metrics: 65536-sample windows (~1.008 ms, ~10 cycles at the 10 kHz board
 // test frequency) of mean / Vpp / AC-RMS, published back-to-back.
 wire signed [11:0] met_mean0,met_mean1;
 wire [12:0] met_vpp0,met_vpp1;
 wire [11:0] met_rms0,met_rms1;
 wire met0_valid,met1_valid,met0_drop,met1_drop,met0_err,met1_err;
 wire [31:0] met0_wid,met1_wid;
 channel_metrics_fix #(.LOG2_N(16),.DATA_W(12)) u_met0(
  .clk(clk_sys),.rst_n(sys_rst_n),.sample_valid(li_adc_valid),
  .sample(ch0_cal),.mean(met_mean0),.vpp(met_vpp0),.rms_ac(met_rms0),
  .metrics_valid(met0_valid),.metrics_ready(1'b1),.window_id(met0_wid),
  .window_ready(),.calc_busy(),.window_dropped(met0_drop),
  .arithmetic_error(met0_err));
 channel_metrics_fix #(.LOG2_N(16),.DATA_W(12)) u_met1(
  .clk(clk_sys),.rst_n(sys_rst_n),.sample_valid(li_adc_valid),
  .sample(ch1_cal),.mean(met_mean1),.vpp(met_vpp1),.rms_ac(met_rms1),
  .metrics_valid(met1_valid),.metrics_ready(1'b1),.window_id(met1_wid),
  .window_ready(),.calc_busy(),.window_dropped(met1_drop),
  .arithmetic_error(met1_err));
 // keep the unused step-3 outputs from floating
 wire unused_step3 = &{1'b0,freq0_valid,freq1_valid,freq1_id,met_mean0,met_mean1,
  met_rms1,met0_wid,met1_wid,met0_valid,met1_valid,met0_drop,met1_drop,
  met0_err,met1_err};

 // ---- equivalent-time display (ET_ENABLE) ----------------------------------
 // The NCO phase that multiplies a sample is the phase-register value on the
 // SAME cycle the sample arrives (the accumulator is read before its own
 // increment), so (ch_cal, nco_phase) at a li_adc_valid cycle is an exact
 // pair.  Both are registered together here; et_fcw travels with them and is
 // exactly the increment between two consecutive valid samples.
 wire et_mode = (ET_ENABLE!=0) && et_mode_sw;
 reg et_v; reg signed [11:0] et_c0,et_c1; reg [31:0] et_ph,et_fcw;
 always @(posedge clk_sys) begin
  et_v<=li_adc_valid; et_c0<=ch0_cal; et_c1<=ch1_cal;
  et_ph<=li_nco_phase; et_fcw<=li_active_fcw;
 end
 wire et_frame_pulse; wire [15:0] et_frame_id;
 wire [ET_AW:0] et_filled_bin,et_ready_bin;
 wire [31:0] et_wraps,et_hits,et_epoch;
 wire et_snapshot_locked,et_reference_ok,et_rd_valid;
 wire [15:0] et_rd_hits;
 wire signed [11:0] et_rd_ch0,et_rd_ch1;
 generate if(ET_ENABLE!=0) begin : g_et_display
  et_display #(.ET_AW(ET_AW),.ET_AVG_SHIFT(ET_AVG_SHIFT),
               .ET_FRAME_CLKS(ET_FRAME_CLKS),.CLK_FREQ(ADC_FS_HZ)) u_et(
   .clk(clk_sys),.rst_n(sys_rst_n),
   .mode(et_mode),.sample_valid(et_v),.ch0(et_c0),.ch1(et_c1),
   .nco_phase(et_ph),.nco_fcw(et_fcw),.afc_locked(afc_ready),
   .reference_ok(afc_precision_ready && precision_estimate_valid && li_qual==2'b11 && li_flags==0 && afc_reason==0),
   .residual0_fcw(precision_error0),.residual1_fcw(precision_error1),
   .cfg_commit(cfg_commit),.freeze(freeze),.hold(display_busy),
   .frame_pulse(et_frame_pulse),.frame_id(et_frame_id),
   .filled(et_filled_bin),.ready_bins(et_ready_bin),
   .wraps(et_wraps),.hits(et_hits),.epoch_id(et_epoch),
   .snapshot_locked(et_snapshot_locked),.snapshot_reference_ok(et_reference_ok),
   .rd_addr(rd_addr),.rd_ch0(et_rd_ch0),.rd_ch1(et_rd_ch1),
   .rd_valid(et_rd_valid),.rd_hits(et_rd_hits),.dbg_cnt());
 end else begin : g_no_et_display
  assign et_frame_pulse=1'b0; assign et_frame_id=16'd0;
  assign et_filled_bin={(ET_AW+1){1'b0}}; assign et_wraps=32'd0; assign et_hits=32'd0;
  assign et_ready_bin=0;assign et_epoch=0;
  assign et_snapshot_locked=0;assign et_reference_ok=0;
  assign et_rd_valid=0;assign et_rd_hits=0;
  assign et_rd_ch0=12'sd0; assign et_rd_ch1=12'sd0;
 end endgenerate
 wire [9:0] et_bins_v  = (ET_ENABLE!=0) ? (10'd1<<ET_AW) : 10'd0;
 // Display source mux: sw[0] up -> ET frame (phase bins, et_bins rows);
 // down -> real-time snapshot frame (DISP_N rows).
 wire snap_ready_disp = et_mode ? et_frame_pulse : snap_ready;
 wire [15:0] frame_id_disp = et_mode ? et_frame_id : frame_id;
 // The READ-DATA source must be latched at the same instant the display latches
 // its header, not switched live.  dn= in the META line is latched at
 // snap_ready; if the mux below stayed live, flipping the ET switch during the
 // ~100 ms frame would deliver rows from the other source while the header still
 // announced the old row count.  disp_et_hold is captured on the very same
 // snap_ready_disp that starts the frame, so header and body always agree.
 reg disp_et_hold;
 always @(posedge clk_sys) begin
  if(!sys_rst_n) disp_et_hold<=1'b0;
  else if(snap_ready_disp && !display_busy) disp_et_hold<=et_mode;
 end
 wire signed [11:0] disp_rd_ch0 = disp_et_hold ? et_rd_ch0 : d0;
 wire signed [11:0] disp_rd_ch1 = disp_et_hold ? et_rd_ch1 : d1;
 // synthesis translate_off
 initial begin
  if(ET_ENABLE!=0 && LOCKIN_ENABLE==0)
   $fatal(1,"ET display needs LOCKIN_ENABLE=1: the NCO is the phase reference");
  if(ET_AW<1 || ET_AW>9) $fatal(1,"ET_AW must be 1..9");
  if(ET_AVG_SHIFT<1 || ET_AVG_SHIFT>8) $fatal(1,"ET_AVG_SHIFT must be 1..8");
  if(ET_FRAME_CLKS<16) $fatal(1,"ET_FRAME_CLKS too small");
 end
 // synthesis translate_on

 // fft_present tells the display which proto to publish.  With FFT_ENABLE=0
// the finder outputs are tied to zero, so the ffs/ffb*/fff*/ffm*/ffid/fflost
// slots carry no information and the line must say proto=5, not 6 - otherwise
// a host would read "finder idle" out of a finder that was never built.
generate if(COM9_TEXT_ENABLE!=0)begin: g_com9_text
vofa_base_display #(.LF_FFT(LOW_FFT_ENABLE && FFT_ENABLE),.CLK_FREQ(ADC_FS_HZ),.BAUD(UART_BAUD),.DISP_AW(DISP_AW),
                    .ET_AW(ET_AW)) u_vofa(
  .fft_present(fft_on),.et_present(ET_ENABLE!=0),
  .clk(clk_sys),.rst_n(sys_rst_n),.snap_ready(snap_ready_disp),.snap_frame_id(frame_id_disp),
  .snap_sample_interval_ns(interval_ns),.snap_trig_pos(trig_pos),
  .snap_filtered(filtered),.snap_fault(fault),.snap_decim_log2(decim),
  .snap_auto_trigger(auto_trigger),.snap_rd_addr(rd_addr),.snap_rd_ch0(disp_rd_ch0),
  .snap_rd_ch1(disp_rd_ch1),.disp_busy(display_busy),.tx(uart_tx),
  .lockin_valid(lockin_valid),.measurement_present(measurement_present),
  .ratio_valid(ratio_valid_g),.measurement_id(measurement_id),.measurement_age(measurement_age),.amp0_q12(li_amp0),.amp1_q12(li_amp1),
  .phase0_turn(li_ph0),.phase1_turn(li_ph1),.phase_diff_turn(li_pdiff),
  .ratio_q16(li_ratio),.lockin_flags(li_flags),.lockin_gap(li_gap),.lockin_drop(li_drop),
  .lockin_detected(li_det),.lockin_qualified(li_qual),
  .ref_fcw(li_active_fcw),.cfg_decim(cfg_decim_sel_eff),.cfg_integration(cfg_integ_eff),
  .freq0_hz(freq0_hz),.freq1_hz(freq1_hz),.freq_result_id(freq0_id),
  .vpp0(met_vpp0),.vpp1(met_vpp1),.rms0(met_rms0),
  .cal_off0(ZERO_OFFSET_CH0),.cal_off1(ZERO_OFFSET_CH1),
  .cal_valid(cal_valid),.cal_clip(cal_clip),
  .afc_state(afc_state),.afc_ready(afc_ready),.afc_reason(afc_reason),
  .afc_error0(residual0),.afc_error1(residual1),.afc_updates(afc_updates),
  .guard_reason0(g_reason0),.guard_reason1(g_reason1),.guard_step0(g_step0),.guard_step1(g_step1),
  .guard_threshold0(g_thresh0),.guard_threshold1(g_thresh1),.frequency_valid(freq_valid),
  .scan_state(scan_state),.scan_locked(scan_locked),
  .scan_best_amp(scan_best_amp),.scan_passes(scan_passes),
  .fft_state(fft_state),.fft_bin0(fft_bin0),.fft_bin1(fft_bin1),
  .fft_freq0(fft_freq0),.fft_freq1(fft_freq1),
  .fft_mag0(fft_mag0),.fft_mag1(fft_mag1),
  .fft_frames(fft_frames),.fft_lost(fft_lost),
  .fft_valid_channels(fft_valid_channels),.fft_rate0(fft_rate0),.fft_rate1(fft_rate1),.fft_age0(fft_age0),.fft_age1(fft_age1),
  .bw_gear(bw_gear),.bw_settling(gear_settling),
  .bw_changes(bw_changes),.bw_phase_step(bw_pstep),
  .aa_order(AA_ORDER4),.tb_auto(disp_tb_auto),.tb_m(disp_timebase),
  .tb_sat(tb_sat),.ch_mismatch(ch_mismatch),.ch_delta(ch_delta),
  // The ET ports must be connected even in an ET_ENABLE=0 build: left open,
  // et_mode is undriven and the display's IDLE state would latch
  // decim/trig_pos/interval_ps through an X select, putting X into r=, t= and
  // dt_ps= on every real-time frame.  With ET_ENABLE=0 these wires are all
  // tied to zero by the generate above, which is exactly what they should be.
  .et_mode(et_mode),.et_bins(et_bins_v),.et_locked(et_snapshot_locked),
  .et_filled(et_filled_bin),.et_wraps(et_wraps),.et_hits(et_hits),
  .et_ready_bins(et_ready_bin),.et_epoch(et_epoch),.et_reference_ok(et_reference_ok),
  .et_rd_valid(et_rd_valid),.et_rd_hits(et_rd_hits));
end else begin: g_no_com9_text
 // An unused UART must idle high. Release only the private text snapshot
 // consumer; do not connect this busy signal to the AB5 frame scheduler.
 assign uart_tx=1'b1;
 assign display_busy=1'b0;
 assign rd_addr={DISP_AW{1'b0}};
end endgenerate
 // User LED order [0..3]: lock, ADC ready, native RAW mode, capture error.
 // Core schematic: user LEDs are active high (FPGA -> LED -> resistor -> GND).
 assign led={(|errors),(disp_timebase==0),adc_ready,locked};
endmodule
`default_nettype wire
