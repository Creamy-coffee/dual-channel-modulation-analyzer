`timescale 1ns/1ps
`default_nettype none
// ---------------------------------------------------------------------------
// Automatic display timebase  --  how many cycles fit on the screen.
// ---------------------------------------------------------------------------
// This is a SEPARATE control from the measurement bandwidth (meas_bw_ctrl):
//
//   display timebase  decides how the waveform LOOKS   (window length, R_disp)
//   measurement bw    decides how STABLE the reading is (lock-in window)
//
// Mixing the two is a design error: a 1024-point RAW frame contains about 473 cycles at 30 MHz
// in 15.75 us (PC zoom is separate), while the lock-in simultaneously integrates 64 ms of it.
//
// RULE
// ----
//   cycles_on_screen = f * DISP_N * 2^m / fs
//   choose m so that about TARGET_CYCLES (8) fill the DISP_N-point frame.
//   The decimation ratio is a power of two (m = log2 R), so the sample
//   interval stays exact and no divider is needed at run time.
//
//   f >= 507.8 kHz  -> m = 0      : keep RAW samples.  At 30 MHz that is
//                                   2.17 samples/cycle; the polyline looks
//                                   coarse but is NOT undersampled -- the ADC
//                                   still samples at 65 MS/s.  Never invent
//                                   samples to make it look smooth.
//   100 kHz         -> m = 2  (R=4)
//   10 kHz          -> m = 6  (R=64)
//   1 kHz           -> m = 9  (R=512)
//   500 Hz          -> m = 10 (R=1024, 8.06 cycles)
//   10 Hz           -> m = 15 (R=32768, 5.16 cycles; the hardware ceiling)
//
// WHY NOT ALWAYS m=0 AND LET THE PC ZOOM
// -------------------------------------
//   The frame only carries DISP_N points.  Without decimation a 500 Hz tone
//   would be 0.008 cycles across the frame -- a flat line.  The points must be
//   spread over a longer time, which is exactly what decimation does.
//
// HYSTERESIS
// ----------
//   Do not chase the ideal m on every frequency update.  Change only when the
//   CURRENT m puts the cycle count outside [MIN_CYCLES, MAX_CYCLES] and it has
//   stayed outside for DWELL frequency updates.  A frequency reading that
//   wobbles by a few Hz around a boundary therefore cannot make the display
//   flicker between two timebases.
// ---------------------------------------------------------------------------
module auto_timebase #(
    parameter integer FS_HZ       = 65000000,
    parameter integer DISP_N      = 1024,
    parameter integer TARGET_CYCLES = 8,
    parameter integer MIN_CYCLES  = 3,
    parameter integer MAX_CYCLES  = 12,
    parameter integer MAX_M       = 15,   // display decimation ceiling, R<=32768
    parameter [7:0]   DWELL       = 8'd4  // frequency updates before a change
)(
    input  wire        clk, rst_n, enable,
    input  wire [31:0] freq_hz,
    input  wire        freq_valid,
    input  wire        freq_update, // one pulse per newly published measurement
    output reg  [3:0]  m,
    output reg         auto_active,
    output reg         saturated,   // even MAX_M cannot reach MIN_CYCLES
    output reg  [7:0]  changes
);
    // freq*2^m at which the frame holds TARGET_CYCLES.  Localparams are
    // constant-folded, so the comparisons below are plain integer compares.
    localparam [47:0] TARGET_NUM = (48'd1 * TARGET_CYCLES * FS_HZ) / DISP_N;   // 507812
    // log-domain half-step boundary: choose m by |log2|, i.e. compare against
    // TARGET/sqrt(2) so the decision is "nearest power of two", not "first one
    // that fits" (which would always over-decimate by up to 2x).
    localparam [47:0] HALF_NUM   = (48'd70711 * TARGET_NUM) / 48'd100000;      // 359082
    localparam [47:0] CYC_LO     = (48'd1 * MIN_CYCLES * FS_HZ) / DISP_N;      // 190429
    localparam [47:0] CYC_HI     = (48'd1 * MAX_CYCLES * FS_HZ) / DISP_N;      // 761718

    localparam [3:0] MAX_M4 = MAX_M;
    reg [3:0]  target_m;
    reg [47:0] scaled;
    integer    i;

    // ---- ideal m from the measured frequency ------------------------------
    // best = (number of doublings that still stay under HALF_NUM).
    always @* begin
        target_m = 4'd0;
        for (i = 0; i < MAX_M; i = i + 1) begin
            scaled = {16'd0, freq_hz} << i;
            if (scaled <= HALF_NUM) target_m = i[3:0] + 4'd1;
        end
    end

    wire [47:0] cycles_num = ({16'd0, freq_hz} << m);   // cycles*63476.6

    reg [7:0] dwell;
    always @(posedge clk) begin
        if (!rst_n) begin
            m           <= 4'd0;
            auto_active <= 1'b0;
            saturated   <= 1'b0;
            changes     <= 8'd0;
            dwell       <= 8'd0;
        end else if (!enable) begin
            // Manual timebase owns the knob: hand back m=0 and say inactive.
            m           <= 4'd0;
            auto_active <= 1'b0;
            saturated   <= 1'b0;
            dwell       <= 8'd0;
        end else begin
            auto_active <= 1'b1;
            if (!freq_valid || freq_hz == 32'd0) begin
                // Losing the lock must NOT move the timebase: a frozen
                // display of the last good window is far more useful than a
                // jump to m=0 the moment the sweep restarts.
                dwell <= 8'd0;
            end else if (freq_update) begin
              saturated <= (target_m == MAX_M4) && (({16'd0,freq_hz} << MAX_M)<CYC_LO);
              if (cycles_num < CYC_LO || cycles_num > CYC_HI) begin
                if (dwell + 8'd1 >= DWELL) begin
                    dwell <= 8'd0;
                    if (target_m != m) begin
                        m <= target_m;
                        if (changes != 8'hff) changes <= changes + 8'd1;
                    end
                end else begin
                    dwell <= dwell + 8'd1;
                end
              end else begin
                dwell <= 8'd0;
              end
            end
        end
    end
    // synthesis translate_off
    initial if(MAX_M<0 || MAX_M>15 || DWELL<1 || FS_HZ<1 || DISP_N<1)
      $fatal(1,"Invalid timebase parameters");
    // synthesis translate_on
endmodule
`default_nettype wire
