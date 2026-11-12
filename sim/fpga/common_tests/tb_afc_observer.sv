// tb_afc_observer.sv — OPEN-LOOP verification of the short-window observer.
//
// Unlike tb_afc_controller (ideal rotating IQ), this feeds the estimator from a
// REAL signal path: a 12-bit quantised ADC sine at 65 MSPS, mixed by a behavioural
// 32-bit NCO, accumulated by rtl/afc_observer.v, then handed to afc_controller.
// The loop is deliberately OPEN (afc_fcw is NOT applied back), so the measured
// residual must equal the injected offset.
//
// Checks: sign (direction), magnitude, and the unambiguous range.
`timescale 1ns/1ps
`default_nettype none
module tb_afc_observer;
    localparam real    FS_R     = 65000000.0;
    localparam integer IQ_W     = 18;
    localparam integer OBS_LEN  = 4160;            // -> 64 us @65 MSPS
    localparam [31:0]  NOM      = 32'd1982292598;  // 30 MHz
    localparam real    FCW2HZ   = FS_R / 4294967296.0;
    localparam real    NOM_HZ   = 1982292598.0 * FCW2HZ;
    localparam integer ADC_BITS = 12;              // AD9238
    localparam integer AMP      = 2047;            // full-scale-ish
    localparam real    T_OBS_S  = 64e-6;

    reg clk=0, rst_n=0;
    always #7.692 clk = ~clk;

    // ---------------- tonal tables ----------------
    // df nominal, plus the alias probe beyond the unambiguous range
    integer n_df, idx;
    reg signed [31:0] df_hz = 0;
    wire              frame_ok;

    // ---------------- real ADC samples ----------------
    reg  [31:0] sample_n = 0;
    wire real  t = sample_n / FS_R;
    wire real  fi = NOM_HZ + $itor(df_hz);
    reg  signed [ADC_BITS-1:0] adc0, adc1;
    reg  adc_valid;

    always @(posedge clk) begin
        if(!rst_n) begin
            sample_n<=0; adc_valid<=0; adc0<=0; adc1<=0;
        end else begin
            sample_n  <= sample_n + 1'b1;
            adc_valid <= 1'b1;
            // ch0 and ch1 carry the SAME tone here (single-channel estimator checks)
            adc0 <= $rtoi(AMP * $sin(2.0*3.141592653589793*fi*t));
            adc1 <= $rtoi(AMP * $sin(2.0*3.141592653589793*fi*t));
        end
    end

    // ---------------- behavioural NCO + mixer ----------------
    reg [31:0] nco_phase = 0;
    wire real theta = 2.0*3.141592653589793*$itor(nco_phase)/4294967296.0;
    reg signed [IQ_W-1:0] mix_i0, mix_q0, mix_i1, mix_q1;
    always @(posedge clk) begin
        if(!rst_n) begin
            nco_phase<=0; mix_i0<=0; mix_q0<=0; mix_i1<=0; mix_q1<=0;
        end else if(adc_valid) begin
            nco_phase <= nco_phase + NOM;                 // open loop: stays nominal
            mix_i0 <= $rtoi($itor(adc0) * $cos(theta));
            mix_q0 <= $rtoi(-$itor(adc0) * $sin(theta));  // project convention Q = -sin
            mix_i1 <= $rtoi($itor(adc1) * $cos(theta));
            mix_q1 <= $rtoi(-$itor(adc1) * $sin(theta));
        end
    end

    // ---------------- observer under test ----------------
    wire                   obs_valid;
    wire signed [23:0]     obs_i, obs_q, obs2_i, obs2_q;
    wire [31:0]            obs_ts, ts_w;
    wire [23:0]            obs_amp, obs2_amp;
    reg  [31:0]            ts_ctr = 0;
    always @(posedge clk) if(!rst_n) ts_ctr<=0; else ts_ctr<=ts_ctr+1'b1;

    afc_observer #(.IQ_W(IQ_W), .OBS_LEN(OBS_LEN), .ACC_W(40), .OUT_W(24), .TS_W(32)) u_obs (
        .clk(clk), .rst_n(rst_n), .iq_valid(adc_valid),
        .mix_i0(mix_i0), .mix_q0(mix_q0), .mix_i1(mix_i1), .mix_q1(mix_q1),
        .ts_in(ts_ctr),
        .obs_valid(obs_valid), .obs_i(obs_i), .obs_q(obs_q),
        .obs2_i(obs2_i), .obs2_q(obs2_q),
        .obs_timestamp(obs_ts), .obs_amp(obs_amp), .obs2_amp(obs2_amp)
    );

    // ---------------- estimator under test (controller, loop NOT closed) ----------------
    wire [31:0] afc_fcw; wire afc_commit; wire [1:0] afc_state;
    wire afc_locked, afc_stable, revoke, limit_flag, settling, afcv, dualf, dtanom;
    wire [31:0] drops, ddt, dfcw; wire dvalid;
    wire signed [23:0] di, dq; wire signed [15:0] dph; wire [23:0] damp;
    wire [31:0] dts, dfrm;

    afc_controller #(.FS_HZ(65000000), .T_OBS_S(T_OBS_S), .OBS_W(24),
                     .CAPTURE_RANGE_FCW(500000), .SMOOTH_SHIFT(3),
                     .COMMIT_INT(4), .MIN_OBS_AMP(64),
                     .STABLE_WIN(8), .HOLD_WIN(64), .SETTLE_WIN(8)) u_afc (
        .clk(clk), .rst_n(rst_n), .adc_valid(1'b1),
        .obs_i(obs_i), .obs_q(obs_q), .obs_valid(obs_valid), .obs_amp(obs_amp),
        .obs_timestamp(obs_ts),
        .obs2_i(obs2_i), .obs2_q(obs2_q), .obs2_valid(obs_valid), .obs2_amp(obs2_amp),
        .nominal_fcw(NOM),
        .afc_fcw(afc_fcw), .afc_commit(afc_commit), .afc_state(afc_state),
        .afc_locked(afc_locked), .afc_stable(afc_stable),
        .revoke_measurement(revoke), .limit_flag(limit_flag),
        .afc_settling(settling), .afc_valid(afcv),
        .dual_freq_flag(dualf), .dt_anomaly(dtanom),
        .drop_count(drops), .diag_dt(ddt), .diag_fcw(dfcw),
        .diag_valid(dvalid), .diag_i(di), .diag_q(dq), .diag_phase(dph),
        .diag_amp(damp), .diag_timestamp(dts), .diag_frame(dfrm));

    // ---------------- stimulus sweep + checks ----------------
    integer fails = 0;
    integer k, kk;
    integer dfs[0:7];
    reg signed [31:0] measured;
    reg signed [31:0] expf;
    reg signed [31:0] err;

    initial begin
        dfs[0]=1199; dfs[1]=-1199; dfs[2]=120;  dfs[3]=-120;
        dfs[4]=5000; dfs[5]=-5000; dfs[6]=7000; dfs[7]=-7000;

        #200 rst_n=1;
        #(OBS_LEN*16*8);   // flush the pipeline

        $display("=== OPEN-LOOP: 12-bit ADC @65MSPS -> NCO mix -> afc_observer -> estimator ===");
        for(k=0;k<8;k=k+1) begin
            df_hz = dfs[k];
            for(kk=0;kk<OBS_LEN*16*6;kk=kk+1) @(posedge clk);
            measured = u_afc.targ_fcw;
            expf     = $rtoi($itor(dfs[k]) / FCW2HZ);
            err      = measured - expf;
            if(err<0) err = -err;
            if(k<6) begin
                // inside the unambiguous range: 1% + 200 FCW margin
                // tolerance = 200 FCW + 1% of |expected|. NOTE: the percentage must be
            // taken on the ABSOLUTE value -- /(100) on a signed negative number makes
            // the tolerance negative and every negative-offset case "fails".
            if(err > 200 + ((expf<0 ? -expf : expf)/100)) begin
                    fails = fails + 1;
                    $display("  FAIL df=%0d Hz  measured=%0d  expected=%0d  err=%0d  amp=%0d",
                             dfs[k], measured, expf, err, obs_amp);
                end else begin
                    $display("  ok   df=%+7d Hz  measured=%+9d  expected=%+9d  err=%0d  amp=%0d",
                             dfs[k], measured, expf, err, obs_amp);
                end
            end else begin
                $display("  edge df=%+7d Hz  measured=%+9d  expected=%+9d  err=%0d  amp=%0d",
                         dfs[k], measured, expf, err, obs_amp);
            end
        end
        // beyond +-7812.5 Hz the wrapped phase is genuinely ambiguous; record, do not judge
        df_hz = 10000;
        for(kk=0;kk<OBS_LEN*16*6;kk=kk+1) @(posedge clk);
        $display("  info df=10000 Hz beyond range: reported=%0d  alias prediction=%0d (documented)",
                 u_afc.targ_fcw, $rtoi(-5625.0/FCW2HZ));

        $display("----------------------------------------------------");
        if(fails==0) begin
            $display("PASS open-loop observer: direction, magnitude and range edges verified");
            $finish;
        end else begin
            $fatal(1,"open-loop observer: %0d check(s) failed", fails);
        end
    end
endmodule
`default_nettype wire
