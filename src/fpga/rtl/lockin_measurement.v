`timescale 1ns/1ps
`default_nettype none
// Step 2: synchronous dual-channel lock-in amplitude/phase measurement.
// Full-rate ADC (ch0, ch1) -> lockin_core_2ch (shared NCO + shared integration
// window) -> lockin_signal_guard (stale-free validity) -> amplitude ratio.
// Complete records are published after division. Live faults invalidate immediately.
// Watchdog allows four integration windows plus 1024 processing clocks.
module lockin_measurement #(
    parameter integer CLK_FREQ = 65000000,
    parameter integer AFC_ENABLE = 0,
    // Validity amplitude floor (Q12) for the lock-in guard. Lowered from the
    // original 32768 so a weak signal is not rejected as "absent".
    // NOTE: 2048 is PRELIMINARY / NOT yet validated. The guard compares the Q12
    // amplitude, which includes the mixer 1/2 gain and CIC normalization, so it
    // is NOT directly the input Vpp. It is only validated by measuring BOTH the
    // zero-input false-positive rate AND the >=20 mVpp detection rate.
    parameter [23:0] MIN_AMPLITUDE_Q12 = 24'd2048
)(
    input wire clk, rst_n,
    input wire adc_valid,                       // already gated by OTR in daq_top
    input wire signed [11:0] adc_ch0, adc_ch1,
    input wire cfg_commit, cfg_reconfig, cfg_enable,
    input wire [31:0] cfg_fcw,                  // reference frequency word
    input wire [1:0] cfg_decim_sel,
    input wire [4:0] cfg_integration_log2,
    // Display-side copies of the active configuration
    input wire [31:0] disp_fcw,
    input wire [1:0] disp_decim,
    input wire [4:0] disp_integration,
    // Outputs (all latched together when a fresh measurement completes)
    output reg lockin_valid,
    output reg [23:0] amp0_q12, amp1_q12,
    output reg signed [15:0] phase0_turn, phase1_turn,
    output reg signed [15:0] phase_diff_turn,
    output reg [31:0] ratio_q16,               // amp1/amp0, Q16 (PC divides by 65536)
    output wire [7:0] flags,                   // live core flags; bit7 = no current result
    output wire [31:0] gap_count, drop_count,
    output reg result_present, ratio_valid,
    output reg [31:0] result_id, result_age,
    output reg [1:0] detected, qualified,     // cleared immediately on invalidation
    output wire [2:0] tracking_state,
    output wire tracking_ready,
    output wire precision_ready,
    output wire precision_estimate_valid,
    output wire signed [31:0] precision_error0,precision_error1,
    output wire signed [31:0] residual0_fcw,residual1_fcw,
    output wire [7:0] tracking_reason,guard_reason0,guard_reason1,
    output wire [15:0] guard_step0,guard_step1,
    output reg signed [15:0] guard_signed_step0,guard_signed_step1,
    output reg guard_steps_valid,
    output wire [28:0] guard_threshold0,guard_threshold1,
    output wire [31:0] tracking_updates,
    output wire [31:0] active_fcw,
    // Live NCO phase (see lockin_core_2ch): paired with the sample on the
    // same adc_valid cycle.  Consumed by the ET display for phase binning.
    output wire [31:0] nco_phase
);

    wire ddc_valid, ddc_publish;
    wire [31:0] ddc_fid, ddc_timestamp, ddc_gaps, ddc_drops;
    wire signed [23:0] ddc_i0, ddc_q0, ddc_i1, ddc_q1;
    wire [23:0] ddc_amp0, ddc_amp1;
    wire signed [15:0] ddc_phase0, ddc_phase1;
    wire [1:0] ddc_phase_valid;
    wire [7:0] ddc_flags;
    wire [1:0] ddc_detected, ddc_qualified;
    // Upper bound = ADC Nyquist: fcw(32.5 MHz) = 2^31 = 0x80000000.
    // The previous bound 0x76276276 was exactly FCW(30 MHz), so any
    // fine-tuned reference above 30 MHz was silently rejected
    // (config_ok=0 -> permanent invalidate).  Guard thresholds untouched.
    wire config_ok=(cfg_fcw>=32'd66 && cfg_fcw<32'h80000000 &&
                    cfg_integration_log2>=3 && cfg_integration_log2<=16);
    wire [5:0] window_shift={1'b0,cfg_integration_log2}+6'd4+{3'd0,cfg_decim_sel,1'b0};
    wire [31:0] stale_limit=(32'd1<<window_shift)*4+32'd1024;
    reg [31:0] source_age;
    wire expired=source_age>=stale_limit;
    wire invalidate=cfg_commit || cfg_reconfig || !cfg_enable || !config_ok || !adc_valid || (expired && !ddc_valid);
    reg [7:0] local_flags;
    reg [31:0] local_drops;
    wire [32:0] drops_sum={1'b0,ddc_drops}+{1'b0,local_drops};
    // bit7: no current measurement; bit6: invalid/overflowed amplitude ratio.
    assign flags=ddc_flags | local_flags | (result_present?8'd0:8'h80);
    assign gap_count=ddc_gaps;
    assign drop_count=drops_sum[32]?32'hffffffff:drops_sum[31:0];

    wire mv_mix;wire signed [27:0] mi0,mq0,mi1,mq1;wire [31:0] mt;
    wire [31:0] tuned_fcw;wire tune_commit;
    wire coarse_ready;
    wire [7:0] coarse_reason;
    wire precise_window=(window_shift>=22);
    assign tracking_ready=coarse_ready && (!precise_window || precision_ready);
    assign tracking_reason=coarse_reason | ((precise_window && !precision_ready)?8'h10:8'd0);
    generate if(AFC_ENABLE) begin : tracking
     wire [31:0] coarse_fcw,fine_fcw;
     wire coarse_commit,fine_commit;
     wire [31:0] coarse_updates,fine_updates;
     wire [32:0] update_sum={1'b0,coarse_updates}+{1'b0,fine_updates};
     assign tracking_updates=update_sum[32]?32'hffffffff:update_sum[31:0];
     wire [7:0] fine_reason;
     assign tuned_fcw=coarse_commit?coarse_fcw:fine_fcw;
     assign tune_commit=coarse_commit || fine_commit;
     afc_precision_ctrl #(.MIN_AMP(MIN_AMPLITUDE_Q12)) fine(
      .clk(clk),.rst_n(rst_n),
      .clear(cfg_commit || !cfg_enable || !config_ok || !adc_valid || !coarse_ready || coarse_commit),
      .reconfig(cfg_reconfig),.in_valid(ddc_valid),.timestamp(ddc_timestamp),
      .current_fcw(active_fcw),.nominal_fcw(cfg_fcw),.window_log2(window_shift),
      .phase0(ddc_phase0),.phase1(ddc_phase1),.phase_valid(ddc_phase_valid),
      .amp0(ddc_amp0),.amp1(ddc_amp1),.commit(fine_commit),.target_fcw(fine_fcw),
      .ready(precision_ready),.estimate_valid(precision_estimate_valid),
      .error0_fcw(precision_error0),.error1_fcw(precision_error1),.reason(fine_reason),.estimates(),.updates(fine_updates));
     local_afc_2ch #(.CLK_FREQ(CLK_FREQ)) afc(.clk(clk),.rst_n(rst_n),.clear(cfg_commit || !cfg_enable || !config_ok),
      .stream_ok(adc_valid),.mix_valid(mv_mix),.mi0(mi0),.mq0(mq0),.mi1(mi1),.mq1(mq1),
      .sample_time(mt),.nominal_fcw(cfg_fcw),.current_fcw(active_fcw),.target_fcw(coarse_fcw),.commit(coarse_commit),
      .ready_measurement(coarse_ready),.state(tracking_state),.residual0_fcw(residual0_fcw),
      .residual1_fcw(residual1_fcw),.reason(coarse_reason),.updates(coarse_updates),
      .observation_id(),.last_timestamp());
    end else begin : fixed_reference
     assign tuned_fcw=cfg_fcw;assign tune_commit=0;assign coarse_ready=1;assign precision_ready=1;
     assign tracking_state=3;assign coarse_reason=0;assign residual0_fcw=0;assign residual1_fcw=0;
     assign tracking_updates=0;assign precision_estimate_valid=0;assign precision_error0=0;assign precision_error1=0;
    end endgenerate
    lockin_core_2ch #(.HANN_WINDOW(1)) u_lockin (
        .clk(clk), .rst_n(rst_n),
        .adc_valid(adc_valid),
        .adc_ch0(adc_ch0), .adc_ch1(adc_ch1),
        .cfg_commit(cfg_commit), .cfg_reconfig(cfg_reconfig), .cfg_enable(cfg_enable),
        .cfg_freq_word(cfg_fcw),
        .cfg_decim_sel(cfg_decim_sel),
        .cfg_integration_log2(cfg_integration_log2),
        .afc_fcw(tuned_fcw), .afc_commit(tune_commit),   // AFC disabled by default in Stage 1; real observer + enable is a later step
        .result_valid(ddc_valid), .result_frame_id(ddc_fid), .result_timestamp(ddc_timestamp),
        .result_i_ch0(ddc_i0), .result_q_ch0(ddc_q0),
        .result_i_ch1(ddc_i1), .result_q_ch1(ddc_q1),
        .amplitude_ch0(ddc_amp0), .amplitude_ch1(ddc_amp1),
        .phase_ch0(ddc_phase0), .phase_ch1(ddc_phase1),
        .phase_valid(ddc_phase_valid), .gap_count(ddc_gaps), .drop_count(ddc_drops),
        .flags(ddc_flags), .active_fcw(active_fcw),
        .nco_phase(nco_phase),
        .mix_valid(mv_mix),.mix_i0(mi0),.mix_q0(mq0),.mix_i1(mi1),.mix_q1(mq1),.mix_time(mt)
    );

    wire signed [15:0] guard_delta0,guard_delta1;
    wire guard_delta_valid;
    lockin_signal_guard #(.MIN_AMPLITUDE_Q12(MIN_AMPLITUDE_Q12)) u_guard (
        .clk(clk), .rst_n(rst_n),
        .invalidate(invalidate),
        .in_valid(ddc_valid),
        .frame_id(ddc_fid),
        .i0(ddc_i0), .q0(ddc_q0), .i1(ddc_i1), .q1(ddc_q1),
        .amp0(ddc_amp0), .amp1(ddc_amp1),
        .phase0(ddc_phase0), .phase1(ddc_phase1),
        .numerical_phase_valid(ddc_phase_valid),
        .out_valid(ddc_publish),
        .detected(ddc_detected), .qualified(ddc_qualified),
        .reason0(guard_reason0),.reason1(guard_reason1),.phase_step0(guard_step0),.phase_step1(guard_step1),
        .phase_delta0(guard_delta0),.phase_delta1(guard_delta1),.phase_delta_valid(guard_delta_valid),
        .threshold0(guard_threshold0),.threshold1(guard_threshold1),.noise0(), .noise1()
    );

    // Capture a complete record, then publish it only after its division.
    reg pending,zero_den,div_start;
    reg signed [15:0] pend_delta0,pend_delta1;
    reg pend_delta_valid;
    reg [39:0] num_r;
    reg [23:0] den_r,pend_amp0,pend_amp1;
    reg signed [15:0] pend_ph0,pend_ph1;
    reg [1:0] pend_detected,pend_qualified;
    reg [31:0] pend_id;
    wire div_busy,div_valid;
    wire [39:0] div_quotient;
    wire ratio_reset_n=rst_n && !invalidate;
    fixed_divider_generic #(.NUM_W(40),.DEN_W(24)) ratio_div(
      .clk(clk),.rst_n(ratio_reset_n),.start(div_start),
      .numerator(num_r),.denominator(den_r),
      .busy(div_busy),.valid(div_valid),.quotient(div_quotient));

    always @(posedge clk) begin
      if(!rst_n || cfg_commit || cfg_reconfig || !cfg_enable || !adc_valid) source_age<=0;
      else if(ddc_valid) source_age<=0;
      else if(source_age!=32'hffffffff) source_age<=source_age+1'b1;
    end
    always @(posedge clk) begin
      lockin_valid<=0;div_start<=0;
      if(!rst_n) begin
        pending<=0;zero_den<=0;num_r<=0;den_r<=0;
        pend_amp0<=0;pend_amp1<=0;pend_ph0<=0;pend_ph1<=0;
        pend_detected<=0;pend_qualified<=0;pend_id<=0;
        pend_delta0<=0;pend_delta1<=0;pend_delta_valid<=0;
        guard_signed_step0<=0;guard_signed_step1<=0;guard_steps_valid<=0;
        amp0_q12<=0;amp1_q12<=0;phase0_turn<=0;phase1_turn<=0;
        phase_diff_turn<=0;ratio_q16<=0;detected<=0;qualified<=0;
        result_present<=0;ratio_valid<=0;result_id<=0;result_age<=0;
        local_flags<=0;local_drops<=0;
      end else begin
        if(result_age!=32'hffffffff) result_age<=result_age+1'b1;
        if(!tracking_ready) begin qualified<=0;guard_steps_valid<=0;end
        if(cfg_commit) begin local_flags<=0;local_drops<=0;end
        if(invalidate) begin
          pending<=0;result_present<=0;ratio_valid<=0;
          detected<=0;qualified<=0;amp0_q12<=0;amp1_q12<=0;
          phase0_turn<=0;phase1_turn<=0;phase_diff_turn<=0;ratio_q16<=0;
          guard_signed_step0<=0;guard_signed_step1<=0;guard_steps_valid<=0;pend_delta_valid<=0;
          // Keep result_id/age as provenance; present=0 prohibits using it.
        end else begin
          if(pending && (zero_den || div_valid)) begin
            amp0_q12<=pend_amp0;amp1_q12<=pend_amp1;
            phase0_turn<=pend_ph0;phase1_turn<=pend_ph1;
            phase_diff_turn<=pend_ph1-pend_ph0;
            ratio_q16<=zero_den?32'd0:((|div_quotient[39:32])?32'hffffffff:div_quotient[31:0]);
            ratio_valid<=!zero_den && !(|div_quotient[39:32]);
            local_flags[6]<=zero_den || (|div_quotient[39:32]);
            detected<=pend_detected;qualified<=tracking_ready?pend_qualified:2'd0;
            guard_signed_step0<=pend_delta0;guard_signed_step1<=pend_delta1;
            guard_steps_valid<=tracking_ready && pend_delta_valid;
            result_id<=pend_id;result_age<=0;result_present<=1;
            lockin_valid<=1;pending<=0;
          end
          if(ddc_publish) begin
            if(!pending && !div_busy) begin
              pend_amp0<=ddc_amp0;pend_amp1<=ddc_amp1;
              pend_ph0<=ddc_phase0;pend_ph1<=ddc_phase1;
              pend_detected<=ddc_detected;pend_qualified<=tracking_ready?ddc_qualified:2'd0;
              pend_delta0<=guard_delta0;pend_delta1<=guard_delta1;pend_delta_valid<=guard_delta_valid;
              pend_id<=ddc_fid;zero_den<=ddc_amp0==0;
              num_r<={ddc_amp1,16'd0};den_r<=ddc_amp0;
              div_start<=ddc_amp0!=0;pending<=1;
            end else begin
              if(local_drops!=32'hffffffff) local_drops<=local_drops+1'b1;
              local_flags[2]<=1;
            end
          end
        end
      end
    end
endmodule
`default_nettype wire
