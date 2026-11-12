`timescale 1ns/1ps
`default_nettype none
// Full-rate DDC, CIC3 with modular 58-bit arithmetic, binomial FIR, integration.
// IQ units: ADC counts * 2^12, including the mixer 1/2 gain.
// Polar amplitude compensates the 1/2 gain; phase uses Q=-sin (positive input phase).
// FIR [1,4,6,4,1]/16 is a baseband low-pass, NOT a CIC droop compensation filter.
module lockin_core_2ch #(
    parameter SIN_FILE="sin_lut_q15.mem", COS_FILE="cos_lut_q15.mem",
    parameter integer HANN_WINDOW=0
)(
    input wire clk,rst_n,
    input wire adc_valid,
    input wire signed [11:0] adc_ch0,adc_ch1,
    input wire cfg_commit,cfg_reconfig,cfg_enable,
    input wire [31:0] cfg_freq_word,
    // Phase-continuous AFC microtune: afc_commit updates fcw ONLY, never resets
    // phase (so the NCO phase stays continuous across frequency corrections).
    input wire [31:0] afc_fcw,
    input wire afc_commit,
    input wire [1:0] cfg_decim_sel,
    input wire [4:0] cfg_integration_log2,
    output reg result_valid,
    output reg [31:0] result_frame_id,result_timestamp,
    output reg signed [23:0] result_i_ch0,result_q_ch0,result_i_ch1,result_q_ch1,
    output reg [23:0] amplitude_ch0,amplitude_ch1,
    output reg signed [15:0] phase_ch0,phase_ch1,
    output reg [1:0] phase_valid,
    output reg [31:0] gap_count,drop_count,
    output reg [7:0] flags,
    output wire mix_valid,
    output wire signed [27:0] mix_i0,mix_q0,mix_i1,mix_q1,
    output wire [31:0] mix_time,
    output wire [31:0] active_fcw,
    // Live NCO phase accumulator.  At a cycle with adc_valid=1 this is the
    // phase that multiplies the sample arriving on that same cycle (the
    // accumulator is read before its increment), so an external consumer can
    // pair (sample, phase) exactly.  Used by et_display for phase binning.
    output wire [31:0] nco_phase
);
    (* rom_style="block" *) reg signed [15:0] sin_rom[0:1023],cos_rom[0:1023];
    initial begin $readmemh(SIN_FILE,sin_rom);$readmemh(COS_FILE,cos_rom);end
    reg [31:0] phase,fcw,ticks;
    reg enabled,running;
    reg [3:0] decim_log2;
    reg [4:0] integration_log2;
    reg signed [15:0] sn,cs;
    reg signed [11:0] x0,x1;
    reg v_lookup,v_mix;
    (* use_dsp="yes" *) reg signed [27:0] mix[0:3];
    reg signed [57:0] s1[0:3],s2[0:3],s3[0:3];
    reg signed [57:0] delay1[0:3],delay2[0:3],delay3[0:3];
    reg signed [57:0] comb1[0:3],comb2[0:3],comb3[0:3];
    reg [9:0] decim_count;
    reg cv1,cv2,cv3,fv;
    reg signed [23:0] hist1[0:3],hist2[0:3],hist3[0:3],hist4[0:3];
    reg signed [27:0] fir[0:3];
    // Accumulate full Q15 products; round only after the complete window.
    // Mean window weight is 1, preserving the existing IQ/amplitude units.
    reg signed [57:0] accum[0:3];
    (* use_dsp="yes" *) reg signed [41:0] window_product[0:3];
    reg signed [23:0] window_sample[0:3];
    reg product_pulse,product_end;
    reg [31:0] product_timestamp;
    reg integration_pulse,integration_end;
    reg [31:0] integration_timestamp;
    (* rom_style="block" *) reg signed [15:0] window_rom[0:1023];
    initial $readmemh(COS_FILE,window_rom);
    // NOTE (Vivado front-end): 'count' must be DECLARED BEFORE its first use
    // in the window_index continuous assignment below (xvlog VRFC 10-3380).
    reg [15:0] count;
    wire [9:0] window_index=integration_log2>=10 ?
        (count>>(integration_log2-10)) : (count<<(10-integration_log2));
    wire [9:0] window_index_next=window_index+10'd1;
    reg signed [15:0] window_cos,window_cos_next,interpolated_cos;
    reg [5:0] window_fraction;
    reg [2:0] window_shift;
    wire signed [16:0] window_delta=$signed({window_cos_next[15],window_cos_next})-
                                     $signed({window_cos[15],window_cos});
    // Linear interpolation removes zero-order-hold images of the 1024-word ROM.
    (* use_dsp48="no" *) wire signed [23:0] interpolation_product=
        window_delta*$signed({1'b0,window_fraction});
    wire [5:0] interpolation_round=(window_shift==0)?6'd0:(6'd1<<(window_shift-1'b1));
    wire signed [24:0] interpolation_rounded=
        {interpolation_product[23],interpolation_product}+$signed({19'd0,interpolation_round});
    // cos=-32768 yields +65536: signed18 is required at the Hann peak.
    wire signed [17:0] window_weight=HANN_WINDOW ?
        (18'sd32768-$signed({{2{interpolated_cos[15]}},interpolated_cos})) : 18'sd32768;
    reg signed [23:0] averaged[0:3];
    reg [3:0] warmup;
    reg [31:0] pending_timestamp;
    reg [2:0] polar_state;
    wire p_ready,p_valid,p_phase_valid;
    wire [23:0] p_amp;
    wire signed [15:0] p_phase;
    wire issue=(polar_state==1 || polar_state==3) && p_ready;
    wire channel1=(polar_state==3);
    wire signed [23:0] avg_i=channel1?averaged[2]:averaged[0];
    wire signed [23:0] avg_q=channel1?averaged[3]:averaged[1];
    wire signed [39:0] polar_i={{15{avg_i[23]}},avg_i,1'b0};
    wire signed [39:0] polar_q={{15{avg_q[23]}},avg_q,1'b0};
    // Upper bound = ADC Nyquist (fcw(32.5 MHz) = 0x80000000); was 0x76276276
    // = FCW(30 MHz) exactly, which rejected any fine-tuned reference >30 MHz.
    wire cfg_ok=(cfg_freq_word>=32'd66 && cfg_freq_word<32'h80000000 &&
                 cfg_integration_log2>=3 && cfg_integration_log2<=16);
    wire reset_filter=cfg_commit || cfg_reconfig || !enabled || !adc_valid;
    wire local_polar_reset=rst_n && !reset_filter;
    wire [10:0] decim_last=(11'd1<<decim_log2)-1'b1;
    wire [16:0] integration_last=(17'd1<<integration_log2)-1'b1;
    wire [5:0] normalize_shift=decim_log2*3+3;
    assign active_fcw=fcw;
    assign nco_phase=phase;
    assign mix_valid=v_mix && !cfg_commit && enabled && adc_valid;
    assign mix_i0=mix[0];assign mix_q0=mix[1];assign mix_i1=mix[2];assign mix_q1=mix[3];
    assign mix_time=ticks;
    function signed [23:0] sat24;
        input signed [57:0] a;
        begin
            if(a>58'sd8388607) sat24=24'sh7fffff;
            else if(a< -58'sd8388608) sat24=24'sh800000;
            else sat24=a[23:0];
        end
    endfunction
    ai_iq_polar_cordic #(.INPUT_SHIFT(0)) polar(
        .clk(clk),.rst_n(local_polar_reset),.in_valid(issue),.in_ready(p_ready),
        .i_sum(polar_i),.q_sum(polar_q),.frame_id_in(16'd0),.branch_in(channel1),
        .out_valid(p_valid),.amp_q12(p_amp),.phase_turn(p_phase),
        .phase_valid(p_phase_valid),.frame_id_out(),.branch_out());
    // Physical-sample NCO: after acquisition starts, advance across valid gaps.
    always @(posedge clk) begin
        sn<=sin_rom[phase[31:22]];cs<=cos_rom[phase[31:22]];
        x0<=adc_ch0;x1<=adc_ch1;
        mix[0]<=x0*cs;mix[1]<=-(x0*sn);
        mix[2]<=x1*cs;mix[3]<=-(x1*sn);
    end
    // Mixer observation stays continuous through a baseband-only reconfigure.
    // CIC/measurement valid pipeline is flushed separately below.
    always @(posedge clk) begin
        if(!rst_n || cfg_commit || !enabled || !adc_valid) begin v_lookup<=0;v_mix<=0;end
        else begin v_lookup<=adc_valid;v_mix<=v_lookup;end
    end
    integer k;
    reg signed [23:0] norm;
    reg signed [27:0] a,b,c,d,e;
    reg signed [57:0] sum_next;
    always @(posedge clk) begin
        if(cv3) begin
            window_cos<=window_rom[window_index];
            window_cos_next<=window_rom[window_index_next];
            window_shift<=integration_log2>10 ? integration_log2-10 : 0;
            window_fraction<=integration_log2>10 ? (count & ((16'd1<<(integration_log2-10))-1'b1)) : 0;
        end
        if(!rst_n) begin
            phase<=0;fcw<=32'h03f03f04;enabled<=1;running<=0;
            decim_log2<=4;integration_log2<=8;ticks<=0;
            gap_count<=0;drop_count<=0;flags<=0;result_frame_id<=0;
            result_timestamp<=0;result_valid<=0;
            result_i_ch0<=0;result_q_ch0<=0;result_i_ch1<=0;result_q_ch1<=0;
            amplitude_ch0<=0;amplitude_ch1<=0;phase_ch0<=0;phase_ch1<=0;phase_valid<=0;
        end else begin
            ticks<=ticks+1'b1;result_valid<=0;
            if(enabled && (running || adc_valid)) begin phase<=phase+fcw;running<=1;end
            if(running && enabled) begin
                if(!adc_valid) begin
                    flags[0]<=1;
                    if(gap_count!=32'hffffffff) gap_count<=gap_count+1'b1;
                end else flags[0]<=0;
                // Gap COUNT is historical. The current-stream fault clears on
                // return; filter/guard/AFC still must rebuild a fresh result.
            end
            if(cfg_commit) begin
                phase<=0;running<=0;fcw<=cfg_freq_word;enabled<=cfg_enable && cfg_ok;
                // Decimation is applied to the MIXED (baseband) signal — i.e.
                // AFTER the quadrature mixer that precedes the CIC (lines 112-115).
                // With a matched reference the wanted component sits near DC, so
                // the low-pass / decimation bandwidth only needs to clear the
                // baseband, NOT the carrier. This is standard DDC: 30 MHz in x
                // 30 MHz ref -> DC term -> decimate. Any reference up to Nyquist
                // (32.5 MHz) works at the original R16 default; forcing R1 was
                // wrong and also shrank the integration time ~16x (breaking the
                // like-for-like comparison the user asked to preserve).
                decim_log2<=4+{cfg_decim_sel,1'b0};
                integration_log2<=cfg_integration_log2;
                flags<={6'd0,!cfg_ok,1'b0};gap_count<=0;drop_count<=0;phase_valid<=0;
            end else begin
              if(cfg_reconfig) begin
                // Bandwidth (gear) change: update ONLY the CIC decimation ratio
                // and integration window.  The NCO phase/fcw and the AFC observer
                // are preserved so the carrier stays phase-continuous and the
                // fine-tracking loop does NOT lose lock on a bandwidth step.
                // A genuine frequency change still uses the cfg_commit branch
                // above, which resets phase and re-latches the reference.
                decim_log2<=4+{cfg_decim_sel,1'b0};
                integration_log2<=cfg_integration_log2;
              end
              // Independent enables: a gear change cannot swallow a tune.
              if(afc_commit && enabled &&
                        (afc_fcw>=32'd66 && afc_fcw<32'h80000000)) begin
                // Phase-continuous AFC microtune: update the NCO phase increment
                // WITHOUT resetting 'phase'. This is the only path that keeps the
                // NCO phase continuous across frequency corrections.
                fcw<=afc_fcw;
              end
            end
        end
        if(!rst_n || reset_filter) begin
            decim_count<=0;cv1<=0;cv2<=0;cv3<=0;fv<=0;
            count<=0;warmup<=0;polar_state<=0;pending_timestamp<=0;
            integration_pulse<=0;integration_end<=0;integration_timestamp<=0;
            product_pulse<=0;product_end<=0;product_timestamp<=0;interpolated_cos<=0;
            for(k=0;k<4;k=k+1) begin
                s1[k]<=0;s2[k]<=0;s3[k]<=0;delay1[k]<=0;delay2[k]<=0;delay3[k]<=0;
                comb1[k]<=0;comb2[k]<=0;comb3[k]<=0;
                hist1[k]<=0;hist2[k]<=0;hist3[k]<=0;hist4[k]<=0;fir[k]<=0;
                accum[k]<=0;averaged[k]<=0;window_product[k]<=0;window_sample[k]<=0;
            end
        end else begin
            cv1<=0;cv2<=cv1;cv3<=cv2;fv<=cv3;
            integration_pulse<=0;
            product_pulse<=0;
            if(v_mix) begin
                for(k=0;k<4;k=k+1) begin
                    // All integrator stages use OLD registers; never saturate them.
                    s1[k]<=s1[k]+{{30{mix[k][27]}},mix[k]};
                    s2[k]<=s2[k]+s1[k];s3[k]<=s3[k]+s2[k];
                    if(decim_count==decim_last[9:0]) begin
                        comb1[k]<=s3[k]+s2[k]-delay1[k];delay1[k]<=s3[k]+s2[k];
                    end
                end
                if(decim_count==decim_last[9:0]) begin decim_count<=0;cv1<=1;end
                else decim_count<=decim_count+1'b1;
            end
            if(cv1) for(k=0;k<4;k=k+1) begin
                comb2[k]<=comb1[k]-delay2[k];delay2[k]<=comb1[k];
            end
            if(cv2) for(k=0;k<4;k=k+1) begin
                comb3[k]<=comb2[k]-delay3[k];delay3[k]<=comb2[k];
            end
            if(cv3) for(k=0;k<4;k=k+1) begin
                norm=sat24(comb3[k]>>>normalize_shift);
                a={{4{norm[23]}},norm};b={{4{hist1[k][23]}},hist1[k]};
                c={{4{hist2[k][23]}},hist2[k]};d={{4{hist3[k][23]}},hist3[k]};
                e={{4{hist4[k][23]}},hist4[k]};
                fir[k]<=a+(b<<<2)+(c<<<2)+(c<<<1)+(d<<<2)+e;
                hist1[k]<=norm;hist2[k]<=hist1[k];hist3[k]<=hist2[k];hist4[k]<=hist3[k];
            end
            if(fv) begin
                if(warmup!=15) warmup<=warmup+1'b1;
                else begin
                    interpolated_cos<=$signed(window_cos)+(interpolation_rounded>>>window_shift);
                    for(k=0;k<4;k=k+1) window_sample[k]<=fir[k][27:4];
                    product_pulse<=1;product_end<=(count==integration_last[15:0]);
                    product_timestamp<=ticks;
                    if(count==integration_last[15:0]) count<=0;
                    else count<=count+1'b1;
                end
            end
            if(product_pulse) begin
                for(k=0;k<4;k=k+1) window_product[k]<=window_sample[k]*window_weight;
                integration_pulse<=1;integration_end<=product_end;
                integration_timestamp<=product_timestamp;
            end
            if(integration_pulse) begin
                    for(k=0;k<4;k=k+1) begin
                        sum_next=accum[k]+{{16{window_product[k][41]}},window_product[k]};
                        if(integration_end) begin
                            if(polar_state==0) averaged[k]<=sum_next>>>(integration_log2+15);
                            accum[k]<=0;
                        end else accum[k]<=sum_next;
                    end
                    if(integration_end) begin
                        if(polar_state==0) begin polar_state<=1;pending_timestamp<=integration_timestamp;end
                        else begin
                            flags[2]<=1;
                            if(drop_count!=32'hffffffff) drop_count<=drop_count+1'b1;
                        end
                    end
            end
            case(polar_state)
                1:if(issue) polar_state<=2;
                2:if(p_valid) begin
                    amplitude_ch0<=p_amp;phase_ch0<=p_phase;phase_valid[0]<=p_phase_valid;
                    polar_state<=3;
                end
                3:if(issue) polar_state<=4;
                4:if(p_valid) begin
                    amplitude_ch1<=p_amp;phase_ch1<=p_phase;phase_valid[1]<=p_phase_valid;
                    result_i_ch0<=averaged[0];result_q_ch0<=averaged[1];
                    result_i_ch1<=averaged[2];result_q_ch1<=averaged[3];
                    result_frame_id<=result_frame_id+1'b1;result_timestamp<=pending_timestamp;
                    result_valid<=1;polar_state<=0;
                end
                default:begin end
            endcase
        end
    end
endmodule
`default_nettype wire
