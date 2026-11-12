`timescale 1ns / 1ps
`default_nettype none
// AD9238 capture: PARALLEL_PORTS=1 uses separate SDR buses.
// The comments below describe the legacy PARALLEL_PORTS=0 mux mode.
// The chip has TWO 12-bit output ports. This design connects ONE port
// and assumes the board enables multiplexing using MUX_SELECT.
// DFS selects offset-binary versus two's-complement; it does NOT enable mux.
// CLK_A, CLK_B, MUX_SELECT, selected output port, and external propagation
// delay determine channel/edge mapping. They are not controlled by this RTL.
// SAME_EDGE pairs a rising sample with the preceding falling sample;
// do not assume simultaneous A/B acquisition without confirming board timing.
// The startup wait and pipeline flush suppress early data after reset/lock.
module ad9238_capture #(
    parameter integer CHANNELS        = 2,
    parameter integer ADC_CLK_DIV     = 2,
    parameter integer STARTUP_CYCLES  = 500000,
    parameter integer PIPELINE_FLUSH  = 16,
    parameter integer OFFSET_BINARY   = 1,
    parameter integer USE_EXT_CLK     = 1,
    parameter integer PARALLEL_PORTS = 0, // 1 = separate SDR data buses
    parameter integer USE_IDDR        = 1,
    parameter integer DEBUG           = 0        // 1=keep ILA mark_debug nets
)(
    input  wire clk,
    input  wire rst_n,
    input  wire adc_clk_ext,
    input  wire mmcm_locked,
    input  wire [11:0] adc_d0, adc_d1, adc_d2, adc_d3,
    input  wire [3:0] adc_otr,
    output wire adc_clk,
    output reg [47:0] samples,
    output reg [3:0] sample_otr,
    output reg sample_valid,
    output reg ready
);
    // Debug nets — only kept when DEBUG=1 to avoid blocking optimization
    generate
    if (DEBUG == 1) begin : g_debug
        (* mark_debug = "true" *) wire dbg_adc_clk = adc_clk;
        (* mark_debug = "true" *) wire [47:0] dbg_samples = samples;
        (* mark_debug = "true" *) wire dbg_valid = sample_valid;
        (* mark_debug = "true" *) wire dbg_ready = ready;
    end
    endgenerate

    function [11:0] to_signed;
        input [11:0] x;
        begin
            // Offset-binary 0x800 is zero; XOR is the exact conversion.
            to_signed = OFFSET_BINARY ? (x ^ 12'h800) : x;
        end
    endfunction

    generate
    if (USE_EXT_CLK) begin : g_ext
        reg [31:0] startup_count;
        reg [7:0] flush_count;
        reg warmed;
        wire local_rst_n = rst_n && mmcm_locked;

        // adc_clk_ext is the capture/processing clock. The physical top
        // may forward a phase-shifted clock to satisfy the ADC round trip.
        assign adc_clk = adc_clk_ext;

        // --------------------------------------------------------------
        // Dual-edge capture using IDDR (guaranteed IOB placement)
        //
        // IDDR in SAME_EDGE mode outputs both edges' data on the
        // rising edge of CLK, so downstream logic runs entirely on
        // posedge.  q1 = rising-edge sample (Channel A),
        // q2 = falling-edge sample (Channel B, from previous half-cycle).
        // --------------------------------------------------------------
        wire [11:0] iddr_q1, iddr_q2;
        wire [11:0] ch_a_raw, ch_b_raw;
        wire [3:0] captured_otr;

        if (PARALLEL_PORTS) begin : g_parallel
            // Capture both physical buses on the SAME rising edge.
            // No reset in IOB registers: startup/flush masks stale data.
            (* IOB = "TRUE" *) reg [11:0] a_ff, b_ff;
            (* IOB = "TRUE" *) reg [1:0] otr_ff;
            always @(posedge adc_clk_ext) begin
                a_ff <= adc_d0;
                b_ff <= adc_d1;
                otr_ff <= adc_otr[1:0];
            end
            assign ch_a_raw = a_ff;
            assign ch_b_raw = b_ff;
            assign captured_otr = {2'b00, otr_ff};
        end
        else if (USE_IDDR) begin : g_iddr
            genvar i;
            for (i = 0; i < 12; i = i + 1) begin : g_iddr_bits
                IDDR #(
                    .DDR_CLK_EDGE("SAME_EDGE"),
                    .INIT_Q1(1'b0),
                    .INIT_Q2(1'b0),
                    .SRTYPE("SYNC")
                ) u_iddr (
                    .Q1(iddr_q1[i]),
                    .Q2(iddr_q2[i]),
                    .C(adc_clk_ext),
                    .CE(1'b1),
                    .D(adc_d0[i]),
                    // Startup + flush masks stale data; no datapath reset.
                    // Avoid a half-cycle path from the global reset tree.
                    .R(1'b0),
                    .S(1'b0)
                );
            end
            assign captured_otr = adc_otr;
            assign ch_a_raw = iddr_q1;  // posedge data → Channel A
            assign ch_b_raw = iddr_q2;  // negedge data → Channel B
        end
        else begin : g_regular_ff
            // Fallback: regular flip-flops (no IDDR instantiation)
            reg [11:0] ch_a_ff, ch_b_ff, ch_b_same_edge;
            // Match IDDR SAME_EDGE latency, including its Q2 rising-edge
            // retiming stage. A direct ch_b_ff output differs by one cycle.
            always @(posedge adc_clk_ext or negedge local_rst_n)
                if (!local_rst_n) begin
                    ch_a_ff <= 12'd0;
                    ch_b_same_edge <= 12'd0;
                end else begin
                    ch_a_ff <= adc_d0;
                    ch_b_same_edge <= ch_b_ff;
                end
            always @(negedge adc_clk_ext or negedge local_rst_n)
                if (!local_rst_n) ch_b_ff <= 12'd0;
                else ch_b_ff <= adc_d0;
            assign captured_otr = adc_otr;
            assign ch_a_raw = ch_a_ff;
            assign ch_b_raw = ch_b_same_edge;
        end

        // Main control + output on posedge
        // Both ch_a_raw and ch_b_raw are stable at posedge in IDDR
        // SAME_EDGE mode, so we can pack them directly.
        always @(posedge adc_clk_ext or negedge local_rst_n) begin
            if (!local_rst_n) begin
                startup_count <= 0;
                flush_count   <= 0;
                warmed        <= 1'b0;
                ready         <= 1'b0;
                samples       <= 48'd0;
                sample_otr    <= 4'd0;
                sample_valid  <= 1'b0;
            end else begin
                sample_valid <= 1'b0;

                if (!warmed) begin
                    if (STARTUP_CYCLES == 0 || startup_count >= STARTUP_CYCLES-1) begin
                        warmed <= 1'b1;
                    end else begin
                        startup_count <= startup_count + 1'b1;
                    end
                end

                if (warmed) begin
                    if (flush_count < PIPELINE_FLUSH) begin
                        flush_count <= flush_count + 1'b1;
                    end else begin
                        // Pack: CH_A in [11:0], CH_B in [23:12]
                        // Upper lanes zeroed (2-channel mode)
                        samples[11:0]  <= to_signed(ch_a_raw);
                        samples[23:12] <= to_signed(ch_b_raw);
                        samples[35:24] <= (CHANNELS == 4) ? to_signed(adc_d2) : 12'd0;
                        samples[47:36] <= (CHANNELS == 4) ? to_signed(adc_d3) : 12'd0;
                        sample_otr     <= captured_otr & ((CHANNELS == 4) ? 4'hF : 4'h3);
                        sample_valid   <= 1'b1;
                        ready          <= 1'b1;
                    end
                end
            end
        end
    end
    else begin : g_div
        localparam integer HALF = ADC_CLK_DIV/2;
        localparam integer DIV_W = (HALF < 2) ? 1 : $clog2(HALF);
        reg [DIV_W-1:0] div_count;
        reg [31:0] startup_count;
        reg [7:0] flush_count;
        reg warmed;
        reg adc_clk_r;
        assign adc_clk = adc_clk_r;

        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin
                div_count<=0; startup_count<=0; flush_count<=0; warmed<=0;
                adc_clk_r<=0; samples<=0; sample_otr<=0; sample_valid<=0; ready<=0;
            end else begin
                sample_valid<=0;
                if(!warmed) begin
                    if(STARTUP_CYCLES==0 || startup_count>=STARTUP_CYCLES-1)
                        warmed<=1'b1;
                    else startup_count<=startup_count+1'b1;
                end
                if(div_count==HALF-1) begin
                    div_count<=0; adc_clk_r<=~adc_clk_r;
                    if(adc_clk_r && warmed) begin
                        if(flush_count<PIPELINE_FLUSH) flush_count<=flush_count+1'b1;
                        else begin
                            samples[11:0]<=to_signed(adc_d0);
                            samples[23:12]<=to_signed(adc_d1);
                            samples[35:24]<=(CHANNELS==4)?to_signed(adc_d2):12'd0;
                            samples[47:36]<=(CHANNELS==4)?to_signed(adc_d3):12'd0;
                            sample_otr<=adc_otr & ((CHANNELS==4)?4'hF:4'h3);
                            sample_valid<=1'b1; ready<=1'b1;
                        end
                    end
                end else div_count<=div_count+1'b1;
            end
        end
    end
    endgenerate

    // synthesis translate_off
    initial begin
        if ((PARALLEL_PORTS && CHANNELS != 2) ||
            (CHANNELS != 2 && CHANNELS != 4) ||
            (ADC_CLK_DIV < 2) || (ADC_CLK_DIV % 2 != 0) ||
            (PIPELINE_FLUSH < 7) || (PIPELINE_FLUSH > 255))
            $fatal(1, "Invalid AD9238 capture parameters");
    end
    // synthesis translate_on
endmodule
`default_nettype wire
