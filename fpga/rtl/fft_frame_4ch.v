`timescale 1ns/1ps
`default_nettype none
// Self-contained, time-multiplexed radix-2 DIT FFT. No Xilinx FFT IP.
// Capture N simultaneous real samples at the input sample strobe, then
// process enabled channels sequentially. Samples arriving during compute
// are intentionally not part of an FFT frame; this is a block analyzer,
// NOT a continuous full-rate spectrum stream. Time-domain metrics remain
// independent and continue running. Input is signed 12 bit, scaled <<4.
// Each butterfly scales by 1/2, yielding a total FFT normalization of 1/N.
// Search bins 1..N/2 (DC excluded; Nyquist included). No window/interpolation.
// Peaks can be harmonics; the result is not a guaranteed fundamental.
module fft_frame_4ch #(
    parameter integer CHANNELS=4,
    parameter integer CONVERGENT_ROUNDING=1,
    parameter integer LOG2_N=13,
    parameter integer ADC_FS_HZ=65000000,
    parameter integer DECIM_LOG2=0,
    parameter integer FRAME_PERIOD_CLKS=5000000,
    parameter ROM_FILE="twiddle_8192.mem"
)(
    input wire clk,input wire rst_n,
    input wire sample_valid,input wire [47:0] samples,
    output wire busy,
    output reg result_valid,
    output reg [511:0] result_packet,
    output reg saturated,
    output reg spectrum_valid,
    output reg [1:0] spectrum_channel,
    output reg [LOG2_N-1:0] spectrum_bin,
    output reg [32:0] spectrum_power,
    // Per-channel peak complex value and 3-point interpolated frequency.
    output reg [63:0] peak_re_packed,
    output reg [63:0] peak_im_packed,
    output reg [127:0] interp_freq_packed,
    output wire scan_saturated
);
    localparam integer N=1<<LOG2_N;
    localparam integer HALF_N=N/2;
    localparam [3:0] IDLE=0,CAPTURE=1,PRIME=2,LOAD=3,MULT=4,
        ROTATE=5,BUTTER=6,WRITE=7,NEXT=8,SCAN_PRIME=9,SCAN_EVAL=10,PUBLISH=11,
        INTERP_WAIT=12,INTERP_DONE=13,INTERP_CALC=14,INTERP_START=15;
    reg [3:0] state;
    reg [31:0] countdown;
    reg [LOG2_N-1:0] capture_index,scan_index;
    reg [LOG2_N:0] half_span,block_base,j;
    reg [3:0] stage;
    reg [1:0] channel;
    reg [511:0] work_packet;
    reg frame_saturated;
    assign scan_saturated=frame_saturated;
    reg [32:0] peak_mag;
    reg [LOG2_N-1:0] peak_bin;
    reg signed [15:0] peak_re, peak_im;
    reg [32:0] prev_mag, peak_left, peak_right;
    // Multi-cycle interpolation division + common-fundamental-bin phase support.
    reg signed [48:0] interp_num_hold, interp_den_hold;
    reg [32:0] interp_left_hold,interp_center_hold,interp_right_hold;
    reg [LOG2_N-1:0] final_bin_hold;
    reg div_start;
    reg signed [16:0] delta_q16;
    reg signed [15:0] ch0_peak_re, ch0_peak_im;
    reg [LOG2_N-1:0] ref_bin;
    reg signed [15:0] ref_ch1_re, ref_ch1_im;
    function [LOG2_N-1:0] reverse_bits;
        input [LOG2_N-1:0] value;
        integer b;
        begin for(b=0;b<LOG2_N;b=b+1) reverse_bits[b]=value[LOG2_N-1-b];end
    endfunction
    wire scanning=(state==SCAN_PRIME || state==SCAN_EVAL);
    wire [LOG2_N:0] butterfly_a=block_base+j;
    wire [LOG2_N:0] butterfly_b=block_base+j+half_span;
    wire [LOG2_N-1:0] addr_a=(state==CAPTURE)?reverse_bits(capture_index):
        (scanning?scan_index:butterfly_a[LOG2_N-1:0]);
    wire [LOG2_N-1:0] addr_b=butterfly_b[LOG2_N-1:0];
    wire [31:0] qa[0:CHANNELS-1];
    wire [31:0] qb[0:CHANNELS-1];
    reg [31:0] write_a,write_b;
    genvar g;
    generate for(g=0;g<CHANNELS;g=g+1) begin: memories
        wire capture_we=(state==CAPTURE && sample_valid);
        wire fft_we=(state==WRITE && channel==g);
        fft_tdp_ram #(.AW(LOG2_N)) ram(
            .clk(clk),.addr_a(addr_a),.addr_b(addr_b),
            .we_a(capture_we || fft_we),.we_b(fft_we),
            .din_a(capture_we?{16'd0,samples[g*12 +:12],4'd0}:write_a),
            .din_b(write_b),.dout_a(qa[g]),.dout_b(qb[g]));
    end endgenerate
    wire [LOG2_N+10:0] twiddle_wide={{10{1'b0}},j} << (LOG2_N-stage);
    wire [LOG2_N-2:0] twiddle_addr=twiddle_wide[LOG2_N-2:0];
    wire [31:0] twiddle;
    fft_twiddle_rom #(.ADDR_W(LOG2_N-1),.ROM_FILE(ROM_FILE)) rom(.clk(clk),.addr(twiddle_addr),.q(twiddle));
    reg signed [15:0] ar,ai,br,bi,wr,wi;
    (* use_dsp="yes" *) reg signed [31:0] prr,pii,pri,pir;
    reg signed [17:0] tr,ti;
    wire signed [32:0] rotate_re={prr[31],prr}-{pii[31],pii};
    wire signed [32:0] rotate_im={pri[31],pri}+{pir[31],pir};
    wire signed [32:0] rotated_re=(rotate_re >>> 15)+
        ((CONVERGENT_ROUNDING && (rotate_re[14:0]>15'd16384 || (rotate_re[14:0]==15'd16384 && rotate_re[15])))?33'sd1:33'sd0);
    wire signed [32:0] rotated_im=(rotate_im >>> 15)+
        ((CONVERGENT_ROUNDING && (rotate_im[14:0]>15'd16384 || (rotate_im[14:0]==15'd16384 && rotate_im[15])))?33'sd1:33'sd0);
    wire signed [18:0] ar_ext={{3{ar[15]}},ar};
    wire signed [18:0] ai_ext={{3{ai[15]}},ai};
    wire signed [18:0] tr_ext={tr[17],tr};
    wire signed [18:0] ti_ext={ti[17],ti};
    wire signed [18:0] plus_re_sum=ar_ext+tr_ext;
    wire signed [18:0] plus_re=(plus_re_sum >>> 1)+
        ((CONVERGENT_ROUNDING && plus_re_sum[0] && plus_re_sum[1])?19'sd1:19'sd0);
    wire signed [18:0] plus_im_sum=ai_ext+ti_ext;
    wire signed [18:0] plus_im=(plus_im_sum >>> 1)+
        ((CONVERGENT_ROUNDING && plus_im_sum[0] && plus_im_sum[1])?19'sd1:19'sd0);
    wire signed [18:0] minus_re_sum=ar_ext-tr_ext;
    wire signed [18:0] minus_re=(minus_re_sum >>> 1)+
        ((CONVERGENT_ROUNDING && minus_re_sum[0] && minus_re_sum[1])?19'sd1:19'sd0);
    wire signed [18:0] minus_im_sum=ai_ext-ti_ext;
    wire signed [18:0] minus_im=(minus_im_sum >>> 1)+
        ((CONVERGENT_ROUNDING && minus_im_sum[0] && minus_im_sum[1])?19'sd1:19'sd0);
    function [15:0] clip16;
        input signed [18:0] x;
        begin
            if(x>19'sd32767) clip16=16'h7FFF;
            else if(x< -19'sd32768) clip16=16'h8000;
            else clip16=x[15:0];
        end
    endfunction
    function clipped;
        input signed [18:0] x;
        begin clipped=(x>19'sd32767 || x< -19'sd32768);end
    endfunction
    wire signed [15:0] scan_re=qa[channel][15:0];
    wire signed [15:0] scan_im=qa[channel][31:16];
    (* use_dsp="yes" *) wire signed [31:0] scan_re2=scan_re*scan_re;
    (* use_dsp="yes" *) wire signed [31:0] scan_im2=scan_im*scan_im;
    wire [32:0] scan_mag={1'b0,scan_re2}+{1'b0,scan_im2};
    wire new_peak=(scan_index!=0 && scan_mag>peak_mag);
    wire [32:0] final_mag=new_peak?scan_mag:peak_mag;
    wire [LOG2_N-1:0] final_bin=new_peak?scan_index:peak_bin;
    wire signed [15:0] final_re=new_peak?scan_re:peak_re;
    wire signed [15:0] final_im=new_peak?scan_im:peak_im;
    wire [32:0] final_left=new_peak?prev_mag:peak_left;
    wire [32:0] final_right=new_peak?33'd0:peak_right;
    // Parabolic interpolation in Q16 bins: delta=0.5*(L-R)/(L-2C+R).
    // 49-bit operands: final_left/right/mag are 33-bit; the <<15 shift needs up
    // to 48 magnitude bits + sign, so 36-bit truncation would corrupt the result.
    wire signed [48:0] interp_num =
        ($signed({{15{1'b0}},interp_left_hold}) - $signed({{15{1'b0}},interp_right_hold})) <<< 15;
    wire signed [48:0] interp_den =
        $signed({{15{1'b0}},interp_left_hold}) - ($signed({{14{1'b0}},interp_center_hold}) <<< 1)
        + $signed({{15{1'b0}},interp_right_hold});
    // The division is multi-cycle (fixed_divider below): operands are latched at
    // scan end and consumed in INTERP_WAIT / INTERP_DONE. delta is clamped to
    // +/-0.5 in Q16 (== +/-32768) so a broken spectrum cannot produce delta=3.7.
    wire div_valid, div_busy;
    wire [47:0] div_q;
    wire [47:0] div_num = interp_num_hold[48] ? (~interp_num_hold[47:0] + 48'd1) : interp_num_hold[47:0];
    wire [47:0] div_den = interp_den_hold[48] ? (~interp_den_hold[47:0] + 48'd1) : interp_den_hold[47:0];
    wire result_neg = interp_num_hold[48] ^ interp_den_hold[48];
    wire signed [48:0] delta_wide = result_neg ? -{1'b0, div_q} : {1'b0, div_q};
    fixed_divider #(.NUM_W(48), .DEN_W(48)) u_interp_div(
        .clk(clk), .rst_n(rst_n), .start(div_start),
        .numerator(div_num), .denominator(div_den),
        .busy(div_busy), .valid(div_valid), .quotient(div_q));
    // Preserve fractional analysis Fs: bin * ADC_FS / (N * 2^DECIM_LOG2).
    (* use_dsp="yes" *) wire [63:0] frequency_num={{(64-LOG2_N){1'b0}},final_bin_hold}*ADC_FS_HZ;
    wire [63:0] frequency_rounded=(frequency_num+
        (64'd1 << (LOG2_N+DECIM_LOG2-1))) >> (LOG2_N+DECIM_LOG2);
    // Interpolated peak frequency in Hz, computed at FULL width.
    // The original one-liner
    //   ((bin<<16)+delta)*ADC_FS_HZ >>> (16+LOG2_N+DECIM_LOG2)
    // assigned into a 32-bit packet field, and Verilog context widths then
    // evaluated the product in 32 bits BEFORE the shift: for a 1.59 MHz tone
    // the wrapped product read back as -5 Hz (verified in simulation).  The
    // wide signed wires below force the multiply and the shift to happen at
    // 48/64 bits; only the final [31:0] slice is truncated (Fs <= 2^31, so
    // the result always fits).
    wire signed [47:0] interp_pos =
        ($signed({{(48-LOG2_N){1'b0}}, final_bin_hold}) <<< 16) + delta_q16;
    (* use_dsp="yes" *) wire signed [63:0] interp_prod = interp_pos * ADC_FS_HZ;
    wire signed [63:0] interp_hz   =
        interp_prod >>> (16+LOG2_N+DECIM_LOG2);
    assign busy=(state!=IDLE);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;countdown<=0;capture_index<=0;scan_index<=0;
            half_span<=1;block_base<=0;j<=0;stage<=1;channel<=0;
            ar<=0;ai<=0;br<=0;bi<=0;wr<=0;wi<=0;
            prr<=0;pii<=0;pri<=0;pir<=0;tr<=0;ti<=0;
            write_a<=0;write_b<=0;peak_mag<=0;peak_bin<=0;peak_re<=0;peak_im<=0;
            prev_mag<=0;peak_left<=0;peak_right<=0;
            interp_num_hold<=0;interp_den_hold<=0;interp_left_hold<=0;interp_center_hold<=0;interp_right_hold<=0;final_bin_hold<=0;div_start<=0;delta_q16<=0;
            ch0_peak_re<=0;ch0_peak_im<=0;ref_bin<=0;ref_ch1_re<=0;ref_ch1_im<=0;
            work_packet<=0;result_packet<=0;result_valid<=0;
            frame_saturated<=0;saturated<=0;
            spectrum_valid<=0;spectrum_channel<=0;spectrum_bin<=0;spectrum_power<=0;
            peak_re_packed<=0;peak_im_packed<=0;interp_freq_packed<=0;
        end else begin
            result_valid<=0;spectrum_valid<=0;
            if(countdown!=0) countdown<=countdown-1'b1;
            case(state)
                IDLE: if(countdown==0) begin
                    capture_index<=0;frame_saturated<=0;work_packet<=0;
                    countdown<=FRAME_PERIOD_CLKS;state<=CAPTURE;
                end
                CAPTURE: if(sample_valid) begin
                    if(capture_index==N-1) begin
                        channel<=0;stage<=1;half_span<=1;block_base<=0;j<=0;
                        state<=PRIME;
                    end else capture_index<=capture_index+1'b1;
                end
                PRIME: state<=LOAD;
                LOAD: begin
                    ar<=qa[channel][15:0];ai<=qa[channel][31:16];
                    br<=qb[channel][15:0];bi<=qb[channel][31:16];
                    wr<=twiddle[15:0];wi<=twiddle[31:16];state<=MULT;
                end
                MULT: begin
                    prr<=br*wr;pii<=bi*wi;pri<=br*wi;pir<=bi*wr;state<=ROTATE;
                end
                ROTATE: begin
                    // Exact unity avoids repeated 32767/32768 loss at j=0.
                    tr<=(j==0)?{{2{br[15]}},br}:rotated_re[17:0];
                    ti<=(j==0)?{{2{bi[15]}},bi}:rotated_im[17:0];state<=BUTTER;
                end
                BUTTER: begin
                    write_a<={clip16(plus_im),clip16(plus_re)};
                    write_b<={clip16(minus_im),clip16(minus_re)};
                    if(clipped(plus_im)||clipped(plus_re)||clipped(minus_im)||clipped(minus_re))
                        frame_saturated<=1;
                    state<=WRITE;
                end
                WRITE: state<=NEXT;
                NEXT: begin
                    if(j+1<half_span) begin j<=j+1'b1;state<=PRIME;end
                    else if(block_base+(half_span<<1)<N) begin
                        j<=0;block_base<=block_base+(half_span<<1);state<=PRIME;
                    end else if(stage<LOG2_N) begin
                        j<=0;block_base<=0;stage<=stage+1'b1;half_span<=half_span<<1;
                        state<=PRIME;
                    end else begin
                        scan_index<=0;peak_mag<=0;peak_bin<=0;peak_re<=0;peak_im<=0;
                        prev_mag<=0;peak_left<=0;peak_right<=0;state<=SCAN_PRIME;
                    end
                end
                SCAN_PRIME: state<=SCAN_EVAL;
                SCAN_EVAL: begin
                    spectrum_valid<=1;spectrum_channel<=channel;
                    spectrum_bin<=scan_index;spectrum_power<=scan_mag;
                    if(new_peak) begin
                        peak_mag<=scan_mag;peak_bin<=scan_index;peak_re<=scan_re;peak_im<=scan_im;
                        peak_left<=prev_mag;peak_right<=0;
                    end else if(scan_index==peak_bin+1'b1) begin
                        peak_right<=scan_mag;
                    end
                    // Phase must use a COMMON fundamental bin: capture CH1's
                    // complex value at CH0's peak bin while scanning CH1.
                    if(channel==1 && scan_index==ref_bin) begin
                        ref_ch1_re<=scan_re;ref_ch1_im<=scan_im;
                    end
                    prev_mag<=scan_mag;
                    if(scan_index==HALF_N) begin
                        peak_re_packed[channel*16 +:16]<=final_re;
                        peak_im_packed[channel*16 +:16]<=final_im;
                        if(channel==0) begin
                            ch0_peak_re<=final_re;ch0_peak_im<=final_im;ref_bin<=final_bin;
                        end
                        if(channel==1) begin
                            // CH1 at CH0's fundamental bin (overrides CH1's own peak).
                            peak_re_packed[31:16]<=ref_ch1_re;
                            peak_im_packed[31:16]<=ref_ch1_im;
                        end
                        work_packet[channel*128+32 +:32]<={{(32-LOG2_N){1'b0}},final_bin};
                        work_packet[channel*128+64 +:32]<=final_mag[31:0];
                        work_packet[channel*128+96 +:32]<={31'd0,final_mag[32]};
                        final_bin_hold<=final_bin;
                        interp_left_hold<=final_left;
                        interp_center_hold<=final_mag;
                        interp_right_hold<=final_right;
                        state<=INTERP_CALC;
                    end else begin scan_index<=scan_index+1'b1;state<=SCAN_PRIME;end
                end
                // Break RAM->power->peak->49bit arithmetic->zero detect.
                // No new exceptions: the arithmetic is physically pipelined.
                INTERP_CALC: begin
                    work_packet[channel*128 +:32]<=frequency_rounded[31:0];
                    interp_num_hold<=interp_num;interp_den_hold<=interp_den;
                    state<=INTERP_START;
                end
                INTERP_START: begin
                    div_start<=(interp_den_hold!=0);state<=INTERP_WAIT;
                end
                INTERP_WAIT: begin
                    div_start<=0;
                    if(interp_den_hold==0) begin
                        delta_q16<=0;state<=INTERP_DONE;
                    end else if(div_valid) begin
                        if(delta_wide>17'sd32768) delta_q16<=17'sd32768;
                        else if(delta_wide< -17'sd32768) delta_q16<=-17'sd32768;
                        else delta_q16<=delta_wide[16:0];
                        state<=INTERP_DONE;
                    end
                end
                INTERP_DONE: begin
                    // 64-bit signed result; only the low 32 bits are stored.
                    interp_freq_packed[channel*32 +:32]<=interp_hz[31:0];
                    if(channel==CHANNELS-1) state<=PUBLISH;
                    else begin
                        channel<=channel+1'b1;stage<=1;half_span<=1;block_base<=0;j<=0;
                        state<=PRIME;
                    end
                end
                PUBLISH: begin
                    result_packet<=work_packet;saturated<=frame_saturated;
                    result_valid<=1;state<=IDLE;
                end
                default: state<=IDLE;
            endcase
        end
    end
    // synthesis translate_off
    initial begin
        if((CHANNELS!=2 && CHANNELS!=4)||LOG2_N<2||LOG2_N>13||DECIM_LOG2<0||DECIM_LOG2>12||
            ADC_FS_HZ<1||FRAME_PERIOD_CLKS<0) $fatal(1,"Invalid FFT parameters");
    end
    // synthesis translate_on
endmodule

// -------------------------------------------------------------------------
// Shared multi-cycle restoring divider. One quotient bit per clock instead of
// a giant combinational divide network. Unsigned operands; callers handle
// sign. Used by fft_frame_4ch (interpolation) and digital_lock_in (DDS FCW).
// -------------------------------------------------------------------------
module fixed_divider #(
    parameter integer NUM_W = 48,
    parameter integer DEN_W = 48
)(
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire [NUM_W-1:0] numerator,
    input wire [DEN_W-1:0] denominator,
    output reg busy,
    output reg valid,
    output reg [NUM_W-1:0] quotient
);
    reg [NUM_W-1:0] rem, quo;
    reg [DEN_W-1:0] den;
    reg [$clog2(NUM_W):0] count;
    wire [NUM_W:0] rem_shifted = {rem, numerator[NUM_W-1-count]};
    wire q_bit = (rem_shifted >= {1'b0, den});
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            busy<=0;valid<=0;rem<=0;quo<=0;den<=0;count<=0;quotient<=0;
        end else begin
            valid<=0;
            if(start && !busy) begin
                rem<=0;quo<=0;den<=denominator;count<=0;busy<=1;
            end else if(busy) begin
                rem<= q_bit ? (rem_shifted[NUM_W-1:0]-den) : rem_shifted[NUM_W-1:0];
                quo<={quo[NUM_W-2:0], q_bit};
                if(count==NUM_W-1) begin
                    busy<=0;valid<=1;
                    quotient<= q_bit ? {quo[NUM_W-2:0],1'b1} : {quo[NUM_W-2:0],1'b0};
                end else count<=count+1'b1;
            end
        end
    end
endmodule
`default_nettype wire
