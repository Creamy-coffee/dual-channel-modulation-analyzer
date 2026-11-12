// =============================================================================
// v32spec_tx.v —— V32SPEC! 频谱帧发送器（AB5 第三帧）
//
// 帧格式（与《频谱帧与窗函数命令》完全一致，小端）：
//   偏移 0..7    magic "V32SPEC!" = 56 33 32 53 50 45 43 21
//   8..11        version      = 1
//   12..15       length       = 44 + bins + 4
//   16..19       wave_frame_id
//   20..23       result_id    每帧 +1
//   24..27       nfft         FFT 点数
//   28..31       bins         本帧 bin 数（正频率半边）
//   32..35       fs_eff_hz    等效采样率
//   36..39       window       0=Hann 1=Blackman 2=FlatTop
//   40..43       scale_dbfs   = 120
//   44..         spectrum     u8[bins]
//   44+bins..    crc32        覆盖前面全部字节（反射多项式 0xEDB88320）
//
// 幅度编码（每 bin 1 字节，见 scripts/generate_spec_tables.py）：
//   dBFS = 20*log10(A/2048)，满刻度 2048 码 = 0 dBFS
//   byte = round((dBFS+120)/120*255)，clamp 0..255
//   FFT 引擎标度 mag = 16*A^2（见 rtl/fft_find_ctrl.v 注释），展开后：
//     byte = 6.39701*log2(mag) + 88.7084
//   实现：前导 1 位置 + 6 位尾数查 64 项 Q8 表 + 一次乘法；
//   与理想曲线最大偏差 1 LSB = 0.47 dB（离线逐点核算，见生成脚本输出）。
//
// 双缓冲：FFT 扫描写入 wbank 时 rbank 正在发送，scan_done 时翻转，
// 因此发送期间新数据不会破坏正在读的帧。bins 必须是 4 的倍数
// （CRC 字段按 4 字节对齐，BINS=2048 满足）。
// =============================================================================
`timescale 1ns / 1ps
`default_nettype none

module v32spec_tx #(
    parameter integer MANUAL_START = 0,
    parameter integer RUNTIME_FS = 0,
    parameter integer UART_STOP_BITS = 1,
    parameter integer BINS       = 2048,        // 本帧携带 bin 数（正频率半边）
    parameter integer NFFT       = 4096,        // FFT 点数，写进头部
    parameter integer CLK_FREQ   = 65_000_000,
    parameter integer BAUD       = 2_000_000,
    parameter integer FS_HZ      = 65_000_000,  // fs_eff
    parameter integer SCALE_DBFS = 120,
    parameter integer MAG_SHIFT  = 4            // mag = 2^MAG_SHIFT * A^2（未直接使用，见注释）
)(
    input  wire        clk,
    input  wire        rst_n,
    // ---- FFT 扫描写入端口（只用 CH0） ------------------------------------
    input  wire        wr_en,
    input  wire [11:0] wr_bin,
    input  wire [32:0] wr_mag,
    // 一次完整扫描结束（bins 个 bin 都写完了），触发发送；busy 期间忽略
    input  wire        scan_done,
    input wire [31:0] snapshot_fs_hz,
    input wire send_start,
    output reg snapshot_ready,
    // ---- 关联与配置字段 --------------------------------------------------
    input  wire [31:0] wave_frame_id,
    input  wire [1:0]  window_sel,
    // ---- 输出 ------------------------------------------------------------
    output wire        tx,
    output wire        busy,
    output reg  [31:0] completed,
    output reg  [31:0] result_id
);

    localparam integer HDR   = 44;
    localparam integer TOTAL = HDR + BINS + 4;

    // ---- 寄存器声明（全部前置，避免前向引用） -----------------------------
    reg  [0:0]  state;                     // 0=IDLE 1=TX
    reg  [12:0] pos;
    reg  [31:0] crc;
    reg         wbank;
    reg         rbank;
    reg  [7:0]  pay_a, pay_b;
    wire [7:0] pay_q = rbank ? pay_b : pay_a;
    reg [31:0] wave_hold;
    reg [1:0] window_hold;
    reg [31:0] fs_hold;

    assign busy = state[0];

    // ---- CRC32（反射，与 V32CAP3 / V32MEAS 相同） -------------------------
    function [31:0] crc_byte;
        input [31:0] c;
        input [7:0]  d;
        integer      k;
        reg   [31:0] v;
        begin
            v = c ^ {24'd0, d};
            for (k = 0; k < 8; k = k + 1)
                v = v[0] ? ((v >> 1) ^ 32'hEDB8_8320) : (v >> 1);
            crc_byte = v;
        end
    endfunction

    // ---- 频谱幅度 -> dBFS 字节 -------------------------------------------
    (* rom_style = "distributed" *) reg [7:0] log2_tab [0:63];
    integer ti;
    initial begin
        for (ti = 0; ti < 64; ti = ti + 1) log2_tab[ti] = 8'd0;
        $readmemh("log2_table_q8.mem", log2_tab);
    end

    // 前导 1 的位置（0..32）
    function [5:0] lead_one;
        input [32:0] v;
        integer      i;
        begin
            lead_one = 6'd0;
            for (i = 0; i < 33; i = i + 1)
                if (v[i]) lead_one = i[5:0];
        end
    endfunction

    wire [5:0]  mag_l   = lead_one(wr_mag);
    wire [39:0] mag_nrm = {{7{1'b0}}, wr_mag} << (6'd31 - mag_l);   // MSB 归到 bit31
    wire [5:0]  mag_m   = mag_nrm[30:25];                           // 隐含 1 之下 6 位
    wire [13:0] l2_q8   = {mag_l, 8'd0} + {6'd0, log2_tab[mag_m]};  // log2(mag) Q8
    // byte = 6.39701*log2(mag) + 88.7084 = (1638*l2_q8 + 5812992) >> 16
    // +32768 = 四舍五入（否则是截断，整体偏低 0.5 LSB）
    (* use_dsp="yes" *) wire [25:0] dbfs_mul = 26'd1638 * l2_q8 + 26'd5812992 + 26'd32768;
    wire [9:0]  dbfs_raw = dbfs_mul[25:16];
    wire [7:0]  enc_new  = (wr_mag == 33'd0) ? 8'd0 :
                           (dbfs_raw > 10'd255) ? 8'd255 : dbfs_raw[7:0];

    // ---- 双缓冲频谱 ------------------------------------------------------
    (* ram_style = "block" *) reg [7:0] bank_a [0:BINS-1];
    (* ram_style = "block" *) reg [7:0] bank_b [0:BINS-1];
    integer bi;
    initial begin
        for (bi = 0; bi < BINS; bi = bi + 1) begin
            bank_a[bi] = 8'd0;
            bank_b[bi] = 8'd0;
        end
    end

    // Read CURRENT payload address. UART is busy for a character after each
    // position increment, giving synchronous RAM time to settle before ready.
    // Reading pos+1 advances the entire spectrum by one bin (CRC still passes).
    wire [12:0] rd_pre  = (pos >= HDR) ? (pos - HDR) : 13'd0;
    wire [11:0] rd_addr = (rd_pre >= BINS) ? (BINS-1) : rd_pre[11:0];

    always @(posedge clk) begin
        if (wr_en && (wr_bin < BINS) && (!MANUAL_START || (!snapshot_ready && !busy))) begin
            if (wbank == 1'b0) bank_a[wr_bin] <= enc_new;
            else               bank_b[wr_bin] <= enc_new;
        end
        // Each bank needs its own synchronous read register. Selecting the
        // memory BEFORE the register prevents Vivado 2018.3 from inferring
        // BRAM and implements this snapshot as hundreds of LUTRAM cells.
        pay_a <= bank_a[rd_addr];
        pay_b <= bank_b[rd_addr];
    end

    // ---- 头部 11 个 32 位字 ----------------------------------------------
    reg [31:0] hdr_word;
    always @* begin
        case (pos[5:2])
            4'd0:    hdr_word = 32'h5332_3356;              // "V32S"
            4'd1:    hdr_word = 32'h2143_4550;              // "PEC!"
            4'd2:    hdr_word = 32'd1;                      // version
            4'd3:    hdr_word = TOTAL;                      // length
            4'd4:    hdr_word = wave_hold;
            4'd5:    hdr_word = result_id;
            4'd6:    hdr_word = NFFT;
            4'd7:    hdr_word = BINS;
            4'd8:    hdr_word = RUNTIME_FS ? fs_hold : FS_HZ;
            4'd9:    hdr_word = {30'd0, window_hold};
            4'd10:   hdr_word = SCALE_DBFS;
            default: hdr_word = 32'd0;
        endcase
    end

    // CRC 字段起点 (HDR+BINS) 必须是 4 的倍数，因此字节 lane 就是 pos[1:0]
    reg  [7:0] byte_data;
    always @* begin
        if (pos < HDR)
            byte_data = hdr_word >> (8 * pos[1:0]);
        else if (pos < HDR + BINS)
            byte_data = pay_q;
        else
            byte_data = (~crc) >> (8 * pos[1:0]);
    end

    // ---- UART ------------------------------------------------------------
    wire tx_ready;
    uart_tx_v32 #(.CLK_FREQ(CLK_FREQ), .BAUD(BAUD), .STOP_BITS(UART_STOP_BITS)) u_uart (
        .clk(clk), .rst_n(rst_n),
        .tx_data(byte_data), .tx_valid(busy), .tx_ready(tx_ready), .tx(tx)
    );

    // ---- 帧状态机 --------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= 1'b0;
            pos       <= 13'd0;
            crc       <= 32'hFFFF_FFFF;
            wbank     <= 1'b0;
            rbank     <= 1'b1;
            completed <= 32'd0;
            result_id <= 32'd0;
            snapshot_ready <= 0;wave_hold<=0;window_hold<=0;fs_hold<=FS_HZ;
        end else if (state == 1'b0) begin
            // Bind metadata to the completed write-bank snapshot, not to a
            // later send_start when the FFT may already be on another band.
            if(scan_done && (!MANUAL_START || !snapshot_ready))
                fs_hold<=RUNTIME_FS ? snapshot_fs_hz : FS_HZ;
            if (MANUAL_START && scan_done) snapshot_ready<=1;
            if ((!MANUAL_START && scan_done) || (MANUAL_START && send_start && snapshot_ready)) begin
                snapshot_ready<=0;wave_hold<=wave_frame_id;window_hold<=window_sel;
                rbank <= wbank;
                wbank <= ~wbank;
                pos   <= 13'd0;
                crc   <= 32'hFFFF_FFFF;
                state <= 1'b1;
            end
        end else begin
            if (tx_ready) begin
                if (pos < HDR + BINS)
                    crc <= crc_byte(crc, byte_data);
                if (pos == TOTAL - 1) begin
                    state     <= 1'b0;
                    pos       <= 13'd0;
                    completed <= completed + 32'd1;
                    result_id <= result_id + 32'd1;
                end else begin
                    pos <= pos + 13'd1;
                end
            end
        end
    end

endmodule

`default_nettype wire
