`timescale 1ns/1ps
`default_nettype none
// =============================================================================
// iq_stream_tx.v —— 解调 I/Q 数据流帧发送器（DDC 输出 -> AB5 UART）
//
// 帧格式（小端，魔数字符串除外）：
//   offset  0 : 8B  魔数 "V32IQ01!" = 56 33 32 49 51 30 31 21
//   offset  8 : 4B  version        = 1
//   offset 12 : 4B  frame_id
//   offset 16 : 4B  fs_hz          = 65000000
//   offset 20 : 4B  decimation     = 4096
//   offset 24 : 4B  points         = 256（本帧 I/Q 复数对数）
//   offset 28 : 4B  flags          bit0=溢出标志（自上一帧起有 FIFO 丢样）
//   offset 32 : 4B  cfg_id         （解调配置编号，Linux 侧据此校验参数一致）
//   offset 36 : 4B  reserved
//   offset 40 : points*4B  {I int16, Q int16}，I 在前
//   offset 40+4*points : 4B CRC32，覆盖 [0, 40+4*points)，低字节先发
//   总长 1068 B（POINTS=256），无帧尾。
//
// FIFO 深度 512 字（2 KB BRAM）。帧发送 5.36 ms（2 Mbaud），数据积累
// 16.13 ms（15869 sps x 256），占空比 ~33%，链路余量充足；溢出时丢样
// 并置帧头 flags.bit0，Linux 侧据此清除同步并重新捕获。
// =============================================================================
module iq_stream_tx #(
    parameter integer POINTS      = 256,       // I/Q pairs per frame
    parameter integer FIFO_DEPTH  = 512,       // 32-bit words
    parameter integer CLK_FREQ    = 65_000_000,
    parameter integer BAUD        = 2_000_000,
    parameter integer FS_HZ       = 65_000_000,
    parameter integer DECIM       = 4096,
    parameter integer RUNTIME_RATE = 0,
    // Compatibility mitigation for byte loss seen in board captures.
    // Adds UART idle margin, not a proven cure for host/adapter loss.
    parameter integer UART_STOP_BITS = 2
)(
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire                     iq_valid,   // FIR output sample pulse
    input  wire signed [15:0]       i_in,
    input  wire signed [15:0]       q_in,
    input  wire [31:0]              cfg_id,     // config revision stamped in header
    input  wire [1:0]               rate_sel,
    input tri1 send_grant,
    output wire frame_ready,
    output wire [$clog2(FIFO_DEPTH):0] fifo_level,
    output wire                     tx,         // to AB5 UART (v32_tx)
    output wire                     busy,
    output reg  [31:0]              completed,
    output reg  [31:0]              overflow     // total dropped samples
);
    localparam [2:0] ST_IDLE = 3'd0, ST_HDR = 3'd1, ST_READ = 3'd2,
                     ST_DATA = 3'd3, ST_TAIL = 3'd4;
    localparam [7:0] M0 = 8'h56, M1 = 8'h33, M2 = 8'h32, M3 = 8'h49,
                     M4 = 8'h51, M5 = 8'h30, M6 = 8'h31, M7 = 8'h21;
    localparam AW = $clog2(FIFO_DEPTH);

    reg [2:0]  state;
    reg [AW:0] wcnt;                // FIFO occupancy, single writer
    reg [AW-1:0] wrp, rdp;
    reg [AW:0] frame_left;          // words remaining in the current frame
    reg [6:0]  pos;                 // header/data byte position
    reg [31:0] frame_id;
    reg [31:0] word_q;
    reg [31:0] cfg_hold;
    reg [31:0] decim_hold;
    reg        oflag_hold;
    reg        oflag_pending;   // only FIFO-write always drives this
    reg        oflag_clear;     // only FSM always drives this
    reg [31:0] crc_lfsr;
    reg [2:0]  crc_bc;          // bit counter 0..7
    reg        crc_active;
    reg [7:0]  crc_byte;
    wire pop = (state == ST_READ);
    wire push = iq_valid && !fifo_full;

    // 32-bit word = {Q[15:0], I[15:0]}
    (* ram_style = "block" *) reg [31:0] fifo [0:FIFO_DEPTH-1];

    wire fifo_full  = (wcnt == FIFO_DEPTH);
    wire fifo_empty = (wcnt == 0);
    wire have_frame = (wcnt >= POINTS);
    wire sending    = (state == ST_HDR) || (state == ST_DATA) || (state == ST_TAIL);

    // ---- UART ----
    reg  [7:0] byte_data;
    wire       tx_ready;
    uart_tx_v32 #(.CLK_FREQ(CLK_FREQ), .BAUD(BAUD), .STOP_BITS(UART_STOP_BITS)) u_uart(
        .clk(clk), .rst_n(rst_n), .tx_data(byte_data), .tx_valid(sending && send_grant),
        .tx_ready(tx_ready), .tx(tx));

    assign busy = (state != ST_IDLE);
    assign frame_ready=have_frame && state==ST_IDLE && rst_n;
    assign fifo_level=wcnt;

    // ---- FIFO write ----
    always @(posedge clk) begin
        if(!rst_n) begin
            wrp <= 0; wcnt <= 0; overflow <= 0; oflag_pending <= 1'b0;
        end else begin
            case ({push,pop})
                2'b10: wcnt <= wcnt + 1'b1;
                2'b01: wcnt <= wcnt - 1'b1;
                default: wcnt <= wcnt;
            endcase
            if(oflag_clear) oflag_pending <= 1'b0;
            if(push) begin
                fifo[wrp] <= {q_in, i_in};
                wrp       <= wrp + 1'b1;
            end else if(iq_valid && fifo_full) begin
                overflow       <= overflow + 32'd1;
                oflag_pending  <= 1'b1;
            end
        end
    end

    // ---- header words ----
    reg [31:0] header_word;
    always @* begin
        case (pos[5:2])
            4'd2:    header_word = 32'd1;            // version
            4'd3:    header_word = frame_id;         // frame_id
            4'd4:    header_word = FS_HZ;            // fs_hz
            4'd5:    header_word = decim_hold;
            4'd6:    header_word = POINTS;           // points
            4'd7:    header_word = {31'd0, oflag_hold}; // flags
            4'd8:    header_word = cfg_hold;         // cfg_id
            default: header_word = 32'd0;            // reserved
        endcase
    end

    // ---- current byte (combinational) ----
    always @* begin
        byte_data = 8'h00;
        if(state == ST_HDR) begin
            if(pos < 6'd8) begin
                case (pos[2:0])
                    3'd0: byte_data = M0;
                    3'd1: byte_data = M1;
                    3'd2: byte_data = M2;
                    3'd3: byte_data = M3;
                    3'd4: byte_data = M4;
                    3'd5: byte_data = M5;
                    3'd6: byte_data = M6;
                    3'd7: byte_data = M7;
                endcase
            end else begin
                byte_data = header_word >> (8 * pos[1:0]);
            end
        end else if(state == ST_DATA) begin
            byte_data = word_q >> (8 * pos[1:0]);    // I low/high, Q low/high
        end else if(state == ST_TAIL) begin
            byte_data = (~crc_lfsr) >> (8 * pos[1:0]);
        end
    end

    // ---- CRC32 (reflected poly 0xEDB88320) as a bit-serial LFSR state
    // machine: 8 clocks per byte, run in parallel with UART transmission
    // (UART takes hundreds of FPGA clocks/byte at 2 Mbaud).
    // reset at frame start, run only while header/data bytes are sent
    // (ST_TAIL carries the computed CRC and must not feed it back)
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            crc_lfsr  <= 32'hFFFF_FFFF;
            crc_bc    <= 3'd0;
            crc_active<= 1'b0;
            crc_byte <= 0;
        end else if(state == ST_IDLE && have_frame && send_grant) begin
            crc_lfsr  <= 32'hFFFF_FFFF;
            crc_active<= 1'b0;
        end else if(tx_ready && send_grant && (state == ST_HDR || state == ST_DATA)) begin
            crc_active <= 1'b1;
            crc_bc     <= 3'd0;
            crc_byte   <= byte_data;
        end else if(crc_active) begin
            if(crc_lfsr[0] ^ crc_byte[crc_bc])
                crc_lfsr <= {1'b0, crc_lfsr[31:1]} ^ 32'hEDB8_8320;
            else
                crc_lfsr <= {1'b0, crc_lfsr[31:1]};
            if(crc_bc == 3'd7) crc_active <= 1'b0;
            else               crc_bc     <= crc_bc + 1'b1;
        end
    end

    // ---- main FSM ----
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state <= ST_IDLE;
            pos   <= 7'd0;
                        frame_id <= 32'd0;
            word_q   <= 32'd0;
            cfg_hold <= 32'd0;
            decim_hold <= DECIM;
            oflag_hold <= 1'b0;
            completed <= 32'd0;
            rdp <= 0; frame_left <= 0;
            oflag_clear <= 1'b0;
        end else begin
            oflag_clear <= 1'b0;
            case(state)
                ST_IDLE: begin
                    if(have_frame && send_grant) begin
                        cfg_hold   <= cfg_id;
                        decim_hold <= RUNTIME_RATE ? (rate_sel==0 ? 8192 : rate_sel==2 ? 2048 : 4096) : DECIM;
                        oflag_hold <= oflag_pending;
                        oflag_clear <= 1'b1;
                        state <= ST_HDR;
                        pos   <= 7'd0;
                                                frame_left <= POINTS;
                        frame_id <= frame_id + 32'd1;
                    end
                end
                ST_HDR: begin
                    if(tx_ready && send_grant) begin
                                                if(pos == 7'd39) begin
                            state <= ST_READ;
                            pos   <= 7'd0;
                        end else begin
                            pos <= pos + 1'b1;
                        end
                    end
                end
                ST_READ: begin
                    // pop one word; give the BRAM a cycle
                    word_q <= fifo[rdp];
                    rdp    <= rdp + 1'b1;
                    frame_left <= frame_left - 1'b1;
                    state  <= ST_DATA;
                    pos    <= 7'd0;
                end
                ST_DATA: begin
                    if(tx_ready && send_grant) begin
                                                if(pos == 7'd3) begin
                            pos <= 7'd0;
                            if(frame_left == 0) begin
                                state <= ST_TAIL;
                            end else begin
                                state <= ST_READ;
                            end
                        end else begin
                            pos <= pos + 1'b1;
                        end
                    end
                end
                ST_TAIL: begin
                    if(tx_ready && send_grant) begin
                        if(pos == 7'd3) begin
                            pos   <= 7'd0;
                            state <= ST_IDLE;
                            completed <= completed + 32'd1;
                        end else begin
                            pos <= pos + 1'b1;
                        end
                    end
                end
                default: state <= ST_IDLE;
            endcase
        end
    end
    // synthesis translate_off
    initial begin
        if(POINTS<1 || POINTS>FIFO_DEPTH || FIFO_DEPTH<2 ||
           (FIFO_DEPTH & (FIFO_DEPTH-1))!=0 || CLK_FREQ/BAUD<2)
            $fatal(1,"Invalid IQ FIFO/UART parameters");
    end
    // synthesis translate_on
endmodule
`default_nettype wire
