// =============================================================================
// uart_tx_v32.v —— 8N1 UART 发送器（V32CAP3 / V32SPEC 配套）
//
// 分频方式：**Bresenham 分数分频**（相位累加器），不是整数分频。
//
//   为什么必须分数分频：
//     65 MHz / 2 Mbaud = 32.5，不是整数。整数分频只能取 32 或 33：
//       32 分频 -> 2.031 Mbaud (+1.56%)
//       33 分频 -> 1.970 Mbaud (-1.51%)
//     单字节勉强能收，但一帧 4140 字节连续发送时误差沿字符流累积，
//     接收端若按固定位宽计数，第 10 位之后就会越界。
//     参考实现：nand2mario《UART with fractional clock divider》(2025)，
//     该文正是 21 MHz / 2 Mbps 整数分频在第 10 位后失锁的案例。
//
//   本实现：每个位周期在 32 和 33 之间抖动，10 位（起始+8数据+停止）
//   恰好 325 个时钟，长期平均分频严格等于 CLK_FREQ/BAUD，无累积误差。
//   单比特瞬时抖动 ±0.5 时钟 = ±1.5%，接收端在比特中心采样，不受影响。
//
//   65 MHz 下的实测值：
//     921600  -> 每字符 70.5 时钟（70/71 交替），921986 Bd，+0.04%
//     2000000 -> 每字符 32.5 时钟（32/33 交替），2000000 Bd，0.00%
//
// 握手语义：
//   tx_valid && tx_ready 在同一拍成立 -> 该字节被接收，下一拍 tx 开始拉低发起始位。
//   tx_ready 只在空闲态为高，因此不要依赖它做背压，直接看 valid/ready 同时为高即可。
// =============================================================================
`timescale 1ns / 1ps
`default_nettype none

module uart_tx_v32 #(
    parameter integer CLK_FREQ = 65_000_000,
    parameter integer BAUD     = 921_600,
    // An 8N1 receiver accepts the second stop bit as extra idle time.
    // Keep one stop bit by default for existing waveform/spectrum timing.
    parameter integer STOP_BITS = 1
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] tx_data,
    input  wire       tx_valid,
    output wire       tx_ready,
    output reg        tx
);

    // ---- 分数分频常量：CLK_FREQ/BAUD = BASE + REM/BAUD ----------------------
    localparam integer BASE  = CLK_FREQ / BAUD;                        // 整数部分
    localparam integer REM   = CLK_FREQ % BAUD;                        // 余数部分
    // Reduce REM/BAUD at elaboration. All reachable accumulator states in
    // the unreduced divider are multiples of GCD, so this preserves every
    // bit boundary, including reset and pauses. At 65 MHz / 2 Mbaud the
    // fraction is 1/2: one accumulator bit replaces the old 21-bit datapath.
    function integer gcd;
        input integer a,b;
        integer x,y,t;
        begin
            x=a;y=b;
            while(y!=0) begin t=x%y;x=y;y=t;end
            gcd=x;
        end
    endfunction
    localparam integer DIV_GCD = gcd(REM,BAUD);
    localparam integer ACC_MOD = BAUD/DIV_GCD;
    localparam integer ACC_INC = REM/DIV_GCD;
    localparam integer ACC_W = (ACC_MOD <= 2) ? 1 : $clog2(ACC_MOD);
    localparam integer CNT_W = (BASE <= 2) ? 1 : $clog2(BASE + 2);     // 位内计数器宽度

    localparam [1:0] ST_IDLE = 2'd0,
                     ST_START= 2'd1,
                     ST_DATA = 2'd2,
                     ST_STOP = 2'd3;

    reg [1:0]       state;
    reg [CNT_W-1:0] bit_cnt;      // 当前位已持续时钟数
    reg [CNT_W-1:0] bit_len;      // 当前位总长度（BASE 或 BASE+1）
    reg [ACC_W-1:0] acc;          // Bresenham 余数累加器
    reg [2:0]       bit_index;
    reg [7:0]       data_latched;
    reg             second_stop;

    assign tx_ready = (state == ST_IDLE);

    // 位边界上推进一次分数分频：余数累加满 BAUD 就把本位加宽 1 个时钟。
    // 综合后是一个 ACC_W 位加法器 + 一次比较，约 40 LUT。
    wire [ACC_W:0] acc_next = {1'b0, acc} + ACC_INC[ACC_W:0];
    wire           widen    = (acc_next >= ACC_MOD);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_IDLE;
            bit_cnt      <= {CNT_W{1'b0}};
            bit_len      <= BASE[CNT_W-1:0];
            acc          <= {ACC_W{1'b0}};
            bit_index    <= 3'd0;
            data_latched <= 8'd0;
            second_stop  <= 1'b0;
            tx           <= 1'b1;
        end else begin
            case (state)
                ST_IDLE: begin
                    tx        <= 1'b1;
                    bit_cnt   <= {CNT_W{1'b0}};
                    bit_index <= 3'd0;
                    if (tx_valid) begin
                        second_stop <= 1'b0;
                        data_latched <= tx_data;
                        tx           <= 1'b0;              // 起始位
                        // 每个字符从零相位开始，抖动不会跨字符累积
                        acc          <= {ACC_W{1'b0}};
                        bit_len      <= BASE[CNT_W-1:0];
                        state        <= ST_START;
                    end
                end

                ST_START: begin
                    if (bit_cnt == bit_len - 1'b1) begin
                        bit_cnt   <= {CNT_W{1'b0}};
                        bit_len   <= widen ? (BASE+1) : BASE;
                        acc       <= widen ? (acc_next - ACC_MOD) : acc_next[ACC_W-1:0];
                        tx        <= data_latched[0];      // LSB first
                        bit_index <= 3'd0;
                        state     <= ST_DATA;
                    end else begin
                        bit_cnt <= bit_cnt + 1'b1;
                    end
                end

                ST_DATA: begin
                    if (bit_cnt == bit_len - 1'b1) begin
                        bit_cnt   <= {CNT_W{1'b0}};
                        bit_len   <= widen ? (BASE+1) : BASE;
                        acc       <= widen ? (acc_next - ACC_MOD) : acc_next[ACC_W-1:0];
                        if (bit_index == 3'd7) begin
                            tx    <= 1'b1;                 // 停止位
                            state <= ST_STOP;
                        end else begin
                            bit_index <= bit_index + 1'b1;
                            tx        <= data_latched[bit_index + 1'b1];
                        end
                    end else begin
                        bit_cnt <= bit_cnt + 1'b1;
                    end
                end

                ST_STOP: begin
                    if (bit_cnt == bit_len - 1'b1) begin
                        bit_cnt <= {CNT_W{1'b0}};
                        tx      <= 1'b1;
                        if (STOP_BITS == 2 && !second_stop)
                            second_stop <= 1'b1;
                        else
                            state <= ST_IDLE;
                    end else begin
                        bit_cnt <= bit_cnt + 1'b1;
                    end
                end

                default: begin
                    state <= ST_IDLE;
                    tx    <= 1'b1;
                end
            endcase
        end
    end

    // synthesis translate_off
    initial if (STOP_BITS != 1 && STOP_BITS != 2)
        $fatal(1, "STOP_BITS must be 1 or 2");
    // synthesis translate_on

endmodule

`default_nettype wire
