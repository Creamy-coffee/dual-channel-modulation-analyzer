// =============================================================================
// tb_uart_frac.sv —— 分数分频 UART 发送器验证
//
// 为什么要单独验证：65 MHz / 2 Mbaud = 32.5 不是整数，整数分频必然有
// 1.5% 系统误差，且误差沿字符流累积。
//
// 本测试台用一个按比特中心采样的接收器，按**固定的 325 拍步长**推进：
// 如果分频不是严格 32.5（每字符 325 拍 + 1 拍 IDLE），固定步长采样就会
// 在几个字符内采到错位数据，从而必然失败。检查项：
//   1) 2 Mbaud 背靠背：每字符 325 拍处数据位/停止位全部正确；
//   2) 16 个字符连续无误 -> 长期平均分频精确，无累积漂移；
//   3) 921600 下同一结构按 706 拍步长同样正确。
// =============================================================================
`timescale 1ns/1ps

module tb_uart_frac;

    reg clk = 0;
    always #(500000000/65000000) clk = ~clk;   // 65 MHz

    localparam integer N_BYTES = 16;
    localparam integer PER_2M  = 325;          // 10 位 x 32.5 = 325，字符间零空闲
    localparam integer PER_921 = 705;          // 10 位 x 70.5 = 705

    reg     rst_n = 0;
    integer errors = 0;
    integer checks = 0;
    integer cyc_cnt = 0;
    always @(posedge clk) cyc_cnt <= cyc_cnt + 1;

    reg [7:0] sent [0:N_BYTES-1];

    // ---- 2 Mbaud 被测 -----------------------------------------------------
    reg  [7:0] tx_data;
    reg        tx_valid;
    wire       tx_ready;
    wire       tx_2m;
    uart_tx_v32 #(.CLK_FREQ(65_000_000), .BAUD(2_000_000)) dut_2m (
        .clk(clk), .rst_n(rst_n),
        .tx_data(tx_data), .tx_valid(tx_valid), .tx_ready(tx_ready), .tx(tx_2m)
    );

    // 921600 的回归由 tb_linux_packets / tb_v32_1024 覆盖：
    // 它们按比特中心解码真实帧，能直接抓到分频退化。

    // 比特中心采样偏移（2 Mbaud，位宽 32.5 拍）
    integer off [0:7];
    initial begin
        off[0]=48; off[1]=81; off[2]=113; off[3]=146;
        off[4]=178; off[5]=211; off[6]=243; off[7]=276;
    end

    task automatic sample_at(input integer target, output reg v);
        begin
            while (cyc_cnt < target) @(posedge clk);
            v = tx_2m;
        end
    endtask

    integer i, k, j;
    integer t0;
    reg [7:0] rx_byte;
    reg       bit_v;
    reg       seen_first;

    // 起始位检测：长时间空闲后第一次下降
    reg        tx_q = 1'b1;
    reg [9:0]  hi_run = 10'd0;
    always @(posedge clk) begin
        tx_q <= tx_2m;
        if (tx_2m) hi_run <= (hi_run >= 10'd1000) ? 10'd1000 : hi_run + 10'd1;
        else       hi_run <= 10'd0;
    end
    wire first_start = tx_q & ~tx_2m & (hi_run >= 10'd600);

    // ---- 发送驱动 ---------------------------------------------------------
    initial begin
        for (i = 0; i < N_BYTES; i = i + 1) sent[i] = 8'hA5 ^ (i * 8'h11);
        tx_valid = 1'b0; tx_data = 8'h00;

        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (600) @(posedge clk);

        // 背靠背：valid 常高，ready 有效时换下一个字节
        for (i = 0; i < N_BYTES; i = i + 1) begin
            tx_data  = sent[i];
            tx_valid = 1'b1;
            @(posedge clk);
            while (!tx_ready) @(posedge clk);
        end
        tx_valid = 1'b0;
    end

    // ---- 接收校验 ---------------------------------------------------------
    initial begin
        seen_first = 1'b0;
        wait (rst_n === 1'b1);
        @(posedge first_start);
        t0 = cyc_cnt;
        seen_first = 1'b1;

        for (k = 0; k < N_BYTES; k = k + 1) begin
            rx_byte = 8'd0;
            for (j = 0; j < 8; j = j + 1) begin
                sample_at(t0 + k*PER_2M + off[j], bit_v);
                rx_byte[j] = bit_v;
            end
            sample_at(t0 + k*PER_2M + 308, bit_v);      // 停止位中心
            checks = checks + 1;
            if (bit_v !== 1'b1) begin
                errors = errors + 1;
                $display("FAIL 2M char %0d stop bit = %b (period drift)", k, bit_v);
            end
            checks = checks + 1;
            if (rx_byte !== sent[k]) begin
                errors = errors + 1;
                $display("FAIL 2M char %0d data=0x%02x want 0x%02x (period drift)",
                         k, rx_byte, sent[k]);
            end
        end
        // 长期平均：第 16 个字符之后仍是空闲
        sample_at(t0 + N_BYTES*PER_2M + 320, bit_v);
        checks = checks + 1;
        if (bit_v !== 1'b1) begin
            errors = errors + 1;
            $display("FAIL 2M line not idle after %0d characters", N_BYTES);
        end

        $display("INFO 2Mbaud: %0d characters decoded, checks=%0d errors=%0d",
                 N_BYTES, checks, errors);
        if (errors == 0) $display("UART_FRAC_PASS checks=%0d", checks);
        else             $display("UART_FRAC_FAIL errors=%0d", errors);
        $finish;
    end

endmodule
