// =============================================================================
// tb_v32spec.sv —— V32SPEC! 频谱帧结构性验证
//
// 做法：给一个合成的幅度斜坡（bin0 最强，往后递减），等一帧发完后
// 在 UART 线上按 32.5 拍的比特中心把 112 个字节全部解出来（每字符 325 拍），落到
// tests/v32spec_wire.bin，再由 scripts/validate_v32spec.py 逐字段核对
// （magic/version/length/ids/nfft/bins/fs/window/scale/payload/CRC32）。
//
// BINS=64 只是为了让仿真在几秒内跑完；帧结构、编码器、CRC 与
// BINS=2048 完全同一套代码。
// =============================================================================
`timescale 1ns/1ps

module tb_v32spec;

    localparam integer BINS  = 64;
    localparam integer NFFT  = 4096;
    localparam integer HDR   = 44;
    localparam integer TOTAL = HDR + BINS + 4;      // 112
    localparam integer PER   = 325;                 // 2 Mbaud 每字符 10*32.5=325 拍（字符间零空闲）

    reg clk = 0;
    always #(500000000/65000000) clk = ~clk;

    reg         rst_n = 0;
    reg         wr_en = 0;
    reg  [11:0] wbin = 0;
    reg  [32:0] wr_mag = 0;
    reg         wrun = 0;
    integer     wa;
    reg         scan_done = 0;
    reg  [31:0] wave_id = 32'd1001;
    reg  [1:0]  window_sel = 2'd1;                  // Blackman，对齐交接包示例

    wire        tx, busy;
    wire [31:0] completed, result_id;

    v32spec_tx #(
        .BINS(BINS), .NFFT(NFFT), .CLK_FREQ(65_000_000), .BAUD(2_000_000),
        .FS_HZ(65_000_000), .SCALE_DBFS(120)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .wr_en(wr_en), .wr_bin(wbin), .wr_mag(wr_mag),
        .scan_done(scan_done),
        .wave_frame_id(wave_id), .window_sel(window_sel),
        .tx(tx), .busy(busy), .completed(completed), .result_id(result_id)
    );

    // ---- 起始位检测（长时间空闲后第一次下降） ----------------------------
    reg        tx_q = 1'b1;
    reg [11:0] hi_run = 12'd0;
    always @(posedge clk) begin
        tx_q <= tx;
        if (tx) hi_run <= (hi_run >= 12'd4000) ? 12'd4000 : hi_run + 12'd1;
        else    hi_run <= 12'd0;
    end
    wire first_start = tx_q & ~tx & (hi_run >= 12'd2000);

    integer off [0:7];
    initial begin
        off[0]=48; off[1]=81; off[2]=113; off[3]=146;
        off[4]=178; off[5]=211; off[6]=243; off[7]=276;
    end

    integer cyc = 0;
    always @(posedge clk) cyc <= cyc + 1;

    task automatic sample_at(input integer target, output reg v);
        begin
            while (cyc < target) @(posedge clk);
            v = tx;
        end
    endtask

    // ---- 波形跳变记录（离线用真正的 UART 解码器复核） --------------------
    integer tr;
    initial tr = $fopen("../tests/v32spec_trace.txt", "w");
    always @(tx) $fwrite(tr, "%0d %0d\n", cyc, tx);

    // ---- 激励 -------------------------------------------------------------
    // wbin 与 wr_mag 必须**同一拍寄存**：否则 DUT 在时钟沿可能采到
    // 新的地址配旧的幅度（delta 竞争），出现隔一个 bin 才写进去的现象。
    function [32:0] mag_of;
        input [11:0] b;
        integer      aa;
        begin
            // Sharp changes catch a one-bin RAM address slip that a smooth
            // ramp can hide inside the allowed 1-LSB encoding tolerance.
            case(b[2:0])
             0:aa=0;1:aa=1;2:aa=8;3:aa=64;
             4:aa=256;5:aa=1024;6:aa=2047;default:aa=300;
            endcase
            mag_of = (33'd1 << 4) * aa * aa;   // mag = 16*A^2
        end
    endfunction

    initial begin
        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (2100) @(posedge clk);        // 线上空闲足够久，便于起始位检测
        wr_mag = mag_of(12'd0);
        wrun = 1'b1; wr_en = 1'b1;
        repeat (BINS + 2) @(posedge clk);
        wr_en = 1'b0; wrun = 1'b0;
        @(posedge clk);
        $display("BANK a0=%02x a1=%02x a2=%02x a3=%02x last=%02x",
                 dut.bank_a[0], dut.bank_a[1], dut.bank_a[2], dut.bank_a[3], dut.bank_a[BINS-1]);
        scan_done = 1'b1;
        @(posedge clk);
        scan_done = 1'b0;
    end

    always @(posedge clk) begin
        if (wrun && wbin < BINS-1) begin
            wbin   <= wbin + 12'd1;
            wr_mag <= mag_of(wbin + 12'd1);
        end
    end

    // ---- 逐字节监视（只打 payload 前 12 个，便于定位） --------------------
    integer lp;
    initial lp = -1;
    always @(posedge clk) begin
        if (dut.pos != lp) begin
            lp = dut.pos;
            if (dut.pos >= 44 && dut.pos < 56)
                $display("  pos=%0d rd_addr=%0d pay_q=%02x byte=%02x data=%02x",
                         dut.pos, dut.rd_addr, dut.pay_q, dut.byte_data, dut.bank_a[dut.pos-44]);
        end
    end

    // ---- 解串并落盘 -------------------------------------------------------
    integer    fd;
    integer    k, j, t0;
    reg [7:0]  rb;
    reg        bv;
    integer    errs = 0;

    initial begin
        fd = 0;
        wait (rst_n === 1'b1);
        @(posedge first_start);
        t0 = cyc;

        fd = $fopen("../tests/v32spec_wire.bin", "wb");
        for (k = 0; k < TOTAL; k = k + 1) begin
            rb = 8'd0;
            for (j = 0; j < 8; j = j + 1) begin
                sample_at(t0 + k*PER + off[j], bv);
                rb[j] = bv;
            end
            sample_at(t0 + k*PER + 308, bv);        // 停止位
            if (bv !== 1'b1) begin
                errs = errs + 1;
                $display("FAIL byte %0d stop bit = %b", k, bv);
            end
            $fwrite(fd, "%c", rb);
        end
        $fclose(fd);

        // 末尾应当是空闲
        sample_at(t0 + TOTAL*PER + 320, bv);
        if (bv !== 1'b1) begin
            errs = errs + 1;
            $display("FAIL line not idle after frame");
        end
        if (completed !== 32'd1) begin
            errs = errs + 1;
            $display("FAIL completed=%0d", completed);
        end

        if (errs == 0) $display("TB_PASS tb_v32spec bytes=%0d bins=%0d", TOTAL, BINS);
        else           $display("TB_FAIL tb_v32spec errors=%0d", errs);
        $finish;
    end

endmodule
