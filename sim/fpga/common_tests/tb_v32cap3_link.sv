`timescale 1ns/1ps
// 端到端验证：复现板上真实状态（snapshot_ready 恒 0），确认 Y4 上真的出现完整 V32CAP3 帧。
//
// 修复前：wave_pause 恒 1 -> v32cap3_tx 永不采帧 -> completed 恒 0 -> 一个字节都发不出。
// 修复后：状态机停在 WAVE，wave_pause 恒 0，采满 1024 点即组帧发送。
//
// 参数缩放：CLK_FREQ/BAUD = 16 拍/bit，帧长 40+1024*4+4 = 4140 B，仿真约 66 万拍。
module tb_v32cap3_link;
 localparam integer CLK_FREQ=16000, BAUD=1000, POINTS=1024;
 localparam integer CPB=CLK_FREQ/BAUD;          // 16 clocks per bit

 reg clk=0, rst_n=0;
 reg snapshot_ready=0;                          // 板上实测状态：恒 0
 reg spec_busy=0;
 reg [31:0] wave_completed;
 wire wave_pause, wave_tx, wave_busy;
 wire spec_start, spec_tx;
 wire tx;

 always #1 clk=~clk;

 // ---- 门闸 + 帧发送器 -------------------------------------------------------
 ab5_frame_sequencer #(.CLK_FREQ(CLK_FREQ),.BAUD(BAUD),.SPEC_ENABLE(1)) u_seq(
  .clk(clk),.rst_n(rst_n),.wave_completed(wave_completed),.snapshot_ready(snapshot_ready),
  .spec_busy(spec_busy),.wave_tx(wave_tx),.spec_tx(spec_tx),.tx(tx),
  .wave_pause(wave_pause),.spec_start(spec_start));

 v32cap3_tx #(.POINTS(POINTS),.LINK_ENABLE(1),.MEAS_ENABLE(0),
              .CLK_FREQ(CLK_FREQ),.BAUD(BAUD),.FS_HZ(65000000),.FRAME_PERIOD(0)) u_v32(
  .clk(clk),.rst_n(rst_n),.link_pause(wave_pause),.sample_valid(1'b1),.decim_log2(4'd0),
  .fault(1'b0),.ch0(12'sd100),.ch1(-12'sd100),.meas_body(992'd0),
  .tx(wave_tx),.busy(wave_busy),.completed(wave_completed),.aborted(),.skipped());

 // ---- 独立 UART 接收模型：把 tx 上的字节还原出来 --------------------------
 reg [7:0] rx_byte; reg rx_valid;
 reg [15:0] tmr; reg [3:0] bitn; reg [9:0] sh; reg act;
 always @(posedge clk) begin
  rx_valid<=1'b0;
  if(!rst_n) begin act<=0; tmr<=0; bitn<=0; sh<=0; end
  else if(!act) begin
   if(tx==1'b0) begin act<=1'b1; tmr<=CPB/2; bitn<=0; end
  end else begin
   if(tmr!=0) tmr<=tmr-1'b1;
   else begin
    tmr<=CPB-1;
    if(bitn==0) begin        // 起始位中点，进入 D0
     bitn<=1;
    end else if(bitn<=8) begin
     sh[bitn-1]<=tx;
     bitn<=bitn+1'b1;
    end else begin           // bitn==9：停止位中点
     act<=0; rx_byte<=sh[7:0]; rx_valid<=1'b1;
    end
   end
  end
 end

 // ---- 魔数滑动窗口 --------------------------------------------------------
 // 左移窗口：最高字节是最早收到的，最低字节是最新收到的
 localparam [63:0] MAGIC = {8'h56,8'h33,8'h32,8'h43,8'h41,8'h50,8'h33,8'h21}; // "V32CAP3!"
 reg [63:0] win;
 integer nbytes=0, nmagic=0, i;

 initial begin
  win=64'd0; nbytes=0; nmagic=0;
  rst_n=0;
  repeat(8) @(posedge clk);
  rst_n=1;

  // 跑足够长时间：至少发出 2 帧
  for(i=0;i<1_600_000;i=i+1) begin
   @(posedge clk);
   if(rx_valid) begin
    nbytes=nbytes+1;
    if(nbytes<=24) $display("BYTE #%0d = %02x",nbytes,rx_byte);
    win={win[55:0],rx_byte};
    if(nbytes>=8 && win==MAGIC) begin
     nmagic=nmagic+1;
     $display("MAGIC_HIT at byte #%0d  completed=%0d",nbytes,wave_completed);
    end
   end
  end

  $display("RESULT completed=%0d bytes_on_wire=%0d magic_hits=%0d",wave_completed,nbytes,nmagic);
  if(wave_completed>=1 && nmagic>=1) $display("PASS Y4 emits complete V32CAP3 frames with snapshot_ready=0");
  else $display("FAIL no frame reached the wire");
  $finish;
 end

 // 看门狗
 initial begin
  #40000000;
  $display("TIMEOUT completed=%0d bytes=%0d magic=%0d",wave_completed,nbytes,nmagic);
  $finish;
 end
endmodule
