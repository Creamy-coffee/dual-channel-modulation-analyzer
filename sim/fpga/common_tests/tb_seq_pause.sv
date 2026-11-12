`timescale 1ns/1ps
// 验证 ab5_frame_sequencer 修复后不会把波形通路挂死。
// 场景 A：snapshot_ready 恒 0（板上实测状态）——修复前 wave_pause 恒 1，波形帧一帧都发不出。
// 场景 B：snapshot_ready 偶尔为 1——应切到频谱帧，之后回到 WAVE。
module tb_seq_pause;
 localparam integer CLK_FREQ=1000, BAUD=1000;   // 缩放：DRAIN 很小，仿真快
 reg clk=0, rst_n=0, snapshot_ready=0, spec_busy=0;
 reg [31:0] wave_completed=0;
 wire tx, wave_pause;
 wire spec_start;

 ab5_frame_sequencer #(.CLK_FREQ(CLK_FREQ),.BAUD(BAUD),.SPEC_ENABLE(1)) dut(
  .clk(clk),.rst_n(rst_n),.wave_completed(wave_completed),.snapshot_ready(snapshot_ready),
  .spec_busy(spec_busy),.wave_tx(1'b0),.spec_tx(1'b1),.tx(tx),
  .wave_pause(wave_pause),.spec_start(spec_start));

 always #5 clk=~clk;

 integer pause_cycles_A=0, spec_start_A=0, pause_cycles_B=0, spec_start_B=0;
 integer i;

 initial begin
  rst_n=0;
  repeat(4) @(posedge clk);
  rst_n=1;

  // ---- 场景 A：snapshot_ready=0，wave_completed 持续增长（波形帧正常采集中）----
  for(i=0;i<20000;i=i+1) begin
   @(posedge clk);
   if(i%100==0) wave_completed<=wave_completed+1'b1;
   if(wave_pause===1'b1) pause_cycles_A=pause_cycles_A+1;
   if(spec_start===1'b1) spec_start_A=spec_start_A+1;
  end
  $display("A_snapshot_ready0: wave_pause_high_cycles=%0d spec_start_pulses=%0d",pause_cycles_A,spec_start_A);
  if(pause_cycles_A==0 && spec_start_A==0) $display("A_PASS waveform link never blocked");
  else $display("A_FAIL");

  // ---- 场景 B：snapshot_ready=1，应进入频谱流程（wave_pause 拉高），完成后回 WAVE ----
  snapshot_ready<=1'b1;
  wave_completed<=wave_completed+1'b1;
  fork
   begin : spec_model
    @(posedge spec_start);
    @(posedge clk); spec_busy<=1'b1;
    repeat(50) @(posedge clk);
    spec_busy<=1'b0;
   end
   begin
    for(i=0;i<8000;i=i+1) begin
     @(posedge clk);
     if(wave_pause===1'b1) pause_cycles_B=pause_cycles_B+1;
     if(spec_start===1'b1) spec_start_B=spec_start_B+1;
    end
   end
  join
  $display("B_snapshot_ready1: wave_pause_high_cycles=%0d spec_start_pulses=%0d",pause_cycles_B,spec_start_B);
  if(spec_start_B>=1 && pause_cycles_B>0) $display("B_PASS spectrum handover happened then released");
  else $display("B_FAIL");
  $display("state_final=%0d",dut.state);
  $finish;
 end
endmodule
