`timescale 1ns/1ps
module tb_harmonic_commands;
 reg clk=0,rst=0,rx=1;always #5 clk=~clk;
 wire he,hc,de,dc,override_auto;wire [31:0] hf,hcfg,df,dcfg;
 wire [3:0] mode,decim;wire [1:0] rate,focus;
 ab5_uart_commands #(.CLK_FREQ(20000000),.BAUD(2000000)) dut(
  .clk(clk),.rst_n(rst),.rx(rx),.rx_copy(1'b1),.demod_en(de),.demod_mode(mode),.ddc_fcw(df),.iq_ch(dc),.cfg_id(dcfg),
  .iq_rate(rate),.wave_focus(focus),.decim_log2(decim),.override_auto(override_auto),
  .harmonic_en(he),.harmonic_ch(hc),.harmonic_fcw(hf),.harmonic_cfg(hcfg));
 task send(input [7:0] b,input bad);
  integer i;begin rx=0;#100;for(i=0;i<8;i=i+1)begin rx=b[i];#100;end rx=!bad;#100;rx=1;#100;end
 endtask
 task command(input [7:0] c,input [7:0] v);begin send('ha5,0);send(c,0);send(v,0);end endtask
 task word_command(input [7:0] c,input [15:0] v);begin send('ha5,0);send(c,0);send(v[7:0],0);send(v[15:8],0);end endtask
 integer k;reg [31:0] saved;
 initial begin
  repeat(4)@(negedge clk);rst=1;repeat(4)@(negedge clk);
  if(he || hc || hf!=66076 || hcfg!=0 || dcfg!=0)$fatal(1,"defaults");
  command('hf0,1);if(hcfg!=1 || !hc || dcfg!=0)$fatal(1,"harmonic channel affected DDC");
  word_command('hf1,'h1234);word_command('he1,'habcd);
  if(hf!=66076 || df!=66076420 || hcfg!=1 || dcfg!=0)$fatal(1,"non-atomic stage");
  word_command('hf2,'h5678);
  if(hf!=32'h12345678 || hcfg!=2 || df!=66076420 || dcfg!=0)$fatal(1,"harmonic staging mixed with DDC");
  word_command('he2,'h0222);
  if(df!=32'habcd0222 || dcfg!=1 || hf!=32'h12345678 || hcfg!=2)$fatal(1,"DDC staging mixed with harmonic");
  command('hf3,1);if(!he || hcfg!=3 || dcfg!=1)$fatal(1,"enable");
  command('hf0,1);command('hf3,1);word_command('hf1,'h1234);word_command('hf2,'h5678);
  if(hcfg!=3 || dcfg!=1)$fatal(1,"duplicate harmonic command was not idempotent");
  command('hf0,2);command('hf3,2);
  send('ha5,0);send('hf3,0);send(0,1);
  if(!he || !hc || hcfg!=3)$fatal(1,"invalid harmonic operand/stop");
  word_command('hf1,'hffff);
  send('ha5,0);send('hf2,0);send(1,0);repeat(2200)@(negedge clk);send(2,0);
  word_command('hf2,'h3333);
  if(hf!=32'h12345678 || hcfg!=3)$fatal(1,"partial command reused staged data");
  // Legacy experimental opcodes remain unknown, and do not poison following commands.
  send('ha5,0);send('he6,0);send('ha5,0);send('he8,0);send('ha5,0);send('hfd,0);
  command('he0,1);command('he3,1);command('he4,2);saved=dcfg;
  command('he5,0);command('he5,2);
  if(mode!=1 || !de || !dc || rate!=2 || focus!=2 || dcfg!=saved || hcfg!=3)$fatal(1,"legacy DDC/focus regression");
  for(k=0;k<16;k=k+1)begin send('ha5,0);send(k,0);if(!override_auto || decim!=k)$fatal(1,"deep RAW decimation");end
  send('ha5,0);send('hff,0);if(override_auto)$fatal(1,"automatic timebase restore");
  command('hf3,0);if(he || hcfg!=4 || dcfg!=saved)$fatal(1,"disable affected DDC");
  $display("HARMONIC_COMMANDS_PASS isolated_atomic_idempotent_timeout_unknown_legacy");$finish;
 end
 initial begin #300000;$fatal(1,"command watchdog");end
endmodule
