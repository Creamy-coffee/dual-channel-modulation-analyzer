`timescale 1ns/1ps
module stop_margin_lane #(parameter real RX_BIT_NS=500.0)(output reg done=0);
 reg clk=0; always #7.6923076923 clk=~clk;
 reg rst=0; reg [7:0] data=0; wire ready,tx;
 integer sent=0,received=0,k; reg [7:0] value;
 uart_tx_v32 #(.BAUD(2000000),.STOP_BITS(2)) dut(
  .clk(clk),.rst_n(rst),.tx_data(data),.tx_valid(sent<1024),.tx_ready(ready),.tx(tx));
 initial begin repeat(5) @(negedge clk);rst=1;end
 always @(posedge clk) if(rst && ready && sent<1024) begin
  sent<=sent+1;data<=data+1'b1;
 end
 initial begin
  wait(rst);
  repeat(1024) begin
   @(negedge tx); #(RX_BIT_NS*1.5);
   for(k=0;k<8;k=k+1) begin value[k]=tx;#(RX_BIT_NS);end
   if(tx!==1 || value!==(received%256))
    $fatal(1,"8N1 RX mismatch bit_ns=%f index=%0d got=%h",RX_BIT_NS,received,value);
   // Check added stop margin independently of the transmitter state.
   #(RX_BIT_NS);
   if(tx!==1) $fatal(1,"second stop missing");
   received=received+1;
  end
  done=1;
 end
endmodule
module tb_uart_stop_margin;
 wire a,b,c;
 stop_margin_lane #(.RX_BIT_NS(490.0)) fast(a);
 stop_margin_lane #(.RX_BIT_NS(500.0)) nominal(b);
 stop_margin_lane #(.RX_BIT_NS(510.0)) slow(c);
 initial begin wait(a&&b&&c);$display("UART_STOP_MARGIN_PASS 1024 bytes per lane, independent 8N1 RX +/-2 percent");$finish;end
 initial begin #10000000;$fatal(1,"timeout");end
endmodule
