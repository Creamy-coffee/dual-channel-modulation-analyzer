`timescale 1ns/1ps
`default_nettype none
// 65 MHz processing plus 300-degree ADC forwarding clock.
// ADC return is captured once per processing cycle with explicit phase constraints.
// HDMI pixel/serializer MMCM is intentionally removed.
module clocking_sys_7series(
    input  wire clk_50m,
    input  wire arst_n,
    output wire clk_sys,
    output wire clk_adc_fwd,
    output wire locked
);
`ifdef SIMULATION
    reg sys65=0;
    reg adc65=0;
    initial begin #5.1282051282; forever begin adc65=~adc65; #7.6923076923; end end
    reg [7:0] lock_count=0;
    always #7.6923076923 sys65=~sys65;
    always @(posedge clk_50m or negedge arst_n)
        if(!arst_n) lock_count<=0;
        else if(!(&lock_count)) lock_count<=lock_count+1'b1;
    assign clk_sys=sys65;
    assign clk_adc_fwd=adc65;
    assign locked=&lock_count;
`else
    wire input_clock, clk_50m_buf;
    wire sys_fb,sys_fb_buf,sys_unbuf,adc_unbuf;
    IBUF u_ibuf(.I(clk_50m),.O(input_clock));
    BUFG u_inbuf(.I(input_clock),.O(clk_50m_buf));
    MMCME2_BASE #(
        .BANDWIDTH("OPTIMIZED"), .CLKIN1_PERIOD(20.0),
        .DIVCLK_DIVIDE(2), .CLKFBOUT_MULT_F(39.0),
        .CLKOUT0_DIVIDE_F(15.0), .CLKOUT0_PHASE(0.0),
        .CLKOUT1_DIVIDE(15), .CLKOUT1_PHASE(300.0),
        .STARTUP_WAIT("FALSE")
    ) u_mmcm_sys (
        .CLKIN1(clk_50m_buf), .CLKFBIN(sys_fb_buf),
        .CLKFBOUT(sys_fb), .CLKFBOUTB(),
        .CLKOUT0(sys_unbuf), .CLKOUT0B(),
        .CLKOUT1(adc_unbuf),.CLKOUT1B(),.CLKOUT2(),.CLKOUT2B(),
        .CLKOUT3(),.CLKOUT3B(),.CLKOUT4(),.CLKOUT5(),.CLKOUT6(),
        .LOCKED(locked), .RST(!arst_n), .PWRDWN(1'b0)
    );
    BUFG u_fb(.I(sys_fb),.O(sys_fb_buf));
    BUFG u_adc(.I(adc_unbuf),.O(clk_adc_fwd));
    BUFG u_sys(.I(sys_unbuf),.O(clk_sys));
`endif
endmodule
`default_nettype wire
