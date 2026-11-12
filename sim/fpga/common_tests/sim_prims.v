`timescale 1ns/1ps
// ---------------------------------------------------------------------------
// Behavioural stand-ins for the Xilinx I/O primitives used by daq_top.
//
// These exist ONLY so that a gate-level-free functional simulation of the full
// top level can run under Icarus.  They are never synthesised: the build
// scripts add rtl/*.v (this file lives in tests/), so Vivado never sees it.
//
// ODDR: forwards D1 on the rising edge of C (daq_top drives D1=1, D2=0 to make
//       a copy of the forwarded ADC clock).  The real primitive toggles on both
//       edges; for the purpose of "does the UART transmit at all" only the fact
//       that it does not hang matters.
// IDDR: captured on both edges; unused by the PARALLEL_PORTS=1 datapath but
//       provided so any configuration compiles.
// ---------------------------------------------------------------------------

module ODDR #(
    parameter DDR_CLK_EDGE = "OPPOSITE_EDGE",
    parameter INIT         = 1'b0,
    parameter SRTYPE       = "SYNC"
)(
    output reg Q,
    input wire C,
    input wire CE,
    input wire D1,
    input wire D2,
    input wire R,
    input wire S
);
    initial Q = INIT;
    always @(posedge C) begin
        if (R)      Q <= 1'b0;
        else if (S) Q <= 1'b1;
        else if (CE) Q <= D1;
    end
endmodule

module IDDR #(
    parameter DDR_CLK_EDGE = "SAME_EDGE",
    parameter INIT_Q1      = 1'b0,
    parameter INIT_Q2      = 1'b0,
    parameter SRTYPE       = "SYNC"
)(
    output reg Q1,
    output reg Q2,
    input wire C,
    input wire CE,
    input wire D,
    input wire R,
    input wire S
);
    initial begin
        Q1 = INIT_Q1;
        Q2 = INIT_Q2;
    end
    always @(posedge C) if (CE) Q1 <= D;
    always @(negedge C) if (CE) Q2 <= D;
endmodule
