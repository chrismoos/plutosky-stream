// Behavioral stand-ins for the 7-series primitives the AD9361 PHY instances,
// enough to run the LVDS framing in an event simulator. They model function,
// not timing: the differential buffers pass the true leg through and the
// output register presents its rising-edge datum while the clock is high.

`timescale 1ns / 1ps

module IBUFDS #(parameter DIFF_TERM = "FALSE", parameter IOSTANDARD = "DEFAULT")
    (output O, input I, input IB);
    assign O = I;
endmodule

module OBUFDS #(parameter IOSTANDARD = "DEFAULT")
    (output O, output OB, input I);
    assign O = I;
    assign OB = ~I;
endmodule

module BUFG (output O, input I);
    assign O = I;
endmodule

module ODDR #(parameter DDR_CLK_EDGE = "SAME_EDGE", parameter INIT = 1'b0,
              parameter SRTYPE = "SYNC")
    (output Q, input C, input CE, input D1, input D2, input R, input S);
    reg rising = INIT;
    reg falling = INIT;
    always @(posedge C) begin
        if (R) begin
            rising <= 1'b0;
            falling <= 1'b0;
        end else if (CE) begin
            rising <= D1;
            falling <= D2;
        end
    end
    assign Q = C ? rising : falling;
endmodule

module IDDR #(parameter DDR_CLK_EDGE = "SAME_EDGE_PIPELINED", parameter SRTYPE = "SYNC",
              parameter INIT_Q1 = 1'b0, parameter INIT_Q2 = 1'b0)
    (output reg Q1, output reg Q2, input C, input CE, input D, input R, input S);
    reg negedge_capture = INIT_Q2;
    initial begin
        Q1 = INIT_Q1;
        Q2 = INIT_Q2;
    end
    always @(negedge C) negedge_capture <= D;
    always @(posedge C) begin
        if (R) begin
            Q1 <= 1'b0;
            Q2 <= 1'b0;
        end else if (CE) begin
            Q1 <= D;
            Q2 <= negedge_capture;
        end
    end
endmodule

module FDPE #(parameter INIT = 1'b1)
    (output reg Q, input C, input CE, input D, input PRE);
    initial Q = INIT;
    always @(posedge C or posedge PRE) begin
        if (PRE)
            Q <= 1'b1;
        else if (CE)
            Q <= D;
    end
endmodule
