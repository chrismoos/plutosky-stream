// Drives the AD9361 side of the LVDS link and records what the PHY makes of it.
//
// The bus is driven the way the RFIC drives it, with the data centered on each
// clock edge rather than changing on it, so the capture registers see a real
// eye. Everything decoded is printed and the Python test compares it against
// the samples that went in.

`timescale 1ns / 1ps

module tb;
    parameter integer PERIOD = 8;
    parameter integer QUARTER = PERIOD / 4;
    parameter integer SAMPLES = 8;
    parameter MODE = 0;   // 0 = 2R2T, 1 = 1R1T

    reg clk = 0;
    reg rst = 1;
    reg frame = 0;
    reg [5:0] data = 0;

    wire [5:0] tx_data_p;
    wire tx_frame_p;
    wire rfic_clk;
    wire rx_valid;
    wire [11:0] rx_ia, rx_qa, rx_ib, rx_qb;
    wire tx_ready;

    reg tx_valid = 0;
    reg [11:0] tx_ia = 0, tx_qa = 0, tx_ib = 0, tx_qb = 0;

    ad9361_phy_top dut (
        .pads_rx_clk_p(clk), .pads_rx_clk_n(~clk),
        .pads_rx_frame_p(frame), .pads_rx_frame_n(~frame),
        .pads_rx_data_p(data), .pads_rx_data_n(~data),
        .pads_tx_clk_p(), .pads_tx_clk_n(),
        .pads_tx_frame_p(tx_frame_p), .pads_tx_frame_n(),
        .pads_tx_data_p(tx_data_p), .pads_tx_data_n(),
        .mode(MODE[0]),
        .tx_valid(tx_valid), .tx_ready(tx_ready),
        .tx_ia(tx_ia), .tx_qa(tx_qa), .tx_ib(tx_ib), .tx_qb(tx_qb),
        .rx_valid(rx_valid), .rx_ia(rx_ia), .rx_qa(rx_qa),
        .rx_ib(rx_ib), .rx_qb(rx_qb),
        .rfic_clk_1(rfic_clk), .rfic_rst_1(rst),
        .sys_clk(clk), .sys_rst(rst)
    );

    always #(PERIOD / 2) clk = ~clk;

    integer n;
    reg [11:0] i1 [0:SAMPLES-1];
    reg [11:0] q1 [0:SAMPLES-1];
    reg [11:0] i2 [0:SAMPLES-1];
    reg [11:0] q2 [0:SAMPLES-1];

    // One bus cycle: the I nibble is placed three quarters into the previous
    // cycle and the Q nibble a quarter into this one.
    task drive_cycle(input [5:0] i_nib, input [5:0] q_nib, input frame_bit);
        begin
            @(negedge clk);
            #(QUARTER);
            data = i_nib;
            frame = frame_bit;
            @(posedge clk);
            #(QUARTER);
            data = q_nib;
        end
    endtask

    initial begin
        $display("MODE %0d", MODE);
        for (n = 0; n < SAMPLES; n = n + 1) begin
            i1[n] = 12'h100 + n * 12'h11;
            q1[n] = 12'h700 - n * 12'h13;
            i2[n] = 12'hA00 + n * 12'h17;
            q2[n] = 12'h0C0 + n * 12'h21;
        end
        repeat (6) @(posedge clk);
        rst = 0;
        repeat (4) @(posedge clk);
        for (n = 0; n < SAMPLES; n = n + 1) begin
            if (MODE == 0) begin
                // 2R2T: channel 1 while the frame is high, channel 2 while low.
                drive_cycle(i1[n][11:6], q1[n][11:6], 1'b1);
                drive_cycle(i1[n][5:0], q1[n][5:0], 1'b1);
                drive_cycle(i2[n][11:6], q2[n][11:6], 1'b0);
                drive_cycle(i2[n][5:0], q2[n][5:0], 1'b0);
            end else begin
                // 1R1T: one channel, two consecutive samples per PHY word.
                drive_cycle(i1[n][11:6], q1[n][11:6], 1'b1);
                drive_cycle(i1[n][5:0], q1[n][5:0], 1'b0);
            end
        end
        repeat (12) @(posedge clk);
        $display("DONE");
        $finish;
    end

    always @(posedge rfic_clk) begin
        if (!rst && rx_valid)
            $display("RX %0d %0d %0d %0d", rx_ia, rx_qa, rx_ib, rx_qb);
    end

    // Transmit: hand the PHY a known word whenever it asks, and record the
    // wire so the Python side can decode it back.
    integer tx_index = 0;
    always @(posedge rfic_clk) begin
        if (!rst) begin
            tx_valid <= 1'b1;
            if (tx_ready) begin
                tx_ia <= i1[tx_index % SAMPLES];
                tx_qa <= q1[tx_index % SAMPLES];
                tx_ib <= i2[tx_index % SAMPLES];
                tx_qb <= q2[tx_index % SAMPLES];
                $display("TXWORD %0d %0d %0d %0d",
                         i1[tx_index % SAMPLES], q1[tx_index % SAMPLES],
                         i2[tx_index % SAMPLES], q2[tx_index % SAMPLES]);
                tx_index = tx_index + 1;
            end
        end
    end

    // Sample the transmit wire in the middle of each half cycle, where a
    // receiver would, rather than on the edge where it is changing.
    reg [5:0] tx_rise;
    reg tx_frame_rise;
    initial forever begin
        @(posedge clk);
        #(QUARTER);
        tx_rise = tx_data_p;
        tx_frame_rise = tx_frame_p;
        @(negedge clk);
        #(QUARTER);
        if (!rst)
            $display("TXBUS %0d %0d %0d %0d", tx_frame_rise, tx_rise,
                     tx_frame_p, tx_data_p);
    end
endmodule
