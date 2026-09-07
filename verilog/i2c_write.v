`timescale 1ns / 1ps

// Purpose: byte-stream I2C master used by the SSD1306 OLED controller. `start`
// begins a transaction and latches address/data. After each acknowledged byte,
// `byte_done` asks the caller to supply the next byte; `last` selects STOP.
//
// SCL/SDA are open-drain controls: an *_oe value of 1 pulls the pin low and 0
// releases it to the pull-up. `ack_err` latches when the slave leaves SDA high
// during the ninth clock. This module supports writes only.
//
// References:
//   NXP UM10204, I2C-bus specification
//   https://www.nxp.com/docs/en/user-guide/UM10204.pdf
//   SSD1306 datasheet: https://www.sunrom.com/download/SSD1306.pdf

module i2c_write #(
    parameter CLK_FREQ = 25_000_000,
    parameter I2C_FREQ = 100_000
) (
    input  wire       clk,
    input  wire       rstn,
    input  wire       start,
    input  wire [6:0] addr,
    input  wire [7:0] data,
    input  wire       last,
    output reg        byte_done,
    output reg        ready,
    output reg        ack_err,
    output reg        scl_oe,
    output reg        sda_oe,
    input  wire       sda_in
);

// Each transmitted bit uses four phases: clock low, set SDA, clock high, then
// advance. TIMER supplies the protocol delays shared by those states.
    localparam IDLE      = 4'd0;
    localparam START_BIT = 4'd1;
    localparam LOAD_BYTE = 4'd2;
    localparam TX1       = 4'd3;
    localparam TX2       = 4'd4;
    localparam TX3       = 4'd5;
    localparam TX4       = 4'd6;
    localparam WAIT_NEXT = 4'd7;
    localparam STOP1     = 4'd8;
    localparam STOP2     = 4'd9;
    localparam STOP3     = 4'd10;
    localparam STOP4     = 4'd11;
    localparam DONE_GAP  = 4'd12;
    localparam TIMER     = 4'd13;

    localparam QUARTER = CLK_FREQ / (4 * I2C_FREQ);
    localparam HALF    = CLK_FREQ / (2 * I2C_FREQ);
    localparam GAP     = 2 * CLK_FREQ / I2C_FREQ;

    reg [3:0]  state, ret_state;
    reg [31:0] timer;
    reg [7:0]  tx_byte;
    reg [3:0]  bit_index;
    reg        sending_addr;
    reg        last_latched;
    reg        nack;

    always @(posedge clk) begin
        if (!rstn) begin
            ready        <= 1'b1;
            ack_err      <= 1'b0;
            scl_oe       <= 1'b0;
            sda_oe       <= 1'b0;
            byte_done    <= 1'b0;
            state        <= IDLE;
            sending_addr <= 1'b0;
            nack         <= 1'b0;
        end else begin
            byte_done <= 1'b0;
            case (state)
                IDLE: begin
                    scl_oe <= 1'b0;
                    sda_oe <= 1'b0;
                    nack   <= 1'b0;
                    if (start) begin
                        ready        <= 1'b0;
                        ack_err      <= 1'b0;
                        sending_addr <= 1'b1;
                        last_latched <= last;
                        tx_byte      <= {addr, 1'b0};
                        bit_index    <= 4'd0;
                        state        <= START_BIT;
                    end else
                        ready <= 1'b1;
                end

                START_BIT: begin
                    scl_oe    <= 1'b0;
                    sda_oe    <= 1'b1;
                    timer     <= HALF;
                    ret_state <= TX1;
                    state     <= TIMER;
                end

                LOAD_BYTE: begin
                    sending_addr <= 1'b0;
                    last_latched <= last;
                    tx_byte      <= data;
                    bit_index    <= 4'd0;
                    state        <= TX1;
                end

                TX1: begin
                    scl_oe    <= 1'b1;
                    timer     <= QUARTER;
                    ret_state <= TX2;
                    state     <= TIMER;
                end

                TX2: begin
                    sda_oe    <= (bit_index == 4'd8) ? 1'b0 : ~tx_byte[7];
                    timer     <= QUARTER;
                    ret_state <= TX3;
                    state     <= TIMER;
                end

                TX3: begin
                    scl_oe    <= 1'b0;
                    timer     <= HALF;
                    ret_state <= TX4;
                    state     <= TIMER;
                end

                TX4: begin
                    if (bit_index == 4'd8) begin
                        if (sda_in)
                            nack <= 1'b1;
                        if (sda_in || (!sending_addr && last_latched))
                            state <= STOP1;
                        else if (sending_addr)
                            state <= LOAD_BYTE;
                        else begin
                            byte_done <= 1'b1;
                            state     <= WAIT_NEXT;
                        end
                    end else begin
                        tx_byte   <= {tx_byte[6:0], 1'b0};
                        bit_index <= bit_index + 1'b1;
                        state     <= TX1;
                    end
                end

                WAIT_NEXT: begin
                    state <= LOAD_BYTE;
                end

                STOP1: begin
                    scl_oe    <= 1'b1;
                    timer     <= QUARTER;
                    ret_state <= STOP2;
                    state     <= TIMER;
                end

                STOP2: begin
                    sda_oe    <= 1'b1;
                    timer     <= QUARTER;
                    ret_state <= STOP3;
                    state     <= TIMER;
                end

                STOP3: begin
                    scl_oe    <= 1'b0;
                    timer     <= HALF;
                    ret_state <= STOP4;
                    state     <= TIMER;
                end

                STOP4: begin
                    sda_oe    <= 1'b0;
                    timer     <= QUARTER;
                    ret_state <= DONE_GAP;
                    state     <= TIMER;
                end

                DONE_GAP: begin
                    ack_err   <= nack;
                    timer     <= GAP;
                    ret_state <= IDLE;
                    state     <= TIMER;
                end

                TIMER: begin
                    if (timer == 32'd0)
                        state <= ret_state;
                    else
                        timer <= timer - 1'b1;
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule
