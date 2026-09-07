`timescale 1ns / 1ps
// Purpose: write one OV7670 register using its SCCB serial-control bus. A
// transaction sends camera write ID 0x42, register address, then register data.
// SCCB is electrically similar to I2C; SIOC/SIOD are open-drain controls where
// 1 pulls low and 0 releases the pin. The ACK slot is released but not checked,
// matching modules/clones that do not provide a dependable ACK indication.
//
// References:
//   OV7670 datasheet (SCCB interface and device address)
//   https://strawberry-linux.com/pub/OV7670.pdf
//   OmniVision Serial Camera Control Bus (SCCB) specification.

module ov7670_sccb #(
    parameter CLK_FREQ  = 25_000_000,
    parameter SCCB_FREQ = 100_000
) (
    input  wire       clk,
    input  wire       start,
    input  wire [7:0] reg_addr,
    input  wire [7:0] reg_data,
    output reg        ready,
    output reg        sioc_oe,  // 1 = pull clock low
    output reg        siod_oe   // 1 = pull data low
);
    localparam CAMERA_ID = 8'h42;

    localparam IDLE         = 4'd0;
    localparam START_BIT    = 4'd1;
    localparam LOAD_BYTE    = 4'd2;
    localparam TX1          = 4'd3;
    localparam TX2          = 4'd4;
    localparam TX3          = 4'd5;
    localparam TX4          = 4'd6;
    localparam STOP1        = 4'd7;
    localparam STOP2        = 4'd8;
    localparam STOP3        = 4'd9;
    localparam STOP4        = 4'd10;
    localparam DONE_GAP     = 4'd11;
    localparam TIMER        = 4'd12;

    localparam QUARTER = CLK_FREQ / (4 * SCCB_FREQ);
    localparam HALF    = CLK_FREQ / (2 * SCCB_FREQ);
    localparam GAP     = 2 * CLK_FREQ / SCCB_FREQ;

    reg [3:0]  state, ret_state;
    reg [31:0] timer;
    reg [7:0]  latched_addr, latched_data, tx_byte;
    reg [1:0]  byte_count;
    reg [3:0]  bit_index;

    initial begin
        ready   = 1'b1;
        sioc_oe = 1'b0;
        siod_oe = 1'b0;
        state   = IDLE;
    end

    always @(posedge clk) begin
        case (state)
            IDLE: begin
                bit_index  <= 4'd0;
                byte_count <= 2'd0;
                if (start) begin
                    latched_addr <= reg_addr;
                    latched_data <= reg_data;
                    ready        <= 1'b0;
                    state        <= START_BIT;
                end else begin
                    ready <= 1'b1;
                end
            end

            START_BIT: begin
                sioc_oe    <= 1'b0;
                siod_oe    <= 1'b1;
                timer      <= QUARTER;
                ret_state  <= LOAD_BYTE;
                state      <= TIMER;
            end

            LOAD_BYTE: begin
                // byte_count 0..2 selects device ID, subaddress, and value;
                // count 3 means all three bytes are complete.
                bit_index <= 4'd0;
                case (byte_count)
                    2'd0: tx_byte <= CAMERA_ID;
                    2'd1: tx_byte <= latched_addr;
                    default: tx_byte <= latched_data;
                endcase
                if (byte_count == 2'd3)
                    state <= STOP1;
                else begin
                    byte_count <= byte_count + 1'b1;
                    state      <= TX1;
                end
            end

            TX1: begin
                sioc_oe   <= 1'b1;
                timer     <= QUARTER;
                ret_state <= TX2;
                state     <= TIMER;
            end

            TX2: begin
                // bit_index 8 is the ACK slot: release SDA
                siod_oe   <= (bit_index == 4'd8) ? 1'b0 : ~tx_byte[7];
                timer     <= QUARTER;
                ret_state <= TX3;
                state     <= TIMER;
            end

            TX3: begin
                sioc_oe   <= 1'b0;
                timer     <= HALF;
                ret_state <= TX4;
                state     <= TIMER;
            end

            TX4: begin
                tx_byte   <= {tx_byte[6:0], 1'b0};
                bit_index <= bit_index + 1'b1;
                state     <= (bit_index == 4'd8) ? LOAD_BYTE : TX1;
            end

            STOP1: begin
                sioc_oe   <= 1'b1;
                timer     <= QUARTER;
                ret_state <= STOP2;
                state     <= TIMER;
            end

            STOP2: begin
                siod_oe   <= 1'b1;
                timer     <= QUARTER;
                ret_state <= STOP3;
                state     <= TIMER;
            end

            STOP3: begin
                sioc_oe   <= 1'b0;
                timer     <= QUARTER;
                ret_state <= STOP4;
                state     <= TIMER;
            end

            STOP4: begin
                siod_oe   <= 1'b0;
                timer     <= QUARTER;
                ret_state <= DONE_GAP;
                state     <= TIMER;
            end

            DONE_GAP: begin
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
endmodule
