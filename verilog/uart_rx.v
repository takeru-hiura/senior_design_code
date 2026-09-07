`timescale 1ns / 1ps
// Purpose: receive asynchronous 8N1 UART bytes from the PC. The input first
// passes through a three-flop synchronizer. After a falling edge, the FSM checks
// the start bit near its center, samples eight data bits LSB first one bit-time
// apart, and accepts the byte only if the stop bit is high.
//
// `valid` pulses for one FPGA clock for each accepted byte. This simple receiver
// has no parity, oversampling, FIFO, or explicit framing-error output.

module uart_rx #(
    parameter CLK_FREQ = 25_000_000,
    parameter BAUD     = 115200
) (
    input  wire       clk,
    input  wire       rstn,
    input  wire       rx,
    output reg  [7:0] data,
    output reg        valid
);
    localparam DIV = CLK_FREQ / BAUD;
    localparam MID = DIV / 2;

    localparam IDLE  = 2'd0;
    localparam START = 2'd1;
    localparam DATA  = 2'd2;
    localparam STOP  = 2'd3;

    // Reduce metastability risk when the asynchronous UART signal enters clk.
    (* ASYNC_REG = "TRUE" *) reg [2:0] rx_sync;
    wire rxd = rx_sync[2];

    reg [1:0]  state;
    reg [15:0] timer;
    reg [2:0]  bit_i;
    reg [7:0]  shift;

    always @(posedge clk) begin
        if (!rstn)
            rx_sync <= 3'b111;
        else
            rx_sync <= {rx_sync[1:0], rx};
    end

    always @(posedge clk) begin
        if (!rstn) begin
            state <= IDLE;
            valid <= 1'b0;
            data  <= 8'h00;
        end else begin
            valid <= 1'b0;
            case (state)
                IDLE: begin
                    if (!rxd) begin
                        timer <= MID[15:0];
                        state <= START;
                    end
                end

                START: begin
                    if (timer == 16'd0) begin
                        if (!rxd) begin
                            timer <= DIV[15:0];
                            bit_i <= 3'd0;
                            state <= DATA;
                        end else
                            state <= IDLE;
                    end else
                        timer <= timer - 1'b1;
                end

                DATA: begin
                    if (timer == 16'd0) begin
                        shift <= {rxd, shift[7:1]};
                        timer <= DIV[15:0];
                        if (bit_i == 3'd7)
                            state <= STOP;
                        else
                            bit_i <= bit_i + 1'b1;
                    end else
                        timer <= timer - 1'b1;
                end

                STOP: begin
                    if (timer == 16'd0) begin
                        if (rxd) begin
                            data  <= shift;
                            valid <= 1'b1;
                        end
                        state <= IDLE;
                    end else
                        timer <= timer - 1'b1;
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule
