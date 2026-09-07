`timescale 1ns / 1ps
// Purpose: initialize an SSD1306 I2C OLED, clear its display RAM, and render two
// 16-character lines. An `update` pulse latches new text and redraws both lines.
// The embedded 5x7 font supplies five columns plus one blank spacing column.
//
// SSD1306 control byte 0x00 selects commands and 0x40 selects display data.
// Commands 0x21/0x22 establish the column/page windows used by the clear and
// text transfers. `i2c_write` handles byte-level bus timing and ACK checking.
//
// Reference: SSD1306 datasheet (commands, addressing, initialization)
// https://www.sunrom.com/download/SSD1306.pdf

module ssd1306 #(
    parameter CLK_FREQ = 25_000_000,
    parameter I2C_FREQ = 100_000,
    parameter [6:0] ADDR = 7'h3C,
    parameter HEIGHT = 64
) (
    input  wire         clk,
    input  wire         rstn,
    input  wire         start,
    input  wire         update,
    input  wire [127:0] line1,
    input  wire [127:0] line2,
    output reg          done,
    output reg          ack_err,
    output wire         scl_oe,
    output wire         sda_oe,
    input  wire         sda_in
);
    localparam DELAY_100MS = CLK_FREQ / 10;
    localparam PAGES       = HEIGHT / 8;
    localparam CLEAR_N     = 11'd1 + (11'd128 * PAGES);
    localparam CHAR_N      = 16;
    localparam TEXT_N      = 11'd1 + (CHAR_N * 11'd6);
    localparam COL0        = 8'd16;
    localparam COL1        = 8'd111;

    localparam S_IDLE      = 4'd0;
    localparam S_WAIT      = 4'd1;
    localparam S_INIT      = 4'd2;
    localparam S_WIN_ALL   = 4'd3;
    localparam S_CLEAR     = 4'd4;
    localparam S_WIN_L1    = 4'd5;
    localparam S_TEXT_L1   = 4'd6;
    localparam S_WIN_L2    = 4'd7;
    localparam S_TEXT_L2   = 4'd8;
    localparam S_DONE      = 4'd9;
    localparam S_FAIL      = 4'd10;

    localparam INIT_N = 11'd27;
    localparam WIN_N  = 11'd7;

    localparam PAGE1 = (HEIGHT == 32) ? 8'd1 : 8'd2;
    localparam PAGE2 = (HEIGHT == 32) ? 8'd2 : 8'd4;

    reg  [3:0]   state;
    reg          busy;
    reg          i2c_start;
    reg          pending;
    reg  [10:0]  idx;
    reg  [31:0]  timer;
    reg  [127:0] l1, l2;
    reg  [3:0]   next_state;
    reg  [10:0]  tx_len;
    reg          i2c_last;
    reg  [7:0]   i2c_data;

    wire        i2c_ready;
    wire        i2c_byte_done;
    wire        i2c_ack_err;

    // Datasheet initialization command stream. Index zero is the I2C control
    // byte; commands that take arguments occupy consecutive entries.
    function [7:0] init_byte;
        input [10:0] i;
        begin
            case (i)
                11'd0:  init_byte = 8'h00;
                11'd1:  init_byte = 8'hAE;
                11'd2:  init_byte = 8'hD5;
                11'd3:  init_byte = 8'h80;
                11'd4:  init_byte = 8'hA8;
                11'd5:  init_byte = HEIGHT[7:0] - 8'd1;
                11'd6:  init_byte = 8'hD3;
                11'd7:  init_byte = 8'h00;
                11'd8:  init_byte = 8'h40;
                11'd9:  init_byte = 8'h8D;
                11'd10: init_byte = 8'h14;
                11'd11: init_byte = 8'h20;
                11'd12: init_byte = 8'h00;
                11'd13: init_byte = 8'hA1;
                11'd14: init_byte = 8'hC8;
                11'd15: init_byte = 8'hDA;
                11'd16: init_byte = (HEIGHT == 32) ? 8'h02 : 8'h12;
                11'd17: init_byte = 8'h81;
                11'd18: init_byte = 8'hCF;
                11'd19: init_byte = 8'hD9;
                11'd20: init_byte = 8'hF1;
                11'd21: init_byte = 8'hDB;
                11'd22: init_byte = 8'h40;
                11'd23: init_byte = 8'hA4;
                11'd24: init_byte = 8'hA6;
                11'd25: init_byte = 8'h2E;
                11'd26: init_byte = 8'hAF;
                default: init_byte = 8'h00;
            endcase
        end
    endfunction

    function [7:0] win_byte;
        input [2:0]  which;
        input [10:0] i;
        reg [7:0] x0, x1, p;
        begin
            case (which)
                3'd0: begin x0 = 8'd0; x1 = 8'd127; p = 8'd0; end
                3'd1: begin x0 = COL0; x1 = COL1;   p = PAGE1; end
                default: begin x0 = COL0; x1 = COL1; p = PAGE2; end
            endcase
            case (i)
                11'd0:  win_byte = 8'h00;
                11'd1:  win_byte = 8'h21;
                11'd2:  win_byte = x0;
                11'd3:  win_byte = x1;
                11'd4:  win_byte = 8'h22;
                11'd5:  win_byte = p;
                11'd6:  win_byte = (which == 3'd0) ? (PAGES - 1) : p;
                default: win_byte = 8'h00;
            endcase
        end
    endfunction

    // Project-local 5x7 bitmap font. Each byte is one vertical pixel column.
    function [7:0] font_col;
        input [7:0] ch;
        input [2:0] col;
        reg [39:0] bits;
        begin
            case (ch)
                " ": bits = {8'h00, 8'h00, 8'h00, 8'h00, 8'h00};
                "-": bits = {8'h08, 8'h08, 8'h08, 8'h08, 8'h08};
                "0": bits = {8'h3E, 8'h51, 8'h49, 8'h45, 8'h3E};
                "1": bits = {8'h00, 8'h42, 8'h7F, 8'h40, 8'h00};
                "2": bits = {8'h42, 8'h61, 8'h51, 8'h49, 8'h46};
                "3": bits = {8'h21, 8'h41, 8'h45, 8'h4B, 8'h31};
                "4": bits = {8'h18, 8'h14, 8'h12, 8'h7F, 8'h10};
                "5": bits = {8'h27, 8'h45, 8'h45, 8'h45, 8'h39};
                "6": bits = {8'h3C, 8'h4A, 8'h49, 8'h49, 8'h30};
                "7": bits = {8'h01, 8'h71, 8'h09, 8'h05, 8'h03};
                "8": bits = {8'h36, 8'h49, 8'h49, 8'h49, 8'h36};
                "9": bits = {8'h06, 8'h49, 8'h49, 8'h29, 8'h1E};
                "A": bits = {8'h7E, 8'h11, 8'h11, 8'h11, 8'h7E};
                "B": bits = {8'h7F, 8'h49, 8'h49, 8'h49, 8'h36};
                "C": bits = {8'h3E, 8'h41, 8'h41, 8'h41, 8'h22};
                "D": bits = {8'h7F, 8'h41, 8'h41, 8'h22, 8'h1C};
                "E": bits = {8'h7F, 8'h49, 8'h49, 8'h49, 8'h41};
                "F": bits = {8'h7F, 8'h09, 8'h09, 8'h09, 8'h01};
                "G": bits = {8'h3E, 8'h41, 8'h51, 8'h51, 8'h73};
                "H": bits = {8'h7F, 8'h08, 8'h08, 8'h08, 8'h7F};
                "I": bits = {8'h00, 8'h41, 8'h7F, 8'h41, 8'h00};
                "J": bits = {8'h20, 8'h40, 8'h41, 8'h3F, 8'h01};
                "K": bits = {8'h7F, 8'h08, 8'h14, 8'h22, 8'h41};
                "L": bits = {8'h7F, 8'h40, 8'h40, 8'h40, 8'h40};
                "M": bits = {8'h7F, 8'h02, 8'h0C, 8'h02, 8'h7F};
                "N": bits = {8'h7F, 8'h04, 8'h08, 8'h10, 8'h7F};
                "O": bits = {8'h3E, 8'h41, 8'h41, 8'h41, 8'h3E};
                "P": bits = {8'h7F, 8'h09, 8'h09, 8'h09, 8'h06};
                "Q": bits = {8'h3E, 8'h41, 8'h51, 8'h21, 8'h5E};
                "R": bits = {8'h7F, 8'h09, 8'h19, 8'h29, 8'h46};
                "S": bits = {8'h26, 8'h49, 8'h49, 8'h49, 8'h32};
                "T": bits = {8'h01, 8'h01, 8'h7F, 8'h01, 8'h01};
                "U": bits = {8'h3F, 8'h40, 8'h40, 8'h40, 8'h3F};
                "V": bits = {8'h1F, 8'h20, 8'h40, 8'h20, 8'h1F};
                "W": bits = {8'h3F, 8'h40, 8'h38, 8'h40, 8'h3F};
                "X": bits = {8'h63, 8'h14, 8'h08, 8'h14, 8'h63};
                "Y": bits = {8'h07, 8'h08, 8'h70, 8'h08, 8'h07};
                "Z": bits = {8'h61, 8'h51, 8'h49, 8'h45, 8'h43};
                default: bits = {8'h7F, 8'h41, 8'h41, 8'h41, 8'h7F};
            endcase
            case (col)
                3'd0: font_col = bits[39:32];
                3'd1: font_col = bits[31:24];
                3'd2: font_col = bits[23:16];
                3'd3: font_col = bits[15:8];
                3'd4: font_col = bits[7:0];
                default: font_col = 8'h00;
            endcase
        end
    endfunction

    function [7:0] line_char;
        input        line;
        input [3:0]  ci;
        begin
            if (line)
                line_char = l2[127-8*ci -: 8];
            else
                line_char = l1[127-8*ci -: 8];
        end
    endfunction

    function [7:0] text_byte;
        input        line;
        input [10:0] i;
        reg [10:0] pix;
        begin
            if (i == 11'd0)
                text_byte = 8'h40;
            else begin
                pix = i - 11'd1;
                text_byte = font_col(line_char(line, pix / 11'd6), pix % 11'd6);
            end
        end
    endfunction

    always @(*) begin
        next_state = state;
        tx_len     = 11'd1;
        i2c_data   = 8'h00;
        case (state)
            S_INIT: begin
                tx_len     = INIT_N;
                next_state = S_WIN_ALL;
                i2c_data   = init_byte(idx);
            end
            S_WIN_ALL: begin
                tx_len     = WIN_N;
                next_state = S_CLEAR;
                i2c_data   = win_byte(3'd0, idx);
            end
            S_CLEAR: begin
                tx_len     = CLEAR_N;
                next_state = S_WIN_L1;
                i2c_data   = (idx == 11'd0) ? 8'h40 : 8'h00;
            end
            S_WIN_L1: begin
                tx_len     = WIN_N;
                next_state = S_TEXT_L1;
                i2c_data   = win_byte(3'd1, idx);
            end
            S_TEXT_L1: begin
                tx_len     = TEXT_N;
                next_state = S_WIN_L2;
                i2c_data   = text_byte(1'b0, idx);
            end
            S_WIN_L2: begin
                tx_len     = WIN_N;
                next_state = S_TEXT_L2;
                i2c_data   = win_byte(3'd2, idx);
            end
            S_TEXT_L2: begin
                tx_len     = TEXT_N;
                next_state = S_DONE;
                i2c_data   = text_byte(1'b1, idx);
            end
            default: ;
        endcase
        i2c_last = (idx == (tx_len - 11'd1));
    end

    i2c_write #(
        .CLK_FREQ (CLK_FREQ),
        .I2C_FREQ (I2C_FREQ)
    ) u_i2c (
        .clk       (clk),
        .rstn      (rstn),
        .start     (i2c_start),
        .addr      (ADDR),
        .data      (i2c_data),
        .last      (i2c_last),
        .byte_done (i2c_byte_done),
        .ready     (i2c_ready),
        .ack_err   (i2c_ack_err),
        .scl_oe    (scl_oe),
        .sda_oe    (sda_oe),
        .sda_in    (sda_in)
    );

    always @(posedge clk) begin
        if (!rstn) begin
            state     <= S_IDLE;
            busy      <= 1'b0;
            i2c_start <= 1'b0;
            pending   <= 1'b0;
            idx       <= 11'd0;
            timer     <= 32'd0;
            done      <= 1'b0;
            ack_err   <= 1'b0;
            l1        <= {16{" "}};
            l2        <= {16{" "}};
        end else begin
            i2c_start <= 1'b0;
            case (state)
                S_IDLE: begin
                    done    <= 1'b0;
                    ack_err <= 1'b0;
                    busy    <= 1'b0;
                    if (start) begin
                        l1    <= line1;
                        l2    <= line2;
                        timer <= DELAY_100MS;
                        state <= S_WAIT;
                    end
                end

                S_WAIT: begin
                    if (update)
                        pending <= 1'b1;
                    if (timer == 32'd0)
                        state <= S_INIT;
                    else
                        timer <= timer - 1'b1;
                end

                S_INIT, S_WIN_ALL, S_CLEAR, S_WIN_L1, S_TEXT_L1, S_WIN_L2, S_TEXT_L2: begin
                    if (update)
                        pending <= 1'b1;
                    if (!busy) begin
                        if (i2c_ready) begin
                            idx       <= 11'd0;
                            i2c_start <= 1'b1;
                            busy      <= 1'b1;
                        end
                    end else begin
                        if (i2c_byte_done)
                            idx <= idx + 1'b1;
                        if (i2c_ready && !i2c_start) begin
                            busy <= 1'b0;
                            if (i2c_ack_err) begin
                                ack_err <= 1'b1;
                                state   <= S_FAIL;
                            end else
                                state <= next_state;
                        end
                    end
                end

                S_DONE: begin
                    done <= 1'b1;
                    if (pending || update) begin
                        pending <= 1'b0;
                        l1      <= line1;
                        l2      <= line2;
                        busy    <= 1'b0;
                        state   <= S_WIN_L1;
                    end
                end

                S_FAIL: ack_err <= 1'b1;

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
