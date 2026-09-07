`timescale 1ns / 1ps
// Purpose: adapt newline-terminated license-plate text from the PC into the two
// fixed-width lines consumed by ssd1306.v. Line 1 is a title; line 2 holds at
// most 16 sanitized characters. Lowercase is converted to uppercase and all
// characters outside A-Z, 0-9, space, and hyphen are ignored.
//
// CR, LF, or CRLF commits a plate and pulses `update` for one clock. This wire
// protocol is project-specific; the electrical format is 115200-baud 8N1 UART.

module plate_uart #(
    parameter CLK_FREQ = 25_000_000,
    parameter BAUD     = 115200
) (
    input  wire         clk,
    input  wire         rstn,
    input  wire         rx,
    output wire [127:0] line1,
    output wire [127:0] line2,
    output reg          update
);
    localparam [127:0] TITLE = "     PLATE      ";

    wire [7:0] rx_data;
    wire       rx_valid;
    reg  [4:0] idx;
    reg        saw_cr;
    reg  [7:0] chars [0:15];
    integer    i;

    assign line1 = TITLE;
    assign line2 = {
        chars[0],  chars[1],  chars[2],  chars[3],
        chars[4],  chars[5],  chars[6],  chars[7],
        chars[8],  chars[9],  chars[10], chars[11],
        chars[12], chars[13], chars[14], chars[15]
    };

    uart_rx #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD     (BAUD)
    ) u_rx (
        .clk   (clk),
        .rstn  (rstn),
        .rx    (rx),
        .data  (rx_data),
        .valid (rx_valid)
    );

    function [7:0] sanitize;
        input [7:0] c;
        begin
            if (c >= "a" && c <= "z")
                sanitize = c - 8'h20;
            else if ((c >= "0" && c <= "9") ||
                     (c >= "A" && c <= "Z") ||
                     c == " " || c == "-")
                sanitize = c;
            else
                sanitize = 8'h00;
        end
    endfunction

    always @(posedge clk) begin
        if (!rstn) begin
            chars[0]  <= " ";
            chars[1]  <= " ";
            chars[2]  <= " ";
            chars[3]  <= " ";
            chars[4]  <= "W";
            chars[5]  <= "A";
            chars[6]  <= "I";
            chars[7]  <= "T";
            chars[8]  <= "I";
            chars[9]  <= "N";
            chars[10] <= "G";
            chars[11] <= " ";
            chars[12] <= " ";
            chars[13] <= " ";
            chars[14] <= " ";
            chars[15] <= " ";
            idx    <= 5'd0;
            saw_cr <= 1'b0;
            update <= 1'b0;
        end else begin
            update <= 1'b0;
            if (rx_valid) begin
                if (rx_data == 8'h0D || rx_data == 8'h0A) begin
                    if (!(saw_cr && rx_data == 8'h0A) && idx != 5'd0) begin
                        for (i = 0; i < 16; i = i + 1)
                            if (i >= idx)
                                chars[i] <= " ";
                        update <= 1'b1;
                    end
                    idx    <= 5'd0;
                    saw_cr <= (rx_data == 8'h0D);
                end else begin
                    saw_cr <= 1'b0;
                    if (idx < 5'd16 && sanitize(rx_data) != 8'h00) begin
                        chars[idx] <= sanitize(rx_data);
                        idx <= idx + 1'b1;
                    end
                end
            end
        end
    end
endmodule
