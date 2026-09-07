`timescale 1ns / 1ps
// Purpose: assemble the OV7670's two RGB565 bytes per pixel and convert each
// pixel to one 8-bit grayscale value before writing the frame buffer. HREF marks
// valid line pixels; `new_frame` resets the linear address at VSYNC.
//
// The luma approximation (R + 2G + B)/4 uses only adders and a shift. It is not
// the exact Rec. 601 luma equation, but it preserves green's greater influence
// while avoiding multipliers and storing only half as many BRAM bits as RGB565.
//
// References:
//   OV7670 datasheet (RGB565 byte order, PCLK/HREF/VSYNC)
//   https://strawberry-linux.com/pub/OV7670.pdf

module ov7670_capture #(
    parameter integer MAX_PIXELS = 307200
) (
    input  wire        pclk,
    input  wire        href,
    input  wire [7:0]  data,
    input  wire        enable,
    input  wire        new_frame,
    output reg  [18:0] wr_addr,
    output reg  [7:0]  wr_data,
    output reg         wr_en
);
    reg        half;
    reg [7:0]  first_byte;
    reg [18:0] pix_count;

    // The camera sends the high RGB565 byte first and the low byte second.
    // Repeating the most significant source bits gives full-width components
    // without multipliers or lookup tables.
    wire [15:0] rgb565 = {first_byte, data};
    wire [7:0] red     = {rgb565[15:11], rgb565[15:13]};
    wire [7:0] green   = {rgb565[10:5],  rgb565[10:9]};
    wire [7:0] blue    = {rgb565[4:0],   rgb565[4:2]};
    // Hardware-friendly luma approximation: (R + 2G + B) / 4.
    wire [9:0] luma_sum = {2'b00, red} + {1'b0, green, 1'b0} + {2'b00, blue};

    initial begin
        wr_en     = 1'b0;
        wr_addr   = 19'd0;
        pix_count = 19'd0;
        half      = 1'b0;
    end

    always @(posedge pclk) begin
        wr_en <= 1'b0;

        if (new_frame) begin
            half      <= 1'b0;
            pix_count <= 19'd0;
            wr_addr   <= 19'd0;
        end else begin
            if (enable && href) begin
                if (!half) begin
                    // First PCLK of a pixel: remember the upper RGB565 byte.
                    first_byte <= data;
                    half       <= 1'b1;
                end else begin
                    // Second PCLK: convert the complete pixel and pulse wr_en.
                    half <= 1'b0;
                    if (pix_count < MAX_PIXELS[18:0]) begin
                        wr_addr   <= pix_count;
                        wr_data   <= luma_sum[9:2];
                        wr_en     <= 1'b1;
                        pix_count <= pix_count + 1'b1;
                    end
                end
            end else begin
                half <= 1'b0;
            end
        end
    end
endmodule
