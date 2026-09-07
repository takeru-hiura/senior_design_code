`timescale 1ns / 1ps
// Streaming 3x3 Sobel accelerator for an 8-bit raster-ordered pixel stream.
//
// The module accepts one grayscale pixel whenever in_valid is asserted and
// produces one stored pixel for every input pixel. Two line buffers and six
// shift registers form a causal 3x3 window; no full-frame processing or CPU is
// involved. The first two rows and first two columns are forced to zero. For
// all other locations the output is |Gx| + |Gy|, saturated to 8 bits, followed
// by an optional binary threshold. Because the window ends at the current
// sample, the edge map is displaced down/right by one pixel relative to a
// mathematically centered convolution.
//
// Throughput: one pixel per valid input clock after warm-up.
// Storage:    2 * IMAGE_WIDTH * 8 bits (line-buffer RAM).

module sobel_stream #(
    parameter integer IMAGE_WIDTH = 640,
    parameter [7:0]   THRESHOLD   = 8'd80,
    parameter integer BINARY_OUT  = 1
) (
    input  wire        clk,
    input  wire        rstn,
    input  wire        frame_start,
    input  wire        in_valid,
    input  wire [18:0] in_addr,
    input  wire [7:0]  in_pixel,
    output reg         out_valid,
    output reg  [18:0] out_addr,
    output reg  [7:0]  out_pixel
);
    // Asynchronous taps let the camera stream remain bubble tolerant. Vivado
    // maps these small line buffers to distributed RAM; the much larger frame
    // buffer remains in BRAM.
    (* ram_style = "distributed" *) reg [7:0] line_1 [0:IMAGE_WIDTH-1];
    (* ram_style = "distributed" *) reg [7:0] line_2 [0:IMAGE_WIDTH-1];

    reg [9:0] x;
    reg [9:0] y;
    reg [7:0] top_l, top_c;
    reg [7:0] mid_l, mid_c;
    reg [7:0] bot_l, bot_c;

    wire [7:0] top_r = line_2[x];
    wire [7:0] mid_r = line_1[x];
    wire [7:0] bot_r = in_pixel;

    // Window naming follows its physical position:
    //   top_l top_c top_r
    //   mid_l mid_c mid_r
    //   bot_l bot_c bot_r
    // The current input is bot_r; earlier samples come from shift registers
    // and the two line buffers.
    wire signed [11:0] gx =
        $signed({4'b0, top_r}) + ($signed({4'b0, mid_r}) <<< 1) + $signed({4'b0, bot_r})
      - $signed({4'b0, top_l}) - ($signed({4'b0, mid_l}) <<< 1) - $signed({4'b0, bot_l});
    wire signed [11:0] gy =
        $signed({4'b0, bot_l}) + ($signed({4'b0, bot_c}) <<< 1) + $signed({4'b0, bot_r})
      - $signed({4'b0, top_l}) - ($signed({4'b0, top_c}) <<< 1) - $signed({4'b0, top_r});
    wire [11:0] abs_gx = gx[11] ? -gx : gx;
    wire [11:0] abs_gy = gy[11] ? -gy : gy;
    // |Gx| + |Gy| avoids a square root and is a common hardware approximation
    // of the Euclidean gradient magnitude.
    wire [12:0] magnitude = {1'b0, abs_gx} + {1'b0, abs_gy};
    wire [7:0] magnitude_sat = |magnitude[12:8] ? 8'hff : magnitude[7:0];
    wire [7:0] edge_pixel = BINARY_OUT
                          ? ((magnitude_sat >= THRESHOLD) ? 8'hff : 8'h00)
                          : magnitude_sat;

    integer i;
    initial begin
        x = 0;
        y = 0;
        out_valid = 0;
        out_addr = 0;
        out_pixel = 0;
        top_l = 0; top_c = 0;
        mid_l = 0; mid_c = 0;
        bot_l = 0; bot_c = 0;
        for (i = 0; i < IMAGE_WIDTH; i = i + 1) begin
            line_1[i] = 0;
            line_2[i] = 0;
        end
    end

    always @(posedge clk) begin
        out_valid <= 1'b0;

        if (!rstn || frame_start) begin
            x <= 10'd0;
            y <= 10'd0;
            top_l <= 8'd0; top_c <= 8'd0;
            mid_l <= 8'd0; mid_c <= 8'd0;
            bot_l <= 8'd0; bot_c <= 8'd0;
        end else if (in_valid) begin
            out_valid <= 1'b1;
            out_addr  <= in_addr;
            // A complete 3x3 neighborhood is unavailable at the top/left edge.
            out_pixel <= (x < 2 || y < 2) ? 8'd0 : edge_pixel;

            line_2[x] <= line_1[x];
            line_1[x] <= in_pixel;
            top_l <= top_c; top_c <= top_r;
            mid_l <= mid_c; mid_c <= mid_r;
            bot_l <= bot_c; bot_c <= bot_r;

            if (x == IMAGE_WIDTH - 1) begin
                x <= 10'd0;
                y <= y + 1'b1;
                top_l <= 8'd0; top_c <= 8'd0;
                mid_l <= 8'd0; mid_c <= 8'd0;
                bot_l <= 8'd0; bot_c <= 8'd0;
            end else begin
                x <= x + 1'b1;
            end
        end
    end
endmodule
