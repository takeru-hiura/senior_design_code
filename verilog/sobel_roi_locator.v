`timescale 1ns / 1ps
// FPGA-friendly license-plate ROI proposal from a binary Sobel stream.
//
// The image is divided into 32x32 tiles.  As pixels arrive, this block scores
// every 4x2-tile (128x64) window and retains the window containing the most
// edge pixels.  That shape is intentionally plate-like, while avoiding a full
// image-sized edge buffer or connected-component engine.  At the next frame
// boundary the winning box is published and remains stable for the complete
// following frame.  The PC adds padding and runs FastALPR on the corresponding
// grayscale crop, falling back to the complete grayscale frame if necessary.
//
// Raster position is counted from edge_valid because sobel_stream emits
// exactly one result per input.

module sobel_roi_locator #(
    parameter integer IMAGE_WIDTH = 640,
    parameter integer TILE_SHIFT  = 5,
    parameter integer TILE_COLS   = 20,
    parameter integer WINDOW_COLS = 4,
    parameter [13:0]  MIN_SCORE   = 14'd350
) (
    input  wire        clk,
    input  wire        rstn,
    input  wire        frame_start,
    input  wire        edge_valid,
    input  wire [7:0]  edge_pixel,
    output reg         roi_valid,
    output reg  [15:0] roi_x0,
    output reg  [15:0] roi_y0,
    output reg  [15:0] roi_x1,
    output reg  [15:0] roi_y1,
    output reg  [15:0] roi_score
);
    localparam integer TILE_SIZE = (1 << TILE_SHIFT);

    reg [9:0] x;
    reg [9:0] y;
    reg [10:0] tile_count [0:TILE_COLS-1];
    reg [10:0] previous_band [0:TILE_COLS-1];

    // The last three completed tiles avoid four reads from tile_count when a
    // plate-width window is scored.  The corresponding registers from the
    // previous band form a 4x2-tile window.
    reg [10:0] row_t1, row_t2, row_t3;
    reg [10:0] prev_t1, prev_t2, prev_t3;

    reg [13:0] best_score;
    reg [15:0] best_x0, best_y0, best_x1, best_y1;

    wire edge_on = |edge_pixel;
    wire [4:0] tile_col = x >> TILE_SHIFT;
    wire [4:0] tile_row = y >> TILE_SHIFT;
    wire tile_first_pixel = (x[TILE_SHIFT-1:0] == 0) &&
                            (y[TILE_SHIFT-1:0] == 0);
    wire tile_last_pixel  = (&x[TILE_SHIFT-1:0]) &&
                            (&y[TILE_SHIFT-1:0]);
    wire band_last_pixel  = (x == IMAGE_WIDTH - 1) &&
                            (&y[TILE_SHIFT-1:0]);

    wire [10:0] finished_tile = tile_count[tile_col] + edge_on;
    wire [10:0] previous_tile = previous_band[tile_col];
    wire [13:0] window_score =
        {3'b000, row_t3} + {3'b000, row_t2} +
        {3'b000, row_t1} + {3'b000, finished_tile} +
        {3'b000, prev_t3} + {3'b000, prev_t2} +
        {3'b000, prev_t1} + {3'b000, previous_tile};

    integer i;
    initial begin
        x = 0;
        y = 0;
        roi_valid = 0;
        roi_x0 = 0; roi_y0 = 0; roi_x1 = 0; roi_y1 = 0; roi_score = 0;
        row_t1 = 0; row_t2 = 0; row_t3 = 0;
        prev_t1 = 0; prev_t2 = 0; prev_t3 = 0;
        best_score = 0;
        best_x0 = 0; best_y0 = 0; best_x1 = 0; best_y1 = 0;
        for (i = 0; i < TILE_COLS; i = i + 1) begin
            tile_count[i] = 0;
            previous_band[i] = 0;
        end
    end

    always @(posedge clk) begin
        if (!rstn) begin
            x <= 0;
            y <= 0;
            roi_valid <= 1'b0;
            roi_x0 <= 0; roi_y0 <= 0; roi_x1 <= 0; roi_y1 <= 0;
            roi_score <= 0;
            row_t1 <= 0; row_t2 <= 0; row_t3 <= 0;
            prev_t1 <= 0; prev_t2 <= 0; prev_t3 <= 0;
            best_score <= 0;
            best_x0 <= 0; best_y0 <= 0; best_x1 <= 0; best_y1 <= 0;
            for (i = 0; i < TILE_COLS; i = i + 1) begin
                tile_count[i] <= 0;
                previous_band[i] <= 0;
            end
        end else if (frame_start) begin
            // Publish the frame that just completed, then begin scoring the
            // new frame.  Coordinates are inclusive at both ends.
            roi_valid <= (best_score >= MIN_SCORE);
            roi_x0 <= best_x0;
            roi_y0 <= best_y0;
            roi_x1 <= best_x1;
            roi_y1 <= best_y1;
            roi_score <= {2'b00, best_score};
            x <= 0;
            y <= 0;
            row_t1 <= 0; row_t2 <= 0; row_t3 <= 0;
            prev_t1 <= 0; prev_t2 <= 0; prev_t3 <= 0;
            best_score <= 0;
            best_x0 <= 0; best_y0 <= 0; best_x1 <= 0; best_y1 <= 0;
        end else if (edge_valid) begin
            if (tile_first_pixel)
                tile_count[tile_col] <= edge_on ? 11'd1 : 11'd0;
            else if (edge_on)
                tile_count[tile_col] <= tile_count[tile_col] + 1'b1;

            if (tile_last_pixel) begin
                row_t3 <= row_t2;
                row_t2 <= row_t1;
                row_t1 <= finished_tile;
                prev_t3 <= prev_t2;
                prev_t2 <= prev_t1;
                prev_t1 <= previous_tile;

                if ((tile_col >= WINDOW_COLS - 1) && (tile_row >= 1) &&
                    (window_score > best_score)) begin
                    best_score <= window_score;
                    best_x0 <= x - (WINDOW_COLS * TILE_SIZE - 1);
                    best_y0 <= y - (2 * TILE_SIZE - 1);
                    best_x1 <= x;
                    best_y1 <= y;
                end
            end

            if (band_last_pixel) begin
                // Save this completed tile band for the next 32 rows.
                for (i = 0; i < TILE_COLS; i = i + 1) begin
                    if (i == TILE_COLS - 1)
                        previous_band[i] <= finished_tile;
                    else
                        previous_band[i] <= tile_count[i];
                end
                row_t1 <= 0; row_t2 <= 0; row_t3 <= 0;
                prev_t1 <= 0; prev_t2 <= 0; prev_t3 <= 0;
            end

            if (x == IMAGE_WIDTH - 1) begin
                x <= 0;
                y <= y + 1'b1;
            end else begin
                x <= x + 1'b1;
            end
        end
    end

endmodule
