`timescale 1ns / 1ps
// Purpose: hold one 640x480 grayscale frame between the asynchronous camera
// write domain and packetizer read domain. Pixels are stored as their upper
// four grayscale bits, reducing the full-frame BRAM requirement by about half.
// The top level expands each sample back to 8 bits before UDP transmission.
//
// Vivado infers a simple dual-port block RAM. Reads are synchronous: `rd_data`
// changes on the clock edge after `rd_addr` is presented, so a consumer must
// account for that one-cycle BRAM latency. This is a single buffer; the camera
// can overwrite pixels while Ethernet is reading the preceding frame.
//
// Reference: AMD UG901, RAM HDL Coding Techniques
// https://docs.amd.com/r/2023.2-English/ug901-vivado-synthesis/RAM-HDL-Coding-Techniques

// AW covers all 307,200 addresses. DW defaults to four because the selected
// video is quantized before storage to keep total BRAM utilization below 50%.
module frame_buffer #(
    parameter integer AW    = 19,
    parameter integer DW    = 4,
    parameter integer DEPTH = 307200
) (
    input  wire            wr_clk,
    input  wire            wr_en,
    input  wire [AW-1:0]   wr_addr,
    input  wire [DW-1:0]   wr_data,
    input  wire            rd_clk,
    input  wire [AW-1:0]   rd_addr,
    output reg  [DW-1:0]   rd_data
);
    // Keep the large frame store in BRAM rather than consuming LUT memory.
    (* ram_style = "block" *) reg [DW-1:0] mem [0:DEPTH-1];

    always @(posedge wr_clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
    end

    always @(posedge rd_clk) begin
        rd_data <= mem[rd_addr];
    end
endmodule
