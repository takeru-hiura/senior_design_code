`timescale 1ns / 1ps


// Purpose: serialize one complete Ethernet MAC frame onto the LAN8720A RMII
// transmit interface. The caller supplies bytes from destination MAC through
// the final payload byte; this block adds the 7-byte preamble, SFD, CRC/FCS,
// and inter-packet gap. Each byte is emitted as four 2-bit RMII symbols at
// 50 MHz, least-significant pair first.
//
// `length` excludes preamble/SFD and FCS. `data_addr` requests the current byte
// from the caller. `busy` remains high until the mandatory idle gap completes.
//
// References:
//   Microchip LAN8720A datasheet (RMII transmit timing)
//   https://ww1.microchip.com/downloads/en/devicedoc/8720a.pdf
//   IEEE Std 802.3, Ethernet frame format, FCS and interpacket gap.
module eth_rmii_tx (
    input  wire        clk,
    input  wire        rstn,
    input  wire        start,
    input  wire [10:0] length,
    input  wire [7:0]  data,
    output reg  [10:0] data_addr,
    output reg         busy,
    (* IOB = "TRUE" *) output reg tx_en,
    (* IOB = "TRUE" *) output reg [1:0] txd
);
    localparam ST_IDLE = 3'd0;
    localparam ST_PRE  = 3'd1;
    localparam ST_BYTE = 3'd2;
    localparam ST_CRC  = 3'd3;
    localparam ST_IPG  = 3'd4;

    reg [2:0]   state;
    reg [1:0]   nib;
    reg [4:0]   pre_n;
    reg [10:0]  byte_idx;
    reg [10:0]  pkt_len;
    reg [10:0]  ipg_cnt;
    reg [1:0]   crc_sel;
    reg         tx_en_n;
    reg [1:0]   txd_n;

    wire        crc_rst = (state == ST_IDLE);
    wire        crc_en  = (state == ST_BYTE) && (nib == 2'd3);
    wire [31:0] crc;
    wire [31:0] fcs = ~crc;

    eth_crc32 u_crc (
        .clk  (clk),
        .rst  (crc_rst),
        .en   (crc_en),
        .data (data),
        .crc  (crc)
    );

    wire [7:0] pre_byte = (pre_n[4:2] == 3'd7) ? 8'hD5 : 8'h55;

    function [1:0] pair;
        input [7:0] b;
        input [1:0] n;
        begin
            case (n)
                2'd0: pair = b[1:0];
                2'd1: pair = b[3:2];
                2'd2: pair = b[5:4];
                default: pair = b[7:6];
            endcase
        end
    endfunction

    always @(posedge clk) begin
        if (!rstn) begin
            tx_en <= 1'b0;
            txd   <= 2'b00;
        end else begin
            tx_en <= tx_en_n;
            txd   <= txd_n;
        end
    end

    always @(posedge clk) begin
        if (!rstn) begin
            state     <= ST_IDLE;
            busy      <= 1'b0;
            tx_en_n   <= 1'b0;
            txd_n     <= 2'b00;
            data_addr <= 11'd0;
            nib       <= 2'd0;
            pre_n     <= 5'd0;
            byte_idx  <= 11'd0;
            crc_sel   <= 2'd0;
        end else begin
            case (state)
                ST_IDLE: begin
                    tx_en_n   <= 1'b0;
                    txd_n     <= 2'b00;
                    busy      <= 1'b0;
                    data_addr <= 11'd0;
                    nib       <= 2'd0;
                    if (start && length != 11'd0) begin
                        busy    <= 1'b1;
                        pkt_len <= length;
                        pre_n   <= 5'd0;
                        state   <= ST_PRE;
                    end
                end

                ST_PRE: begin
                    tx_en_n <= 1'b1;
                    txd_n   <= pair(pre_byte, pre_n[1:0]);
                    if (pre_n == 5'd31) begin
                        byte_idx  <= 11'd0;
                        data_addr <= 11'd0;
                        nib       <= 2'd0;
                        state     <= ST_BYTE;
                    end else
                        pre_n <= pre_n + 1'b1;
                end

                ST_BYTE: begin
                    tx_en_n <= 1'b1;
                    txd_n   <= pair(data, nib);
                    if (nib == 2'd3) begin
                        nib <= 2'd0;
                        if (byte_idx == pkt_len - 1) begin
                            crc_sel <= 2'd0;
                            state   <= ST_CRC;
                        end else begin
                            byte_idx  <= byte_idx + 1'b1;
                            data_addr <= byte_idx + 1'b1;
                        end
                    end else
                        nib <= nib + 1'b1;
                end

                ST_CRC: begin
                    tx_en_n <= 1'b1;
                    case (crc_sel)
                        2'd0:    txd_n <= pair(fcs[7:0],    nib);
                        2'd1:    txd_n <= pair(fcs[15:8],   nib);
                        2'd2:    txd_n <= pair(fcs[23:16],  nib);
                        default: txd_n <= pair(fcs[31:24],  nib);
                    endcase
                    if (nib == 2'd3) begin
                        nib <= 2'd0;
                        if (crc_sel == 2'd3) begin
                            // Keep last FCS dibit on the wire this cycle.
                            // Clearing TXEN here drops 2 bits; at 10 Mbps the
                            // 10-cycle hold hid that, at 100 Mbps the NIC
                            // discards every frame (activity blinks, no UDP).
                            ipg_cnt <= 11'd0;
                            state   <= ST_IPG;
                        end else
                            crc_sel <= crc_sel + 1'b1;
                    end else
                        nib <= nib + 1'b1;
                end

                ST_IPG: begin
                    tx_en_n <= 1'b0;
                    txd_n   <= 2'b00;
                    ipg_cnt <= ipg_cnt + 1'b1;
                    if (ipg_cnt == 11'd47)
                        state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
