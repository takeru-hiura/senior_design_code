`timescale 1ns / 1ps
// Purpose: packetize the grayscale frame buffer into Ethernet/IPv4/UDP frames
// for the PC. The frame-buffer clock domain fills a 192-byte packet RAM; a
// toggle synchronizer tells the Ethernet domain when that packet is ready.
//
// The format-3 project-specific UDP header is 26 bytes, in network byte order:
//   "N4VD", frame_id, packet_index, packet_count, width, height, format,
//   roi_x0, roi_y0, roi_x1, roi_y1, roi_score.
// Coordinates are inclusive. A score of zero means that no ROI met the FPGA's
// minimum edge-density threshold. Format 2 (the former 16-byte header) remains
// supported by the PC receiver for use with older bitstreams.
//
// The addresses/checksums are constants because every datagram has the same
// size and fixed endpoints. Change DEST_MAC if the PC adapter's MAC changes.
// References:
//   RFC 791 (IPv4): https://www.rfc-editor.org/rfc/rfc791
//   RFC 768 (UDP):  https://www.rfc-editor.org/rfc/rfc768
//   Microchip LAN8720A: https://ww1.microchip.com/downloads/en/devicedoc/8720a.pdf

module eth_video_udp (
    input  wire        clk_fb,
    input  wire        clk_eth,
    input  wire        rstn_fb,
    input  wire        rstn_eth,
    input  wire        enable,
    input  wire [7:0]  fb_data,
    output reg  [18:0] fb_addr,
    input  wire        roi_valid,
    input  wire [15:0] roi_x0,
    input  wire [15:0] roi_y0,
    input  wire [15:0] roi_x1,
    input  wire [15:0] roi_y1,
    input  wire [15:0] roi_score,
    output wire        eth_txen,
    output wire [1:0]  eth_txd
);

    // 307,200 pixels / 192 pixels per packet = 1,600 packets per frame.
    localparam integer PIX_PER_PKT = 192;
    localparam integer PKT_BYTES   = 260;
    localparam [15:0]  PKT_CNT     = 16'd1600;
    localparam [47:0]  DEST_MAC    = 48'h00E0_4C51_1930;
    localparam [47:0]  SRC_MAC     = 48'h0200_0000_0001;
    localparam [31:0]  SRC_IP      = 32'hC0A8_010A; // 192.168.1.10
    localparam [31:0]  DEST_IP     = 32'hC0A8_0102; // 192.168.1.2

    localparam ST_IDLE = 2'd0;
    localparam ST_FILL = 2'd1;
    localparam ST_KICK = 2'd2;
    localparam ST_WAIT = 2'd3;

    // One packet is staged here before crossing from clk_fb to clk_eth. The
    // toggle handshake keeps the multi-bit payload stable while it is read.
    reg [7:0]  pix [0:191];
    reg [1:0]  vstate;
    reg [7:0]  pix_i;
    reg [18:0] pix_base;
    reg [15:0] frame_id;
    reg [15:0] pkt_idx;
    reg        fill_tog;
    reg [80:0] roi_meta_fb, roi_sync_fb;
    reg        roi_valid_frame;
    reg [15:0] roi_x0_frame, roi_y0_frame, roi_x1_frame, roi_y1_frame;
    reg [15:0] roi_score_frame;

    (* ASYNC_REG = "TRUE" *) reg [2:0] busy_sync;
    (* ASYNC_REG = "TRUE" *) reg [2:0] fill_sync;
    wire tx_busy;

    always @(posedge clk_fb) begin
        if (!rstn_fb) begin
            busy_sync <= 3'b000;
            roi_meta_fb <= 81'd0;
            roi_sync_fb <= 81'd0;
        end else begin
            busy_sync <= {busy_sync[1:0], tx_busy};
            // The ROI result is stable for a camera frame. Two vector stages
            // provide ample settling time before it is latched for a complete
            // transmitted frame below.
            roi_meta_fb <= {roi_valid, roi_x0, roi_y0, roi_x1, roi_y1, roi_score};
            roi_sync_fb <= roi_meta_fb;
        end
    end
    wire tx_busy_v = busy_sync[2];

    // Frame-buffer side: copy 192 sequential pixels into the packet staging
    // RAM, toggle fill_tog, then wait until the transmitter reports busy.
    always @(posedge clk_fb) begin
        if (!rstn_fb) begin
            vstate    <= ST_IDLE;
            pix_i     <= 8'd0;
            pix_base  <= 19'd0;
            frame_id  <= 16'd0;
            pkt_idx   <= 16'd0;
            fill_tog  <= 1'b0;
            fb_addr   <= 19'd0;
            roi_valid_frame <= 1'b0;
            roi_x0_frame <= 0; roi_y0_frame <= 0;
            roi_x1_frame <= 0; roi_y1_frame <= 0; roi_score_frame <= 0;
        end else begin
            case (vstate)
                ST_IDLE: begin
                    fb_addr <= pix_base;
                    if (enable && !tx_busy_v) begin
                        if (pkt_idx == 0) begin
                            roi_valid_frame <= roi_sync_fb[80];
                            roi_x0_frame <= roi_sync_fb[79:64];
                            roi_y0_frame <= roi_sync_fb[63:48];
                            roi_x1_frame <= roi_sync_fb[47:32];
                            roi_y1_frame <= roi_sync_fb[31:16];
                            roi_score_frame <= roi_sync_fb[80] ? roi_sync_fb[15:0] : 16'd0;
                        end
                        vstate <= ST_FILL;
                    end
                end
                ST_FILL: begin
                    pix[pix_i] <= fb_data;
                    if (pix_i == PIX_PER_PKT - 1) begin
                        pix_i  <= 8'd0;
                        vstate <= ST_KICK;
                    end else begin
                        pix_i   <= pix_i + 1'b1;
                        fb_addr <= pix_base + {11'd0, pix_i} + 19'd1;
                    end
                end
                ST_KICK: begin
                    fill_tog <= ~fill_tog;
                    vstate   <= ST_WAIT;
                end
                ST_WAIT: begin
                    if (tx_busy_v) begin
                        if (pkt_idx == PKT_CNT - 1) begin
                            pkt_idx  <= 16'd0;
                            pix_base <= 19'd0;
                            frame_id <= frame_id + 1'b1;
                        end else begin
                            pkt_idx  <= pkt_idx + 1'b1;
                            pix_base <= pix_base + PIX_PER_PKT[18:0];
                        end
                        vstate <= ST_IDLE;
                    end
                end
                default: vstate <= ST_IDLE;
            endcase
        end
    end

    always @(posedge clk_eth) begin
        if (!rstn_eth)
            fill_sync <= 3'b000;
        else
            fill_sync <= {fill_sync[1:0], fill_tog};
    end
    wire fill_pulse = fill_sync[1] ^ fill_sync[2];

    wire [10:0] tx_addr;
    wire [7:0]  tx_data;
    reg         tx_start;
    reg [15:0]  frame_id_e;
    reg [15:0]  pkt_idx_e;
    reg         roi_valid_e;
    reg [15:0]  roi_x0_e, roi_y0_e, roi_x1_e, roi_y1_e, roi_score_e;

    // Ethernet, IPv4, UDP, and N4VD header bytes are generated directly from
    // the requested transmit address; only the video payload needs RAM.
    function [7:0] hdr_byte;
        input [7:0]  i;
        input [15:0] fid;
        input [15:0] pid;
        begin
            case (i)
                8'd0:  hdr_byte = DEST_MAC[47:40];
                8'd1:  hdr_byte = DEST_MAC[39:32];
                8'd2:  hdr_byte = DEST_MAC[31:24];
                8'd3:  hdr_byte = DEST_MAC[23:16];
                8'd4:  hdr_byte = DEST_MAC[15:8];
                8'd5:  hdr_byte = DEST_MAC[7:0];
                8'd6:  hdr_byte = SRC_MAC[47:40];
                8'd7:  hdr_byte = SRC_MAC[39:32];
                8'd8:  hdr_byte = SRC_MAC[31:24];
                8'd9:  hdr_byte = SRC_MAC[23:16];
                8'd10: hdr_byte = SRC_MAC[15:8];
                8'd11: hdr_byte = SRC_MAC[7:0];
                8'd12: hdr_byte = 8'h08;
                8'd13: hdr_byte = 8'h00;
                8'd14: hdr_byte = 8'h45;
                8'd15: hdr_byte = 8'h00;
                8'd16: hdr_byte = 8'h00;
                8'd17: hdr_byte = 8'hF6;
                8'd18: hdr_byte = 8'h00;
                8'd19: hdr_byte = 8'h00;
                8'd20: hdr_byte = 8'h40;
                8'd21: hdr_byte = 8'h00;
                8'd22: hdr_byte = 8'h40;
                8'd23: hdr_byte = 8'h11;
                8'd24: hdr_byte = 8'hB6;
                8'd25: hdr_byte = 8'h9A;
                8'd26: hdr_byte = SRC_IP[31:24];
                8'd27: hdr_byte = SRC_IP[23:16];
                8'd28: hdr_byte = SRC_IP[15:8];
                8'd29: hdr_byte = SRC_IP[7:0];
                8'd30: hdr_byte = DEST_IP[31:24];
                8'd31: hdr_byte = DEST_IP[23:16];
                8'd32: hdr_byte = DEST_IP[15:8];
                8'd33: hdr_byte = DEST_IP[7:0];
                8'd34: hdr_byte = 8'h13;
                8'd35: hdr_byte = 8'h88;
                8'd36: hdr_byte = 8'h13;
                8'd37: hdr_byte = 8'h88;
                8'd38: hdr_byte = 8'h00;
                8'd39: hdr_byte = 8'hE2;
                8'd40: hdr_byte = 8'h00;
                8'd41: hdr_byte = 8'h00;
                8'd42: hdr_byte = 8'h4E;
                8'd43: hdr_byte = 8'h34;
                8'd44: hdr_byte = 8'h56;
                8'd45: hdr_byte = 8'h44;
                8'd46: hdr_byte = fid[15:8];
                8'd47: hdr_byte = fid[7:0];
                8'd48: hdr_byte = pid[15:8];
                8'd49: hdr_byte = pid[7:0];
                8'd50: hdr_byte = PKT_CNT[15:8];
                8'd51: hdr_byte = PKT_CNT[7:0];
                8'd52: hdr_byte = 8'h02;
                8'd53: hdr_byte = 8'h80;
                8'd54: hdr_byte = 8'h01;
                8'd55: hdr_byte = 8'hE0;
                8'd56: hdr_byte = 8'h00;
                8'd57: hdr_byte = 8'h03; // format 3: grayscale plus ROI metadata
                8'd58: hdr_byte = roi_valid_e ? roi_x0_e[15:8] : 8'h00;
                8'd59: hdr_byte = roi_valid_e ? roi_x0_e[7:0]  : 8'h00;
                8'd60: hdr_byte = roi_valid_e ? roi_y0_e[15:8] : 8'h00;
                8'd61: hdr_byte = roi_valid_e ? roi_y0_e[7:0]  : 8'h00;
                8'd62: hdr_byte = roi_valid_e ? roi_x1_e[15:8] : 8'h00;
                8'd63: hdr_byte = roi_valid_e ? roi_x1_e[7:0]  : 8'h00;
                8'd64: hdr_byte = roi_valid_e ? roi_y1_e[15:8] : 8'h00;
                8'd65: hdr_byte = roi_valid_e ? roi_y1_e[7:0]  : 8'h00;
                8'd66: hdr_byte = roi_valid_e ? roi_score_e[15:8] : 8'h00;
                8'd67: hdr_byte = roi_valid_e ? roi_score_e[7:0]  : 8'h00;
                default: hdr_byte = 8'h00;
            endcase
        end
    endfunction

    wire [7:0] pix_n = tx_addr - 11'd68;

    assign tx_data = (tx_addr < 11'd68) ? hdr_byte(tx_addr[7:0], frame_id_e, pkt_idx_e)
                                        : pix[pix_n];

    eth_rmii_tx u_tx (
        .clk       (clk_eth),
        .rstn      (rstn_eth),
        .start     (tx_start),
        .length    (PKT_BYTES[10:0]),
        .data      (tx_data),
        .data_addr (tx_addr),
        .busy      (tx_busy),
        .tx_en     (eth_txen),
        .txd       (eth_txd)
    );

    always @(posedge clk_eth) begin
        if (!rstn_eth) begin
            tx_start   <= 1'b0;
            frame_id_e <= 16'd0;
            pkt_idx_e  <= 16'd0;
            roi_valid_e <= 1'b0;
            roi_x0_e <= 0; roi_y0_e <= 0; roi_x1_e <= 0; roi_y1_e <= 0;
            roi_score_e <= 0;
        end else begin
            tx_start <= 1'b0;
            if (fill_pulse && !tx_busy) begin
                tx_start   <= 1'b1;
                frame_id_e <= frame_id;
                pkt_idx_e  <= pkt_idx;
                roi_valid_e <= roi_valid_frame;
                roi_x0_e <= roi_x0_frame;
                roi_y0_e <= roi_y0_frame;
                roi_x1_e <= roi_x1_frame;
                roi_y1_e <= roi_y1_frame;
                roi_score_e <= roi_score_frame;
            end
        end
    end
endmodule
