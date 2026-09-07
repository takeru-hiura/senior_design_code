`timescale 1ns / 1ps
// Top-level data flow:
//   OV7670 RGB565 -> grayscale -> streaming Sobel -> frame BRAM -> UDP -> PC
//   PC FastALPR text -> USB-UART -> 16-character buffer -> SSD1306 OLED
//
// This module owns board-level clock/reset sequencing and open-drain pin wiring.
// PCLK must use JB10/H16, a clock-capable input. The camera is configured by
// SCCB after clock lock; Ethernet and the display operate independently.
//
// References:
//   Nexys4 DDR manual: https://digilent.com/reference/_media/nexys4-ddr/nexys4ddr_rm.pdf
//   OV7670 datasheet: https://strawberry-linux.com/pub/OV7670.pdf
//   LAN8720A datasheet: https://ww1.microchip.com/downloads/en/devicedoc/8720a.pdf
//   SSD1306 datasheet: https://www.sunrom.com/download/SSD1306.pdf

module ov7670_nexys4ddr_top (
    input  wire        CLK100MHZ,
    input  wire        CPU_RESETN,
    input  wire        edge_enable,
    input  wire [7:0]  cam_d,
    input  wire        cam_pclk,
    input  wire        cam_href,
    input  wire        cam_vsync,
    output wire        cam_xclk,
    output wire        cam_reset,
    output wire        cam_pwdn,
    inout  wire        cam_sioc,
    inout  wire        cam_siod,
    output wire        eth_refclk,
    output wire        eth_rstn,
    output wire        eth_txen,
    output wire [1:0]  eth_txd,
    output wire        eth_mdc,
    inout  wire        eth_mdio,
    inout  wire        oled_scl,
    inout  wire        oled_sda,
    input  wire        uart_rxd
);
    wire clk_25;
    wire clk_eth;
    wire clk_locked;
    wire pclk;
    wire pclk_ibuf;

    wire        config_done;
    wire [7:0]  rom_addr;
    wire [15:0] rom_data;
    wire        sccb_ready;
    wire [7:0]  sccb_reg;
    wire [7:0]  sccb_val;
    wire        sccb_start;
    wire        sioc_oe;
    wire        siod_oe;

    wire [18:0] wr_addr;
    wire [7:0]  wr_data;
    wire        wr_en;
    wire [18:0] edge_addr;
    wire [7:0]  edge_data;
    wire        edge_en;
    (* ASYNC_REG = "TRUE" *) reg edge_enable_meta, edge_enable_p;
    reg edge_mode_frame;
    wire [18:0] fb_wr_addr = edge_mode_frame ? edge_addr : wr_addr;
    wire [7:0]  fb_wr_data = edge_mode_frame ? edge_data : wr_data;
    wire        fb_wr_en   = edge_mode_frame ? edge_en   : wr_en;
    wire [3:0]  fb_wr_pixel = fb_wr_data[7:4];
    wire [3:0]  rd_pixel_4;
    // Replicate the stored nibble so the UDP interface remains 8-bit and its
    // brightness range remains 0..255 (0x0 -> 0x00, 0xF -> 0xFF).
    wire [7:0]  rd_pixel = {rd_pixel_4, rd_pixel_4};
    wire [18:0] eth_fb_addr;

    (* IOB = "TRUE" *) reg [7:0] cam_d_q;
    (* IOB = "TRUE" *) reg       cam_href_q;
    (* ASYNC_REG = "TRUE" *) reg config_done_meta, config_done_p;
    (* ASYNC_REG = "TRUE" *) reg [2:0] vs_25;
    (* ASYNC_REG = "TRUE" *) reg [2:0] frame_tog_p;
    reg frame_tog;

    reg [23:0] boot_cnt;
    reg        cam_reset_r;
    reg        config_go;

    // Generate the 25 MHz camera/control clock and 50 MHz RMII clock.
    clocks u_clocks (
        .clk100_in (CLK100MHZ),
        .resetn    (CPU_RESETN),
        .clk_25    (clk_25),
        .clk_eth   (clk_eth),
        .locked    (clk_locked)
    );

    ODDR #(
        .DDR_CLK_EDGE ("OPPOSITE_EDGE"),
        .INIT         (1'b0),
        .SRTYPE       ("SYNC")
    ) u_xclk (
        .Q  (cam_xclk),
        .C  (clk_25),
        .CE (1'b1),
        .D1 (1'b1),
        .D2 (1'b0),
        .R  (1'b0),
        .S  (1'b0)
    );

    IBUF u_pclk_ibuf (
        .I (cam_pclk),
        .O (pclk_ibuf)
    );
    BUFG u_pclk_bufg (
        .I (pclk_ibuf),
        .O (pclk)
    );

    // Register camera pins at the I/O boundary and synchronize the board
    // switch into the camera pixel-clock domain.
    always @(posedge pclk) begin
        if (!CPU_RESETN) begin
            cam_d_q    <= 8'd0;
            cam_href_q <= 1'b0;
            edge_enable_meta <= 1'b0;
            edge_enable_p    <= 1'b0;
            edge_mode_frame  <= 1'b0;
        end else begin
            cam_d_q    <= cam_d;
            cam_href_q <= cam_href;
            edge_enable_meta <= edge_enable;
            edge_enable_p    <= edge_enable_meta;

            // Apply a switch change only at a frame boundary. This prevents
            // one transmitted frame from containing a mixture of grayscale
            // and Sobel pixels when SW0 is moved during active video.
            if (frame_tog_p[1] ^ frame_tog_p[2])
                edge_mode_frame <= edge_enable_p;
        end
    end

    // Configuration status and frame events originate in clk_25. Two-stage
    // synchronizers make both safe to consume in the PCLK domain.
    always @(posedge pclk) begin
        if (!CPU_RESETN) begin
            config_done_meta <= 1'b0;
            config_done_p    <= 1'b0;
            frame_tog_p      <= 3'b000;
        end else begin
            config_done_meta <= config_done;
            config_done_p    <= config_done_meta;
            frame_tog_p      <= {frame_tog_p[1:0], frame_tog};
        end
    end

    // VSYNC is sampled on the 25 MHz clock so a stopped PCLK cannot miss frame
    // starts. A toggle carries each detected edge safely into the PCLK domain.
    always @(posedge clk_25) begin
        if (!CPU_RESETN || !clk_locked) begin
            vs_25     <= 3'b000;
            frame_tog <= 1'b0;
        end else begin
            vs_25 <= {vs_25[1:0], cam_vsync};
            if (vs_25[1] & ~vs_25[2])
                frame_tog <= ~frame_tog;
        end
    end

    assign cam_pwdn  = 1'b0;
    assign cam_reset = cam_reset_r;
    assign cam_sioc  = sioc_oe ? 1'b0 : 1'bz;
    assign cam_siod  = siod_oe ? 1'b0 : 1'bz;

    // Hold the camera in reset during clock startup, then launch its SCCB
    // configuration after roughly 120 ms at 25 MHz.
    always @(posedge clk_25) begin
        if (!CPU_RESETN || !clk_locked) begin
            boot_cnt    <= 24'd0;
            cam_reset_r <= 1'b0;
            config_go   <= 1'b0;
        end else begin
            config_go <= 1'b0;
            if (boot_cnt != 24'd3_000_000)
                boot_cnt <= boot_cnt + 1'b1;
            cam_reset_r <= (boot_cnt > 24'd25_000);
            if (boot_cnt == 24'd2_999_999)
                config_go <= 1'b1;
        end
    end

    ov7670_config_rom u_rom (
        .clk  (clk_25),
        .addr (rom_addr),
        .data (rom_data)
    );

    ov7670_config #(
        .CLK_FREQ (25_000_000)
    ) u_config (
        .clk        (clk_25),
        .start      (config_go),
        .sccb_ready (sccb_ready),
        .rom_data   (rom_data),
        .rom_addr   (rom_addr),
        .sccb_reg   (sccb_reg),
        .sccb_val   (sccb_val),
        .sccb_start (sccb_start),
        .done       (config_done)
    );

    ov7670_sccb #(
        .CLK_FREQ  (25_000_000),
        .SCCB_FREQ (100_000)
    ) u_sccb (
        .clk      (clk_25),
        .start    (sccb_start),
        .reg_addr (sccb_reg),
        .reg_data (sccb_val),
        .ready    (sccb_ready),
        .sioc_oe  (sioc_oe),
        .siod_oe  (siod_oe)
    );

    ov7670_capture u_capture (
        .pclk      (pclk),
        .href      (cam_href_q),
        .data      (cam_d_q),
        .enable    (config_done_p),
        .new_frame (frame_tog_p[1] ^ frame_tog_p[2]),
        .wr_addr   (wr_addr),
        .wr_data   (wr_data),
        .wr_en     (wr_en)
    );

    // FPGA vision accelerator: two line buffers form a 3x3 window and a
    // fully streaming Sobel datapath emits one thresholded edge pixel per
    // valid camera pixel. THRESHOLD is a synthesis-time starting point; a
    // SW0 selects its edge output (1) or grayscale bypass (0), enabling direct
    // hardware/software A/B captures without rebuilding the bitstream. The
    // synchronized selection is latched at VSYNC so each frame has one mode.
    sobel_stream #(
        .IMAGE_WIDTH (640),
        .THRESHOLD   (8'd80),
        .BINARY_OUT  (1)
    ) u_sobel (
        .clk         (pclk),
        .rstn        (CPU_RESETN & config_done_p),
        .frame_start (frame_tog_p[1] ^ frame_tog_p[2]),
        .in_valid    (wr_en),
        .in_addr     (wr_addr),
        .in_pixel    (wr_data),
        .out_valid   (edge_en),
        .out_addr    (edge_addr),
        .out_pixel   (edge_data)
    );

    frame_buffer #(
        .DW (4)
    ) u_fb (
        .wr_clk  (pclk),
        .wr_en   (fb_wr_en),
        .wr_addr (fb_wr_addr),
        .wr_data (fb_wr_pixel),
        .rd_clk  (clk_25),
        .rd_addr (eth_fb_addr),
        .rd_data (rd_pixel_4)
    );

    // Invert REFCLK (LiteEth DDROutput i1=0,i2=1). LAN8720 samples TXD/TXEN
    // on REFCLK rising; that edge is clk_eth falling, 10 ns after the IOB FFs
    // update (4 ns setup / 1.5 ns hold at 100 Mbps). Do not use a 0-degree
    // REFCLK — that fails hold and is why earlier 100 Mbps frames were dropped.
    ODDR #(
        .DDR_CLK_EDGE ("OPPOSITE_EDGE"),
        .INIT         (1'b0),
        .SRTYPE       ("SYNC")
    ) u_eth_refclk (
        .Q  (eth_refclk),
        .C  (clk_eth),
        .CE (1'b1),
        .D1 (1'b0),
        .D2 (1'b1),
        .R  (1'b0),
        .S  (1'b0)
    );

    reg [21:0] phy_rst_cnt;
    reg        eth_rstn_r;
    always @(posedge clk_eth) begin
        if (!CPU_RESETN || !clk_locked) begin
            phy_rst_cnt <= 22'd0;
            eth_rstn_r  <= 1'b0;
        end else begin
            if (phy_rst_cnt != 22'h3FFFFF)
                phy_rst_cnt <= phy_rst_cnt + 1'b1;
            eth_rstn_r <= (phy_rst_cnt > 22'd2500000);
        end
    end

    assign eth_rstn = eth_rstn_r;

    eth_phy_link u_phy (
        .clk   (clk_eth),
        .rstn  (CPU_RESETN & clk_locked & eth_rstn_r),
        .mdc   (eth_mdc),
        .mdio  (eth_mdio)
    );

    eth_video_udp u_eth (
        .clk_fb    (clk_25),
        .clk_eth   (clk_eth),
        .rstn_fb   (CPU_RESETN & clk_locked & eth_rstn_r),
        .rstn_eth  (CPU_RESETN & clk_locked & eth_rstn_r),
        .enable    (config_done & eth_rstn_r),
        .fb_data   (rd_pixel),
        .fb_addr   (eth_fb_addr),
        .eth_txen  (eth_txen),
        .eth_txd   (eth_txd)
    );

    wire        oled_scl_oe;
    wire        oled_sda_oe;
    wire [127:0] oled_line1;
    wire [127:0] oled_line2;
    wire        oled_update;
    reg         oled_go;
    reg         oled_started;

    assign oled_scl = oled_scl_oe ? 1'b0 : 1'bz;
    assign oled_sda = oled_sda_oe ? 1'b0 : 1'bz;

    always @(posedge clk_25) begin
        if (!CPU_RESETN || !clk_locked) begin
            oled_go      <= 1'b0;
            oled_started <= 1'b0;
        end else if (!oled_started) begin
            oled_go      <= 1'b1;
            oled_started <= 1'b1;
        end else
            oled_go <= 1'b0;
    end

    plate_uart #(
        .CLK_FREQ (25_000_000)
    ) u_plate (
        .clk    (clk_25),
        .rstn   (CPU_RESETN & clk_locked),
        .rx     (uart_rxd),
        .line1  (oled_line1),
        .line2  (oled_line2),
        .update (oled_update)
    );

    ssd1306 #(
        .CLK_FREQ (25_000_000),
        .ADDR     (7'h3C),
        .HEIGHT   (64)
    ) u_oled (
        .clk     (clk_25),
        .rstn    (CPU_RESETN & clk_locked),
        .start   (oled_go),
        .update  (oled_update),
        .line1   (oled_line1),
        .line2   (oled_line2),
        .done    (),
        .ack_err (),
        .scl_oe  (oled_scl_oe),
        .sda_oe  (oled_sda_oe),
        .sda_in  (oled_sda)
    );

endmodule
