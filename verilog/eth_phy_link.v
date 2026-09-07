`timescale 1ns / 1ps
// Purpose: configure the Nexys4 DDR's LAN8720A PHY over its Clause 22 MDIO
// management interface. The controller advertises 100BASE-TX capability and
// restarts auto-negotiation once after reset.
//
// The 64-bit `shreg` holds one complete MDIO write transaction: 32 preamble
// ones, start, write opcode, PHY address, register address, turnaround, data.
// The controller drives MDIO during the write transaction and releases it
// after the final bit.
//
// References:
//   Microchip LAN8720A datasheet (MDIO, PHY registers and RMII)
//   https://ww1.microchip.com/downloads/en/devicedoc/8720a.pdf
//   IEEE Std 802.3, Clause 22 management interface.

module eth_phy_link (
    input  wire clk,
    input  wire rstn,
    output reg  mdc,
    inout  wire mdio
);

    // The LAN8720A is strapped to PHY address 1 on the Nexys4 DDR.
    localparam [4:0] PHY = 5'd1;

    // Divide the 50 MHz input to a conservative 500 kHz MDC clock.
    localparam DIV = 6'd50;

    // Advertise 100BASE-TX full/half duplex plus the required selector field.
    localparam [15:0] ANAR_100 = 16'h0181;

    // Enable and restart auto-negotiation after writing the advertisement.
    localparam [15:0] BMCR_AN  = 16'h1200;

    localparam ST_WAIT  = 2'd0;
    localparam ST_LOAD  = 2'd1;
    localparam ST_BITS  = 2'd2;
    localparam ST_GAP   = 2'd3;

    localparam [26:0] WAIT_BOOT = 27'd2_500_000;
    localparam [26:0] WAIT_GAP  = 27'd1_000_000;
    localparam [26:0] WAIT_AN   = 27'd90_000_000;

    reg [5:0]  div;
    reg        mdc_rise;
    reg        mdio_oe, mdio_out;
    reg [1:0]  state;
    reg        which;
    reg [6:0]  bit_n;
    reg [63:0] shreg;
    reg [26:0] wait_n;
    reg        done;

    assign mdio = mdio_oe ? mdio_out : 1'bz;

    always @(posedge clk) begin
        if (!rstn) begin
            div      <= 6'd0;
            mdc      <= 1'b0;
            mdc_rise <= 1'b0;
        end else if (done) begin
            mdc      <= 1'b0;
            mdc_rise <= 1'b0;
        end else begin
            mdc_rise <= 1'b0;
            if (div == DIV - 1) begin
                div <= 6'd0;
                mdc <= ~mdc;
                if (!mdc) mdc_rise <= 1'b1;
            end else
                div <= div + 1'b1;
        end
    end

    // Shift one management frame out on each MDC rising edge. The state
    // machine writes ANAR first, then BMCR, and leaves MDIO released.
    always @(posedge clk) begin
        if (!rstn) begin
            state    <= ST_WAIT;
            which    <= 1'b0;
            wait_n   <= 27'd0;
            done     <= 1'b0;
            mdio_oe  <= 1'b0;
            mdio_out <= 1'b1;
            bit_n    <= 7'd0;
        end else if (done) begin
            mdio_oe <= 1'b0;
        end else case (state)
            ST_WAIT: begin
                mdio_oe <= 1'b0;
                if (wait_n == WAIT_BOOT) begin
                    wait_n <= 27'd0;
                    state  <= ST_LOAD;
                end else
                    wait_n <= wait_n + 1'b1;
            end

            ST_LOAD: begin
                bit_n    <= 7'd0;
                mdio_oe  <= 1'b1;
                if (!which)
                    shreg <= {32'hFFFF_FFFF, 2'b01, 2'b01, PHY, 5'd4, 2'b10, ANAR_100};
                else
                    shreg <= {32'hFFFF_FFFF, 2'b01, 2'b01, PHY, 5'd0, 2'b10, BMCR_AN};
                mdio_out <= 1'b1;
                state    <= ST_BITS;
            end

            ST_BITS: begin
                mdio_oe  <= 1'b1;
                mdio_out <= shreg[63];
                if (mdc_rise) begin
                    shreg <= {shreg[62:0], 1'b0};
                    bit_n <= bit_n + 1'b1;
                    if (bit_n == 7'd63) begin
                        mdio_oe <= 1'b0;
                        wait_n  <= 27'd0;
                        state   <= ST_GAP;
                    end
                end
            end

            ST_GAP: begin
                mdio_oe <= 1'b0;
                if (!which) begin
                    if (wait_n == WAIT_GAP) begin
                        wait_n <= 27'd0;
                        which  <= 1'b1;
                        state  <= ST_LOAD;
                    end else
                        wait_n <= wait_n + 1'b1;
                end else if (wait_n == WAIT_AN) begin
                    wait_n <= 27'd0;
                    done   <= 1'b1;
                end else
                    wait_n <= wait_n + 1'b1;
            end

            default: state <= ST_WAIT;
        endcase
    end
endmodule
