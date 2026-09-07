`timescale 1ns / 1ps

// Purpose: derive the camera/video clock (25 MHz) and RMII reference clock
// (50 MHz) from the Nexys4 DDR's 100 MHz oscillator.
//
// The MMCME2_BASE primitive first raises the internal VCO to 1 GHz, then
// divides that clock by 40 and 20. BUFG instances distribute the two results
// on the FPGA's low-skew global clock network. Keep dependent logic in reset
// until `locked` is asserted.
//
// References:
//   AMD UG472, 7 Series FPGAs Clocking Resources
//   https://docs.amd.com/v/u/en-US/ug472_7Series_Clocking
//   AMD UG953, 7 Series Libraries Guide (MMCME2_BASE, IBUF, BUFG)
//   https://docs.amd.com/r/en-US/ug953-vivado-7series-libraries

module clocks (
    input  wire clk100_in,
    input  wire resetn,
    output wire clk_25,
    output wire clk_eth,
    output wire locked
);

    // The MMCM outputs are routed through global buffers before use.
    wire clk100_ibuf;
    wire clkfb;
    wire clk_25_u;
    wire clk_eth_u;
    wire locked_i;
    // Buffer the board oscillator before it enters the MMCM.
    IBUF ibuf_clk100 (
        .I (clk100_in),
        .O (clk100_ibuf)
    );

    // Mixed-mode clock manager: VCO = 100 MHz * 10 / 1 = 1 GHz.
    MMCME2_BASE #(
        .BANDWIDTH          ("OPTIMIZED"),
        .CLKIN1_PERIOD      (10.0),
        .CLKFBOUT_MULT_F    (10.000),
        .CLKFBOUT_PHASE     (0.000),
        .CLKOUT0_DIVIDE_F   (40.000),
        .CLKOUT0_DUTY_CYCLE (0.500),
        .CLKOUT0_PHASE      (0.000),
        .CLKOUT1_DIVIDE     (20),
        .CLKOUT1_DUTY_CYCLE (0.500),
        .CLKOUT1_PHASE      (0.000),
        .DIVCLK_DIVIDE      (1),
        .REF_JITTER1        (0.010),
        .STARTUP_WAIT       ("FALSE")
    ) u_mmcm (
        .CLKIN1    (clk100_ibuf),
        .CLKFBIN   (clkfb),
        .RST       (~resetn),
        .PWRDWN    (1'b0),
        .CLKFBOUT  (clkfb),
        .CLKFBOUTB (),
        .CLKOUT0   (clk_25_u),
        .CLKOUT0B  (),
        .CLKOUT1   (clk_eth_u),
        .CLKOUT1B  (),
        .CLKOUT2   (),
        .CLKOUT2B  (),
        .CLKOUT3   (),
        .CLKOUT3B  (),
        .CLKOUT4   (),
        .CLKOUT5   (),
        .CLKOUT6   (),
        .LOCKED    (locked_i)
    );

    // Global buffers keep clock skew low across the device.
    BUFG u_bufg_25 (
        .I (clk_25_u),
        .O (clk_25)
    );

    BUFG bufg_eth (
        .I (clk_eth_u),
        .O (clk_eth)
    );

    assign locked = locked_i;
endmodule
